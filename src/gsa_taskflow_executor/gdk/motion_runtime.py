"""GDK motion_plan 运动规划运行时。

通过 agibot_gdk.Robot 执行已验证的关节位置和小范围末端位姿运动。
安全门（ENABLE_GDK_CONTROL + CONFIRM_GDK_CONTROL）和恢复门
（timeout/cancel 后的 recovery 检查）是硬前置条件。
"""

from __future__ import annotations

import importlib
import os
from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from math import acos, degrees, isfinite, sqrt
from typing import Any

from gsa_taskflow_executor.taskflow.models import (
    MotionPlanParams,
    MotionPlanTarget,
)
from gsa_taskflow_executor.taskflow.skill_params import (
    MOTION_SPEED_MAX,
    MOTION_SPEED_MIN,
)

from .control_probe import (
    ARM_FRAME_NAMES,
    CONTROL_GROUP_DUAL_ARM,
    CONTROL_GROUP_LEFT_ARM,
    CONTROL_GROUP_RIGHT_ARM,
    DUAL_ARM_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    JointSnapshot,
    assert_joint_within_limit,
    assert_positions_within_limits,
    collect_dual_arm_snapshot,
    extract_arm_frame_poses,
    frame_position_delta_m,
    is_zero_error,
    position_diffs,
    read_joint_position,
    serializable_frame_poses,
    utc_now_iso,
    validate_motion_status,
    validate_whole_body_status,
)
from .readonly import GDK_BACKEND, to_jsonable
from .recovery import maybe_mark_gdk_recovery_required, recovery_refused_payload
from .session import (
    PROCESS_MANAGED_RELEASE_RESULT,
    GdkSessionImportError,
    GdkSessionInitError,
    GdkSessionManager,
)
from .subprocess_runtime import GDK_PARENT_LOCK_POLICY, run_motion_abs_joint_in_subprocess

ACTION_TASKFLOW_ABS_JOINT = "taskflow_abs_joint"
ACTION_TASKFLOW_ABS_POSE = "taskflow_abs_pose"
TASKFLOW_ABS_JOINT_CONFIRMATION = "TASKFLOW_ABS_JOINT"  # 安全门确认令牌
GDK_CONTROL_MODE_UNSUPPORTED = "GDK_CONTROL_MODE_UNSUPPORTED"
UNSUPPORTED_CARTESIAN_IMPEDANCE_MODE = "CTRL_CARTESIAN_IMPEDANCE"
UNSUPPORTED_CONTROL_MODE_MESSAGE = "当前为笛卡尔阻抗模式，请切换到关节位置/规划控制模式后重试"
TASKFLOW_ABS_POSE_ALLOWED_BODY_PARTS = ("left_arm", "right_arm")
TASKFLOW_ABS_POSE_MAX_TRANSLATION_M = 0.01
TASKFLOW_ABS_POSE_MAX_ROTATION_DEG = 5.0
TASKFLOW_ABS_POSE_LIFE_TIME_SECONDS = 0.5
WAIST_JOINTS = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]


class UnsupportedGdkControlModeError(RuntimeError):
    """GDK 控制模式不支持关节位置 move_* 接口时抛出。"""

    def __init__(
        self,
        *,
        motion_status: Any,
        unsupported_fields: Sequence[Mapping[str, object]],
    ) -> None:
        super().__init__(UNSUPPORTED_CONTROL_MODE_MESSAGE)
        self.motion_status = motion_status
        self.unsupported_fields = [dict(field) for field in unsupported_fields]


def run_gdk_motion_plan_abs_joint(
    motion_params: MotionPlanParams,
    *,
    environ: Mapping[str, str] | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """执行 motion_plan 运动规划（对外入口）。

    前置检查: 安全门 → 速度校验 → target 校验 → 恢复门。
    生产环境通过持久 worker 子进程执行；测试环境在进程内执行。
    """
    env = environ if environ is not None else os.environ

    # 安全门：必须显式确认 ENABLE_GDK_CONTROL=1 + CONFIRM_GDK_CONTROL 令牌
    gate_result = check_taskflow_abs_joint_safety_gate(env)
    if gate_result is not None:
        return gate_result

    velocity_validation_result = validate_motion_velocity(motion_params.speed)
    if velocity_validation_result is not None:
        return velocity_validation_result

    validation_result = validate_motion_plan_targets(motion_params.targets)
    if validation_result is not None:
        return validation_result

    # 恢复门：上次 timeout/cancel 后必须先采集位姿确认安全
    recovery_result = build_recovery_refused_result(env)
    if recovery_result is not None:
        return recovery_result

    # 测试 fixture 注入 import_module → 进程内执行；生产 → worker 子进程
    if should_use_in_process_runtime(import_module, session_manager):
        return run_gdk_motion_plan_abs_joint_in_process(
            motion_params,
            import_module=import_module,
            session_manager=session_manager,
        )

    action = motion_action(motion_params)

    # 父进程通过 GdkSessionManager 持锁，C 扩展调用在常驻 worker 子进程完成
    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=False,
            purpose=action,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, action=action)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
            action=action,
        )

    with lease:
        result = run_motion_abs_joint_in_subprocess(
            motion_params,
            action=action,
            safety_gate=confirmed_taskflow_safety_gate(),
        )
        maybe_mark_gdk_recovery_required(result, operation=action)
        result["gdk_parent_lock"] = {
            **lease.to_payload(),
            "policy": GDK_PARENT_LOCK_POLICY,
        }
        return result


def run_gdk_motion_plan_abs_joint_in_process(
    motion_params: MotionPlanParams,
    *,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """进程内执行 motion_plan（测试模式）。直接 import agibot_gdk 并调用。"""
    action = motion_action(motion_params)
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=True,
            purpose=action,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error, action=action)
    except GdkSessionInitError as error:
        return refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            action=action,
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, action=action)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
            action=action,
        )

    with lease:
        result: dict[str, object]
        try:
            if lease.agibot_gdk is None:
                raise RuntimeError("GDK session lease missing initialized module")
            robot = lease.agibot_gdk.Robot()
            result = execute_motion_plan_targets(
                robot,
                motion_params,
                agibot_gdk=lease.agibot_gdk,
            )
        except UnsupportedGdkControlModeError as error:
            result = refused_control_mode_result(error)
        except Exception as error:
            result = unavailable_result(
                "execute_motion_plan_targets",
                error,
                action=action,
            )

        result["gdk_init"] = lease.init_result
        result["gdk_release"] = dict(PROCESS_MANAGED_RELEASE_RESULT)
        result["gdk_session"] = lease.to_payload()
        maybe_mark_gdk_recovery_required(result, operation=action)

    return result


def should_use_in_process_runtime(
    import_module: Callable[[str], Any],
    session_manager: GdkSessionManager | None,
) -> bool:
    """测试 fixture 注入 mock import_module 时走进程内路径。"""
    if import_module is not importlib.import_module:
        return True
    return (
        session_manager is not None
        and session_manager.import_module is not importlib.import_module
    )


def confirmed_taskflow_safety_gate() -> dict[str, object]:
    """已通过安全门确认的 payload。"""
    return {
        "enabled": True,
        "confirmed": True,
        "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }


def motion_action(motion_params: MotionPlanParams) -> str:
    """根据首个 target 推断 worker/timeout payload action。"""
    control_type = motion_params.targets[0].control_type if motion_params.targets else "ABS_JOINT"
    return motion_action_for_control_type(control_type)


def motion_action_for_control_type(control_type: str) -> str:
    """根据 control_type 推断结果 payload action。"""
    if control_type == "ABS_POSE":
        return ACTION_TASKFLOW_ABS_POSE
    return ACTION_TASKFLOW_ABS_JOINT


def execute_motion_plan_targets(
    robot: Any,
    motion_params: MotionPlanParams,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
    """按 control_type 分派运动运行时。"""
    control_type = motion_params.targets[0].control_type
    if control_type == "ABS_POSE":
        if agibot_gdk is None:
            raise RuntimeError("ABS_POSE requires initialized agibot_gdk module")
        return execute_abs_pose_target(robot, motion_params, agibot_gdk=agibot_gdk)
    return execute_abs_joint_targets(robot, motion_params, agibot_gdk=agibot_gdk)


def execute_abs_joint_targets(
    robot: Any,
    motion_params: MotionPlanParams,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
    """执行所有 ABS_JOINT targets。

    arm 目标会合并成一次 move_arm_joint；waist 和 arm 的先后顺序尊重 YAML target
    首次出现顺序，二维码定位回初始拍照点位依赖“先腰部、再手臂”恢复视角。
    """
    targets_by_part = {
        target.body_part: read_abs_joint_action_data(target)
        for target in motion_params.targets
    }
    executed_groups: list[dict[str, object]] = []

    # 机械臂统一走 move_arm_joint，但单臂/双臂对应不同 control_group 与入参维度。
    arm_targets = {
        body_part: targets_by_part[body_part]
        for body_part in ("left_arm", "right_arm")
        if body_part in targets_by_part
    }
    velocity = float(motion_params.speed)
    group_order = ordered_abs_joint_groups(motion_params.targets)
    for group in group_order:
        if group == "waist" and "waist" in targets_by_part:
            executed_groups.append(
                execute_waist_abs_joint_target(
                    robot,
                    targets_by_part["waist"],
                    velocity,
                    agibot_gdk=agibot_gdk,
                )
            )
        elif group == "arms" and arm_targets:
            executed_groups.append(
                execute_arm_abs_joint_targets(
                    robot,
                    arm_targets,
                    velocity,
                    agibot_gdk=agibot_gdk,
                )
            )

    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_ABS_JOINT,
        "collected_at": utc_now_iso(),
        "speed": motion_params.speed,
        "requested_speed": motion_params.speed,
        "requested_speed_unit": "gdk_velocity",
        "timeout": motion_params.timeout,
        "effective_gdk_velocity": velocity,
        "gdk_velocity": velocity,
        "velocity_source": "taskflow_speed",
        "speed_mapping_applied": True,
        "groups": executed_groups,
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
    }


def ordered_abs_joint_groups(targets: Sequence[MotionPlanTarget]) -> list[str]:
    """按 target 首次出现顺序返回执行组；左右臂归并为同一个 arms 组。"""
    ordered: list[str] = []
    for target in targets:
        group = "arms" if target.body_part in {"left_arm", "right_arm"} else target.body_part
        if group in {"arms", "waist"} and group not in ordered:
            ordered.append(group)
    return ordered


def execute_abs_pose_target(
    robot: Any,
    motion_params: MotionPlanParams,
    *,
    agibot_gdk: Any,
) -> dict[str, object]:
    """执行首版 ABS_POSE。

    当前只开放单臂单 target。真机验证表明 end_effector_pose_control 小位移可用，
    但不是高精度笛卡尔伺服，因此这里强制目标相对当前末端 pose 只允许小范围变化。
    """
    target = motion_params.targets[0]
    arm = target.body_part
    pose_values = read_abs_pose_action_data(target)
    origin = collect_dual_arm_snapshot(robot)
    ensure_supported_move_control_mode(origin.motion_status, agibot_gdk=agibot_gdk)
    frame_poses = extract_arm_frame_poses(origin.motion_status)
    frame_name = ARM_FRAME_NAMES[arm]
    current_pose = frame_poses.get(frame_name)
    if current_pose is None:
        raise RuntimeError(f"No {frame_name} pose in motion_control_status")

    target_pose_payload = {
        "position": pose_values[:3],
        "orientation": normalize_quaternion(pose_values[3:]),
    }
    translation_delta = position_delta(
        read_pose_position(current_pose, frame_name),
        target_pose_payload["position"],
    )
    translation_norm = vector_norm(translation_delta)
    rotation_deg = quaternion_angle_deg(
        read_pose_orientation(current_pose, frame_name),
        target_pose_payload["orientation"],
    )
    assert_abs_pose_delta_within_limits(translation_norm, rotation_deg)

    end_pose = build_gdk_end_effector_pose(
        agibot_gdk=agibot_gdk,
        frame_poses=frame_poses,
        arm=arm,
        target_pose_payload=target_pose_payload,
    )
    control_method = getattr(robot, "end_effector_pose_control", None)
    if not callable(control_method):
        raise RuntimeError("robot.end_effector_pose_control is unavailable")

    move_return = control_method(end_pose)
    if not is_zero_error(move_return):
        raise RuntimeError(f"end_effector_pose_control returned {move_return!r}")

    after = collect_dual_arm_snapshot(robot)
    after_frame_poses = extract_arm_frame_poses(after.motion_status)
    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_ABS_POSE,
        "collected_at": utc_now_iso(),
        "body_part": arm,
        "requested_body_parts": [arm],
        "control_type": "ABS_POSE",
        "method": "end_effector_pose_control",
        "end_effector_group": to_jsonable(get_abs_pose_group_value(agibot_gdk, arm)),
        "life_time_seconds": TASKFLOW_ABS_POSE_LIFE_TIME_SECONDS,
        "requested_speed": motion_params.speed,
        "requested_speed_unit": "gdk_velocity",
        "speed_mapping_applied": False,
        "timeout": motion_params.timeout,
        "arm_frame_name": frame_name,
        "origin_positions": origin.positions,
        "after_positions": after.positions,
        "diffs": position_diffs(after.positions, origin.positions),
        "current_frame_poses": serializable_frame_poses(frame_poses),
        "target_pose": target_pose_payload,
        "target_translation_delta_m": {
            "x": translation_delta[0],
            "y": translation_delta[1],
            "z": translation_delta[2],
            "norm": translation_norm,
        },
        "target_rotation_delta_deg": rotation_deg,
        "max_translation_m": TASKFLOW_ABS_POSE_MAX_TRANSLATION_M,
        "max_rotation_deg": TASKFLOW_ABS_POSE_MAX_ROTATION_DEG,
        "after_frame_poses": serializable_frame_poses(after_frame_poses),
        "after_arm_frame_delta_m": frame_position_delta_m(
            before=frame_poses,
            after=after_frame_poses,
            frame_name=frame_name,
        ),
        "move_return": to_jsonable(move_return),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": snapshot_raw(origin),
    }


def execute_arm_abs_joint_targets(
    robot: Any,
    targets_by_part: Mapping[str, Sequence[float]],
    velocity: float,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
    """通过 move_arm_joint 执行机械臂运动。采集前后快照，校验控制模式。"""
    origin = collect_dual_arm_snapshot(robot)
    ensure_supported_move_control_mode(origin.motion_status, agibot_gdk=agibot_gdk)

    limits = robot.get_joint_limits()
    if not isinstance(limits, Mapping):
        raise TypeError("robot.get_joint_limits() did not return a mapping")

    target_positions, velocities, control_group, joint_order, interface_mode = (
        build_arm_move_command(
            targets_by_part=targets_by_part,
            origin_positions=origin.positions,
            velocity=velocity,
        )
    )
    assert_arm_target_positions_within_limits(limits, joint_order, target_positions)

    move_return = robot.move_arm_joint(target_positions, velocities, control_group)
    if not is_zero_error(move_return):
        raise RuntimeError(f"move_arm_joint returned {move_return!r}")

    after = collect_dual_arm_snapshot(robot)
    return {
        "body_part": "arms",
        "requested_body_parts": list(targets_by_part.keys()),
        "method": "move_arm_joint",
        "control_group": control_group,
        "control_group_semantics": "0=left_arm_7d, 1=right_arm_7d, 2=dual_arm_14d",
        "interface_mode": interface_mode,
        "joint_order": list(joint_order),
        "positions_len": len(target_positions),
        "velocities_len": len(velocities),
        "velocities": velocities,
        "effective_gdk_velocity": velocity,
        "velocity_source": "taskflow_speed",
        "origin_positions": origin.positions,
        "target_positions": target_positions,
        "after_positions": after.positions,
        "diffs": position_diffs(after.positions, origin.positions),
        "move_return": to_jsonable(move_return),
        "raw": snapshot_raw(origin),
    }


def build_arm_move_command(
    *,
    targets_by_part: Mapping[str, Sequence[float]],
    origin_positions: Sequence[float],
    velocity: float,
) -> tuple[list[float], list[float], int, Sequence[str], str]:
    """构造 move_arm_joint 入参。

    真机验证结论：
    - 单左臂：7 维 positions/velocities + control_group=0
    - 单右臂：7 维 positions/velocities + control_group=1
    - 双臂同步：14 维 positions/velocities + control_group=2

    只有同时包含左右臂目标时才合并 origin 快照；单臂目标直接走 7 维接口，避免
    把未请求的一侧也纳入控制命令。
    """
    has_left = "left_arm" in targets_by_part
    has_right = "right_arm" in targets_by_part

    if has_left and not has_right:
        target_positions = [float(value) for value in targets_by_part["left_arm"]]
        return (
            target_positions,
            [velocity] * len(LEFT_ARM_JOINTS),
            CONTROL_GROUP_LEFT_ARM,
            LEFT_ARM_JOINTS,
            "single_left_arm_7d",
        )

    if has_right and not has_left:
        target_positions = [float(value) for value in targets_by_part["right_arm"]]
        return (
            target_positions,
            [velocity] * len(RIGHT_ARM_JOINTS),
            CONTROL_GROUP_RIGHT_ARM,
            RIGHT_ARM_JOINTS,
            "single_right_arm_7d",
        )

    target_positions = list(origin_positions)
    if has_left:
        target_positions[: len(LEFT_ARM_JOINTS)] = [
            float(value) for value in targets_by_part["left_arm"]
        ]
    if has_right:
        target_positions[len(LEFT_ARM_JOINTS) :] = [
            float(value) for value in targets_by_part["right_arm"]
        ]
    return (
        target_positions,
        [velocity] * len(DUAL_ARM_JOINTS),
        CONTROL_GROUP_DUAL_ARM,
        DUAL_ARM_JOINTS,
        "dual_arm_14d",
    )


def assert_arm_target_positions_within_limits(
    limits: Mapping[str, object],
    joint_order: Sequence[str],
    target_positions: Sequence[float],
) -> None:
    """按本次 move_arm_joint 的实际 joint_order 做限位校验。"""
    if len(joint_order) == len(DUAL_ARM_JOINTS):
        assert_positions_within_limits(limits, target_positions)
        return
    if len(target_positions) != len(joint_order):
        raise RuntimeError(f"expected {len(joint_order)} positions, got {len(target_positions)}")
    for joint_name, position in zip(joint_order, target_positions, strict=True):
        assert_joint_within_limit(limits, joint_name, float(position))


def execute_waist_abs_joint_target(
    robot: Any,
    target_positions: Sequence[float],
    velocity: float,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
    """通过 move_waist_joint 执行腰部运动。"""
    origin = collect_waist_snapshot(robot)
    ensure_supported_move_control_mode(origin.motion_status, agibot_gdk=agibot_gdk)
    target = [float(value) for value in target_positions]
    limits = robot.get_joint_limits()
    if not isinstance(limits, Mapping):
        raise TypeError("robot.get_joint_limits() did not return a mapping")
    assert_waist_positions_within_limits(limits, target)

    velocities = [velocity] * len(WAIST_JOINTS)
    move_return = robot.move_waist_joint(target, velocities)
    if not is_zero_error(move_return):
        raise RuntimeError(f"move_waist_joint returned {move_return!r}")

    after = collect_waist_snapshot(robot)
    return {
        "body_part": "waist",
        "requested_body_parts": ["waist"],
        "method": "move_waist_joint",
        "joint_order": list(WAIST_JOINTS),
        "positions_len": len(target),
        "velocities_len": len(velocities),
        "velocities": velocities,
        "effective_gdk_velocity": velocity,
        "velocity_source": "taskflow_speed",
        "origin_positions": origin.positions,
        "target_positions": target,
        "after_positions": after.positions,
        "diffs": position_diffs(after.positions, origin.positions),
        "move_return": to_jsonable(move_return),
        "raw": snapshot_raw(origin),
    }


def collect_waist_snapshot(robot: Any) -> JointSnapshot:
    """采集腰部关节状态快照（含运动控制状态校验）。"""
    joint_states = robot.get_joint_states()
    limits = robot.get_joint_limits()
    motion_status = robot.get_motion_control_status()
    whole_body_status = robot.get_whole_body_status()

    validate_motion_status(motion_status)
    validate_whole_body_status(whole_body_status)

    if not isinstance(joint_states, Mapping):
        raise TypeError("robot.get_joint_states() did not return a mapping")
    if not isinstance(limits, Mapping):
        raise TypeError("robot.get_joint_limits() did not return a mapping")

    states = joint_states.get("states")
    if not isinstance(states, Sequence) or isinstance(states, str | bytes | bytearray):
        raise TypeError("joint_states['states'] is not a sequence")

    states_by_name = {
        state["name"]: state
        for state in states
        if isinstance(state, Mapping) and isinstance(state.get("name"), str)
    }
    positions: list[float] = []
    for joint_name in WAIST_JOINTS:
        state = states_by_name.get(joint_name)
        if state is None:
            raise RuntimeError(f"missing joint state: {joint_name}")
        if not is_zero_error(state.get("error_code")):
            raise RuntimeError(f"{joint_name} error_code={state.get('error_code')}")
        position = read_joint_position(state)
        assert_joint_within_limit(limits, joint_name, position)
        positions.append(position)

    return JointSnapshot(
        positions=positions,
        motion_status=motion_status,
        whole_body_status=whole_body_status,
    )


def ensure_supported_move_control_mode(
    motion_status: Any,
    *,
    agibot_gdk: Any | None = None,
) -> None:
    """拒绝笛卡尔阻抗模式。检查 control_mode/mode 字段和 repr。"""
    candidates = cartesian_impedance_control_mode_candidates(agibot_gdk)
    unsupported_fields: list[dict[str, object]] = []
    has_control_mode = hasattr(motion_status, "control_mode")

    for field_name in ("control_mode", "mode"):
        raw_value = getattr(motion_status, field_name, None)
        allow_numeric_match = field_name == "control_mode" or not has_control_mode
        if is_cartesian_impedance_mode(
            raw_value,
            candidates,
            allow_numeric_match=allow_numeric_match,
        ):
            unsupported_fields.append(
                {
                    "field": field_name,
                    "value": to_jsonable(raw_value),
                    "repr": repr(raw_value),
                }
            )

    # 兜底：repr 中包含 CTRL_CARTESIAN_IMPEDANCE 字符串
    status_repr = repr(motion_status)
    if not unsupported_fields and UNSUPPORTED_CARTESIAN_IMPEDANCE_MODE in status_repr:
        unsupported_fields.append(
            {
                "field": "repr",
                "value": status_repr,
                "repr": status_repr,
            }
        )

    if unsupported_fields:
        raise UnsupportedGdkControlModeError(
            motion_status=motion_status,
            unsupported_fields=unsupported_fields,
        )


def cartesian_impedance_control_mode_candidates(agibot_gdk: Any | None) -> tuple[Any, ...]:
    """从 agibot_gdk 模块提取 CTRL_CARTESIAN_IMPEDANCE 候选值。"""
    if agibot_gdk is None:
        return ()

    candidates: list[Any] = []
    for container in (agibot_gdk, getattr(agibot_gdk, "MotionControlMode", None)):
        if container is None:
            continue
        candidate = getattr(container, UNSUPPORTED_CARTESIAN_IMPEDANCE_MODE, None)
        if candidate is not None:
            candidates.append(candidate)
    return tuple(candidates)


def is_cartesian_impedance_mode(
    value: Any,
    candidates: Sequence[Any],
    *,
    allow_numeric_match: bool,
) -> bool:
    """检查值是否匹配笛卡尔阻抗模式（字符串/枚举/数值比较）。"""
    if value is None:
        return False

    value_text = f"{value!s} {value!r}"
    if UNSUPPORTED_CARTESIAN_IMPEDANCE_MODE in value_text:
        return True

    for candidate in candidates:
        if value == candidate:
            return True
        candidate_text = f"{candidate!s} {candidate!r}"
        if UNSUPPORTED_CARTESIAN_IMPEDANCE_MODE in candidate_text and value_text in {
            candidate_text,
            str(candidate),
            repr(candidate),
        }:
            return True
        if allow_numeric_match and int_values_equal(value, candidate):
            return True

    return False


def int_values_equal(left: Any, right: Any) -> bool:
    try:
        return int(left) == int(right)
    except (TypeError, ValueError):
        return False


def motion_control_status_payload(motion_status: Any) -> dict[str, object]:
    """构建运动控制状态 payload。"""
    return {
        "mode": to_jsonable(getattr(motion_status, "mode", None)),
        "control_mode": to_jsonable(getattr(motion_status, "control_mode", None)),
        "error_code": to_jsonable(getattr(motion_status, "error_code", None)),
        "error_msg": str(getattr(motion_status, "error_msg", "") or ""),
        "repr": repr(motion_status),
    }


def assert_waist_positions_within_limits(
    limits: Mapping[str, Any],
    positions: Sequence[float],
) -> None:
    """校验腰部关节位置在限位内。"""
    if len(positions) != len(WAIST_JOINTS):
        raise RuntimeError(f"expected {len(WAIST_JOINTS)} waist positions, got {len(positions)}")
    for joint_name, position in zip(WAIST_JOINTS, positions, strict=True):
        assert_joint_within_limit(limits, joint_name, float(position))


def read_abs_joint_action_data(target: MotionPlanTarget) -> list[float]:
    """从 MotionPlanTarget 提取关节角度数组。拒绝变量引用（必须在传入前 resolve）。"""
    if target.control_type != "ABS_JOINT":
        raise RuntimeError(f"{target.body_part} 只支持 ABS_JOINT，当前为 {target.control_type}")
    if isinstance(target.action_data, str):
        raise RuntimeError(f"{target.body_part}.action_data 变量引用未解析为关节数组")
    return [float(value) for value in target.action_data]


def read_abs_pose_action_data(target: MotionPlanTarget) -> list[float]:
    """从 MotionPlanTarget 提取绝对末端 pose: [x, y, z, qx, qy, qz, qw]。"""
    if target.control_type != "ABS_POSE":
        raise RuntimeError(f"{target.body_part} 只支持 ABS_POSE，当前为 {target.control_type}")
    if isinstance(target.action_data, str):
        raise RuntimeError(f"{target.body_part}.action_data 变量引用未解析为 pose 数组")
    values = [float(value) for value in target.action_data]
    if len(values) != 7:
        raise RuntimeError(f"{target.body_part}.action_data ABS_POSE 长度必须是 7")
    if not all(isfinite(value) for value in values):
        raise RuntimeError(f"{target.body_part}.action_data ABS_POSE 必须全部是有限数字")
    normalize_quaternion(values[3:])
    return values


def build_gdk_end_effector_pose(
    *,
    agibot_gdk: Any,
    frame_poses: Mapping[str, Mapping[str, object]],
    arm: str,
    target_pose_payload: Mapping[str, Sequence[float]],
) -> Any:
    end_pose_cls = getattr(agibot_gdk, "EndEffectorPose", None)
    pose_cls = getattr(agibot_gdk, "Pose", None)
    if not callable(end_pose_cls):
        raise RuntimeError("agibot_gdk.EndEffectorPose is unavailable")
    if not callable(pose_cls):
        raise RuntimeError("agibot_gdk.Pose is unavailable")

    left_frame = ARM_FRAME_NAMES["left_arm"]
    right_frame = ARM_FRAME_NAMES["right_arm"]
    left_raw_pose = frame_poses.get(left_frame, {}).get("raw_pose")
    right_raw_pose = frame_poses.get(right_frame, {}).get("raw_pose")
    if left_raw_pose is None:
        raise RuntimeError(f"No {left_frame} pose in motion_control_status")
    if right_raw_pose is None:
        raise RuntimeError(f"No {right_frame} pose in motion_control_status")

    end_pose = end_pose_cls()
    end_pose.group = get_abs_pose_group_value(agibot_gdk, arm)
    end_pose.life_time = TASKFLOW_ABS_POSE_LIFE_TIME_SECONDS
    # GDK EndEffectorPose 同时携带左右臂 pose。只控制目标臂，另一侧保持当前
    # motion_status 里的末端 pose，避免未请求的手臂被意外带动。
    target_pose = build_gdk_pose(
        pose_cls=pose_cls,
        position=target_pose_payload["position"],
        orientation=target_pose_payload["orientation"],
    )
    end_pose.left_end_effector_pose = (
        target_pose
        if arm == "left_arm"
        else copy_gdk_pose(pose_cls=pose_cls, pose=left_raw_pose)
    )
    end_pose.right_end_effector_pose = (
        target_pose
        if arm == "right_arm"
        else copy_gdk_pose(pose_cls=pose_cls, pose=right_raw_pose)
    )
    return end_pose


def get_abs_pose_group_value(agibot_gdk: Any, arm: str) -> object:
    if arm == "left_arm":
        name = "kLeftArm"
    elif arm == "right_arm":
        name = "kRightArm"
    else:
        raise RuntimeError(f"unsupported ABS_POSE arm: {arm}")

    value = getattr(agibot_gdk, name, None)
    if value is not None:
        return value
    enum_cls = getattr(agibot_gdk, "EndEffectorControlGroup", None)
    if enum_cls is not None:
        value = getattr(enum_cls, name, None)
        if value is not None:
            return value
    raise RuntimeError(f"agibot_gdk.{name} is unavailable")


def build_gdk_pose(
    *,
    pose_cls: Any,
    position: Sequence[float],
    orientation: Sequence[float],
) -> Any:
    pose = pose_cls()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    return pose


def copy_gdk_pose(*, pose_cls: Any, pose: Any) -> Any:
    return build_gdk_pose(
        pose_cls=pose_cls,
        position=[
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ],
        orientation=[
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ],
    )


def read_pose_position(pose_payload: Mapping[str, object], frame_name: str) -> list[float]:
    position = pose_payload.get("position")
    if not isinstance(position, list) or len(position) != 3:
        raise RuntimeError(f"{frame_name}.position invalid")
    return [float(value) for value in position]


def read_pose_orientation(pose_payload: Mapping[str, object], frame_name: str) -> list[float]:
    orientation = pose_payload.get("orientation")
    if not isinstance(orientation, list) or len(orientation) != 4:
        raise RuntimeError(f"{frame_name}.orientation invalid")
    return normalize_quaternion([float(value) for value in orientation])


def position_delta(origin: Sequence[float], target: Sequence[float]) -> list[float]:
    return [
        float(target_value) - float(origin_value)
        for origin_value, target_value in zip(origin, target, strict=True)
    ]


def vector_norm(values: Sequence[float]) -> float:
    return sqrt(sum(float(value) * float(value) for value in values))


def normalize_quaternion(values: Sequence[float]) -> list[float]:
    if len(values) != 4:
        raise RuntimeError("quaternion length must be 4")
    norm = vector_norm(values)
    if norm <= 1e-12:
        raise RuntimeError("quaternion norm must be positive")
    return [float(value) / norm for value in values]


def quaternion_angle_deg(origin: Sequence[float], target: Sequence[float]) -> float:
    left = normalize_quaternion(origin)
    right = normalize_quaternion(target)
    dot = abs(sum(a * b for a, b in zip(left, right, strict=True)))
    clamped = min(1.0, max(-1.0, dot))
    return degrees(2.0 * acos(clamped))


def assert_abs_pose_delta_within_limits(
    translation_norm: float,
    rotation_deg: float,
) -> None:
    if translation_norm > TASKFLOW_ABS_POSE_MAX_TRANSLATION_M:
        raise RuntimeError(
            "ABS_POSE translation delta "
            f"{translation_norm:.6f}m exceeds {TASKFLOW_ABS_POSE_MAX_TRANSLATION_M:.6f}m"
        )
    if rotation_deg > TASKFLOW_ABS_POSE_MAX_ROTATION_DEG:
        raise RuntimeError(
            "ABS_POSE rotation delta "
            f"{rotation_deg:.3f}deg exceeds {TASKFLOW_ABS_POSE_MAX_ROTATION_DEG:.3f}deg"
        )


def validate_motion_plan_targets(
    targets: Sequence[MotionPlanTarget],
) -> dict[str, object] | None:
    """校验 motion_plan targets。失败返回 refused_result，且尽量在 import GDK 前完成。"""
    if not targets:
        return refused_result(
            stage="validate_params",
            message="motion_plan targets must not be empty",
        )

    control_types = {target.control_type for target in targets}
    if len(control_types) != 1:
        return refused_result(
            stage="validate_params",
            message=f"motion_plan targets control_type must be uniform: {sorted(control_types)}",
        )
    control_type = targets[0].control_type
    if control_type == "ABS_JOINT":
        return validate_abs_joint_targets(targets)
    if control_type == "ABS_POSE":
        return validate_abs_pose_targets(targets)
    return refused_result(
        stage="validate_params",
        message=f"motion_plan control_type 不支持执行: {control_type}",
        action=motion_action_for_control_type(control_type),
    )


def validate_abs_joint_targets(
    targets: Sequence[MotionPlanTarget],
) -> dict[str, object] | None:
    """校验所有 ABS_JOINT target 的 action_data 为有效关节数组。"""
    for target in targets:
        try:
            read_abs_joint_action_data(target)
        except Exception as error:
            return refused_result(
                stage="validate_params",
                message=str(error),
                extra={"target": deepcopy(target.__dict__)},
            )
    return None


def validate_abs_pose_targets(
    targets: Sequence[MotionPlanTarget],
) -> dict[str, object] | None:
    """ABS_POSE 只允许左右臂单 target，禁止腰部/双臂和变量未解析。"""
    if len(targets) != 1:
        return refused_result(
            stage="validate_params",
            message="ABS_POSE first version supports exactly one target",
            action=ACTION_TASKFLOW_ABS_POSE,
        )
    target = targets[0]
    if target.body_part not in TASKFLOW_ABS_POSE_ALLOWED_BODY_PARTS:
        return refused_result(
            stage="validate_params",
            message=(
                "ABS_POSE first version only supports one of "
                f"{list(TASKFLOW_ABS_POSE_ALLOWED_BODY_PARTS)}, got {target.body_part}"
            ),
            action=ACTION_TASKFLOW_ABS_POSE,
            extra={"target": deepcopy(target.__dict__)},
        )
    try:
        read_abs_pose_action_data(target)
    except Exception as error:
        return refused_result(
            stage="validate_params",
            message=str(error),
            action=ACTION_TASKFLOW_ABS_POSE,
            extra={"target": deepcopy(target.__dict__)},
        )
    return None


def validate_motion_velocity(speed: float) -> dict[str, object] | None:
    """校验速度在 [MOTION_SPEED_MIN, MOTION_SPEED_MAX] 范围内。"""
    if not isfinite(speed) or speed < MOTION_SPEED_MIN or speed > MOTION_SPEED_MAX:
        return refused_result(
            stage="validate_params",
            message=(
                f"speed must be between {MOTION_SPEED_MIN} and {MOTION_SPEED_MAX}"
            ),
        )
    return None


def check_taskflow_abs_joint_safety_gate(
    env: Mapping[str, str],
) -> dict[str, object] | None:
    """检查安全门：ENABLE_GDK_CONTROL=1 且 CONFIRM_GDK_CONTROL 令牌匹配。"""
    if env.get("ENABLE_GDK_CONTROL") != "1":
        return refused_result(
            stage="safety_gate",
            message="ENABLE_GDK_CONTROL must be 1",
        )
    if env.get("CONFIRM_GDK_CONTROL") != TASKFLOW_ABS_JOINT_CONFIRMATION:
        return refused_result(
            stage="safety_gate",
            message="CONFIRM_GDK_CONTROL mismatch",
        )
    return None


def build_recovery_refused_result(env: Mapping[str, str]) -> dict[str, object] | None:
    """检查恢复门：timeout/cancel 后需要先 get_current_pose 确认安全。"""
    recovery_result = recovery_refused_payload(env)
    if recovery_result is None:
        return None
    return refused_result(
        stage="gdk_recovery_required",
        message=str(recovery_result["error_msg"]),
        extra={
            **recovery_result,
            "safety_gate": confirmed_taskflow_safety_gate(),
        },
    )


def snapshot_raw(snapshot: JointSnapshot) -> dict[str, object]:
    return {
        "motion_status": to_jsonable(snapshot.motion_status),
        "whole_body_status": to_jsonable(snapshot.whole_body_status),
    }


def refused_control_mode_result(error: UnsupportedGdkControlModeError) -> dict[str, object]:
    """构建控制模式不支持的结果。"""
    status_payload = motion_control_status_payload(error.motion_status)
    return refused_result(
        stage="gdk_control_mode_unsupported",
        message=UNSUPPORTED_CONTROL_MODE_MESSAGE,
        extra={
            "error_code": GDK_CONTROL_MODE_UNSUPPORTED,
            "motion_control_status": status_payload,
            "unsupported_control_mode_fields": error.unsupported_fields,
            "safety_gate": {
                "enabled": True,
                "confirmed": True,
                "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
            },
            "raw": {
                "motion_status": status_payload,
            },
        },
    )


def refused_result(
    *,
    stage: str,
    message: str,
    action: str = ACTION_TASKFLOW_ABS_JOINT,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """构建 refused 结果（安全门/校验/恢复门拒绝）。executed=False, available=False。"""
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_type": "GdkMotionRuntimeRefused",
        "error_msg": message,
        "safety_gate": {
            "enabled": True,
            "confirmed": False,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {},
    }
    if extra:
        payload.update(dict(extra))
    return payload


def unavailable_result(
    stage: str,
    error: Exception,
    *,
    action: str = ACTION_TASKFLOW_ABS_JOINT,
) -> dict[str, object]:
    """构建 unavailable 结果（GDK 异常）。executed=False, available=False。"""
    return {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_type": type(error).__name__,
        "error_msg": str(error),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {},
    }
