"""GDK ABS_JOINT 运动规划运行时。

通过 agibot_gdk.Robot 执行已验证的关节位置运动。v1 只支持 ABS_JOINT，
拒绝笛卡尔阻抗模式。安全门（ENABLE_GDK_CONTROL + CONFIRM_GDK_CONTROL）和
恢复门（timeout/cancel 后的 recovery 检查）是硬前置条件。
"""

from __future__ import annotations

import importlib
import os
from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from math import isfinite
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
    CONTROL_GROUP_DUAL_ARM,
    DUAL_ARM_JOINTS,
    LEFT_ARM_JOINTS,
    JointSnapshot,
    assert_joint_within_limit,
    assert_positions_within_limits,
    collect_dual_arm_snapshot,
    is_zero_error,
    position_diffs,
    read_joint_position,
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
TASKFLOW_ABS_JOINT_CONFIRMATION = "TASKFLOW_ABS_JOINT"  # 安全门确认令牌
GDK_CONTROL_MODE_UNSUPPORTED = "GDK_CONTROL_MODE_UNSUPPORTED"
UNSUPPORTED_CARTESIAN_IMPEDANCE_MODE = "CTRL_CARTESIAN_IMPEDANCE"
UNSUPPORTED_CONTROL_MODE_MESSAGE = "当前为笛卡尔阻抗模式，请切换到关节位置/规划控制模式后重试"
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


class GdkArmMoveCommandError(RuntimeError):
    """move_arm_joint 失败时保留下发参数，便于现场排障。"""

    def __init__(
        self,
        *,
        original_error: Exception,
        command: Mapping[str, object],
    ) -> None:
        super().__init__(str(original_error))
        self.original_error = original_error
        self.command = dict(command)


def run_gdk_motion_plan_abs_joint(
    motion_params: MotionPlanParams,
    *,
    environ: Mapping[str, str] | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """执行 ABS_JOINT 运动规划（对外入口）。

    前置检查: 安全门 → 速度校验 → 关节目标校验 → 恢复门。
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

    validation_result = validate_abs_joint_targets(motion_params.targets)
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

    # 父进程通过 GdkSessionManager 持锁，C 扩展调用在常驻 worker 子进程完成
    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=False,
            purpose=ACTION_TASKFLOW_ABS_JOINT,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
        )

    with lease:
        result = run_motion_abs_joint_in_subprocess(
            motion_params,
            safety_gate=confirmed_taskflow_safety_gate(),
        )
        maybe_mark_gdk_recovery_required(result, operation=ACTION_TASKFLOW_ABS_JOINT)
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
    """进程内执行 ABS_JOINT（测试模式）。直接 import agibot_gdk 并调用。"""
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=True,
            purpose=ACTION_TASKFLOW_ABS_JOINT,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error)
    except GdkSessionInitError as error:
        return refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
        )

    with lease:
        result: dict[str, object]
        try:
            if lease.agibot_gdk is None:
                raise RuntimeError("GDK session lease missing initialized module")
            robot = lease.agibot_gdk.Robot()
            result = execute_abs_joint_targets(
                robot,
                motion_params,
                agibot_gdk=lease.agibot_gdk,
            )
        except UnsupportedGdkControlModeError as error:
            result = refused_control_mode_result(error)
        except Exception as error:
            result = unavailable_result("execute_abs_joint_targets", error)

        result["gdk_init"] = lease.init_result
        result["gdk_release"] = dict(PROCESS_MANAGED_RELEASE_RESULT)
        result["gdk_session"] = lease.to_payload()
        maybe_mark_gdk_recovery_required(result, operation=ACTION_TASKFLOW_ABS_JOINT)

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


def execute_abs_joint_targets(
    robot: Any,
    motion_params: MotionPlanParams,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
    """执行所有 ABS_JOINT targets：先 arm，再 waist。"""
    targets_by_part = {
        target.body_part: read_abs_joint_action_data(target)
        for target in motion_params.targets
    }
    executed_groups: list[dict[str, object]] = []

    # 机械臂统一走已验证的 14 维 control_group=2 路径。单臂业务语义仍然保留：
    # 未请求的一侧用动作前快照保持，避免默认依赖现场尚未充分复验的 7 维单臂接口。
    arm_targets = {
        body_part: targets_by_part[body_part]
        for body_part in ("left_arm", "right_arm")
        if body_part in targets_by_part
    }
    velocity = float(motion_params.speed)
    if arm_targets:
        executed_groups.append(
            execute_arm_abs_joint_targets(
                robot,
                arm_targets,
                velocity,
                agibot_gdk=agibot_gdk,
            )
        )

    # 腰部单独执行
    if "waist" in targets_by_part:
        executed_groups.append(
            execute_waist_abs_joint_target(
                robot,
                targets_by_part["waist"],
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

    command_diagnostic = build_arm_move_command_diagnostic(
        targets_by_part=targets_by_part,
        target_positions=target_positions,
        velocities=velocities,
        control_group=control_group,
        joint_order=joint_order,
        interface_mode=interface_mode,
        velocity=velocity,
        origin_positions=origin.positions,
    )
    try:
        move_return = robot.move_arm_joint(target_positions, velocities, control_group)
    except Exception as error:
        raise GdkArmMoveCommandError(
            original_error=error,
            command=command_diagnostic,
        ) from error
    if not is_zero_error(move_return):
        command_diagnostic["move_return"] = to_jsonable(move_return)
        raise GdkArmMoveCommandError(
            original_error=RuntimeError(f"move_arm_joint returned {move_return!r}"),
            command=command_diagnostic,
        )

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

    默认使用真机主链路已反复验证的 14 维接口。即便 workflow 只请求单臂，
    也会把另一侧填入动作前当前位姿，用 GDK `control_group=2` 一次下发。
    这样对调度层仍是“单臂目标”，对 GDK 边界则避开 7 维单臂接口的现场兼容风险。
    """
    has_left = "left_arm" in targets_by_part
    has_right = "right_arm" in targets_by_part

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
        build_dual_arm_interface_mode(has_left=has_left, has_right=has_right),
    )


def build_dual_arm_interface_mode(*, has_left: bool, has_right: bool) -> str:
    """描述 14 维命令里的业务目标；未请求侧由快照保持。"""
    if has_left and has_right:
        return "dual_arm_14d"
    if has_left:
        return "dual_arm_14d_hold_right"
    if has_right:
        return "dual_arm_14d_hold_left"
    return "dual_arm_14d_hold_current"


def build_arm_move_command_diagnostic(
    *,
    targets_by_part: Mapping[str, Sequence[float]],
    target_positions: Sequence[float],
    velocities: Sequence[float],
    control_group: int,
    joint_order: Sequence[str],
    interface_mode: str,
    velocity: float,
    origin_positions: Sequence[float],
) -> dict[str, object]:
    """构建轻量诊断；CSV 字符串能穿过 status 深度限制，便于现场直接查看。"""
    deltas = [
        round(float(target) - float(origin), 9)
        for target, origin in zip(target_positions, origin_positions, strict=True)
    ]
    return {
        "method": "move_arm_joint",
        "requested_body_parts": list(targets_by_part.keys()),
        "control_group": control_group,
        "control_group_semantics": "0=left_arm_7d, 1=right_arm_7d, 2=dual_arm_14d",
        "interface_mode": interface_mode,
        "joint_order": list(joint_order),
        "positions_len": len(target_positions),
        "velocities_len": len(velocities),
        "effective_gdk_velocity": velocity,
        "target_positions": [float(value) for value in target_positions],
        "origin_positions": [float(value) for value in origin_positions],
        "position_deltas": deltas,
        "target_positions_csv": format_float_csv(target_positions),
        "origin_positions_csv": format_float_csv(origin_positions),
        "position_deltas_csv": format_float_csv(deltas),
    }


def format_float_csv(values: Sequence[float]) -> str:
    return ",".join(f"{float(value):.9g}" for value in values)


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


def validate_abs_joint_targets(
    targets: Sequence[MotionPlanTarget],
) -> dict[str, object] | None:
    """校验所有 target 的 action_data 为有效关节数组。失败返回 refused_result。"""
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
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """构建 refused 结果（安全门/校验/恢复门拒绝）。executed=False, available=False。"""
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_ABS_JOINT,
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


def unavailable_result(stage: str, error: Exception) -> dict[str, object]:
    """构建 unavailable 结果（GDK 异常）。executed=False, available=False。"""
    original_error = getattr(error, "original_error", error)
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_ABS_JOINT,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_type": type(original_error).__name__,
        "error_msg": str(original_error),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {},
    }
    command = getattr(error, "command", None)
    if isinstance(command, Mapping):
        payload["move_arm_command"] = dict(command)
    return payload
