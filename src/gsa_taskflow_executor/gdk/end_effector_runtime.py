"""GDK 末端执行器控制运行时。

默认通过 agibot_gdk.Robot.move_ee_pos() 这个阻塞 GDK 控制接口控制夹爪开合。
若同一条 workflow 前序节点已经进入 end_effector_pose_control 伺服链路，
则改走伺服夹爪路径，避免混用 GDK 文档明确禁止的末端控制接口。
支持 omnipicker/dahuan/ctek90d 三种单关节末端（opening 0~1 线性映射）；
o10_t2/o12_t2 等多关节末端暂未适配。
"""

from __future__ import annotations

import importlib
import os
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from math import ceil
from typing import Any

from gsa_taskflow_executor.taskflow.models import EndEffectorParams

from .control_probe import (
    ARM_FRAME_NAMES,
    copy_gdk_pose,
    extract_arm_frame_poses,
    get_abs_pose_group_value,
    is_zero_error,
    utc_now_iso,
)
from .motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from .readonly import GDK_BACKEND, to_jsonable
from .recovery import maybe_mark_gdk_recovery_required, recovery_refused_payload
from .session import (
    PROCESS_MANAGED_RELEASE_RESULT,
    GdkSessionImportError,
    GdkSessionInitError,
    GdkSessionManager,
)
from .subprocess_runtime import GDK_PARENT_LOCK_POLICY, run_end_effector_in_subprocess

ACTION_TASKFLOW_END_EFFECTOR = "taskflow_end_effector"
GDK_END_EFFECTOR_TYPE_UNKNOWN = "GDK_END_EFFECTOR_TYPE_UNKNOWN"
GDK_END_EFFECTOR_TYPE_UNSUPPORTED = "GDK_END_EFFECTOR_TYPE_UNSUPPORTED"
GDK_END_EFFECTOR_TYPE_MISMATCH = "GDK_END_EFFECTOR_TYPE_MISMATCH"
GDK_END_EFFECTOR_SERVO_STATE_UNAVAILABLE = "GDK_END_EFFECTOR_SERVO_STATE_UNAVAILABLE"
GDK_END_EFFECTOR_SERVO_STATE_MISMATCH = "GDK_END_EFFECTOR_SERVO_STATE_MISMATCH"

# PDF 示例给出了这些单关节末端的开/合范围；opening=1 表示打开，0 表示闭合。
SINGLE_JOINT_END_EFFECTOR_RANGES = {
    "omnipicker": {"open": -0.785, "closed": 0.0},
    "dahuan": {"open": 0.0, "closed": 0.025},
    "ctek90d": {"open": -0.91, "closed": 0.0},
}
MULTI_JOINT_END_EFFECTOR_TYPES = frozenset({"o10_t2", "o12_t2"})

SERVO_GRIPPER_METHOD = "end_effector_pose_control"
SERVO_GRIPPER_CONTROL_METHOD = "servo_gripper_hold_current_pose"
SERVO_GRIPPER_RATE_HZ = 50.0
SERVO_GRIPPER_LIFE_TIME_SECONDS = 0.02
SERVO_GRIPPER_MIN_DURATION_SECONDS = 0.3
SERVO_GRIPPER_FINAL_HOLD_SECONDS = 0.2


@dataclass(frozen=True)
class EndEffectorMoveCommand:
    end_effector_type: str | None
    positions: list[float] | None
    requested_openings: list[float]
    left_opening: float | None
    right_opening: float | None
    left_end_effector_type: str | None
    right_end_effector_type: str | None


@dataclass(frozen=True)
class ServoGripperJoints:
    names: list[str]
    start_positions: list[float]


@dataclass(frozen=True)
class ServoGripperCommand:
    names: list[str]
    start_positions: list[float]
    target_positions: list[float]
    controlled_arms: list[str]


def run_gdk_end_effector_control(
    end_effector_params: EndEffectorParams,
    *,
    environ: Mapping[str, str] | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
    sleep: Callable[[float], None] = time.sleep,
    prefer_servo: bool = False,
) -> dict[str, object]:
    """控制末端执行器开合。

    prefer_servo=True 时复用 end_effector_pose_control 下发夹爪关节目标。
    这是为 ABS_POSE 后续夹爪动作准备的安全路径：GDK 4.1.5 文档明确
    move_ee_pos 不应和伺服接口混用。
    """

    env = environ if environ is not None else os.environ
    gate_result = check_taskflow_end_effector_safety_gate(env)
    if gate_result is not None:
        return gate_result

    recovery_result = build_recovery_refused_result(env)
    if recovery_result is not None:
        return recovery_result

    if should_use_in_process_runtime(import_module, session_manager, sleep):
        return run_gdk_end_effector_control_in_process(
            end_effector_params,
            import_module=import_module,
            session_manager=session_manager,
            sleep=sleep,
            prefer_servo=prefer_servo,
        )

    # 末端控制可能走 move_ee_pos，也可能走 servo gripper；二者都触碰 GDK 控制通道，
    # 父进程只保留互斥锁，阻塞和超时恢复交给常驻 worker。
    # 常驻 worker 复用 GDK 初始化，超时时再杀掉并重启该 worker。
    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=False,
            purpose=ACTION_TASKFLOW_END_EFFECTOR,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
            safety_confirmed=True,
        )

    with lease:
        result = run_end_effector_in_subprocess(
            end_effector_params,
            safety_gate=confirmed_taskflow_safety_gate(),
            prefer_servo=prefer_servo,
        )
        maybe_mark_gdk_recovery_required(result, operation=ACTION_TASKFLOW_END_EFFECTOR)
        result["gdk_parent_lock"] = {
            **lease.to_payload(),
            "policy": GDK_PARENT_LOCK_POLICY,
        }
        return result


def run_gdk_end_effector_control_in_process(
    end_effector_params: EndEffectorParams,
    *,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
    sleep: Callable[[float], None] = time.sleep,
    prefer_servo: bool = False,
) -> dict[str, object]:
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=True,
            purpose=ACTION_TASKFLOW_END_EFFECTOR,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error)
    except GdkSessionInitError as error:
        return refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            safety_confirmed=True,
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
            safety_confirmed=True,
        )

    with lease:
        result: dict[str, object]
        try:
            if lease.agibot_gdk is None:
                raise RuntimeError("GDK session lease missing initialized module")
            robot = lease.agibot_gdk.Robot()
            result = execute_end_effector_control(
                robot,
                end_effector_params,
                agibot_gdk=lease.agibot_gdk,
                sleep=sleep,
                prefer_servo=prefer_servo,
            )
        except Exception as error:
            result = unavailable_result("execute_end_effector_control", error)

        result["gdk_init"] = lease.init_result
        result["gdk_release"] = dict(PROCESS_MANAGED_RELEASE_RESULT)
        result["gdk_session"] = lease.to_payload()
        maybe_mark_gdk_recovery_required(result, operation=ACTION_TASKFLOW_END_EFFECTOR)

    return result


def should_use_in_process_runtime(
    import_module: Callable[[str], Any],
    session_manager: GdkSessionManager | None,
    sleep: Callable[[float], None],
) -> bool:
    if import_module is not importlib.import_module:
        return True
    if sleep is not time.sleep:
        return True
    return (
        session_manager is not None
        and session_manager.import_module is not importlib.import_module
    )


def confirmed_taskflow_safety_gate() -> dict[str, object]:
    return {
        "enabled": True,
        "confirmed": True,
        "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }


def execute_end_effector_control(
    robot: Any,
    end_effector_params: EndEffectorParams,
    *,
    agibot_gdk: Any,
    sleep: Callable[[float], None] = time.sleep,
    prefer_servo: bool = False,
) -> dict[str, object]:
    before_end_state = read_end_state(robot)
    command_result = build_end_effector_move_command(end_effector_params, before_end_state)
    if isinstance(command_result, dict):
        return command_result
    command = command_result

    if command.end_effector_type is None:
        return refused_result(
            stage="resolve_end_effector_type",
            message=(
                "末端型号为空，且 get_end_state() 未返回可识别的末端执行器类型"
            ),
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_TYPE_UNKNOWN,
                "target_end": end_effector_params.target_end,
                "before_end_state": to_jsonable(before_end_state),
            },
        )

    if command.positions is None:
        return refused_result(
            stage="validate_end_effector_type",
            message=(
                "当前开合度 0~1 映射仅支持 omnipicker/dahuan/ctek90d；"
                f"{command.end_effector_type} 需要补充多关节映射后再开放"
            ),
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_TYPE_UNSUPPORTED,
                "target_end": end_effector_params.target_end,
                "end_effector_type": command.end_effector_type,
                "supported_end_effector_types": sorted(SINGLE_JOINT_END_EFFECTOR_RANGES),
                "known_multi_joint_end_effector_types": sorted(
                    MULTI_JOINT_END_EFFECTOR_TYPES
                ),
                "before_end_state": to_jsonable(before_end_state),
            },
        )

    if prefer_servo:
        return execute_servo_gripper_control(
            robot,
            end_effector_params,
            command,
            before_end_state=before_end_state,
            agibot_gdk=agibot_gdk,
            sleep=sleep,
        )

    joint_states = build_gdk_joint_states(
        agibot_gdk,
        group=end_effector_params.target_end,
        target_type=command.end_effector_type,
        positions=command.positions,
    )
    move_return = robot.move_ee_pos(joint_states)
    if not is_zero_error(move_return):
        raise RuntimeError(f"move_ee_pos returned {move_return!r}")

    wait_after_command = end_effector_params.post_wait_seconds > 0
    if wait_after_command:
        # move_ee_pos 本身是阻塞控制调用；后置等待用于给末端状态与下游动作
        # 留出稳定窗口，不能把 worker 可取消等同于 GDK 控制器已确认停稳。
        sleep(end_effector_params.post_wait_seconds)

    after_end_state = read_end_state(robot)
    actual_openness = extract_actual_openness(after_end_state, end_effector_params.target_end)
    actual_openness_source = "gdk_after_end_state"
    if actual_openness is None:
        # 真机 get_end_state() 字段形态仍需继续现场归档；解析不到时保留请求值用于下游联调。
        actual_openness = command.requested_openings
        actual_openness_source = "requested_opening_fallback"
    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
        "collected_at": utc_now_iso(),
        "method": "move_ee_pos",
        "target_end": end_effector_params.target_end,
        "group": end_effector_params.target_end,
        "end_effector_type": command.end_effector_type,
        "target_type": command.end_effector_type,
        "opening": end_effector_params.opening,
        "left_opening": command.left_opening,
        "right_opening": command.right_opening,
        "left_end_effector_type": command.left_end_effector_type,
        "right_end_effector_type": command.right_end_effector_type,
        "post_wait_seconds": end_effector_params.post_wait_seconds,
        "wait_after_command": wait_after_command,
        "actual_openness": actual_openness,
        "actual_openness_source": actual_openness_source,
        "target_positions": command.positions,
        "positions_len": len(command.positions),
        "positions_layout": build_positions_layout(end_effector_params.target_end),
        "joint_states": {
            "group": end_effector_params.target_end,
            "target_type": command.end_effector_type,
            "nums": len(command.positions),
            "positions": command.positions,
            "positions_layout": build_positions_layout(end_effector_params.target_end),
        },
        "move_return": to_jsonable(move_return),
        "timeout": end_effector_params.timeout,
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {
            "before_end_state": to_jsonable(before_end_state),
            "after_end_state": to_jsonable(after_end_state),
        },
    }


def execute_servo_gripper_control(
    robot: Any,
    end_effector_params: EndEffectorParams,
    command: EndEffectorMoveCommand,
    *,
    before_end_state: Mapping[str, Any],
    agibot_gdk: Any,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """通过 end_effector_pose_control 驱动夹爪关节。

    这个路径专门用于 ABS_POSE 之后的末端控制：保持当前左右末端 pose 不变，
    只在 EndEffectorPose 中填入夹爪 joint_names / joint_positions，避免从
    伺服接口切回 move_ee_pos 导致 GDK 控制通道冲突。
    """

    servo_command_result = build_servo_gripper_command(
        end_effector_params,
        command,
        before_end_state,
    )
    if isinstance(servo_command_result, dict):
        return servo_command_result
    servo_command = servo_command_result

    motion_status = robot.get_motion_control_status()
    frame_poses = extract_arm_frame_poses(motion_status)
    missing_frame_names = [
        frame_name
        for frame_name in (ARM_FRAME_NAMES["left_arm"], ARM_FRAME_NAMES["right_arm"])
        if frame_name not in frame_poses
    ]
    if missing_frame_names:
        return refused_result(
            stage="read_motion_control_status",
            message="motion_control_status 缺少末端 frame，无法通过伺服接口保持当前位姿",
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_SERVO_STATE_UNAVAILABLE,
                "target_end": end_effector_params.target_end,
                "missing_frame_names": missing_frame_names,
                "raw": {"motion_status": to_jsonable(motion_status)},
            },
        )

    control_method = getattr(robot, SERVO_GRIPPER_METHOD, None)
    if not callable(control_method):
        raise RuntimeError(f"robot.{SERVO_GRIPPER_METHOD} is unavailable")

    group_value = get_servo_gripper_group_value(agibot_gdk, end_effector_params.target_end)
    duration_seconds = max(
        float(end_effector_params.post_wait_seconds),
        SERVO_GRIPPER_MIN_DURATION_SECONDS,
    )
    step_count = max(2, int(ceil(duration_seconds * SERVO_GRIPPER_RATE_HZ)))
    final_hold_steps = max(1, int(ceil(SERVO_GRIPPER_FINAL_HOLD_SECONDS * SERVO_GRIPPER_RATE_HZ)))
    dt_seconds = 1.0 / SERVO_GRIPPER_RATE_HZ
    move_return: object = 0

    for step_index in range(step_count):
        ratio = step_index / (step_count - 1)
        joint_positions = interpolate_positions(
            servo_command.start_positions,
            servo_command.target_positions,
            ratio,
        )
        move_return = send_servo_gripper_control_step(
            agibot_gdk=agibot_gdk,
            frame_poses=frame_poses,
            group_value=group_value,
            joint_names=servo_command.names,
            joint_positions=joint_positions,
            control_method=control_method,
            step_index=step_index + 1,
        )
        sleep(dt_seconds)

    for hold_index in range(final_hold_steps):
        move_return = send_servo_gripper_control_step(
            agibot_gdk=agibot_gdk,
            frame_poses=frame_poses,
            group_value=group_value,
            joint_names=servo_command.names,
            joint_positions=servo_command.target_positions,
            control_method=control_method,
            step_index=step_count + hold_index + 1,
        )
        sleep(dt_seconds)

    after_end_state = read_end_state(robot)
    actual_openness = extract_actual_openness(after_end_state, end_effector_params.target_end)
    actual_openness_source = "gdk_after_end_state"
    if actual_openness is None:
        actual_openness = command.requested_openings
        actual_openness_source = "requested_opening_fallback"

    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
        "collected_at": utc_now_iso(),
        "method": SERVO_GRIPPER_METHOD,
        "control_method": SERVO_GRIPPER_CONTROL_METHOD,
        "target_end": end_effector_params.target_end,
        "group": end_effector_params.target_end,
        "end_effector_group": to_jsonable(group_value),
        "end_effector_type": command.end_effector_type,
        "target_type": command.end_effector_type,
        "opening": end_effector_params.opening,
        "left_opening": command.left_opening,
        "right_opening": command.right_opening,
        "left_end_effector_type": command.left_end_effector_type,
        "right_end_effector_type": command.right_end_effector_type,
        "post_wait_seconds": end_effector_params.post_wait_seconds,
        "wait_after_command": True,
        "actual_openness": actual_openness,
        "actual_openness_source": actual_openness_source,
        "target_positions": servo_command.target_positions,
        "start_positions": servo_command.start_positions,
        "positions_len": len(servo_command.target_positions),
        "positions_layout": build_positions_layout(end_effector_params.target_end),
        "end_effector_joint_names": servo_command.names,
        "end_effector_joint_positions": servo_command.target_positions,
        "controlled_arms": servo_command.controlled_arms,
        "servo_duration_seconds": duration_seconds,
        "servo_rate_hz": SERVO_GRIPPER_RATE_HZ,
        "servo_life_time_seconds": SERVO_GRIPPER_LIFE_TIME_SECONDS,
        "servo_step_count": step_count,
        "servo_final_hold_steps": final_hold_steps,
        "move_return": to_jsonable(move_return),
        "timeout": end_effector_params.timeout,
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {
            "before_end_state": to_jsonable(before_end_state),
            "after_end_state": to_jsonable(after_end_state),
            "motion_status_before": to_jsonable(motion_status),
        },
    }


def build_servo_gripper_command(
    end_effector_params: EndEffectorParams,
    command: EndEffectorMoveCommand,
    before_end_state: Mapping[str, Any],
) -> ServoGripperCommand | dict[str, object]:
    if command.positions is None:
        raise RuntimeError("servo gripper command requires target positions")

    names: list[str] = []
    start_positions: list[float] = []
    target_positions: list[float] = []
    controlled_arms: list[str] = []
    for target_end in target_end_sides(end_effector_params.target_end):
        side_result = read_servo_side_joints(before_end_state, target_end)
        if isinstance(side_result, dict):
            return side_result
        side_target_position_result = read_servo_side_target_position(
            end_effector_params.target_end,
            target_end,
            command.positions,
        )
        if isinstance(side_target_position_result, dict):
            return side_target_position_result
        side_target_position = side_target_position_result
        names.extend(side_result.names)
        start_positions.extend(side_result.start_positions)
        target_positions.extend([side_target_position] * len(side_result.names))
        controlled_arms.append(target_end_to_arm(target_end))

    return ServoGripperCommand(
        names=names,
        start_positions=start_positions,
        target_positions=target_positions,
        controlled_arms=controlled_arms,
    )


def read_servo_side_target_position(
    target_end: str,
    side_target_end: str,
    command_positions: Sequence[float],
) -> float | dict[str, object]:
    if target_end == "dual_tool":
        expected_len = 2
        if len(command_positions) != expected_len:
            return refused_result(
                stage="validate_servo_gripper_target_positions",
                message="dual_tool servo gripper 需要左右两侧各 1 个目标开合位置",
                safety_confirmed=True,
                extra={
                    "error_code": GDK_END_EFFECTOR_SERVO_STATE_MISMATCH,
                    "target_end": target_end,
                    "target_positions_len": len(command_positions),
                    "positions_layout": build_positions_layout(target_end),
                },
            )
        return float(command_positions[0 if side_target_end == "left_tool" else 1])

    if len(command_positions) != 1:
        return refused_result(
            stage="validate_servo_gripper_target_positions",
            message="单侧 servo gripper 需要 1 个目标开合位置",
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_SERVO_STATE_MISMATCH,
                "target_end": target_end,
                "target_positions_len": len(command_positions),
                "positions_layout": build_positions_layout(target_end),
            },
        )
    return float(command_positions[0])


def read_servo_side_joints(
    end_state: Mapping[str, Any],
    target_end: str,
) -> ServoGripperJoints | dict[str, object]:
    side = "left" if target_end == "left_tool" else "right"
    for candidate in servo_side_state_candidates(end_state, target_end, side):
        names = read_string_sequence_from_keys(
            candidate,
            ("names", "joint_names", "end_effector_joint_names"),
        )
        positions = read_servo_positions_from_state(candidate)
        if names and positions and len(names) == len(positions):
            return ServoGripperJoints(names=names, start_positions=positions)

    return refused_result(
        stage="read_servo_gripper_joints",
        message="get_end_state() 未返回可用于伺服夹爪控制的关节名/当前位置",
        safety_confirmed=True,
        extra={
            "error_code": GDK_END_EFFECTOR_SERVO_STATE_UNAVAILABLE,
            "target_end": target_end,
            "before_end_state": to_jsonable(end_state),
        },
    )


def servo_side_state_candidates(
    end_state: Mapping[str, Any],
    target_end: str,
    side: str,
) -> list[Mapping[str, Any]]:
    candidates: list[Mapping[str, Any]] = []
    for key in (
        f"{side}_end_state",
        target_end,
        f"{side}_tool",
        f"{side}_end",
        f"{side}_end_effector",
        side,
    ):
        value = end_state.get(key)
        if isinstance(value, Mapping):
            candidates.append(value)
    return candidates


def read_string_sequence_from_keys(
    mapping: Mapping[str, Any],
    keys: Sequence[str],
) -> list[str] | None:
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray):
            strings = [item.strip() for item in value if isinstance(item, str) and item.strip()]
            if len(strings) == len(value) and strings:
                return strings
    return None


def read_servo_positions_from_state(mapping: Mapping[str, Any]) -> list[float] | None:
    positions = first_numeric_sequence(
        mapping,
        ("positions", "joint_positions", "end_effector_joint_positions"),
    )
    if positions is not None:
        return positions

    end_states = mapping.get("end_states")
    if not isinstance(end_states, Sequence) or isinstance(end_states, str | bytes | bytearray):
        return None

    positions = []
    for state in end_states:
        position = (
            state.get("position")
            if isinstance(state, Mapping)
            else getattr(state, "position", None)
        )
        if isinstance(position, bool) or not isinstance(position, int | float):
            return None
        positions.append(float(position))
    return positions if positions else None


def target_end_sides(target_end: str) -> list[str]:
    if target_end == "dual_tool":
        return ["left_tool", "right_tool"]
    return [target_end]


def target_end_to_arm(target_end: str) -> str:
    if target_end == "left_tool":
        return "left_arm"
    if target_end == "right_tool":
        return "right_arm"
    raise RuntimeError(f"unsupported target_end for servo gripper: {target_end}")


def get_servo_gripper_group_value(agibot_gdk: Any, target_end: str) -> object:
    if target_end == "left_tool":
        return get_abs_pose_group_value(agibot_gdk, "left_arm")
    if target_end == "right_tool":
        return get_abs_pose_group_value(agibot_gdk, "right_arm")

    value = getattr(agibot_gdk, "kBothArms", None)
    if value is not None:
        return value
    enum_cls = getattr(agibot_gdk, "EndEffectorControlGroup", None)
    if enum_cls is not None:
        value = getattr(enum_cls, "kBothArms", None)
        if value is not None:
            return value
    raise RuntimeError("agibot_gdk.EndEffectorControlGroup.kBothArms is unavailable")


def send_servo_gripper_control_step(
    *,
    agibot_gdk: Any,
    frame_poses: Mapping[str, Mapping[str, object]],
    group_value: object,
    joint_names: Sequence[str],
    joint_positions: Sequence[float],
    control_method: Callable[[Any], object],
    step_index: int,
) -> object:
    end_pose = build_hold_current_servo_gripper_pose(
        agibot_gdk=agibot_gdk,
        frame_poses=frame_poses,
        group_value=group_value,
        joint_names=joint_names,
        joint_positions=joint_positions,
    )
    move_return = control_method(end_pose)
    if not is_zero_error(move_return):
        raise RuntimeError(
            f"{SERVO_GRIPPER_METHOD} returned {move_return!r} at gripper step {step_index}"
        )
    return move_return


def build_hold_current_servo_gripper_pose(
    *,
    agibot_gdk: Any,
    frame_poses: Mapping[str, Mapping[str, object]],
    group_value: object,
    joint_names: Sequence[str],
    joint_positions: Sequence[float],
) -> Any:
    end_pose_cls = getattr(agibot_gdk, "EndEffectorPose", None)
    if not callable(end_pose_cls):
        raise RuntimeError("agibot_gdk.EndEffectorPose is unavailable")

    left_frame = ARM_FRAME_NAMES["left_arm"]
    right_frame = ARM_FRAME_NAMES["right_arm"]
    left_raw_pose = frame_poses.get(left_frame, {}).get("raw_pose")
    right_raw_pose = frame_poses.get(right_frame, {}).get("raw_pose")
    if left_raw_pose is None:
        raise RuntimeError(f"No {left_frame} pose in motion_control_status")
    if right_raw_pose is None:
        raise RuntimeError(f"No {right_frame} pose in motion_control_status")

    end_pose = end_pose_cls()
    end_pose.group = group_value
    end_pose.life_time = SERVO_GRIPPER_LIFE_TIME_SECONDS
    # 即使只控制单侧夹爪，也同步带上左右当前 pose，避免伺服接口拿默认 pose 造成姿态跳变。
    end_pose.left_end_effector_pose = copy_gdk_pose(agibot_gdk, left_raw_pose)
    end_pose.right_end_effector_pose = copy_gdk_pose(agibot_gdk, right_raw_pose)
    end_pose.end_effector_joint_names = list(joint_names)
    end_pose.end_effector_joint_positions = [float(position) for position in joint_positions]
    return end_pose


def interpolate_positions(
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    ratio: float,
) -> list[float]:
    return [
        float(start) + (float(target) - float(start)) * ratio
        for start, target in zip(start_positions, target_positions, strict=True)
    ]


def check_taskflow_end_effector_safety_gate(
    env: Mapping[str, str],
) -> dict[str, object] | None:
    if env.get("ENABLE_GDK_CONTROL") != "1":
        return refused_result(
            stage="safety_gate",
            message="ENABLE_GDK_CONTROL must be 1",
            safety_confirmed=False,
        )
    if env.get("CONFIRM_GDK_CONTROL") != TASKFLOW_ABS_JOINT_CONFIRMATION:
        return refused_result(
            stage="safety_gate",
            message="CONFIRM_GDK_CONTROL mismatch",
            safety_confirmed=False,
        )
    return None


def build_recovery_refused_result(env: Mapping[str, str]) -> dict[str, object] | None:
    recovery_result = recovery_refused_payload(env)
    if recovery_result is None:
        return None
    return refused_result(
        stage="gdk_recovery_required",
        message=str(recovery_result["error_msg"]),
        safety_confirmed=True,
        extra={
            **recovery_result,
            "safety_gate": confirmed_taskflow_safety_gate(),
        },
    )


def read_end_state(robot: Any) -> Mapping[str, Any]:
    end_state = robot.get_end_state()
    if not isinstance(end_state, Mapping):
        raise TypeError("robot.get_end_state() did not return a mapping")
    return end_state


def build_end_effector_move_command(
    end_effector_params: EndEffectorParams,
    end_state: Mapping[str, Any],
) -> EndEffectorMoveCommand | dict[str, object]:
    if end_effector_params.target_end == "dual_tool":
        left_type = resolve_single_end_effector_type(
            end_effector_params.left_end_effector_type,
            end_effector_params.end_effector_type,
            "left_tool",
            end_state,
        )
        right_type = resolve_single_end_effector_type(
            end_effector_params.right_end_effector_type,
            end_effector_params.end_effector_type,
            "right_tool",
            end_state,
        )
        if left_type is None or right_type is None:
            return EndEffectorMoveCommand(
                end_effector_type=None,
                positions=None,
                requested_openings=[
                    read_side_opening(end_effector_params, "left_tool"),
                    read_side_opening(end_effector_params, "right_tool"),
                ],
                left_opening=read_side_opening(end_effector_params, "left_tool"),
                right_opening=read_side_opening(end_effector_params, "right_tool"),
                left_end_effector_type=left_type,
                right_end_effector_type=right_type,
            )
        if left_type != right_type:
            return refused_result(
                stage="validate_end_effector_type",
                message="dual_tool 需要左右末端型号一致，因为 move_ee_pos 只有一个 target_type",
                safety_confirmed=True,
                extra={
                    "error_code": GDK_END_EFFECTOR_TYPE_MISMATCH,
                    "target_end": end_effector_params.target_end,
                    "left_end_effector_type": left_type,
                    "right_end_effector_type": right_type,
                    "before_end_state": to_jsonable(end_state),
                },
            )
        left_opening = read_side_opening(end_effector_params, "left_tool")
        right_opening = read_side_opening(end_effector_params, "right_tool")
        left_positions = build_positions_for_opening(left_type, left_opening)
        right_positions = build_positions_for_opening(right_type, right_opening)
        positions = (
            [*left_positions, *right_positions]
            if left_positions is not None and right_positions is not None
            else None
        )
        return EndEffectorMoveCommand(
            end_effector_type=left_type,
            positions=positions,
            requested_openings=[left_opening, right_opening],
            left_opening=left_opening,
            right_opening=right_opening,
            left_end_effector_type=left_type,
            right_end_effector_type=right_type,
        )

    end_effector_type = resolve_end_effector_type(
        end_effector_params.end_effector_type,
        end_effector_params.target_end,
        end_state,
    )
    opening = read_side_opening(end_effector_params, end_effector_params.target_end)
    return EndEffectorMoveCommand(
        end_effector_type=end_effector_type,
        positions=(
            build_positions_for_opening(end_effector_type, opening)
            if end_effector_type is not None
            else None
        ),
        requested_openings=[opening],
        left_opening=opening if end_effector_params.target_end == "left_tool" else None,
        right_opening=opening if end_effector_params.target_end == "right_tool" else None,
        left_end_effector_type=(
            end_effector_type if end_effector_params.target_end == "left_tool" else None
        ),
        right_end_effector_type=(
            end_effector_type if end_effector_params.target_end == "right_tool" else None
        ),
    )


def resolve_single_end_effector_type(
    side_configured_type: str | None,
    fallback_configured_type: str | None,
    target_end: str,
    end_state: Mapping[str, Any],
) -> str | None:
    configured_type = side_configured_type or fallback_configured_type
    return resolve_end_effector_type(configured_type, target_end, end_state)


def read_side_opening(end_effector_params: EndEffectorParams, target_end: str) -> float:
    if target_end == "left_tool":
        if end_effector_params.left_opening is not None:
            return end_effector_params.left_opening
        if end_effector_params.opening is not None:
            return end_effector_params.opening
        raise ValueError("left_tool 缺少 opening/left_opening")
    if target_end == "right_tool":
        if end_effector_params.right_opening is not None:
            return end_effector_params.right_opening
        if end_effector_params.opening is not None:
            return end_effector_params.opening
        raise ValueError("right_tool 缺少 opening/right_opening")
    if end_effector_params.opening is None:
        raise ValueError("dual_tool 缺少 opening 或左右开度")
    return end_effector_params.opening


def resolve_end_effector_type(
    configured_type: str | None,
    target_end: str,
    end_state: Mapping[str, Any],
) -> str | None:
    """解析末端执行器型号：优先配置值，否则从 get_end_state() 推断。"""
    if configured_type:
        return normalize_end_effector_type(configured_type)
    if target_end == "dual_tool":
        left_type = infer_end_effector_type(end_state, "left_tool")
        right_type = infer_end_effector_type(end_state, "right_tool")
        if left_type is not None and left_type == right_type:
            return left_type
        return None
    return infer_end_effector_type(end_state, target_end)


def infer_end_effector_type(
    end_state: Mapping[str, Any],
    target_end: str,
) -> str | None:
    """从 get_end_state() 返回值推断末端型号。按 top-level → nested → list 顺序查找。"""
    side = "left" if target_end == "left_tool" else "right"
    top_level_keys = (
        f"{side}_end_effector_type",
        f"{side}_end_effector_model",
        f"{side}_tool_type",
        f"{side}_tool_model",
        f"{side}_end_type",
        f"{side}_end_model",
    )
    value = first_string_value(end_state, top_level_keys)
    if value is not None:
        return normalize_end_effector_type(value)

    nested_keys = (
        target_end,
        f"{side}_tool",
        f"{side}_end",
        f"{side}_end_effector",
        side,
    )
    for key in nested_keys:
        nested = end_state.get(key)
        if isinstance(nested, Mapping):
            value = first_string_value(
                nested,
                (
                    "target_type",
                    "end_effector_type",
                    "type",
                    "model",
                    "end_effector_model",
                ),
            )
            if value is not None:
                return normalize_end_effector_type(value)

    for list_key in ("states", "end_states", "tools"):
        value = infer_end_effector_type_from_items(end_state.get(list_key), target_end, side)
        if value is not None:
            return value

    return None


def infer_end_effector_type_from_items(
    value: Any,
    target_end: str,
    side: str,
) -> str | None:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return None

    aliases = {target_end, side, f"{side}_tool", f"{side}_end"}
    for item in value:
        if not isinstance(item, Mapping):
            continue
        item_group = first_string_value(item, ("group", "name", "target_end", "end"))
        if item_group not in aliases:
            continue
        item_type = first_string_value(
            item,
            ("target_type", "end_effector_type", "type", "model", "end_effector_model"),
        )
        if item_type is not None:
            return normalize_end_effector_type(item_type)
    return None


def extract_actual_openness(
    end_state: Mapping[str, Any],
    target_end: str,
) -> list[float] | None:
    if target_end == "dual_tool":
        left = extract_actual_openness(end_state, "left_tool")
        right = extract_actual_openness(end_state, "right_tool")
        if left is None or right is None:
            return None
        return [*left, *right]

    side = "left" if target_end == "left_tool" else "right"
    nested_keys = (
        target_end,
        f"{side}_tool",
        f"{side}_end",
        f"{side}_end_effector",
        side,
    )
    for key in nested_keys:
        nested = end_state.get(key)
        if isinstance(nested, Mapping):
            value = first_numeric_sequence(
                nested,
                ("actual_openness", "openness", "opening", "open_degree"),
            )
            if value is not None:
                return value

    value = first_numeric_sequence(
        end_state,
        (
            f"{side}_actual_openness",
            f"{side}_openness",
            f"{side}_opening",
            "actual_openness",
        ),
    )
    if value is not None:
        return value

    for list_key in ("states", "end_states", "tools"):
        value = extract_actual_openness_from_items(end_state.get(list_key), target_end, side)
        if value is not None:
            return value
    return None


def extract_actual_openness_from_items(
    value: Any,
    target_end: str,
    side: str,
) -> list[float] | None:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return None

    aliases = {target_end, side, f"{side}_tool", f"{side}_end"}
    for item in value:
        if not isinstance(item, Mapping):
            continue
        item_group = first_string_value(item, ("group", "name", "target_end", "end"))
        if item_group not in aliases:
            continue
        openness = first_numeric_sequence(
            item,
            ("actual_openness", "openness", "opening", "open_degree"),
        )
        if openness is not None:
            return openness
    return None


def fallback_actual_openness(target_end: str, opening: float) -> list[float]:
    if target_end == "dual_tool":
        return [opening, opening]
    return [opening]


def first_numeric_sequence(mapping: Mapping[str, Any], keys: Sequence[str]) -> list[float] | None:
    for key in keys:
        value = mapping.get(key)
        numbers = normalize_numeric_sequence(value)
        if numbers is not None:
            return numbers
    return None


def normalize_numeric_sequence(value: Any) -> list[float] | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return [float(value)]
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return None

    numbers: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int | float):
            return None
        numbers.append(float(item))
    return numbers if numbers else None


def first_string_value(mapping: Mapping[str, Any], keys: Sequence[str]) -> str | None:
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def normalize_end_effector_type(value: str) -> str:
    return value.strip().lower()


def build_positions_for_opening(
    end_effector_type: str,
    opening: float,
) -> list[float] | None:
    joint_range = SINGLE_JOINT_END_EFFECTOR_RANGES.get(end_effector_type)
    if joint_range is None:
        return None

    open_position = joint_range["open"]
    closed_position = joint_range["closed"]
    return [closed_position + (open_position - closed_position) * opening]


def build_positions_layout(target_end: str) -> str:
    if target_end == "dual_tool":
        return "left_tool_then_right_tool"
    return target_end


def build_gdk_joint_states(
    agibot_gdk: Any,
    *,
    group: str,
    target_type: str,
    positions: Sequence[float],
) -> Any:
    joint_states = agibot_gdk.JointStates()
    joint_states.group = group
    joint_states.target_type = target_type
    states = []
    for position in positions:
        state = agibot_gdk.JointState()
        state.position = float(position)
        states.append(state)
    joint_states.states = states
    joint_states.nums = len(states)
    return joint_states


def refused_result(
    *,
    stage: str,
    message: str,
    safety_confirmed: bool,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_type": "GdkEndEffectorRuntimeRefused",
        "error_msg": message,
        "safety_gate": {
            "enabled": True,
            "confirmed": safety_confirmed,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {},
    }
    if extra:
        payload.update(dict(extra))
    return payload


def unavailable_result(stage: str, error: Exception) -> dict[str, object]:
    return {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
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
