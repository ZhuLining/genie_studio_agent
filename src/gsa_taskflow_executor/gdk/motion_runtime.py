from __future__ import annotations

import importlib
import os
from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from math import isfinite
from typing import Any

from gsa_taskflow_executor.taskflow.parser import (
    MOTION_SPEED_MAX,
    MOTION_SPEED_MIN,
    MotionPlanParams,
    MotionPlanTarget,
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
from .session import (
    PROCESS_MANAGED_RELEASE_RESULT,
    GdkSessionImportError,
    GdkSessionInitError,
    GdkSessionManager,
)
from .subprocess_runtime import GDK_PARENT_LOCK_POLICY, run_motion_abs_joint_in_subprocess

ACTION_TASKFLOW_ABS_JOINT = "taskflow_abs_joint"
TASKFLOW_ABS_JOINT_CONFIRMATION = "TASKFLOW_ABS_JOINT"
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
    """当前 GDK 控制模式不支持关节位置 move_* 接口。"""

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
    """Execute verified ABS_JOINT taskflow targets through agibot_gdk.

    This runtime is intentionally narrow: it only supports the G2 interfaces
    verified in docs, and refuses to import GDK until the taskflow control
    safety gate is explicitly confirmed.
    """

    env = environ if environ is not None else os.environ
    gate_result = check_taskflow_abs_joint_safety_gate(env)
    if gate_result is not None:
        return gate_result

    velocity_validation_result = validate_motion_velocity(motion_params.speed)
    if velocity_validation_result is not None:
        return velocity_validation_result

    validation_result = validate_abs_joint_targets(motion_params.targets)
    if validation_result is not None:
        return validation_result

    if should_use_in_process_runtime(import_module, session_manager):
        return run_gdk_motion_plan_abs_joint_in_process(
            motion_params,
            import_module=import_module,
            session_manager=session_manager,
        )

    # 父进程只持有调度互斥锁，不导入 GDK；真正的 C 扩展调用在子进程内完成，
    # 超时后可以杀掉子进程并释放父进程 worker，避免 MQTT 执行队列永久卡死。
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

    return result


def should_use_in_process_runtime(
    import_module: Callable[[str], Any],
    session_manager: GdkSessionManager | None,
) -> bool:
    if import_module is not importlib.import_module:
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


def execute_abs_joint_targets(
    robot: Any,
    motion_params: MotionPlanParams,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
    targets_by_part = {
        target.body_part: read_abs_joint_action_data(target)
        for target in motion_params.targets
    }
    executed_groups: list[dict[str, object]] = []

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
    origin = collect_dual_arm_snapshot(robot)
    ensure_supported_move_control_mode(origin.motion_status, agibot_gdk=agibot_gdk)
    target_positions = list(origin.positions)

    if "left_arm" in targets_by_part:
        target_positions[: len(LEFT_ARM_JOINTS)] = [
            float(value) for value in targets_by_part["left_arm"]
        ]
    if "right_arm" in targets_by_part:
        target_positions[len(LEFT_ARM_JOINTS) :] = [
            float(value) for value in targets_by_part["right_arm"]
        ]

    limits = robot.get_joint_limits()
    if not isinstance(limits, Mapping):
        raise TypeError("robot.get_joint_limits() did not return a mapping")
    assert_positions_within_limits(limits, target_positions)

    velocities = [velocity] * len(DUAL_ARM_JOINTS)
    move_return = robot.move_arm_joint(target_positions, velocities, CONTROL_GROUP_DUAL_ARM)
    if not is_zero_error(move_return):
        raise RuntimeError(f"move_arm_joint returned {move_return!r}")

    after = collect_dual_arm_snapshot(robot)
    return {
        "body_part": "arms",
        "requested_body_parts": list(targets_by_part.keys()),
        "method": "move_arm_joint",
        "control_group": CONTROL_GROUP_DUAL_ARM,
        "joint_order": list(DUAL_ARM_JOINTS),
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


def execute_waist_abs_joint_target(
    robot: Any,
    target_positions: Sequence[float],
    velocity: float,
    *,
    agibot_gdk: Any | None = None,
) -> dict[str, object]:
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
    if len(positions) != len(WAIST_JOINTS):
        raise RuntimeError(f"expected {len(WAIST_JOINTS)} waist positions, got {len(positions)}")
    for joint_name, position in zip(WAIST_JOINTS, positions, strict=True):
        assert_joint_within_limit(limits, joint_name, float(position))


def read_abs_joint_action_data(target: MotionPlanTarget) -> list[float]:
    if target.control_type != "ABS_JOINT":
        raise RuntimeError(f"{target.body_part} 只支持 ABS_JOINT，当前为 {target.control_type}")
    if isinstance(target.action_data, str):
        raise RuntimeError(f"{target.body_part}.action_data 变量引用未解析为关节数组")
    return [float(value) for value in target.action_data]


def validate_abs_joint_targets(
    targets: Sequence[MotionPlanTarget],
) -> dict[str, object] | None:
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


def snapshot_raw(snapshot: JointSnapshot) -> dict[str, object]:
    return {
        "motion_status": to_jsonable(snapshot.motion_status),
        "whole_body_status": to_jsonable(snapshot.whole_body_status),
    }


def refused_control_mode_result(error: UnsupportedGdkControlModeError) -> dict[str, object]:
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
    return {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_ABS_JOINT,
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
