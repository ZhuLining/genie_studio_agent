from __future__ import annotations

import importlib
import os
from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from typing import Any

from .gdk_control_probe import (
    CONTROL_GROUP_DUAL_ARM,
    DEFAULT_VELOCITY,
    DUAL_ARM_JOINTS,
    LEFT_ARM_JOINTS,
    JointSnapshot,
    assert_joint_within_limit,
    assert_positions_within_limits,
    collect_dual_arm_snapshot,
    initialize_gdk,
    is_zero_error,
    position_diffs,
    read_joint_position,
    release_gdk,
    utc_now_iso,
    validate_motion_status,
    validate_whole_body_status,
)
from .gdk_readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from .taskflow_parser import MotionPlanParams, MotionPlanTarget

ACTION_TASKFLOW_ABS_JOINT = "taskflow_abs_joint"
TASKFLOW_ABS_JOINT_CONFIRMATION = "TASKFLOW_ABS_JOINT"
WAIST_JOINTS = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]


def run_gdk_motion_plan_abs_joint(
    motion_params: MotionPlanParams,
    *,
    environ: Mapping[str, str] | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
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

    validation_result = validate_abs_joint_targets(motion_params.targets)
    if validation_result is not None:
        return validation_result

    agibot_gdk = None
    gdk_initialized = False
    result: dict[str, object] = {}

    try:
        agibot_gdk = import_module(GDK_MODULE_NAME)
    except Exception as error:
        return unavailable_result("import_agibot_gdk", error)

    try:
        init_result = initialize_gdk(agibot_gdk)
    except Exception as error:
        return unavailable_result("gdk_init", error)

    if init_result.get("called") is True and init_result.get("success") is not True:
        return refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            extra={"gdk_init": init_result},
        )

    gdk_initialized = bool(init_result.get("called"))

    try:
        robot = agibot_gdk.Robot()
        result = execute_abs_joint_targets(robot, motion_params)
        result["gdk_init"] = init_result
    except Exception as error:
        result = unavailable_result("execute_abs_joint_targets", error)
        result["gdk_init"] = init_result
    finally:
        if agibot_gdk is not None and gdk_initialized and result:
            result["gdk_release"] = release_gdk(agibot_gdk)

    return result


def execute_abs_joint_targets(robot: Any, motion_params: MotionPlanParams) -> dict[str, object]:
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
    if arm_targets:
        executed_groups.append(execute_arm_abs_joint_targets(robot, arm_targets))

    if "waist" in targets_by_part:
        executed_groups.append(execute_waist_abs_joint_target(robot, targets_by_part["waist"]))

    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_ABS_JOINT,
        "collected_at": utc_now_iso(),
        "speed": motion_params.speed,
        "timeout": motion_params.timeout,
        "gdk_velocity": DEFAULT_VELOCITY,
        "velocity_source": "verified_default",
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
) -> dict[str, object]:
    origin = collect_dual_arm_snapshot(robot)
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

    velocities = [DEFAULT_VELOCITY] * len(DUAL_ARM_JOINTS)
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
) -> dict[str, object]:
    origin = collect_waist_snapshot(robot)
    target = [float(value) for value in target_positions]
    limits = robot.get_joint_limits()
    if not isinstance(limits, Mapping):
        raise TypeError("robot.get_joint_limits() did not return a mapping")
    assert_waist_positions_within_limits(limits, target)

    velocities = [DEFAULT_VELOCITY] * len(WAIST_JOINTS)
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
