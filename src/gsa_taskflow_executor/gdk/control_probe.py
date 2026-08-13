"""GDK 控制探针 — 手动控制操作和关节状态工具函数。

提供 hold_current、nudge_left_j7、nudge_right_j7 等现场测试命令，
以及关节快照采集、限位校验、位置差计算等运动规划的共享工具。
"""

from __future__ import annotations

import importlib
import os
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from .readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable

LEFT_ARM_JOINTS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_ARM_JOINTS = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]
DUAL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
CONTROL_GROUP_DUAL_ARM = 2
DEFAULT_VELOCITY = 0.02
NUDGE_J7_DELTA_RAD = 0.005

ACTION_HOLD_CURRENT = "hold_current"
ACTION_NUDGE_LEFT_J7 = "nudge_left_j7_0p005"
ACTION_NUDGE_RIGHT_J7 = "nudge_right_j7_0p005"

ACTION_CONFIRMATION_TOKENS = {
    ACTION_HOLD_CURRENT: "HOLD_CURRENT_DUAL_ARM",
    ACTION_NUDGE_LEFT_J7: "NUDGE_LEFT_J7_0P005",
    ACTION_NUDGE_RIGHT_J7: "NUDGE_RIGHT_J7_0P005",
}

ALLOWED_ACTIONS = tuple(ACTION_CONFIRMATION_TOKENS)


@dataclass(frozen=True)
class JointSnapshot:
    positions: list[float]
    motion_status: object
    whole_body_status: Mapping[str, Any]


def run_gdk_control_probe(
    action: str,
    *,
    environ: Mapping[str, str] | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
    sleep: Callable[[float], None] = time.sleep,
    settle_seconds: float = 2.0,
) -> dict[str, object]:
    """Run one manually gated GDK control probe action.

    This function intentionally refuses to import or touch GDK unless both
    environment gates are present. It is for SSH/manual smoke tests only.
    """

    env = environ if environ is not None else os.environ
    if action not in ACTION_CONFIRMATION_TOKENS:
        return refused_result(
            action=action,
            stage="validate_action",
            message=f"unsupported GDK control probe action: {action}",
        )

    gate_result = check_safety_gate(action, env)
    if gate_result is not None:
        return gate_result

    agibot_gdk = None
    gdk_initialized = False

    try:
        agibot_gdk = import_module(GDK_MODULE_NAME)
    except Exception as error:
        return unavailable_result(action, "import_agibot_gdk", error)

    try:
        init_result = initialize_gdk(agibot_gdk)
    except Exception as error:
        return unavailable_result(action, "gdk_init", error)

    if init_result.get("called") is True and init_result.get("success") is not True:
        return refused_result(
            action=action,
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            extra={"gdk_init": init_result},
        )

    gdk_initialized = bool(init_result.get("called"))
    result: dict[str, object] = {}

    try:
        robot_factory = agibot_gdk.Robot
        robot = robot_factory()
        result = execute_action(
            action=action,
            robot=robot,
            sleep=sleep,
            settle_seconds=settle_seconds,
        )
        result["gdk_init"] = init_result
    except Exception as error:
        result = unavailable_result(action, "execute_action", error)
        result["gdk_init"] = init_result
    finally:
        if agibot_gdk is not None and gdk_initialized and result:
            result["gdk_release"] = release_gdk(agibot_gdk)

    return result


def execute_action(
    *,
    action: str,
    robot: Any,
    sleep: Callable[[float], None],
    settle_seconds: float,
) -> dict[str, object]:
    origin = collect_dual_arm_snapshot(robot)
    velocities = [DEFAULT_VELOCITY] * len(DUAL_ARM_JOINTS)

    if action == ACTION_HOLD_CURRENT:
        move_return = robot.move_arm_joint(
            list(origin.positions),
            list(velocities),
            CONTROL_GROUP_DUAL_ARM,
        )
        sleep(settle_seconds)
        after = collect_dual_arm_snapshot(robot)
        return success_result(
            action=action,
            origin=origin,
            target_positions=list(origin.positions),
            after=after,
            move_returns={"hold_current": to_jsonable(move_return)},
            velocities=velocities,
        )

    if action == ACTION_NUDGE_LEFT_J7:
        return execute_j7_nudge_action(
            action=action,
            robot=robot,
            origin=origin,
            velocities=velocities,
            joint_index=len(LEFT_ARM_JOINTS) - 1,
            joint_name=LEFT_ARM_JOINTS[-1],
            diff_key="mid_left_j7_diff",
            sleep=sleep,
            settle_seconds=settle_seconds,
        )

    if action == ACTION_NUDGE_RIGHT_J7:
        return execute_j7_nudge_action(
            action=action,
            robot=robot,
            origin=origin,
            velocities=velocities,
            joint_index=len(DUAL_ARM_JOINTS) - 1,
            joint_name=RIGHT_ARM_JOINTS[-1],
            diff_key="mid_right_j7_diff",
            sleep=sleep,
            settle_seconds=settle_seconds,
        )

    raise ValueError(f"unsupported action after validation: {action}")


def execute_j7_nudge_action(
    *,
    action: str,
    robot: Any,
    origin: JointSnapshot,
    velocities: list[float],
    joint_index: int,
    joint_name: str,
    diff_key: str,
    sleep: Callable[[float], None],
    settle_seconds: float,
) -> dict[str, object]:
    target = list(origin.positions)
    target[joint_index] += NUDGE_J7_DELTA_RAD
    assert_positions_within_limits(robot.get_joint_limits(), target)

    nudge_return = robot.move_arm_joint(target, velocities, CONTROL_GROUP_DUAL_ARM)
    sleep(settle_seconds)
    mid = collect_dual_arm_snapshot(robot)

    return_return = robot.move_arm_joint(
        list(origin.positions),
        velocities,
        CONTROL_GROUP_DUAL_ARM,
    )
    sleep(settle_seconds)
    after = collect_dual_arm_snapshot(robot)

    result = success_result(
        action=action,
        origin=origin,
        target_positions=target,
        after=after,
        move_returns={
            "nudge": to_jsonable(nudge_return),
            "return_to_origin": to_jsonable(return_return),
        },
        velocities=velocities,
    )
    result["nudge_joint_name"] = joint_name
    result["nudge_delta_rad"] = NUDGE_J7_DELTA_RAD
    result["mid_positions"] = mid.positions
    result["mid_diffs"] = position_diffs(mid.positions, origin.positions)
    result[diff_key] = mid.positions[joint_index] - origin.positions[joint_index]
    return result


def collect_dual_arm_snapshot(robot: Any) -> JointSnapshot:
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
    for joint_name in DUAL_ARM_JOINTS:
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


def validate_motion_status(motion_status: Any) -> None:
    error_code = getattr(motion_status, "error_code", 0)
    if not is_zero_error(error_code):
        error_msg = getattr(motion_status, "error_msg", "")
        raise RuntimeError(f"motion control error_code={error_code} error_msg={error_msg!r}")


def validate_whole_body_status(whole_body_status: Mapping[str, Any]) -> None:
    if not isinstance(whole_body_status, Mapping):
        raise TypeError("robot.get_whole_body_status() did not return a mapping")

    for key in (
        "left_arm_error",
        "right_arm_error",
        "left_end_error",
        "right_end_error",
        "waist_error",
        "lift_error",
        "neck_error",
        "chassis_error",
    ):
        if not is_zero_error(whole_body_status.get(key)):
            raise RuntimeError(f"{key}={whole_body_status.get(key)}")

    if whole_body_status.get("left_arm_estop") or whole_body_status.get("right_arm_estop"):
        raise RuntimeError("arm estop is active")


def read_joint_position(state: Mapping[str, Any]) -> float:
    if "motor_position" in state:
        return float(state["motor_position"])
    return float(state["position"])


def assert_positions_within_limits(
    limits: Mapping[str, Any],
    positions: Sequence[float],
) -> None:
    if len(positions) != len(DUAL_ARM_JOINTS):
        raise RuntimeError(
            f"expected {len(DUAL_ARM_JOINTS)} positions, got {len(positions)}"
        )
    for joint_name, position in zip(DUAL_ARM_JOINTS, positions, strict=True):
        assert_joint_within_limit(limits, joint_name, float(position))


def assert_joint_within_limit(
    limits: Mapping[str, Any],
    joint_name: str,
    position: float,
) -> None:
    limit = limits.get(joint_name)
    if not isinstance(limit, Mapping):
        raise RuntimeError(f"{joint_name} has no joint limit")
    min_value = float(limit["min"])
    max_value = float(limit["max"])
    if not (min_value <= position <= max_value):
        raise RuntimeError(f"{joint_name} position {position} outside limit {limit}")


def check_safety_gate(
    action: str,
    env: Mapping[str, str],
) -> dict[str, object] | None:
    expected_confirmation = ACTION_CONFIRMATION_TOKENS[action]
    if env.get("ENABLE_GDK_CONTROL") != "1":
        return refused_result(
            action=action,
            stage="safety_gate",
            message="ENABLE_GDK_CONTROL must be 1",
            extra={"expected_confirmation": expected_confirmation},
        )
    if env.get("CONFIRM_GDK_CONTROL") != expected_confirmation:
        return refused_result(
            action=action,
            stage="safety_gate",
            message="CONFIRM_GDK_CONTROL mismatch",
            extra={"expected_confirmation": expected_confirmation},
        )
    return None


def initialize_gdk(agibot_gdk: Any) -> dict[str, object]:
    gdk_init = getattr(agibot_gdk, "gdk_init", None)
    if not callable(gdk_init):
        return {"called": False, "success": True, "return": None}

    ret = gdk_init()
    return {
        "called": True,
        "success": is_gdk_init_success(agibot_gdk, ret),
        "return": to_jsonable(ret),
    }


def is_gdk_init_success(agibot_gdk: Any, ret: Any) -> bool:
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    success_value = getattr(gdk_res, "kSuccess", None) if gdk_res is not None else None
    if success_value is not None and ret == success_value:
        return True
    if isinstance(ret, int):
        return ret == 0
    return "kSuccess" in repr(ret)


def release_gdk(agibot_gdk: Any) -> dict[str, object]:
    gdk_release = getattr(agibot_gdk, "gdk_release", None)
    if not callable(gdk_release):
        return {"called": False, "success": True, "return": None}
    try:
        ret = gdk_release()
    except Exception as error:
        return {
            "called": True,
            "success": False,
            "error_type": type(error).__name__,
            "error_msg": str(error),
        }
    return {"called": True, "success": True, "return": to_jsonable(ret)}


def success_result(
    *,
    action: str,
    origin: JointSnapshot,
    target_positions: list[float],
    after: JointSnapshot,
    move_returns: dict[str, object],
    velocities: list[float],
) -> dict[str, object]:
    diffs = position_diffs(after.positions, origin.positions)
    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "control_group": CONTROL_GROUP_DUAL_ARM,
        "joint_order": list(DUAL_ARM_JOINTS),
        "positions_len": len(target_positions),
        "velocities_len": len(velocities),
        "velocities": velocities,
        "origin_positions": origin.positions,
        "target_positions": target_positions,
        "after_positions": after.positions,
        "diffs": diffs,
        "max_abs_diff": max((abs(diff) for diff in diffs), default=0.0),
        "move_returns": move_returns,
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": ACTION_CONFIRMATION_TOKENS[action],
        },
        "raw": {
            "motion_status": to_jsonable(origin.motion_status),
            "whole_body_status": to_jsonable(origin.whole_body_status),
        },
    }


def refused_result(
    *,
    action: str,
    stage: str,
    message: str,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "control_group": CONTROL_GROUP_DUAL_ARM,
        "joint_order": list(DUAL_ARM_JOINTS),
        "error_stage": stage,
        "error_type": "GdkControlProbeRefused",
        "error_msg": message,
        "safety_gate": {
            "enabled": True,
            "confirmed": False,
        },
        "raw": {},
    }
    if extra:
        payload.update(dict(extra))
    return payload


def unavailable_result(
    action: str,
    stage: str,
    error: Exception,
) -> dict[str, object]:
    return {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "control_group": CONTROL_GROUP_DUAL_ARM,
        "joint_order": list(DUAL_ARM_JOINTS),
        "error_stage": stage,
        "error_type": type(error).__name__,
        "error_msg": str(error),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": ACTION_CONFIRMATION_TOKENS.get(action),
        },
        "raw": {},
    }


def position_diffs(
    positions: Sequence[float],
    origin_positions: Sequence[float],
) -> list[float]:
    return [
        float(position) - float(origin)
        for position, origin in zip(positions, origin_positions, strict=True)
    ]


def is_zero_error(error_code: Any) -> bool:
    if error_code is None:
        return True
    if isinstance(error_code, bool):
        return not error_code
    if isinstance(error_code, int | float):
        return error_code == 0
    if isinstance(error_code, str):
        return error_code.strip() in {"", "0"}
    return False


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()
