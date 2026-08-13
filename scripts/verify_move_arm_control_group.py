#!/usr/bin/env python3
"""Verify move_arm_joint control_group=0/1 semantics on a real robot.

Run this script on the robot host only. It intentionally does not import the
executor runtime, subscribe to MQTT, or execute a taskflow.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from typing import Any

CONFIRM_TOKEN = "VERIFY_MOVE_ARM_GROUP_01"
DEFAULT_VELOCITY = 0.02
DEFAULT_NUDGE_RAD = 0.003
DEFAULT_SETTLE_SECONDS = 2.0

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
ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify agibot_gdk.Robot.move_arm_joint control_group=0/1.",
    )
    parser.add_argument(
        "--case",
        choices=(
            "readonly",
            "hold-14-left-group",
            "hold-14-right-group",
            "hold-left7",
            "hold-right7",
            "nudge-left7",
            "nudge-right7",
        ),
        default="readonly",
        help="Run exactly one verification case.",
    )
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY)
    parser.add_argument("--nudge-rad", type=float, default=DEFAULT_NUDGE_RAD)
    parser.add_argument("--settle-seconds", type=float, default=DEFAULT_SETTLE_SECONDS)
    args = parser.parse_args()

    if args.velocity <= 0:
        raise SystemExit("--velocity must be > 0")
    if args.nudge_rad <= 0:
        raise SystemExit("--nudge-rad must be > 0")
    if args.settle_seconds < 0:
        raise SystemExit("--settle-seconds must be >= 0")

    if args.case != "readonly":
        require_control_confirmation()

    try:
        import agibot_gdk  # type: ignore[import-not-found]
    except Exception as error:
        print_json(
            {
                "ok": False,
                "case": args.case,
                "stage": "import_agibot_gdk",
                "error_type": type(error).__name__,
                "error_msg": str(error),
            }
        )
        return 2

    init_result = call_gdk_init_if_available(agibot_gdk)
    robot = agibot_gdk.Robot()
    try:
        result = run_case(
            robot,
            case=args.case,
            velocity=args.velocity,
            nudge_rad=args.nudge_rad,
            settle_seconds=args.settle_seconds,
        )
        result["gdk_init"] = init_result
        print_json(result)
        return 0 if result.get("ok") is True else 1
    finally:
        release_result = call_gdk_release_if_available(agibot_gdk)
        if release_result["called"]:
            print_json({"event": "gdk_release", "gdk_release": release_result})


def run_case(
    robot: Any,
    *,
    case: str,
    velocity: float,
    nudge_rad: float,
    settle_seconds: float,
) -> dict[str, object]:
    before = collect_snapshot(robot)
    validate_snapshot(before)

    if case == "readonly":
        return {
            "ok": True,
            "case": case,
            "collected_at": utc_now_iso(),
            "joint_order": ARM_JOINTS,
            "positions14": before.positions14,
            "left_positions7": before.left_positions7,
            "right_positions7": before.right_positions7,
            "motion_status": to_jsonable(before.motion_status),
            "whole_body_status": to_jsonable(before.whole_body_status),
        }

    if case == "hold-14-left-group":
        return execute_move_case(
            robot,
            case=case,
            before=before,
            positions=before.positions14,
            velocities=[velocity] * 14,
            control_group=0,
            settle_seconds=settle_seconds,
        )

    if case == "hold-14-right-group":
        return execute_move_case(
            robot,
            case=case,
            before=before,
            positions=before.positions14,
            velocities=[velocity] * 14,
            control_group=1,
            settle_seconds=settle_seconds,
        )

    if case == "hold-left7":
        return execute_move_case(
            robot,
            case=case,
            before=before,
            positions=before.left_positions7,
            velocities=[velocity] * 7,
            control_group=0,
            settle_seconds=settle_seconds,
        )

    if case == "hold-right7":
        return execute_move_case(
            robot,
            case=case,
            before=before,
            positions=before.right_positions7,
            velocities=[velocity] * 7,
            control_group=1,
            settle_seconds=settle_seconds,
        )

    if case == "nudge-left7":
        target = list(before.left_positions7)
        target[-1] += nudge_rad
        assert_target_within_limits(before.limits, LEFT_ARM_JOINTS[-1], target[-1])
        nudge = execute_move_case(
            robot,
            case=case,
            before=before,
            positions=target,
            velocities=[velocity] * 7,
            control_group=0,
            settle_seconds=settle_seconds,
        )
        return_to_origin = execute_raw_move(
            robot,
            positions=before.left_positions7,
            velocities=[velocity] * 7,
            control_group=0,
            settle_seconds=settle_seconds,
        )
        after_return = collect_snapshot(robot)
        return {
            **nudge,
            "nudge_rad": nudge_rad,
            "return_to_origin": return_to_origin,
            "after_return_diff": diff_summary(after_return.positions14, before.positions14),
        }

    if case == "nudge-right7":
        target = list(before.right_positions7)
        target[-1] += nudge_rad
        assert_target_within_limits(before.limits, RIGHT_ARM_JOINTS[-1], target[-1])
        nudge = execute_move_case(
            robot,
            case=case,
            before=before,
            positions=target,
            velocities=[velocity] * 7,
            control_group=1,
            settle_seconds=settle_seconds,
        )
        return_to_origin = execute_raw_move(
            robot,
            positions=before.right_positions7,
            velocities=[velocity] * 7,
            control_group=1,
            settle_seconds=settle_seconds,
        )
        after_return = collect_snapshot(robot)
        return {
            **nudge,
            "nudge_rad": nudge_rad,
            "return_to_origin": return_to_origin,
            "after_return_diff": diff_summary(after_return.positions14, before.positions14),
        }

    raise ValueError(f"unsupported case: {case}")


def execute_move_case(
    robot: Any,
    *,
    case: str,
    before: ArmSnapshot,
    positions: Sequence[float],
    velocities: Sequence[float],
    control_group: int,
    settle_seconds: float,
) -> dict[str, object]:
    move = execute_raw_move(
        robot,
        positions=positions,
        velocities=velocities,
        control_group=control_group,
        settle_seconds=settle_seconds,
    )
    after = collect_snapshot(robot)
    diff = diff_summary(after.positions14, before.positions14)
    return {
        "ok": move["ok"],
        "case": case,
        "collected_at": utc_now_iso(),
        "method": "move_arm_joint",
        "control_group": control_group,
        "positions_len": len(positions),
        "velocities_len": len(velocities),
        "move": move,
        "joint_order": ARM_JOINTS,
        "before_positions14": before.positions14,
        "after_positions14": after.positions14,
        "diff": diff,
    }


def execute_raw_move(
    robot: Any,
    *,
    positions: Sequence[float],
    velocities: Sequence[float],
    control_group: int,
    settle_seconds: float,
) -> dict[str, object]:
    try:
        ret = robot.move_arm_joint(list(positions), list(velocities), control_group)
    except Exception as error:
        return {
            "ok": False,
            "error_type": type(error).__name__,
            "error_msg": str(error),
        }
    if settle_seconds:
        time.sleep(settle_seconds)
    return {
        "ok": is_zero_return(ret),
        "return": to_jsonable(ret),
    }


class ArmSnapshot:
    def __init__(
        self,
        *,
        positions14: list[float],
        limits: Mapping[str, object],
        motion_status: Any,
        whole_body_status: Any,
    ) -> None:
        self.positions14 = positions14
        self.left_positions7 = positions14[:7]
        self.right_positions7 = positions14[7:]
        self.limits = limits
        self.motion_status = motion_status
        self.whole_body_status = whole_body_status


def collect_snapshot(robot: Any) -> ArmSnapshot:
    joint_states = robot.get_joint_states()
    limits = robot.get_joint_limits()
    motion_status = robot.get_motion_control_status()
    whole_body_status = robot.get_whole_body_status()

    if not isinstance(joint_states, Mapping):
        raise TypeError("get_joint_states() did not return a mapping")
    if not isinstance(limits, Mapping):
        raise TypeError("get_joint_limits() did not return a mapping")
    states = joint_states.get("states")
    if not isinstance(states, Sequence) or isinstance(states, str | bytes | bytearray):
        raise TypeError("joint_states['states'] is not a sequence")

    states_by_name = {
        state["name"]: state
        for state in states
        if isinstance(state, Mapping) and isinstance(state.get("name"), str)
    }
    positions: list[float] = []
    for joint_name in ARM_JOINTS:
        state = states_by_name.get(joint_name)
        if state is None:
            raise RuntimeError(f"missing joint state: {joint_name}")
        if not is_zero_return(state.get("error_code")):
            raise RuntimeError(f"{joint_name} error_code={state.get('error_code')}")
        position = read_joint_position(state)
        assert_target_within_limits(limits, joint_name, position)
        positions.append(position)

    return ArmSnapshot(
        positions14=positions,
        limits=limits,
        motion_status=motion_status,
        whole_body_status=whole_body_status,
    )


def validate_snapshot(snapshot: ArmSnapshot) -> None:
    motion_error_code = getattr(snapshot.motion_status, "error_code", None)
    if motion_error_code is not None and not is_zero_return(motion_error_code):
        raise RuntimeError(f"motion control error_code={motion_error_code}")

    whole = snapshot.whole_body_status
    if isinstance(whole, Mapping):
        for key in (
            "left_arm_error",
            "right_arm_error",
            "left_end_error",
            "right_end_error",
            "waist_error",
            "chassis_error",
        ):
            value = whole.get(key)
            if value is not None and not is_zero_return(value):
                raise RuntimeError(f"whole_body_status {key}={value}")
        for key in ("left_arm_estop", "right_arm_estop"):
            if whole.get(key) is True:
                raise RuntimeError(f"whole_body_status {key}=True")


def read_joint_position(state: Mapping[str, object]) -> float:
    raw = state.get("motor_position", state.get("position"))
    if isinstance(raw, bool) or not isinstance(raw, int | float):
        raise TypeError(f"invalid joint position for {state.get('name')}: {raw!r}")
    return float(raw)


def assert_target_within_limits(
    limits: Mapping[str, object],
    joint_name: str,
    target: float,
) -> None:
    raw_limit = limits.get(joint_name)
    if not isinstance(raw_limit, Mapping):
        raise RuntimeError(f"missing joint limit: {joint_name}")
    minimum = read_float_limit(raw_limit, "min", joint_name)
    maximum = read_float_limit(raw_limit, "max", joint_name)
    if target < minimum or target > maximum:
        raise RuntimeError(
            f"{joint_name} target {target} outside limit [{minimum}, {maximum}]"
        )


def read_float_limit(limit: Mapping[str, object], key: str, joint_name: str) -> float:
    raw = limit.get(key)
    if isinstance(raw, bool) or not isinstance(raw, int | float):
        raise TypeError(f"invalid {joint_name} limit {key}: {raw!r}")
    return float(raw)


def diff_summary(after: Sequence[float], before: Sequence[float]) -> dict[str, object]:
    diffs = [float(a) - float(b) for a, b in zip(after, before, strict=True)]
    left = diffs[:7]
    right = diffs[7:]
    return {
        "max_abs": max(abs(item) for item in diffs),
        "left_max_abs": max(abs(item) for item in left),
        "right_max_abs": max(abs(item) for item in right),
        "left_j7": left[-1],
        "right_j7": right[-1],
        "diffs": diffs,
    }


def require_control_confirmation() -> None:
    if os.environ.get("ENABLE_GDK_CONTROL") != "1":
        raise SystemExit("ENABLE_GDK_CONTROL must be 1 for motion cases")
    if os.environ.get("CONFIRM_GDK_CONTROL") != CONFIRM_TOKEN:
        raise SystemExit(f"CONFIRM_GDK_CONTROL must be {CONFIRM_TOKEN}")


def call_gdk_init_if_available(agibot_gdk: Any) -> dict[str, object]:
    gdk_init = getattr(agibot_gdk, "gdk_init", None)
    if not callable(gdk_init):
        return {"called": False, "success": True, "return": None}
    try:
        ret = gdk_init()
    except Exception as error:
        return {
            "called": True,
            "success": False,
            "error_type": type(error).__name__,
            "error_msg": str(error),
        }
    return {"called": True, "success": is_success_gdk_return(agibot_gdk, ret), "return": str(ret)}


def call_gdk_release_if_available(agibot_gdk: Any) -> dict[str, object]:
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
    return {"called": True, "success": is_success_gdk_return(agibot_gdk, ret), "return": str(ret)}


def is_success_gdk_return(agibot_gdk: Any, ret: Any) -> bool:
    success = getattr(agibot_gdk, "kSuccess", None)
    if success is not None and ret == success:
        return True
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    enum_success = getattr(gdk_res, "kSuccess", None) if gdk_res is not None else None
    if enum_success is not None and ret == enum_success:
        return True
    return is_zero_return(ret)


def is_zero_return(value: Any) -> bool:
    if isinstance(value, bool):
        return value is False
    try:
        return int(value) == 0
    except (TypeError, ValueError):
        return False


def to_jsonable(value: Any) -> object:
    if value is None or isinstance(value, str | int | float | bool):
        return value
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "__dict__"):
        return to_jsonable(vars(value))
    return repr(value)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def print_json(payload: Mapping[str, object]) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print_json(
            {
                "ok": False,
                "stage": "unhandled_exception",
                "error_type": type(error).__name__,
                "error_msg": str(error),
            }
        )
        raise SystemExit(1) from error
