from __future__ import annotations

import importlib
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from .control_probe import (
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    is_zero_error,
    read_joint_position,
    utc_now_iso,
)
from .motion_runtime import WAIST_JOINTS
from .readonly import GDK_BACKEND, to_jsonable
from .session import GdkSessionImportError, GdkSessionInitError, GdkSessionManager


def run_gdk_current_pose_snapshot(
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """Collect the current joint pose used by the desktop ABS_JOINT form.

    This is read-only by design: it creates ``agibot_gdk.Robot`` and calls only
    status/query methods. Motion commands stay in taskflow GDK runtimes.
    """

    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=True,
            purpose="current_pose",
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error)
    except GdkSessionInitError as error:
        return unavailable_result(
            "gdk_init",
            RuntimeError(str(error)),
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose)

    with lease:
        if lease.agibot_gdk is None:
            return unavailable_result(
                "gdk_session_acquire",
                RuntimeError("GDK session lease missing initialized module"),
            )

        try:
            robot = lease.agibot_gdk.Robot()
        except Exception as error:
            return unavailable_result("create_robot", error)

        try:
            joint_states = robot.get_joint_states()
        except Exception as error:
            return unavailable_result("get_joint_states", error)
        if not isinstance(joint_states, Mapping):
            return unavailable_result(
                "parse_joint_states",
                TypeError("robot.get_joint_states() did not return a mapping"),
            )

        try:
            limits = robot.get_joint_limits()
        except Exception as error:
            return unavailable_result("get_joint_limits", error)
        if not isinstance(limits, Mapping):
            return unavailable_result(
                "parse_joint_limits",
                TypeError("robot.get_joint_limits() did not return a mapping"),
            )

        try:
            motion_status = robot.get_motion_control_status()
        except Exception as error:
            return unavailable_result("get_motion_control_status", error)

        try:
            whole_body_status = robot.get_whole_body_status()
        except Exception as error:
            return unavailable_result("get_whole_body_status", error)
        if not isinstance(whole_body_status, Mapping):
            return unavailable_result(
                "parse_whole_body_status",
                TypeError("robot.get_whole_body_status() did not return a mapping"),
            )

        try:
            snapshot = build_current_pose_snapshot(
                joint_states=joint_states,
                limits=limits,
                motion_status=motion_status,
                whole_body_status=whole_body_status,
            )
        except Exception as error:
            return unavailable_result("parse_joint_states", error)

        snapshot["gdk_init"] = lease.init_result
        snapshot["gdk_session"] = lease.to_payload()
        return snapshot


def build_current_pose_snapshot(
    *,
    joint_states: Mapping[str, Any],
    limits: Mapping[str, Any],
    motion_status: Any,
    whole_body_status: Mapping[str, Any],
) -> dict[str, object]:
    states = joint_states.get("states")
    if not isinstance(states, Sequence) or isinstance(states, str | bytes | bytearray):
        raise TypeError("joint_states['states'] is not a sequence")

    state_items = [state for state in states if isinstance(state, Mapping)]
    states_by_name = {
        state["name"]: state
        for state in state_items
        if isinstance(state.get("name"), str)
    }
    return {
        "available": True,
        "backend": GDK_BACKEND,
        "collectedAt": utc_now_iso(),
        "jointCount": read_joint_count(joint_states.get("nums"), state_items),
        "groups": {
            "left_arm": build_group_snapshot("left_arm", LEFT_ARM_JOINTS, states_by_name, limits),
            "right_arm": build_group_snapshot(
                "right_arm",
                RIGHT_ARM_JOINTS,
                states_by_name,
                limits,
            ),
            "waist": build_group_snapshot("waist", WAIST_JOINTS, states_by_name, limits),
        },
        "nonzeroErrorJoints": build_nonzero_error_joints(state_items),
        "motionStatus": {
            "errorCode": to_jsonable(getattr(motion_status, "error_code", None)),
            "errorMsg": str(getattr(motion_status, "error_msg", "") or ""),
        },
        "wholeBodyStatus": to_jsonable(whole_body_status),
    }


def build_group_snapshot(
    body_part: str,
    joint_names: Sequence[str],
    states_by_name: Mapping[str, Mapping[str, Any]],
    limits: Mapping[str, Any],
) -> dict[str, object]:
    joints = []
    for joint_name in joint_names:
        state = states_by_name.get(joint_name)
        if state is None:
            raise RuntimeError(f"missing joint state: {joint_name}")
        joints.append(
            {
                "name": joint_name,
                "position": read_joint_position(state),
                "limit": read_limit(limits, joint_name),
                "errorCode": to_jsonable(state.get("error_code")),
            }
        )

    return {
        "bodyPart": body_part,
        "jointNames": list(joint_names),
        "positions": [joint["position"] for joint in joints],
        "joints": joints,
    }


def read_joint_count(raw_count: Any, states: Sequence[Mapping[str, Any]]) -> int:
    if isinstance(raw_count, bool):
        return len(states)
    if isinstance(raw_count, int):
        return raw_count
    return len(states)


def read_limit(limits: Mapping[str, Any], joint_name: str) -> dict[str, float] | None:
    limit = limits.get(joint_name)
    if not isinstance(limit, Mapping):
        return None
    return {
        "min": float(limit["min"]),
        "max": float(limit["max"]),
    }


def build_nonzero_error_joints(
    states: Sequence[Mapping[str, Any]],
) -> list[dict[str, object]]:
    failed: list[dict[str, object]] = []
    for state in states:
        error_code = state.get("error_code")
        if is_zero_error(error_code):
            continue
        name = state.get("name")
        failed.append(
            {
                "name": name if isinstance(name, str) and name else "<unknown>",
                "errorCode": to_jsonable(error_code),
            }
        )
    return failed


def busy_result(*, active_purpose: str | None) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_BACKEND,
        "collectedAt": utc_now_iso(),
        "jointCount": 0,
        "groups": {},
        "nonzeroErrorJoints": [],
        "motionStatus": {
            "errorCode": None,
            "errorMsg": "",
        },
        "wholeBodyStatus": {},
        "busy": True,
        "errorStage": "gdk_session_busy",
        "errorType": "GdkSessionBusy",
        "errorMsg": "GDK 正在执行控制动作，当前位姿读取已拒绝",
        "activePurpose": active_purpose,
    }


def unavailable_result(
    stage: str,
    error: Exception,
    *,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": GDK_BACKEND,
        "collectedAt": utc_now_iso(),
        "jointCount": 0,
        "groups": {},
        "nonzeroErrorJoints": [],
        "motionStatus": {
            "errorCode": None,
            "errorMsg": "",
        },
        "wholeBodyStatus": {},
        "errorStage": stage,
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }
    if extra:
        result.update(dict(extra))
    return result
