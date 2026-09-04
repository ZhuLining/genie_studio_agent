"""GDK 当前关节位姿快照。

通过只读 GDK 接口采集 arm/waist 关节位置、速度、力矩，含运动控制状态校验。
支持显式多帧恢复确认（timeout/cancel 后普通位姿查询不会解锁控制）。
"""

from __future__ import annotations

import importlib
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from .control_probe import (
    ARM_FRAME_NAMES,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    extract_arm_frame_poses,
    is_zero_error,
    read_joint_position,
    utc_now_iso,
)
from .motion_runtime import WAIST_JOINTS
from .readonly import GDK_BACKEND, to_jsonable
from .recovery import (
    GDK_RECOVERY_NOT_CONFIRMED_CODE,
    GdkRecoveryStabilityPolicy,
    attach_gdk_recovery_payload,
    confirm_gdk_recovery_from_snapshots,
    current_gdk_recovery_payload,
)
from .session import GdkSessionImportError, GdkSessionInitError, GdkSessionManager

ACTION_CONFIRM_GDK_RECOVERY = "confirm_gdk_recovery"


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
        attach_gdk_recovery_payload(snapshot, current_gdk_recovery_payload())
        return snapshot


def run_gdk_recovery_confirmation_snapshot(
    *,
    sample_count: int,
    sample_interval_seconds: float,
    max_joint_velocity: float,
    max_position_delta: float,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """显式恢复确认：多帧只读采样停稳后才清除 STOP_UNCONFIRMED。

    普通 get_current_pose 只能读取状态，不能解除恢复门。这里独立成确认动作，
    是为了让 UI/现场操作把“看一眼当前位姿”和“确认机器人已停稳可继续控制”分开。
    """

    policy = GdkRecoveryStabilityPolicy(
        sample_count=sample_count,
        sample_interval_seconds=sample_interval_seconds,
        max_joint_velocity=max_joint_velocity,
        max_position_delta=max_position_delta,
    )
    samples: list[Mapping[str, Any]] = []
    for index in range(policy.sample_count):
        snapshot = run_gdk_current_pose_snapshot(
            import_module=import_module,
            session_manager=session_manager,
        )
        samples.append(snapshot)
        if snapshot.get("available") is not True:
            return build_recovery_confirmation_result(
                confirmed=False,
                policy=policy,
                samples=samples,
                gdk_recovery={
                    "confirmed": False,
                    "code": GDK_RECOVERY_NOT_CONFIRMED_CODE,
                    "reason": "snapshot_unavailable",
                    "failed_sample_index": index,
                    "snapshot": snapshot,
                },
            )
        if index < policy.sample_count - 1:
            sleep(policy.sample_interval_seconds)

    confirmation = confirm_gdk_recovery_from_snapshots(samples, policy)
    confirmed = isinstance(confirmation, Mapping) and confirmation.get("confirmed") is True
    return build_recovery_confirmation_result(
        confirmed=confirmed,
        policy=policy,
        samples=samples,
        gdk_recovery=confirmation,
    )


def build_recovery_confirmation_result(
    *,
    confirmed: bool,
    policy: GdkRecoveryStabilityPolicy,
    samples: Sequence[Mapping[str, Any]],
    gdk_recovery: Mapping[str, object] | None,
) -> dict[str, object]:
    """构建显式恢复确认响应，避免把完整 GDK raw 无限塞进 MQTT。"""

    result: dict[str, object] = {
        "available": confirmed,
        "backend": GDK_BACKEND,
        "action": ACTION_CONFIRM_GDK_RECOVERY,
        "confirmed": confirmed,
        "sampleCount": len(samples),
        "sampleIntervalSeconds": policy.sample_interval_seconds,
        "policy": policy.to_payload(),
        "collectedAt": utc_now_iso(),
        "samples": [compact_recovery_sample(sample) for sample in samples],
    }
    if gdk_recovery is not None:
        result["gdk_recovery"] = dict(gdk_recovery)
    if not confirmed:
        result["errorCode"] = GDK_RECOVERY_NOT_CONFIRMED_CODE
        result["errorMsg"] = "GDK 恢复确认未通过，控制命令仍保持阻断"
    return result


def compact_recovery_sample(sample: Mapping[str, Any]) -> dict[str, object]:
    """保留多帧确认需要的摘要，避免响应过大。"""

    return {
        "available": sample.get("available") is True,
        "busy": sample.get("busy") is True,
        "collectedAt": sample.get("collectedAt"),
        "motionStatus": sample.get("motionStatus"),
        "wholeBodyStatus": sample.get("wholeBodyStatus"),
        "groups": {
            group_name: {
                "positions": group.get("positions"),
                "velocities": group.get("velocities"),
            }
            for group_name, group in iter_group_mappings(sample)
        },
    }


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
        "framePoses": build_arm_frame_pose_snapshots(motion_status),
        "nonzeroErrorJoints": build_nonzero_error_joints(state_items),
        "motionStatus": {
            "mode": to_jsonable(getattr(motion_status, "mode", None)),
            "controlMode": to_jsonable(getattr(motion_status, "control_mode", None)),
            "errorCode": to_jsonable(getattr(motion_status, "error_code", None)),
            "errorMsg": str(getattr(motion_status, "error_msg", "") or ""),
        },
        "wholeBodyStatus": to_jsonable(whole_body_status),
    }


def build_arm_frame_pose_snapshots(motion_status: Any) -> dict[str, dict[str, object]]:
    """从 motion_control_status 读取左右臂末端绝对位姿。

    ABS_POSE 使用的是末端 frame pose，不是关节角；这里只读 arm_l/r_end_link，
    不读取或切换 tool0/TCP frame，避免把坐标系策略混入“获取当前值”按钮。
    """

    frame_poses = extract_arm_frame_poses(motion_status)
    snapshots: dict[str, dict[str, object]] = {}
    for body_part, frame_name in ARM_FRAME_NAMES.items():
        pose = frame_poses.get(frame_name)
        if pose is None:
            continue
        position = pose.get("position")
        orientation = pose.get("orientation")
        if not isinstance(position, Sequence) or isinstance(position, str | bytes | bytearray):
            continue
        if not isinstance(orientation, Sequence) or isinstance(
            orientation,
            str | bytes | bytearray,
        ):
            continue
        position_values = [float(value) for value in position]
        orientation_values = [float(value) for value in orientation]
        if len(position_values) != 3 or len(orientation_values) != 4:
            continue
        snapshots[body_part] = {
            "bodyPart": body_part,
            "frameName": frame_name,
            "position": position_values,
            "orientation": orientation_values,
            "values": [*position_values, *orientation_values],
        }
    return snapshots


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
                "velocity": read_joint_velocity(state),
                "limit": read_limit(limits, joint_name),
                "errorCode": to_jsonable(state.get("error_code")),
            }
        )

    return {
        "bodyPart": body_part,
        "jointNames": list(joint_names),
        "positions": [joint["position"] for joint in joints],
        "velocities": [joint["velocity"] for joint in joints],
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


def read_joint_velocity(state: Mapping[str, Any]) -> float | None:
    value = state.get("motor_velocity", state.get("velocity"))
    if value is None:
        return None
    return float(value)


def iter_group_mappings(sample: Mapping[str, Any]) -> list[tuple[str, Mapping[str, Any]]]:
    groups = sample.get("groups")
    if not isinstance(groups, Mapping):
        return []
    result: list[tuple[str, Mapping[str, Any]]] = []
    for group_name in ("left_arm", "right_arm", "waist"):
        group = groups.get(group_name)
        if isinstance(group, Mapping):
            result.append((group_name, group))
    return result


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
