"""GDK 控制探针 — 手动控制操作和关节状态工具函数。

提供 hold_current、nudge_left_j7、nudge_right_j7 等现场测试命令，
以及关节快照采集、限位校验、位置差计算等运动规划的共享工具。
"""

from __future__ import annotations

import importlib
import inspect
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
ARM_FRAME_NAMES = {
    "left_arm": "arm_l_end_link",
    "right_arm": "arm_r_end_link",
}
CONTROL_GROUP_LEFT_ARM = 0
CONTROL_GROUP_RIGHT_ARM = 1
CONTROL_GROUP_DUAL_ARM = 2
DEFAULT_VELOCITY = 0.02
NUDGE_J7_DELTA_RAD = 0.005
ABS_POSE_LIFE_TIME_SECONDS = 0.5
ABS_POSE_CONTROL_METHOD = "end_effector_pose_control"
ABS_POSE_NUDGE_DELTA_M = 0.001
ABS_POSE_NUDGE_AXIS = "z"

ACTION_HOLD_CURRENT = "hold_current"
ACTION_NUDGE_LEFT_J7 = "nudge_left_j7_0p005"
ACTION_NUDGE_RIGHT_J7 = "nudge_right_j7_0p005"
ACTION_ABS_POSE_DRY_RUN = "abs_pose_dry_run"
ACTION_ABS_POSE_HOLD_CURRENT_LEFT = "abs_pose_hold_current_left"
ACTION_ABS_POSE_HOLD_CURRENT_RIGHT = "abs_pose_hold_current_right"
ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001 = "abs_pose_nudge_left_z_0p001"
ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_1S = "abs_pose_nudge_left_z_0p001_life_1s"
ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S = "abs_pose_nudge_left_z_0p001_life_2s"

ACTION_CONFIRMATION_TOKENS = {
    ACTION_HOLD_CURRENT: "HOLD_CURRENT_DUAL_ARM",
    ACTION_NUDGE_LEFT_J7: "NUDGE_LEFT_J7_0P005",
    ACTION_NUDGE_RIGHT_J7: "NUDGE_RIGHT_J7_0P005",
    ACTION_ABS_POSE_DRY_RUN: "ABS_POSE_DRY_RUN",
    ACTION_ABS_POSE_HOLD_CURRENT_LEFT: "ABS_POSE_HOLD_CURRENT_LEFT",
    ACTION_ABS_POSE_HOLD_CURRENT_RIGHT: "ABS_POSE_HOLD_CURRENT_RIGHT",
    ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001: "ABS_POSE_NUDGE_LEFT_Z_0P001",
    ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_1S: "ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_1S",
    ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S: "ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S",
}

ALLOWED_ACTIONS = tuple(ACTION_CONFIRMATION_TOKENS)


@dataclass(frozen=True)
class JointSnapshot:
    positions: list[float]
    motion_status: object
    whole_body_status: Mapping[str, Any]


@dataclass(frozen=True)
class AbsPoseTarget:
    end_pose: object
    group_value: object


@dataclass(frozen=True)
class AbsPoseProbeState:
    motion_status: object
    whole_body_status: object
    positions: list[float]
    frame_poses: dict[str, dict[str, object]]
    diffs: list[float]
    motion_error_code: object
    motion_error_msg: str
    whole_body_error: str | None

    @property
    def motion_status_ok(self) -> bool:
        return is_zero_error(self.motion_error_code)

    @property
    def whole_body_status_ok(self) -> bool:
        return self.whole_body_error is None

    @property
    def ok(self) -> bool:
        return self.motion_status_ok and self.whole_body_status_ok


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
            agibot_gdk=agibot_gdk,
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
    agibot_gdk: Any,
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

    if action == ACTION_ABS_POSE_DRY_RUN:
        return execute_abs_pose_dry_run(
            action=action,
            agibot_gdk=agibot_gdk,
            robot=robot,
            origin=origin,
        )

    if action == ACTION_ABS_POSE_HOLD_CURRENT_LEFT:
        return execute_abs_pose_hold_current(
            action=action,
            arm="left_arm",
            agibot_gdk=agibot_gdk,
            robot=robot,
            origin=origin,
            sleep=sleep,
            settle_seconds=settle_seconds,
        )

    if action == ACTION_ABS_POSE_HOLD_CURRENT_RIGHT:
        return execute_abs_pose_hold_current(
            action=action,
            arm="right_arm",
            agibot_gdk=agibot_gdk,
            robot=robot,
            origin=origin,
            sleep=sleep,
            settle_seconds=settle_seconds,
        )

    if action == ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001:
        return execute_abs_pose_nudge_and_return(
            action=action,
            arm="left_arm",
            axis=ABS_POSE_NUDGE_AXIS,
            delta_m=ABS_POSE_NUDGE_DELTA_M,
            life_time_seconds=ABS_POSE_LIFE_TIME_SECONDS,
            agibot_gdk=agibot_gdk,
            robot=robot,
            origin=origin,
            sleep=sleep,
            settle_seconds=settle_seconds,
        )

    if action == ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_1S:
        return execute_abs_pose_nudge_and_return(
            action=action,
            arm="left_arm",
            axis=ABS_POSE_NUDGE_AXIS,
            delta_m=ABS_POSE_NUDGE_DELTA_M,
            life_time_seconds=1.0,
            agibot_gdk=agibot_gdk,
            robot=robot,
            origin=origin,
            sleep=sleep,
            settle_seconds=settle_seconds,
        )

    if action == ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S:
        return execute_abs_pose_nudge_and_return(
            action=action,
            arm="left_arm",
            axis=ABS_POSE_NUDGE_AXIS,
            delta_m=ABS_POSE_NUDGE_DELTA_M,
            life_time_seconds=2.0,
            agibot_gdk=agibot_gdk,
            robot=robot,
            origin=origin,
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


def execute_abs_pose_dry_run(
    *,
    action: str,
    agibot_gdk: Any,
    robot: Any,
    origin: JointSnapshot,
) -> dict[str, object]:
    frame_poses = extract_arm_frame_poses(origin.motion_status)
    dry_run_targets = {
        arm: build_abs_pose_target_summary(
            agibot_gdk=agibot_gdk,
            frame_poses=frame_poses,
            arm=arm,
        )
        for arm in ("left_arm", "right_arm")
    }
    return {
        "available": True,
        "executed": False,
        "probe_succeeded": True,
        "motion_attempted": False,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "control_method": ABS_POSE_CONTROL_METHOD,
        "control_method_available": callable(getattr(robot, ABS_POSE_CONTROL_METHOD, None)),
        "candidate_methods": discover_abs_pose_methods(robot),
        "frame_names": motion_frame_names(origin.motion_status),
        "current_frame_poses": serializable_frame_poses(frame_poses),
        "dry_run_targets": dry_run_targets,
        "safety_note": (
            "只构造 EndEffectorPose，不调用位姿控制；正式运动前仍需单独执行 "
            "hold-current 探针并以后置 motion_status 为准。"
        ),
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


def execute_abs_pose_hold_current(
    *,
    action: str,
    arm: str,
    agibot_gdk: Any,
    robot: Any,
    origin: JointSnapshot,
    sleep: Callable[[float], None],
    settle_seconds: float,
) -> dict[str, object]:
    frame_poses = extract_arm_frame_poses(origin.motion_status)
    target = build_abs_pose_target(
        agibot_gdk=agibot_gdk,
        frame_poses=frame_poses,
        arm=arm,
        life_time_seconds=ABS_POSE_LIFE_TIME_SECONDS,
    )
    control_method = getattr(robot, ABS_POSE_CONTROL_METHOD, None)
    if not callable(control_method):
        raise RuntimeError(f"robot.{ABS_POSE_CONTROL_METHOD} is unavailable")

    # ABS_POSE 的最小真机探针只保持当前末端位姿，不叠加偏移；GDK 返回值
    # 不能单独代表安全成功，必须同步记录后置 motion_status/whole_body_status。
    control_return = control_method(target.end_pose)
    sleep(settle_seconds)
    after_motion_status = robot.get_motion_control_status()
    after_whole_body_status = robot.get_whole_body_status()
    after_positions = read_dual_arm_positions_without_status_validation(robot)
    diffs = position_diffs(after_positions, origin.positions)
    motion_error_code = getattr(after_motion_status, "error_code", 0)
    motion_error_msg = getattr(after_motion_status, "error_msg", "")
    whole_body_error = validate_whole_body_status_error(after_whole_body_status)
    return {
        "available": True,
        "executed": True,
        "probe_succeeded": True,
        "motion_attempted": True,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "control_method": ABS_POSE_CONTROL_METHOD,
        "arm": arm,
        "arm_frame_name": ARM_FRAME_NAMES[arm],
        "end_effector_group": to_jsonable(target.group_value),
        "life_time_seconds": ABS_POSE_LIFE_TIME_SECONDS,
        "control_return": to_jsonable(control_return),
        "origin_positions": origin.positions,
        "after_positions": after_positions,
        "diffs": diffs,
        "max_abs_diff": max((abs(diff) for diff in diffs), default=0.0),
        "motion_status_ok_after": is_zero_error(motion_error_code),
        "motion_error_code_after": to_jsonable(motion_error_code),
        "motion_error_msg_after": str(motion_error_msg),
        "whole_body_status_ok_after": whole_body_error is None,
        "whole_body_status_error_after": whole_body_error,
        "current_frame_poses": serializable_frame_poses(frame_poses),
        "after_frame_poses": serializable_frame_poses(extract_arm_frame_poses(after_motion_status)),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": ACTION_CONFIRMATION_TOKENS[action],
        },
        "raw": {
            "motion_status_before": to_jsonable(origin.motion_status),
            "whole_body_status_before": to_jsonable(origin.whole_body_status),
            "motion_status_after": to_jsonable(after_motion_status),
            "whole_body_status_after": to_jsonable(after_whole_body_status),
        },
    }


def execute_abs_pose_nudge_and_return(
    *,
    action: str,
    arm: str,
    axis: str,
    delta_m: float,
    life_time_seconds: float,
    agibot_gdk: Any,
    robot: Any,
    origin: JointSnapshot,
    sleep: Callable[[float], None],
    settle_seconds: float,
) -> dict[str, object]:
    frame_poses = extract_arm_frame_poses(origin.motion_status)
    origin_target = build_abs_pose_target(
        agibot_gdk=agibot_gdk,
        frame_poses=frame_poses,
        arm=arm,
        life_time_seconds=life_time_seconds,
    )
    nudge_target = build_abs_pose_target(
        agibot_gdk=agibot_gdk,
        frame_poses=frame_poses,
        arm=arm,
        life_time_seconds=life_time_seconds,
    )
    offset_abs_pose_target(nudge_target, arm=arm, axis=axis, delta_m=delta_m)

    control_method = getattr(robot, ABS_POSE_CONTROL_METHOD, None)
    if not callable(control_method):
        raise RuntimeError(f"robot.{ABS_POSE_CONTROL_METHOD} is unavailable")

    # 小偏移探针是正式 ABS_POSE 前的最后一道真机门禁：先 1mm 单轴偏移，
    # 中间状态正常才尝试回原，避免在控制器报错后继续叠加命令。
    nudge_return = control_method(nudge_target.end_pose)
    sleep(settle_seconds)
    mid = capture_abs_pose_probe_state(robot, origin_positions=origin.positions)

    return_attempted = mid.ok
    return_return: object | None = None
    after: AbsPoseProbeState | None = None
    return_skipped_reason: str | None = None
    if return_attempted:
        return_return = control_method(origin_target.end_pose)
        sleep(settle_seconds)
        after = capture_abs_pose_probe_state(robot, origin_positions=origin.positions)
    else:
        return_skipped_reason = (
            "nudge 后 motion/whole-body 状态异常，已跳过回原命令，需现场人工判断。"
        )

    frame_name = ARM_FRAME_NAMES[arm]
    probe_succeeded = mid.ok and after is not None and after.ok
    return {
        "available": True,
        "executed": True,
        "probe_succeeded": probe_succeeded,
        "motion_attempted": True,
        "backend": GDK_BACKEND,
        "action": action,
        "collected_at": utc_now_iso(),
        "control_method": ABS_POSE_CONTROL_METHOD,
        "arm": arm,
        "arm_frame_name": frame_name,
        "end_effector_group": to_jsonable(nudge_target.group_value),
        "life_time_seconds": life_time_seconds,
        "nudge_axis": axis,
        "nudge_delta_m": delta_m,
        "return_attempted": return_attempted,
        "return_skipped_reason": return_skipped_reason,
        "control_returns": {
            "nudge": to_jsonable(nudge_return),
            "return_to_origin": to_jsonable(return_return),
        },
        "origin_positions": origin.positions,
        "current_frame_poses": serializable_frame_poses(frame_poses),
        "target_frame_delta_m": {axis: delta_m},
        "mid": abs_pose_state_payload(mid),
        "mid_arm_frame_delta_m": frame_position_delta_m(
            before=frame_poses,
            after=mid.frame_poses,
            frame_name=frame_name,
        ),
        "after": abs_pose_state_payload(after) if after is not None else None,
        "after_arm_frame_delta_m": (
            frame_position_delta_m(
                before=frame_poses,
                after=after.frame_poses,
                frame_name=frame_name,
            )
            if after is not None
            else None
        ),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": ACTION_CONFIRMATION_TOKENS[action],
        },
        "raw": {
            "motion_status_before": to_jsonable(origin.motion_status),
            "whole_body_status_before": to_jsonable(origin.whole_body_status),
            "motion_status_mid": to_jsonable(mid.motion_status),
            "whole_body_status_mid": to_jsonable(mid.whole_body_status),
            "motion_status_after": to_jsonable(after.motion_status) if after else None,
            "whole_body_status_after": to_jsonable(after.whole_body_status)
            if after
            else None,
        },
    }


def build_abs_pose_target_summary(
    *,
    agibot_gdk: Any,
    frame_poses: Mapping[str, Mapping[str, object]],
    arm: str,
) -> dict[str, object]:
    target = build_abs_pose_target(
        agibot_gdk=agibot_gdk,
        frame_poses=frame_poses,
        arm=arm,
        life_time_seconds=ABS_POSE_LIFE_TIME_SECONDS,
    )
    return {
        "arm": arm,
        "arm_frame_name": ARM_FRAME_NAMES[arm],
        "group": to_jsonable(target.group_value),
        "life_time_seconds": ABS_POSE_LIFE_TIME_SECONDS,
        "left_end_effector_pose": pose_to_dict(
            getattr(target.end_pose, "left_end_effector_pose", None)
        ),
        "right_end_effector_pose": pose_to_dict(
            getattr(target.end_pose, "right_end_effector_pose", None)
        ),
    }


def build_abs_pose_target(
    *,
    agibot_gdk: Any,
    frame_poses: Mapping[str, Mapping[str, object]],
    arm: str,
    life_time_seconds: float,
) -> AbsPoseTarget:
    left_frame = ARM_FRAME_NAMES["left_arm"]
    right_frame = ARM_FRAME_NAMES["right_arm"]
    left_pose = frame_poses.get(left_frame, {}).get("raw_pose")
    right_pose = frame_poses.get(right_frame, {}).get("raw_pose")
    if left_pose is None:
        raise RuntimeError(f"No {left_frame} pose in motion_control_status")
    if right_pose is None:
        raise RuntimeError(f"No {right_frame} pose in motion_control_status")

    end_pose_cls = getattr(agibot_gdk, "EndEffectorPose", None)
    if not callable(end_pose_cls):
        raise RuntimeError("agibot_gdk.EndEffectorPose is unavailable")

    end_pose = end_pose_cls()
    group_value = get_abs_pose_group_value(agibot_gdk, arm)
    end_pose.group = group_value
    end_pose.life_time = float(life_time_seconds)
    end_pose.left_end_effector_pose = copy_gdk_pose(agibot_gdk, left_pose)
    end_pose.right_end_effector_pose = copy_gdk_pose(agibot_gdk, right_pose)
    return AbsPoseTarget(end_pose=end_pose, group_value=group_value)


def offset_abs_pose_target(
    target: AbsPoseTarget,
    *,
    arm: str,
    axis: str,
    delta_m: float,
) -> None:
    if axis not in {"x", "y", "z"}:
        raise ValueError(f"unsupported ABS_POSE nudge axis: {axis}")
    pose_attr = (
        "left_end_effector_pose"
        if arm == "left_arm"
        else "right_end_effector_pose"
    )
    pose = getattr(target.end_pose, pose_attr)
    position = pose.position
    setattr(position, axis, float(getattr(position, axis)) + float(delta_m))


def get_abs_pose_group_value(agibot_gdk: Any, arm: str) -> object:
    if arm == "left_arm":
        name = "kLeftArm"
    elif arm == "right_arm":
        name = "kRightArm"
    else:
        raise ValueError(f"unsupported ABS_POSE arm: {arm}")

    value = getattr(agibot_gdk, name, None)
    if value is not None:
        return value

    enum_cls = getattr(agibot_gdk, "EndEffectorControlGroup", None)
    if enum_cls is not None:
        value = getattr(enum_cls, name, None)
        if value is not None:
            return value
    raise RuntimeError(f"agibot_gdk.{name} is unavailable")


def capture_abs_pose_probe_state(
    robot: Any,
    *,
    origin_positions: Sequence[float],
) -> AbsPoseProbeState:
    motion_status = robot.get_motion_control_status()
    whole_body_status = robot.get_whole_body_status()
    positions = read_dual_arm_positions_without_status_validation(robot)
    diffs = position_diffs(positions, origin_positions)
    motion_error_code = getattr(motion_status, "error_code", 0)
    motion_error_msg = str(getattr(motion_status, "error_msg", ""))
    whole_body_error = validate_whole_body_status_error(whole_body_status)
    return AbsPoseProbeState(
        motion_status=motion_status,
        whole_body_status=whole_body_status,
        positions=positions,
        frame_poses=extract_arm_frame_poses(motion_status),
        diffs=diffs,
        motion_error_code=motion_error_code,
        motion_error_msg=motion_error_msg,
        whole_body_error=whole_body_error,
    )


def abs_pose_state_payload(state: AbsPoseProbeState | None) -> dict[str, object] | None:
    if state is None:
        return None
    return {
        "positions": state.positions,
        "diffs": state.diffs,
        "max_abs_diff": max((abs(diff) for diff in state.diffs), default=0.0),
        "motion_status_ok": state.motion_status_ok,
        "motion_error_code": to_jsonable(state.motion_error_code),
        "motion_error_msg": state.motion_error_msg,
        "whole_body_status_ok": state.whole_body_status_ok,
        "whole_body_status_error": state.whole_body_error,
        "frame_poses": serializable_frame_poses(state.frame_poses),
    }


def frame_position_delta_m(
    *,
    before: Mapping[str, Mapping[str, object]],
    after: Mapping[str, Mapping[str, object]],
    frame_name: str,
) -> dict[str, object] | None:
    before_pose = before.get(frame_name)
    after_pose = after.get(frame_name)
    if before_pose is None or after_pose is None:
        return None
    before_position = before_pose.get("position")
    after_position = after_pose.get("position")
    if not isinstance(before_position, list) or not isinstance(after_position, list):
        return None
    if len(before_position) != 3 or len(after_position) != 3:
        return None
    delta = [
        float(after_value) - float(before_value)
        for before_value, after_value in zip(before_position, after_position, strict=True)
    ]
    return {
        "x": delta[0],
        "y": delta[1],
        "z": delta[2],
        "norm": sum(value * value for value in delta) ** 0.5,
    }


def copy_gdk_pose(agibot_gdk: Any, pose: Any) -> Any:
    pose_cls = getattr(agibot_gdk, "Pose", None)
    if not callable(pose_cls):
        raise RuntimeError("agibot_gdk.Pose is unavailable")
    copied = pose_cls()
    copied.position.x = float(pose.position.x)
    copied.position.y = float(pose.position.y)
    copied.position.z = float(pose.position.z)
    copied.orientation.x = float(pose.orientation.x)
    copied.orientation.y = float(pose.orientation.y)
    copied.orientation.z = float(pose.orientation.z)
    copied.orientation.w = float(pose.orientation.w)
    return copied


def extract_arm_frame_poses(motion_status: Any) -> dict[str, dict[str, object]]:
    names = motion_frame_names(motion_status)
    poses = list(getattr(motion_status, "frame_poses", []))
    result: dict[str, dict[str, object]] = {}
    for frame_name in ARM_FRAME_NAMES.values():
        try:
            index = names.index(frame_name)
        except ValueError:
            continue
        if index >= len(poses):
            continue
        pose = poses[index]
        pose_dict = pose_to_dict(pose)
        if pose_dict is None:
            continue
        result[frame_name] = {**pose_dict, "raw_pose": pose}
    return result


def serializable_frame_poses(
    frame_poses: Mapping[str, Mapping[str, object]],
) -> dict[str, dict[str, object]]:
    return {
        frame_name: {
            key: value
            for key, value in pose_data.items()
            if key != "raw_pose"
        }
        for frame_name, pose_data in frame_poses.items()
    }


def motion_frame_names(motion_status: Any) -> list[str]:
    return [str(value) for value in getattr(motion_status, "frame_names", [])]


def pose_to_dict(pose: Any) -> dict[str, list[float]] | None:
    if pose is None:
        return None
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        return None
    return {
        "position": [
            float(position.x),
            float(position.y),
            float(position.z),
        ],
        "orientation": [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
    }


def discover_abs_pose_methods(robot: Any) -> list[dict[str, object]]:
    explicit_names = {
        "end_effector_pose_control",
        "motion_plan_request",
        "motion_plan_request_full",
        "multi_motion_plan_request",
        "force_position_control",
    }
    discovered = set(explicit_names)
    for name in dir(robot):
        lower = name.lower()
        if name.startswith("_"):
            continue
        if (
            any(keyword in lower for keyword in ("pose", "cartesian"))
            and any(keyword in lower for keyword in ("move", "control", "plan", "request"))
        ):
            discovered.add(name)

    methods: list[dict[str, object]] = []
    for name in sorted(discovered):
        try:
            value = getattr(robot, name)
        except Exception as error:
            methods.append(
                {
                    "name": name,
                    "available": False,
                    "error_type": type(error).__name__,
                    "error_msg": str(error),
                }
            )
            continue
        if not callable(value):
            continue
        methods.append(
            {
                "name": name,
                "available": True,
                "signature": callable_signature(value),
                "doc": compact_doc(getattr(value, "__doc__", None)),
            }
        )
    return methods


def callable_signature(value: Any) -> str:
    try:
        return str(inspect.signature(value))
    except Exception as error:
        return f"unavailable: {type(error).__name__}: {error}"


def compact_doc(doc: Any) -> str | None:
    if not isinstance(doc, str):
        return None
    return " ".join(doc.strip().split())[:500]


def read_dual_arm_positions_without_status_validation(robot: Any) -> list[float]:
    joint_states = robot.get_joint_states()
    if not isinstance(joint_states, Mapping):
        raise TypeError("robot.get_joint_states() did not return a mapping")
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
        positions.append(read_joint_position(state))
    return positions


def validate_whole_body_status_error(whole_body_status: Any) -> str | None:
    try:
        validate_whole_body_status(whole_body_status)
    except Exception as error:
        return str(error)
    return None


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
