from __future__ import annotations

from typing import Any

import pytest

from gsa_taskflow_executor.gdk.control_probe import (
    ARM_FRAME_NAMES,
    CONTROL_GROUP_DUAL_ARM,
    CONTROL_GROUP_LEFT_ARM,
    CONTROL_GROUP_RIGHT_ARM,
    DUAL_ARM_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
)
from gsa_taskflow_executor.gdk.motion_runtime import (
    ACTION_TASKFLOW_ABS_POSE,
    TASKFLOW_ABS_JOINT_CONFIRMATION,
    TASKFLOW_ABS_POSE_LIFE_TIME_SECONDS,
    TASKFLOW_ABS_POSE_MAX_TRANSLATION_ENV,
    WAIST_JOINTS,
    run_gdk_motion_plan_abs_joint,
)
from gsa_taskflow_executor.gdk.recovery import current_gdk_recovery_requirement
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.gdk.subprocess_runtime import GDK_OPERATION_TIMEOUT_CODE
from gsa_taskflow_executor.taskflow.parser import MotionPlanParams, MotionPlanTarget


def disable_abs_pose_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "gsa_taskflow_executor.gdk.motion_runtime.time.sleep",
        lambda _seconds: None,
    )


class FakeMotionStatus:
    def __init__(
        self,
        *,
        mode: object = 1,
        control_mode: object = 1,
        error_code: object = 0,
        error_msg: str = "",
        frame_names: list[str] | None = None,
        frame_poses: list[object] | None = None,
    ) -> None:
        self.mode = mode
        self.control_mode = control_mode
        self.error_code = error_code
        self.error_msg = error_msg
        self.frame_names = list(frame_names or [])
        self.frame_poses = list(frame_poses or [])


class FakeVector3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.z = z


class FakeQuaternion:
    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        w: float = 1.0,
    ) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class FakePose:
    def __init__(self) -> None:
        self.position = FakeVector3()
        self.orientation = FakeQuaternion()


class FakeEndEffectorPose:
    def __init__(self) -> None:
        self.group: object | None = None
        self.life_time = 0.0
        self.left_end_effector_pose = FakePose()
        self.right_end_effector_pose = FakePose()


def make_pose(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> FakePose:
    pose = FakePose()
    pose.position = FakeVector3(x, y, z)
    pose.orientation = FakeQuaternion(qx, qy, qz, qw)
    return pose


def clone_pose(pose: FakePose) -> FakePose:
    return make_pose(
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


class FakeRobot:
    def __init__(self) -> None:
        self.arm_positions = [index * 0.01 for index in range(len(DUAL_ARM_JOINTS))]
        self.waist_positions = [index * 0.001 for index in range(len(WAIST_JOINTS))]
        self.left_pose = make_pose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
        self.right_pose = make_pose(0.4, -0.2, 0.8, 0.1, 0.2, 0.3, 0.9)
        self.motion_status = FakeMotionStatus()
        self.arm_move_calls: list[tuple[list[float], list[float], int]] = []
        self.waist_move_calls: list[tuple[list[float], list[float]]] = []
        self.pose_control_calls: list[FakeEndEffectorPose] = []
        self.move_call_order: list[str] = []

    def get_joint_states(self) -> dict[str, object]:
        states = [
            {
                "name": name,
                "position": self.arm_positions[index],
                "motor_position": self.arm_positions[index],
                "error_code": 0,
            }
            for index, name in enumerate(DUAL_ARM_JOINTS)
        ]
        states.extend(
            {
                "name": name,
                "position": self.waist_positions[index],
                "motor_position": self.waist_positions[index],
                "error_code": 0,
            }
            for index, name in enumerate(WAIST_JOINTS)
        )
        return {
            "nums": len(states),
            "states": states,
        }

    def get_joint_limits(self) -> dict[str, dict[str, float]]:
        return {
            name: {"min": -10.0, "max": 10.0}
            for name in [*DUAL_ARM_JOINTS, *WAIST_JOINTS]
        }

    def get_motion_control_status(self) -> FakeMotionStatus:
        self.motion_status.frame_names = [
            ARM_FRAME_NAMES["left_arm"],
            ARM_FRAME_NAMES["right_arm"],
        ]
        self.motion_status.frame_poses = [self.left_pose, self.right_pose]
        return self.motion_status

    def get_whole_body_status(self) -> dict[str, object]:
        return {
            "left_arm_error": 0,
            "right_arm_error": 0,
            "left_arm_estop": False,
            "right_arm_estop": False,
            "left_end_error": 0,
            "right_end_error": 0,
            "waist_error": 0,
            "lift_error": 0,
            "neck_error": 0,
            "chassis_error": 0,
        }

    def move_arm_joint(
        self,
        positions: list[float],
        velocities: list[float],
        control_group: int,
    ) -> int:
        self.move_call_order.append("arm")
        self.arm_move_calls.append((list(positions), list(velocities), control_group))
        if len(velocities) != len(positions):
            raise RuntimeError(
                f"expected velocities length {len(positions)}, got {len(velocities)}"
            )
        if control_group == CONTROL_GROUP_LEFT_ARM:
            if len(positions) != len(LEFT_ARM_JOINTS):
                raise RuntimeError(f"expected 7 left arm positions, got {len(positions)}")
            self.arm_positions[: len(LEFT_ARM_JOINTS)] = list(positions)
        elif control_group == CONTROL_GROUP_RIGHT_ARM:
            if len(positions) != len(RIGHT_ARM_JOINTS):
                raise RuntimeError(f"expected 7 right arm positions, got {len(positions)}")
            self.arm_positions[len(LEFT_ARM_JOINTS) :] = list(positions)
        elif control_group == CONTROL_GROUP_DUAL_ARM:
            if len(positions) != len(DUAL_ARM_JOINTS):
                raise RuntimeError(f"expected 14 dual arm positions, got {len(positions)}")
            self.arm_positions = list(positions)
        else:
            raise RuntimeError(f"unsupported control_group: {control_group}")
        return 0

    def move_waist_joint(self, positions: list[float], velocities: list[float]) -> int:
        self.move_call_order.append("waist")
        self.waist_move_calls.append((list(positions), list(velocities)))
        self.waist_positions = list(positions)
        return 0

    def end_effector_pose_control(self, end_pose: FakeEndEffectorPose) -> int:
        self.move_call_order.append("pose")
        self.pose_control_calls.append(end_pose)
        self.left_pose = clone_pose(end_pose.left_end_effector_pose)
        self.right_pose = clone_pose(end_pose.right_end_effector_pose)
        return 0


class FakeAgibotGdk:
    robot = FakeRobot()
    init_called = 0
    release_called = 0
    kLeftArm = 4
    kRightArm = 8
    Pose = FakePose
    EndEffectorPose = FakeEndEffectorPose

    class GDKRes:
        kSuccess = 0

    @classmethod
    def reset(cls) -> None:
        cls.robot = FakeRobot()
        cls.init_called = 0
        cls.release_called = 0

    @classmethod
    def gdk_init(cls) -> int:
        cls.init_called += 1
        return cls.GDKRes.kSuccess

    @classmethod
    def gdk_release(cls) -> None:
        cls.release_called += 1

    @classmethod
    def Robot(cls) -> FakeRobot:  # noqa: N802 - mirrors agibot_gdk API
        return cls.robot


def test_gdk_motion_runtime_refuses_before_importing_without_safety_gate() -> None:
    def forbidden_import(_name: str) -> Any:
        raise AssertionError("GDK must not be imported without safety confirmation")

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=[0.1] * 7,
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={},
        import_module=forbidden_import,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "safety_gate"
    assert result["error_msg"] == "ENABLE_GDK_CONTROL must be 1"


def test_gdk_motion_runtime_refuses_speed_outside_safe_range() -> None:
    def forbidden_import(_name: str) -> Any:
        raise AssertionError("GDK must not be imported with invalid speed")

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=[0.1] * 7,
                ),
            ),
            speed=0.5,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=forbidden_import,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "validate_params"
    assert result["error_msg"] == "speed must be between 0.001 and 0.1"


def test_gdk_motion_runtime_executes_right_arm_and_waist_abs_joint() -> None:
    FakeAgibotGdk.reset()
    right_target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    waist_target = [0.01, 0.02, 0.03, 0.04, 0.05]
    expected_velocity = 0.05

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=right_target,
                ),
                MotionPlanTarget(
                    body_part="waist",
                    control_type="ABS_JOINT",
                    action_data=waist_target,
                ),
            ),
            speed=expected_velocity,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["speed"] == expected_velocity
    assert result["requested_speed"] == expected_velocity
    assert result["requested_speed_unit"] == "gdk_velocity"
    assert result["speed_mapping_applied"] is True
    assert result["effective_gdk_velocity"] == expected_velocity
    assert result["gdk_velocity"] == expected_velocity
    assert result["velocity_source"] == "taskflow_speed"
    assert FakeAgibotGdk.init_called == 1
    assert FakeAgibotGdk.release_called == 0
    assert result["gdk_release"] == {
        "called": False,
        "success": True,
        "reason": "process_managed_session",
    }
    assert result["gdk_session"]["policy"] == "process_managed_session"
    assert result["gdk_session"]["purpose"] == "taskflow_abs_joint"
    assert FakeAgibotGdk.robot.arm_move_calls == [
        (
            right_target,
            [expected_velocity] * len(RIGHT_ARM_JOINTS),
            CONTROL_GROUP_RIGHT_ARM,
        )
    ]
    assert FakeAgibotGdk.robot.waist_move_calls == [
        (waist_target, [expected_velocity] * len(WAIST_JOINTS))
    ]

    groups = result["groups"]
    assert isinstance(groups, list)
    assert groups[0]["method"] == "move_arm_joint"
    assert groups[0]["requested_body_parts"] == ["right_arm"]
    assert groups[0]["control_group"] == CONTROL_GROUP_RIGHT_ARM
    assert groups[0]["interface_mode"] == "single_right_arm_7d"
    assert groups[0]["joint_order"] == list(RIGHT_ARM_JOINTS)
    assert groups[0]["positions_len"] == len(RIGHT_ARM_JOINTS)
    assert groups[0]["velocities_len"] == len(RIGHT_ARM_JOINTS)
    assert groups[0]["target_positions"] == right_target
    assert groups[0]["effective_gdk_velocity"] == expected_velocity
    assert groups[0]["velocity_source"] == "taskflow_speed"
    assert groups[1]["method"] == "move_waist_joint"
    assert groups[1]["effective_gdk_velocity"] == expected_velocity
    assert groups[1]["velocity_source"] == "taskflow_speed"
    assert FakeAgibotGdk.robot.move_call_order == ["arm", "waist"]


def test_gdk_motion_runtime_honors_waist_before_arm_order() -> None:
    FakeAgibotGdk.reset()
    right_target = [0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27]
    waist_target = [0.01, 0.02, 0.03, 0.04, 0.05]
    expected_velocity = 0.03

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="waist",
                    control_type="ABS_JOINT",
                    action_data=waist_target,
                ),
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=right_target,
                ),
            ),
            speed=expected_velocity,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert FakeAgibotGdk.robot.move_call_order == ["waist", "arm"]
    groups = result["groups"]
    assert isinstance(groups, list)
    assert groups[0]["method"] == "move_waist_joint"
    assert groups[0]["target_positions"] == waist_target
    assert groups[1]["method"] == "move_arm_joint"
    assert groups[1]["requested_body_parts"] == ["right_arm"]
    assert groups[1]["target_positions"] == right_target


def test_gdk_motion_runtime_executes_left_arm_abs_joint_with_single_arm_group() -> None:
    FakeAgibotGdk.reset()
    left_target = [0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17]
    expected_velocity = 0.04
    origin_right = list(FakeAgibotGdk.robot.arm_positions[len(LEFT_ARM_JOINTS) :])

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="left_arm",
                    control_type="ABS_JOINT",
                    action_data=left_target,
                ),
            ),
            speed=expected_velocity,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert FakeAgibotGdk.robot.arm_move_calls == [
        (
            left_target,
            [expected_velocity] * len(LEFT_ARM_JOINTS),
            CONTROL_GROUP_LEFT_ARM,
        )
    ]
    assert FakeAgibotGdk.robot.arm_positions[: len(LEFT_ARM_JOINTS)] == left_target
    assert FakeAgibotGdk.robot.arm_positions[len(LEFT_ARM_JOINTS) :] == origin_right

    groups = result["groups"]
    assert isinstance(groups, list)
    assert groups[0]["method"] == "move_arm_joint"
    assert groups[0]["requested_body_parts"] == ["left_arm"]
    assert groups[0]["control_group"] == CONTROL_GROUP_LEFT_ARM
    assert groups[0]["interface_mode"] == "single_left_arm_7d"
    assert groups[0]["joint_order"] == list(LEFT_ARM_JOINTS)
    assert groups[0]["positions_len"] == len(LEFT_ARM_JOINTS)
    assert groups[0]["velocities_len"] == len(LEFT_ARM_JOINTS)
    assert groups[0]["target_positions"] == left_target


def test_gdk_motion_runtime_executes_dual_arm_abs_joint_with_dual_group() -> None:
    FakeAgibotGdk.reset()
    left_target = [0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17]
    right_target = [0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27]
    expected_velocity = 0.03

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="left_arm",
                    control_type="ABS_JOINT",
                    action_data=left_target,
                ),
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=right_target,
                ),
            ),
            speed=expected_velocity,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    expected_target = left_target + right_target
    assert result["executed"] is True
    assert FakeAgibotGdk.robot.arm_move_calls == [
        (
            expected_target,
            [expected_velocity] * len(DUAL_ARM_JOINTS),
            CONTROL_GROUP_DUAL_ARM,
        )
    ]

    groups = result["groups"]
    assert isinstance(groups, list)
    assert groups[0]["method"] == "move_arm_joint"
    assert groups[0]["requested_body_parts"] == ["left_arm", "right_arm"]
    assert groups[0]["control_group"] == CONTROL_GROUP_DUAL_ARM
    assert groups[0]["interface_mode"] == "dual_arm_14d"
    assert groups[0]["joint_order"] == list(DUAL_ARM_JOINTS)
    assert groups[0]["positions_len"] == len(DUAL_ARM_JOINTS)
    assert groups[0]["velocities_len"] == len(DUAL_ARM_JOINTS)
    assert groups[0]["target_positions"] == expected_target


def test_gdk_motion_runtime_executes_left_arm_abs_pose(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    disable_abs_pose_sleep(monkeypatch)
    FakeAgibotGdk.reset()
    target_pose = [0.101, 0.202, 0.303, 0.0, 0.0, 0.0, 1.0]

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="left_arm",
                    control_type="ABS_POSE",
                    action_data=target_pose,
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["action"] == ACTION_TASKFLOW_ABS_POSE
    assert result["method"] == "end_effector_pose_control"
    assert result["arm_frame_name"] == ARM_FRAME_NAMES["left_arm"]
    assert result["end_effector_group"] == FakeAgibotGdk.kLeftArm
    assert result["life_time_seconds"] == TASKFLOW_ABS_POSE_LIFE_TIME_SECONDS
    assert result["target_pose"] == {
        "position": target_pose[:3],
        "orientation": target_pose[3:],
    }
    assert result["requested_speed_unit"] == "m/s"
    assert result["effective_linear_speed_mps"] == 0.05
    assert result["speed_mapping_applied"] is True
    assert result["trajectory_policy"] == "linear_position_slerp_orientation_50hz"
    assert result["trajectory_step_count"] == 4
    assert result["final_hold_steps"] == 10
    assert result["control_call_count"] == 14
    assert result["trajectory_max_step_m"] == pytest.approx(0.001)
    assert FakeAgibotGdk.robot.move_call_order == ["pose"] * 14
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 14
    first_call = FakeAgibotGdk.robot.pose_control_calls[0]
    assert first_call.left_end_effector_pose.position.x == pytest.approx(0.10025)
    call = FakeAgibotGdk.robot.pose_control_calls[-1]
    assert call.group == FakeAgibotGdk.kLeftArm
    assert call.left_end_effector_pose.position.x == target_pose[0]
    assert call.left_end_effector_pose.position.y == target_pose[1]
    assert call.left_end_effector_pose.position.z == target_pose[2]
    assert call.right_end_effector_pose.position.x == 0.4
    assert result["gdk_session"]["purpose"] == ACTION_TASKFLOW_ABS_POSE


def test_gdk_motion_runtime_executes_right_arm_abs_pose(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    disable_abs_pose_sleep(monkeypatch)
    FakeAgibotGdk.reset()
    target_pose = [
        0.401,
        -0.199,
        0.801,
        0.10259783520851541,
        0.20519567041703082,
        0.3077935056255462,
        0.9233805168766387,
    ]

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_POSE",
                    action_data=target_pose,
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["action"] == ACTION_TASKFLOW_ABS_POSE
    assert result["body_part"] == "right_arm"
    assert result["arm_frame_name"] == ARM_FRAME_NAMES["right_arm"]
    assert result["end_effector_group"] == FakeAgibotGdk.kRightArm
    assert result["target_pose"] == {
        "position": target_pose[:3],
        "orientation": target_pose[3:],
    }
    assert result["trajectory_step_count"] == 2
    assert result["control_call_count"] == 12
    assert FakeAgibotGdk.robot.move_call_order == ["pose"] * 12
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 12
    call = FakeAgibotGdk.robot.pose_control_calls[-1]
    assert call.group == FakeAgibotGdk.kRightArm
    assert call.right_end_effector_pose.position.x == target_pose[0]
    assert call.right_end_effector_pose.position.y == target_pose[1]
    assert call.right_end_effector_pose.position.z == target_pose[2]
    assert call.left_end_effector_pose.position.x == 0.1
    assert result["gdk_session"]["purpose"] == ACTION_TASKFLOW_ABS_POSE


def test_gdk_motion_runtime_refuses_waist_abs_pose_before_import() -> None:
    def forbidden_import(_name: str) -> Any:
        raise AssertionError("GDK must not be imported for unsupported ABS_POSE target")

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="waist",
                    control_type="ABS_POSE",
                    action_data=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=forbidden_import,
    )

    assert result["executed"] is False
    assert result["action"] == ACTION_TASKFLOW_ABS_POSE
    assert result["error_stage"] == "validate_params"
    assert "only supports one of" in str(result["error_msg"])


def test_gdk_motion_runtime_refuses_abs_pose_large_delta_before_control_call() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="left_arm",
                    control_type="ABS_POSE",
                    action_data=[0.45, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["action"] == ACTION_TASKFLOW_ABS_POSE
    assert result["error_stage"] == "execute_motion_plan_targets"
    assert "ABS_POSE translation delta" in str(result["error_msg"])
    assert FakeAgibotGdk.robot.pose_control_calls == []


def test_gdk_motion_runtime_allows_abs_pose_delta_with_explicit_limit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    disable_abs_pose_sleep(monkeypatch)
    FakeAgibotGdk.reset()
    target_pose = [0.45, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="left_arm",
                    control_type="ABS_POSE",
                    action_data=target_pose,
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
            TASKFLOW_ABS_POSE_MAX_TRANSLATION_ENV: "0.40",
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["max_translation_m"] == 0.4
    assert result["target_translation_delta_m"]["norm"] == pytest.approx(0.35)
    assert result["trajectory_step_count"] == 350
    assert result["control_call_count"] == 360
    last_call = FakeAgibotGdk.robot.pose_control_calls[-1]
    assert last_call.left_end_effector_pose.position.x == pytest.approx(0.45)


def test_gdk_motion_runtime_refuses_abs_pose_when_interpolation_exceeds_timeout() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="left_arm",
                    control_type="ABS_POSE",
                    action_data=[0.2, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                ),
            ),
            speed=0.001,
            timeout=1.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["action"] == ACTION_TASKFLOW_ABS_POSE
    assert result["error_stage"] == "execute_motion_plan_targets"
    assert "interpolated trajectory duration" in str(result["error_msg"])
    assert FakeAgibotGdk.robot.pose_control_calls == []


def test_gdk_motion_runtime_reuses_process_session_across_multiple_calls() -> None:
    FakeAgibotGdk.reset()
    manager = GdkSessionManager(import_module=lambda _name: FakeAgibotGdk)
    env = {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }
    motion_params = MotionPlanParams(
        targets=(
            MotionPlanTarget(
                body_part="right_arm",
                control_type="ABS_JOINT",
                action_data=[0.1] * 7,
            ),
        ),
        speed=0.05,
        timeout=50.0,
    )

    first = run_gdk_motion_plan_abs_joint(
        motion_params,
        environ=env,
        session_manager=manager,
    )
    second = run_gdk_motion_plan_abs_joint(
        motion_params,
        environ=env,
        session_manager=manager,
    )
    shutdown = manager.shutdown()

    assert first["executed"] is True
    assert second["executed"] is True
    assert first["gdk_init"]["reused"] is False
    assert second["gdk_init"]["reused"] is True
    assert first["gdk_release"]["called"] is False
    assert second["gdk_release"]["called"] is False
    assert FakeAgibotGdk.init_called == 1
    assert shutdown["called"] is True
    assert FakeAgibotGdk.release_called == 1


def test_gdk_motion_runtime_releases_parent_lock_after_subprocess_timeout(monkeypatch) -> None:
    manager = GdkSessionManager()

    def fake_subprocess(_motion_params: MotionPlanParams, **_kwargs: object) -> dict[str, object]:
        return {
            "available": False,
            "executed": False,
            "backend": "agibot_gdk.Robot",
            "action": "taskflow_abs_joint",
            "error_stage": "gdk_operation_timeout",
            "error_code": GDK_OPERATION_TIMEOUT_CODE,
            "error_type": "GdkOperationTimeout",
            "error_msg": "timed out",
        }

    monkeypatch.setattr(
        "gsa_taskflow_executor.gdk.motion_runtime.run_motion_abs_joint_in_subprocess",
        fake_subprocess,
    )

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=[0.1] * 7,
                ),
            ),
            speed=0.05,
            timeout=0.1,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        session_manager=manager,
    )

    assert result["executed"] is False
    assert result["error_code"] == GDK_OPERATION_TIMEOUT_CODE
    assert manager.busy is False
    requirement = current_gdk_recovery_requirement()
    assert requirement is not None
    assert requirement.reason == "worker_timeout"

    refused = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=[0.1] * 7,
                ),
            ),
            speed=0.05,
            timeout=0.1,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: (_ for _ in ()).throw(
            AssertionError("GDK must not be imported while recovery is required")
        ),
    )

    assert refused["executed"] is False
    assert refused["error_code"] == "GDK_RECOVERY_REQUIRED"

    lease = manager.acquire(blocking=False, initialize=False, purpose="current_pose")
    assert lease is not None
    lease.release()


def test_gdk_motion_runtime_refuses_cartesian_impedance_before_arm_move() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.motion_status = FakeMotionStatus(
        control_mode="CTRL_CARTESIAN_IMPEDANCE",
    )

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="right_arm",
                    control_type="ABS_JOINT",
                    action_data=[0.1] * 7,
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "gdk_control_mode_unsupported"
    assert result["error_code"] == "GDK_CONTROL_MODE_UNSUPPORTED"
    assert result["error_msg"] == "当前为笛卡尔阻抗模式，请切换到关节位置/规划控制模式后重试"
    assert result["motion_control_status"]["control_mode"] == "CTRL_CARTESIAN_IMPEDANCE"
    assert FakeAgibotGdk.robot.arm_move_calls == []


def test_gdk_motion_runtime_refuses_cartesian_impedance_before_waist_move() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.motion_status = FakeMotionStatus(
        mode="CTRL_CARTESIAN_IMPEDANCE",
    )

    result = run_gdk_motion_plan_abs_joint(
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part="waist",
                    control_type="ABS_JOINT",
                    action_data=[0.01] * 5,
                ),
            ),
            speed=0.05,
            timeout=50.0,
        ),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "gdk_control_mode_unsupported"
    assert result["error_code"] == "GDK_CONTROL_MODE_UNSUPPORTED"
    assert FakeAgibotGdk.robot.waist_move_calls == []
