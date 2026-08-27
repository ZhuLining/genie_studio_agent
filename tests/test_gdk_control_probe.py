from __future__ import annotations

from typing import Any

import pytest

from gsa_taskflow_executor.gdk.control_probe import (
    ACTION_ABS_POSE_CONTROL_FRAME,
    ACTION_ABS_POSE_DRY_RUN,
    ACTION_ABS_POSE_HOLD_CURRENT_RIGHT,
    ACTION_ABS_POSE_NUDGE_LEFT_X_0P001,
    ACTION_ABS_POSE_NUDGE_LEFT_Y_0P001,
    ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001,
    ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S,
    ACTION_ABS_POSE_NUDGE_RIGHT_X_0P001,
    ACTION_ABS_POSE_NUDGE_RIGHT_Y_0P001,
    ACTION_ABS_POSE_NUDGE_RIGHT_Z_0P001,
    ACTION_ABS_POSE_NUDGE_RIGHT_Z_0P001_LIFE_2S,
    ACTION_HOLD_CURRENT,
    ACTION_NUDGE_LEFT_J7,
    ACTION_NUDGE_RIGHT_J7,
    CONTROL_GROUP_DUAL_ARM,
    DUAL_ARM_JOINTS,
    run_gdk_control_probe,
)


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


class FakeMotionStatus:
    error_code = 0
    error_msg = ""

    def __init__(self) -> None:
        self.frame_names = ["arm_l_end_link", "arm_r_end_link"]
        self.frame_poses = [
            make_fake_pose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0),
            make_fake_pose(0.4, -0.2, 0.8, 0.1, 0.2, 0.3, 0.9),
        ]


class FakeEndEffectorPose:
    def __init__(self) -> None:
        self.group = 0
        self.left_end_effector_pose = FakePose()
        self.right_end_effector_pose = FakePose()
        self.life_time = 0.0


def make_fake_pose(
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


class FakeRobot:
    def __init__(self) -> None:
        self.positions = [index * 0.01 for index in range(len(DUAL_ARM_JOINTS))]
        self.move_calls: list[tuple[list[float], list[float], int]] = []
        self.pose_control_calls: list[FakeEndEffectorPose] = []

    def get_joint_states(self) -> dict[str, object]:
        return {
            "nums": len(DUAL_ARM_JOINTS),
            "states": [
                {
                    "name": name,
                    "position": self.positions[index],
                    "motor_position": self.positions[index],
                    "error_code": 0,
                }
                for index, name in enumerate(DUAL_ARM_JOINTS)
            ],
        }

    def get_joint_limits(self) -> dict[str, dict[str, float]]:
        return {name: {"min": -10.0, "max": 10.0} for name in DUAL_ARM_JOINTS}

    def get_motion_control_status(self) -> FakeMotionStatus:
        return FakeMotionStatus()

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
        self.move_calls.append((list(positions), list(velocities), control_group))
        self.positions = list(positions)
        return 0

    def end_effector_pose_control(self, end_pose: FakeEndEffectorPose) -> int:
        self.pose_control_calls.append(end_pose)
        return 0

    def get_end_pose_control_frame(self) -> list[str]:
        return ["base_link", "tool0"]

    def switch_end_pose_control_frame(self) -> int:
        raise AssertionError("control frame probe must not switch frame")


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


def test_gdk_control_probe_refuses_without_safety_gate_before_importing() -> None:
    def forbidden_import(_name: str) -> Any:
        raise AssertionError("GDK must not be imported without safety confirmation")

    result = run_gdk_control_probe(
        ACTION_HOLD_CURRENT,
        environ={},
        import_module=forbidden_import,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "safety_gate"
    assert result["expected_confirmation"] == "HOLD_CURRENT_DUAL_ARM"


def test_gdk_control_probe_hold_current_uses_14_positions_and_control_group_2() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_HOLD_CURRENT,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "HOLD_CURRENT_DUAL_ARM",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["positions_len"] == 14
    assert result["velocities_len"] == 14
    assert result["control_group"] == CONTROL_GROUP_DUAL_ARM
    assert result["max_abs_diff"] == 0.0
    assert result["gdk_release"] == {"called": True, "success": True, "return": None}
    assert FakeAgibotGdk.robot.move_calls == [
        (
            [index * 0.01 for index in range(len(DUAL_ARM_JOINTS))],
            [0.02] * len(DUAL_ARM_JOINTS),
            CONTROL_GROUP_DUAL_ARM,
        )
    ]
    assert FakeAgibotGdk.init_called == 1
    assert FakeAgibotGdk.release_called == 1


def test_gdk_control_probe_abs_pose_dry_run_builds_targets_without_motion() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_DRY_RUN,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_DRY_RUN",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is False
    assert result["probe_succeeded"] is True
    assert result["motion_attempted"] is False
    assert result["control_method_available"] is True
    assert FakeAgibotGdk.robot.move_calls == []
    assert FakeAgibotGdk.robot.pose_control_calls == []
    frame_poses = result["current_frame_poses"]
    assert isinstance(frame_poses, dict)
    assert frame_poses["arm_r_end_link"]["position"] == [0.4, -0.2, 0.8]
    dry_run_targets = result["dry_run_targets"]
    assert isinstance(dry_run_targets, dict)
    assert dry_run_targets["right_arm"]["group"] == 8


def test_gdk_control_probe_abs_pose_control_frame_is_read_only() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_CONTROL_FRAME,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_CONTROL_FRAME",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is False
    assert result["probe_succeeded"] is True
    assert result["motion_attempted"] is False
    assert result["control_frame_method_available"] is True
    assert result["switch_method_available"] is True
    assert result["control_frame"] == ["base_link", "tool0"]
    assert FakeAgibotGdk.robot.move_calls == []
    assert FakeAgibotGdk.robot.pose_control_calls == []


def test_gdk_control_probe_abs_pose_hold_current_right_calls_pose_control() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_HOLD_CURRENT_RIGHT,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_HOLD_CURRENT_RIGHT",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["probe_succeeded"] is True
    assert result["motion_attempted"] is True
    assert result["arm"] == "right_arm"
    assert result["end_effector_group"] == 8
    assert result["motion_status_ok_after"] is True
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 1
    call = FakeAgibotGdk.robot.pose_control_calls[0]
    assert call.group == 8
    assert call.life_time == pytest.approx(0.5)
    assert call.right_end_effector_pose.position.x == pytest.approx(0.4)
    assert call.right_end_effector_pose.orientation.w == pytest.approx(0.9)


def test_gdk_control_probe_abs_pose_nudge_left_z_returns_to_origin() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_LEFT_Z_0P001",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["probe_succeeded"] is True
    assert result["arm"] == "left_arm"
    assert result["nudge_axis"] == "z"
    assert result["nudge_delta_m"] == pytest.approx(0.001)
    assert result["return_attempted"] is True
    assert result["control_returns"] == {"nudge": 0, "return_to_origin": 0}
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 2
    nudge_call, return_call = FakeAgibotGdk.robot.pose_control_calls
    assert nudge_call.group == 4
    assert nudge_call.left_end_effector_pose.position.z == pytest.approx(0.301)
    assert return_call.group == 4
    assert return_call.left_end_effector_pose.position.z == pytest.approx(0.3)
    mid = result["mid"]
    assert isinstance(mid, dict)
    assert mid["motion_status_ok"] is True


def test_gdk_control_probe_abs_pose_nudge_left_x_and_y_return_to_origin() -> None:
    FakeAgibotGdk.reset()

    x_result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_LEFT_X_0P001,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_LEFT_X_0P001",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert x_result["executed"] is True
    assert x_result["probe_succeeded"] is True
    assert x_result["nudge_axis"] == "x"
    x_nudge_call, x_return_call = FakeAgibotGdk.robot.pose_control_calls
    assert x_nudge_call.left_end_effector_pose.position.x == pytest.approx(0.101)
    assert x_return_call.left_end_effector_pose.position.x == pytest.approx(0.1)

    FakeAgibotGdk.reset()
    y_result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_LEFT_Y_0P001,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_LEFT_Y_0P001",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert y_result["executed"] is True
    assert y_result["probe_succeeded"] is True
    assert y_result["nudge_axis"] == "y"
    y_nudge_call, y_return_call = FakeAgibotGdk.robot.pose_control_calls
    assert y_nudge_call.left_end_effector_pose.position.y == pytest.approx(0.201)
    assert y_return_call.left_end_effector_pose.position.y == pytest.approx(0.2)


def test_gdk_control_probe_abs_pose_nudge_left_z_life_2s_uses_longer_lifetime() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_LEFT_Z_0P001_LIFE_2S",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["probe_succeeded"] is True
    assert result["life_time_seconds"] == pytest.approx(2.0)
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 2
    assert FakeAgibotGdk.robot.pose_control_calls[0].life_time == pytest.approx(2.0)
    assert FakeAgibotGdk.robot.pose_control_calls[1].life_time == pytest.approx(2.0)


def test_gdk_control_probe_abs_pose_nudge_right_z_returns_to_origin() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_RIGHT_Z_0P001,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_RIGHT_Z_0P001",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["probe_succeeded"] is True
    assert result["arm"] == "right_arm"
    assert result["nudge_axis"] == "z"
    assert result["end_effector_group"] == 8
    assert result["return_attempted"] is True
    assert result["control_returns"] == {"nudge": 0, "return_to_origin": 0}
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 2
    nudge_call, return_call = FakeAgibotGdk.robot.pose_control_calls
    assert nudge_call.group == 8
    assert nudge_call.right_end_effector_pose.position.z == pytest.approx(0.801)
    assert nudge_call.left_end_effector_pose.position.z == pytest.approx(0.3)
    assert return_call.group == 8
    assert return_call.right_end_effector_pose.position.z == pytest.approx(0.8)
    mid = result["mid"]
    assert isinstance(mid, dict)
    assert mid["motion_status_ok"] is True


def test_gdk_control_probe_abs_pose_nudge_right_x_and_y_return_to_origin() -> None:
    FakeAgibotGdk.reset()

    x_result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_RIGHT_X_0P001,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_RIGHT_X_0P001",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert x_result["executed"] is True
    assert x_result["probe_succeeded"] is True
    assert x_result["arm"] == "right_arm"
    assert x_result["nudge_axis"] == "x"
    x_nudge_call, x_return_call = FakeAgibotGdk.robot.pose_control_calls
    assert x_nudge_call.right_end_effector_pose.position.x == pytest.approx(0.401)
    assert x_return_call.right_end_effector_pose.position.x == pytest.approx(0.4)

    FakeAgibotGdk.reset()
    y_result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_RIGHT_Y_0P001,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_RIGHT_Y_0P001",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert y_result["executed"] is True
    assert y_result["probe_succeeded"] is True
    assert y_result["arm"] == "right_arm"
    assert y_result["nudge_axis"] == "y"
    y_nudge_call, y_return_call = FakeAgibotGdk.robot.pose_control_calls
    assert y_nudge_call.right_end_effector_pose.position.y == pytest.approx(-0.199)
    assert y_return_call.right_end_effector_pose.position.y == pytest.approx(-0.2)


def test_gdk_control_probe_abs_pose_nudge_right_z_life_2s_uses_longer_lifetime() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_ABS_POSE_NUDGE_RIGHT_Z_0P001_LIFE_2S,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "ABS_POSE_NUDGE_RIGHT_Z_0P001_LIFE_2S",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["probe_succeeded"] is True
    assert result["arm"] == "right_arm"
    assert result["life_time_seconds"] == pytest.approx(2.0)
    assert len(FakeAgibotGdk.robot.pose_control_calls) == 2
    assert FakeAgibotGdk.robot.pose_control_calls[0].life_time == pytest.approx(2.0)
    assert FakeAgibotGdk.robot.pose_control_calls[1].life_time == pytest.approx(2.0)


def test_gdk_control_probe_nudge_left_j7_returns_to_origin() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_NUDGE_LEFT_J7,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "NUDGE_LEFT_J7_0P005",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["nudge_joint_name"] == "idx27_arm_l_joint7"
    assert result["mid_left_j7_diff"] == pytest.approx(0.005)
    assert result["max_abs_diff"] == 0.0
    assert len(FakeAgibotGdk.robot.move_calls) == 2
    assert FakeAgibotGdk.robot.move_calls[0][0][6] == pytest.approx(0.065)
    assert FakeAgibotGdk.robot.move_calls[1][0][6] == 0.06


def test_gdk_control_probe_nudge_right_j7_returns_to_origin() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_control_probe(
        ACTION_NUDGE_RIGHT_J7,
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "NUDGE_RIGHT_J7_0P005",
        },
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        settle_seconds=0,
    )

    assert result["executed"] is True
    assert result["nudge_joint_name"] == "idx67_arm_r_joint7"
    assert result["mid_right_j7_diff"] == pytest.approx(0.005)
    assert result["max_abs_diff"] == 0.0
    assert len(FakeAgibotGdk.robot.move_calls) == 2
    assert FakeAgibotGdk.robot.move_calls[0][0][-1] == pytest.approx(0.135)
    assert FakeAgibotGdk.robot.move_calls[1][0][-1] == 0.13
