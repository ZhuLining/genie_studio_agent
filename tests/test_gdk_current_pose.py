from __future__ import annotations

from gsa_taskflow_executor.gdk_current_pose import run_gdk_current_pose_snapshot

LEFT_ARM = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_ARM = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]
WAIST = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]


class FakeMotionStatus:
    error_code = 0
    error_msg = ""


class FakeRobot:
    def get_joint_states(self) -> dict[str, object]:
        states = [
            {
                "name": name,
                "motor_position": index / 100,
                "error_code": 0,
            }
            for index, name in enumerate([*LEFT_ARM, *RIGHT_ARM, *WAIST], start=1)
        ]
        return {"nums": 22, "states": states}

    def get_joint_limits(self) -> dict[str, object]:
        return {
            name: {"min": -3.14, "max": 3.14}
            for name in [*LEFT_ARM, *RIGHT_ARM, *WAIST]
        }

    def get_motion_control_status(self) -> FakeMotionStatus:
        return FakeMotionStatus()

    def get_whole_body_status(self) -> dict[str, object]:
        return {"left_arm_error": 0, "right_arm_error": 0}


class FakeGdk:
    Robot = FakeRobot


def test_current_pose_snapshot_matches_desktop_contract() -> None:
    result = run_gdk_current_pose_snapshot(import_module=lambda _name: FakeGdk)

    assert result["available"] is True
    assert result["backend"] == "agibot_gdk.Robot"
    assert result["jointCount"] == 22
    assert result["groups"]["left_arm"]["positions"] == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    assert result["groups"]["right_arm"]["positions"] == [0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14]
    assert result["groups"]["waist"]["positions"] == [0.15, 0.16, 0.17, 0.18, 0.19]
    assert result["groups"]["waist"]["joints"][0]["limit"] == {"min": -3.14, "max": 3.14}
    assert result["motionStatus"] == {"errorCode": 0, "errorMsg": ""}
    assert result["wholeBodyStatus"] == {"left_arm_error": 0, "right_arm_error": 0}


def test_current_pose_snapshot_reports_import_failure() -> None:
    def fail_import(_name: str) -> object:
        raise ModuleNotFoundError("agibot_gdk")

    result = run_gdk_current_pose_snapshot(import_module=fail_import)

    assert result["available"] is False
    assert result["errorStage"] == "import_agibot_gdk"
    assert result["errorType"] == "ModuleNotFoundError"
