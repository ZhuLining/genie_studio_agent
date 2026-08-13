from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.gdk.current_pose import run_gdk_current_pose_snapshot
from gsa_taskflow_executor.gdk.recovery import (
    current_gdk_recovery_requirement,
    mark_gdk_recovery_required,
)
from gsa_taskflow_executor.gdk.session import GdkSessionManager

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
    mode = 1
    control_mode = 1
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
    init_called = 0

    class GDKRes:
        kSuccess = 0

    @classmethod
    def reset(cls) -> None:
        cls.init_called = 0

    @classmethod
    def gdk_init(cls) -> int:
        cls.init_called += 1
        return cls.GDKRes.kSuccess

    Robot = FakeRobot


def test_current_pose_snapshot_matches_desktop_contract() -> None:
    FakeGdk.reset()
    result = run_gdk_current_pose_snapshot(import_module=lambda _name: FakeGdk)

    assert result["available"] is True
    assert result["backend"] == "agibot_gdk.Robot"
    assert result["jointCount"] == 22
    assert result["groups"]["left_arm"]["positions"] == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    assert result["groups"]["right_arm"]["positions"] == [0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14]
    assert result["groups"]["waist"]["positions"] == [0.15, 0.16, 0.17, 0.18, 0.19]
    assert result["groups"]["waist"]["joints"][0]["limit"] == {"min": -3.14, "max": 3.14}
    assert result["motionStatus"] == {
        "mode": 1,
        "controlMode": 1,
        "errorCode": 0,
        "errorMsg": "",
    }
    assert result["wholeBodyStatus"] == {"left_arm_error": 0, "right_arm_error": 0}
    assert FakeGdk.init_called == 1
    assert result["gdk_session"]["policy"] == "process_managed_session"
    assert result["gdk_session"]["purpose"] == "current_pose"


def test_current_pose_snapshot_confirms_gdk_recovery() -> None:
    FakeGdk.reset()
    mark_gdk_recovery_required(
        operation="taskflow_abs_joint",
        reason="worker_timeout",
        source_result={"error_code": "GDK_OPERATION_TIMEOUT"},
    )

    result = run_gdk_current_pose_snapshot(import_module=lambda _name: FakeGdk)

    assert result["available"] is True
    assert result["gdk_recovery"]["confirmed"] is True
    assert current_gdk_recovery_requirement() is None


def test_current_pose_snapshot_reports_import_failure() -> None:
    def fail_import(_name: str) -> object:
        raise ModuleNotFoundError("agibot_gdk")

    result = run_gdk_current_pose_snapshot(import_module=fail_import)

    assert result["available"] is False
    assert result["errorStage"] == "import_agibot_gdk"
    assert result["errorType"] == "ModuleNotFoundError"


def test_current_pose_snapshot_returns_busy_without_importing_gdk() -> None:
    def forbidden_import(_name: str) -> Any:
        raise AssertionError("current pose must not import GDK while control lock is busy")

    manager = GdkSessionManager()
    lease = manager.acquire(blocking=True, initialize=False, purpose="taskflow_abs_joint")
    assert lease is not None
    try:
        result = run_gdk_current_pose_snapshot(
            import_module=forbidden_import,
            session_manager=manager,
        )

        assert result["available"] is False
        assert result["busy"] is True
        assert result["errorStage"] == "gdk_session_busy"
        assert result["errorMsg"] == "GDK 正在执行控制动作，当前位姿读取已拒绝"
        assert result["activePurpose"] == "taskflow_abs_joint"
    finally:
        lease.release()
