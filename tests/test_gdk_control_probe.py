from __future__ import annotations

from typing import Any

import pytest

from gsa_taskflow_executor.gdk_control_probe import (
    ACTION_HOLD_CURRENT,
    ACTION_NUDGE_LEFT_J7,
    ACTION_NUDGE_RIGHT_J7,
    CONTROL_GROUP_DUAL_ARM,
    DUAL_ARM_JOINTS,
    run_gdk_control_probe,
)


class FakeMotionStatus:
    error_code = 0
    error_msg = ""


class FakeRobot:
    def __init__(self) -> None:
        self.positions = [index * 0.01 for index in range(len(DUAL_ARM_JOINTS))]
        self.move_calls: list[tuple[list[float], list[float], int]] = []

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


class FakeAgibotGdk:
    robot = FakeRobot()
    init_called = 0
    release_called = 0

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
