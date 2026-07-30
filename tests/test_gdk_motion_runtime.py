from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.gdk_control_probe import (
    CONTROL_GROUP_DUAL_ARM,
    DUAL_ARM_JOINTS,
)
from gsa_taskflow_executor.gdk_motion_runtime import (
    TASKFLOW_ABS_JOINT_CONFIRMATION,
    WAIST_JOINTS,
    run_gdk_motion_plan_abs_joint,
)
from gsa_taskflow_executor.taskflow_parser import MotionPlanParams, MotionPlanTarget


class FakeMotionStatus:
    error_code = 0
    error_msg = ""


class FakeRobot:
    def __init__(self) -> None:
        self.arm_positions = [index * 0.01 for index in range(len(DUAL_ARM_JOINTS))]
        self.waist_positions = [index * 0.001 for index in range(len(WAIST_JOINTS))]
        self.arm_move_calls: list[tuple[list[float], list[float], int]] = []
        self.waist_move_calls: list[tuple[list[float], list[float]]] = []

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
        self.arm_move_calls.append((list(positions), list(velocities), control_group))
        self.arm_positions = list(positions)
        return 0

    def move_waist_joint(self, positions: list[float], velocities: list[float]) -> int:
        self.waist_move_calls.append((list(positions), list(velocities)))
        self.waist_positions = list(positions)
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
    assert FakeAgibotGdk.release_called == 1
    assert FakeAgibotGdk.robot.arm_move_calls == [
        (
            [index * 0.01 for index in range(7)] + right_target,
            [expected_velocity] * len(DUAL_ARM_JOINTS),
            CONTROL_GROUP_DUAL_ARM,
        )
    ]
    assert FakeAgibotGdk.robot.waist_move_calls == [
        (waist_target, [expected_velocity] * len(WAIST_JOINTS))
    ]

    groups = result["groups"]
    assert isinstance(groups, list)
    assert groups[0]["method"] == "move_arm_joint"
    assert groups[0]["requested_body_parts"] == ["right_arm"]
    assert groups[0]["effective_gdk_velocity"] == expected_velocity
    assert groups[0]["velocity_source"] == "taskflow_speed"
    assert groups[1]["method"] == "move_waist_joint"
    assert groups[1]["effective_gdk_velocity"] == expected_velocity
    assert groups[1]["velocity_source"] == "taskflow_speed"
