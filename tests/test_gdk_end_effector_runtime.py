from __future__ import annotations

from typing import Any

import pytest

from gsa_taskflow_executor.gdk.end_effector_runtime import (
    ACTION_TASKFLOW_END_EFFECTOR,
    GDK_END_EFFECTOR_TYPE_UNKNOWN,
    GDK_END_EFFECTOR_TYPE_UNSUPPORTED,
    run_gdk_end_effector_control,
)
from gsa_taskflow_executor.gdk.motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.taskflow.parser import EndEffectorParams


class FakeJointState:
    def __init__(self) -> None:
        self.position = 0.0


class FakeJointStates:
    def __init__(self) -> None:
        self.group = ""
        self.target_type = ""
        self.states: list[FakeJointState] = []
        self.nums = 0


class FakeEndEffectorRobot:
    def __init__(self) -> None:
        self.end_state: dict[str, object] = {"left_tool": {"target_type": "omnipicker"}}
        self.after_end_state: dict[str, object] | None = None
        self.move_calls: list[FakeJointStates] = []
        self.manager_to_probe: GdkSessionManager | None = None
        self.readonly_lease_during_move: object = "not_checked"

    def get_end_state(self) -> dict[str, object]:
        return self.end_state

    def move_ee_pos(self, joint_states: FakeJointStates) -> int:
        self.move_calls.append(joint_states)
        if self.after_end_state is not None:
            self.end_state = self.after_end_state
        if self.manager_to_probe is not None:
            self.readonly_lease_during_move = self.manager_to_probe.acquire(
                blocking=False,
                initialize=True,
                purpose="current_pose",
            )
        return 0


class FakeAgibotGdk:
    robot = FakeEndEffectorRobot()
    init_called = 0
    release_called = 0
    JointState = FakeJointState
    JointStates = FakeJointStates

    class GDKRes:
        kSuccess = 0

    @classmethod
    def reset(cls) -> None:
        cls.robot = FakeEndEffectorRobot()
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
    def Robot(cls) -> FakeEndEffectorRobot:  # noqa: N802 - mirrors agibot_gdk API
        return cls.robot


def test_end_effector_runtime_refuses_before_importing_without_safety_gate() -> None:
    def forbidden_import(_name: str) -> Any:
        raise AssertionError("GDK must not be imported without safety confirmation")

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type="omnipicker",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ={},
        import_module=forbidden_import,
    )

    assert result["executed"] is False
    assert result["action"] == ACTION_TASKFLOW_END_EFFECTOR
    assert result["error_stage"] == "safety_gate"
    assert result["error_msg"] == "ENABLE_GDK_CONTROL must be 1"


def test_end_effector_runtime_executes_single_joint_type() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type="omnipicker",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["action"] == ACTION_TASKFLOW_END_EFFECTOR
    assert result["method"] == "move_ee_pos"
    assert result["target_end"] == "left_tool"
    assert result["end_effector_type"] == "omnipicker"
    assert result["target_positions"] == pytest.approx([-0.3925])
    assert result["post_wait_seconds"] == 0.0
    assert result["wait_after_command"] is False
    assert result["actual_openness"] == pytest.approx([0.5])
    assert result["actual_openness_source"] == "requested_opening_fallback"
    assert FakeAgibotGdk.init_called == 1
    assert FakeAgibotGdk.release_called == 0
    assert result["gdk_release"] == {
        "called": False,
        "success": True,
        "reason": "process_managed_session",
    }

    [joint_states] = FakeAgibotGdk.robot.move_calls
    assert joint_states.group == "left_tool"
    assert joint_states.target_type == "omnipicker"
    assert joint_states.nums == 1
    assert [state.position for state in joint_states.states] == pytest.approx([-0.3925])


def test_end_effector_runtime_prefers_actual_openness_from_after_state() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.after_end_state = {
        "left_tool": {
            "target_type": "omnipicker",
            "actual_openness": [0.62],
        }
    }

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type="omnipicker",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["actual_openness"] == pytest.approx([0.62])
    assert result["actual_openness_source"] == "gdk_after_end_state"


def test_end_effector_runtime_waits_after_move_before_reading_after_state() -> None:
    FakeAgibotGdk.reset()
    sleep_calls: list[float] = []

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type="omnipicker",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=1.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
        sleep=sleep_calls.append,
    )

    assert result["executed"] is True
    assert sleep_calls == [1.0]
    assert result["post_wait_seconds"] == 1.0
    assert result["wait_after_command"] is True


def test_end_effector_runtime_infers_type_from_end_state() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.end_state = {"right_tool": {"target_type": "dahuan"}}

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="right_tool",
            end_effector_type=None,
            opening=0.25,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["end_effector_type"] == "dahuan"
    assert result["target_positions"] == pytest.approx([0.01875])
    [joint_states] = FakeAgibotGdk.robot.move_calls
    assert joint_states.group == "right_tool"
    assert joint_states.target_type == "dahuan"


def test_end_effector_runtime_refuses_unknown_robot_reported_type() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.end_state = {"left_tool": {}}

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type=None,
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["error_code"] == GDK_END_EFFECTOR_TYPE_UNKNOWN
    assert result["error_stage"] == "resolve_end_effector_type"
    assert FakeAgibotGdk.robot.move_calls == []


def test_end_effector_runtime_refuses_multi_joint_type_without_mapping() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type="o10_t2",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["error_code"] == GDK_END_EFFECTOR_TYPE_UNSUPPORTED
    assert result["error_stage"] == "validate_end_effector_type"
    assert FakeAgibotGdk.robot.move_calls == []


def test_end_effector_runtime_holds_gdk_session_during_move_call() -> None:
    FakeAgibotGdk.reset()
    manager = GdkSessionManager(import_module=lambda _name: FakeAgibotGdk)
    FakeAgibotGdk.robot.manager_to_probe = manager

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="left_tool",
            end_effector_type="omnipicker",
            opening=1.0,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        session_manager=manager,
    )

    assert result["executed"] is True
    assert FakeAgibotGdk.robot.readonly_lease_during_move is None


def confirmed_env() -> dict[str, str]:
    return {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }
