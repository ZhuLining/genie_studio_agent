from __future__ import annotations

import pytest

import gsa_taskflow_executor.code_scripts.runtime as code_script_runtime
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.taskflow.parser import (
    ScriptOutputVariable,
    ScriptParams,
)


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
        self.move_calls: list[FakeJointStates] = []

    def get_end_state(self) -> dict[str, object]:
        return {"right_tool": {"actual_openness": [0.6]}}

    def move_ee_pos(self, joint_states: FakeJointStates) -> int:
        self.move_calls.append(joint_states)
        return 0


class FakeAgibotGdk:
    robot = FakeEndEffectorRobot()
    init_called = 0
    JointState = FakeJointState
    JointStates = FakeJointStates

    class GDKRes:
        kSuccess = 0

    @classmethod
    def reset(cls) -> None:
        cls.robot = FakeEndEffectorRobot()
        cls.init_called = 0

    @classmethod
    def Robot(cls) -> FakeEndEffectorRobot:
        return cls.robot

    @classmethod
    def gdk_init(cls) -> int:
        cls.init_called += 1
        return cls.GDKRes.kSuccess


def test_code_script_rejects_legacy_probe_script_id_without_running_subprocess() -> None:
    def forbidden_runner(*_args: object, **_kwargs: object) -> object:
        raise AssertionError("legacy probe scripts must not run through the code node")

    result = code_script_runtime.run_code_script(
        ScriptParams(script_id="gdk_hold_current_dual_arm", timeout=20),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
        },
        runner=forbidden_runner,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "validate_script"
    assert result["error_msg"] == "unsupported script_id: gdk_hold_current_dual_arm"
    assert result["safety_gate"] == {
        "enabled": False,
        "confirmed": True,
        "reason": "no_gdk_control",
    }


def test_code_echo_inputs_runs_without_gdk_safety_gate() -> None:
    def forbidden_runner(*_args: object, **_kwargs: object) -> object:
        raise AssertionError("code_echo_inputs must not start a subprocess")

    result = code_script_runtime.run_code_script(
        ScriptParams(
            script_id="code_echo_inputs",
            timeout=50,
            output_variables=(
                ScriptOutputVariable(name="out_1", value_type="string"),
            ),
        ),
        environ={},
        runner=forbidden_runner,
        inputs={"out_1": "hello"},
    )

    assert result["executed"] is True
    assert result["backend"] == "executor_builtin_code"
    assert result["safety_gate"] == {
        "enabled": False,
        "confirmed": True,
        "reason": "no_gdk_control",
    }
    assert result["outputs"] == {"out_1": "hello"}


def test_code_echo_inputs_requires_matching_input_name() -> None:
    result = code_script_runtime.run_code_script(
        ScriptParams(
            script_id="code_echo_inputs",
            timeout=50,
            output_variables=(
                ScriptOutputVariable(name="out_1", value_type="string"),
            ),
        ),
        environ={},
        inputs={"other": "hello"},
    )

    assert result["executed"] is False
    assert result["backend"] == "executor_builtin_code"
    assert result["error_stage"] == "validate_outputs"
    assert result["safety_gate"]["enabled"] is False


def test_code_opening_plus_adjusts_and_clamps() -> None:
    result = code_script_runtime.run_code_script(
        ScriptParams(
            script_id="code_opening_plus_0p1",
            timeout=50,
            output_variables=(
                ScriptOutputVariable(name="adjusted_opening", value_type="number"),
            ),
        ),
        environ={},
        inputs={"actual_openness": [0.95]},
    )

    assert result["executed"] is True
    assert result["backend"] == "executor_builtin_code"
    assert result["outputs"] == {"adjusted_opening": 1.0}
    assert result["source_opening"] == 0.95
    assert result["delta"] == 0.1
    assert result["clamped"] is True
    assert result["safety_gate"]["enabled"] is False


def test_code_opening_minus_adjusts_without_clamp() -> None:
    result = code_script_runtime.run_code_script(
        ScriptParams(
            script_id="code_opening_minus_0p1",
            timeout=50,
            output_variables=(
                ScriptOutputVariable(name="adjusted_opening", value_type="number"),
            ),
        ),
        environ={},
        inputs={"actual_openness": [0.5]},
    )

    assert result["executed"] is True
    assert result["outputs"] == {"adjusted_opening": 0.4}
    assert result["source_opening"] == 0.5
    assert result["delta"] == -0.1
    assert result["clamped"] is False


def test_code_opening_adjustment_requires_actual_openness_array() -> None:
    result = code_script_runtime.run_code_script(
        ScriptParams(script_id="code_opening_plus_0p1", timeout=50),
        environ={},
        inputs={"actual_openness": "0.5"},
    )

    assert result["executed"] is False
    assert result["backend"] == "executor_builtin_code"
    assert result["error_stage"] == "validate_inputs"
    assert result["safety_gate"]["enabled"] is False


def test_code_move_end_effector_refuses_before_gdk_import_without_safety_gate() -> None:
    def forbidden_gdk_import(_name: str) -> object:
        raise AssertionError("GDK code script must not import agibot_gdk before safety gate")

    result = code_script_runtime.run_code_script(
        ScriptParams(script_id="code_move_end_effector", timeout=50),
        environ={},
        session_manager=GdkSessionManager(import_module=forbidden_gdk_import),
        inputs={"opening": 0.6},
    )

    assert result["executed"] is False
    assert result["error_stage"] == "safety_gate"
    assert result["error_msg"] == "ENABLE_GDK_CONTROL must be 1"


def test_code_move_end_effector_runs_whitelisted_script_file_directly() -> None:
    FakeAgibotGdk.reset()
    runtime_env = {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
    }
    manager = GdkSessionManager(import_module=lambda _name: FakeAgibotGdk)

    result = code_script_runtime.run_code_script(
        ScriptParams(
            script_id="code_move_end_effector",
            timeout=50,
            output_variables=(
                ScriptOutputVariable(name="actual_openness", value_type="array"),
            ),
        ),
        environ=runtime_env,
        session_manager=manager,
        inputs={
            "opening": 0.6,
            "target_end": "right_tool",
            "end_effector_type": "omnipicker",
        },
    )

    assert result["executed"] is True
    assert result["script_module"] == (
        "gsa_taskflow_executor.code_scripts.scripts.move_end_effector"
    )
    assert result["code_backend"] == "executor_file_gdk_script"
    assert result["method"] == "move_ee_pos"
    assert result["outputs"]["actual_openness"] == [0.6]
    assert result["outputs"]["target_end"] == "right_tool"
    assert result["outputs"]["end_effector_type"] == "omnipicker"
    assert result["gdk_session"]["purpose"] == "code_script:code_move_end_effector"
    assert FakeAgibotGdk.init_called == 1
    assert len(FakeAgibotGdk.robot.move_calls) == 1
    joint_states = FakeAgibotGdk.robot.move_calls[0]
    assert joint_states.group == "right_tool"
    assert joint_states.target_type == "omnipicker"
    assert joint_states.nums == 1
    assert joint_states.states[0].position == pytest.approx(-0.471)


def test_code_move_end_effector_requires_opening_input() -> None:
    result = code_script_runtime.run_code_script(
        ScriptParams(script_id="code_move_end_effector", timeout=50),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
        },
        session_manager=GdkSessionManager(import_module=lambda _name: FakeAgibotGdk),
        inputs={},
    )

    assert result["executed"] is False
    assert result["backend"] == "executor_file_gdk_script"
    assert result["error_stage"] == "validate_inputs"
