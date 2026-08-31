from __future__ import annotations

from typing import Any

import pytest

from gsa_taskflow_executor.gdk.control_probe import ARM_FRAME_NAMES
from gsa_taskflow_executor.gdk.end_effector_runtime import (
    ACTION_TASKFLOW_END_EFFECTOR,
    GDK_END_EFFECTOR_TYPE_MISMATCH,
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
        self.end_effector_joint_names: list[str] = []
        self.end_effector_joint_positions: list[float] = []


class FakeMotionStatus:
    def __init__(self) -> None:
        self.frame_names = [ARM_FRAME_NAMES["left_arm"], ARM_FRAME_NAMES["right_arm"]]
        self.frame_poses = [
            make_pose(0.5, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0),
            make_pose(0.5, -0.3, 1.0, 0.0, 0.0, 0.0, 1.0),
        ]


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


class FakeEndEffectorRobot:
    def __init__(self) -> None:
        self.end_state: dict[str, object] = {"left_tool": {"target_type": "omnipicker"}}
        self.after_end_state: dict[str, object] | None = None
        self.move_calls: list[FakeJointStates] = []
        self.servo_calls: list[FakeEndEffectorPose] = []
        self.motion_status = FakeMotionStatus()
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

    def get_motion_control_status(self) -> FakeMotionStatus:
        return self.motion_status

    def end_effector_pose_control(self, end_pose: FakeEndEffectorPose) -> int:
        self.servo_calls.append(end_pose)
        if self.after_end_state is not None:
            self.end_state = self.after_end_state
        return 0


class FakeAgibotGdk:
    robot = FakeEndEffectorRobot()
    init_called = 0
    release_called = 0
    JointState = FakeJointState
    JointStates = FakeJointStates
    Pose = FakePose
    EndEffectorPose = FakeEndEffectorPose

    class EndEffectorControlGroup:
        kLeftArm = 4
        kRightArm = 5
        kBothArms = 6

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


def test_end_effector_runtime_can_use_servo_gripper_path_after_abs_pose() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.end_state = {
        "right_tool": {"target_type": "omnipicker"},
        "right_end_state": {
            "names": ["right_gripper_joint"],
            "end_states": [{"position": 0.0}],
        },
    }
    sleep_calls: list[float] = []

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="right_tool",
            end_effector_type="omnipicker",
            opening=0.3,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
        sleep=sleep_calls.append,
        prefer_servo=True,
    )

    assert result["executed"] is True
    assert result["method"] == "end_effector_pose_control"
    assert result["control_method"] == "servo_gripper_hold_current_pose"
    assert result["controlled_arms"] == ["right_arm"]
    assert result["target_positions"] == pytest.approx([-0.2355])
    assert result["end_effector_joint_names"] == ["right_gripper_joint"]
    assert result["end_effector_joint_positions"] == pytest.approx([-0.2355])
    assert FakeAgibotGdk.robot.move_calls == []

    assert len(FakeAgibotGdk.robot.servo_calls) == (
        result["servo_step_count"] + result["servo_final_hold_steps"]
    )
    assert len(sleep_calls) == len(FakeAgibotGdk.robot.servo_calls)
    first_call = FakeAgibotGdk.robot.servo_calls[0]
    last_call = FakeAgibotGdk.robot.servo_calls[-1]
    assert first_call.group == FakeAgibotGdk.EndEffectorControlGroup.kRightArm
    assert first_call.end_effector_joint_positions == pytest.approx([0.0])
    assert last_call.end_effector_joint_names == ["right_gripper_joint"]
    assert last_call.end_effector_joint_positions == pytest.approx([-0.2355])


def test_servo_gripper_path_expands_single_side_target_to_all_reported_joints() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.end_state = {
        "right_tool": {"target_type": "omnipicker"},
        "right_end_state": {
            "names": ["right_gripper_joint_1", "right_gripper_joint_2"],
            "end_states": [{"position": 0.0}, {"position": 0.1}],
        },
    }

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="right_tool",
            end_effector_type="omnipicker",
            opening=1.0,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
        sleep=lambda _seconds: None,
        prefer_servo=True,
    )

    assert result["executed"] is True
    assert result["end_effector_joint_names"] == [
        "right_gripper_joint_1",
        "right_gripper_joint_2",
    ]
    assert result["target_positions"] == pytest.approx([-0.785, -0.785])
    assert FakeAgibotGdk.robot.servo_calls[-1].end_effector_joint_positions == pytest.approx(
        [-0.785, -0.785]
    )


def test_end_effector_runtime_executes_dual_tool_with_left_then_right_positions() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="dual_tool",
            end_effector_type="omnipicker",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
            left_opening=0.25,
            right_opening=0.75,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["target_end"] == "dual_tool"
    assert result["group"] == "dual_tool"
    assert result["target_positions"] == pytest.approx([-0.19625, -0.58875])
    assert result["positions_len"] == 2
    assert result["positions_layout"] == "left_tool_then_right_tool"
    assert result["actual_openness"] == pytest.approx([0.25, 0.75])
    assert result["left_opening"] == pytest.approx(0.25)
    assert result["right_opening"] == pytest.approx(0.75)

    [joint_states] = FakeAgibotGdk.robot.move_calls
    assert joint_states.group == "dual_tool"
    assert joint_states.target_type == "omnipicker"
    assert joint_states.nums == 2
    assert [state.position for state in joint_states.states] == pytest.approx(
        [-0.19625, -0.58875]
    )


def test_end_effector_runtime_refuses_dual_tool_type_mismatch() -> None:
    FakeAgibotGdk.reset()

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="dual_tool",
            end_effector_type=None,
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
            left_end_effector_type="omnipicker",
            right_end_effector_type="dahuan",
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is False
    assert result["error_code"] == GDK_END_EFFECTOR_TYPE_MISMATCH
    assert result["error_stage"] == "validate_end_effector_type"
    assert FakeAgibotGdk.robot.move_calls == []


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


def test_end_effector_runtime_prefers_dual_actual_openness_from_after_state() -> None:
    FakeAgibotGdk.reset()
    FakeAgibotGdk.robot.after_end_state = {
        "left_tool": {
            "target_type": "omnipicker",
            "actual_openness": [0.62],
        },
        "right_tool": {
            "target_type": "omnipicker",
            "actual_openness": [0.58],
        },
    }

    result = run_gdk_end_effector_control(
        EndEffectorParams(
            target_end="dual_tool",
            end_effector_type="omnipicker",
            opening=0.5,
            timeout=20.0,
            post_wait_seconds=0.0,
        ),
        environ=confirmed_env(),
        import_module=lambda _name: FakeAgibotGdk,
    )

    assert result["executed"] is True
    assert result["actual_openness"] == pytest.approx([0.62, 0.58])
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
