from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.gdk_readonly import run_gdk_readonly_probe


class FakeRobot:
    def get_joint_states(self) -> dict[str, object]:
        return {
            "timestamp": 4503286092744,
            "nums": 4,
            "states": [
                {
                    "name": "idx21_arm_l_joint1",
                    "position": 1.0,
                    "velocity": 0.0,
                    "effort": 0.0,
                    "error_code": 0,
                },
                {
                    "name": "idx22_arm_l_joint2",
                    "position": 2.0,
                    "velocity": 0.0,
                    "effort": 0.0,
                    "error_code": "0",
                },
                {
                    "name": "idx61_arm_r_joint1",
                    "position": 3.0,
                    "velocity": 0.0,
                    "effort": 0.0,
                    "error_code": 7,
                },
                {
                    "name": "idx62_arm_r_joint2",
                    "position": 4.0,
                    "velocity": 0.0,
                    "effort": 0.0,
                    "error_code": None,
                },
            ],
        }


class FakeAgibotGdk:
    Robot = FakeRobot


def test_gdk_readonly_probe_summarizes_joint_states() -> None:
    result = run_gdk_readonly_probe(lambda _name: FakeAgibotGdk)

    assert result["available"] is True
    assert result["backend"] == "agibot_gdk.Robot"
    assert result["joint_count"] == 4
    assert result["joint_names"] == [
        "idx21_arm_l_joint1",
        "idx22_arm_l_joint2",
        "idx61_arm_r_joint1",
        "idx62_arm_r_joint2",
    ]
    assert result["left_arm_joint_names"] == [
        "idx21_arm_l_joint1",
        "idx22_arm_l_joint2",
    ]
    assert result["right_arm_joint_names"] == [
        "idx61_arm_r_joint1",
        "idx62_arm_r_joint2",
    ]
    assert result["nonzero_error_joints"] == [
        {"name": "idx61_arm_r_joint1", "error_code": 7}
    ]
    assert result["raw"] == {"get_joint_states": FakeRobot().get_joint_states()}


def test_gdk_readonly_probe_gracefully_handles_missing_module() -> None:
    def missing_importer(_name: str) -> Any:
        raise ModuleNotFoundError("No module named agibot_gdk")

    result = run_gdk_readonly_probe(missing_importer)

    assert result["available"] is False
    assert result["backend"] == "agibot_gdk.Robot"
    assert result["joint_count"] == 0
    assert result["joint_names"] == []
    assert result["raw"] == {}
    assert result["error_stage"] == "import_agibot_gdk"
    assert result["error_type"] == "ModuleNotFoundError"


def test_gdk_readonly_probe_gracefully_handles_bad_joint_payload() -> None:
    class BadRobot:
        def get_joint_states(self) -> list[object]:
            return []

    class BadAgibotGdk:
        Robot = BadRobot

    result = run_gdk_readonly_probe(lambda _name: BadAgibotGdk)

    assert result["available"] is False
    assert result["error_stage"] == "parse_joint_states"
    assert result["error_type"] == "TypeError"
