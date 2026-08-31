from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.gdk.readonly import run_gdk_env_check, run_gdk_readonly_probe


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


def test_gdk_env_check_imports_module_without_creating_robot() -> None:
    env = {
        "PYTHONPATH": "/opt/agibot/python",
        "LD_LIBRARY_PATH": "/opt/agibot/lib",
        "CYCLONEDDS_URI": "file:///etc/agibot/cyclonedds.xml",
        "UNRELATED": "value",
    }

    result = run_gdk_env_check(lambda _name: FakeAgibotGdk, environ=env)

    assert result["available"] is True
    assert result["backend"] == "agibot_gdk"
    assert result["environment"] == {
        "pythonpath_configured": True,
        "ld_library_path_configured": True,
        "gdk_related_keys": ["CYCLONEDDS_URI", "LD_LIBRARY_PATH", "PYTHONPATH"],
    }
    assert result["module"] == {
        "required_attributes": ["Robot"],
        "has_robot": True,
        "has_gdk_init": False,
    }


def test_gdk_env_check_returns_error_when_import_fails() -> None:
    def missing_importer(_name: str) -> Any:
        raise ModuleNotFoundError("No module named agibot_gdk")

    result = run_gdk_env_check(missing_importer, environ={})

    assert result["available"] is False
    assert result["backend"] == "agibot_gdk"
    assert result["error_stage"] == "import_agibot_gdk"
    assert result["error_type"] == "ModuleNotFoundError"
    assert result["environment"] == {
        "pythonpath_configured": False,
        "ld_library_path_configured": False,
        "gdk_related_keys": [],
    }


def test_gdk_env_check_rejects_module_without_robot_factory() -> None:
    class BrokenAgibotGdk:
        pass

    result = run_gdk_env_check(lambda _name: BrokenAgibotGdk, environ={})

    assert result["available"] is False
    assert result["error_stage"] == "validate_agibot_gdk_module"
    assert result["missing_attributes"] == ["Robot"]


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
