from __future__ import annotations

import pytest

from fixtures import VALID_RIGHT_ARM_YAML
from gsa_taskflow_executor.config import ExecutorSettings
from gsa_taskflow_executor.skill_registry import (
    SkillRegistry,
    SkillRegistryError,
)
from gsa_taskflow_executor.taskflow_parser import parse_taskflow_yaml


def test_default_registry_contains_motion_plan_mock() -> None:
    registry = SkillRegistry.default()

    skill = registry.require("motion_plan_skill")

    assert skill.adapter == "mock"
    assert skill.mock_type == "motion_plan"
    assert registry.summary()["skill_count"] == 1


def test_registry_loads_from_mapping() -> None:
    registry = SkillRegistry.from_mapping(
        {
            "skills": {
                "qr_detect_skill": {
                    "adapter": "mock",
                    "mock_type": "generic",
                    "description": "mock qr detection",
                }
            }
        }
    )

    skill = registry.require("qr_detect_skill")

    assert skill.adapter == "mock"
    assert skill.mock_type == "generic"
    assert skill.description == "mock qr detection"


def test_registry_loads_from_settings_file(tmp_path) -> None:
    registry_file = tmp_path / "skills.yaml"
    registry_file.write_text(
        """
skills:
  motion_plan_skill:
    adapter: mock
    mock_type: motion_plan
""",
        encoding="utf-8",
    )

    registry = SkillRegistry.from_settings(
        ExecutorSettings(skill_registry_file=str(registry_file))
    )

    assert registry.source == str(registry_file)
    assert registry.require("motion_plan_skill").mock_type == "motion_plan"


def test_registry_rejects_gdk_adapter_in_phase_1() -> None:
    with pytest.raises(SkillRegistryError, match="只支持 mock"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "motion_plan_skill": {
                        "adapter": "gdk",
                    }
                }
            }
        )


def test_registry_rejects_unregistered_taskflow_skill() -> None:
    taskflow = parse_taskflow_yaml(
        VALID_RIGHT_ARM_YAML.replace("motion_plan_skill", "unknown_skill")
    )
    registry = SkillRegistry.default()

    with pytest.raises(SkillRegistryError, match="未注册 skill_name"):
        registry.validate_taskflow(taskflow)


def test_registry_validates_motion_plan_alias_params() -> None:
    yaml_payload = VALID_RIGHT_ARM_YAML.replace("motion_plan_skill", "custom_motion")
    taskflow = parse_taskflow_yaml(yaml_payload)
    registry = SkillRegistry.from_mapping(
        {
            "skills": {
                "custom_motion": {
                    "adapter": "mock",
                    "mock_type": "motion_plan",
                }
            }
        }
    )

    registry.validate_taskflow(taskflow)
