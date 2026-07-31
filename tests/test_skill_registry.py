from __future__ import annotations

import pytest

from fixtures import VALID_RIGHT_ARM_YAML, VALID_SCRIPT_AND_MOTION_YAML
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.skills.registry import (
    SkillRegistry,
    SkillRegistryError,
)
from gsa_taskflow_executor.taskflow.parser import parse_taskflow_yaml


def test_default_registry_contains_gdk_motion_plan() -> None:
    registry = SkillRegistry.default()

    skill = registry.require("motion_plan_skill")
    script_skill = registry.require("script_skill")

    assert skill.adapter == "gdk"
    assert skill.implementation == "motion_plan"
    assert script_skill.adapter == "gdk"
    assert script_skill.implementation == "script"
    assert registry.summary()["skill_count"] == 2


def test_registry_loads_from_mapping() -> None:
    registry = SkillRegistry.from_mapping(
        {
            "skills": {
                "motion_plan_skill": {
                    "adapter": "gdk",
                    "implementation": "motion_plan",
                        "description": "verified gdk motion skill",
                },
                "script_skill": {
                    "adapter": "gdk",
                    "implementation": "script",
                    "description": "guarded gdk script skill",
                }
            }
        }
    )

    skill = registry.require("motion_plan_skill")
    script_skill = registry.require("script_skill")

    assert skill.adapter == "gdk"
    assert skill.implementation == "motion_plan"
    assert skill.description == "verified gdk motion skill"
    assert script_skill.adapter == "gdk"
    assert script_skill.implementation == "script"
    assert script_skill.description == "guarded gdk script skill"


def test_registry_loads_from_settings_file(tmp_path) -> None:
    registry_file = tmp_path / "skills.yaml"
    registry_file.write_text(
        """
skills:
  motion_plan_skill:
    adapter: gdk
    implementation: motion_plan
""",
        encoding="utf-8",
    )

    registry = SkillRegistry.from_settings(
        ExecutorSettings(skill_registry_file=str(registry_file))
    )

    assert registry.source == str(registry_file)
    assert registry.require("motion_plan_skill").implementation == "motion_plan"


def test_registry_accepts_gdk_motion_plan_adapter() -> None:
    registry = SkillRegistry.from_mapping(
        {
            "skills": {
                "motion_plan_skill": {
                    "adapter": "gdk",
                },
                "script_skill": {
                    "adapter": "gdk",
                }
            }
        }
    )

    skill = registry.require("motion_plan_skill")
    script_skill = registry.require("script_skill")

    assert skill.adapter == "gdk"
    assert skill.implementation == "motion_plan"
    assert script_skill.adapter == "gdk"
    assert script_skill.implementation == "script"


def test_registry_rejects_non_motion_plan_implementation() -> None:
    with pytest.raises(SkillRegistryError, match="implementation 只支持 motion_plan"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "motion_plan_skill": {
                        "adapter": "gdk",
                        "implementation": "generic",
                    }
                }
            }
        )


def test_registry_rejects_non_script_implementation() -> None:
    with pytest.raises(SkillRegistryError, match="implementation 只支持 script"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "script_skill": {
                        "adapter": "gdk",
                        "implementation": "motion_plan",
                    }
                }
            }
        )


def test_registry_rejects_mock_adapter() -> None:
    with pytest.raises(SkillRegistryError, match="adapter 只支持 gdk"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "motion_plan_skill": {
                        "adapter": "mock",
                    }
                }
            }
        )


def test_registry_rejects_non_mvp_skill() -> None:
    with pytest.raises(
        SkillRegistryError,
        match="MVP 当前只支持 motion_plan_skill 和 script_skill",
    ):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "qr_detect_skill": {
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
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    registry = SkillRegistry.default()

    registry.validate_taskflow(taskflow)


def test_registry_validates_script_skill_params() -> None:
    taskflow = parse_taskflow_yaml(VALID_SCRIPT_AND_MOTION_YAML)
    registry = SkillRegistry.default()

    registry.validate_taskflow(taskflow)
