from __future__ import annotations

import pytest

from fixtures import (
    VALID_CODE_AND_MOTION_YAML,
    VALID_END_EFFECTOR_YAML,
    VALID_FORCE_CONTROL_YAML,
    VALID_RIGHT_ARM_YAML,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.skills.registry import (
    SkillRegistry,
    SkillRegistryError,
)
from gsa_taskflow_executor.taskflow.capabilities import EXECUTOR_SKILL_IMPLEMENTATIONS
from gsa_taskflow_executor.taskflow.parser import parse_taskflow_yaml


def test_default_registry_contains_gdk_motion_plan() -> None:
    registry = SkillRegistry.default()

    skill = registry.require("motion_plan_skill")
    script_skill = registry.require("script_skill")
    end_effector_skill = registry.require("control_end_effector_skill")
    force_control_skill = registry.require("force_control_skill")
    qr_pose_skill = registry.require("qr_pose_skill")

    assert skill.adapter == "gdk"
    assert skill.implementation == "motion_plan"
    assert script_skill.adapter == "gdk"
    assert script_skill.implementation == "script"
    assert end_effector_skill.adapter == "gdk"
    assert end_effector_skill.implementation == "end_effector"
    assert force_control_skill.adapter == "gdk"
    assert force_control_skill.implementation == "force_control"
    assert qr_pose_skill.adapter == "gdk"
    assert qr_pose_skill.implementation == "qr_pose"
    assert registry.summary()["skill_count"] == 5


def test_default_registry_is_derived_from_executor_capabilities() -> None:
    registry = SkillRegistry.default()

    assert set(registry.skills) == set(EXECUTOR_SKILL_IMPLEMENTATIONS)
    for skill_name, implementation in EXECUTOR_SKILL_IMPLEMENTATIONS.items():
        assert registry.require(skill_name).implementation == implementation


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
                },
                "control_end_effector_skill": {
                    "adapter": "gdk",
                    "implementation": "end_effector",
                    "description": "guarded gdk end-effector skill",
                },
                "force_control_skill": {
                    "adapter": "gdk",
                    "implementation": "force_control",
                    "description": "blocked gdk force-control skill",
                },
                "qr_pose_skill": {
                    "adapter": "gdk",
                    "implementation": "qr_pose",
                    "description": "read-only qr pose skill",
                },
            }
        }
    )

    skill = registry.require("motion_plan_skill")
    script_skill = registry.require("script_skill")
    end_effector_skill = registry.require("control_end_effector_skill")
    force_control_skill = registry.require("force_control_skill")
    qr_pose_skill = registry.require("qr_pose_skill")

    assert skill.adapter == "gdk"
    assert skill.implementation == "motion_plan"
    assert skill.description == "verified gdk motion skill"
    assert script_skill.adapter == "gdk"
    assert script_skill.implementation == "script"
    assert script_skill.description == "guarded gdk script skill"
    assert end_effector_skill.adapter == "gdk"
    assert end_effector_skill.implementation == "end_effector"
    assert end_effector_skill.description == "guarded gdk end-effector skill"
    assert force_control_skill.adapter == "gdk"
    assert force_control_skill.implementation == "force_control"
    assert force_control_skill.description == "blocked gdk force-control skill"
    assert qr_pose_skill.adapter == "gdk"
    assert qr_pose_skill.implementation == "qr_pose"
    assert qr_pose_skill.description == "read-only qr pose skill"


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
                },
                "control_end_effector_skill": {
                    "adapter": "gdk",
                },
                "force_control_skill": {
                    "adapter": "gdk",
                },
                "qr_pose_skill": {
                    "adapter": "gdk",
                },
            }
        }
    )

    skill = registry.require("motion_plan_skill")
    script_skill = registry.require("script_skill")
    end_effector_skill = registry.require("control_end_effector_skill")
    force_control_skill = registry.require("force_control_skill")
    qr_pose_skill = registry.require("qr_pose_skill")

    assert skill.adapter == "gdk"
    assert skill.implementation == "motion_plan"
    assert script_skill.adapter == "gdk"
    assert script_skill.implementation == "script"
    assert end_effector_skill.adapter == "gdk"
    assert end_effector_skill.implementation == "end_effector"
    assert force_control_skill.adapter == "gdk"
    assert force_control_skill.implementation == "force_control"
    assert qr_pose_skill.adapter == "gdk"
    assert qr_pose_skill.implementation == "qr_pose"


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


def test_registry_rejects_non_end_effector_implementation() -> None:
    with pytest.raises(SkillRegistryError, match="implementation 只支持 end_effector"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "control_end_effector_skill": {
                        "adapter": "gdk",
                        "implementation": "motion_plan",
                    }
                }
            }
        )


def test_registry_rejects_non_force_control_implementation() -> None:
    with pytest.raises(SkillRegistryError, match="implementation 只支持 force_control"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "force_control_skill": {
                        "adapter": "gdk",
                        "implementation": "motion_plan",
                    }
                }
            }
        )


def test_registry_rejects_non_qr_pose_implementation() -> None:
    with pytest.raises(SkillRegistryError, match="implementation 只支持 qr_pose"):
        SkillRegistry.from_mapping(
            {
                "skills": {
                    "qr_pose_skill": {
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
        match=(
            "MVP 当前只支持 motion_plan_skill、script_skill、"
            "control_end_effector_skill、force_control_skill 和 qr_pose_skill"
        ),
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
    taskflow = parse_taskflow_yaml(VALID_CODE_AND_MOTION_YAML)
    registry = SkillRegistry.default()

    registry.validate_taskflow(taskflow)


def test_registry_validates_end_effector_skill_params() -> None:
    taskflow = parse_taskflow_yaml(VALID_END_EFFECTOR_YAML)
    registry = SkillRegistry.default()

    registry.validate_taskflow(taskflow)


def test_registry_validates_force_control_skill_params() -> None:
    taskflow = parse_taskflow_yaml(VALID_FORCE_CONTROL_YAML)
    registry = SkillRegistry.default()

    registry.validate_taskflow(taskflow)
