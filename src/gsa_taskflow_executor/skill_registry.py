from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

import yaml

from .config import ExecutorSettings
from .taskflow_parser import TaskflowDefinition, TaskflowParseError, parse_motion_plan_params

SkillAdapter = Literal["mock"]
MockSkillType = Literal["generic", "motion_plan"]
YamlMapping = Mapping[str, Any]


class SkillRegistryError(ValueError):
    """Raised when skill registry config is invalid or a skill is missing."""


@dataclass(frozen=True)
class SkillDefinition:
    name: str
    adapter: SkillAdapter
    mock_type: MockSkillType = "generic"
    description: str = ""
    raw: YamlMapping | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "adapter": self.adapter,
            "mock_type": self.mock_type,
            "description": self.description,
        }


@dataclass(frozen=True)
class SkillRegistry:
    skills: dict[str, SkillDefinition]
    source: str = "default"

    @classmethod
    def default(cls) -> SkillRegistry:
        return cls(
            skills={
                "motion_plan_skill": SkillDefinition(
                    name="motion_plan_skill",
                    adapter="mock",
                    mock_type="motion_plan",
                    description="Phase-1 mock motion planning skill.",
                )
            }
        )

    @classmethod
    def from_settings(cls, settings: ExecutorSettings) -> SkillRegistry:
        if settings.skill_registry_file:
            return cls.from_file(settings.skill_registry_file)
        return cls.default()

    @classmethod
    def from_file(cls, path: str | Path) -> SkillRegistry:
        registry_path = Path(path).expanduser()
        if not registry_path.exists():
            raise SkillRegistryError(f"Skill Registry 文件不存在: {registry_path}")
        if not registry_path.is_file():
            raise SkillRegistryError(f"Skill Registry 路径不是文件: {registry_path}")

        try:
            raw = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
        except yaml.YAMLError as error:
            raise SkillRegistryError(f"Skill Registry YAML 解析失败: {error}") from error
        return cls.from_mapping(raw, source=str(registry_path))

    @classmethod
    def from_mapping(cls, raw: Any, source: str = "memory") -> SkillRegistry:
        root = expect_mapping(raw, "root")
        raw_skills = expect_mapping(root.get("skills"), "root.skills")
        if not raw_skills:
            raise SkillRegistryError("root.skills 不能为空")

        skills: dict[str, SkillDefinition] = {}
        for raw_name, raw_config in raw_skills.items():
            if not isinstance(raw_name, str) or not raw_name.strip():
                raise SkillRegistryError("skills key 必须是非空字符串")
            name = raw_name.strip()
            if name in skills:
                raise SkillRegistryError(f"Skill 名称重复: {name}")
            skills[name] = parse_skill_definition(name, raw_config)

        return cls(skills=skills, source=source)

    def require(self, skill_name: str) -> SkillDefinition:
        skill = self.skills.get(skill_name)
        if skill is None:
            raise SkillRegistryError(f"未注册 skill_name: {skill_name}")
        return skill

    def validate_taskflow(self, definition: TaskflowDefinition) -> None:
        for node in definition.worker_nodes:
            if node.skill_name is None:
                raise SkillRegistryError(f"worker 节点缺少 skill_name: {node.node_id}")
            skill = self.require(node.skill_name)
            if skill.mock_type == "motion_plan":
                try:
                    parse_motion_plan_params(
                        node.params_template,
                        f"nodes[{node.node_id}].params_template",
                    )
                except TaskflowParseError as error:
                    raise SkillRegistryError(str(error)) from error

    def summary(self) -> dict[str, object]:
        return {
            "source": self.source,
            "skill_count": len(self.skills),
            "skills": [skill.to_dict() for skill in self.skills.values()],
        }


def parse_skill_definition(name: str, raw_config: Any) -> SkillDefinition:
    path = f"skills.{name}"
    config = expect_mapping(raw_config, path)
    adapter_value = read_required_string(config, "adapter", path)
    if adapter_value != "mock":
        raise SkillRegistryError(
            f"{path}.adapter 第一阶段只支持 mock，不支持 {adapter_value}"
        )

    mock_type_value = read_optional_string(
        config,
        ("mock_type", "mock", "implementation"),
        fallback="generic",
    )
    if mock_type_value not in {"generic", "motion_plan"}:
        raise SkillRegistryError(
            f"{path}.mock_type 第一阶段只支持 generic 或 motion_plan，当前为 {mock_type_value}"
        )

    return SkillDefinition(
        name=name,
        adapter="mock",
        mock_type=cast(MockSkillType, mock_type_value),
        description=read_optional_string(config, ("description",), fallback=""),
        raw=deepcopy(dict(config)),
    )


def expect_mapping(value: Any, path: str) -> YamlMapping:
    if not isinstance(value, Mapping):
        raise SkillRegistryError(f"{path} 必须是对象")
    return value


def read_required_string(mapping: YamlMapping, key: str, path: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise SkillRegistryError(f"{path}.{key} 必须是非空字符串")
    return value.strip()


def read_optional_string(
    mapping: YamlMapping,
    keys: tuple[str, ...],
    fallback: str,
) -> str:
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return fallback
