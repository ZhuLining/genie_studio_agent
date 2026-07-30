from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Literal, Protocol

from .gdk_motion_runtime import run_gdk_motion_plan_abs_joint
from .skill_registry import SkillDefinition, SkillRegistry, SkillRegistryError
from .taskflow_parser import (
    MotionPlanParams,
    TaskflowNode,
    TaskflowParseError,
    parse_motion_plan_params,
)
from .variable_store import VariableStore, VariableStoreError

SkillOutcome = Literal["success", "error"]


class SkillRuntimeError(RuntimeError):
    """Raised when a node cannot be executed by the skill runtime."""


@dataclass(frozen=True)
class SkillExecutionContext:
    app_execution_id: str
    variable_store: VariableStore
    mode: str = "gdk"


@dataclass(frozen=True)
class SkillResult:
    outcome: SkillOutcome = "success"
    detail: dict[str, object] | None = None
    outputs: dict[str, object] | None = None


class Skill(Protocol):
    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        """Execute one parsed taskflow node."""


class AssignSkill:
    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        if node.node_type != "assign":
            raise SkillRuntimeError(f"assign runtime 收到非 assign 节点: {node.node_id}")

        assignments = context.variable_store.resolve_value(node.assignments)
        if not isinstance(assignments, Mapping):
            raise SkillRuntimeError(f"assign 节点 assignments 解析后不是对象: {node.node_id}")

        return SkillResult(
            outcome="success",
            detail={
                "mode": context.mode,
                "assignments": deepcopy(dict(assignments)),
            },
            outputs={"assignments": deepcopy(dict(assignments))},
        )


class MotionPlanSkillGdk:
    def __init__(self, environ: Mapping[str, str] | None = None) -> None:
        self.environ = environ

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        resolved_params = context.variable_store.resolve_value(node.params_template)
        if not isinstance(resolved_params, Mapping):
            raise SkillRuntimeError(f"{node.node_id}.params_template 解析后不是对象")

        try:
            motion_params = parse_motion_plan_params(resolved_params, "params_template")
        except TaskflowParseError as error:
            raise SkillRuntimeError(str(error)) from error

        if self.environ is None:
            gdk_result = run_gdk_motion_plan_abs_joint(motion_params)
        else:
            gdk_result = run_gdk_motion_plan_abs_joint(
                motion_params,
                environ=self.environ,
            )
        if gdk_result.get("executed") is not True:
            error_msg = gdk_result.get("error_msg")
            raise SkillRuntimeError(str(error_msg or "GDK ABS_JOINT 执行失败"))

        outputs = build_motion_plan_outputs(
            app_execution_id=context.app_execution_id,
            skill_name=node.skill_name,
            mode=context.mode,
            params_template=resolved_params,
            motion_params=motion_params,
        )
        outputs["gdk_result"] = deepcopy(gdk_result)
        return SkillResult(
            outcome="success",
            detail={
                "skill_name": node.skill_name,
                "mode": context.mode,
                "adapter": "gdk",
                "params_template": deepcopy(dict(resolved_params)),
                "gdk_result": deepcopy(gdk_result),
            },
            outputs=outputs,
        )


class SkillRuntime:
    def __init__(
        self,
        registry: SkillRegistry | None = None,
        environ: Mapping[str, str] | None = None,
    ) -> None:
        self.registry = registry or SkillRegistry.default()
        self.assign_skill = AssignSkill()
        self.gdk_skills: dict[str, Skill] = {
            "motion_plan": MotionPlanSkillGdk(environ=environ),
        }

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        if node.node_type == "assign":
            return self.assign_skill.run(node, context)
        if node.node_type != "worker":
            raise SkillRuntimeError(f"不支持的节点类型: {node.node_type}")
        if node.skill_name is None:
            raise SkillRuntimeError(f"worker 节点缺少 skill_name: {node.node_id}")

        try:
            skill_definition = self.registry.require(node.skill_name)
            skill = self.resolve_skill(skill_definition)
            return skill.run(node, context)
        except (SkillRegistryError, VariableStoreError) as error:
            raise SkillRuntimeError(str(error)) from error

    def resolve_skill(self, skill_definition: SkillDefinition) -> Skill:
        if skill_definition.adapter == "gdk":
            return self.resolve_gdk_skill(skill_definition)
        raise SkillRuntimeError(f"不支持的 skill adapter: {skill_definition.adapter}")

    def resolve_gdk_skill(self, skill_definition: SkillDefinition) -> Skill:
        skill = self.gdk_skills.get(skill_definition.implementation)
        if skill is None:
            raise SkillRuntimeError(f"不支持的 GDK skill 类型: {skill_definition.implementation}")
        return skill


def build_motion_plan_outputs(
    *,
    app_execution_id: str,
    skill_name: str | None,
    mode: str,
    params_template: Mapping[str, Any],
    motion_params: MotionPlanParams,
) -> dict[str, object]:
    motion_targets = [
        {
            "body_part": target.body_part,
            "control_type": target.control_type,
            "action_data": deepcopy(target.action_data),
        }
        for target in motion_params.targets
    ]
    primary_target = motion_params.targets[0]

    return {
        "app_execution_id": app_execution_id,
        "skill_name": skill_name,
        "mode": mode,
        "motion_targets": motion_targets,
        "primary_body_part": primary_target.body_part,
        "primary_control_type": primary_target.control_type,
        "final_joint": deepcopy(primary_target.action_data),
        "speed": motion_params.speed,
        "requested_speed": motion_params.speed,
        "requested_speed_unit": "gdk_velocity",
        "timeout": motion_params.timeout,
        "resolved_params_template": deepcopy(dict(params_template)),
    }
