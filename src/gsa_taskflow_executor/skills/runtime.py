from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
from math import isfinite
from typing import Any, Literal, Protocol

from gsa_taskflow_executor.code_scripts.runtime import run_code_script
from gsa_taskflow_executor.gdk.end_effector_runtime import run_gdk_end_effector_control
from gsa_taskflow_executor.gdk.force_control_runtime import run_gdk_force_control_unverified
from gsa_taskflow_executor.gdk.motion_runtime import run_gdk_motion_plan_abs_joint
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.skills.registry import (
    SkillDefinition,
    SkillRegistry,
    SkillRegistryError,
)
from gsa_taskflow_executor.taskflow.parser import (
    EndEffectorParams,
    ForceControlParams,
    MotionPlanParams,
    ScriptInputMapping,
    ScriptOutputVariable,
    ScriptParams,
    TaskflowNode,
    TaskflowParseError,
    parse_end_effector_params,
    parse_force_control_params,
    parse_motion_plan_params,
    parse_script_params,
)
from gsa_taskflow_executor.taskflow.variables import VariableStore, VariableStoreError

SkillOutcome = Literal["success", "error"]


class SkillRuntimeError(RuntimeError):
    """Raised when a node cannot be executed by the skill runtime."""

    def __init__(
        self,
        message: str,
        *,
        detail: Mapping[str, object] | None = None,
    ) -> None:
        super().__init__(message)
        self.detail = dict(detail) if detail is not None else None


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
    def __init__(
        self,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.environ = environ
        self.gdk_session_manager = gdk_session_manager

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        resolved_params = context.variable_store.resolve_value(node.params_template)
        if not isinstance(resolved_params, Mapping):
            raise SkillRuntimeError(f"{node.node_id}.params_template 解析后不是对象")

        try:
            motion_params = parse_motion_plan_params(resolved_params, "params_template")
        except TaskflowParseError as error:
            raise SkillRuntimeError(str(error)) from error

        gdk_result = run_gdk_motion_plan_abs_joint(
            motion_params,
            environ=self.environ,
            session_manager=self.gdk_session_manager,
        )
        if gdk_result.get("executed") is not True:
            error_msg = gdk_result.get("error_msg")
            message = str(error_msg or "GDK ABS_JOINT 执行失败")
            raise SkillRuntimeError(
                message,
                detail=build_gdk_error_detail(message, gdk_result),
            )

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


class ScriptSkillGdk:
    def __init__(
        self,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.environ = environ
        self.gdk_session_manager = gdk_session_manager

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        # 输入映射本身是变量引用声明，不能像运动参数一样整体 resolve；
        # 只在执行前解析 mapping.variable_ref，避免把契约字段改成运行值。
        raw_params = node.params_template
        if not isinstance(raw_params, Mapping):
            raise SkillRuntimeError(f"{node.node_id}.params_template 解析后不是对象")

        try:
            script_params = parse_script_params(raw_params, "params_template")
        except TaskflowParseError as error:
            raise SkillRuntimeError(str(error)) from error

        inputs = resolve_script_inputs(
            script_params.input_mappings,
            context.variable_store,
            node_id=node.node_id,
        )
        script_result = run_code_script(
            script_params,
            environ=self.environ,
            session_manager=self.gdk_session_manager,
            inputs=inputs,
        )
        if script_result.get("executed") is not True:
            error_msg = script_result.get("error_msg")
            message = str(error_msg or "GDK script 执行失败")
            raise SkillRuntimeError(
                message,
                detail=build_gdk_error_detail(message, script_result),
            )

        declared_outputs = extract_declared_script_outputs(
            script_params.output_variables,
            script_result,
            node_id=node.node_id,
        )
        outputs = build_script_outputs(
            app_execution_id=context.app_execution_id,
            skill_name=node.skill_name,
            mode=context.mode,
            params_template=raw_params,
            script_params=script_params,
            inputs=inputs,
        )
        outputs.update(declared_outputs)
        outputs["script_result"] = deepcopy(script_result)
        return SkillResult(
            outcome="success",
            detail={
                "skill_name": node.skill_name,
                "mode": context.mode,
                "adapter": "gdk",
                "params_template": deepcopy(dict(raw_params)),
                "inputs": deepcopy(inputs),
                "declared_outputs": deepcopy(declared_outputs),
                "script_result": deepcopy(script_result),
            },
            outputs=outputs,
        )


class EndEffectorSkillGdk:
    def __init__(
        self,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.environ = environ
        self.gdk_session_manager = gdk_session_manager

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        resolved_params = context.variable_store.resolve_value(node.params_template)
        if not isinstance(resolved_params, Mapping):
            raise SkillRuntimeError(f"{node.node_id}.params_template 解析后不是对象")

        try:
            end_effector_params = parse_end_effector_params(resolved_params, "params_template")
        except TaskflowParseError as error:
            raise SkillRuntimeError(str(error)) from error

        end_effector_result = run_gdk_end_effector_control(
            end_effector_params,
            environ=self.environ,
            session_manager=self.gdk_session_manager,
        )
        if end_effector_result.get("executed") is not True:
            error_msg = end_effector_result.get("error_msg")
            message = str(error_msg or "GDK 末端控制执行失败")
            raise SkillRuntimeError(
                message,
                detail=build_gdk_error_detail(message, end_effector_result),
            )

        outputs = build_end_effector_outputs(
            app_execution_id=context.app_execution_id,
            skill_name=node.skill_name,
            mode=context.mode,
            params_template=resolved_params,
            end_effector_params=end_effector_params,
        )
        actual_openness = end_effector_result.get("actual_openness")
        if not isinstance(actual_openness, list):
            actual_openness = [end_effector_params.opening]
        outputs["actual_openness"] = deepcopy(actual_openness)
        outputs["actual_openness_source"] = str(
            end_effector_result.get("actual_openness_source", "requested_opening_fallback")
        )
        resolved_end_effector_type = end_effector_result.get("end_effector_type")
        outputs["end_effector_type"] = (
            resolved_end_effector_type
            if isinstance(resolved_end_effector_type, str)
            else end_effector_params.end_effector_type or ""
        )
        outputs["end_effector_result"] = deepcopy(end_effector_result)
        return SkillResult(
            outcome="success",
            detail={
                "skill_name": node.skill_name,
                "mode": context.mode,
                "adapter": "gdk",
                "params_template": deepcopy(dict(resolved_params)),
                "end_effector_result": deepcopy(end_effector_result),
            },
            outputs=outputs,
        )


class ForceControlSkillGdk:
    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        resolved_params = context.variable_store.resolve_value(node.params_template)
        if not isinstance(resolved_params, Mapping):
            raise SkillRuntimeError(f"{node.node_id}.params_template 解析后不是对象")

        try:
            force_params = parse_force_control_params(resolved_params, "params_template")
        except TaskflowParseError as error:
            raise SkillRuntimeError(str(error)) from error

        force_control_result = run_gdk_force_control_unverified(force_params)
        error_msg = force_control_result.get("error_msg")
        message = str(error_msg or "GDK 力控执行未开放")
        raise SkillRuntimeError(
            message,
            detail=build_gdk_error_detail(message, force_control_result),
        )


class SkillRuntime:
    def __init__(
        self,
        registry: SkillRegistry | None = None,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.registry = registry or SkillRegistry.default()
        self.assign_skill = AssignSkill()
        self.gdk_skills: dict[str, Skill] = {
            "motion_plan": MotionPlanSkillGdk(
                environ=environ,
                gdk_session_manager=gdk_session_manager,
            ),
            "script": ScriptSkillGdk(
                environ=environ,
                gdk_session_manager=gdk_session_manager,
            ),
            "end_effector": EndEffectorSkillGdk(
                environ=environ,
                gdk_session_manager=gdk_session_manager,
            ),
            "force_control": ForceControlSkillGdk(),
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


def build_gdk_error_detail(
    message: str,
    gdk_result: Mapping[str, object],
) -> dict[str, object]:
    detail: dict[str, object] = {
        "error": message,
        "gdk_result": deepcopy(dict(gdk_result)),
    }
    error_code = gdk_result.get("error_code")
    if isinstance(error_code, str) and error_code:
        detail["error_code"] = error_code
    error_stage = gdk_result.get("error_stage")
    if isinstance(error_stage, str) and error_stage:
        detail["error_stage"] = error_stage
    return detail


def resolve_script_inputs(
    input_mappings: Sequence[ScriptInputMapping],
    variable_store: VariableStore,
    *,
    node_id: str,
) -> dict[str, object]:
    inputs: dict[str, object] = {}
    for mapping in input_mappings:
        value = variable_store.resolve(mapping.variable_ref)
        if not value_matches_script_type(value, mapping.value_type):
            raise SkillRuntimeError(
                f"代码节点 {node_id} 的输入参数 {mapping.name} 类型不匹配",
                detail={
                    "error_stage": "resolve_input_mappings",
                    "input_name": mapping.name,
                    "expected_type": mapping.value_type,
                    "variable_ref": mapping.variable_ref,
                    "actual_type": type(value).__name__,
                },
            )
        inputs[mapping.name] = value
    return inputs


def extract_declared_script_outputs(
    output_variables: Sequence[ScriptOutputVariable],
    script_result: Mapping[str, object],
    *,
    node_id: str,
) -> dict[str, object]:
    if not output_variables:
        return {}

    raw_outputs = script_result.get("outputs")
    if not isinstance(raw_outputs, Mapping):
        raise SkillRuntimeError(
            f"代码节点 {node_id} 声明了输出变量，但执行结果未返回 outputs",
            detail={
                "error_stage": "validate_declared_outputs",
                "script_result": deepcopy(dict(script_result)),
            },
        )

    outputs: dict[str, object] = {}
    for output_variable in output_variables:
        if output_variable.name not in raw_outputs:
            raise SkillRuntimeError(
                f"代码节点 {node_id} 缺少声明输出: {output_variable.name}",
                detail={
                    "error_stage": "validate_declared_outputs",
                    "output_name": output_variable.name,
                    "expected_type": output_variable.value_type,
                    "script_result": deepcopy(dict(script_result)),
                },
            )
        value = raw_outputs[output_variable.name]
        if not value_matches_script_type(value, output_variable.value_type):
            raise SkillRuntimeError(
                f"代码节点 {node_id} 的输出变量 {output_variable.name} 类型不匹配",
                detail={
                    "error_stage": "validate_declared_outputs",
                    "output_name": output_variable.name,
                    "expected_type": output_variable.value_type,
                    "actual_type": type(value).__name__,
                    "script_result": deepcopy(dict(script_result)),
                },
            )
        outputs[output_variable.name] = deepcopy(value)
    return outputs


def value_matches_script_type(value: object, expected_type: str) -> bool:
    if expected_type == "string":
        return isinstance(value, str)
    if expected_type == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected_type == "number":
        return isinstance(value, int | float) and not isinstance(value, bool) and isfinite(value)
    if expected_type == "boolean":
        return isinstance(value, bool)
    if expected_type == "time":
        return isinstance(value, str)
    if expected_type == "object":
        return isinstance(value, Mapping)
    if expected_type == "array":
        return isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray)
    return False


def build_script_outputs(
    *,
    app_execution_id: str,
    skill_name: str | None,
    mode: str,
    params_template: Mapping[str, Any],
    script_params: ScriptParams,
    inputs: Mapping[str, object],
) -> dict[str, object]:
    return {
        "app_execution_id": app_execution_id,
        "skill_name": skill_name,
        "mode": mode,
        "script_id": script_params.script_id,
        "timeout": script_params.timeout,
        "input_mappings": [
            {
                "name": mapping.name,
                "type": mapping.value_type,
                "variable_ref": mapping.variable_ref,
            }
            for mapping in script_params.input_mappings
        ],
        "output_variables": [
            {
                "name": output.name,
                "type": output.value_type,
            }
            for output in script_params.output_variables
        ],
        "inputs": deepcopy(dict(inputs)),
        "resolved_params_template": deepcopy(dict(params_template)),
    }


def build_end_effector_outputs(
    *,
    app_execution_id: str,
    skill_name: str | None,
    mode: str,
    params_template: Mapping[str, Any],
    end_effector_params: EndEffectorParams,
) -> dict[str, object]:
    return {
        "app_execution_id": app_execution_id,
        "skill_name": skill_name,
        "mode": mode,
        "target_end": end_effector_params.target_end,
        "end_effector_type": end_effector_params.end_effector_type,
        "opening": end_effector_params.opening,
        "timeout": end_effector_params.timeout,
        "post_wait_seconds": end_effector_params.post_wait_seconds,
        "resolved_params_template": deepcopy(dict(params_template)),
    }


def build_force_control_outputs(
    *,
    app_execution_id: str,
    skill_name: str | None,
    mode: str,
    params_template: Mapping[str, Any],
    force_params: ForceControlParams,
) -> dict[str, object]:
    return {
        "app_execution_id": app_execution_id,
        "skill_name": skill_name,
        "mode": mode,
        "method": force_params.method,
        "arm": force_params.arm,
        "delta_xyz": deepcopy(list(force_params.delta_xyz)),
        "force_threshold": force_params.force_threshold,
        "timeout_s": force_params.timeout_s,
        "control_hz": force_params.control_hz,
        "step": force_params.step,
        "resolved_params_template": deepcopy(dict(params_template)),
    }
