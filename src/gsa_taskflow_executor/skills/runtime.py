"""技能运行时 — 将 taskflow 节点分发到 GDK 技能实现。

SkillRuntime.run(node, context) 分发逻辑::

    assign → AssignSkill（写入 VariableStore）
    worker → SkillRegistry.require(skill_name) → GDK skill
      ├─ motion_plan  → MotionPlanSkillGdk  → run_gdk_motion_plan_abs_joint()
      ├─ script       → ScriptSkillGdk       → run_code_script()
      ├─ end_effector → EndEffectorSkillGdk  → run_gdk_end_effector_control()
      └─ force_control→ ForceControlSkillGdk → 硬阻断

变量解析策略:
- motion/end_effector: 整体 resolve params_template（$.variables.* → 实际值）
- script: 只逐个 resolve input_mappings.variable_ref，保留 schema 结构
"""

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
from gsa_taskflow_executor.taskflow.models import (
    EndEffectorParams,
    ForceControlParams,
    MotionPlanParams,
    ScriptInputMapping,
    ScriptOutputVariable,
    ScriptParams,
    TaskflowNode,
    TaskflowParseError,
)
from gsa_taskflow_executor.taskflow.skill_params import (
    parse_end_effector_params,
    parse_force_control_params,
    parse_motion_plan_params,
    parse_script_params,
)
from gsa_taskflow_executor.taskflow.variables import VariableStore, VariableStoreError

SkillOutcome = Literal["success", "error"]


class SkillRuntimeError(RuntimeError):
    """技能执行失败。

    detail 携带 error_code/error_stage/gdk_result，用于分类 error/cancelled。
    """

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
    """技能执行上下文。"""
    app_execution_id: str
    variable_store: VariableStore
    mode: str = "gdk"


@dataclass(frozen=True)
class SkillResult:
    """技能执行结果。"""
    outcome: SkillOutcome = "success"
    detail: dict[str, object] | None = None
    outputs: dict[str, object] | None = None


class Skill(Protocol):
    """技能接口。由各 GDK skill runtime 实现。"""

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        ...


# ============================================================
# 技能实现
# ============================================================


class AssignSkill:
    """assign 节点技能 — 将 assignments 写入 VariableStore。无 GDK 交互。"""

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
    """motion_plan_skill — ABS_JOINT 运动规划。

    需要 environ（安全门 ENV）和 gdk_session_manager（GDK 生命周期）。
    """

    def __init__(
        self,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.environ = environ
        self.gdk_session_manager = gdk_session_manager

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        """流程: resolve 变量引用 → 解析 MotionPlanParams → 调用 GDK → 构建 outputs。"""
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
    """script_skill — 白名单脚本执行。

    与 motion/end_effector 不同，input_mappings 不能整体 resolve——只逐个解析
    variable_ref，否则会把 schema 字段（name/type）也替换成运行值。
    """

    def __init__(
        self,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.environ = environ
        self.gdk_session_manager = gdk_session_manager

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        """流程: 解析 ScriptParams → 逐个 resolve input variable_ref → 执行脚本 → 校验 outputs。"""
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
    """control_end_effector_skill — 夹爪开合控制。"""

    def __init__(
        self,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.environ = environ
        self.gdk_session_manager = gdk_session_manager

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        """流程: resolve 变量引用 → 解析 EndEffectorParams → 调用 GDK → 构建 outputs。

        actual_openness 可能与 requested 不同（物理限位）。
        """
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
        # 实际开度可能不同于请求值
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
    """force_control_skill — 当前硬阻断。保留参数校验但执行总是拒绝。"""

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


# ============================================================
# SkillRuntime — 中心分发器
# ============================================================


class SkillRuntime:
    """将 taskflow 节点分发到对应技能实现。

    持有 SkillRegistry（白名单）、预实例化的 GDK 技能对象。
    run() 是 SkillRuntimeNodeRunner 调用的入口。
    """

    def __init__(
        self,
        registry: SkillRegistry | None = None,
        environ: Mapping[str, str] | None = None,
        gdk_session_manager: GdkSessionManager | None = None,
    ) -> None:
        self.registry = registry or SkillRegistry.default()
        self.assign_skill = AssignSkill()
        # GDK 技能预实例化，共享 environ 和 session manager
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
        """分发: assign → AssignSkill, worker → registry 查 skill_name → resolve GDK 技能。"""
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
        """按 SkillDefinition.adapter 找到具体 Skill 实例。目前仅支持 gdk adapter。"""
        if skill_definition.adapter == "gdk":
            return self.resolve_gdk_skill(skill_definition)
        raise SkillRuntimeError(f"不支持的 skill adapter: {skill_definition.adapter}")

    def resolve_gdk_skill(self, skill_definition: SkillDefinition) -> Skill:
        """按 implementation 名查找预实例化的 GDK 技能。"""
        skill = self.gdk_skills.get(skill_definition.implementation)
        if skill is None:
            raise SkillRuntimeError(f"不支持的 GDK skill 类型: {skill_definition.implementation}")
        return skill


# ============================================================
# 输出构建器 — 提取参数和结果字段，构造一致的 outputs dict
# ============================================================


def build_motion_plan_outputs(
    *,
    app_execution_id: str,
    skill_name: str | None,
    mode: str,
    params_template: Mapping[str, Any],
    motion_params: MotionPlanParams,
) -> dict[str, object]:
    """构建运动规划 outputs。"""
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
    """构建 GDK 错误 detail。提取 error_code/error_stage 供调度器分类。"""
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


# ============================================================
# 脚本输入/输出解析与校验
# ============================================================


def resolve_script_inputs(
    input_mappings: Sequence[ScriptInputMapping],
    variable_store: VariableStore,
    *,
    node_id: str,
) -> dict[str, object]:
    """逐个 resolve input_mappings 的 variable_ref，类型校验。"""
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
    """校验脚本声明的输出变量：必须存在且类型匹配。确保下游节点不会遇到意外类型。"""
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
    """检查运行时值是否匹配声明的类型。

    - integer 排除 bool（Python 中 bool 是 int 子类）
    - number 排除 bool 且要求有限值
    - array 排除 str/bytes/bytearray（它们也是 Sequence）
    """
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
    """构建脚本执行 outputs。"""
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
    """构建末端控制 outputs。调用方额外补充 actual_openness 和 end_effector_type。"""
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
    """构建力控 outputs（当前未被使用，力控硬阻断时抛异常）。"""
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
