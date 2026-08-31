"""技能运行时 — 将 taskflow 节点分发到 GDK 技能实现。

SkillRuntime.run(node, context) 分发逻辑::

    assign → AssignSkill（写入 VariableStore）
    worker → SkillRegistry.require(skill_name) → GDK skill
      ├─ motion_plan  → MotionPlanSkillGdk  → run_gdk_motion_plan_abs_joint()
      ├─ script       → ScriptSkillGdk       → run_code_script()
      ├─ end_effector → EndEffectorSkillGdk  → run_gdk_end_effector_control()
      ├─ force_control→ ForceControlSkillGdk → 硬阻断
      └─ qr_pose      → QrPoseSkillGdk       → 回初始拍照点位后二维码定位

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
from gsa_taskflow_executor.qr_mapping.pose_service import QrPoseService
from gsa_taskflow_executor.skills.registry import (
    SkillDefinition,
    SkillRegistry,
    SkillRegistryError,
)
from gsa_taskflow_executor.taskflow.models import (
    EndEffectorParams,
    ForceControlParams,
    MotionPlanParams,
    QrPoseParams,
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
    parse_qr_pose_params,
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
    """motion_plan_skill — ABS_JOINT/ABS_POSE 运动规划。

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
            message = str(error_msg or "GDK motion_plan 执行失败")
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

        prefer_servo = should_prefer_servo_end_effector_path(
            context.variable_store,
            end_effector_params=end_effector_params,
        )
        if prefer_servo:
            end_effector_result = run_gdk_end_effector_control(
                end_effector_params,
                environ=self.environ,
                session_manager=self.gdk_session_manager,
                prefer_servo=True,
            )
        else:
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
        outputs["prefer_servo"] = prefer_servo
        # GDK runtime 会补齐从真机状态推断出的型号与左右开度；这些值必须进入
        # VariableStore，否则后续代码节点会拿到解析前的空值。
        merge_end_effector_result_outputs(outputs, end_effector_result)
        # 实际开度可能不同于请求值
        actual_openness = end_effector_result.get("actual_openness")
        if not isinstance(actual_openness, list):
            actual_openness = build_requested_openness_fallback(
                outputs,
                end_effector_params=end_effector_params,
            )
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
                "prefer_servo": prefer_servo,
                "end_effector_result": deepcopy(end_effector_result),
            },
            outputs=outputs,
        )


def should_prefer_servo_end_effector_path(
    variable_store: VariableStore,
    *,
    end_effector_params: EndEffectorParams,
) -> bool:
    """判断末端控制是否应复用伺服接口。

    GDK 4.1.5 文档明确 move_ee_pos 不可与伺服接口混用。二维码定位后常见链路是
    ABS_POSE → 末端控制，因此只要同一 workflow 上游同臂执行过 ABS_POSE，就把
    夹爪开合切到 end_effector_pose_control 的 joint_names/joint_positions 路径。
    """

    target_arms = end_effector_target_arms(end_effector_params.target_end)
    for node_scope in variable_store.variables.values():
        if upstream_node_used_abs_pose_servo(node_scope, target_arms=target_arms):
            return True
    return False


def upstream_node_used_abs_pose_servo(
    node_scope: object,
    *,
    target_arms: set[str],
) -> bool:
    if not isinstance(node_scope, Mapping):
        return False
    detail = node_scope.get("detail")
    if not isinstance(detail, Mapping):
        return False

    outputs = detail.get("outputs")
    if isinstance(outputs, Mapping) and motion_outputs_match_abs_pose(
        outputs,
        target_arms=target_arms,
    ):
        return True

    gdk_result = detail.get("gdk_result")
    if isinstance(gdk_result, Mapping):
        return gdk_result_matches_abs_pose_servo(gdk_result, target_arms=target_arms)
    return False


def motion_outputs_match_abs_pose(
    outputs: Mapping[str, object],
    *,
    target_arms: set[str],
) -> bool:
    if outputs.get("primary_control_type") != "ABS_POSE":
        return False
    body_part = outputs.get("primary_body_part")
    if isinstance(body_part, str) and body_part:
        return body_part in target_arms
    gdk_result = outputs.get("gdk_result")
    if isinstance(gdk_result, Mapping):
        return gdk_result_matches_abs_pose_servo(gdk_result, target_arms=target_arms)
    return True


def gdk_result_matches_abs_pose_servo(
    gdk_result: Mapping[str, object],
    *,
    target_arms: set[str],
) -> bool:
    if not (
        gdk_result.get("control_type") == "ABS_POSE"
        or gdk_result.get("action") == "taskflow_abs_pose"
        or gdk_result.get("method") == "end_effector_pose_control"
    ):
        return False

    result_arms = read_gdk_result_arms(gdk_result)
    if not result_arms:
        return True
    return bool(result_arms.intersection(target_arms))


def read_gdk_result_arms(gdk_result: Mapping[str, object]) -> set[str]:
    arms: set[str] = set()
    body_part = gdk_result.get("body_part")
    if isinstance(body_part, str) and body_part:
        arms.add(body_part)
    requested_body_parts = gdk_result.get("requested_body_parts")
    if isinstance(requested_body_parts, Sequence) and not isinstance(
        requested_body_parts,
        str | bytes | bytearray,
    ):
        arms.update(part for part in requested_body_parts if isinstance(part, str) and part)
    return arms


def end_effector_target_arms(target_end: str) -> set[str]:
    if target_end == "left_tool":
        return {"left_arm"}
    if target_end == "right_tool":
        return {"right_arm"}
    if target_end == "dual_tool":
        return {"left_arm", "right_arm"}
    return set()


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


class QrPoseSkillGdk:
    """qr_pose_skill — 手臂和腰部回初始拍照点位后定位，输出目标点 pose/action_data。

    该 skill 复用二维码建图/点位录制产物。默认先通过 ABS_JOINT 安全门
    回到初始拍照点位的手臂和腰部姿态，再做只读采样和 SDK 定位；
    目标点执行仍由下游节点决定。
    """

    def __init__(self, service: QrPoseService | None = None) -> None:
        self.service = service

    def run(self, node: TaskflowNode, context: SkillExecutionContext) -> SkillResult:
        resolved_params = context.variable_store.resolve_value(node.params_template)
        if not isinstance(resolved_params, Mapping):
            raise SkillRuntimeError(f"{node.node_id}.params_template 解析后不是对象")
        if self.service is None:
            raise SkillRuntimeError("executor 未配置二维码定位服务")

        try:
            qr_pose_params = parse_qr_pose_params(resolved_params, "params_template")
        except TaskflowParseError as error:
            raise SkillRuntimeError(str(error)) from error

        result = self.service.locate(qr_pose_params)
        if result.get("available") is not True:
            message = str(result.get("errorMsg") or "二维码定位失败")
            raise SkillRuntimeError(message, detail=build_gdk_error_detail(message, result))

        outputs = build_qr_pose_outputs(
            app_execution_id=context.app_execution_id,
            skill_name=node.skill_name,
            mode=context.mode,
            params_template=resolved_params,
            qr_pose_params=qr_pose_params,
            result=result,
        )
        return SkillResult(
            outcome="success",
            detail={
                "skill_name": node.skill_name,
                "mode": context.mode,
                "adapter": "gdk",
                "params_template": deepcopy(dict(resolved_params)),
                "qr_pose_result": deepcopy(result),
            },
            outputs=outputs,
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
        qr_pose_service: QrPoseService | None = None,
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
            "qr_pose": QrPoseSkillGdk(service=qr_pose_service),
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

    outputs: dict[str, object] = {
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
    if primary_target.control_type == "ABS_POSE":
        outputs["final_pose"] = deepcopy(primary_target.action_data)
    return outputs


def build_qr_pose_outputs(
    *,
    app_execution_id: str,
    skill_name: str | None,
    mode: str,
    params_template: Mapping[str, Any],
    qr_pose_params: QrPoseParams,
    result: Mapping[str, object],
) -> dict[str, object]:
    """构建二维码定位 outputs。action_data 按目标点名称索引，供下游变量引用。"""

    return {
        "app_execution_id": app_execution_id,
        "skill_name": skill_name,
        "mode": mode,
        "robot_serial": qr_pose_params.robot_serial,
        "project_name": qr_pose_params.project_name,
        "map_name": result.get("mapName"),
        "initial_photo_point_name": qr_pose_params.initial_photo_point_name,
        "target_point_names": deepcopy(result.get("targetPointNames")),
        "pose": deepcopy(result.get("pose")),
        "poses": deepcopy(result.get("poses")),
        "action_data": deepcopy(result.get("action_data")),
        "current_tag_pose": deepcopy(result.get("currentTagPose")),
        "quality": deepcopy(result.get("quality")),
        "initial_photo_return": deepcopy(result.get("initialPhotoReturn")),
        "artifact_paths": deepcopy(result.get("artifactPaths")),
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
        "left_end_effector_type": end_effector_params.left_end_effector_type,
        "right_end_effector_type": end_effector_params.right_end_effector_type,
        "left_opening": end_effector_params.left_opening,
        "right_opening": end_effector_params.right_opening,
        "timeout": end_effector_params.timeout,
        "post_wait_seconds": end_effector_params.post_wait_seconds,
        "resolved_params_template": deepcopy(dict(params_template)),
    }


def merge_end_effector_result_outputs(
    outputs: dict[str, object],
    end_effector_result: Mapping[str, object],
) -> None:
    """回写 GDK 侧最终采用的末端控制参数，供下游节点读取真实执行语义。"""

    for key in (
        "target_end",
        "opening",
        "left_opening",
        "right_opening",
        "left_end_effector_type",
        "right_end_effector_type",
        "method",
        "control_method",
        "controlled_arms",
    ):
        if key in end_effector_result:
            outputs[key] = deepcopy(end_effector_result[key])


def build_requested_openness_fallback(
    outputs: Mapping[str, object],
    *,
    end_effector_params: EndEffectorParams,
) -> list[float]:
    """当 GDK 未返回实际开度时，用最终请求开度兜底。"""

    if end_effector_params.target_end == "dual_tool":
        side_openings = [
            read_output_number(outputs.get("left_opening")),
            read_output_number(outputs.get("right_opening")),
        ]
        if all(value is not None for value in side_openings):
            return [float(value) for value in side_openings if value is not None]

    opening = read_output_number(outputs.get("opening"))
    if opening is not None:
        return [opening]
    if end_effector_params.opening is not None:
        return [end_effector_params.opening]
    return []


def read_output_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


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
