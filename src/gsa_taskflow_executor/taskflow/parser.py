from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from typing import Any

import yaml

from gsa_taskflow_executor.code_scripts.registry import CODE_SCRIPT_IDS

YamlMapping = Mapping[str, Any]
MOTION_SPEED_MIN = 0.001
MOTION_SPEED_MAX = 0.1
DEFAULT_SCRIPT_TIMEOUT = 50.0
DEFAULT_END_EFFECTOR_TIMEOUT = 20.0
DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS = 1.0
END_EFFECTOR_TARGET_ALIASES = {
    "left": "left_tool",
    "left_end": "left_tool",
    "left_tool": "left_tool",
    "左末端": "left_tool",
    "right": "right_tool",
    "right_end": "right_tool",
    "right_tool": "right_tool",
    "右末端": "right_tool",
}
SCRIPT_IDS = CODE_SCRIPT_IDS
SCRIPT_VALUE_TYPE_ALIASES = {
    "string": "string",
    "String": "string",
    "integer": "integer",
    "Integer": "integer",
    "number": "number",
    "Number": "number",
    "boolean": "boolean",
    "Boolean": "boolean",
    "time": "time",
    "Time": "time",
    "object": "object",
    "Object": "object",
    "array": "array",
    "Array": "array",
}


class TaskflowParseError(ValueError):
    """Raised when a taskflow YAML payload is structurally invalid."""


@dataclass(frozen=True)
class TaskflowTransition:
    from_node: str
    outcome: str
    to_node: str


@dataclass(frozen=True)
class MotionPlanTarget:
    body_part: str
    control_type: str
    action_data: list[float] | str


@dataclass(frozen=True)
class MotionPlanParams:
    targets: tuple[MotionPlanTarget, ...]
    speed: float
    timeout: float


@dataclass(frozen=True)
class ScriptParams:
    script_id: str
    timeout: float
    input_mappings: tuple[ScriptInputMapping, ...] = ()
    output_variables: tuple[ScriptOutputVariable, ...] = ()


@dataclass(frozen=True)
class ScriptInputMapping:
    name: str
    value_type: str
    variable_ref: str


@dataclass(frozen=True)
class ScriptOutputVariable:
    name: str
    value_type: str


@dataclass(frozen=True)
class EndEffectorParams:
    target_end: str
    end_effector_type: str | None
    opening: float
    timeout: float
    post_wait_seconds: float = DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS


@dataclass(frozen=True)
class TaskflowNode:
    node_id: str
    node_type: str
    assignments: YamlMapping
    skill_name: str | None
    params_template: YamlMapping
    capture_state_detail: bool
    output_var: str | None
    output_contract: YamlMapping

    @property
    def is_worker(self) -> bool:
        return self.node_type == "worker"


@dataclass(frozen=True)
class TaskflowDefinition:
    start_node: str
    app_execution_id: str
    nodes: tuple[TaskflowNode, ...]
    transitions: tuple[TaskflowTransition, ...]

    @property
    def node_ids(self) -> tuple[str, ...]:
        return tuple(node.node_id for node in self.nodes)

    @property
    def worker_nodes(self) -> tuple[TaskflowNode, ...]:
        return tuple(node for node in self.nodes if node.is_worker)

    def summary(self) -> dict[str, Any]:
        return {
            "start_node": self.start_node,
            "app_execution_id": self.app_execution_id,
            "node_count": len(self.nodes),
            "worker_count": len(self.worker_nodes),
            "transition_count": len(self.transitions),
            "node_ids": list(self.node_ids),
        }


def parse_taskflow_yaml(payload: str) -> TaskflowDefinition:
    try:
        raw = yaml.safe_load(payload)
    except yaml.YAMLError as error:
        raise TaskflowParseError(f"YAML 解析失败: {error}") from error

    root = expect_mapping(raw, "root")
    start_node = read_required_string(root, "start_node", "root")
    app_execution_id = read_required_string(root, "app_execution_id", "root")
    nodes = parse_nodes(root.get("nodes"))
    transitions = parse_transitions(root.get("transitions"))

    definition = TaskflowDefinition(
        start_node=start_node,
        app_execution_id=app_execution_id,
        nodes=tuple(nodes),
        transitions=tuple(transitions),
    )
    validate_definition(definition)
    return definition


def parse_nodes(raw_nodes: Any) -> list[TaskflowNode]:
    if not isinstance(raw_nodes, list) or not raw_nodes:
        raise TaskflowParseError("nodes 必须是非空数组")

    nodes: list[TaskflowNode] = []
    for index, raw_node in enumerate(raw_nodes):
        path = f"nodes[{index}]"
        node = expect_mapping(raw_node, path)
        node_id = read_required_string(node, "id", path)
        node_type = read_required_string(node, "type", path)

        if node_type == "assign":
            assignments = expect_mapping(node.get("assignments", {}), f"{path}.assignments")
            nodes.append(
                TaskflowNode(
                    node_id=node_id,
                    node_type=node_type,
                    assignments=assignments,
                    skill_name=None,
                    params_template={},
                    capture_state_detail=False,
                    output_var=None,
                    output_contract={},
                )
            )
            continue

        if node_type != "worker":
            raise TaskflowParseError(f"{path}.type 暂不支持: {node_type}")

        skill_name = read_required_string(node, "skill_name", path)
        params_template = expect_mapping(node.get("params_template"), f"{path}.params_template")
        validate_worker_params(skill_name, params_template, path)
        nodes.append(
            TaskflowNode(
                node_id=node_id,
                node_type=node_type,
                assignments={},
                skill_name=skill_name,
                params_template=params_template,
                capture_state_detail=read_bool(node.get("capture_state_detail"), False),
                output_var=read_optional_string(node.get("output_var"), f"{path}.output_var"),
                output_contract=expect_mapping(
                    node.get("output_contract", {}),
                    f"{path}.output_contract",
                ),
            )
        )

    return nodes


def parse_transitions(raw_transitions: Any) -> list[TaskflowTransition]:
    if not isinstance(raw_transitions, list):
        raise TaskflowParseError("transitions 必须是数组")

    transitions: list[TaskflowTransition] = []
    for index, raw_transition in enumerate(raw_transitions):
        path = f"transitions[{index}]"
        transition = expect_mapping(raw_transition, path)
        transitions.append(
            TaskflowTransition(
                from_node=read_required_string(transition, "from", path),
                outcome=read_required_string(transition, "outcome", path),
                to_node=read_required_string(transition, "to", path),
            )
        )

    return transitions


def validate_definition(definition: TaskflowDefinition) -> None:
    seen: set[str] = set()
    for node in definition.nodes:
        if node.node_id in seen:
            raise TaskflowParseError(f"节点 ID 重复: {node.node_id}")
        seen.add(node.node_id)

    if definition.start_node not in seen:
        raise TaskflowParseError(f"start_node 不存在: {definition.start_node}")

    for transition in definition.transitions:
        if transition.from_node not in seen:
            raise TaskflowParseError(f"transition.from 不存在: {transition.from_node}")
        if transition.to_node not in seen:
            raise TaskflowParseError(f"transition.to 不存在: {transition.to_node}")
        if transition.outcome != "success":
            raise TaskflowParseError(f"第一阶段只支持 success transition: {transition.outcome}")


def validate_worker_params(skill_name: str, params: YamlMapping, path: str) -> None:
    if skill_name == "motion_plan_skill":
        parse_motion_plan_params(params, f"{path}.params_template")
    elif skill_name == "script_skill":
        parse_script_params(params, f"{path}.params_template")
    elif skill_name == "control_end_effector_skill":
        parse_end_effector_params(params, f"{path}.params_template")


def parse_motion_plan_params(params: YamlMapping, path: str) -> MotionPlanParams:
    speed = read_motion_speed(params.get("speed"), f"{path}.speed")
    timeout = read_positive_number(params.get("timeout"), f"{path}.timeout")
    targets: list[MotionPlanTarget] = []

    for body_part in ("left_arm", "right_arm", "waist"):
        if body_part not in params:
            continue
        target = expect_mapping(params[body_part], f"{path}.{body_part}")
        control_type = read_required_string(target, "control_type", f"{path}.{body_part}")
        action_data = parse_action_data(
            target.get("action_data"),
            body_part,
            control_type,
            f"{path}.{body_part}.action_data",
        )
        targets.append(
            MotionPlanTarget(
                body_part=body_part,
                control_type=control_type,
                action_data=action_data,
            )
        )

    if not targets:
        raise TaskflowParseError(f"{path} 至少需要 left_arm/right_arm/waist 之一")

    return MotionPlanParams(targets=tuple(targets), speed=speed, timeout=timeout)


def parse_script_params(params: YamlMapping, path: str) -> ScriptParams:
    script_id = read_required_string(params, "script_id", path)
    if script_id not in SCRIPT_IDS:
        raise TaskflowParseError(f"{path}.script_id 未在白名单中: {script_id}")
    timeout = read_positive_number_with_default(
        params.get("timeout"),
        f"{path}.timeout",
        DEFAULT_SCRIPT_TIMEOUT,
    )
    input_mappings = parse_script_input_mappings(
        params.get("input_mappings", ()),
        f"{path}.input_mappings",
    )
    output_variables = parse_script_output_variables(
        params.get("output_variables", ()),
        f"{path}.output_variables",
    )
    return ScriptParams(
        script_id=script_id,
        timeout=timeout,
        input_mappings=tuple(input_mappings),
        output_variables=tuple(output_variables),
    )


def parse_script_input_mappings(raw: Any, path: str) -> list[ScriptInputMapping]:
    if raw in (None, ()):
        return []
    if not isinstance(raw, list):
        raise TaskflowParseError(f"{path} 必须是数组")

    seen_names: set[str] = set()
    mappings: list[ScriptInputMapping] = []
    for index, item in enumerate(raw):
        item_path = f"{path}[{index}]"
        mapping = expect_mapping(item, item_path)
        if is_blank_script_mapping_row(mapping, ("name", "type", "variable_ref", "variableRef")):
            continue
        name = read_required_string(mapping, "name", item_path)
        validate_script_variable_name(name, f"{item_path}.name")
        if name in seen_names:
            raise TaskflowParseError(f"{item_path}.name 重复: {name}")
        seen_names.add(name)
        variable_ref = read_script_variable_ref(mapping, item_path)
        mappings.append(
            ScriptInputMapping(
                name=name,
                value_type=read_script_value_type(mapping.get("type"), f"{item_path}.type"),
                variable_ref=variable_ref,
            )
        )
    return mappings


def parse_script_output_variables(raw: Any, path: str) -> list[ScriptOutputVariable]:
    if raw in (None, ()):
        return []
    if not isinstance(raw, list):
        raise TaskflowParseError(f"{path} 必须是数组")

    seen_names: set[str] = set()
    outputs: list[ScriptOutputVariable] = []
    for index, item in enumerate(raw):
        item_path = f"{path}[{index}]"
        output = expect_mapping(item, item_path)
        if is_blank_script_mapping_row(output, ("name", "type")):
            continue
        name = read_required_string(output, "name", item_path)
        validate_script_variable_name(name, f"{item_path}.name")
        if name in seen_names:
            raise TaskflowParseError(f"{item_path}.name 重复: {name}")
        seen_names.add(name)
        outputs.append(
            ScriptOutputVariable(
                name=name,
                value_type=read_script_value_type(output.get("type"), f"{item_path}.type"),
            )
        )
    return outputs


def parse_end_effector_params(params: YamlMapping, path: str) -> EndEffectorParams:
    target_end = read_end_effector_target(params, path)
    end_effector_type = read_optional_string(
        params.get("end_effector_type", params.get("target_type")),
        f"{path}.end_effector_type",
    )
    opening = read_end_effector_opening(params.get("opening"), f"{path}.opening")
    timeout = read_positive_number_with_default(
        params.get("timeout"),
        f"{path}.timeout",
        DEFAULT_END_EFFECTOR_TIMEOUT,
    )
    post_wait_seconds = read_non_negative_number_with_default(
        params.get("post_wait_seconds", params.get("postWaitSeconds")),
        f"{path}.post_wait_seconds",
        DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS,
    )
    return EndEffectorParams(
        target_end=target_end,
        end_effector_type=end_effector_type,
        opening=opening,
        timeout=timeout,
        post_wait_seconds=post_wait_seconds,
    )


def parse_action_data(raw: Any, body_part: str, control_type: str, path: str) -> list[float] | str:
    if isinstance(raw, str):
        value = raw.strip()
        if value.startswith("$.variables."):
            return value

    if control_type != "ABS_JOINT":
        raise TaskflowParseError(f"{path} 第一阶段只支持 ABS_JOINT，当前为 {control_type}")

    if not isinstance(raw, list):
        raise TaskflowParseError(f"{path} 必须是关节数组")

    expected_length = 5 if body_part == "waist" else 7
    if len(raw) != expected_length:
        raise TaskflowParseError(
            f"{path} 长度必须是 {expected_length}，当前为 {len(raw)}"
        )

    values = [to_float(item, f"{path}[{index}]") for index, item in enumerate(raw)]
    return values


def expect_mapping(value: Any, path: str) -> YamlMapping:
    if not isinstance(value, Mapping):
        raise TaskflowParseError(f"{path} 必须是对象")
    return value


def read_required_string(mapping: YamlMapping, key: str, path: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise TaskflowParseError(f"{path}.{key} 必须是非空字符串")
    return value.strip()


def read_script_variable_ref(mapping: YamlMapping, path: str) -> str:
    raw = mapping.get("variable_ref", mapping.get("variableRef"))
    if not isinstance(raw, str) or not raw.strip():
        raise TaskflowParseError(f"{path}.variable_ref 必须是非空字符串")
    variable_ref = raw.strip()
    if not variable_ref.startswith("$.variables."):
        raise TaskflowParseError(f"{path}.variable_ref 必须是 $.variables. 开头的变量引用")
    return variable_ref


def read_script_value_type(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TaskflowParseError(f"{path} 必须是非空字符串")
    value_type = SCRIPT_VALUE_TYPE_ALIASES.get(value.strip())
    if value_type is None:
        allowed = "/".join(sorted(set(SCRIPT_VALUE_TYPE_ALIASES.values())))
        raise TaskflowParseError(f"{path} 类型无效，只支持 {allowed}")
    return value_type


def validate_script_variable_name(name: str, path: str) -> None:
    # 变量引用路径用 "." 分段；输出名含点会让下游解析到错误层级。
    if "." in name:
        raise TaskflowParseError(f"{path} 不能包含 .")


def is_blank_script_mapping_row(mapping: YamlMapping, keys: tuple[str, ...]) -> bool:
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return False
        if value is not None and not isinstance(value, str):
            return False
    return True


def read_optional_string(value: Any, path: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise TaskflowParseError(f"{path} 必须是字符串")
    return value.strip() or None


def read_end_effector_target(params: YamlMapping, path: str) -> str:
    raw = read_required_string(params, "target_end", path)
    target = END_EFFECTOR_TARGET_ALIASES.get(raw)
    if target is None:
        raise TaskflowParseError(f"{path}.target_end 只支持 left_tool/right_tool")
    return target


def read_bool(value: Any, fallback: bool) -> bool:
    if value is None:
        return fallback
    if isinstance(value, bool):
        return value
    return fallback


def read_positive_number(value: Any, path: str) -> float:
    number = to_float(value, path)
    if number <= 0:
        raise TaskflowParseError(f"{path} 必须大于 0")
    return number


def read_positive_number_with_default(value: Any, path: str, fallback: float) -> float:
    if value is None:
        return fallback
    return read_positive_number(value, path)


def read_non_negative_number_with_default(value: Any, path: str, fallback: float) -> float:
    if value is None:
        return fallback
    number = to_float(value, path)
    if number < 0:
        raise TaskflowParseError(f"{path} 必须大于等于 0")
    return number


def read_end_effector_opening(value: Any, path: str) -> float:
    number = to_float(value, path)
    if number < 0 or number > 1:
        raise TaskflowParseError(f"{path} 必须在 0 到 1 之间")
    return number


def read_motion_speed(value: Any, path: str) -> float:
    number = read_positive_number(value, path)
    if number < MOTION_SPEED_MIN or number > MOTION_SPEED_MAX:
        raise TaskflowParseError(
            f"{path} 必须在 {MOTION_SPEED_MIN} 到 {MOTION_SPEED_MAX} 之间"
        )
    return number


def to_float(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise TaskflowParseError(f"{path} 必须是数字")
    if isinstance(value, int | float):
        number = float(value)
        if isfinite(number):
            return number
    raise TaskflowParseError(f"{path} 必须是数字")
