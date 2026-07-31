from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from typing import Any

import yaml

YamlMapping = Mapping[str, Any]
MOTION_SPEED_MIN = 0.001
MOTION_SPEED_MAX = 0.1
SCRIPT_IDS = frozenset(
    {
        "gdk_hold_current_dual_arm",
        "gdk_nudge_left_j7_0p005",
        "gdk_nudge_right_j7_0p005",
    }
)


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
    timeout = read_positive_number(params.get("timeout"), f"{path}.timeout")
    return ScriptParams(script_id=script_id, timeout=timeout)


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


def read_optional_string(value: Any, path: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise TaskflowParseError(f"{path} 必须是字符串")
    return value.strip() or None


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
