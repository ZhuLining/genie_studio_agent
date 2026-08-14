"""Taskflow YAML 解析与 DAG 结构校验。

YAML 结构概览::

    start_node: <node_id>
    app_execution_id: <string>
    nodes:
      - id: <string>
        type: worker | assign | timer | loop | end
        # worker 节点
        skill_name: <registered skill name>
        params_template: { ... }        # 技能参数，支持 $.variables.* 变量引用
        capture_state_detail: bool      # 执行后是否采集机器人状态
        output_var: <string>            # 输出存储变量名
        # assign 节点
        assignments: { key: value }     # 直接写入 VariableStore
        # timer 节点
        timer_mode: "rel"
        duration: <float>               # 等待秒数
        # loop 节点
        loop_mode: "count"
        children: [<node_id>, ...]      # 线性链子节点（v1 不嵌套）
        iteration_max: <int>            # 最大迭代次数 1-100
        # end 节点
        # 无参数；表示显式流程终点；每个 workflow 只能有一个
    transitions:
      - from: <node_id>
        outcome: "success"              # v1 只支持 success
        to: <node_id>
"""

from __future__ import annotations

from typing import Any

import yaml

from gsa_taskflow_executor.taskflow.models import (
    EndEffectorParams,
    ForceControlParams,
    LoopParams,
    MotionPlanParams,
    MotionPlanTarget,
    ScriptInputMapping,
    ScriptOutputVariable,
    ScriptParams,
    TaskflowDefinition,
    TaskflowNode,
    TaskflowParseError,
    TaskflowTransition,
    TimerParams,
    YamlMapping,
)
from gsa_taskflow_executor.taskflow.readers import (
    expect_mapping,
    read_bool,
    read_int_in_range,
    read_int_in_range_with_default,
    read_non_negative_number,
    read_non_negative_number_with_default,
    read_number_in_range_with_default,
    read_optional_string,
    read_positive_int,
    read_positive_number,
    read_positive_number_with_default,
    read_required_string,
    read_required_string_list,
    to_float,
)
from gsa_taskflow_executor.taskflow.skill_params import (
    DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS,
    DEFAULT_END_EFFECTOR_TIMEOUT,
    DEFAULT_FORCE_CONTROL_HZ,
    DEFAULT_FORCE_CONTROL_STEP,
    DEFAULT_FORCE_CONTROL_TIMEOUT,
    DEFAULT_SCRIPT_TIMEOUT,
    END_EFFECTOR_TARGET_ALIASES,
    FORCE_CONTROL_ARMS,
    FORCE_CONTROL_HZ_MAX,
    FORCE_CONTROL_HZ_MIN,
    FORCE_CONTROL_METHOD_MOVE_UNTIL_FORCE,
    FORCE_CONTROL_METHOD_SMOOTH_MOVE,
    FORCE_CONTROL_METHODS,
    FORCE_CONTROL_STEP_MAX,
    FORCE_CONTROL_STEP_MIN,
    LOOP_MODE_COUNT,
    MAX_LOOP_ITERATIONS,
    MOTION_SPEED_MAX,
    MOTION_SPEED_MIN,
    SCRIPT_IDS,
    SCRIPT_VALUE_TYPE_ALIASES,
    TIMER_MODE_REL,
    is_blank_script_mapping_row,
    parse_action_data,
    parse_end_effector_params,
    parse_force_control_params,
    parse_loop_params,
    parse_motion_plan_params,
    parse_script_input_mappings,
    parse_script_output_variables,
    parse_script_params,
    parse_timer_params,
    read_end_effector_opening,
    read_end_effector_target,
    read_force_delta_xyz,
    read_motion_speed,
    read_script_value_type,
    read_script_variable_ref,
    validate_script_variable_name,
    validate_worker_params,
)

__all__ = [
    "DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS",
    "DEFAULT_END_EFFECTOR_TIMEOUT",
    "DEFAULT_FORCE_CONTROL_HZ",
    "DEFAULT_FORCE_CONTROL_STEP",
    "DEFAULT_FORCE_CONTROL_TIMEOUT",
    "DEFAULT_SCRIPT_TIMEOUT",
    "END_EFFECTOR_TARGET_ALIASES",
    "EndEffectorParams",
    "FORCE_CONTROL_ARMS",
    "FORCE_CONTROL_HZ_MAX",
    "FORCE_CONTROL_HZ_MIN",
    "FORCE_CONTROL_METHOD_MOVE_UNTIL_FORCE",
    "FORCE_CONTROL_METHOD_SMOOTH_MOVE",
    "FORCE_CONTROL_METHODS",
    "FORCE_CONTROL_STEP_MAX",
    "FORCE_CONTROL_STEP_MIN",
    "ForceControlParams",
    "LOOP_MODE_COUNT",
    "LoopParams",
    "MAX_LOOP_ITERATIONS",
    "MOTION_SPEED_MAX",
    "MOTION_SPEED_MIN",
    "MotionPlanParams",
    "MotionPlanTarget",
    "SCRIPT_IDS",
    "SCRIPT_VALUE_TYPE_ALIASES",
    "ScriptInputMapping",
    "ScriptOutputVariable",
    "ScriptParams",
    "TIMER_MODE_REL",
    "TaskflowDefinition",
    "TaskflowNode",
    "TaskflowParseError",
    "TaskflowTransition",
    "TimerParams",
    "YamlMapping",
    "expect_mapping",
    "is_blank_script_mapping_row",
    "parse_action_data",
    "parse_end_effector_params",
    "parse_force_control_params",
    "parse_loop_params",
    "parse_motion_plan_params",
    "parse_nodes",
    "parse_script_input_mappings",
    "parse_script_output_variables",
    "parse_script_params",
    "parse_taskflow_yaml",
    "parse_timer_params",
    "parse_transitions",
    "read_bool",
    "read_end_effector_opening",
    "read_end_effector_target",
    "read_force_delta_xyz",
    "read_int_in_range",
    "read_int_in_range_with_default",
    "read_motion_speed",
    "read_non_negative_number",
    "read_non_negative_number_with_default",
    "read_number_in_range_with_default",
    "read_optional_string",
    "read_positive_int",
    "read_positive_number",
    "read_positive_number_with_default",
    "read_required_string",
    "read_required_string_list",
    "read_script_value_type",
    "read_script_variable_ref",
    "to_float",
    "validate_definition",
    "validate_loop_body_transitions",
    "validate_script_variable_name",
    "validate_worker_params",
]


def parse_taskflow_yaml(payload: str) -> TaskflowDefinition:
    """解析 YAML 字符串为 TaskflowDefinition。

    流程: YAML parse → 提取 root 字段 → 解析 nodes → 解析 transitions → 结构校验。
    """

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
    """解析 nodes 列表。按 node.type 分发: assign → end → timer → loop → worker。"""

    if not isinstance(raw_nodes, list) or not raw_nodes:
        raise TaskflowParseError("nodes 必须是非空数组")

    nodes: list[TaskflowNode] = []
    for index, raw_node in enumerate(raw_nodes):
        path = f"nodes[{index}]"
        node = expect_mapping(raw_node, path)
        node_id = read_required_string(node, "id", path)
        node_type = read_required_string(node, "type", path)

        # assign: 直接写入 VariableStore
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

        # end: 显式流程终点。它是结构节点，不执行技能，也不能有出边。
        if node_type == "end":
            nodes.append(
                TaskflowNode(
                    node_id=node_id,
                    node_type=node_type,
                    assignments={},
                    skill_name=None,
                    params_template={},
                    capture_state_detail=False,
                    output_var=None,
                    output_contract={},
                )
            )
            continue

        # timer: 相对延时等待
        if node_type == "timer":
            timer_params = parse_timer_params(node, path)
            nodes.append(
                TaskflowNode(
                    node_id=node_id,
                    node_type=node_type,
                    assignments={},
                    skill_name=None,
                    params_template={},
                    capture_state_detail=False,
                    output_var=None,
                    output_contract={},
                    timer_mode=timer_params.timer_mode,
                    duration=timer_params.duration,
                )
            )
            continue

        # loop: 固定次数循环
        if node_type == "loop":
            loop_params = parse_loop_params(node, path)
            nodes.append(
                TaskflowNode(
                    node_id=node_id,
                    node_type=node_type,
                    assignments={},
                    skill_name=None,
                    params_template={},
                    capture_state_detail=False,
                    output_var=None,
                    output_contract={},
                    loop_mode=loop_params.loop_mode,
                    children=loop_params.children,
                    iteration_max=loop_params.iteration_max,
                )
            )
            continue

        # worker: 技能执行节点。技能参数校验被拆到 skill_params，parser 只保留结构入口。
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
    """解析 transitions 列表。每项含 from/outcome/to。"""

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
    """校验 taskflow 的完整结构正确性。

    检查项:
    1. 节点 ID 唯一
    2. start_node 存在
    3. 所有 transition 的 from/to 节点存在
    4. 只支持 success outcome
    5. loop children 只能是 worker/timer，不嵌套
    6. 每个子节点最多属于一个 loop
    7. transition 不跨 loop 边界
    8. loop 内部为单入口单出口线性链
    9. start_node 不能有入边；end 节点不能有出边
    10. workflow 必须有且仅有一个终止点；普通节点同一 outcome 只能有一条出边
    """

    seen: set[str] = set()
    nodes_by_id: dict[str, TaskflowNode] = {}

    for node in definition.nodes:
        if node.node_id in seen:
            raise TaskflowParseError(f"节点 ID 重复: {node.node_id}")
        seen.add(node.node_id)
        nodes_by_id[node.node_id] = node

    if definition.start_node not in seen:
        raise TaskflowParseError(f"start_node 不存在: {definition.start_node}")

    # 构建 child → parent loop 映射（校验排他性）
    child_to_loop: dict[str, str] = {}
    for node in definition.nodes:
        if not node.is_loop:
            continue
        for child_id in node.children:
            child = nodes_by_id.get(child_id)
            if child is None:
                raise TaskflowParseError(f"loop.children 不存在: {child_id}")
            if child.is_loop:
                raise TaskflowParseError("v1 暂不支持嵌套循环")
            if child.node_type not in {"worker", "timer"}:
                raise TaskflowParseError(f"loop.children 只支持 worker/timer 节点: {child_id}")
            existing_loop = child_to_loop.get(child_id)
            if existing_loop is not None:
                raise TaskflowParseError(
                    f"节点 {child_id} 同时属于多个循环: {existing_loop}, {node.node_id}"
                )
            child_to_loop[child_id] = node.node_id

    outgoing_by_node_outcome: dict[tuple[str, str], list[TaskflowTransition]] = {}
    incoming_by_node: dict[str, list[TaskflowTransition]] = {}
    for transition in definition.transitions:
        if transition.from_node not in seen:
            raise TaskflowParseError(f"transition.from 不存在: {transition.from_node}")
        if transition.to_node not in seen:
            raise TaskflowParseError(f"transition.to 不存在: {transition.to_node}")
        if transition.outcome != "success":
            raise TaskflowParseError(f"第一阶段只支持 success transition: {transition.outcome}")
        source_node = nodes_by_id[transition.from_node]
        if source_node.is_end:
            raise TaskflowParseError(f"end 节点不能有出边: {transition.from_node}")
        outgoing_by_node_outcome.setdefault(
            (transition.from_node, transition.outcome),
            [],
        ).append(transition)
        incoming_by_node.setdefault(transition.to_node, []).append(transition)
        # 跨 loop 边界禁止
        source_loop = child_to_loop.get(transition.from_node)
        target_loop = child_to_loop.get(transition.to_node)
        if source_loop != target_loop:
            raise TaskflowParseError("循环内部节点只能连接同一个循环内部节点")

    if incoming_by_node.get(definition.start_node):
        raise TaskflowParseError(f"start_node 不能有入边: {definition.start_node}")

    for (from_node, outcome), outgoing in outgoing_by_node_outcome.items():
        if len(outgoing) > 1:
            raise TaskflowParseError(
                f"节点 {from_node} 的 {outcome} transition 不唯一；"
                "当前版本不支持同一节点多条同 outcome 出边"
            )

    terminal_node_ids = definition.terminal_node_ids
    if len(terminal_node_ids) != 1:
        if len(terminal_node_ids) == 0:
            raise TaskflowParseError("workflow 必须包含一个结束节点")
        raise TaskflowParseError(
            "workflow 只能包含一个结束节点: " + ", ".join(terminal_node_ids)
        )

    for node in definition.nodes:
        if node.is_loop:
            validate_loop_body_transitions(definition, node)


def validate_loop_body_transitions(
    definition: TaskflowDefinition,
    loop_node: TaskflowNode,
) -> None:
    """校验 loop body 为有效线性链：单入口、单出口、无分叉、全连通。"""

    child_ids = set(loop_node.children)
    internal_transitions = [
        transition
        for transition in definition.transitions
        if transition.from_node in child_ids and transition.to_node in child_ids
    ]
    # N 个节点的线性链有 N-1 条边
    if len(internal_transitions) != max(0, len(loop_node.children) - 1):
        raise TaskflowParseError(f"循环 {loop_node.node_id} 内部必须是单入口单出口线性链")

    incoming_count = {child_id: 0 for child_id in loop_node.children}
    outgoing_count = {child_id: 0 for child_id in loop_node.children}
    adjacency: dict[str, list[str]] = {child_id: [] for child_id in loop_node.children}
    for transition in internal_transitions:
        outgoing_count[transition.from_node] += 1
        incoming_count[transition.to_node] += 1
        adjacency[transition.from_node].append(transition.to_node)

    entry_ids = [child_id for child_id, count in incoming_count.items() if count == 0]
    exit_ids = [child_id for child_id, count in outgoing_count.items() if count == 0]
    if len(entry_ids) != 1 or len(exit_ids) != 1:
        raise TaskflowParseError(f"循环 {loop_node.node_id} 内部必须有且仅有一个入口和出口")

    # 每节点入度/出度 <=1，非入口/出口节点恰好为 1
    for child_id in loop_node.children:
        incoming = incoming_count[child_id]
        outgoing = outgoing_count[child_id]
        if (
            incoming > 1
            or outgoing > 1
            or (child_id != entry_ids[0] and incoming != 1)
            or (child_id != exit_ids[0] and outgoing != 1)
        ):
            raise TaskflowParseError(f"循环 {loop_node.node_id} 内部节点 {child_id} 不是线性链")

    # 从入口遍历验证全连通
    visited: set[str] = set()
    current_id: str | None = entry_ids[0]
    while current_id is not None and current_id not in visited:
        visited.add(current_id)
        next_ids = adjacency[current_id]
        current_id = next_ids[0] if next_ids else None

    if len(visited) != len(loop_node.children):
        raise TaskflowParseError(f"循环 {loop_node.node_id} 内部节点未全部连通")
