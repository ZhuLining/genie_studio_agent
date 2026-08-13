"""Taskflow 解析后的领域模型。

这些 dataclass 是 executor 内部调度契约：parser/skill runtime/GDK runtime 共享同一套
不可变结构，避免 YAML 原始 dict 在控制链路里继续漂移。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

YamlMapping = Mapping[str, Any]


class TaskflowParseError(ValueError):
    """Taskflow YAML 结构无效。"""


@dataclass(frozen=True)
class TaskflowTransition:
    """DAG 有向边。v1 只支持 outcome="success"。"""

    from_node: str
    outcome: str
    to_node: str


@dataclass(frozen=True)
class MotionPlanTarget:
    """单个身体部件的运动目标。action_data 可以是关节角度列表或 $.variables.* 引用。"""

    body_part: str
    control_type: str
    action_data: list[float] | str


@dataclass(frozen=True)
class MotionPlanParams:
    """motion_plan_skill 参数。至少一个 body_part target。"""

    targets: tuple[MotionPlanTarget, ...]
    speed: float
    timeout: float


@dataclass(frozen=True)
class ScriptParams:
    """script_skill 参数。script_id 必须在白名单中。"""

    script_id: str
    timeout: float
    input_mappings: tuple[ScriptInputMapping, ...] = ()
    output_variables: tuple[ScriptOutputVariable, ...] = ()


@dataclass(frozen=True)
class ScriptInputMapping:
    """将 VariableStore 值绑定到脚本输入参数。variable_ref 格式: $.variables.<node>.<path>"""

    name: str
    value_type: str
    variable_ref: str


@dataclass(frozen=True)
class ScriptOutputVariable:
    """脚本声明的输出变量。name 不能含 "."。"""

    name: str
    value_type: str


@dataclass(frozen=True)
class EndEffectorParams:
    """control_end_effector_skill 参数。opening: 0(闭)-1(开)。"""

    target_end: str
    end_effector_type: str | None
    opening: float
    timeout: float
    post_wait_seconds: float = 1.0
    left_end_effector_type: str | None = None
    right_end_effector_type: str | None = None
    left_opening: float | None = None
    right_opening: float | None = None


@dataclass(frozen=True)
class ForceControlParams:
    """force_control_skill 参数。当前线上硬阻断，力控 GDK API 未验证。"""

    method: str
    arm: str
    delta_xyz: tuple[float, float, float]
    force_threshold: float | None
    timeout_s: float
    control_hz: int
    step: float


@dataclass(frozen=True)
class TimerParams:
    """timer 节点参数。v1 只支持 rel 模式。"""

    timer_mode: str
    duration: float


@dataclass(frozen=True)
class LoopParams:
    """loop 节点参数。v1 只支持 count 模式，不嵌套。"""

    loop_mode: str
    children: tuple[str, ...]
    iteration_max: int


@dataclass(frozen=True)
class TaskflowNode:
    """统一节点模型。按 node_type 使用不同字段子集。"""

    node_id: str
    node_type: str
    assignments: YamlMapping
    skill_name: str | None
    params_template: YamlMapping
    capture_state_detail: bool
    output_var: str | None
    output_contract: YamlMapping
    # loop 专用
    loop_mode: str | None = None
    children: tuple[str, ...] = ()
    iteration_max: int | None = None
    # timer 专用
    timer_mode: str | None = None
    duration: float | None = None

    @property
    def is_worker(self) -> bool:
        return self.node_type == "worker"

    @property
    def is_loop(self) -> bool:
        return self.node_type == "loop"

    @property
    def is_timer(self) -> bool:
        return self.node_type == "timer"

    @property
    def is_end(self) -> bool:
        return self.node_type == "end"


@dataclass(frozen=True)
class TaskflowDefinition:
    """完整解析后的 taskflow DAG。由 parse_taskflow_yaml() 产出。"""

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

    @property
    def terminal_node_ids(self) -> tuple[str, ...]:
        from_node_ids = {transition.from_node for transition in self.transitions}
        loop_child_ids = {
            child_id
            for node in self.nodes
            if node.is_loop
            for child_id in node.children
        }
        return tuple(
            node.node_id
            for node in self.nodes
            if node.node_id not in loop_child_ids
            and (node.is_end or node.node_id not in from_node_ids)
        )

    def summary(self) -> dict[str, Any]:
        """轻量摘要，用于日志和状态上报。"""

        return {
            "start_node": self.start_node,
            "app_execution_id": self.app_execution_id,
            "node_count": len(self.nodes),
            "worker_count": len(self.worker_nodes),
            "transition_count": len(self.transitions),
            "node_ids": list(self.node_ids),
            "terminal_node_id": self.terminal_node_ids[0]
            if len(self.terminal_node_ids) == 1
            else None,
            "terminal_node_ids": list(self.terminal_node_ids),
            "terminal_node_count": len(self.terminal_node_ids),
        }
