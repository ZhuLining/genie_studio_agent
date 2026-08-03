from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from time import perf_counter
from typing import Literal

from gsa_taskflow_executor.skills.runtime import (
    SkillExecutionContext,
    SkillRuntime,
    SkillRuntimeError,
)
from gsa_taskflow_executor.taskflow.parser import (
    TaskflowDefinition,
    TaskflowNode,
    TaskflowTransition,
)
from gsa_taskflow_executor.taskflow.variables import VariableStore

NodeOutcome = Literal["success", "error"]
NodeExecutionStatus = Literal["running", "success", "error"]


class TaskflowScheduleError(RuntimeError):
    """Raised when a parsed taskflow cannot be scheduled safely."""


@dataclass(frozen=True)
class NodeRunResult:
    outcome: NodeOutcome = "success"
    detail: dict[str, object] | None = None
    outputs: dict[str, object] | None = None


@dataclass(frozen=True)
class ScheduledNodeEvent:
    node_id: str
    node_type: str
    skill_name: str | None
    outcome: NodeOutcome
    step_index: int


@dataclass(frozen=True)
class NodeExecutionEvent:
    app_execution_id: str
    node: TaskflowNode
    status: NodeExecutionStatus
    started_at: str
    finished_at: str | None = None
    duration_ms: int | None = None
    result: NodeRunResult | None = None
    variables: dict[str, object] | None = None


@dataclass(frozen=True)
class ScheduleResult:
    app_execution_id: str
    terminal_node_id: str
    outcome: NodeOutcome
    events: tuple[ScheduledNodeEvent, ...]
    variables: dict[str, object]

    @property
    def visited_node_ids(self) -> tuple[str, ...]:
        return tuple(event.node_id for event in self.events)

    def summary(self) -> dict[str, object]:
        return {
            "app_execution_id": self.app_execution_id,
            "terminal_node_id": self.terminal_node_id,
            "outcome": self.outcome,
            "visited_node_ids": list(self.visited_node_ids),
            "step_count": len(self.events),
            "variables": self.variables,
        }


NodeRunner = Callable[[TaskflowNode, VariableStore], NodeRunResult]
NodeExecutionEventHandler = Callable[[NodeExecutionEvent], None]


class SkillRuntimeNodeRunner:
    """Node runner backed by the GDK skill runtime."""

    def __init__(
        self,
        app_execution_id: str,
        mode: str = "gdk",
        runtime: SkillRuntime | None = None,
    ) -> None:
        self.app_execution_id = app_execution_id
        self.mode = mode
        self.runtime = runtime or SkillRuntime()

    def __call__(self, node: TaskflowNode, variable_store: VariableStore) -> NodeRunResult:
        try:
            result = self.runtime.run(
                node,
                SkillExecutionContext(
                    app_execution_id=self.app_execution_id,
                    variable_store=variable_store,
                    mode=self.mode,
                ),
            )
        except SkillRuntimeError as error:
            detail: dict[str, object] = {"error": str(error)}
            if error.detail is not None:
                detail.update(error.detail)
            return NodeRunResult(outcome="error", detail=detail)

        return NodeRunResult(
            outcome=result.outcome,
            detail=result.detail,
            outputs=result.outputs,
        )


class TaskflowScheduler:
    """Linear DAG scheduler for MVP taskflows."""

    def __init__(
        self,
        definition: TaskflowDefinition,
        node_runner: NodeRunner | None = None,
        variable_store: VariableStore | None = None,
        node_event_handler: NodeExecutionEventHandler | None = None,
        max_steps: int | None = None,
    ) -> None:
        self.definition = definition
        self.node_runner = node_runner or SkillRuntimeNodeRunner(definition.app_execution_id)
        self.variable_store = variable_store or VariableStore()
        self.seed_system_variables()
        self.node_event_handler = node_event_handler
        self.max_steps = max_steps or (len(definition.nodes) + 1)
        self.nodes_by_id = {node.node_id: node for node in definition.nodes}
        self.transitions_by_from = build_transition_index(definition.transitions)

    def run(self) -> ScheduleResult:
        current_node_id = self.definition.start_node
        visited: set[str] = set()
        events: list[ScheduledNodeEvent] = []

        for step_index in range(self.max_steps):
            if current_node_id in visited:
                raise TaskflowScheduleError(f"检测到循环执行节点: {current_node_id}")
            visited.add(current_node_id)

            node = self.nodes_by_id.get(current_node_id)
            if node is None:
                raise TaskflowScheduleError(f"节点不存在: {current_node_id}")

            started_at = utc_now_iso()
            started_monotonic = perf_counter()
            self.emit_node_event(
                NodeExecutionEvent(
                    app_execution_id=self.definition.app_execution_id,
                    node=node,
                    status="running",
                    started_at=started_at,
                )
            )
            result = self.node_runner(node, self.variable_store)
            self.write_node_variables(node, result)
            finished_at = utc_now_iso()
            duration_ms = max(0, int((perf_counter() - started_monotonic) * 1000))
            self.emit_node_event(
                NodeExecutionEvent(
                    app_execution_id=self.definition.app_execution_id,
                    node=node,
                    status=result.outcome,
                    started_at=started_at,
                    finished_at=finished_at,
                    duration_ms=duration_ms,
                    result=result,
                    variables=self.variable_store.snapshot()["variables"],
                )
            )
            events.append(
                ScheduledNodeEvent(
                    node_id=node.node_id,
                    node_type=node.node_type,
                    skill_name=node.skill_name,
                    outcome=result.outcome,
                    step_index=step_index,
                )
            )

            if result.outcome == "error":
                return ScheduleResult(
                    app_execution_id=self.definition.app_execution_id,
                    terminal_node_id=node.node_id,
                    outcome="error",
                    events=tuple(events),
                    variables=self.variable_store.snapshot()["variables"],
                )

            next_node_id = self.next_node_id(node.node_id, result.outcome)
            if next_node_id is None:
                return ScheduleResult(
                    app_execution_id=self.definition.app_execution_id,
                    terminal_node_id=node.node_id,
                    outcome="success",
                    events=tuple(events),
                    variables=self.variable_store.snapshot()["variables"],
                )

            current_node_id = next_node_id

        raise TaskflowScheduleError(f"DAG 调度超过最大步数: {self.max_steps}")

    def next_node_id(self, node_id: str, outcome: NodeOutcome) -> str | None:
        transitions = [
            transition
            for transition in self.transitions_by_from.get(node_id, ())
            if transition.outcome == outcome
        ]
        if not transitions:
            return None
        if len(transitions) > 1:
            raise TaskflowScheduleError(f"节点 {node_id} 的 {outcome} transition 不唯一")
        return transitions[0].to_node

    def write_node_variables(self, node: TaskflowNode, result: NodeRunResult) -> None:
        output_var = node.output_var or node.node_id
        detail: dict[str, object] = {
            "status": result.outcome,
            "node_type": node.node_type,
        }
        if result.detail is not None:
            detail.update(result.detail)
        if result.outputs is not None:
            detail["outputs"] = result.outputs
        self.variable_store.set_node_detail(output_var, detail)

    def emit_node_event(self, event: NodeExecutionEvent) -> None:
        if self.node_event_handler is not None:
            self.node_event_handler(event)

    def seed_system_variables(self) -> None:
        if "system" in self.variable_store.variables:
            return
        # 系统变量不是画布节点产物，但下游代码节点需要通过同一变量空间稳定引用。
        self.variable_store.set_node_detail(
            "system",
            {
                "status": "success",
                "node_type": "system",
                "outputs": {
                    "timestamp": utc_now_iso(),
                    "app_execution_id": self.definition.app_execution_id,
                },
            },
        )


def build_transition_index(
    transitions: tuple[TaskflowTransition, ...],
) -> dict[str, tuple[TaskflowTransition, ...]]:
    buckets: dict[str, list[TaskflowTransition]] = {}
    for transition in transitions:
        buckets.setdefault(transition.from_node, []).append(transition)
    return {node_id: tuple(items) for node_id, items in buckets.items()}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()
