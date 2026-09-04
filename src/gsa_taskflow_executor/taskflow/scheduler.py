"""Taskflow DAG 线性调度器。

从 start_node 开始按 transition 逐节点执行，每个节点:
1. 检查取消 → 跳过并终止
2. 执行节点（worker → SkillRuntime, assign → 写变量, timer → sleep, loop → 迭代子节点）
3. 按 outcome 查找下个节点
4. error/cancelled → 立即终止，无更多 transition → 正常结束

调度器非线程安全，在 taskflow worker 线程内运行。取消通过 cancel_checker 回调跨线程协作。
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from time import perf_counter
from time import sleep as default_sleep
from typing import Literal

from gsa_taskflow_executor.skills.runtime import (
    SkillExecutionContext,
    SkillRuntime,
    SkillRuntimeError,
)
from gsa_taskflow_executor.taskflow.control import (
    TASKFLOW_CANCELLED_CODE,
    TaskflowCancellation,
)
from gsa_taskflow_executor.taskflow.models import (
    TaskflowDefinition,
    TaskflowNode,
    TaskflowTransition,
)
from gsa_taskflow_executor.taskflow.variables import (
    VariableStore,
    VariableStoreError,
    is_variable_reference,
)

# GDK 操作被取消时的错误码
GDK_OPERATION_CANCELLED_CODE = "GDK_OPERATION_CANCELLED"
OUTPUT_CONTRACT_VIOLATION_CODE = "OUTPUT_CONTRACT_VIOLATION"

NodeOutcome = Literal["success", "error", "cancelled"]
NodeExecutionStatus = Literal["running", "success", "error", "cancelled"]


class TaskflowScheduleError(RuntimeError):
    """调度异常：循环引用、节点缺失、步数超限等。"""


# ============================================================
# 执行结果和事件数据模型
# ============================================================


@dataclass(frozen=True)
class NodeRunResult:
    """单节点执行结果。"""
    outcome: NodeOutcome = "success"
    detail: dict[str, object] | None = None
    outputs: dict[str, object] | None = None


@dataclass(frozen=True)
class ScheduledNodeEvent:
    """调度事件（最终状态上报用，每步一个）。"""
    node_id: str
    node_type: str
    skill_name: str | None
    outcome: NodeOutcome
    step_index: int


@dataclass(frozen=True)
class NodeExecutionEvent:
    """节点生命周期事件（实时状态上报用）。每节点发送 running + 最终状态两次。"""
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
    """调度完成结果。"""
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


# 回调类型别名
NodeRunner = Callable[[TaskflowNode, VariableStore], NodeRunResult]
NodeExecutionEventHandler = Callable[[NodeExecutionEvent], None]
Sleep = Callable[[float], None]
CancellationChecker = Callable[[], TaskflowCancellation | None]


# ============================================================
# SkillRuntimeNodeRunner — 将调度器适配到 GDK skill runtime
# ============================================================


class SkillRuntimeNodeRunner:
    """通过 SkillRuntime 执行节点的默认 NodeRunner。"""

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
            return NodeRunResult(
                outcome=error_detail_to_outcome(detail),
                detail=detail,
            )

        return NodeRunResult(
            outcome=result.outcome,
            detail=result.detail,
            outputs=result.outputs,
        )


# ============================================================
# TaskflowScheduler — 线性 DAG 遍历器
# ============================================================


class TaskflowScheduler:
    """从 start_node 沿 transition 遍历执行 DAG。

    Usage::

        scheduler = TaskflowScheduler(
            definition,
            node_runner=SkillRuntimeNodeRunner(app_execution_id, runtime=skill_runtime),
            cancel_checker=lambda: controller.current_cancellation(app_execution_id),
        )
        result = scheduler.run()
    """

    def __init__(
        self,
        definition: TaskflowDefinition,
        node_runner: NodeRunner | None = None,
        variable_store: VariableStore | None = None,
        node_event_handler: NodeExecutionEventHandler | None = None,
        sleep: Sleep | None = None,
        max_steps: int | None = None,
        cancel_checker: CancellationChecker | None = None,
    ) -> None:
        self.definition = definition
        self.node_runner = node_runner or SkillRuntimeNodeRunner(definition.app_execution_id)
        self.variable_store = variable_store or VariableStore()
        self.seed_system_variables()
        self.node_event_handler = node_event_handler
        self.sleep = sleep or default_sleep
        self.max_steps = max_steps or calculate_default_max_steps(definition)
        self.cancel_checker = cancel_checker
        self.step_index = 0
        # O(1) 查找索引
        self.nodes_by_id = {node.node_id: node for node in definition.nodes}
        self.transitions_by_from = build_transition_index(definition.transitions)

    def run(self) -> ScheduleResult:
        """执行 taskflow DAG，返回 ScheduleResult。"""
        current_node_id = self.definition.start_node
        visited: set[str] = set()
        events: list[ScheduledNodeEvent] = []
        self.step_index = 0

        while self.step_index < self.max_steps:
            # 循环检测：节点重复出现说明 taskflow 有环路
            if current_node_id in visited:
                raise TaskflowScheduleError(f"检测到循环执行节点: {current_node_id}")
            visited.add(current_node_id)

            node = self.nodes_by_id.get(current_node_id)
            if node is None:
                raise TaskflowScheduleError(f"节点不存在: {current_node_id}")

            # 取消检查
            cancellation = self.current_cancellation()
            if cancellation is not None:
                result, node_events = self.execute_cancelled_node(node, cancellation)
                events.extend(node_events)
                return ScheduleResult(
                    app_execution_id=self.definition.app_execution_id,
                    terminal_node_id=node.node_id,
                    outcome="cancelled",
                    events=tuple(events),
                    variables=self.variable_store.snapshot()["variables"],
                )

            # 执行当前节点
            result, node_events = self.execute_node(node)
            events.extend(node_events)

            # error/cancelled → 终止
            if result.outcome in {"error", "cancelled"}:
                return ScheduleResult(
                    app_execution_id=self.definition.app_execution_id,
                    terminal_node_id=node.node_id,
                    outcome=result.outcome,
                    events=tuple(events),
                    variables=self.variable_store.snapshot()["variables"],
                )

            # 查找下一个节点
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

    # ---- 节点执行分发 ----

    def execute_node(self, node: TaskflowNode) -> tuple[NodeRunResult, list[ScheduledNodeEvent]]:
        """按节点类型分发。loop 走多迭代路径。"""
        if node.is_loop:
            return self.execute_loop_node(node)
        return self.execute_node_once(node)

    def execute_node_once(
        self,
        node: TaskflowNode,
    ) -> tuple[NodeRunResult, list[ScheduledNodeEvent]]:
        """执行单个非 loop 节点。发 running → 执行 → 写变量 → 发最终事件。"""
        step_index = self.reserve_step_index()
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

        result = self.execute_regular_node(node)
        result = self.record_node_result(node, result)

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
        return result, [
            ScheduledNodeEvent(
                node_id=node.node_id,
                node_type=node.node_type,
                skill_name=node.skill_name,
                outcome=result.outcome,
                step_index=step_index,
            )
        ]

    def execute_regular_node(self, node: TaskflowNode) -> NodeRunResult:
        """执行非 loop 节点：end → 终止记录，timer → sleep，其它 → node_runner。"""
        if node.is_loop:
            raise TaskflowScheduleError("v1 暂不支持嵌套循环")
        if node.is_end:
            return NodeRunResult(
                outcome="success",
                detail={"terminal": True},
                outputs={"terminal": True},
            )
        if node.is_timer:
            return self.execute_timer_node(node)
        # 执行前再检查一次取消（上一节点持锁期间可能收到取消）
        cancellation = self.current_cancellation()
        if cancellation is not None:
            return build_cancelled_node_result(cancellation)
        return self.node_runner(node, self.variable_store)

    # ---- Timer 执行 ----

    def execute_timer_node(self, node: TaskflowNode) -> NodeRunResult:
        """执行 timer 节点。有 cancel_checker 时每 100ms 轮询取消，否则直接 sleep。"""
        duration = float(node.duration or 0)
        started = perf_counter()

        if self.cancel_checker is None:
            self.sleep(duration)
        else:
            deadline = started + duration
            while True:
                cancellation = self.current_cancellation()
                if cancellation is not None:
                    return build_cancelled_node_result(
                        cancellation,
                        extra={"timer_interrupted": True, "duration": duration},
                    )
                remaining = deadline - perf_counter()
                if remaining <= 0:
                    break
                self.sleep(min(remaining, 0.1))

        elapsed_seconds = max(0.0, perf_counter() - started)
        outputs: dict[str, object] = {
            "duration": duration,
            "elapsed_seconds": elapsed_seconds,
        }
        return NodeRunResult(
            outcome="success",
            detail={
                "timer_mode": node.timer_mode or "rel",
                **outputs,
            },
            outputs=outputs,
        )

    # ---- Loop 执行 ----

    def execute_loop_node(
        self,
        node: TaskflowNode,
    ) -> tuple[NodeRunResult, list[ScheduledNodeEvent]]:
        """执行 loop 节点：迭代 child 线性链 iteration_max 次。

        每次迭代前检查取消；子节点 error/cancelled 时提前终止。
        """
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

        loop_events: list[ScheduledNodeEvent] = []
        iteration_results: list[dict[str, object]] = []
        iterations: list[dict[str, object]] = []
        completed_iterations = 0
        child_nodes = self.ordered_loop_children(node)
        iteration_max = node.iteration_max or 0

        result = NodeRunResult(outcome="success")
        for iteration_index in range(iteration_max):
            cancellation = self.current_cancellation()
            if cancellation is not None:
                cancelled_loop_outputs = {
                    "contract_version": 2,
                    "loop_mode": node.loop_mode or "count",
                    "iteration_max": iteration_max,
                    "completed_iterations": completed_iterations,
                    "iteration_results": iteration_results,
                    "iterations": iterations,
                    "last_iteration": iterations[-1] if iterations else None,
                }
                result = build_cancelled_node_result(
                    cancellation,
                    extra={
                        **cancelled_loop_outputs,
                        "interrupted_iteration": iteration_index + 1,
                    },
                    outputs=cancelled_loop_outputs,
                )
                break

            child_result, child_events, failed_child_id = self.execute_loop_iteration(child_nodes)
            loop_events.extend(child_events)

            if child_result.outcome in {"error", "cancelled"}:
                child_outcome = child_result.outcome
                iteration_results.append(
                    {
                        "iteration": iteration_index + 1,
                        "outcome": child_outcome,
                        "failed_child_node": failed_child_id or "",
                    }
                )
                iterations.append(
                    self.build_loop_iteration_snapshot(
                        iteration_index=iteration_index,
                        outcome=child_outcome,
                        child_nodes=child_nodes,
                        failed_child_id=failed_child_id,
                    )
                )
                failed_loop_outputs: dict[str, object] = {
                    "contract_version": 2,
                    "loop_mode": node.loop_mode or "count",
                    "iteration_max": iteration_max,
                    "completed_iterations": completed_iterations,
                    "iteration_results": iteration_results,
                    "iterations": iterations,
                    "last_iteration": iterations[-1] if iterations else None,
                }
                result = NodeRunResult(
                    outcome=child_outcome,
                    detail={
                        **failed_loop_outputs,
                        "failed_iteration": iteration_index + 1,
                        "failed_child_node": failed_child_id or "",
                    },
                    outputs=failed_loop_outputs,
                )
                break

            completed_iterations += 1
            iteration_results.append(
                {
                    "iteration": iteration_index + 1,
                    "outcome": "success",
                }
            )
            iterations.append(
                self.build_loop_iteration_snapshot(
                    iteration_index=iteration_index,
                    outcome="success",
                    child_nodes=child_nodes,
                    failed_child_id=None,
                )
            )
        else:
            # 全部迭代成功完成
            success_loop_outputs = {
                "contract_version": 2,
                "loop_mode": node.loop_mode or "count",
                "iteration_max": iteration_max,
                "completed_iterations": completed_iterations,
                "iteration_results": iteration_results,
                "iterations": iterations,
                "last_iteration": iterations[-1] if iterations else None,
            }
            result = NodeRunResult(
                outcome="success",
                detail=success_loop_outputs,
                outputs=success_loop_outputs,
            )

        result = self.record_node_result(node, result)
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
        loop_events.append(
            ScheduledNodeEvent(
                node_id=node.node_id,
                node_type=node.node_type,
                skill_name=node.skill_name,
                outcome=result.outcome,
                step_index=self.reserve_step_index(),
            )
        )
        return result, loop_events

    def execute_loop_iteration(
        self,
        child_nodes: tuple[TaskflowNode, ...],
    ) -> tuple[NodeRunResult, list[ScheduledNodeEvent], str | None]:
        """执行一次 loop body 迭代。返回 (结果, 事件列表, 失败子节点ID)。"""
        events: list[ScheduledNodeEvent] = []
        last_result = NodeRunResult(outcome="success")
        for child_node in child_nodes:
            cancellation = self.current_cancellation()
            if cancellation is not None:
                cancelled_result = build_cancelled_node_result(cancellation)
                child_result, child_events = self.execute_cancelled_node(
                    child_node,
                    cancellation,
                )
                events.extend(child_events)
                return cancelled_result, events, child_node.node_id

            last_result, child_events = self.execute_node_once(child_node)
            events.extend(child_events)
            if last_result.outcome in {"error", "cancelled"}:
                return last_result, events, child_node.node_id

        return last_result, events, None

    def build_loop_iteration_snapshot(
        self,
        *,
        iteration_index: int,
        outcome: NodeOutcome,
        child_nodes: Sequence[TaskflowNode],
        failed_child_id: str | None,
    ) -> dict[str, object]:
        """复制当轮子节点 detail，形成 loop v2 变量历史。

        子节点自己的变量仍保留“最新值”语义，避免破坏旧引用；历史值只挂在
        $.variables.<loop>.detail.outputs.iterations 下，供后续代码节点显式索引。
        """

        variables = self.variable_store.snapshot()["variables"]
        nodes: dict[str, object] = {}
        for child_node in child_nodes:
            output_var = child_node.output_var or child_node.node_id
            node_scope = variables.get(output_var)
            if not isinstance(node_scope, Mapping):
                continue
            detail = node_scope.get("detail")
            if not isinstance(detail, Mapping):
                continue
            nodes[child_node.node_id] = {
                "node_id": child_node.node_id,
                "output_var": output_var,
                "detail": dict(detail),
            }

        snapshot: dict[str, object] = {
            "index": iteration_index,
            "iteration": iteration_index + 1,
            "outcome": outcome,
            "nodes": nodes,
        }
        if failed_child_id:
            snapshot["failed_child_node"] = failed_child_id
        return snapshot

    # ---- 取消处理 ----

    def execute_cancelled_node(
        self,
        node: TaskflowNode,
        cancellation: TaskflowCancellation,
    ) -> tuple[NodeRunResult, list[ScheduledNodeEvent]]:
        """记录取消节点（不实际执行）。"""
        return self.execute_node_once_with_result(
            node,
            build_cancelled_node_result(cancellation),
        )

    def execute_node_once_with_result(
        self,
        node: TaskflowNode,
        result: NodeRunResult,
    ) -> tuple[NodeRunResult, list[ScheduledNodeEvent]]:
        """用预置结果记录节点（取消时用，耗时 0）。"""
        step_index = self.reserve_step_index()
        started_at = utc_now_iso()
        self.emit_node_event(
            NodeExecutionEvent(
                app_execution_id=self.definition.app_execution_id,
                node=node,
                status="running",
                started_at=started_at,
            )
        )
        result = self.record_node_result(node, result)
        finished_at = utc_now_iso()
        self.emit_node_event(
            NodeExecutionEvent(
                app_execution_id=self.definition.app_execution_id,
                node=node,
                status=result.outcome,
                started_at=started_at,
                finished_at=finished_at,
                duration_ms=0,
                result=result,
                variables=self.variable_store.snapshot()["variables"],
            )
        )
        return result, [
            ScheduledNodeEvent(
                node_id=node.node_id,
                node_type=node.node_type,
                skill_name=node.skill_name,
                outcome=result.outcome,
                step_index=step_index,
            )
        ]

    # ---- 图遍历辅助 ----

    def ordered_loop_children(self, loop_node: TaskflowNode) -> tuple[TaskflowNode, ...]:
        """按 transition 拓扑排序 loop body 子节点（从入口到出口线性链）。"""
        child_ids = set(loop_node.children)
        incoming_count = {child_id: 0 for child_id in loop_node.children}
        next_by_source: dict[str, str] = {}
        for transition in self.definition.transitions:
            if transition.from_node not in child_ids or transition.to_node not in child_ids:
                continue
            incoming_count[transition.to_node] += 1
            next_by_source[transition.from_node] = transition.to_node

        entry_ids = [child_id for child_id, count in incoming_count.items() if count == 0]
        if len(entry_ids) != 1:
            raise TaskflowScheduleError(f"循环 {loop_node.node_id} 内部入口不唯一")

        ordered: list[TaskflowNode] = []
        visited: set[str] = set()
        current_id: str | None = entry_ids[0]
        while current_id is not None and current_id not in visited:
            child_node = self.nodes_by_id.get(current_id)
            if child_node is None:
                raise TaskflowScheduleError(f"循环内部节点不存在: {current_id}")
            ordered.append(child_node)
            visited.add(current_id)
            current_id = next_by_source.get(current_id)

        if len(ordered) != len(loop_node.children):
            raise TaskflowScheduleError(f"循环 {loop_node.node_id} 内部节点未全部连通")
        return tuple(ordered)

    # ---- 内部辅助方法 ----

    def reserve_step_index(self) -> int:
        """分配下一步序号，超限抛异常。"""
        if self.step_index >= self.max_steps:
            raise TaskflowScheduleError(f"DAG 调度超过最大步数: {self.max_steps}")
        step_index = self.step_index
        self.step_index += 1
        return step_index

    def next_node_id(self, node_id: str, outcome: NodeOutcome) -> str | None:
        """按 outcome 匹配 transition 找下一个节点。多匹配抛异常。"""
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
        """将节点执行结果写入 VariableStore。key = output_var 或 node_id。"""
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

    def record_node_result(self, node: TaskflowNode, result: NodeRunResult) -> NodeRunResult:
        """写入节点结果，并在 success 后立刻校验 output_contract。

        契约校验必须发生在变量写入之后，因为 required_paths 使用
        $.variables.* 访问运行时变量空间；失败时覆盖当前节点 detail，让状态回传明确指向本节点。
        """
        self.write_node_variables(node, result)
        contract_result = self.validate_output_contract(node, result)
        if contract_result is result:
            return result
        self.write_node_variables(node, contract_result)
        return contract_result

    def validate_output_contract(self, node: TaskflowNode, result: NodeRunResult) -> NodeRunResult:
        """校验 output_contract.required_paths；缺失时将当前节点转为 error。"""
        if result.outcome != "success":
            return result

        required_paths, invalid_entries = read_required_output_paths(node.output_contract)
        if not required_paths and not invalid_entries:
            return result

        missing_paths: list[str] = []
        for required_path in required_paths:
            try:
                self.variable_store.resolve(required_path)
            except VariableStoreError:
                missing_paths.append(required_path)

        if not missing_paths and not invalid_entries:
            return result

        return build_output_contract_error_result(
            node=node,
            original_result=result,
            required_paths=required_paths,
            missing_paths=missing_paths,
            invalid_entries=invalid_entries,
        )

    def emit_node_event(self, event: NodeExecutionEvent) -> None:
        if self.node_event_handler is not None:
            self.node_event_handler(event)

    def current_cancellation(self) -> TaskflowCancellation | None:
        if self.cancel_checker is None:
            return None
        return self.cancel_checker()

    def seed_system_variables(self) -> None:
        """初始化 $.variables.system 命名空间。

        提供 timestamp 和 app_execution_id，下游代码节点可通过 $.variables.system.* 稳定引用。
        只写一次，不覆盖。
        """
        if "system" in self.variable_store.variables:
            return
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


# ============================================================
# 模块级辅助函数
# ============================================================


def build_transition_index(
    transitions: tuple[TaskflowTransition, ...],
) -> dict[str, tuple[TaskflowTransition, ...]]:
    """构建 from_node → outgoing transitions 的 O(1) 查找索引。"""
    buckets: dict[str, list[TaskflowTransition]] = {}
    for transition in transitions:
        buckets.setdefault(transition.from_node, []).append(transition)
    return {node_id: tuple(items) for node_id, items in buckets.items()}


def build_cancelled_node_result(
    cancellation: TaskflowCancellation,
    *,
    extra: dict[str, object] | None = None,
    outputs: dict[str, object] | None = None,
) -> NodeRunResult:
    """构建取消结果 NodeRunResult。"""
    detail = cancellation.to_detail()
    if extra:
        detail.update(extra)
    return NodeRunResult(outcome="cancelled", detail=detail, outputs=outputs)


def read_required_output_paths(
    output_contract: Mapping[str, object],
) -> tuple[tuple[str, ...], tuple[object, ...]]:
    """读取 output_contract.required_paths，并返回有效路径与异常配置项。"""
    raw_required_paths = output_contract.get("required_paths")
    if raw_required_paths is None:
        return (), ()
    if isinstance(raw_required_paths, str | bytes | bytearray) or not isinstance(
        raw_required_paths,
        Sequence,
    ):
        return (), (raw_required_paths,)

    required_paths: list[str] = []
    invalid_entries: list[object] = []
    for item in raw_required_paths:
        if isinstance(item, str) and is_variable_reference(item):
            required_paths.append(item.strip())
        else:
            invalid_entries.append(item)
    return tuple(required_paths), tuple(invalid_entries)


def build_output_contract_error_result(
    *,
    node: TaskflowNode,
    original_result: NodeRunResult,
    required_paths: tuple[str, ...],
    missing_paths: Sequence[str],
    invalid_entries: Sequence[object],
) -> NodeRunResult:
    """构建 output_contract 校验失败结果。"""
    detail: dict[str, object] = {
        "error": "output_contract required_paths 校验失败",
        "error_code": OUTPUT_CONTRACT_VIOLATION_CODE,
        "error_stage": "output_contract",
        "output_var": node.output_var or node.node_id,
        "required_paths": list(required_paths),
        "missing_paths": list(missing_paths),
        "invalid_required_paths": list(invalid_entries),
        "original_outcome": original_result.outcome,
    }
    if original_result.detail is not None:
        detail["original_detail"] = original_result.detail
    if original_result.outputs is not None:
        detail["original_outputs"] = original_result.outputs
    return NodeRunResult(outcome="error", detail=detail)


def error_detail_to_outcome(detail: Mapping[str, object]) -> NodeOutcome:
    """从错误 detail 判断是 cancelled 还是 error。

    检查顶层 error_code 和嵌套 gdk_result.error_code 中的取消标记。
    """
    error_code = detail.get("error_code")
    if error_code in {TASKFLOW_CANCELLED_CODE, GDK_OPERATION_CANCELLED_CODE}:
        return "cancelled"
    gdk_result = detail.get("gdk_result")
    if isinstance(gdk_result, dict) and gdk_result.get("error_code") in {
        TASKFLOW_CANCELLED_CODE,
        GDK_OPERATION_CANCELLED_CODE,
    }:
        return "cancelled"
    return "error"


def calculate_default_max_steps(definition: TaskflowDefinition) -> int:
    """计算安全步数上限：节点数 + loop 迭代步数 + 1。"""
    loop_child_steps = sum(
        (node.iteration_max or 0) * len(node.children)
        for node in definition.nodes
        if node.is_loop
    )
    return len(definition.nodes) + loop_child_steps + 1


def utc_now_iso() -> str:
    """当前 UTC 时间的 ISO 8601 字符串。"""
    return datetime.now(timezone.utc).isoformat()
