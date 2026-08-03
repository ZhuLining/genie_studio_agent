from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from datetime import datetime, timezone
from typing import Any, Literal

from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.taskflow.parser import TaskflowDefinition
from gsa_taskflow_executor.taskflow.scheduler import NodeExecutionEvent, ScheduleResult

TaskflowRuntimeState = Literal["RUNNING", "OVER", "ERROR"]
StatusPublisher = Callable[[Mapping[str, Any]], None]


class TaskflowStatusReporter:
    """Publish executor status payloads compatible with the GSA client."""

    def __init__(
        self,
        settings: ExecutorSettings,
        publish_status: StatusPublisher,
    ) -> None:
        self.settings = settings
        self.publish_status = publish_status

    def publish_execution_started(self, definition: TaskflowDefinition) -> None:
        self.publish_status(
            self.build_base_payload(
                app_execution_id=definition.app_execution_id,
                task_state="RUNNING",
                timestamp=utc_now_iso(),
                extra={
                    "start_node": definition.start_node,
                    "node_count": len(definition.nodes),
                    "worker_count": len(definition.worker_nodes),
                },
            )
        )

    def publish_node_event(self, event: NodeExecutionEvent) -> None:
        task_state = map_node_status_to_task_state(event.status)
        payload = self.build_base_payload(
            app_execution_id=event.app_execution_id,
            task_state=task_state,
            timestamp=event.finished_at or event.started_at,
            extra={"sub_task": build_sub_task_payload(event, task_state)},
        )
        self.publish_status(payload)

    def publish_execution_finished(self, result: ScheduleResult) -> None:
        task_state: TaskflowRuntimeState = "OVER" if result.outcome == "success" else "ERROR"
        self.publish_status(
            self.build_base_payload(
                app_execution_id=result.app_execution_id,
                task_state=task_state,
                timestamp=utc_now_iso(),
                extra={
                    "terminal_node_id": result.terminal_node_id,
                    "visited_node_ids": list(result.visited_node_ids),
                    "step_count": len(result.events),
                    "variables": deepcopy(result.variables),
                },
            )
        )

    def publish_execution_error(
        self,
        *,
        message: str,
        app_execution_id: str | None = None,
    ) -> None:
        self.publish_status(
            self.build_base_payload(
                app_execution_id=app_execution_id,
                task_state="ERROR",
                timestamp=utc_now_iso(),
                extra={"error_msg": message, "error": message},
            )
        )

    def build_base_payload(
        self,
        *,
        app_execution_id: str | None,
        task_state: TaskflowRuntimeState,
        timestamp: str,
        extra: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "aid": self.settings.executor_aid,
            "task_state": task_state,
            "status": task_state,
            "timestamp": timestamp,
            "timestamp_ms": iso_to_timestamp_ms(timestamp),
            "executor_mode": self.settings.executor_mode,
        }
        if app_execution_id:
            payload["app_execution_id"] = app_execution_id
        if extra:
            payload.update(deepcopy(dict(extra)))
        return payload


def build_sub_task_payload(
    event: NodeExecutionEvent,
    state: TaskflowRuntimeState,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "node_id": event.node.node_id,
        "node_name": event.node.node_id,
        "node_type": event.node.node_type,
        "skill_name": event.node.skill_name,
        "state": state,
        "status": state,
        "started_at": event.started_at,
    }
    if event.finished_at is not None:
        payload["finished_at"] = event.finished_at
    if event.duration_ms is not None:
        payload["duration_ms"] = event.duration_ms
    if event.result is not None:
        payload["detail"] = deepcopy(event.result.detail or {})
        payload["outputs"] = deepcopy(event.result.outputs or {})
        error_message = read_error_message(event.result.detail)
        if error_message:
            payload["error_msg"] = error_message
            payload["error"] = error_message
        error_code = read_string_field(event.result.detail, "error_code")
        if error_code:
            payload["error_code"] = error_code
        error_stage = read_string_field(event.result.detail, "error_stage")
        if error_stage:
            payload["error_stage"] = error_stage
    if event.variables is not None:
        payload["variables"] = deepcopy(event.variables)
    return payload


def map_node_status_to_task_state(status: str) -> TaskflowRuntimeState:
    if status == "running":
        return "RUNNING"
    if status == "success":
        return "OVER"
    return "ERROR"


def read_error_message(detail: Mapping[str, object] | None) -> str | None:
    if detail is None:
        return None
    value = detail.get("error")
    return value if isinstance(value, str) and value else None


def read_string_field(detail: Mapping[str, object] | None, key: str) -> str | None:
    if detail is None:
        return None
    value = detail.get(key)
    return value if isinstance(value, str) and value else None


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def iso_to_timestamp_ms(value: str) -> int:
    return int(datetime.fromisoformat(value).timestamp() * 1000)
