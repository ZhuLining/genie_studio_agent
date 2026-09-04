"""MQTT 状态上报。

TaskflowStatusReporter 发布 RUNNING/OVER/ERROR/CANCELED/STOP_UNCONFIRMED 状态载荷。
StatusSequence 生成单调递增序列号，保证桌面端即使 MQTT 乱序也能正确排序。
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from datetime import datetime, timezone
from threading import Lock
from typing import Any, Literal

from gsa_taskflow_executor.gdk.recovery import (
    STOP_UNCONFIRMED_STATE,
    current_gdk_recovery_requirement,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.payload_sanitizer import (
    PayloadSanitizerConfig,
    sanitize_status_payload,
)
from gsa_taskflow_executor.taskflow.control import TASKFLOW_CANCELLED_CODE
from gsa_taskflow_executor.taskflow.models import TaskflowDefinition
from gsa_taskflow_executor.taskflow.scheduler import NodeExecutionEvent, ScheduleResult

TaskflowRuntimeState = Literal["RUNNING", "OVER", "ERROR", "CANCELED", "STOP_UNCONFIRMED"]
StatusPublisher = Callable[[Mapping[str, Any]], None]


class StatusSequence:
    """线程安全状态序号；MQTT 重连或乱序时客户端可据此丢弃旧状态。"""

    def __init__(self) -> None:
        self._lock = Lock()
        self._next_value = 1

    def next(self) -> int:
        with self._lock:
            value = self._next_value
            self._next_value += 1
            return value


class TaskflowStatusReporter:
    """Publish executor status payloads compatible with the GSA client."""

    def __init__(
        self,
        settings: ExecutorSettings,
        publish_status: StatusPublisher,
        status_sequence: StatusSequence | None = None,
    ) -> None:
        self.settings = settings
        self.publish_status = publish_status
        self.status_sequence = status_sequence or StatusSequence()
        self.payload_sanitizer_config = PayloadSanitizerConfig.from_settings(settings)

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
        sub_task_state = map_node_status_to_sub_task_state(event.status)
        task_state = map_node_status_to_execution_state(event.status)
        payload = self.build_base_payload(
            app_execution_id=event.app_execution_id,
            task_state=task_state,
            timestamp=event.finished_at or event.started_at,
            extra={"sub_task": build_sub_task_payload(event, sub_task_state)},
        )
        self.publish_status(payload)

    def publish_execution_finished(self, result: ScheduleResult) -> None:
        if result.outcome == "success":
            task_state: TaskflowRuntimeState = "OVER"
        elif result.outcome == "cancelled":
            task_state = (
                STOP_UNCONFIRMED_STATE
                if current_gdk_recovery_requirement() is not None
                else "CANCELED"
            )
        else:
            task_state = "ERROR"
        extra: dict[str, Any] = {
            "terminal_node_id": result.terminal_node_id,
            "visited_node_ids": list(result.visited_node_ids),
            "step_count": len(result.events),
            "variables": deepcopy(result.variables),
        }
        if result.outcome == "cancelled":
            recovery_requirement = current_gdk_recovery_requirement()
            extra.update(
                {
                    "cancelled": True,
                    "cancel_state": task_state,
                    "error_code": TASKFLOW_CANCELLED_CODE,
                    "error_msg": "Taskflow cancelled",
                    "error": "Taskflow cancelled",
                }
            )
            if recovery_requirement is not None:
                extra.update(
                    {
                        "stop_state": STOP_UNCONFIRMED_STATE,
                        "robot_stop_confirmed": False,
                        "gdk_recovery": recovery_requirement.to_payload(),
                    }
                )
        self.publish_status(
            self.build_base_payload(
                app_execution_id=result.app_execution_id,
                task_state=task_state,
                timestamp=utc_now_iso(),
                extra=extra,
            )
        )

    def publish_execution_canceling(
        self,
        *,
        app_execution_id: str,
        request_id: str,
        reason: str,
        gdk_cancel_result: Mapping[str, object] | None = None,
    ) -> None:
        extra: dict[str, Any] = {
            "cancelled": True,
            "cancel_state": "CANCELING",
            "cancel_request_id": request_id,
            "cancel_reason": reason,
        }
        if gdk_cancel_result is not None:
            extra["gdk_cancel_result"] = deepcopy(dict(gdk_cancel_result))
            if gdk_cancel_result_requires_stop_confirmation(gdk_cancel_result):
                extra["stop_state"] = STOP_UNCONFIRMED_STATE
                extra["robot_stop_confirmed"] = False
        self.publish_status(
            self.build_base_payload(
                app_execution_id=app_execution_id,
                task_state="RUNNING",
                timestamp=utc_now_iso(),
                extra=extra,
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
            "status_seq": self.status_sequence.next(),
            "executor_mode": self.settings.executor_mode,
        }
        if app_execution_id:
            payload["app_execution_id"] = app_execution_id
        if extra:
            payload.update(deepcopy(dict(extra)))
        return sanitize_status_payload(payload, config=self.payload_sanitizer_config)


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
        if event.result.detail is not None:
            for key in (
                "cancelled",
                "cancel_state",
                "cancel_reason",
                "cancel_request_id",
                "cancel_requested_at",
            ):
                if key in event.result.detail:
                    payload[key] = deepcopy(event.result.detail[key])
    if event.variables is not None:
        payload["variables"] = deepcopy(event.variables)
    return payload


def map_node_status_to_execution_state(status: str) -> TaskflowRuntimeState:
    # 顶层 task_state 表示整条 Taskflow 生命周期；单个节点成功不能发 OVER，
    # 否则客户端会提前把本次执行标记完成并停止跟踪后续节点。
    if status == "cancelled":
        return "CANCELED"
    if status == "error":
        return "ERROR"
    return "RUNNING"


def map_node_status_to_sub_task_state(status: str) -> TaskflowRuntimeState:
    if status == "running":
        return "RUNNING"
    if status == "success":
        return "OVER"
    if status == "cancelled":
        return "CANCELED"
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


def gdk_cancel_result_requires_stop_confirmation(
    result: Mapping[str, object],
) -> bool:
    """判断本次取消是否切断过 GDK worker，若是则机器人停止需要另行确认。"""

    if result.get("called") is not True:
        return False
    nested_result = result.get("result")
    if isinstance(nested_result, Mapping):
        return nested_result.get("error_code") == "GDK_OPERATION_CANCELLED"
    return result.get("error_code") == "GDK_OPERATION_CANCELLED"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def iso_to_timestamp_ms(value: str) -> int:
    return int(datetime.fromisoformat(value).timestamp() * 1000)
