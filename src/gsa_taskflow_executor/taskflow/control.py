"""Taskflow 取消模型。

取消消息不走 taskflow FIFO 队列（否则长 GDK 调用会挡住取消本身）。
TaskflowExecutionController 保存轻量状态，cancel_gdk_command 负责终止 worker 子进程。

取消流程:
1. MQTT cancel topic → handle_taskflow_cancel_message (paho 回调)
2. parse_taskflow_cancel_request() 解析 JSON
3. controller.request_cancel() → 标记取消 + 调 cancel_gdk_command kill worker
4. 调度器轮询 current_cancellation() → 协作式提前返回 CANCELED
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from threading import Lock
from typing import Any

TASKFLOW_CANCELLED_CODE = "TASKFLOW_CANCELLED"
TASKFLOW_CANCEL_REQUEST_TYPE = "cancel_taskflow"


@dataclass(frozen=True)
class TaskflowCancelRequest:
    """解析后的取消请求。"""
    request_id: str
    app_execution_id: str | None  # None 时匹配当前活跃 taskflow
    reason: str
    topic: str
    received_at: str


@dataclass(frozen=True)
class TaskflowCancellation:
    """活跃的取消标记。调度器轮询此对象判断是否提前终止。"""
    app_execution_id: str
    request_id: str
    reason: str
    requested_at: str

    def to_detail(self) -> dict[str, object]:
        """转为 error detail dict（含 cancel 上下文）。"""
        return {
            "error": "Taskflow cancelled",
            "error_code": TASKFLOW_CANCELLED_CODE,
            "error_stage": "taskflow_cancelled",
            "cancelled": True,
            "cancel_state": "CANCELED",
            "cancel_reason": self.reason,
            "cancel_request_id": self.request_id,
            "cancel_requested_at": self.requested_at,
        }


CancelGdkCommand = Callable[[str], Mapping[str, object]]


class TaskflowExecutionController:
    """Track the single active taskflow and cooperative cancellation requests.

    取消消息不能走 Taskflow FIFO 队列，否则长时间 GDK 调用会挡住取消本身。
    本控制器只保存轻量状态；真正的 GDK 阻塞调用由注入的 cancel_gdk_command 负责终止 worker。
    """

    def __init__(
        self,
        *,
        cancel_gdk_command: CancelGdkCommand | None = None,
    ) -> None:
        self._lock = Lock()
        self._cancel_gdk_command = cancel_gdk_command
        self._active_app_execution_id: str | None = None
        self._active_cancellation: TaskflowCancellation | None = None
        self._pending_cancellations: dict[str, TaskflowCancellation] = {}

    def start_execution(self, app_execution_id: str) -> TaskflowCancellation | None:
        with self._lock:
            self._active_app_execution_id = app_execution_id
            self._active_cancellation = self._pending_cancellations.pop(app_execution_id, None)
            return self._active_cancellation

    def finish_execution(self, app_execution_id: str) -> None:
        with self._lock:
            if self._active_app_execution_id != app_execution_id:
                return
            self._active_app_execution_id = None
            self._active_cancellation = None

    def current_cancellation(self, app_execution_id: str) -> TaskflowCancellation | None:
        with self._lock:
            if self._active_app_execution_id != app_execution_id:
                return None
            return self._active_cancellation

    def diagnostics(self) -> dict[str, object]:
        """返回当前执行/取消状态快照，供 health-check 和现场排障读取。"""
        with self._lock:
            return {
                "active_app_execution_id": self._active_app_execution_id,
                "active_cancellation": (
                    self._active_cancellation.to_detail()
                    if self._active_cancellation is not None
                    else None
                ),
                "pending_cancellation_count": len(self._pending_cancellations),
                "pending_cancellation_app_execution_ids": sorted(self._pending_cancellations),
            }

    def request_cancel(self, request: TaskflowCancelRequest) -> dict[str, object]:
        gdk_cancel_reason: str | None = None
        with self._lock:
            target_app_execution_id = request.app_execution_id or self._active_app_execution_id
            if target_app_execution_id is None:
                return {
                    "accepted": False,
                    "state": "NO_ACTIVE_TASKFLOW",
                    "request_id": request.request_id,
                    "reason": request.reason,
                }

            cancellation = TaskflowCancellation(
                app_execution_id=target_app_execution_id,
                request_id=request.request_id,
                reason=request.reason,
                requested_at=request.received_at,
            )
            is_active = self._active_app_execution_id == target_app_execution_id
            if is_active:
                self._active_cancellation = cancellation
                gdk_cancel_reason = request.reason
                state = "CANCELING"
            else:
                self._pending_cancellations[target_app_execution_id] = cancellation
                state = "QUEUED"

        gdk_cancel_result: Mapping[str, object] | None = None
        if gdk_cancel_reason is not None and self._cancel_gdk_command is not None:
            gdk_cancel_result = self._cancel_gdk_command(gdk_cancel_reason)

        result: dict[str, object] = {
            "accepted": True,
            "state": state,
            "app_execution_id": target_app_execution_id,
            "request_id": request.request_id,
            "reason": request.reason,
        }
        if gdk_cancel_result is not None:
            result["gdk_cancel_result"] = dict(gdk_cancel_result)
        return result


def parse_taskflow_cancel_request(
    *,
    topic: str,
    payload: str,
    topic_filter: str,
) -> TaskflowCancelRequest:
    decoded: Mapping[str, Any] = {}
    stripped_payload = payload.strip()
    if stripped_payload:
        try:
            raw = json.loads(stripped_payload)
        except json.JSONDecodeError as error:
            raise ValueError(f"Taskflow cancel request is not valid JSON: {error.msg}") from error
        if not isinstance(raw, Mapping):
            raise ValueError("Taskflow cancel request must be a JSON object")
        decoded = raw

    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != TASKFLOW_CANCEL_REQUEST_TYPE:
        raise ValueError(f"Unsupported taskflow control request type: {request_type}")

    app_execution_id = (
        read_optional_string(decoded, "app_execution_id")
        or read_optional_string(decoded, "appExecutionId")
        or extract_single_level_wildcard(topic_filter, topic)
    )
    received_at = datetime.now(timezone.utc).isoformat()
    request_id = (
        read_optional_string(decoded, "requestId")
        or read_optional_string(decoded, "request_id")
        or f"cancel-{received_at}"
    )
    reason = read_optional_string(decoded, "reason") or "cancel requested"
    return TaskflowCancelRequest(
        request_id=request_id,
        app_execution_id=app_execution_id,
        reason=reason,
        topic=topic,
        received_at=received_at,
    )


def read_optional_string(value: Mapping[str, Any], key: str) -> str | None:
    raw = value.get(key)
    if not isinstance(raw, str):
        return None
    stripped = raw.strip()
    return stripped or None


def extract_single_level_wildcard(topic_filter: str, topic: str) -> str | None:
    filter_parts = topic_filter.split("/")
    topic_parts = topic.split("/")
    if len(filter_parts) != len(topic_parts):
        return None
    for filter_part, topic_part in zip(filter_parts, topic_parts, strict=True):
        if filter_part == "+":
            return topic_part.strip() or None
        if filter_part != topic_part:
            return None
    return None
