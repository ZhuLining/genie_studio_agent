from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from queue import Empty, Full, Queue
from threading import Lock, Thread
from typing import Final

from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent

MqttMessageHandler = Callable[[TaskflowMessage], None]
DEFAULT_WORKER_STOP_TIMEOUT_SECONDS: Final = 5.0
QUEUE_FULL_POLICY_REJECT: Final = "reject"


class MqttMessageQueueError(RuntimeError):
    """MQTT worker 队列无法接收消息时抛出。"""


class MqttMessageQueueFull(MqttMessageQueueError):
    """MQTT worker 队列已满时抛出。"""


class MqttMessageQueueStopped(MqttMessageQueueError):
    """worker 已停止或正在停止时仍尝试入队。"""


@dataclass(frozen=True)
class QueuedMqttMessage:
    """队列内部消息包装，记录入队时间用于计算排队耗时。"""

    message: TaskflowMessage
    sequence: int
    enqueued_at: str
    enqueued_at_monotonic: float


class MqttMessageWorkerQueue:
    """把 MQTT 消息处理从 paho 回调线程迁移到独立 worker。

    paho 的 on_message 回调应尽快返回，避免真机长动作阻塞 MQTT 网络循环。
    对 Taskflow 这类必须串行的处理器，本队列用单 worker 保持 FIFO 执行语义。
    """

    def __init__(
        self,
        *,
        name: str,
        handler: MqttMessageHandler,
        maxsize: int,
        queue_full_policy: str = QUEUE_FULL_POLICY_REJECT,
        logger: logging.Logger | None = None,
        event_writer: JsonlEventWriter | None = None,
    ) -> None:
        if maxsize <= 0:
            raise ValueError("maxsize must be greater than 0")
        if queue_full_policy != QUEUE_FULL_POLICY_REJECT:
            raise ValueError("queue_full_policy currently only supports reject")

        self.name = name
        self.handler = handler
        self.maxsize = maxsize
        self.queue_full_policy = queue_full_policy
        self.logger = logger or logging.getLogger(__name__)
        self.event_writer = event_writer
        self._queue: Queue[QueuedMqttMessage | object] = Queue(maxsize=maxsize)
        self._stop_item = object()
        self._lock = Lock()
        self._thread: Thread | None = None
        self._stopping = False
        self._next_sequence = 0
        self._enqueued_count = 0
        self._dequeued_count = 0
        self._completed_count = 0
        self._failed_count = 0
        self._rejected_count = 0
        self._last_rejected_at: str | None = None
        self._last_error: str | None = None
        self._last_queue_wait_ms: int | None = None
        self._max_queue_wait_ms: int | None = None
        self._total_queue_wait_ms = 0
        self._active_item: QueuedMqttMessage | None = None
        self._active_started_at: str | None = None
        self._active_started_at_monotonic: float | None = None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stopping = False
            # daemon=True 让 CLI 异常退出时不会被 worker 线程永久挂住。
            self._thread = Thread(
                target=self._run,
                name=self.name,
                daemon=True,
            )
            self._thread.start()

        self.record_event(
            RuntimeEvent(
                event_type="mqtt_message_worker_started",
                message=f"{self.name} started",
                payload={
                    "queue_name": self.name,
                    "maxsize": self.maxsize,
                    "queue_full_policy": self.queue_full_policy,
                },
            )
        )

    def stop(self, timeout: float = DEFAULT_WORKER_STOP_TIMEOUT_SECONDS) -> None:
        with self._lock:
            self._stopping = True
            thread = self._thread
            if thread is None:
                return
            try:
                # 优先用哨兵唤醒 worker；若队列已满，worker 也会通过轮询超时看到 stopping。
                self._queue.put(self._stop_item, timeout=1.0)
            except Full:
                self.logger.warning("%s queue is full while stopping", self.name)

        thread.join(timeout=timeout)
        if thread.is_alive():
            self.logger.warning("%s did not stop within %.1fs", self.name, timeout)
            self.record_event(
                RuntimeEvent(
                    event_type="mqtt_message_worker_stop_timeout",
                    level="warning",
                    message=f"{self.name} did not stop within timeout",
                    payload={"queue_name": self.name, "timeout": timeout},
                )
            )
            return

        with self._lock:
            if self._thread is thread:
                self._thread = None

        self.record_event(
            RuntimeEvent(
                event_type="mqtt_message_worker_stopped",
                message=f"{self.name} stopped",
                payload={"queue_name": self.name},
            )
        )

    def enqueue(self, message: TaskflowMessage) -> int:
        with self._lock:
            if self._stopping:
                raise MqttMessageQueueStopped(f"{self.name} is stopping")
            if self._thread is None or not self._thread.is_alive():
                raise MqttMessageQueueStopped(f"{self.name} is not running")

            sequence = self._next_sequence + 1
            queued_message = QueuedMqttMessage(
                message=message,
                sequence=sequence,
                enqueued_at=utc_now_iso(),
                enqueued_at_monotonic=time.monotonic(),
            )
            try:
                # 入队必须是非阻塞的：队列满时快速失败，不能反向拖住 paho 回调线程。
                self._queue.put_nowait(queued_message)
            except Full as error:
                self._rejected_count += 1
                self._last_rejected_at = utc_now_iso()
                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_queue_full",
                        level="error",
                        message=f"{self.name} queue is full",
                        topic=message.topic,
                        payload={
                            "queue_name": self.name,
                            "maxsize": self.maxsize,
                            "queue_full_policy": self.queue_full_policy,
                            "queued_count": self._queue.qsize(),
                            "rejected_count": self._rejected_count,
                        },
                    )
                )
                raise MqttMessageQueueFull(f"{self.name} queue is full") from error

            self._next_sequence = sequence
            self._enqueued_count += 1
            queued_count = self._queue.qsize()

        self.record_event(
            RuntimeEvent(
                event_type="mqtt_message_queued",
                message=f"{self.name} message queued",
                topic=message.topic,
                payload={
                    "queue_name": self.name,
                    "queued_count": queued_count,
                    "maxsize": self.maxsize,
                    "queue_full_policy": self.queue_full_policy,
                    "sequence": sequence,
                    "payload_bytes": message.payload_bytes,
                },
            )
        )
        return queued_count

    def snapshot(self) -> dict[str, object]:
        """返回队列运行快照，供 CLI diagnostics 和现场排障读取。"""
        with self._lock:
            return self._snapshot_locked()

    def _run(self) -> None:
        while True:
            try:
                # 轮询超时用于处理“停止时队列刚好满，哨兵无法入队”的边界。
                item = self._queue.get(timeout=0.2)
            except Empty:
                if self._stopping:
                    return
                continue
            try:
                if item is self._stop_item:
                    return
                if self._stopping:
                    message = item.message if isinstance(item, QueuedMqttMessage) else None
                    self.record_event(
                        RuntimeEvent(
                            event_type="mqtt_message_discarded_after_stop",
                            level="warning",
                            message=f"{self.name} discarded queued message after stop",
                            topic=message.topic if message is not None else None,
                            payload={"queue_name": self.name},
                        )
                    )
                    continue

                if not isinstance(item, QueuedMqttMessage):
                    raise TypeError(f"{self.name} received an invalid queue item")

                message = item.message
                queue_wait_ms = elapsed_ms_since(item.enqueued_at_monotonic)
                with self._lock:
                    self._dequeued_count += 1
                    self._last_queue_wait_ms = queue_wait_ms
                    self._max_queue_wait_ms = (
                        queue_wait_ms
                        if self._max_queue_wait_ms is None
                        else max(self._max_queue_wait_ms, queue_wait_ms)
                    )
                    self._total_queue_wait_ms += queue_wait_ms
                    self._active_item = item
                    self._active_started_at = utc_now_iso()
                    self._active_started_at_monotonic = time.monotonic()

                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_worker_started_message",
                        message=f"{self.name} started message",
                        topic=message.topic,
                        payload={
                            "queue_name": self.name,
                            "sequence": item.sequence,
                            "queue_wait_ms": queue_wait_ms,
                        },
                    )
                )
                self.handler(message)
                with self._lock:
                    self._completed_count += 1
                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_worker_finished_message",
                        message=f"{self.name} finished message",
                        topic=message.topic,
                        payload={"queue_name": self.name, "sequence": item.sequence},
                    )
                )
            except Exception as error:
                with self._lock:
                    self._failed_count += 1
                    self._last_error = str(error)
                self.logger.exception("%s message handler failed", self.name)
                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_worker_handler_error",
                        level="error",
                        message=str(error),
                        topic=item.message.topic if isinstance(item, QueuedMqttMessage) else None,
                        payload={"queue_name": self.name},
                    )
                )
            finally:
                if isinstance(item, QueuedMqttMessage):
                    with self._lock:
                        if self._active_item is item:
                            self._active_item = None
                            self._active_started_at = None
                            self._active_started_at_monotonic = None
                self._queue.task_done()

    def _snapshot_locked(self) -> dict[str, object]:
        active_item = self._active_item
        active_run_ms: int | None = None
        if self._active_started_at_monotonic is not None:
            active_run_ms = elapsed_ms_since(self._active_started_at_monotonic)
        average_queue_wait_ms: float | None = None
        if self._dequeued_count > 0:
            average_queue_wait_ms = round(
                self._total_queue_wait_ms / self._dequeued_count,
                2,
            )
        thread_alive = self._thread is not None and self._thread.is_alive()
        return {
            "name": self.name,
            "maxsize": self.maxsize,
            "queue_full_policy": self.queue_full_policy,
            "pending_count": self._queue.qsize(),
            "thread_alive": thread_alive,
            "stopping": self._stopping,
            "active": active_item is not None,
            "active_topic": active_item.message.topic if active_item is not None else None,
            "active_payload_bytes": (
                active_item.message.payload_bytes if active_item is not None else None
            ),
            "active_sequence": active_item.sequence if active_item is not None else None,
            "active_enqueued_at": active_item.enqueued_at if active_item is not None else None,
            "active_started_at": self._active_started_at,
            "active_run_ms": active_run_ms,
            "last_queue_wait_ms": self._last_queue_wait_ms,
            "max_queue_wait_ms": self._max_queue_wait_ms,
            "average_queue_wait_ms": average_queue_wait_ms,
            "enqueued_count": self._enqueued_count,
            "dequeued_count": self._dequeued_count,
            "completed_count": self._completed_count,
            "failed_count": self._failed_count,
            "rejected_count": self._rejected_count,
            "last_rejected_at": self._last_rejected_at,
            "last_error": self._last_error,
        }

    def record_event(self, event: RuntimeEvent) -> None:
        if self.event_writer is not None:
            self.event_writer.write(event)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def elapsed_ms_since(started_at_monotonic: float) -> int:
    return max(0, int((time.monotonic() - started_at_monotonic) * 1000))
