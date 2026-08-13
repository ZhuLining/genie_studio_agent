from __future__ import annotations

import logging
from collections.abc import Callable
from queue import Empty, Full, Queue
from threading import Lock, Thread
from typing import Final

from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent

MqttMessageHandler = Callable[[TaskflowMessage], None]
DEFAULT_WORKER_STOP_TIMEOUT_SECONDS: Final = 5.0


class MqttMessageQueueError(RuntimeError):
    """MQTT worker 队列无法接收消息时抛出。"""


class MqttMessageQueueFull(MqttMessageQueueError):
    """MQTT worker 队列已满时抛出。"""


class MqttMessageQueueStopped(MqttMessageQueueError):
    """worker 已停止或正在停止时仍尝试入队。"""


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
        logger: logging.Logger | None = None,
        event_writer: JsonlEventWriter | None = None,
    ) -> None:
        if maxsize <= 0:
            raise ValueError("maxsize must be greater than 0")

        self.name = name
        self.handler = handler
        self.maxsize = maxsize
        self.logger = logger or logging.getLogger(__name__)
        self.event_writer = event_writer
        self._queue: Queue[TaskflowMessage | object] = Queue(maxsize=maxsize)
        self._stop_item = object()
        self._lock = Lock()
        self._thread: Thread | None = None
        self._stopping = False

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
                payload={"queue_name": self.name, "maxsize": self.maxsize},
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

            try:
                # 入队必须是非阻塞的：队列满时快速失败，不能反向拖住 paho 回调线程。
                self._queue.put_nowait(message)
            except Full as error:
                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_queue_full",
                        level="error",
                        message=f"{self.name} queue is full",
                        topic=message.topic,
                        payload={
                            "queue_name": self.name,
                            "maxsize": self.maxsize,
                            "queued_count": self._queue.qsize(),
                        },
                    )
                )
                raise MqttMessageQueueFull(f"{self.name} queue is full") from error

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
                    "payload_bytes": message.payload_bytes,
                },
            )
        )
        return queued_count

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
                    self.record_event(
                        RuntimeEvent(
                            event_type="mqtt_message_discarded_after_stop",
                            level="warning",
                            message=f"{self.name} discarded queued message after stop",
                            payload={"queue_name": self.name},
                        )
                    )
                    continue

                if not isinstance(item, TaskflowMessage):
                    raise TypeError(f"{self.name} received an invalid queue item")

                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_worker_started_message",
                        message=f"{self.name} started message",
                        topic=item.topic,
                        payload={"queue_name": self.name},
                    )
                )
                self.handler(item)
                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_worker_finished_message",
                        message=f"{self.name} finished message",
                        topic=item.topic,
                        payload={"queue_name": self.name},
                    )
                )
            except Exception as error:
                self.logger.exception("%s message handler failed", self.name)
                self.record_event(
                    RuntimeEvent(
                        event_type="mqtt_message_worker_handler_error",
                        level="error",
                        message=str(error),
                        topic=item.topic if isinstance(item, TaskflowMessage) else None,
                        payload={"queue_name": self.name},
                    )
                )
            finally:
                self._queue.task_done()

    def record_event(self, event: RuntimeEvent) -> None:
        if self.event_writer is not None:
            self.event_writer.write(event)
