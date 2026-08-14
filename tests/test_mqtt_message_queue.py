from __future__ import annotations

import threading
import time

import pytest

from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.mqtt.message_queue import (
    MqttMessageQueueFull,
    MqttMessageQueueStopped,
    MqttMessageWorkerQueue,
)


def make_message(payload: str) -> TaskflowMessage:
    return TaskflowMessage(
        topic="gsa/self/taskflow_yaml",
        payload=payload,
        received_at="2026-07-31T00:00:00+00:00",
    )


def test_worker_queue_runs_messages_fifo_without_blocking_enqueue() -> None:
    first_started = threading.Event()
    release_first = threading.Event()
    second_finished = threading.Event()
    events: list[tuple[str, str]] = []

    def handle(message: TaskflowMessage) -> None:
        events.append(("start", message.payload))
        if message.payload == "first":
            first_started.set()
            assert release_first.wait(timeout=1)
        events.append(("finish", message.payload))
        if message.payload == "second":
            second_finished.set()

    queue = MqttMessageWorkerQueue(
        name="test-taskflow-worker",
        handler=handle,
        maxsize=4,
    )
    queue.start()
    try:
        queue.enqueue(make_message("first"))
        assert first_started.wait(timeout=1)

        started = time.monotonic()
        queue.enqueue(make_message("second"))
        elapsed_ms = (time.monotonic() - started) * 1000

        assert elapsed_ms < 100
        assert events == [("start", "first")]

        release_first.set()
        assert second_finished.wait(timeout=1)
        assert events == [
            ("start", "first"),
            ("finish", "first"),
            ("start", "second"),
            ("finish", "second"),
        ]
    finally:
        release_first.set()
        queue.stop()


def test_worker_queue_rejects_when_full() -> None:
    queue = MqttMessageWorkerQueue(
        name="test-full-worker",
        handler=lambda _message: None,
        maxsize=1,
    )

    with pytest.raises(MqttMessageQueueStopped):
        queue.enqueue(make_message("before-start"))

    queue.start()
    try:
        queue.stop()
        with pytest.raises(MqttMessageQueueStopped):
            queue.enqueue(make_message("after-stop"))
    finally:
        queue.stop()

    blocking_handler_started = threading.Event()
    blocking_handler_release = threading.Event()

    def blocking_handler(_message: TaskflowMessage) -> None:
        blocking_handler_started.set()
        blocking_handler_release.wait(timeout=1)

    queue = MqttMessageWorkerQueue(
        name="test-full-running-worker",
        handler=blocking_handler,
        maxsize=1,
    )
    queue.start()
    try:
        queue.enqueue(make_message("running"))
        assert blocking_handler_started.wait(timeout=1)
        queue.enqueue(make_message("queued"))
        with pytest.raises(MqttMessageQueueFull):
            queue.enqueue(make_message("overflow"))
    finally:
        blocking_handler_release.set()
        queue.stop()

    snapshot = queue.snapshot()
    assert snapshot["maxsize"] == 1
    assert snapshot["queue_full_policy"] == "reject"
    assert snapshot["rejected_count"] == 1


def test_worker_queue_snapshot_tracks_active_and_wait_metrics() -> None:
    handler_started = threading.Event()
    release_handler = threading.Event()

    def blocking_handler(_message: TaskflowMessage) -> None:
        handler_started.set()
        release_handler.wait(timeout=1)

    queue = MqttMessageWorkerQueue(
        name="test-observable-worker",
        handler=blocking_handler,
        maxsize=2,
    )
    queue.start()
    try:
        queue.enqueue(make_message("active"))
        assert handler_started.wait(timeout=1)

        snapshot = queue.snapshot()
        assert snapshot["thread_alive"] is True
        assert snapshot["active"] is True
        assert snapshot["active_topic"] == "gsa/self/taskflow_yaml"
        assert snapshot["active_payload_bytes"] == len("active")
        assert snapshot["enqueued_count"] == 1
        assert snapshot["dequeued_count"] == 1
        assert snapshot["last_queue_wait_ms"] is not None
        assert snapshot["active_started_at"] is not None
        assert snapshot["active_run_ms"] is not None
    finally:
        release_handler.set()
        queue.stop()

    finished_snapshot = queue.snapshot()
    assert finished_snapshot["active"] is False
    assert finished_snapshot["completed_count"] == 1
