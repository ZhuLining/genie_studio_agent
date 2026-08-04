from __future__ import annotations

import os
import time
from typing import Any

from gsa_taskflow_executor.gdk.subprocess_runtime import (
    GDK_OPERATION_TIMEOUT_CODE,
    GDK_SUBPROCESS_POLICY,
    run_gdk_subprocess,
)
from gsa_taskflow_executor.gdk.worker_runtime import GDK_WORKER_POLICY, GdkWorkerProcessManager


def success_child(result_queue: Any) -> None:
    result_queue.put({"available": True, "executed": True, "backend": "test"})


def hanging_child(_result_queue: Any) -> None:
    time.sleep(10)


def echo_worker(command_queue: Any, result_queue: Any) -> None:
    while True:
        command = command_queue.get()
        command_id = command["command_id"]
        if command["kind"] == "shutdown":
            result_queue.put(
                {
                    "command_id": command_id,
                    "result": {
                        "available": True,
                        "executed": False,
                        "backend": "test",
                        "action": "shutdown",
                        "gdk_release": {"called": True, "success": True},
                    },
                }
            )
            return

        result_queue.put(
            {
                "command_id": command_id,
                "result": {
                    "available": True,
                    "executed": True,
                    "backend": "test",
                    "action": command["action"],
                    "worker_pid": os.getpid(),
                },
            }
        )


def selective_worker(command_queue: Any, result_queue: Any) -> None:
    while True:
        command = command_queue.get()
        command_id = command["command_id"]
        if command["kind"] == "shutdown":
            result_queue.put(
                {
                    "command_id": command_id,
                    "result": {
                        "available": True,
                        "executed": False,
                        "backend": "test",
                        "action": "shutdown",
                        "gdk_release": {"called": True, "success": True},
                    },
                }
            )
            return
        if command["kind"] == "sleep":
            time.sleep(10)
            continue
        result_queue.put(
            {
                "command_id": command_id,
                "result": {
                    "available": True,
                    "executed": True,
                    "backend": "test",
                    "action": command["action"],
                    "worker_pid": os.getpid(),
                },
            }
        )


def test_gdk_subprocess_returns_child_result() -> None:
    result = run_gdk_subprocess(
        operation="unit_success",
        action="unit_success",
        backend="test",
        timeout_seconds=2.0,
        child_target=success_child,
        child_args=(),
        safety_gate={"enabled": True, "confirmed": True},
    )

    assert result["executed"] is True
    assert result["subprocess"]["policy"] == GDK_SUBPROCESS_POLICY
    assert result["subprocess"]["timed_out"] is False


def test_gdk_subprocess_times_out_and_terminates_child() -> None:
    result = run_gdk_subprocess(
        operation="unit_timeout",
        action="unit_timeout",
        backend="test",
        timeout_seconds=0.1,
        child_target=hanging_child,
        child_args=(),
        safety_gate={"enabled": True, "confirmed": True},
        terminate_grace_seconds=0.1,
    )

    assert result["executed"] is False
    assert result["error_code"] == GDK_OPERATION_TIMEOUT_CODE
    assert result["error_stage"] == "gdk_operation_timeout"
    assert result["timeout_seconds"] == 0.1
    assert result["subprocess"]["policy"] == GDK_SUBPROCESS_POLICY
    assert result["subprocess"]["timed_out"] is True
    assert result["subprocess"]["terminated"] is True


def test_gdk_worker_reuses_process_for_successive_commands() -> None:
    manager = GdkWorkerProcessManager(
        worker_target=echo_worker,
        terminate_grace_seconds=0.1,
    )
    try:
        first = manager.run_command(
            kind="echo",
            payload={},
            action="unit_echo",
            backend="test",
            timeout_seconds=2.0,
            safety_gate={"enabled": True, "confirmed": True},
        )
        second = manager.run_command(
            kind="echo",
            payload={},
            action="unit_echo",
            backend="test",
            timeout_seconds=2.0,
            safety_gate={"enabled": True, "confirmed": True},
        )
    finally:
        shutdown = manager.shutdown(timeout_seconds=2.0)

    assert first["executed"] is True
    assert second["executed"] is True
    assert first["subprocess"]["policy"] == GDK_WORKER_POLICY
    assert second["subprocess"]["policy"] == GDK_WORKER_POLICY
    assert first["subprocess"]["worker_started"] is True
    assert second["subprocess"]["worker_reused"] is True
    assert first["worker_pid"] == second["worker_pid"]
    assert shutdown["success"] is True
    assert shutdown["gdk_release"]["called"] is True


def test_gdk_worker_timeout_kills_process_and_next_command_restarts() -> None:
    manager = GdkWorkerProcessManager(
        worker_target=selective_worker,
        terminate_grace_seconds=0.1,
    )
    try:
        timed_out = manager.run_command(
            kind="sleep",
            payload={},
            action="unit_sleep",
            backend="test",
            timeout_seconds=0.1,
            safety_gate={"enabled": True, "confirmed": True},
        )
        restarted = manager.run_command(
            kind="echo",
            payload={},
            action="unit_echo",
            backend="test",
            timeout_seconds=2.0,
            safety_gate={"enabled": True, "confirmed": True},
        )
    finally:
        manager.shutdown(timeout_seconds=2.0)

    assert timed_out["executed"] is False
    assert timed_out["error_code"] == GDK_OPERATION_TIMEOUT_CODE
    assert timed_out["subprocess"]["policy"] == GDK_WORKER_POLICY
    assert timed_out["subprocess"]["timed_out"] is True
    assert timed_out["subprocess"]["terminated"] is True
    assert restarted["executed"] is True
    assert restarted["subprocess"]["worker_started"] is True
