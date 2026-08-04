from __future__ import annotations

import time
from typing import Any

from gsa_taskflow_executor.gdk.subprocess_runtime import (
    GDK_OPERATION_TIMEOUT_CODE,
    GDK_SUBPROCESS_POLICY,
    run_gdk_subprocess,
)


def success_child(result_queue: Any) -> None:
    result_queue.put({"available": True, "executed": True, "backend": "test"})


def hanging_child(_result_queue: Any) -> None:
    time.sleep(10)


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
