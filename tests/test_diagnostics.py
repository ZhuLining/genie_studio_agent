from __future__ import annotations

from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.diagnostics import (
    build_health_check_payload,
    build_runtime_diagnostics_payload,
    health_check_exit_code,
)
from gsa_taskflow_executor.taskflow.control import (
    TaskflowCancelRequest,
    TaskflowExecutionController,
)


def test_health_check_payload_reports_warning_for_safety_gate_and_ok_roundtrip() -> None:
    settings = ExecutorSettings(
        executor_aid="aid-1",
        taskflow_queue_maxsize=4,
        robot_state_queue_maxsize=2,
    )

    payload = build_health_check_payload(
        settings=settings,
        runtime_env={},
        skill_registry_summary={"source": "default", "skill_count": 0, "skills": []},
        version="unit",
        mqtt_probe=lambda _settings: {"ok": True, "topic": "health/topic"},
    )

    assert payload["status"] == "warning"
    assert health_check_exit_code(payload) == 0
    checks = {check["name"]: check for check in payload["checks"]}
    assert checks["taskflow_gdk_safety_gate"]["status"] == "warning"
    assert checks["mqtt_status_roundtrip"]["status"] == "ok"
    queue_detail = checks["queue_policy"]["detail"]
    assert queue_detail["taskflow_queue_maxsize"] == 4
    assert queue_detail["robot_state_queue_maxsize"] == 2


def test_health_check_payload_returns_error_when_mqtt_roundtrip_fails() -> None:
    settings = ExecutorSettings()

    payload = build_health_check_payload(
        settings=settings,
        runtime_env={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
        },
        skill_registry_summary={"source": "default", "skill_count": 0, "skills": []},
        version="unit",
        mqtt_probe=lambda _settings: {
            "ok": False,
            "stage": "connect_timeout",
            "error_msg": "MQTT connect timeout",
        },
    )

    assert payload["status"] == "error"
    assert health_check_exit_code(payload) == 1
    checks = {check["name"]: check for check in payload["checks"]}
    assert checks["mqtt_status_roundtrip"]["status"] == "error"


def test_runtime_diagnostics_includes_execution_session_worker_and_queues() -> None:
    settings = ExecutorSettings(executor_aid="aid-1")
    diagnostics = build_runtime_diagnostics_payload(
        settings=settings,
        queue_snapshots=[
            {
                "name": "taskflow-execution-worker",
                "pending_count": 1,
                "active": True,
            }
        ],
        execution_diagnostics={"active_app_execution_id": "run-1"},
        gdk_session_diagnostics={"busy": True, "active_purpose": "taskflow"},
        gdk_worker_diagnostics={"active_command_id": "cmd-1"},
    )

    assert diagnostics["executor"]["aid"] == "aid-1"
    assert diagnostics["mqtt"]["status_topic"] == "gsa/self/aid-1/status"
    assert diagnostics["queues"][0]["pending_count"] == 1
    assert diagnostics["execution"]["active_app_execution_id"] == "run-1"
    assert diagnostics["gdk_session"]["busy"] is True
    assert diagnostics["gdk_worker"]["active_command_id"] == "cmd-1"


def test_execution_controller_diagnostics_reports_active_and_pending_cancel() -> None:
    controller = TaskflowExecutionController()
    controller.start_execution("run-active")
    controller.request_cancel(
        TaskflowCancelRequest(
            request_id="cancel-pending",
            app_execution_id="run-next",
            reason="operator stop",
            topic="gsa/self/taskflow/run-next/cancel",
            received_at="2026-08-12T00:00:00+00:00",
        )
    )

    diagnostics = controller.diagnostics()

    assert diagnostics["active_app_execution_id"] == "run-active"
    assert diagnostics["active_cancellation"] is None
    assert diagnostics["pending_cancellation_count"] == 1
    assert diagnostics["pending_cancellation_app_execution_ids"] == ["run-next"]
