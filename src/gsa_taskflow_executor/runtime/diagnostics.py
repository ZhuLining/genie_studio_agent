"""Executor health-check 与运行时诊断输出。

诊断逻辑必须保持只读：health-check 可以触达 MQTT broker 做回环探测，但不导入 GDK、
不启动 GDK worker，也不执行任何机器人控制。
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
from threading import Event
from typing import Any, Literal
from urllib.parse import urlparse
from uuid import uuid4

from gsa_taskflow_executor.gdk.motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from gsa_taskflow_executor.mqtt.gateway import (
    create_client,
    default_port_for_scheme,
    import_paho_mqtt,
    is_success_reason_code,
    publish_mqtt,
    wait_for_publish,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings

DiagnosticStatus = Literal["ok", "warning", "error"]
MqttProbe = Callable[[ExecutorSettings], Mapping[str, object]]


@dataclass(frozen=True)
class DiagnosticCheck:
    name: str
    status: DiagnosticStatus
    message: str
    detail: Mapping[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def build_health_check_payload(
    *,
    settings: ExecutorSettings,
    runtime_env: Mapping[str, str],
    skill_registry_summary: Mapping[str, object],
    version: str,
    mqtt_probe: MqttProbe | None = None,
    queue_snapshots: Sequence[Mapping[str, object]] | None = None,
    execution_diagnostics: Mapping[str, object] | None = None,
    gdk_session_diagnostics: Mapping[str, object] | None = None,
    gdk_worker_diagnostics: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """构建 --health-check 输出。error 会导致 CLI 非 0 退出。"""
    resolved_queue_snapshots = (
        [dict(snapshot) for snapshot in queue_snapshots]
        if queue_snapshots is not None
        else build_configured_queue_snapshots(settings)
    )
    resolved_execution_diagnostics = dict(
        execution_diagnostics or default_execution_diagnostics()
    )
    resolved_gdk_session_diagnostics = dict(
        gdk_session_diagnostics or default_gdk_session_diagnostics()
    )
    resolved_gdk_worker_diagnostics = dict(
        gdk_worker_diagnostics or default_gdk_worker_diagnostics()
    )
    runtime_diagnostics = build_runtime_diagnostics_payload(
        settings=settings,
        queue_snapshots=resolved_queue_snapshots,
        execution_diagnostics=resolved_execution_diagnostics,
        gdk_session_diagnostics=resolved_gdk_session_diagnostics,
        gdk_worker_diagnostics=resolved_gdk_worker_diagnostics,
    )
    resolved_mqtt_probe = mqtt_probe or probe_mqtt_status_roundtrip
    checks = [
        build_config_check(settings),
        build_safety_gate_check(runtime_env),
        build_mqtt_status_roundtrip_check(settings, resolved_mqtt_probe),
        build_mqtt_topics_check(settings),
        build_queue_policy_check(settings, resolved_queue_snapshots),
        build_skill_registry_check(skill_registry_summary),
        build_execution_state_check(resolved_execution_diagnostics),
        build_gdk_session_check(resolved_gdk_session_diagnostics),
        build_gdk_worker_check(resolved_gdk_worker_diagnostics),
    ]
    status = overall_status(checks)
    return {
        "type": "executor_health_check",
        "version": version,
        "checked_at": utc_now_iso(),
        "status": status,
        "checks": [check.to_dict() for check in checks],
        "diagnostics": runtime_diagnostics,
    }


def build_runtime_diagnostics_payload(
    *,
    settings: ExecutorSettings,
    queue_snapshots: Sequence[Mapping[str, object]],
    execution_diagnostics: Mapping[str, object] | None = None,
    gdk_session_diagnostics: Mapping[str, object] | None = None,
    gdk_worker_diagnostics: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """构建 listen 运行态也能复用的只读快照。"""
    return {
        "executor": {
            "aid": settings.executor_aid,
            "mode": settings.executor_mode,
            "log_dir": settings.executor_log_dir,
        },
        "mqtt": {
            "broker_url": settings.mqtt_broker_url,
            "client_id": settings.mqtt_client_id,
            "taskflow_input_topic": settings.taskflow_input_topic,
            "taskflow_cancel_topic_filter": settings.taskflow_cancel_topic_filter,
            "status_topic": settings.status_topic,
            "robot_state_request_topics": list(settings.robot_state_request_topics),
            "status_qos": settings.mqtt_status_qos,
            "terminal_status_qos": settings.mqtt_terminal_status_qos,
            "terminal_status_retain": settings.mqtt_terminal_status_retain,
        },
        "queues": [dict(snapshot) for snapshot in queue_snapshots],
        "execution": dict(execution_diagnostics or default_execution_diagnostics()),
        "gdk_session": dict(gdk_session_diagnostics or default_gdk_session_diagnostics()),
        "gdk_worker": dict(gdk_worker_diagnostics or default_gdk_worker_diagnostics()),
    }


def probe_mqtt_status_roundtrip(settings: ExecutorSettings) -> dict[str, object]:
    """用临时 client 做 MQTT connect/subscribe/publish 回环。

    探测 topic 放在 status topic 子路径下，避免污染客户端订阅的正式状态 topic。
    """
    started_at = time.monotonic()
    broker = urlparse(settings.mqtt_broker_url)
    host = broker.hostname
    if not host:
        return build_mqtt_probe_error(
            settings=settings,
            stage="parse_broker_url",
            started_at=started_at,
            message="MQTT_BROKER_URL 缺少 host",
        )
    port = broker.port or default_port_for_scheme(broker.scheme)
    probe_id = uuid4().hex
    probe_topic = f"{settings.status_topic}/health_check/{probe_id}"
    probe_payload = json.dumps(
        {
            "type": "executor_health_check_probe",
            "probeId": probe_id,
            "aid": settings.executor_aid,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )
    connect_timeout = settings.diagnostics_mqtt_connect_timeout

    try:
        mqtt = import_paho_mqtt()
        probe_settings = replace(
            settings,
            mqtt_client_id=f"{settings.mqtt_client_id}-health-{probe_id[:8]}",
        )
        client = create_client(mqtt, probe_settings, broker.scheme)
    except Exception as error:
        return build_mqtt_probe_error(
            settings=settings,
            stage="create_client",
            started_at=started_at,
            error=error,
        )

    connected = Event()
    subscribed = Event()
    received = Event()
    failure: dict[str, object] = {}
    loop_started = False

    def on_connect(
        _client: Any,
        _userdata: Any,
        _flags: Any,
        reason_code: Any,
        _properties: Any = None,
    ) -> None:
        if is_success_reason_code(reason_code):
            connected.set()
            return
        failure.update({"stage": "connect", "reason_code": str(reason_code)})
        connected.set()

    def on_subscribe(
        _client: Any,
        _userdata: Any,
        _mid: Any,
        _reason_codes: Any,
        _properties: Any = None,
    ) -> None:
        subscribed.set()

    def on_message(_client: Any, _userdata: Any, message: Any) -> None:
        raw_topic = getattr(message, "topic", "")
        raw_payload = getattr(message, "payload", b"")
        try:
            decoded_payload = (
                raw_payload.decode("utf-8") if isinstance(raw_payload, bytes) else str(raw_payload)
            )
        except UnicodeDecodeError:
            return
        if raw_topic == probe_topic and decoded_payload == probe_payload:
            received.set()

    try:
        client.on_connect = on_connect
        client.on_subscribe = on_subscribe
        client.on_message = on_message
        client.reconnect_delay_set(min_delay=1, max_delay=2)
        if broker.scheme in {"mqtts", "wss"}:
            client.tls_set()
        client.connect(host, port, keepalive=10)
        client.loop_start()
        loop_started = True

        if not connected.wait(timeout=connect_timeout):
            return build_mqtt_probe_error(
                settings=settings,
                stage="connect_timeout",
                started_at=started_at,
                host=host,
                port=port,
                message="MQTT connect timeout",
            )
        if failure:
            return build_mqtt_probe_error(
                settings=settings,
                stage=str(failure.get("stage") or "connect"),
                started_at=started_at,
                host=host,
                port=port,
                message=str(failure.get("reason_code") or "MQTT connect failed"),
            )

        client.subscribe(probe_topic, qos=1)
        if not subscribed.wait(timeout=connect_timeout):
            return build_mqtt_probe_error(
                settings=settings,
                stage="subscribe_timeout",
                started_at=started_at,
                host=host,
                port=port,
                topic=probe_topic,
                message="MQTT subscribe timeout",
            )

        publish_result = publish_mqtt(client, probe_topic, probe_payload, qos=1, retain=False)
        wait_for_publish(publish_result, timeout=connect_timeout)
        if not received.wait(timeout=connect_timeout):
            return build_mqtt_probe_error(
                settings=settings,
                stage="roundtrip_timeout",
                started_at=started_at,
                host=host,
                port=port,
                topic=probe_topic,
                message="MQTT roundtrip timeout",
            )
    except Exception as error:
        return build_mqtt_probe_error(
            settings=settings,
            stage="mqtt_roundtrip",
            started_at=started_at,
            host=host,
            port=port,
            topic=probe_topic,
            error=error,
        )
    finally:
        if loop_started:
            client.disconnect()
            client.loop_stop()

    return {
        "ok": True,
        "scheme": broker.scheme,
        "host": host,
        "port": port,
        "topic": probe_topic,
        "qos": 1,
        "timeout_seconds": connect_timeout,
        "elapsed_ms": elapsed_ms_since(started_at),
    }


def build_configured_queue_snapshots(settings: ExecutorSettings) -> list[dict[str, object]]:
    return [
        {
            "name": "taskflow-execution-worker",
            "role": "taskflow_execution",
            "maxsize": settings.taskflow_queue_maxsize,
            "queue_full_policy": settings.taskflow_queue_full_policy,
            "configured_only": True,
        },
        {
            "name": "robot-state-worker",
            "role": "robot_state",
            "maxsize": settings.robot_state_queue_maxsize,
            "queue_full_policy": settings.robot_state_queue_full_policy,
            "configured_only": True,
        },
    ]


def build_config_check(settings: ExecutorSettings) -> DiagnosticCheck:
    return DiagnosticCheck(
        name="config",
        status="ok",
        message="executor settings validated",
        detail={
            "executor_aid": settings.executor_aid,
            "executor_mode": settings.executor_mode,
            "execution_log_dir": str(settings.execution_log_dir),
        },
    )


def build_safety_gate_check(runtime_env: Mapping[str, str]) -> DiagnosticCheck:
    enabled = runtime_env.get("ENABLE_GDK_CONTROL") == "1"
    confirmed = runtime_env.get("CONFIRM_GDK_CONTROL") == TASKFLOW_ABS_JOINT_CONFIRMATION
    status: DiagnosticStatus = "ok" if enabled and confirmed else "warning"
    return DiagnosticCheck(
        name="taskflow_gdk_safety_gate",
        status=status,
        message=(
            "taskflow GDK control gate is enabled"
            if status == "ok"
            else "taskflow GDK control gate is not fully enabled"
        ),
        detail={
            "enabled": enabled,
            "confirmed": confirmed,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
    )


def build_mqtt_status_roundtrip_check(
    settings: ExecutorSettings,
    mqtt_probe: MqttProbe,
) -> DiagnosticCheck:
    try:
        result = dict(mqtt_probe(settings))
    except Exception as error:
        result = {
            "ok": False,
            "stage": "mqtt_probe_exception",
            "error_type": type(error).__name__,
            "error_msg": str(error),
        }
    ok = result.get("ok") is True
    return DiagnosticCheck(
        name="mqtt_status_roundtrip",
        status="ok" if ok else "error",
        message=(
            "MQTT connect/subscribe/publish roundtrip succeeded"
            if ok
            else "MQTT connect/subscribe/publish roundtrip failed"
        ),
        detail=result,
    )


def build_mqtt_topics_check(settings: ExecutorSettings) -> DiagnosticCheck:
    return DiagnosticCheck(
        name="mqtt_topics",
        status="ok",
        message="MQTT topics resolved",
        detail={
            "taskflow_input_topic": settings.taskflow_input_topic,
            "taskflow_cancel_topic_filter": settings.taskflow_cancel_topic_filter,
            "status_topic": settings.status_topic,
            "robot_state_request_topics": list(settings.robot_state_request_topics),
            "terminal_status_qos": settings.mqtt_terminal_status_qos,
            "terminal_status_retain": settings.mqtt_terminal_status_retain,
        },
    )


def build_queue_policy_check(
    settings: ExecutorSettings,
    queue_snapshots: Sequence[Mapping[str, object]],
) -> DiagnosticCheck:
    return DiagnosticCheck(
        name="queue_policy",
        status="ok",
        message="MQTT worker queue policies resolved",
        detail={
            "taskflow_queue_maxsize": settings.taskflow_queue_maxsize,
            "taskflow_queue_full_policy": settings.taskflow_queue_full_policy,
            "robot_state_queue_maxsize": settings.robot_state_queue_maxsize,
            "robot_state_queue_full_policy": settings.robot_state_queue_full_policy,
            "queues": [dict(snapshot) for snapshot in queue_snapshots],
        },
    )


def build_skill_registry_check(skill_registry_summary: Mapping[str, object]) -> DiagnosticCheck:
    return DiagnosticCheck(
        name="skill_registry",
        status="ok",
        message="skill registry loaded",
        detail=dict(skill_registry_summary),
    )


def build_execution_state_check(execution: Mapping[str, object]) -> DiagnosticCheck:
    active_app_execution_id = execution.get("active_app_execution_id")
    status: DiagnosticStatus = "warning" if isinstance(active_app_execution_id, str) else "ok"
    return DiagnosticCheck(
        name="taskflow_execution_state",
        status=status,
        message=(
            f"taskflow is running: {active_app_execution_id}"
            if status == "warning"
            else "no active taskflow"
        ),
        detail=dict(execution),
    )


def build_gdk_session_check(gdk_session: Mapping[str, object]) -> DiagnosticCheck:
    busy = gdk_session.get("busy") is True
    return DiagnosticCheck(
        name="gdk_session",
        status="warning" if busy else "ok",
        message="GDK session is busy" if busy else "GDK session is idle",
        detail=dict(gdk_session),
    )


def build_gdk_worker_check(gdk_worker: Mapping[str, object]) -> DiagnosticCheck:
    active_command_id = gdk_worker.get("active_command_id")
    status: DiagnosticStatus = "warning" if isinstance(active_command_id, str) else "ok"
    return DiagnosticCheck(
        name="gdk_worker",
        status=status,
        message=(
            f"GDK worker command is active: {active_command_id}"
            if status == "warning"
            else "GDK worker has no active command"
        ),
        detail=dict(gdk_worker),
    )


def overall_status(checks: Sequence[DiagnosticCheck]) -> DiagnosticStatus:
    statuses = {check.status for check in checks}
    if "error" in statuses:
        return "error"
    if "warning" in statuses:
        return "warning"
    return "ok"


def health_check_exit_code(payload: Mapping[str, object]) -> int:
    return 1 if payload.get("status") == "error" else 0


def default_execution_diagnostics() -> dict[str, object]:
    return {
        "active_app_execution_id": None,
        "active_cancellation": None,
        "pending_cancellation_count": 0,
        "pending_cancellation_app_execution_ids": [],
    }


def default_gdk_session_diagnostics() -> dict[str, object]:
    return {
        "policy": "process_managed_session",
        "busy": False,
        "active_purpose": None,
        "initialized": False,
        "init_result": {"called": False, "success": True, "return": None},
    }


def default_gdk_worker_diagnostics() -> dict[str, object]:
    return {
        "policy": "persistent_gdk_worker",
        "started": False,
        "process": {
            "policy": "persistent_gdk_worker",
            "pid": None,
            "exitcode": None,
            "command_id": None,
            "timed_out": False,
            "terminated": False,
            "killed": False,
            "worker_started": False,
            "worker_reused": False,
        },
        "active_command_id": None,
        "active_command": None,
        "pending_result_count": 0,
        "cancelled_result_count": 0,
    }


def build_mqtt_probe_error(
    *,
    settings: ExecutorSettings,
    stage: str,
    started_at: float,
    host: str | None = None,
    port: int | None = None,
    topic: str | None = None,
    message: str | None = None,
    error: Exception | None = None,
) -> dict[str, object]:
    broker = urlparse(settings.mqtt_broker_url)
    detail: dict[str, object] = {
        "ok": False,
        "stage": stage,
        "scheme": broker.scheme,
        "host": host or broker.hostname or "",
        "port": port or broker.port or default_port_for_scheme(broker.scheme),
        "timeout_seconds": settings.diagnostics_mqtt_connect_timeout,
        "elapsed_ms": elapsed_ms_since(started_at),
    }
    if topic is not None:
        detail["topic"] = topic
    if message is not None:
        detail["error_msg"] = message
    if error is not None:
        detail["error_type"] = type(error).__name__
        detail["error_msg"] = str(error)
    return detail


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def elapsed_ms_since(started_at_monotonic: float) -> int:
    return max(0, int((time.monotonic() - started_at_monotonic) * 1000))
