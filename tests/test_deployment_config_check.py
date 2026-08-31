from __future__ import annotations

from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.deployment import (
    build_deployment_config_check_payload,
    deployment_config_exit_code,
)


def test_deployment_config_check_rejects_missing_explicit_env_values() -> None:
    payload = build_deployment_config_check_payload(
        settings=ExecutorSettings(),
        runtime_env={},
    )

    assert payload["status"] == "error"
    assert deployment_config_exit_code(payload) == 1
    checks = {check["name"]: check for check in payload["checks"]}
    assert checks["mqtt_broker_url"]["status"] == "error"
    assert checks["executor_aid"]["status"] == "error"


def test_deployment_config_check_rejects_dev_executor_aid() -> None:
    payload = build_deployment_config_check_payload(
        settings=ExecutorSettings(
            mqtt_broker_url="mqtt://broker.internal:1883",
            executor_aid="gsa-dev",
        ),
        runtime_env={
            "MQTT_BROKER_URL": "mqtt://broker.internal:1883",
            "EXECUTOR_AID": "gsa-dev",
        },
    )

    assert payload["status"] == "error"
    checks = {check["name"]: check for check in payload["checks"]}
    assert checks["executor_aid"]["status"] == "error"
    assert "gsa-dev" in checks["executor_aid"]["detail"]["status_topic"]


def test_deployment_config_check_requires_explicit_local_broker_confirmation() -> None:
    payload = build_deployment_config_check_payload(
        settings=ExecutorSettings(
            mqtt_broker_url="mqtt://127.0.0.1:1883",
            executor_aid="G2A0004BC01053",
        ),
        runtime_env={
            "MQTT_BROKER_URL": "mqtt://127.0.0.1:1883",
            "EXECUTOR_AID": "G2A0004BC01053",
        },
    )

    assert payload["status"] == "error"
    checks = {check["name"]: check for check in payload["checks"]}
    assert checks["mqtt_broker_url"]["status"] == "error"


def test_deployment_config_check_allows_confirmed_local_broker() -> None:
    payload = build_deployment_config_check_payload(
        settings=ExecutorSettings(
            mqtt_broker_url="mqtt://127.0.0.1:1883",
            executor_aid="G2A0004BC01053",
        ),
        runtime_env={
            "MQTT_BROKER_URL": "mqtt://127.0.0.1:1883",
            "ALLOW_LOCAL_MQTT_BROKER": "1",
            "EXECUTOR_AID": "G2A0004BC01053",
        },
    )

    assert payload["status"] == "ok"
    assert deployment_config_exit_code(payload) == 0
