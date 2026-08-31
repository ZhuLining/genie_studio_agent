from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
from typing import Literal
from urllib.parse import urlparse

from gsa_taskflow_executor.runtime.config import ExecutorSettings

DeploymentStatus = Literal["ok", "warning", "error"]

ALLOW_LOCAL_MQTT_BROKER_ENV = "ALLOW_LOCAL_MQTT_BROKER"
DEVELOPMENT_EXECUTOR_AIDS = frozenset({"gsa-dev"})
LOCAL_MQTT_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})


@dataclass(frozen=True)
class DeploymentConfigCheck:
    name: str
    status: DeploymentStatus
    message: str
    detail: Mapping[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def build_deployment_config_check_payload(
    *,
    settings: ExecutorSettings,
    runtime_env: Mapping[str, str],
) -> dict[str, object]:
    """发布启动前配置门禁，避免开发默认值进入真机长连接服务。"""

    checks = [
        build_mqtt_broker_check(settings=settings, runtime_env=runtime_env),
        build_executor_aid_check(settings=settings, runtime_env=runtime_env),
    ]
    status = overall_status(checks)
    return {
        "type": "executor_deployment_config_check",
        "status": status,
        "checks": [check.to_dict() for check in checks],
        "resolved": {
            "mqtt_broker_url": settings.mqtt_broker_url,
            "executor_aid": settings.executor_aid,
            "status_topic": settings.status_topic,
        },
    }


def deployment_config_exit_code(payload: Mapping[str, object]) -> int:
    return 1 if payload.get("status") == "error" else 0


def build_mqtt_broker_check(
    *,
    settings: ExecutorSettings,
    runtime_env: Mapping[str, str],
) -> DeploymentConfigCheck:
    raw_value = runtime_env.get("MQTT_BROKER_URL", "").strip()
    if not raw_value:
        return DeploymentConfigCheck(
            name="mqtt_broker_url",
            status="error",
            message="MQTT_BROKER_URL 必须在部署 env 中显式配置。",
            detail={"env": "MQTT_BROKER_URL"},
        )

    broker = urlparse(settings.mqtt_broker_url)
    host = (broker.hostname or "").lower()
    local_broker_allowed = runtime_env.get(ALLOW_LOCAL_MQTT_BROKER_ENV, "").strip() == "1"
    if host in LOCAL_MQTT_HOSTS and not local_broker_allowed:
        return DeploymentConfigCheck(
            name="mqtt_broker_url",
            status="error",
            message=(
                "MQTT_BROKER_URL 指向本机地址；只有确认 broker 与 executor 同机时，"
                f"才能设置 {ALLOW_LOCAL_MQTT_BROKER_ENV}=1 放行。"
            ),
            detail={
                "mqtt_broker_url": settings.mqtt_broker_url,
                "allow_env": ALLOW_LOCAL_MQTT_BROKER_ENV,
            },
        )

    return DeploymentConfigCheck(
        name="mqtt_broker_url",
        status="ok",
        message="MQTT_BROKER_URL 已通过部署校验。",
        detail={
            "mqtt_broker_url": settings.mqtt_broker_url,
            "local_broker_confirmed": host in LOCAL_MQTT_HOSTS,
        },
    )


def build_executor_aid_check(
    *,
    settings: ExecutorSettings,
    runtime_env: Mapping[str, str],
) -> DeploymentConfigCheck:
    raw_value = runtime_env.get("EXECUTOR_AID", "").strip()
    if not raw_value:
        return DeploymentConfigCheck(
            name="executor_aid",
            status="error",
            message="EXECUTOR_AID 必须在部署 env 中显式配置。",
            detail={"env": "EXECUTOR_AID"},
        )

    normalized_aid = settings.executor_aid.strip().lower()
    if normalized_aid in DEVELOPMENT_EXECUTOR_AIDS:
        return DeploymentConfigCheck(
            name="executor_aid",
            status="error",
            message="EXECUTOR_AID 仍是开发默认值，会导致状态 topic 与客户端目标 AID 错配。",
            detail={
                "executor_aid": settings.executor_aid,
                "status_topic": settings.status_topic,
            },
        )

    return DeploymentConfigCheck(
        name="executor_aid",
        status="ok",
        message="EXECUTOR_AID 已通过部署校验。",
        detail={
            "executor_aid": settings.executor_aid,
            "status_topic": settings.status_topic,
        },
    )


def overall_status(checks: Sequence[DeploymentConfigCheck]) -> DeploymentStatus:
    statuses = {check.status for check in checks}
    if "error" in statuses:
        return "error"
    if "warning" in statuses:
        return "warning"
    return "ok"
