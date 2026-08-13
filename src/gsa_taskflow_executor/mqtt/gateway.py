from __future__ import annotations

import json
import logging
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any
from urllib.parse import urlparse

from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.event_log import (
    JsonlEventWriter,
    RuntimeEvent,
    configure_stdout_logging,
)

TaskflowMessageHandler = Callable[["TaskflowMessage"], None]
RobotStateMessageHandler = Callable[["TaskflowMessage"], None]


class MqttGatewayError(RuntimeError):
    """Raised when the MQTT gateway cannot connect, subscribe, or publish."""


@dataclass(frozen=True)
class TaskflowMessage:
    topic: str
    payload: str
    received_at: str

    @property
    def payload_bytes(self) -> int:
        return len(self.payload.encode("utf-8"))


class MqttGateway:
    """MQTT adapter for taskflow YAML input and status output."""

    def __init__(
        self,
        settings: ExecutorSettings,
        on_taskflow_message: TaskflowMessageHandler,
        on_robot_state_message: RobotStateMessageHandler | None = None,
        logger: logging.Logger | None = None,
        event_writer: JsonlEventWriter | None = None,
        mqtt_module: Any | None = None,
    ) -> None:
        self.settings = settings
        self.on_taskflow_message = on_taskflow_message
        self.on_robot_state_message = on_robot_state_message
        self.logger = logger or configure_stdout_logging()
        self.event_writer = event_writer
        self._mqtt_module = mqtt_module
        self._client: Any | None = None

    def connect(self) -> None:
        broker = urlparse(self.settings.mqtt_broker_url)

        mqtt = self._mqtt_module or import_paho_mqtt()
        client = create_client(mqtt, self.settings, broker.scheme)
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        client.on_subscribe = self._on_subscribe
        client.reconnect_delay_set(min_delay=1, max_delay=30)

        host = broker.hostname
        if not host:
            raise MqttGatewayError("MQTT_BROKER_URL 缺少 host")
        port = broker.port or default_port_for_scheme(broker.scheme)

        if broker.scheme in {"mqtts", "wss"}:
            client.tls_set()

        self._client = client
        self.logger.info("connecting MQTT broker %s:%s", host, port)
        try:
            client.connect(host, port, keepalive=30)
            client.loop_start()
        except OSError as error:
            self._client = None
            message = (
                f"无法连接 MQTT broker {host}:{port}，"
                "请确认 mosquitto 正在监听该端口"
            )
            raise MqttGatewayError(message) from error

    def disconnect(self) -> None:
        if self._client is None:
            return

        client = self._client
        self._client = None
        client.loop_stop()
        client.disconnect()
        self.logger.info("MQTT gateway disconnected")

    def run_forever(self) -> None:
        self.connect()
        self.logger.info(
            "listening taskflow YAML on %s, status topic %s, robot state request topics %s",
            self.settings.taskflow_input_topic,
            self.settings.status_topic,
            ", ".join(self.settings.robot_state_request_topics),
        )
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            self.logger.info("received interrupt, stopping MQTT gateway")
        finally:
            self.disconnect()

    def publish_status(self, payload: Mapping[str, Any]) -> None:
        if self._client is None:
            raise MqttGatewayError("MQTT 尚未连接，不能发布状态")

        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        # This method is usually called from paho's on_message callback. Waiting for publish
        # there can deadlock the network loop, so status publishing is intentionally async.
        self._client.publish(self.settings.status_topic, body, qos=0)
        self.record_event(
            RuntimeEvent(
                event_type="mqtt_status_published",
                message="status payload published",
                topic=self.settings.status_topic,
                payload={"payload": payload},
            )
        )

    def publish_json(
        self,
        topic: str,
        payload: Mapping[str, Any],
        *,
        event_type: str = "mqtt_json_published",
        message: str = "JSON payload published",
    ) -> None:
        if self._client is None:
            raise MqttGatewayError("MQTT 尚未连接，不能发布消息")

        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._client.publish(topic, body, qos=0)
        self.record_event(
            RuntimeEvent(
                event_type=event_type,
                message=message,
                topic=topic,
                payload={"payload": payload},
            )
        )

    def handle_message(self, topic: str, payload: bytes) -> None:
        received = TaskflowMessage(
            topic=topic,
            payload=payload.decode("utf-8"),
            received_at=datetime.now(timezone.utc).isoformat(),
        )
        self.logger.info(
            "received taskflow YAML topic=%s bytes=%s",
            received.topic,
            received.payload_bytes,
        )
        self.record_event(
            RuntimeEvent(
                event_type="taskflow_yaml_received",
                message="taskflow YAML received",
                topic=received.topic,
                payload={
                    "payload_bytes": received.payload_bytes,
                    "payload_preview": received.payload[:2000],
                },
            )
        )

        try:
            self.on_taskflow_message(received)
        except Exception as error:
            self.logger.exception("taskflow YAML handler failed")
            self.record_event(
                RuntimeEvent(
                    event_type="taskflow_yaml_handler_error",
                    level="error",
                    message=str(error),
                    topic=received.topic,
                )
            )
            raise

    def handle_robot_state_message(self, topic: str, payload: bytes) -> None:
        received = TaskflowMessage(
            topic=topic,
            payload=payload.decode("utf-8"),
            received_at=datetime.now(timezone.utc).isoformat(),
        )
        self.logger.info(
            "received robot state request topic=%s bytes=%s",
            received.topic,
            received.payload_bytes,
        )
        self.record_event(
            RuntimeEvent(
                event_type="robot_state_request_received",
                message="robot state request received",
                topic=received.topic,
                payload={
                    "payload_bytes": received.payload_bytes,
                    "payload_preview": received.payload[:1000],
                },
            )
        )

        if self.on_robot_state_message is None:
            self.logger.warning("robot state request handler is not configured")
            return

        try:
            self.on_robot_state_message(received)
        except Exception as error:
            self.logger.exception("robot state request handler failed")
            self.record_event(
                RuntimeEvent(
                    event_type="robot_state_request_handler_error",
                    level="error",
                    message=str(error),
                    topic=received.topic,
                )
            )
            raise

    def record_event(self, event: RuntimeEvent) -> None:
        if self.event_writer is not None:
            self.event_writer.write(event)

    def _on_connect(
        self,
        client: Any,
        _userdata: Any,
        _flags: Any,
        reason_code: Any,
        _properties: Any = None,
    ) -> None:
        if is_success_reason_code(reason_code):
            self.logger.info("MQTT connected, subscribing %s", self.settings.taskflow_input_topic)
            client.subscribe(self.settings.taskflow_input_topic, qos=0)
            if self.on_robot_state_message is not None:
                for topic in self.settings.robot_state_request_topics:
                    self.logger.info("MQTT connected, subscribing %s", topic)
                    client.subscribe(topic, qos=0)
            self.record_event(
                RuntimeEvent(
                    event_type="mqtt_connected",
                    message="MQTT connected",
                    topic=self.settings.taskflow_input_topic,
                )
            )
            return

        message = f"MQTT connect failed: {reason_code}"
        self.logger.error(message)
        self.record_event(
            RuntimeEvent(
                event_type="mqtt_connect_error",
                level="error",
                message=message,
            )
        )

    def _on_disconnect(
        self,
        _client: Any,
        _userdata: Any,
        _disconnect_flags: Any = None,
        reason_code: Any = None,
        _properties: Any = None,
    ) -> None:
        self.logger.warning("MQTT disconnected: %s", reason_code)
        self.record_event(
            RuntimeEvent(
                event_type="mqtt_disconnected",
                level="warning",
                message=str(reason_code),
            )
        )

    def _on_subscribe(
        self,
        _client: Any,
        _userdata: Any,
        _mid: Any,
        reason_codes: Any,
        _properties: Any = None,
    ) -> None:
        self.logger.info("MQTT subscribed %s: %s", self.settings.taskflow_input_topic, reason_codes)
        self.record_event(
            RuntimeEvent(
                event_type="mqtt_subscribed",
                message="MQTT subscribed",
                topic=self.settings.taskflow_input_topic,
                payload={"reason_codes": str(reason_codes)},
            )
        )

    def _on_message(self, _client: Any, _userdata: Any, message: Any) -> None:
        if (
            self.on_robot_state_message is not None
            and message.topic in self.settings.robot_state_request_topics
        ):
            self.handle_robot_state_message(message.topic, message.payload)
            return
        self.handle_message(message.topic, message.payload)


def import_paho_mqtt() -> Any:
    try:
        import paho.mqtt.client as mqtt
    except ModuleNotFoundError as error:
        raise MqttGatewayError(
            "缺少 paho-mqtt，请先执行 python -m pip install -e \".[dev]\""
        ) from error
    return mqtt


def create_client(mqtt: Any, settings: ExecutorSettings, scheme: str) -> Any:
    transport = "websockets" if scheme in {"ws", "wss"} else "tcp"
    callback_api_version = getattr(getattr(mqtt, "CallbackAPIVersion", None), "VERSION2", None)
    if callback_api_version is not None:
        return mqtt.Client(
            callback_api_version=callback_api_version,
            client_id=settings.mqtt_client_id,
            transport=transport,
        )

    return mqtt.Client(client_id=settings.mqtt_client_id, transport=transport)


def default_port_for_scheme(scheme: str) -> int:
    if scheme in {"mqtts", "wss"}:
        return 8883
    if scheme == "ws":
        return 80
    return 1883


def is_success_reason_code(reason_code: Any) -> bool:
    if getattr(reason_code, "is_failure", False):
        return False
    try:
        return int(reason_code) == 0
    except (TypeError, ValueError):
        return str(reason_code).lower() in {"0", "success"}


def wait_for_publish(result: Any) -> None:
    if hasattr(result, "wait_for_publish"):
        result.wait_for_publish()
