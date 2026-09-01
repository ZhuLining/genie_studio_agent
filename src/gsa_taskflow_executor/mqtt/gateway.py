"""MQTT gateway — paho-mqtt 封装，负责连接、订阅、发布和消息路由。

_on_message 在 paho 网络线程回调，按 topic 路由:
- robot_state topics → handle_robot_state_message → robot_state_queue
- cancel topic → handle_taskflow_cancel_message (直接处理，不经过队列)
- 其他 → handle_message → taskflow_queue
"""

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
from gsa_taskflow_executor.runtime.payload_sanitizer import (
    PayloadSanitizerConfig,
    payload_preview,
    sanitize_event_payload,
)

TaskflowMessageHandler = Callable[["TaskflowMessage"], None]
RobotStateMessageHandler = Callable[["TaskflowMessage"], None]
TaskflowCancelMessageHandler = Callable[["TaskflowMessage"], None]
MQTT_LOOP_WATCHDOG_INTERVAL_SECONDS = 5.0


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
        on_taskflow_cancel_message: TaskflowCancelMessageHandler | None = None,
        logger: logging.Logger | None = None,
        event_writer: JsonlEventWriter | None = None,
        mqtt_module: Any | None = None,
    ) -> None:
        self.settings = settings
        self.on_taskflow_message = on_taskflow_message
        self.on_robot_state_message = on_robot_state_message
        self.on_taskflow_cancel_message = on_taskflow_cancel_message
        self.logger = logger or configure_stdout_logging()
        self.event_writer = event_writer
        self._mqtt_module = mqtt_module
        self._client: Any | None = None
        self._payload_sanitizer_config = PayloadSanitizerConfig.from_settings(settings)

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

    def run_forever(
        self,
        *,
        watchdog_interval_seconds: float = MQTT_LOOP_WATCHDOG_INTERVAL_SECONDS,
    ) -> None:
        self.connect()
        self.logger.info(
            "listening taskflow YAML on %s, cancel topic %s, status topic %s, "
            "robot state request topics %s",
            self.settings.taskflow_input_topic,
            self.settings.taskflow_cancel_topic_filter,
            self.settings.status_topic,
            ", ".join(self.settings.robot_state_request_topics),
        )
        try:
            while True:
                time.sleep(watchdog_interval_seconds)
                self.ensure_network_loop_alive()
        except KeyboardInterrupt:
            self.logger.info("received interrupt, stopping MQTT gateway")
        finally:
            self.disconnect()

    def ensure_network_loop_alive(self) -> None:
        if self._client is None:
            raise MqttGatewayError("MQTT client missing while gateway is running")

        thread = getattr(self._client, "_thread", None)
        is_alive = getattr(thread, "is_alive", None)
        if thread is None or not callable(is_alive):
            return
        if is_alive():
            return

        message = "MQTT network loop thread stopped unexpectedly"
        self.logger.error(message)
        self.record_event(
            RuntimeEvent(
                event_type="mqtt_loop_thread_dead",
                level="error",
                message=message,
            )
        )
        raise MqttGatewayError(message)

    def publish_status(
        self,
        payload: Mapping[str, Any],
        *,
        wait_for_terminal: bool = True,
    ) -> None:
        if self._client is None:
            raise MqttGatewayError("MQTT 尚未连接，不能发布状态")

        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        terminal = is_terminal_status_payload(payload)
        qos = (
            self.settings.mqtt_terminal_status_qos
            if terminal
            else self.settings.mqtt_status_qos
        )
        retain = self.settings.mqtt_terminal_status_retain if terminal else False
        result = publish_mqtt(
            self._client,
            self.settings.status_topic,
            body,
            qos=qos,
            retain=retain,
        )
        if terminal and wait_for_terminal and qos > 0:
            wait_for_publish(result, timeout=self.settings.mqtt_terminal_status_wait_timeout)
        self.record_event(
            RuntimeEvent(
                event_type="mqtt_status_published",
                message="status payload published",
                topic=self.settings.status_topic,
                payload={
                    "payload": sanitize_event_payload(
                        dict(payload),
                        config=self._payload_sanitizer_config,
                    ),
                    "qos": qos,
                    "retain": retain,
                    "terminal": terminal,
                },
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
        result = self._client.publish(topic, body, qos=0)
        publish_rc = getattr(result, "rc", None)
        if publish_rc not in (None, 0):
            self.logger.error(
                "MQTT publish failed topic=%s rc=%s payload=%s",
                topic,
                publish_rc,
                sanitize_event_payload(
                    dict(payload),
                    config=self._payload_sanitizer_config,
                ),
            )
        self.record_event(
            RuntimeEvent(
                event_type=event_type,
                message=message,
                topic=topic,
                payload={
                    "payload": sanitize_event_payload(
                        dict(payload),
                        config=self._payload_sanitizer_config,
                    )
                },
            )
        )

    def handle_message(self, topic: str, payload: bytes) -> None:
        decoded_payload = self.decode_utf8_payload(
            topic,
            payload,
            route="taskflow_yaml",
        )
        if decoded_payload is None:
            return
        received = TaskflowMessage(
            topic=topic,
            payload=decoded_payload,
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
                    "payload_preview": payload_preview(
                        received.payload,
                        config=self._payload_sanitizer_config,
                    ),
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

    def handle_taskflow_cancel_message(self, topic: str, payload: bytes) -> None:
        decoded_payload = self.decode_utf8_payload(
            topic,
            payload,
            route="taskflow_cancel",
        )
        if decoded_payload is None:
            return
        received = TaskflowMessage(
            topic=topic,
            payload=decoded_payload,
            received_at=datetime.now(timezone.utc).isoformat(),
        )
        self.logger.info(
            "received taskflow cancel topic=%s bytes=%s",
            received.topic,
            received.payload_bytes,
        )
        self.record_event(
            RuntimeEvent(
                event_type="taskflow_cancel_received",
                message="taskflow cancel request received",
                topic=received.topic,
                payload={
                    "payload_bytes": received.payload_bytes,
                    "payload_preview": payload_preview(
                        received.payload,
                        config=self._payload_sanitizer_config,
                    ),
                },
            )
        )

        if self.on_taskflow_cancel_message is None:
            self.logger.warning("taskflow cancel handler is not configured")
            return

        try:
            self.on_taskflow_cancel_message(received)
        except Exception as error:
            self.logger.exception("taskflow cancel handler failed")
            self.record_event(
                RuntimeEvent(
                    event_type="taskflow_cancel_handler_error",
                    level="error",
                    message=str(error),
                    topic=received.topic,
                )
            )

    def handle_robot_state_message(self, topic: str, payload: bytes) -> None:
        decoded_payload = self.decode_utf8_payload(
            topic,
            payload,
            route="robot_state",
        )
        if decoded_payload is None:
            return
        received = TaskflowMessage(
            topic=topic,
            payload=decoded_payload,
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
                    "payload_preview": payload_preview(
                        received.payload,
                        config=self._payload_sanitizer_config,
                    ),
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

    def decode_utf8_payload(
        self,
        topic: str,
        payload: bytes,
        *,
        route: str,
    ) -> str | None:
        try:
            return payload.decode("utf-8")
        except UnicodeDecodeError as error:
            # MQTT 回调运行在 paho 网络线程，坏编码只能记录并丢弃，不能把异常抛回网络循环。
            preview = payload.decode("utf-8", errors="replace")
            self.logger.warning("invalid UTF-8 MQTT payload topic=%s: %s", topic, error)
            self.record_event(
                RuntimeEvent(
                    event_type="mqtt_invalid_payload_encoding",
                    level="warning",
                    message=str(error),
                    topic=topic,
                    payload={
                        "route": route,
                        "payload_bytes": len(payload),
                        "payload_preview": payload_preview(
                            preview,
                            config=self._payload_sanitizer_config,
                        ),
                    },
                )
            )
            return None

    def record_event(self, event: RuntimeEvent) -> None:
        if self.event_writer is None:
            return
        try:
            self.event_writer.write(event)
        except Exception:
            self.logger.exception("failed to write runtime event")

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
            if self.on_taskflow_cancel_message is not None:
                self.logger.info(
                    "MQTT connected, subscribing %s",
                    self.settings.taskflow_cancel_topic_filter,
                )
                client.subscribe(self.settings.taskflow_cancel_topic_filter, qos=0)
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
        try:
            topic = normalize_mqtt_topic(getattr(message, "topic", ""))
            payload = normalize_mqtt_payload(getattr(message, "payload", b""))
            if (
                self.on_robot_state_message is not None
                and topic in self.settings.robot_state_request_topics
            ):
                self.handle_robot_state_message(topic, payload)
                return
            if (
                self.on_taskflow_cancel_message is not None
                and mqtt_topic_matches_filter(
                    self.settings.taskflow_cancel_topic_filter,
                    topic,
                )
            ):
                self.handle_taskflow_cancel_message(topic, payload)
                return
            self.handle_message(topic, payload)
        except Exception as error:
            # paho 的 on_message 在网络线程执行，异常不能上抛，否则进程可能活着但永久失聪。
            topic = normalize_mqtt_topic(getattr(message, "topic", ""))
            raw_payload = getattr(message, "payload", b"")
            payload_bytes = len(raw_payload) if isinstance(raw_payload, bytes) else None
            self.logger.exception("MQTT message callback failed")
            self.record_event(
                RuntimeEvent(
                    event_type="mqtt_message_route_error",
                    level="error",
                    message=str(error),
                    topic=topic,
                    payload={"payload_bytes": payload_bytes},
                )
            )


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


def normalize_mqtt_topic(topic: Any) -> str:
    if isinstance(topic, str):
        return topic
    if isinstance(topic, bytes):
        return topic.decode("utf-8", errors="replace")
    return str(topic)


def normalize_mqtt_payload(payload: Any) -> bytes:
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, bytearray):
        return bytes(payload)
    if isinstance(payload, str):
        return payload.encode("utf-8")
    raise TypeError(f"MQTT payload must be bytes or str, got {type(payload).__name__}")


def mqtt_topic_matches_filter(topic_filter: str, topic: str) -> bool:
    filter_parts = topic_filter.split("/")
    topic_parts = topic.split("/")
    for index, filter_part in enumerate(filter_parts):
        if filter_part == "#":
            return index == len(filter_parts) - 1
        if index >= len(topic_parts):
            return False
        if filter_part == "+":
            continue
        if filter_part != topic_parts[index]:
            return False
    return len(topic_parts) == len(filter_parts)


def is_terminal_status_payload(payload: Mapping[str, Any]) -> bool:
    task_state = str(payload.get("task_state") or payload.get("status") or "").upper()
    if task_state in {"OVER", "CANCELED"}:
        return True
    if "terminal_node_id" in payload:
        return True
    return task_state == "ERROR" and "sub_task" not in payload


def publish_mqtt(
    client: Any,
    topic: str,
    payload: str,
    *,
    qos: int,
    retain: bool,
) -> Any:
    try:
        return client.publish(topic, payload, qos=qos, retain=retain)
    except TypeError:
        return client.publish(topic, payload, qos=qos)


def wait_for_publish(result: Any, *, timeout: float | None = None) -> None:
    if hasattr(result, "wait_for_publish"):
        try:
            result.wait_for_publish(timeout=timeout)
        except TypeError:
            result.wait_for_publish()
    rc = getattr(result, "rc", None)
    if rc not in (None, 0):
        logging.getLogger(__name__).error("MQTT publish rejected rc=%s", rc)
