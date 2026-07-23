import json

import pytest

from gsa_taskflow_executor.config import ExecutorSettings
from gsa_taskflow_executor.mqtt_gateway import (
    MqttGateway,
    MqttGatewayError,
    default_port_for_scheme,
    is_success_reason_code,
)
from gsa_taskflow_executor.runtime_logging import JsonlEventWriter


class FakePublishResult:
    def __init__(self) -> None:
        self.waited = False

    def wait_for_publish(self) -> None:
        self.waited = True


class FakeClient:
    def __init__(self) -> None:
        self.published: list[tuple[str, str, int]] = []
        self.publish_results: list[FakePublishResult] = []

    def publish(self, topic: str, payload: str, qos: int = 0) -> FakePublishResult:
        self.published.append((topic, payload, qos))
        result = FakePublishResult()
        self.publish_results.append(result)
        return result


def test_handle_message_calls_handler_and_writes_jsonl(tmp_path) -> None:
    received_payloads: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda message: received_payloads.append(message.payload),
        event_writer=JsonlEventWriter(tmp_path),
    )

    gateway.handle_message("taskflow/taskflow_yaml", b"start_node: \xe5\xbc\x80\xe5\xa7\x8b\n")

    assert received_payloads == ["start_node: 开始\n"]
    [event_file] = list(tmp_path.glob("*.jsonl"))
    event = json.loads(event_file.read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "taskflow_yaml_received"
    assert event["topic"] == "taskflow/taskflow_yaml"


def test_publish_status_uses_status_topic(tmp_path) -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(executor_aid="aid-1"),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    gateway._client = client

    gateway.publish_status({"task_state": "running"})

    [(topic, payload, qos)] = client.published
    assert topic == "taskflow/aid-1/status"
    assert json.loads(payload) == {"task_state": "running"}
    assert qos == 0
    assert client.publish_results[0].waited is False


def test_publish_status_requires_connection() -> None:
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
    )

    with pytest.raises(MqttGatewayError):
        gateway.publish_status({"task_state": "running"})


@pytest.mark.parametrize(
    ("scheme", "port"),
    [("mqtt", 1883), ("mqtts", 8883), ("ws", 80), ("wss", 8883)],
)
def test_default_port_for_scheme(scheme: str, port: int) -> None:
    assert default_port_for_scheme(scheme) == port


def test_success_reason_code() -> None:
    assert is_success_reason_code(0)
    assert is_success_reason_code("Success")
    assert not is_success_reason_code(1)
