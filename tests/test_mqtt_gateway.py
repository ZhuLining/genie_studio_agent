import json

import pytest

from gsa_taskflow_executor.mqtt.gateway import (
    MqttGateway,
    MqttGatewayError,
    default_port_for_scheme,
    is_success_reason_code,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter


class FakePublishResult:
    def __init__(self) -> None:
        self.waited = False
        self.timeout: float | None = None

    def wait_for_publish(self, timeout: float | None = None) -> None:
        self.waited = True
        self.timeout = timeout


class FakeClient:
    def __init__(self) -> None:
        self.published: list[tuple[str, str, int, bool]] = []
        self.publish_results: list[FakePublishResult] = []
        self.subscribed: list[tuple[str, int]] = []

    def publish(
        self,
        topic: str,
        payload: str,
        qos: int = 0,
        retain: bool = False,
    ) -> FakePublishResult:
        self.published.append((topic, payload, qos, retain))
        result = FakePublishResult()
        self.publish_results.append(result)
        return result

    def subscribe(self, topic: str, qos: int = 0) -> None:
        self.subscribed.append((topic, qos))


def test_handle_message_calls_handler_and_writes_jsonl(tmp_path) -> None:
    received_payloads: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda message: received_payloads.append(message.payload),
        event_writer=JsonlEventWriter(tmp_path),
    )

    gateway.handle_message("gsa/self/taskflow_yaml", b"start_node: \xe5\xbc\x80\xe5\xa7\x8b\n")

    assert received_payloads == ["start_node: 开始\n"]
    [event_file] = list(tmp_path.glob("*.jsonl"))
    event = json.loads(event_file.read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "taskflow_yaml_received"
    assert event["topic"] == "gsa/self/taskflow_yaml"
    assert event["payload"]["payload_preview"] == {
        "preview": "start_node: 开始\n",
        "characters": len("start_node: 开始\n"),
        "bytes": len("start_node: 开始\n".encode()),
        "truncated": False,
    }


def test_publish_status_uses_status_topic(tmp_path) -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(executor_aid="aid-1"),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    gateway._client = client

    gateway.publish_status({"task_state": "running"})

    [(topic, payload, qos, retain)] = client.published
    assert topic == "gsa/self/aid-1/status"
    assert json.loads(payload) == {"task_state": "running"}
    assert qos == 0
    assert retain is False
    assert client.publish_results[0].waited is False


def test_publish_terminal_status_waits_for_qos_ack(tmp_path) -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(
            executor_aid="aid-1",
            mqtt_terminal_status_qos=1,
            mqtt_terminal_status_wait_timeout=1.25,
        ),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    gateway._client = client

    gateway.publish_status({"task_state": "OVER", "terminal_node_id": "结束"})

    [(topic, payload, qos, retain)] = client.published
    assert topic == "gsa/self/aid-1/status"
    assert json.loads(payload)["terminal_node_id"] == "结束"
    assert qos == 1
    assert retain is False
    assert client.publish_results[0].waited is True
    assert client.publish_results[0].timeout == 1.25


def test_publish_terminal_status_can_skip_qos_ack_wait(tmp_path) -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(
            executor_aid="aid-1",
            mqtt_terminal_status_qos=1,
            mqtt_terminal_status_wait_timeout=1.25,
        ),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    gateway._client = client

    gateway.publish_status(
        {"task_state": "ERROR", "error_msg": "queue full"},
        wait_for_terminal=False,
    )

    [(topic, payload, qos, retain)] = client.published
    assert topic == "gsa/self/aid-1/status"
    assert json.loads(payload)["error_msg"] == "queue full"
    assert qos == 1
    assert retain is False
    assert client.publish_results[0].waited is False


def test_publish_canceled_status_is_terminal_and_waits_for_qos_ack(tmp_path) -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(
            executor_aid="aid-1",
            mqtt_terminal_status_qos=1,
            mqtt_terminal_status_wait_timeout=1.25,
        ),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    gateway._client = client

    gateway.publish_status({"task_state": "CANCELED", "cancel_state": "CANCELED"})

    [(topic, payload, qos, retain)] = client.published
    assert topic == "gsa/self/aid-1/status"
    assert json.loads(payload)["task_state"] == "CANCELED"
    assert qos == 1
    assert retain is False
    assert client.publish_results[0].waited is True
    assert client.publish_results[0].timeout == 1.25


def test_publish_status_requires_connection() -> None:
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
    )

    with pytest.raises(MqttGatewayError):
        gateway.publish_status({"task_state": "running"})


def test_publish_json_uses_given_topic() -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
    )
    gateway._client = client

    gateway.publish_json("gsa/self/robot/state/get_current_pose/response", {"ok": True})

    [(topic, payload, qos, retain)] = client.published
    assert topic == "gsa/self/robot/state/get_current_pose/response"
    assert json.loads(payload) == {"ok": True}
    assert qos == 0
    assert retain is False


def test_robot_state_message_uses_independent_handler() -> None:
    received_taskflow: list[str] = []
    received_robot_state: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda message: received_taskflow.append(message.payload),
        on_robot_state_message=lambda message: received_robot_state.append(message.payload),
    )

    gateway.handle_robot_state_message(
        "gsa/self/robot/state/get_current_pose/request",
        b'{"requestId":"req-1"}',
    )

    assert received_taskflow == []
    assert received_robot_state == ['{"requestId":"req-1"}']


def test_on_connect_subscribes_robot_state_topic_when_handler_configured() -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
        on_robot_state_message=lambda _message: None,
    )

    gateway._on_connect(client, None, None, 0)

    assert client.subscribed == [
        ("gsa/self/taskflow_yaml", 0),
        ("gsa/self/robot/state/get_current_pose/request", 0),
        ("gsa/self/robot/state/get_robot_identity/request", 0),
        ("gsa/self/robot/state/get_camera_frame/request", 0),
        ("gsa/self/robot/state/get_camera_calibration/request", 0),
        ("gsa/self/robot/state/camera_capture/start/request", 0),
        ("gsa/self/robot/state/camera_capture/stop/request", 0),
        ("gsa/self/robot/qr_mapping/get_qr_project_path/request", 0),
        ("gsa/self/robot/qr_mapping/get_qr_project_snapshot/request", 0),
        ("gsa/self/robot/qr_mapping/list_qr_projects/request", 0),
        ("gsa/self/robot/qr_mapping/start_capture/request", 0),
        ("gsa/self/robot/qr_mapping/stop_capture/request", 0),
        ("gsa/self/robot/qr_mapping/build_map/request", 0),
        ("gsa/self/robot/qr_mapping/delete_map/request", 0),
        ("gsa/self/robot/qr_mapping/read_pcd_preview/request", 0),
        ("gsa/self/robot/qr_mapping/save_target_point/request", 0),
        ("gsa/self/robot/qr_mapping/save_initial_photo_point/request", 0),
        ("gsa/self/robot/qr_mapping/submit_point_recording/request", 0),
    ]


def test_on_connect_subscribes_cancel_topic_when_handler_configured() -> None:
    client = FakeClient()
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
        on_taskflow_cancel_message=lambda _message: None,
    )

    gateway._on_connect(client, None, None, 0)

    assert client.subscribed == [
        ("gsa/self/taskflow_yaml", 0),
        ("gsa/self/taskflow/+/cancel", 0),
    ]


def test_on_message_routes_camera_frame_topic_to_robot_state_handler() -> None:
    received_robot_state: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
        on_robot_state_message=lambda message: received_robot_state.append(message.topic),
    )
    message = type(
        "Message",
        (),
        {
            "topic": "gsa/self/robot/state/get_camera_frame/request",
            "payload": b'{"requestId":"req-camera"}',
        },
    )()

    gateway._on_message(None, None, message)

    assert received_robot_state == ["gsa/self/robot/state/get_camera_frame/request"]


def test_on_message_routes_camera_calibration_topic_to_robot_state_handler() -> None:
    received_robot_state: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
        on_robot_state_message=lambda message: received_robot_state.append(message.topic),
    )
    message = type(
        "Message",
        (),
        {
            "topic": "gsa/self/robot/state/get_camera_calibration/request",
            "payload": b'{"requestId":"req-calibration"}',
        },
    )()

    gateway._on_message(None, None, message)

    assert received_robot_state == ["gsa/self/robot/state/get_camera_calibration/request"]


def test_on_message_routes_taskflow_cancel_topic_to_cancel_handler() -> None:
    received_cancel_topics: list[str] = []
    received_taskflows: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda message: received_taskflows.append(message.topic),
        on_taskflow_cancel_message=lambda message: received_cancel_topics.append(message.topic),
    )
    message = type(
        "Message",
        (),
        {
            "topic": "gsa/self/taskflow/run-1/cancel",
            "payload": b'{"reason":"stop"}',
        },
    )()

    gateway._on_message(None, None, message)

    assert received_taskflows == []
    assert received_cancel_topics == ["gsa/self/taskflow/run-1/cancel"]


def test_handlers_do_not_raise_back_to_paho_thread(tmp_path) -> None:
    def fail_handler(_message) -> None:
        raise RuntimeError("handler boom")

    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=fail_handler,
        on_robot_state_message=fail_handler,
        on_taskflow_cancel_message=fail_handler,
        event_writer=JsonlEventWriter(tmp_path),
    )

    gateway.handle_message("gsa/self/taskflow_yaml", b"start_node: start\n")
    gateway.handle_taskflow_cancel_message(
        "gsa/self/taskflow/run-1/cancel",
        b'{"reason":"stop"}',
    )
    gateway.handle_robot_state_message(
        "gsa/self/robot/state/get_current_pose/request",
        b'{"requestId":"req-1"}',
    )

    [event_file] = list(tmp_path.glob("*.jsonl"))
    event_types = [
        json.loads(line)["event_type"]
        for line in event_file.read_text(encoding="utf-8").splitlines()
    ]
    assert "taskflow_yaml_handler_error" in event_types
    assert "taskflow_cancel_handler_error" in event_types
    assert "robot_state_request_handler_error" in event_types


def test_invalid_utf8_payload_is_recorded_and_dropped(tmp_path) -> None:
    received_payloads: list[str] = []
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda message: received_payloads.append(message.payload),
        event_writer=JsonlEventWriter(tmp_path),
    )

    gateway.handle_message("gsa/self/taskflow_yaml", b"\xff\xfe")

    assert received_payloads == []
    [event_file] = list(tmp_path.glob("*.jsonl"))
    event = json.loads(event_file.read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "mqtt_invalid_payload_encoding"
    assert event["topic"] == "gsa/self/taskflow_yaml"
    assert event["payload"]["route"] == "taskflow_yaml"
    assert event["payload"]["payload_bytes"] == 2


def test_on_message_swallows_route_errors(tmp_path) -> None:
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    message = type(
        "Message",
        (),
        {
            "topic": "gsa/self/taskflow_yaml",
            "payload": object(),
        },
    )()

    gateway._on_message(None, None, message)

    [event_file] = list(tmp_path.glob("*.jsonl"))
    event = json.loads(event_file.read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "mqtt_message_route_error"
    assert event["topic"] == "gsa/self/taskflow_yaml"


def test_network_loop_watchdog_raises_when_paho_thread_died(tmp_path) -> None:
    class DeadThread:
        def is_alive(self) -> bool:
            return False

    client = type("Client", (), {"_thread": DeadThread()})()
    gateway = MqttGateway(
        settings=ExecutorSettings(),
        on_taskflow_message=lambda _message: None,
        event_writer=JsonlEventWriter(tmp_path),
    )
    gateway._client = client

    with pytest.raises(MqttGatewayError, match="network loop thread stopped"):
        gateway.ensure_network_loop_alive()

    [event_file] = list(tmp_path.glob("*.jsonl"))
    event = json.loads(event_file.read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "mqtt_loop_thread_dead"


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
