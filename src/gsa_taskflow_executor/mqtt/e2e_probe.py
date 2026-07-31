from __future__ import annotations

import argparse
import json
import sys
import threading
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from gsa_taskflow_executor.mqtt.gateway import (
    create_client,
    default_port_for_scheme,
    import_paho_mqtt,
    is_success_reason_code,
    wait_for_publish,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings

DEFAULT_TIMEOUT_SEC = 10.0
DEFAULT_INPUT_TOPIC = "gsa/self/taskflow_yaml"
DEFAULT_STATUS_TOPIC = "gsa/self/gsa-dev/status"


@dataclass(frozen=True)
class E2EProbeConfig:
    broker_url: str
    input_topic: str
    status_topic: str
    yaml_file: Path
    timeout_sec: float
    client_id: str
    app_execution_id: str


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gsa-taskflow-e2e",
        description="Publish sample taskflow YAML and wait for executor status callbacks.",
    )
    parser.add_argument(
        "--broker-url",
        default="mqtt://127.0.0.1:1883",
        help="MQTT broker URL, for example mqtt://127.0.0.1:1883.",
    )
    parser.add_argument(
        "--input-topic",
        default=DEFAULT_INPUT_TOPIC,
        help="Topic used to publish taskflow YAML.",
    )
    parser.add_argument(
        "--status-topic",
        default=DEFAULT_STATUS_TOPIC,
        help="Topic used to subscribe executor status payloads.",
    )
    parser.add_argument(
        "--yaml-file",
        type=Path,
        default=default_sample_path(),
        help="Taskflow YAML file to publish.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SEC,
        help="Seconds to wait for a terminal status payload.",
    )
    parser.add_argument(
        "--client-id",
        help="MQTT client id for the probe. A random id is used by default.",
    )
    parser.add_argument(
        "--app-execution-id",
        help="Override app_execution_id before publishing. A random id is used by default.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    config = config_from_args(args)

    try:
        return run_probe(config)
    except Exception as error:
        print(f"E2E probe failed: {error}", file=sys.stderr)
        return 1


def config_from_args(args: argparse.Namespace) -> E2EProbeConfig:
    client_id = str(args.client_id or f"gsa-e2e-probe-{uuid.uuid4()}")
    app_execution_id = str(args.app_execution_id or uuid.uuid4())
    return E2EProbeConfig(
        broker_url=str(args.broker_url),
        input_topic=str(args.input_topic),
        status_topic=str(args.status_topic),
        yaml_file=Path(args.yaml_file),
        timeout_sec=float(args.timeout),
        client_id=client_id,
        app_execution_id=app_execution_id,
    )


def run_probe(config: E2EProbeConfig) -> int:
    payload = prepare_yaml_payload(config.yaml_file, config.app_execution_id)
    broker = urlparse(config.broker_url)
    if broker.scheme not in {"mqtt", "mqtts", "ws", "wss"}:
        raise ValueError(f"不支持的 broker scheme: {broker.scheme}")
    if not broker.hostname:
        raise ValueError("broker URL 缺少 host")

    mqtt = import_paho_mqtt()
    settings = ExecutorSettings(
        mqtt_broker_url=config.broker_url,
        mqtt_client_id=config.client_id,
    )
    client = create_client(mqtt, settings, broker.scheme)
    connected = threading.Event()
    subscribed = threading.Event()
    terminal_received = threading.Event()
    errors: list[str] = []
    received_payloads: list[dict[str, Any]] = []

    def on_connect(
        current_client: Any,
        _userdata: Any,
        _flags: Any,
        reason_code: Any,
        _properties: Any = None,
    ) -> None:
        if not is_success_reason_code(reason_code):
            errors.append(f"MQTT connect failed: {reason_code}")
            connected.set()
            return
        current_client.subscribe(config.status_topic, qos=0)
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
        status_payload = decode_status_payload(message.payload)
        received_payloads.append(status_payload)
        print(format_status_line(len(received_payloads), message.topic, status_payload), flush=True)
        if is_terminal_execution_payload(status_payload, config.app_execution_id):
            terminal_received.set()

    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message

    port = broker.port or default_port_for_scheme(broker.scheme)
    if broker.scheme in {"mqtts", "wss"}:
        client.tls_set()

    print(f"connecting {broker.hostname}:{port}")
    client.connect(broker.hostname, port, keepalive=30)
    client.loop_start()
    try:
        if not connected.wait(timeout=config.timeout_sec):
            print("timeout waiting MQTT connection", file=sys.stderr)
            return 2
        if errors:
            print(errors[0], file=sys.stderr)
            return 2
        if not subscribed.wait(timeout=config.timeout_sec):
            print(f"timeout subscribing {config.status_topic}", file=sys.stderr)
            return 2

        print(f"subscribed {config.status_topic}")
        print(f"publishing {config.input_topic} app_execution_id={config.app_execution_id}")
        publish_result = client.publish(config.input_topic, payload, qos=0)
        wait_for_publish(publish_result)

        if not terminal_received.wait(timeout=config.timeout_sec):
            print("timeout waiting terminal status payload", file=sys.stderr)
            return 3

        print(f"received {len(received_payloads)} status payloads")
        return 0
    finally:
        client.loop_stop()
        client.disconnect()


def prepare_yaml_payload(yaml_file: Path, app_execution_id: str) -> str:
    text = yaml_file.read_text(encoding="utf-8")
    lines = text.splitlines()
    replaced = False
    for index, line in enumerate(lines):
        if line.strip().startswith("app_execution_id:"):
            indent = line[: len(line) - len(line.lstrip())]
            lines[index] = f"{indent}app_execution_id: {app_execution_id}"
            replaced = True
            break
    if not replaced:
        insert_at = 1 if lines else 0
        lines.insert(insert_at, f"app_execution_id: {app_execution_id}")
    return "\n".join(lines) + "\n"


def decode_status_payload(payload: bytes) -> dict[str, Any]:
    text = payload.decode("utf-8")
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return {"raw": text}
    if isinstance(parsed, dict):
        return {str(key): value for key, value in parsed.items()}
    return {"raw": parsed}


def format_status_line(index: int, topic: str, payload: Mapping[str, Any]) -> str:
    sub_task = as_record(payload.get("sub_task")) or {}
    task_state = read_string(payload, ("task_state", "status")) or "-"
    node_id = read_string(sub_task, ("node_id", "node_name")) or "-"
    node_state = read_string(sub_task, ("state", "status")) or "-"
    return f"[{index:02d}] {topic} task_state={task_state} node={node_id} state={node_state}"


def is_terminal_execution_payload(
    payload: Mapping[str, Any],
    expected_app_execution_id: str,
) -> bool:
    app_execution_id = read_string(payload, ("app_execution_id", "appExecutionId"))
    if app_execution_id and app_execution_id != expected_app_execution_id:
        return False
    task_state = (read_string(payload, ("task_state", "status")) or "").upper()
    return task_state in {"OVER", "ERROR"} and "terminal_node_id" in payload


def as_record(value: Any) -> dict[str, Any] | None:
    if value and isinstance(value, dict):
        return {str(key): item for key, item in value.items()}
    return None


def read_string(record: Mapping[str, Any], keys: Sequence[str]) -> str | None:
    for key in keys:
        value = record.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
        if isinstance(value, int | float) and not isinstance(value, bool):
            return str(value)
    return None


def default_sample_path() -> Path:
    return Path(__file__).resolve().parents[2] / "examples" / "right_arm_abs_joint.yaml"


if __name__ == "__main__":
    raise SystemExit(main())
