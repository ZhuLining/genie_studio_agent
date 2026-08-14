import json

from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent


def test_jsonl_event_writer(tmp_path) -> None:
    writer = JsonlEventWriter(tmp_path)
    path = writer.write(
        RuntimeEvent(
            event_type="node_status",
            message="节点进入 RUNNING",
            app_execution_id="exec-1",
            node_id="位姿调整-位控",
            topic="gsa/self/gsa-dev/status",
            payload={"status": "running"},
        )
    )

    lines = path.read_text(encoding="utf-8").splitlines()
    payload = json.loads(lines[0])

    assert path.parent == tmp_path
    assert payload["event_type"] == "node_status"
    assert payload["node_id"] == "位姿调整-位控"
    assert payload["payload"] == {"status": "running"}


def test_jsonl_event_writer_sanitizes_payload(tmp_path) -> None:
    writer = JsonlEventWriter(tmp_path)
    path = writer.write(
        RuntimeEvent(
            event_type="mqtt_status_published",
            payload={
                "payload": {
                    "api_token": "secret",
                    "variables": {
                        "节点1": {
                            "detail": {
                                "status": "success",
                                "node_type": "worker",
                                "outputs": {"imageBase64": "a" * 1000},
                            }
                        }
                    },
                }
            },
        )
    )

    event = json.loads(path.read_text(encoding="utf-8").splitlines()[0])
    logged_payload = event["payload"]["payload"]

    assert logged_payload["api_token"] == "[REDACTED]"
    assert logged_payload["variables"]["summary_only"] is True
    assert logged_payload["variables"]["nodes"]["节点1"]["output_keys"] == ["imageBase64"]
