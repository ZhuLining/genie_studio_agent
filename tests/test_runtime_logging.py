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
