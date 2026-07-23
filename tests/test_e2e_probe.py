from pathlib import Path

from gsa_taskflow_executor.e2e_probe import (
    format_status_line,
    is_terminal_execution_payload,
    prepare_yaml_payload,
)


def test_prepare_yaml_payload_overrides_app_execution_id(tmp_path: Path) -> None:
    yaml_file = tmp_path / "taskflow.yaml"
    yaml_file.write_text(
        "start_node: 开始\napp_execution_id: old-id\nnodes: []\n",
        encoding="utf-8",
    )

    payload = prepare_yaml_payload(yaml_file, "new-id")

    assert "app_execution_id: new-id" in payload
    assert "app_execution_id: old-id" not in payload


def test_terminal_execution_payload_requires_terminal_node_and_matching_execution_id() -> None:
    assert is_terminal_execution_payload(
        {
            "app_execution_id": "run-1",
            "task_state": "OVER",
            "terminal_node_id": "结束",
        },
        "run-1",
    )
    assert not is_terminal_execution_payload(
        {
            "app_execution_id": "run-1",
            "task_state": "OVER",
            "sub_task": {"node_id": "开始"},
        },
        "run-1",
    )
    assert not is_terminal_execution_payload(
        {
            "app_execution_id": "other-run",
            "task_state": "OVER",
            "terminal_node_id": "结束",
        },
        "run-1",
    )


def test_format_status_line_prints_node_state() -> None:
    line = format_status_line(
        2,
        "taskflow/gsa-dev/status",
        {
            "task_state": "OVER",
            "sub_task": {
                "node_id": "位姿调整-位控",
                "state": "OVER",
            },
        },
    )

    assert line == "[02] taskflow/gsa-dev/status task_state=OVER node=位姿调整-位控 state=OVER"
