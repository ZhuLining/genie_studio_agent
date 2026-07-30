from __future__ import annotations

import json
from pathlib import Path

from gsa_taskflow_executor import cli


def test_gdk_readonly_probe_cli_prints_json_and_writes_event(
    tmp_path: Path,
    capsys,
    monkeypatch,
) -> None:
    log_dir = tmp_path / "logs"
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://127.0.0.1:1883",
                "EXECUTOR_LOG_DIR=" + str(log_dir),
            ]
        ),
        encoding="utf-8",
    )
    probe_payload = {
        "available": False,
        "backend": "agibot_gdk.Robot",
        "joint_count": 0,
        "joint_names": [],
        "left_arm_joint_names": [],
        "right_arm_joint_names": [],
        "nonzero_error_joints": [],
        "raw": {},
        "error_stage": "import_agibot_gdk",
        "error_type": "ModuleNotFoundError",
        "error_msg": "No module named agibot_gdk",
    }
    monkeypatch.setattr(cli, "run_gdk_readonly_probe", lambda: probe_payload)

    exit_code = cli.main(["--env-file", str(env_file), "--gdk-readonly-probe"])

    assert exit_code == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed == probe_payload

    log_files = list((log_dir / "executions").glob("*.jsonl"))
    assert len(log_files) == 1
    event = json.loads(log_files[0].read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "gdk_readonly_probe"
    assert event["level"] == "warning"
    assert event["payload"] == {"probe": probe_payload}


def test_gdk_control_probe_cli_prints_json_and_writes_event(
    tmp_path: Path,
    capsys,
    monkeypatch,
) -> None:
    log_dir = tmp_path / "logs"
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://127.0.0.1:1883",
                "EXECUTOR_LOG_DIR=" + str(log_dir),
                "ENABLE_GDK_CONTROL=1",
                "CONFIRM_GDK_CONTROL=HOLD_CURRENT_DUAL_ARM",
            ]
        ),
        encoding="utf-8",
    )
    probe_payload = {
        "available": True,
        "executed": True,
        "backend": "agibot_gdk.Robot",
        "action": "hold_current",
    }
    captured_env: dict[str, str] = {}

    def fake_control_probe(action, *, environ=None):
        assert environ is not None
        captured_env.update(environ)
        return {**probe_payload, "action": action}

    monkeypatch.setattr(cli, "run_gdk_control_probe", fake_control_probe)

    exit_code = cli.main(
        ["--env-file", str(env_file), "--gdk-control-probe", "hold_current"]
    )

    assert exit_code == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed == probe_payload
    assert captured_env["ENABLE_GDK_CONTROL"] == "1"
    assert captured_env["CONFIRM_GDK_CONTROL"] == "HOLD_CURRENT_DUAL_ARM"

    log_files = list((log_dir / "executions").glob("*.jsonl"))
    assert len(log_files) == 1
    event = json.loads(log_files[0].read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "gdk_control_probe"
    assert event["level"] == "info"
    assert event["payload"] == {"probe": probe_payload}
