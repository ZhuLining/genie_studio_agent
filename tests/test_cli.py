from __future__ import annotations

import json
from pathlib import Path

from gsa_taskflow_executor import cli


def test_print_config_includes_env_file_safety_gate_and_speed_limits(
    tmp_path: Path,
    capsys,
) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://127.0.0.1:1883",
                "ENABLE_GDK_CONTROL=1",
                "CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT",
            ]
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(["--env-file", str(env_file), "--print-config"])

    assert exit_code == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed["mqtt_broker_url"] == "mqtt://127.0.0.1:1883"
    assert printed["taskflow_gdk_safety_gate"] == {
        "enabled": True,
        "confirmed": True,
        "expected_confirmation": "TASKFLOW_ABS_JOINT",
    }
    assert printed["motion_speed_limits"] == {
        "unit": "gdk_velocity",
        "min": 0.001,
        "max": 0.1,
    }


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
    assert event["payload"]["probe"]["available"] is False
    assert event["payload"]["probe"]["error_stage"] == "import_agibot_gdk"
    assert event["payload"]["probe"]["raw"]["omitted"] is True


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


def test_health_check_cli_prints_json_and_returns_health_exit_code(
    tmp_path: Path,
    capsys,
    monkeypatch,
) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://127.0.0.1:1883",
                "DIAGNOSTICS_MQTT_CONNECT_TIMEOUT=0.1",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        cli,
        "build_health_check_payload",
        lambda **_kwargs: {
            "type": "executor_health_check",
            "status": "error",
            "checks": [
                {
                    "name": "mqtt_status_roundtrip",
                    "status": "error",
                    "message": "failed",
                    "detail": {"ok": False},
                }
            ],
        },
    )

    exit_code = cli.main(["--env-file", str(env_file), "--health-check"])

    assert exit_code == 1
    printed = json.loads(capsys.readouterr().out)
    assert printed["type"] == "executor_health_check"
    assert printed["status"] == "error"


def test_publish_robot_state_queue_error_uses_point_recording_response_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    cli.publish_robot_state_queue_error(
        cli.TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/save_initial_photo_point/request",
            payload=json.dumps({"requestId": "req-photo"}),
            received_at="2026-08-20T00:00:00+00:00",
        ),
        settings=cli.ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        error_message="robot_state queue is full",
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/save_initial_photo_point/response"
    assert payload["type"] == "save_qr_initial_photo_point"
    assert payload["requestId"] == "req-photo"
    assert payload["ok"] is False
    assert payload["error"]["code"] == "QUEUE_UNAVAILABLE"
