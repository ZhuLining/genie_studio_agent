import pytest

from gsa_taskflow_executor.config import ConfigError, ExecutorSettings, read_env_file


def test_default_status_topic() -> None:
    settings = ExecutorSettings()

    assert settings.status_topic == "taskflow/gsa-dev/status"


def test_env_overrides(monkeypatch) -> None:
    monkeypatch.setenv("MQTT_BROKER_URL", "mqtt://172.17.11.65:1883")
    monkeypatch.setenv("EXECUTOR_AID", "robot-aid")

    settings = ExecutorSettings.from_env()

    assert settings.mqtt_broker_url == "mqtt://172.17.11.65:1883"
    assert settings.status_topic == "taskflow/robot-aid/status"


def test_env_file_loading(tmp_path) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://10.0.0.2:1883",
                "EXECUTOR_AID=field-aid",
                "EXECUTOR_MODE=dry-run",
                "SKILL_REGISTRY_FILE=skills.example.yaml",
            ]
        ),
        encoding="utf-8",
    )

    settings = ExecutorSettings.from_env_file(env_file)

    assert settings.mqtt_broker_url == "mqtt://10.0.0.2:1883"
    assert settings.executor_mode == "dry-run"
    assert settings.skill_registry_file == "skills.example.yaml"
    assert settings.status_topic == "taskflow/field-aid/status"


def test_process_env_overrides_env_file(tmp_path, monkeypatch) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text("EXECUTOR_AID=file-aid\n", encoding="utf-8")
    monkeypatch.setenv("EXECUTOR_AID", "process-aid")

    settings = ExecutorSettings.from_env(env_file=env_file)

    assert settings.status_topic == "taskflow/process-aid/status"


def test_invalid_status_topic_template() -> None:
    with pytest.raises(ConfigError):
        ExecutorSettings(taskflow_status_topic_template="taskflow/static/status").validate()


def test_read_env_file_rejects_invalid_line(tmp_path) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text("BROKEN_LINE\n", encoding="utf-8")

    with pytest.raises(ConfigError):
        read_env_file(env_file)
