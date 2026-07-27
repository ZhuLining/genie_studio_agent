import pytest

from gsa_taskflow_executor.config import ConfigError, ExecutorSettings, read_env_file


def test_default_status_topic() -> None:
    settings = ExecutorSettings()

    assert settings.status_topic == "taskflow/gsa-dev/status"
    assert settings.robot_current_pose_request_topic == "robot/state/get_current_pose/request"
    assert settings.robot_current_pose_response_topic == "robot/state/get_current_pose/response"


def test_env_overrides(monkeypatch) -> None:
    monkeypatch.setenv("MQTT_BROKER_URL", "mqtt://172.17.11.65:1883")
    monkeypatch.setenv("EXECUTOR_AID", "robot-aid")
    monkeypatch.setenv("ROBOT_CURRENT_POSE_REQUEST_TOPIC", "robot/custom/request")
    monkeypatch.setenv("ROBOT_CURRENT_POSE_RESPONSE_TOPIC", "robot/custom/response")

    settings = ExecutorSettings.from_env()

    assert settings.mqtt_broker_url == "mqtt://172.17.11.65:1883"
    assert settings.status_topic == "taskflow/robot-aid/status"
    assert settings.robot_current_pose_request_topic == "robot/custom/request"
    assert settings.robot_current_pose_response_topic == "robot/custom/response"


def test_env_file_loading(tmp_path) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://10.0.0.2:1883",
                "EXECUTOR_AID=field-aid",
                "EXECUTOR_MODE=gdk",
                "SKILL_REGISTRY_FILE=skills.example.yaml",
            ]
        ),
        encoding="utf-8",
    )

    settings = ExecutorSettings.from_env_file(env_file)

    assert settings.mqtt_broker_url == "mqtt://10.0.0.2:1883"
    assert settings.executor_mode == "gdk"
    assert settings.skill_registry_file == "skills.example.yaml"
    assert settings.status_topic == "taskflow/field-aid/status"


def test_executor_mode_allows_gdk() -> None:
    settings = ExecutorSettings(executor_mode="gdk")

    settings.validate()

    assert settings.executor_mode == "gdk"


def test_executor_mode_rejects_mock() -> None:
    with pytest.raises(ConfigError):
        ExecutorSettings(executor_mode="mock").validate()


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
