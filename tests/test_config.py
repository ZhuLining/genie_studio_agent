import pytest

from gsa_taskflow_executor.runtime.config import (
    ConfigError,
    ExecutorSettings,
    build_env_source,
    read_env_file,
)


def test_default_status_topic() -> None:
    settings = ExecutorSettings()

    assert settings.status_topic == "gsa/self/gsa-dev/status"
    assert (
        settings.robot_current_pose_request_topic
        == "gsa/self/robot/state/get_current_pose/request"
    )
    assert (
        settings.robot_current_pose_response_topic
        == "gsa/self/robot/state/get_current_pose/response"
    )
    assert (
        settings.robot_camera_frame_request_topic
        == "gsa/self/robot/state/get_camera_frame/request"
    )
    assert (
        settings.robot_camera_frame_response_topic
        == "gsa/self/robot/state/get_camera_frame/response"
    )
    assert (
        settings.robot_camera_calibration_request_topic
        == "gsa/self/robot/state/get_camera_calibration/request"
    )
    assert (
        settings.robot_camera_calibration_response_topic
        == "gsa/self/robot/state/get_camera_calibration/response"
    )
    assert (
        settings.robot_camera_capture_start_request_topic
        == "gsa/self/robot/state/camera_capture/start/request"
    )
    assert (
        settings.robot_camera_capture_stop_request_topic
        == "gsa/self/robot/state/camera_capture/stop/request"
    )
    assert (
        settings.robot_camera_capture_frame_topic_template
        == "gsa/self/robot/state/camera_capture/{sessionId}/frame"
    )
    assert settings.gsa_data_root == "/data/gsa"
    assert (
        settings.qr_mapping_project_path_request_topic
        == "gsa/self/robot/qr_mapping/get_qr_project_path/request"
    )
    assert (
        settings.qr_mapping_project_snapshot_request_topic
        == "gsa/self/robot/qr_mapping/get_qr_project_snapshot/request"
    )
    assert (
        settings.qr_mapping_capture_start_request_topic
        == "gsa/self/robot/qr_mapping/start_capture/request"
    )
    assert (
        settings.qr_mapping_capture_frame_topic_template
        == "gsa/self/robot/qr_mapping/capture/{sessionId}/frame"
    )
    assert (
        settings.qr_mapping_build_map_request_topic
        == "gsa/self/robot/qr_mapping/build_map/request"
    )
    assert (
        settings.qr_mapping_pcd_preview_request_topic
        == "gsa/self/robot/qr_mapping/read_pcd_preview/request"
    )
    assert (
        settings.point_recording_save_target_request_topic
        == "gsa/self/robot/qr_mapping/save_target_point/request"
    )
    assert (
        settings.point_recording_save_initial_photo_request_topic
        == "gsa/self/robot/qr_mapping/save_initial_photo_point/request"
    )
    assert (
        settings.point_recording_submit_request_topic
        == "gsa/self/robot/qr_mapping/submit_point_recording/request"
    )
    assert settings.qr_mapping_sdk_path == ""
    assert settings.qr_mapping_sdk_python == "python3"
    assert settings.qr_mapping_build_timeout_seconds == 300.0
    assert settings.qr_localize_sdk_path == ""
    assert settings.qr_localize_sdk_python == "python3"
    assert settings.qr_localize_timeout_seconds == 120.0
    assert (
        "gsa/self/robot/qr_mapping/save_target_point/request"
        in settings.robot_state_request_topics
    )
    assert settings.taskflow_cancel_topic_filter == "gsa/self/taskflow/+/cancel"
    assert settings.mqtt_status_qos == 0
    assert settings.mqtt_terminal_status_qos == 1
    assert settings.mqtt_terminal_status_retain is False
    assert settings.mqtt_terminal_status_wait_timeout == 2.0
    assert settings.taskflow_queue_maxsize == 16
    assert settings.taskflow_queue_full_policy == "reject"
    assert settings.robot_state_queue_maxsize == 8
    assert settings.robot_state_queue_full_policy == "reject"
    assert settings.diagnostics_mqtt_connect_timeout == 2.0
    assert settings.payload_max_string_length == 512
    assert settings.payload_max_collection_items == 20
    assert settings.payload_max_depth == 6
    assert settings.payload_include_full_variables is False


def test_env_overrides(monkeypatch) -> None:
    monkeypatch.setenv("MQTT_BROKER_URL", "mqtt://172.17.11.65:1883")
    monkeypatch.setenv("EXECUTOR_AID", "robot-aid")
    monkeypatch.setenv("ROBOT_CURRENT_POSE_REQUEST_TOPIC", "robot/custom/request")
    monkeypatch.setenv("ROBOT_CURRENT_POSE_RESPONSE_TOPIC", "robot/custom/response")
    monkeypatch.setenv("ROBOT_CAMERA_FRAME_REQUEST_TOPIC", "robot/camera/request")
    monkeypatch.setenv("ROBOT_CAMERA_FRAME_RESPONSE_TOPIC", "robot/camera/response")
    monkeypatch.setenv("ROBOT_CAMERA_CALIBRATION_REQUEST_TOPIC", "robot/calibration/request")
    monkeypatch.setenv("ROBOT_CAMERA_CALIBRATION_RESPONSE_TOPIC", "robot/calibration/response")
    monkeypatch.setenv("ROBOT_CAMERA_CAPTURE_START_REQUEST_TOPIC", "robot/capture/start")
    monkeypatch.setenv("ROBOT_CAMERA_CAPTURE_START_RESPONSE_TOPIC", "robot/capture/start/resp")
    monkeypatch.setenv("ROBOT_CAMERA_CAPTURE_STOP_REQUEST_TOPIC", "robot/capture/stop")
    monkeypatch.setenv("ROBOT_CAMERA_CAPTURE_STOP_RESPONSE_TOPIC", "robot/capture/stop/resp")
    monkeypatch.setenv(
        "ROBOT_CAMERA_CAPTURE_FRAME_TOPIC_TEMPLATE",
        "robot/capture/{sessionId}/frame",
    )
    monkeypatch.setenv("GSA_DATA_ROOT", "/tmp/gsa-data")
    monkeypatch.setenv("QR_MAPPING_PROJECT_PATH_REQUEST_TOPIC", "qr/path/request")
    monkeypatch.setenv("QR_MAPPING_PROJECT_PATH_RESPONSE_TOPIC", "qr/path/response")
    monkeypatch.setenv("QR_MAPPING_PROJECT_SNAPSHOT_REQUEST_TOPIC", "qr/snapshot/request")
    monkeypatch.setenv("QR_MAPPING_PROJECT_SNAPSHOT_RESPONSE_TOPIC", "qr/snapshot/response")
    monkeypatch.setenv("QR_MAPPING_CAPTURE_START_REQUEST_TOPIC", "qr/capture/start/request")
    monkeypatch.setenv("QR_MAPPING_CAPTURE_START_RESPONSE_TOPIC", "qr/capture/start/response")
    monkeypatch.setenv("QR_MAPPING_CAPTURE_STOP_REQUEST_TOPIC", "qr/capture/stop/request")
    monkeypatch.setenv("QR_MAPPING_CAPTURE_STOP_RESPONSE_TOPIC", "qr/capture/stop/response")
    monkeypatch.setenv("QR_MAPPING_CAPTURE_FRAME_TOPIC_TEMPLATE", "qr/capture/{sessionId}/frame")
    monkeypatch.setenv("QR_MAPPING_BUILD_MAP_REQUEST_TOPIC", "qr/build/request")
    monkeypatch.setenv("QR_MAPPING_BUILD_MAP_RESPONSE_TOPIC", "qr/build/response")
    monkeypatch.setenv("QR_MAPPING_DELETE_MAP_REQUEST_TOPIC", "qr/delete/request")
    monkeypatch.setenv("QR_MAPPING_DELETE_MAP_RESPONSE_TOPIC", "qr/delete/response")
    monkeypatch.setenv("QR_MAPPING_PCD_PREVIEW_REQUEST_TOPIC", "qr/pcd/request")
    monkeypatch.setenv("QR_MAPPING_PCD_PREVIEW_RESPONSE_TOPIC", "qr/pcd/response")
    monkeypatch.setenv("POINT_RECORDING_SAVE_TARGET_REQUEST_TOPIC", "point/target/request")
    monkeypatch.setenv("POINT_RECORDING_SAVE_TARGET_RESPONSE_TOPIC", "point/target/response")
    monkeypatch.setenv(
        "POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TOPIC",
        "point/photo/request",
    )
    monkeypatch.setenv(
        "POINT_RECORDING_SAVE_INITIAL_PHOTO_RESPONSE_TOPIC",
        "point/photo/response",
    )
    monkeypatch.setenv("POINT_RECORDING_SUBMIT_REQUEST_TOPIC", "point/submit/request")
    monkeypatch.setenv("POINT_RECORDING_SUBMIT_RESPONSE_TOPIC", "point/submit/response")
    monkeypatch.setenv("QR_MAPPING_SDK_PATH", "/opt/qr_mapping_sdk")
    monkeypatch.setenv("QR_MAPPING_SDK_PYTHON", "/opt/venv/bin/python")
    monkeypatch.setenv("QR_MAPPING_BUILD_TIMEOUT_SECONDS", "123.5")
    monkeypatch.setenv("QR_LOCALIZE_SDK_PATH", "/opt/qr_localize_sdk")
    monkeypatch.setenv("QR_LOCALIZE_SDK_PYTHON", "/opt/localize-venv/bin/python")
    monkeypatch.setenv("QR_LOCALIZE_TIMEOUT_SECONDS", "45.5")
    monkeypatch.setenv("TASKFLOW_CANCEL_TOPIC_FILTER", "robot/taskflow/+/cancel")
    monkeypatch.setenv("MQTT_STATUS_QOS", "1")
    monkeypatch.setenv("MQTT_TERMINAL_STATUS_QOS", "2")
    monkeypatch.setenv("MQTT_TERMINAL_STATUS_RETAIN", "true")
    monkeypatch.setenv("MQTT_TERMINAL_STATUS_WAIT_TIMEOUT", "3.5")
    monkeypatch.setenv("TASKFLOW_QUEUE_MAXSIZE", "32")
    monkeypatch.setenv("TASKFLOW_QUEUE_FULL_POLICY", "reject")
    monkeypatch.setenv("ROBOT_STATE_QUEUE_MAXSIZE", "12")
    monkeypatch.setenv("ROBOT_STATE_QUEUE_FULL_POLICY", "reject")
    monkeypatch.setenv("DIAGNOSTICS_MQTT_CONNECT_TIMEOUT", "4.5")
    monkeypatch.setenv("PAYLOAD_MAX_STRING_LENGTH", "128")
    monkeypatch.setenv("PAYLOAD_MAX_COLLECTION_ITEMS", "5")
    monkeypatch.setenv("PAYLOAD_MAX_DEPTH", "4")
    monkeypatch.setenv("PAYLOAD_INCLUDE_FULL_VARIABLES", "true")

    settings = ExecutorSettings.from_env()

    assert settings.mqtt_broker_url == "mqtt://172.17.11.65:1883"
    assert settings.status_topic == "gsa/self/robot-aid/status"
    assert settings.robot_current_pose_request_topic == "robot/custom/request"
    assert settings.robot_current_pose_response_topic == "robot/custom/response"
    assert settings.robot_camera_frame_request_topic == "robot/camera/request"
    assert settings.robot_camera_frame_response_topic == "robot/camera/response"
    assert settings.robot_camera_calibration_request_topic == "robot/calibration/request"
    assert settings.robot_camera_calibration_response_topic == "robot/calibration/response"
    assert settings.robot_camera_capture_start_request_topic == "robot/capture/start"
    assert settings.robot_camera_capture_start_response_topic == "robot/capture/start/resp"
    assert settings.robot_camera_capture_stop_request_topic == "robot/capture/stop"
    assert settings.robot_camera_capture_stop_response_topic == "robot/capture/stop/resp"
    assert settings.robot_camera_capture_frame_topic_template == "robot/capture/{sessionId}/frame"
    assert settings.gsa_data_root == "/tmp/gsa-data"
    assert settings.qr_mapping_project_path_request_topic == "qr/path/request"
    assert settings.qr_mapping_project_path_response_topic == "qr/path/response"
    assert settings.qr_mapping_project_snapshot_request_topic == "qr/snapshot/request"
    assert settings.qr_mapping_project_snapshot_response_topic == "qr/snapshot/response"
    assert settings.qr_mapping_capture_start_request_topic == "qr/capture/start/request"
    assert settings.qr_mapping_capture_start_response_topic == "qr/capture/start/response"
    assert settings.qr_mapping_capture_stop_request_topic == "qr/capture/stop/request"
    assert settings.qr_mapping_capture_stop_response_topic == "qr/capture/stop/response"
    assert settings.qr_mapping_capture_frame_topic_template == "qr/capture/{sessionId}/frame"
    assert settings.qr_mapping_build_map_request_topic == "qr/build/request"
    assert settings.qr_mapping_build_map_response_topic == "qr/build/response"
    assert settings.qr_mapping_delete_map_request_topic == "qr/delete/request"
    assert settings.qr_mapping_delete_map_response_topic == "qr/delete/response"
    assert settings.qr_mapping_pcd_preview_request_topic == "qr/pcd/request"
    assert settings.qr_mapping_pcd_preview_response_topic == "qr/pcd/response"
    assert settings.point_recording_save_target_request_topic == "point/target/request"
    assert settings.point_recording_save_target_response_topic == "point/target/response"
    assert settings.point_recording_save_initial_photo_request_topic == "point/photo/request"
    assert settings.point_recording_save_initial_photo_response_topic == "point/photo/response"
    assert settings.point_recording_submit_request_topic == "point/submit/request"
    assert settings.point_recording_submit_response_topic == "point/submit/response"
    assert settings.qr_mapping_sdk_path == "/opt/qr_mapping_sdk"
    assert settings.qr_mapping_sdk_python == "/opt/venv/bin/python"
    assert settings.qr_mapping_build_timeout_seconds == 123.5
    assert settings.qr_localize_sdk_path == "/opt/qr_localize_sdk"
    assert settings.qr_localize_sdk_python == "/opt/localize-venv/bin/python"
    assert settings.qr_localize_timeout_seconds == 45.5
    assert settings.taskflow_cancel_topic_filter == "robot/taskflow/+/cancel"
    assert settings.mqtt_status_qos == 1
    assert settings.mqtt_terminal_status_qos == 2
    assert settings.mqtt_terminal_status_retain is True
    assert settings.mqtt_terminal_status_wait_timeout == 3.5
    assert settings.taskflow_queue_maxsize == 32
    assert settings.taskflow_queue_full_policy == "reject"
    assert settings.robot_state_queue_maxsize == 12
    assert settings.robot_state_queue_full_policy == "reject"
    assert settings.diagnostics_mqtt_connect_timeout == 4.5
    assert settings.payload_max_string_length == 128
    assert settings.payload_max_collection_items == 5
    assert settings.payload_max_depth == 4
    assert settings.payload_include_full_variables is True


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
    assert settings.status_topic == "gsa/self/field-aid/status"


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

    assert settings.status_topic == "gsa/self/process-aid/status"


def test_build_env_source_keeps_env_precedence_over_env_file(tmp_path) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "EXECUTOR_AID=file-aid",
                "ENABLE_GDK_CONTROL=1",
                "CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT",
            ]
        ),
        encoding="utf-8",
    )

    source = build_env_source(
        env={"EXECUTOR_AID": "process-aid"},
        env_file=env_file,
    )

    assert source["EXECUTOR_AID"] == "process-aid"
    assert source["ENABLE_GDK_CONTROL"] == "1"
    assert source["CONFIRM_GDK_CONTROL"] == "TASKFLOW_ABS_JOINT"


def test_invalid_status_topic_template() -> None:
    with pytest.raises(ConfigError):
        ExecutorSettings(taskflow_status_topic_template="taskflow/static/status").validate()


def test_invalid_mqtt_status_qos() -> None:
    with pytest.raises(ConfigError):
        ExecutorSettings(mqtt_terminal_status_qos=3).validate()


def test_invalid_queue_config() -> None:
    with pytest.raises(ConfigError):
        ExecutorSettings(taskflow_queue_maxsize=0).validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(robot_state_queue_maxsize=0).validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(taskflow_queue_full_policy="drop_oldest").validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(robot_state_queue_full_policy="drop_oldest").validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(diagnostics_mqtt_connect_timeout=0).validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(payload_max_string_length=0).validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(payload_max_collection_items=0).validate()
    with pytest.raises(ConfigError):
        ExecutorSettings(payload_max_depth=0).validate()


def test_read_env_file_rejects_invalid_line(tmp_path) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text("BROKEN_LINE\n", encoding="utf-8")

    with pytest.raises(ConfigError):
        read_env_file(env_file)
