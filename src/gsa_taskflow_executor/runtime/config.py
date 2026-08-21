from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from os import environ
from pathlib import Path
from urllib.parse import urlparse


class ConfigError(ValueError):
    """配置不完整或不安全。"""


EnvMapping = Mapping[str, str]


@dataclass(frozen=True)
class ExecutorSettings:
    """GDK taskflow executor 运行时配置。

    所有字段可通过同名大写环境变量覆盖（如 MQTT_BROKER_URL）。
    broker_url scheme 只支持 mqtt/mqtts/ws/wss。
    """

    # MQTT 连接
    mqtt_broker_url: str = "mqtt://127.0.0.1:1883"
    mqtt_client_id: str = "gsa-taskflow-executor-dev"
    # taskflow 输入/取消/状态 topic
    taskflow_input_topic: str = "gsa/self/taskflow_yaml"
    taskflow_cancel_topic_filter: str = "gsa/self/taskflow/+/cancel"
    taskflow_status_topic_template: str = "gsa/self/{aid}/status"
    # MQTT QoS 和终端状态配置
    mqtt_status_qos: int = 0
    mqtt_terminal_status_qos: int = 1
    mqtt_terminal_status_retain: bool = False
    mqtt_terminal_status_wait_timeout: float = 2.0
    # MQTT 回调线程只做快速入队；队列满时当前只允许显式 reject，避免隐式丢弃控制请求。
    taskflow_queue_maxsize: int = 16
    taskflow_queue_full_policy: str = "reject"
    robot_state_queue_maxsize: int = 8
    robot_state_queue_full_policy: str = "reject"
    diagnostics_mqtt_connect_timeout: float = 2.0
    # 状态/日志默认只传摘要；完整变量和 GDK raw 可能很大且含现场数据，不能直接塞进 MQTT。
    payload_max_string_length: int = 512
    payload_max_collection_items: int = 20
    payload_max_depth: int = 6
    payload_include_full_variables: bool = False
    # 机器人位姿
    robot_current_pose_request_topic: str = "gsa/self/robot/state/get_current_pose/request"
    robot_current_pose_response_topic: str = "gsa/self/robot/state/get_current_pose/response"
    # 相机单帧
    robot_camera_frame_request_topic: str = "gsa/self/robot/state/get_camera_frame/request"
    robot_camera_frame_response_topic: str = "gsa/self/robot/state/get_camera_frame/response"
    # 相机标定
    robot_camera_calibration_request_topic: str = (
        "gsa/self/robot/state/get_camera_calibration/request"
    )
    robot_camera_calibration_response_topic: str = (
        "gsa/self/robot/state/get_camera_calibration/response"
    )
    # 相机连续采集
    robot_camera_capture_start_request_topic: str = (
        "gsa/self/robot/state/camera_capture/start/request"
    )
    robot_camera_capture_start_response_topic: str = (
        "gsa/self/robot/state/camera_capture/start/response"
    )
    robot_camera_capture_stop_request_topic: str = (
        "gsa/self/robot/state/camera_capture/stop/request"
    )
    robot_camera_capture_stop_response_topic: str = (
        "gsa/self/robot/state/camera_capture/stop/response"
    )
    robot_camera_capture_frame_topic_template: str = (
        "gsa/self/robot/state/camera_capture/{sessionId}/frame"
    )
    # 二维码建图远端资源服务。客户端只传 robotSerial/projectName，路径由 executor 侧生成。
    gsa_data_root: str = "/data/gsa"
    qr_mapping_project_path_request_topic: str = (
        "gsa/self/robot/qr_mapping/get_qr_project_path/request"
    )
    qr_mapping_project_path_response_topic: str = (
        "gsa/self/robot/qr_mapping/get_qr_project_path/response"
    )
    qr_mapping_project_snapshot_request_topic: str = (
        "gsa/self/robot/qr_mapping/get_qr_project_snapshot/request"
    )
    qr_mapping_project_snapshot_response_topic: str = (
        "gsa/self/robot/qr_mapping/get_qr_project_snapshot/response"
    )
    qr_mapping_project_list_request_topic: str = (
        "gsa/self/robot/qr_mapping/list_qr_projects/request"
    )
    qr_mapping_project_list_response_topic: str = (
        "gsa/self/robot/qr_mapping/list_qr_projects/response"
    )
    qr_mapping_capture_start_request_topic: str = (
        "gsa/self/robot/qr_mapping/start_capture/request"
    )
    qr_mapping_capture_start_response_topic: str = (
        "gsa/self/robot/qr_mapping/start_capture/response"
    )
    qr_mapping_capture_stop_request_topic: str = (
        "gsa/self/robot/qr_mapping/stop_capture/request"
    )
    qr_mapping_capture_stop_response_topic: str = (
        "gsa/self/robot/qr_mapping/stop_capture/response"
    )
    qr_mapping_capture_frame_topic_template: str = (
        "gsa/self/robot/qr_mapping/capture/{sessionId}/frame"
    )
    qr_mapping_build_map_request_topic: str = (
        "gsa/self/robot/qr_mapping/build_map/request"
    )
    qr_mapping_build_map_response_topic: str = (
        "gsa/self/robot/qr_mapping/build_map/response"
    )
    qr_mapping_delete_map_request_topic: str = (
        "gsa/self/robot/qr_mapping/delete_map/request"
    )
    qr_mapping_delete_map_response_topic: str = (
        "gsa/self/robot/qr_mapping/delete_map/response"
    )
    qr_mapping_pcd_preview_request_topic: str = (
        "gsa/self/robot/qr_mapping/read_pcd_preview/request"
    )
    qr_mapping_pcd_preview_response_topic: str = (
        "gsa/self/robot/qr_mapping/read_pcd_preview/response"
    )
    point_recording_save_target_request_topic: str = (
        "gsa/self/robot/qr_mapping/save_target_point/request"
    )
    point_recording_save_target_response_topic: str = (
        "gsa/self/robot/qr_mapping/save_target_point/response"
    )
    point_recording_save_initial_photo_request_topic: str = (
        "gsa/self/robot/qr_mapping/save_initial_photo_point/request"
    )
    point_recording_save_initial_photo_response_topic: str = (
        "gsa/self/robot/qr_mapping/save_initial_photo_point/response"
    )
    point_recording_submit_request_topic: str = (
        "gsa/self/robot/qr_mapping/submit_point_recording/request"
    )
    point_recording_submit_response_topic: str = (
        "gsa/self/robot/qr_mapping/submit_point_recording/response"
    )
    qr_mapping_sdk_path: str = ""
    qr_mapping_sdk_python: str = "python3"
    qr_mapping_build_timeout_seconds: float = 300.0
    qr_localize_sdk_path: str = ""
    qr_localize_sdk_python: str = "python3"
    qr_localize_timeout_seconds: float = 120.0
    # executor 标识
    executor_aid: str = "gsa-dev"
    executor_mode: str = "gdk"
    executor_log_dir: str = "logs"
    skill_registry_file: str = ""

    @classmethod
    def from_env(
        cls,
        env: EnvMapping | None = None,
        env_file: str | Path | None = None,
    ) -> ExecutorSettings:
        """从环境变量加载配置，校验后返回。"""
        source = build_env_source(env=env, env_file=env_file)

        settings = cls(
            mqtt_broker_url=source.get("MQTT_BROKER_URL", cls.mqtt_broker_url).strip(),
            mqtt_client_id=source.get("MQTT_CLIENT_ID", cls.mqtt_client_id).strip(),
            taskflow_input_topic=source.get(
                "TASKFLOW_INPUT_TOPIC",
                cls.taskflow_input_topic,
            ).strip(),
            taskflow_cancel_topic_filter=source.get(
                "TASKFLOW_CANCEL_TOPIC_FILTER",
                cls.taskflow_cancel_topic_filter,
            ).strip(),
            taskflow_status_topic_template=source.get(
                "TASKFLOW_STATUS_TOPIC_TEMPLATE",
                cls.taskflow_status_topic_template,
            ).strip(),
            mqtt_status_qos=read_int(source, "MQTT_STATUS_QOS", cls.mqtt_status_qos),
            mqtt_terminal_status_qos=read_int(
                source,
                "MQTT_TERMINAL_STATUS_QOS",
                cls.mqtt_terminal_status_qos,
            ),
            mqtt_terminal_status_retain=read_bool(
                source,
                "MQTT_TERMINAL_STATUS_RETAIN",
                cls.mqtt_terminal_status_retain,
            ),
            mqtt_terminal_status_wait_timeout=read_float(
                source,
                "MQTT_TERMINAL_STATUS_WAIT_TIMEOUT",
                cls.mqtt_terminal_status_wait_timeout,
            ),
            taskflow_queue_maxsize=read_int(
                source,
                "TASKFLOW_QUEUE_MAXSIZE",
                cls.taskflow_queue_maxsize,
            ),
            taskflow_queue_full_policy=source.get(
                "TASKFLOW_QUEUE_FULL_POLICY",
                cls.taskflow_queue_full_policy,
            ).strip(),
            robot_state_queue_maxsize=read_int(
                source,
                "ROBOT_STATE_QUEUE_MAXSIZE",
                cls.robot_state_queue_maxsize,
            ),
            robot_state_queue_full_policy=source.get(
                "ROBOT_STATE_QUEUE_FULL_POLICY",
                cls.robot_state_queue_full_policy,
            ).strip(),
            diagnostics_mqtt_connect_timeout=read_float(
                source,
                "DIAGNOSTICS_MQTT_CONNECT_TIMEOUT",
                cls.diagnostics_mqtt_connect_timeout,
            ),
            payload_max_string_length=read_int(
                source,
                "PAYLOAD_MAX_STRING_LENGTH",
                cls.payload_max_string_length,
            ),
            payload_max_collection_items=read_int(
                source,
                "PAYLOAD_MAX_COLLECTION_ITEMS",
                cls.payload_max_collection_items,
            ),
            payload_max_depth=read_int(
                source,
                "PAYLOAD_MAX_DEPTH",
                cls.payload_max_depth,
            ),
            payload_include_full_variables=read_bool(
                source,
                "PAYLOAD_INCLUDE_FULL_VARIABLES",
                cls.payload_include_full_variables,
            ),
            robot_current_pose_request_topic=source.get(
                "ROBOT_CURRENT_POSE_REQUEST_TOPIC",
                cls.robot_current_pose_request_topic,
            ).strip(),
            robot_current_pose_response_topic=source.get(
                "ROBOT_CURRENT_POSE_RESPONSE_TOPIC",
                cls.robot_current_pose_response_topic,
            ).strip(),
            robot_camera_frame_request_topic=source.get(
                "ROBOT_CAMERA_FRAME_REQUEST_TOPIC",
                cls.robot_camera_frame_request_topic,
            ).strip(),
            robot_camera_frame_response_topic=source.get(
                "ROBOT_CAMERA_FRAME_RESPONSE_TOPIC",
                cls.robot_camera_frame_response_topic,
            ).strip(),
            robot_camera_calibration_request_topic=source.get(
                "ROBOT_CAMERA_CALIBRATION_REQUEST_TOPIC",
                cls.robot_camera_calibration_request_topic,
            ).strip(),
            robot_camera_calibration_response_topic=source.get(
                "ROBOT_CAMERA_CALIBRATION_RESPONSE_TOPIC",
                cls.robot_camera_calibration_response_topic,
            ).strip(),
            robot_camera_capture_start_request_topic=source.get(
                "ROBOT_CAMERA_CAPTURE_START_REQUEST_TOPIC",
                cls.robot_camera_capture_start_request_topic,
            ).strip(),
            robot_camera_capture_start_response_topic=source.get(
                "ROBOT_CAMERA_CAPTURE_START_RESPONSE_TOPIC",
                cls.robot_camera_capture_start_response_topic,
            ).strip(),
            robot_camera_capture_stop_request_topic=source.get(
                "ROBOT_CAMERA_CAPTURE_STOP_REQUEST_TOPIC",
                cls.robot_camera_capture_stop_request_topic,
            ).strip(),
            robot_camera_capture_stop_response_topic=source.get(
                "ROBOT_CAMERA_CAPTURE_STOP_RESPONSE_TOPIC",
                cls.robot_camera_capture_stop_response_topic,
            ).strip(),
            robot_camera_capture_frame_topic_template=source.get(
                "ROBOT_CAMERA_CAPTURE_FRAME_TOPIC_TEMPLATE",
                cls.robot_camera_capture_frame_topic_template,
            ).strip(),
            gsa_data_root=source.get("GSA_DATA_ROOT", cls.gsa_data_root).strip(),
            qr_mapping_project_path_request_topic=source.get(
                "QR_MAPPING_PROJECT_PATH_REQUEST_TOPIC",
                cls.qr_mapping_project_path_request_topic,
            ).strip(),
            qr_mapping_project_path_response_topic=source.get(
                "QR_MAPPING_PROJECT_PATH_RESPONSE_TOPIC",
                cls.qr_mapping_project_path_response_topic,
            ).strip(),
            qr_mapping_project_snapshot_request_topic=source.get(
                "QR_MAPPING_PROJECT_SNAPSHOT_REQUEST_TOPIC",
                cls.qr_mapping_project_snapshot_request_topic,
            ).strip(),
            qr_mapping_project_snapshot_response_topic=source.get(
                "QR_MAPPING_PROJECT_SNAPSHOT_RESPONSE_TOPIC",
                cls.qr_mapping_project_snapshot_response_topic,
            ).strip(),
            qr_mapping_project_list_request_topic=source.get(
                "QR_MAPPING_PROJECT_LIST_REQUEST_TOPIC",
                cls.qr_mapping_project_list_request_topic,
            ).strip(),
            qr_mapping_project_list_response_topic=source.get(
                "QR_MAPPING_PROJECT_LIST_RESPONSE_TOPIC",
                cls.qr_mapping_project_list_response_topic,
            ).strip(),
            qr_mapping_capture_start_request_topic=source.get(
                "QR_MAPPING_CAPTURE_START_REQUEST_TOPIC",
                cls.qr_mapping_capture_start_request_topic,
            ).strip(),
            qr_mapping_capture_start_response_topic=source.get(
                "QR_MAPPING_CAPTURE_START_RESPONSE_TOPIC",
                cls.qr_mapping_capture_start_response_topic,
            ).strip(),
            qr_mapping_capture_stop_request_topic=source.get(
                "QR_MAPPING_CAPTURE_STOP_REQUEST_TOPIC",
                cls.qr_mapping_capture_stop_request_topic,
            ).strip(),
            qr_mapping_capture_stop_response_topic=source.get(
                "QR_MAPPING_CAPTURE_STOP_RESPONSE_TOPIC",
                cls.qr_mapping_capture_stop_response_topic,
            ).strip(),
            qr_mapping_capture_frame_topic_template=source.get(
                "QR_MAPPING_CAPTURE_FRAME_TOPIC_TEMPLATE",
                cls.qr_mapping_capture_frame_topic_template,
            ).strip(),
            qr_mapping_build_map_request_topic=source.get(
                "QR_MAPPING_BUILD_MAP_REQUEST_TOPIC",
                cls.qr_mapping_build_map_request_topic,
            ).strip(),
            qr_mapping_build_map_response_topic=source.get(
                "QR_MAPPING_BUILD_MAP_RESPONSE_TOPIC",
                cls.qr_mapping_build_map_response_topic,
            ).strip(),
            qr_mapping_delete_map_request_topic=source.get(
                "QR_MAPPING_DELETE_MAP_REQUEST_TOPIC",
                cls.qr_mapping_delete_map_request_topic,
            ).strip(),
            qr_mapping_delete_map_response_topic=source.get(
                "QR_MAPPING_DELETE_MAP_RESPONSE_TOPIC",
                cls.qr_mapping_delete_map_response_topic,
            ).strip(),
            qr_mapping_pcd_preview_request_topic=source.get(
                "QR_MAPPING_PCD_PREVIEW_REQUEST_TOPIC",
                cls.qr_mapping_pcd_preview_request_topic,
            ).strip(),
            qr_mapping_pcd_preview_response_topic=source.get(
                "QR_MAPPING_PCD_PREVIEW_RESPONSE_TOPIC",
                cls.qr_mapping_pcd_preview_response_topic,
            ).strip(),
            point_recording_save_target_request_topic=source.get(
                "POINT_RECORDING_SAVE_TARGET_REQUEST_TOPIC",
                cls.point_recording_save_target_request_topic,
            ).strip(),
            point_recording_save_target_response_topic=source.get(
                "POINT_RECORDING_SAVE_TARGET_RESPONSE_TOPIC",
                cls.point_recording_save_target_response_topic,
            ).strip(),
            point_recording_save_initial_photo_request_topic=source.get(
                "POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TOPIC",
                cls.point_recording_save_initial_photo_request_topic,
            ).strip(),
            point_recording_save_initial_photo_response_topic=source.get(
                "POINT_RECORDING_SAVE_INITIAL_PHOTO_RESPONSE_TOPIC",
                cls.point_recording_save_initial_photo_response_topic,
            ).strip(),
            point_recording_submit_request_topic=source.get(
                "POINT_RECORDING_SUBMIT_REQUEST_TOPIC",
                cls.point_recording_submit_request_topic,
            ).strip(),
            point_recording_submit_response_topic=source.get(
                "POINT_RECORDING_SUBMIT_RESPONSE_TOPIC",
                cls.point_recording_submit_response_topic,
            ).strip(),
            qr_mapping_sdk_path=source.get(
                "QR_MAPPING_SDK_PATH",
                cls.qr_mapping_sdk_path,
            ).strip(),
            qr_mapping_sdk_python=source.get(
                "QR_MAPPING_SDK_PYTHON",
                cls.qr_mapping_sdk_python,
            ).strip(),
            qr_mapping_build_timeout_seconds=read_float(
                source,
                "QR_MAPPING_BUILD_TIMEOUT_SECONDS",
                cls.qr_mapping_build_timeout_seconds,
            ),
            qr_localize_sdk_path=source.get(
                "QR_LOCALIZE_SDK_PATH",
                cls.qr_localize_sdk_path,
            ).strip(),
            qr_localize_sdk_python=source.get(
                "QR_LOCALIZE_SDK_PYTHON",
                cls.qr_localize_sdk_python,
            ).strip(),
            qr_localize_timeout_seconds=read_float(
                source,
                "QR_LOCALIZE_TIMEOUT_SECONDS",
                cls.qr_localize_timeout_seconds,
            ),
            executor_aid=source.get("EXECUTOR_AID", cls.executor_aid).strip(),
            executor_mode=source.get("EXECUTOR_MODE", cls.executor_mode).strip(),
            executor_log_dir=source.get("EXECUTOR_LOG_DIR", cls.executor_log_dir).strip(),
            skill_registry_file=source.get(
                "SKILL_REGISTRY_FILE",
                cls.skill_registry_file,
            ).strip(),
        )
        settings.validate()
        return settings

    @classmethod
    def from_env_file(cls, env_file: str | Path) -> ExecutorSettings:
        """从 env 文件加载配置。"""
        return cls.from_env(env={}, env_file=env_file)

    @property
    def status_topic(self) -> str:
        """格式化的状态上报 topic（替换 {aid}）。"""
        return self.taskflow_status_topic_template.format(aid=self.executor_aid)

    @property
    def robot_state_request_topics(self) -> tuple[str, ...]:
        """所有 robot_state 订阅 topic。"""
        return (
            self.robot_current_pose_request_topic,
            self.robot_camera_frame_request_topic,
            self.robot_camera_calibration_request_topic,
            self.robot_camera_capture_start_request_topic,
            self.robot_camera_capture_stop_request_topic,
            self.qr_mapping_project_path_request_topic,
            self.qr_mapping_project_snapshot_request_topic,
            self.qr_mapping_project_list_request_topic,
            self.qr_mapping_capture_start_request_topic,
            self.qr_mapping_capture_stop_request_topic,
            self.qr_mapping_build_map_request_topic,
            self.qr_mapping_delete_map_request_topic,
            self.qr_mapping_pcd_preview_request_topic,
            self.point_recording_save_target_request_topic,
            self.point_recording_save_initial_photo_request_topic,
            self.point_recording_submit_request_topic,
        )

    @property
    def log_dir_path(self) -> Path:
        return Path(self.executor_log_dir).expanduser()

    @property
    def execution_log_dir(self) -> Path:
        """JSONL 事件日志目录。"""
        return self.log_dir_path / "executions"

    def validate(self) -> None:
        """校验所有必填字段、MQTT scheme、QoS 范围等。"""
        require_non_empty("MQTT_BROKER_URL", self.mqtt_broker_url)
        require_non_empty("MQTT_CLIENT_ID", self.mqtt_client_id)
        require_non_empty("TASKFLOW_INPUT_TOPIC", self.taskflow_input_topic)
        require_non_empty("TASKFLOW_CANCEL_TOPIC_FILTER", self.taskflow_cancel_topic_filter)
        require_non_empty("TASKFLOW_STATUS_TOPIC_TEMPLATE", self.taskflow_status_topic_template)
        require_non_empty(
            "ROBOT_CURRENT_POSE_REQUEST_TOPIC",
            self.robot_current_pose_request_topic,
        )
        require_non_empty(
            "ROBOT_CURRENT_POSE_RESPONSE_TOPIC",
            self.robot_current_pose_response_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_FRAME_REQUEST_TOPIC",
            self.robot_camera_frame_request_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_FRAME_RESPONSE_TOPIC",
            self.robot_camera_frame_response_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CALIBRATION_REQUEST_TOPIC",
            self.robot_camera_calibration_request_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CALIBRATION_RESPONSE_TOPIC",
            self.robot_camera_calibration_response_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CAPTURE_START_REQUEST_TOPIC",
            self.robot_camera_capture_start_request_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CAPTURE_START_RESPONSE_TOPIC",
            self.robot_camera_capture_start_response_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CAPTURE_STOP_REQUEST_TOPIC",
            self.robot_camera_capture_stop_request_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CAPTURE_STOP_RESPONSE_TOPIC",
            self.robot_camera_capture_stop_response_topic,
        )
        require_non_empty(
            "ROBOT_CAMERA_CAPTURE_FRAME_TOPIC_TEMPLATE",
            self.robot_camera_capture_frame_topic_template,
        )
        require_non_empty("GSA_DATA_ROOT", self.gsa_data_root)
        require_non_empty(
            "QR_MAPPING_PROJECT_PATH_REQUEST_TOPIC",
            self.qr_mapping_project_path_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_PROJECT_PATH_RESPONSE_TOPIC",
            self.qr_mapping_project_path_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_PROJECT_SNAPSHOT_REQUEST_TOPIC",
            self.qr_mapping_project_snapshot_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_PROJECT_SNAPSHOT_RESPONSE_TOPIC",
            self.qr_mapping_project_snapshot_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_PROJECT_LIST_REQUEST_TOPIC",
            self.qr_mapping_project_list_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_PROJECT_LIST_RESPONSE_TOPIC",
            self.qr_mapping_project_list_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_CAPTURE_START_REQUEST_TOPIC",
            self.qr_mapping_capture_start_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_CAPTURE_START_RESPONSE_TOPIC",
            self.qr_mapping_capture_start_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_CAPTURE_STOP_REQUEST_TOPIC",
            self.qr_mapping_capture_stop_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_CAPTURE_STOP_RESPONSE_TOPIC",
            self.qr_mapping_capture_stop_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_CAPTURE_FRAME_TOPIC_TEMPLATE",
            self.qr_mapping_capture_frame_topic_template,
        )
        require_non_empty(
            "QR_MAPPING_BUILD_MAP_REQUEST_TOPIC",
            self.qr_mapping_build_map_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_BUILD_MAP_RESPONSE_TOPIC",
            self.qr_mapping_build_map_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_DELETE_MAP_REQUEST_TOPIC",
            self.qr_mapping_delete_map_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_DELETE_MAP_RESPONSE_TOPIC",
            self.qr_mapping_delete_map_response_topic,
        )
        require_non_empty(
            "QR_MAPPING_PCD_PREVIEW_REQUEST_TOPIC",
            self.qr_mapping_pcd_preview_request_topic,
        )
        require_non_empty(
            "QR_MAPPING_PCD_PREVIEW_RESPONSE_TOPIC",
            self.qr_mapping_pcd_preview_response_topic,
        )
        require_non_empty(
            "POINT_RECORDING_SAVE_TARGET_REQUEST_TOPIC",
            self.point_recording_save_target_request_topic,
        )
        require_non_empty(
            "POINT_RECORDING_SAVE_TARGET_RESPONSE_TOPIC",
            self.point_recording_save_target_response_topic,
        )
        require_non_empty(
            "POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TOPIC",
            self.point_recording_save_initial_photo_request_topic,
        )
        require_non_empty(
            "POINT_RECORDING_SAVE_INITIAL_PHOTO_RESPONSE_TOPIC",
            self.point_recording_save_initial_photo_response_topic,
        )
        require_non_empty(
            "POINT_RECORDING_SUBMIT_REQUEST_TOPIC",
            self.point_recording_submit_request_topic,
        )
        require_non_empty(
            "POINT_RECORDING_SUBMIT_RESPONSE_TOPIC",
            self.point_recording_submit_response_topic,
        )
        require_non_empty("QR_MAPPING_SDK_PYTHON", self.qr_mapping_sdk_python)
        require_non_empty("QR_LOCALIZE_SDK_PYTHON", self.qr_localize_sdk_python)
        require_non_empty("EXECUTOR_AID", self.executor_aid)
        require_non_empty("EXECUTOR_MODE", self.executor_mode)
        require_non_empty("EXECUTOR_LOG_DIR", self.executor_log_dir)

        broker = urlparse(self.mqtt_broker_url)
        if broker.scheme not in {"mqtt", "mqtts", "ws", "wss"}:
            raise ConfigError(
                "MQTT_BROKER_URL 只支持 mqtt、mqtts、ws 或 wss scheme"
            )
        if not broker.hostname:
            raise ConfigError("MQTT_BROKER_URL 必须包含 broker host")
        if "{aid}" not in self.taskflow_status_topic_template:
            raise ConfigError("TASKFLOW_STATUS_TOPIC_TEMPLATE 必须包含 {aid}")
        if not 0 <= self.mqtt_status_qos <= 2:
            raise ConfigError("MQTT_STATUS_QOS 必须在 0~2 之间")
        if not 0 <= self.mqtt_terminal_status_qos <= 2:
            raise ConfigError("MQTT_TERMINAL_STATUS_QOS 必须在 0~2 之间")
        if self.mqtt_terminal_status_wait_timeout <= 0:
            raise ConfigError("MQTT_TERMINAL_STATUS_WAIT_TIMEOUT 必须大于 0")
        if self.taskflow_queue_maxsize <= 0:
            raise ConfigError("TASKFLOW_QUEUE_MAXSIZE 必须大于 0")
        if self.robot_state_queue_maxsize <= 0:
            raise ConfigError("ROBOT_STATE_QUEUE_MAXSIZE 必须大于 0")
        if self.taskflow_queue_full_policy != "reject":
            raise ConfigError("TASKFLOW_QUEUE_FULL_POLICY 当前只支持 reject")
        if self.robot_state_queue_full_policy != "reject":
            raise ConfigError("ROBOT_STATE_QUEUE_FULL_POLICY 当前只支持 reject")
        if self.diagnostics_mqtt_connect_timeout <= 0:
            raise ConfigError("DIAGNOSTICS_MQTT_CONNECT_TIMEOUT 必须大于 0")
        if self.payload_max_string_length <= 0:
            raise ConfigError("PAYLOAD_MAX_STRING_LENGTH 必须大于 0")
        if self.payload_max_collection_items <= 0:
            raise ConfigError("PAYLOAD_MAX_COLLECTION_ITEMS 必须大于 0")
        if self.payload_max_depth <= 0:
            raise ConfigError("PAYLOAD_MAX_DEPTH 必须大于 0")
        if "{sessionId}" not in self.robot_camera_capture_frame_topic_template:
            raise ConfigError("ROBOT_CAMERA_CAPTURE_FRAME_TOPIC_TEMPLATE 必须包含 {sessionId}")
        if "{sessionId}" not in self.qr_mapping_capture_frame_topic_template:
            raise ConfigError("QR_MAPPING_CAPTURE_FRAME_TOPIC_TEMPLATE 必须包含 {sessionId}")
        if self.qr_mapping_build_timeout_seconds <= 0:
            raise ConfigError("QR_MAPPING_BUILD_TIMEOUT_SECONDS 必须大于 0")
        if self.qr_localize_timeout_seconds <= 0:
            raise ConfigError("QR_LOCALIZE_TIMEOUT_SECONDS 必须大于 0")
        if self.executor_mode != "gdk":
            raise ConfigError("EXECUTOR_MODE 只支持 gdk")

    def to_dict(self) -> dict[str, object]:
        """转为 dict（含计算属性 status_topic 和 execution_log_dir）。"""
        data = asdict(self)
        data["status_topic"] = self.status_topic
        data["execution_log_dir"] = str(self.execution_log_dir)
        return data


def require_non_empty(name: str, value: str) -> None:
    """断言 value 非空字符串。"""
    if not value:
        raise ConfigError(f"{name} 不能为空")


def read_int(source: Mapping[str, str], name: str, default: int) -> int:
    """读取整型环境变量。"""
    value = source.get(name)
    if value is None:
        return default
    try:
        return int(value.strip())
    except ValueError as error:
        raise ConfigError(f"{name} 必须是整数") from error


def read_float(source: Mapping[str, str], name: str, default: float) -> float:
    """读取浮点型环境变量。"""
    value = source.get(name)
    if value is None:
        return default
    try:
        return float(value.strip())
    except ValueError as error:
        raise ConfigError(f"{name} 必须是数字") from error


def read_bool(source: Mapping[str, str], name: str, default: bool) -> bool:
    """读取布尔型环境变量。支持 1/0、true/false、yes/no、y/n、on/off。"""
    value = source.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "no", "n", "off"}:
        return False
    raise ConfigError(f"{name} 必须是布尔值")


def read_env_file(path: Path) -> dict[str, str]:
    """读取 KEY=VALUE 格式的 env 文件。跳过空行和 # 注释行，支持 export 前缀。"""
    if not path.exists():
        raise ConfigError(f"env 文件不存在: {path}")
    if not path.is_file():
        raise ConfigError(f"env 路径不是文件: {path}")

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            raise ConfigError(f"{path}:{line_number} 缺少 KEY=VALUE 格式")

        key, value = line.split("=", 1)
        key = key.strip()
        if not key:
            raise ConfigError(f"{path}:{line_number} env key 不能为空")
        values[key] = strip_env_value(value.strip())

    return values


def build_env_source(
    env: EnvMapping | None = None,
    env_file: str | Path | None = None,
) -> dict[str, str]:
    """构建环境变量源：env_file 作为基础层，进程 env（或传入 env）作为覆盖层。"""
    source = dict(environ if env is None else env)
    if env_file is not None:
        source = {**read_env_file(Path(env_file)), **source}
    return source


def strip_env_value(value: str) -> str:
    """去除 env 值的引号包裹（单引号或双引号）。"""
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value
