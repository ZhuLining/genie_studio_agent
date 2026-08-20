"""MQTT 机器人状态请求处理。

负责分发桌面端的只读查询请求：当前位姿、相机帧、相机标定、相机连续采集启停。
请求模型/JSON 解析和响应 builder 已拆到独立模块，便于后续协议版本演进。
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

from gsa_taskflow_executor.gdk.camera_calibration import run_gdk_camera_calibration_snapshot
from gsa_taskflow_executor.gdk.camera_capture import CameraCaptureStartParams
from gsa_taskflow_executor.gdk.camera_frame import run_gdk_camera_frame_snapshot
from gsa_taskflow_executor.gdk.current_pose import run_gdk_current_pose_snapshot
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.qr_mapping.build_service import (
    ACTION_BUILD_QR_MAP,
    ACTION_DELETE_QR_MAP,
    ACTION_READ_QR_PCD_PREVIEW,
    QrBuildService,
)
from gsa_taskflow_executor.qr_mapping.capture_service import (
    ACTION_START_QR_CAPTURE,
    ACTION_STOP_QR_CAPTURE,
    QrCaptureStartParams,
)
from gsa_taskflow_executor.qr_mapping.point_recording_service import (
    PointRecordingSaveInitialPhotoParams,
    PointRecordingSaveTargetParams,
    PointRecordingService,
    PointRecordingSubmitParams,
)
from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.mqtt.robot_state_models import (
    CAMERA_CALIBRATION_REQUEST_TYPE,
    CAMERA_CAPTURE_START_REQUEST_TYPE,
    CAMERA_CAPTURE_STOP_REQUEST_TYPE,
    CAMERA_FRAME_REQUEST_TYPE,
    CURRENT_POSE_REQUEST_TYPE,
    QR_BUILD_MAP_REQUEST_TYPE,
    QR_CAPTURE_START_REQUEST_TYPE,
    QR_CAPTURE_STOP_REQUEST_TYPE,
    QR_DELETE_MAP_REQUEST_TYPE,
    QR_PCD_PREVIEW_REQUEST_TYPE,
    POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE,
    POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE,
    POINT_RECORDING_SUBMIT_REQUEST_TYPE,
    QR_PROJECT_PATH_REQUEST_TYPE,
    QR_PROJECT_SNAPSHOT_REQUEST_TYPE,
    CameraCalibrationRequest,
    CameraCaptureStartRequest,
    CameraCaptureStopRequest,
    CameraFrameRequest,
    CurrentPoseRequest,
    QrBuildMapRequest,
    QrCaptureStartRequest,
    QrCaptureStopRequest,
    QrDeleteMapRequest,
    QrPcdPreviewRequest,
    PointRecordingSaveInitialPhotoRequest,
    PointRecordingSaveTargetRequest,
    PointRecordingSubmitRequest,
    QrProjectPathRequest,
    QrProjectSnapshotRequest,
    parse_camera_calibration_request,
    parse_camera_capture_start_request,
    parse_camera_capture_stop_request,
    parse_camera_frame_request,
    parse_current_pose_request,
    parse_json_object,
    parse_qr_build_map_request,
    parse_qr_capture_start_request,
    parse_qr_capture_stop_request,
    parse_qr_delete_map_request,
    parse_qr_pcd_preview_request,
    parse_point_recording_save_initial_photo_request,
    parse_point_recording_save_target_request,
    parse_point_recording_submit_request,
    parse_qr_project_path_request,
    parse_qr_project_snapshot_request,
    read_bool,
    read_camera_ids,
    read_optional_string,
    read_positive_int,
    read_reply_topic,
    read_request_id_from_object,
    read_request_type,
)
from gsa_taskflow_executor.mqtt.robot_state_responses import (
    CAMERA_BUSY_ERROR_MESSAGE,
    CAMERA_CALIBRATION_BUSY_ERROR_MESSAGE,
    ROBOT_BUSY_ERROR_CODE,
    ROBOT_BUSY_ERROR_MESSAGE,
    build_camera_calibration_response,
    build_camera_capture_start_response,
    build_camera_capture_stop_response,
    build_camera_frame_response,
    build_current_pose_response,
    build_qr_build_map_response,
    build_qr_capture_start_response,
    build_qr_capture_stop_response,
    build_qr_delete_map_response,
    build_qr_pcd_preview_response,
    build_point_recording_save_initial_photo_response,
    build_point_recording_save_target_response,
    build_point_recording_submit_response,
    build_qr_project_path_response,
    build_qr_project_snapshot_response,
    error_response,
    read_error_code,
    read_error_message,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent
from gsa_taskflow_executor.qr_mapping.project_store import (
    ACTION_GET_QR_PROJECT_PATH,
    ACTION_GET_QR_PROJECT_SNAPSHOT,
    QrProjectStore,
)

__all__ = [
    "CAMERA_BUSY_ERROR_MESSAGE",
    "CAMERA_CALIBRATION_BUSY_ERROR_MESSAGE",
    "CAMERA_CALIBRATION_REQUEST_TYPE",
    "CAMERA_CAPTURE_START_REQUEST_TYPE",
    "CAMERA_CAPTURE_STOP_REQUEST_TYPE",
    "CAMERA_FRAME_REQUEST_TYPE",
    "CURRENT_POSE_REQUEST_TYPE",
    "QR_BUILD_MAP_REQUEST_TYPE",
    "QR_CAPTURE_START_REQUEST_TYPE",
    "QR_CAPTURE_STOP_REQUEST_TYPE",
    "QR_DELETE_MAP_REQUEST_TYPE",
    "QR_PCD_PREVIEW_REQUEST_TYPE",
    "POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE",
    "POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE",
    "POINT_RECORDING_SUBMIT_REQUEST_TYPE",
    "QR_PROJECT_PATH_REQUEST_TYPE",
    "QR_PROJECT_SNAPSHOT_REQUEST_TYPE",
    "ROBOT_BUSY_ERROR_CODE",
    "ROBOT_BUSY_ERROR_MESSAGE",
    "CameraCalibrationRequest",
    "CameraCaptureStartRequest",
    "CameraCaptureStopRequest",
    "CameraFrameRequest",
    "CurrentPoseRequest",
    "QrBuildMapRequest",
    "QrCaptureStartRequest",
    "QrCaptureStopRequest",
    "QrDeleteMapRequest",
    "QrPcdPreviewRequest",
    "PointRecordingSaveInitialPhotoRequest",
    "PointRecordingSaveTargetRequest",
    "PointRecordingSubmitRequest",
    "QrProjectPathRequest",
    "QrProjectSnapshotRequest",
    "build_camera_calibration_response",
    "build_camera_capture_start_response",
    "build_camera_capture_stop_response",
    "build_camera_frame_response",
    "build_current_pose_response",
    "build_qr_build_map_response",
    "build_qr_capture_start_response",
    "build_qr_capture_stop_response",
    "build_qr_delete_map_response",
    "build_qr_pcd_preview_response",
    "build_point_recording_save_initial_photo_response",
    "build_point_recording_save_target_response",
    "build_point_recording_submit_response",
    "build_qr_project_path_response",
    "build_qr_project_snapshot_response",
    "error_response",
    "handle_camera_calibration_request",
    "handle_camera_capture_start_request",
    "handle_camera_capture_stop_request",
    "handle_camera_frame_request",
    "handle_current_pose_request",
    "handle_qr_build_map_request",
    "handle_qr_capture_start_request",
    "handle_qr_capture_stop_request",
    "handle_qr_delete_map_request",
    "handle_qr_pcd_preview_request",
    "handle_point_recording_save_initial_photo_request",
    "handle_point_recording_save_target_request",
    "handle_point_recording_submit_request",
    "handle_robot_state_request",
    "parse_camera_calibration_request",
    "parse_camera_capture_start_request",
    "parse_camera_capture_stop_request",
    "parse_camera_frame_request",
    "parse_current_pose_request",
    "parse_json_object",
    "parse_qr_build_map_request",
    "parse_qr_capture_start_request",
    "parse_qr_capture_stop_request",
    "parse_qr_delete_map_request",
    "parse_qr_pcd_preview_request",
    "parse_point_recording_save_initial_photo_request",
    "parse_point_recording_save_target_request",
    "parse_point_recording_submit_request",
    "parse_qr_project_path_request",
    "parse_qr_project_snapshot_request",
    "read_bool",
    "read_camera_ids",
    "read_error_code",
    "read_error_message",
    "read_optional_string",
    "read_positive_int",
    "read_reply_topic",
    "read_request_id_from_object",
    "read_request_type",
    "write_robot_state_event",
]

# 回调类型别名
RobotStatePublisher = Callable[[str, Mapping[str, Any]], None]
CurrentPoseCollector = Callable[[], Mapping[str, object]]
CameraFrameCollector = Callable[[str, int], Mapping[str, object]]
CameraCalibrationCollector = Callable[[tuple[str, ...], int, bool], Mapping[str, object]]
CameraCaptureStartCollector = Callable[[CameraCaptureStartParams], Mapping[str, object]]
CameraCaptureStopCollector = Callable[[str], Mapping[str, object]]
QrProjectPathCollector = Callable[[str, str], Mapping[str, object]]
QrProjectSnapshotCollector = Callable[[str, str, int], Mapping[str, object]]
QrCaptureStartCollector = Callable[[QrCaptureStartParams], Mapping[str, object]]
QrCaptureStopCollector = Callable[[str], Mapping[str, object]]
QrBuildMapCollector = Callable[[str, str, str, str, str, float], Mapping[str, object]]
QrDeleteMapCollector = Callable[[str, str, str], Mapping[str, object]]
QrPcdPreviewCollector = Callable[[str, str, str, int], Mapping[str, object]]
PointRecordingSaveTargetCollector = Callable[
    [PointRecordingSaveTargetParams],
    Mapping[str, object],
]
PointRecordingSaveInitialPhotoCollector = Callable[
    [PointRecordingSaveInitialPhotoParams],
    Mapping[str, object],
]
PointRecordingSubmitCollector = Callable[[PointRecordingSubmitParams], Mapping[str, object]]


def handle_robot_state_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_current_pose: CurrentPoseCollector = run_gdk_current_pose_snapshot,
    collect_camera_frame: CameraFrameCollector = run_gdk_camera_frame_snapshot,
    collect_camera_calibration: CameraCalibrationCollector = run_gdk_camera_calibration_snapshot,
    start_camera_capture: CameraCaptureStartCollector | None = None,
    stop_camera_capture: CameraCaptureStopCollector | None = None,
    get_qr_project_path: QrProjectPathCollector | None = None,
    get_qr_project_snapshot: QrProjectSnapshotCollector | None = None,
    start_qr_capture: QrCaptureStartCollector | None = None,
    stop_qr_capture: QrCaptureStopCollector | None = None,
    build_qr_map: QrBuildMapCollector | None = None,
    delete_qr_map: QrDeleteMapCollector | None = None,
    read_qr_pcd_preview: QrPcdPreviewCollector | None = None,
    save_point_recording_target: PointRecordingSaveTargetCollector | None = None,
    save_point_recording_initial_photo: PointRecordingSaveInitialPhotoCollector | None = None,
    submit_point_recording: PointRecordingSubmitCollector | None = None,
) -> None:
    """按请求类型分发到对应 handler。通过 payload type 字段和 MQTT topic 双重匹配。"""

    request_type = read_request_type(message.payload)
    if (
        request_type == POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE
        or message.topic == settings.point_recording_save_target_request_topic
    ):
        handle_point_recording_save_target_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            save_target=save_point_recording_target,
        )
        return

    if (
        request_type == POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE
        or message.topic == settings.point_recording_save_initial_photo_request_topic
    ):
        handle_point_recording_save_initial_photo_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            save_initial_photo=save_point_recording_initial_photo,
        )
        return

    if (
        request_type == POINT_RECORDING_SUBMIT_REQUEST_TYPE
        or message.topic == settings.point_recording_submit_request_topic
    ):
        handle_point_recording_submit_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            submit_recording=submit_point_recording,
        )
        return

    if (
        request_type == QR_CAPTURE_START_REQUEST_TYPE
        or message.topic == settings.qr_mapping_capture_start_request_topic
    ):
        handle_qr_capture_start_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            start_qr_capture=start_qr_capture,
        )
        return

    if (
        request_type == QR_CAPTURE_STOP_REQUEST_TYPE
        or message.topic == settings.qr_mapping_capture_stop_request_topic
    ):
        handle_qr_capture_stop_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            stop_qr_capture=stop_qr_capture,
        )
        return

    if (
        request_type == QR_BUILD_MAP_REQUEST_TYPE
        or message.topic == settings.qr_mapping_build_map_request_topic
    ):
        handle_qr_build_map_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            build_qr_map=build_qr_map,
        )
        return

    if (
        request_type == QR_DELETE_MAP_REQUEST_TYPE
        or message.topic == settings.qr_mapping_delete_map_request_topic
    ):
        handle_qr_delete_map_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            delete_qr_map=delete_qr_map,
        )
        return

    if (
        request_type == QR_PCD_PREVIEW_REQUEST_TYPE
        or message.topic == settings.qr_mapping_pcd_preview_request_topic
    ):
        handle_qr_pcd_preview_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            read_qr_pcd_preview=read_qr_pcd_preview,
        )
        return

    if (
        request_type == QR_PROJECT_SNAPSHOT_REQUEST_TYPE
        or message.topic == settings.qr_mapping_project_snapshot_request_topic
    ):
        handle_qr_project_snapshot_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            collect_snapshot=get_qr_project_snapshot,
        )
        return

    if (
        request_type == QR_PROJECT_PATH_REQUEST_TYPE
        or message.topic == settings.qr_mapping_project_path_request_topic
    ):
        handle_qr_project_path_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            collect_snapshot=get_qr_project_path,
        )
        return

    if (
        request_type == CAMERA_CAPTURE_START_REQUEST_TYPE
        or message.topic == settings.robot_camera_capture_start_request_topic
    ):
        handle_camera_capture_start_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            start_camera_capture=start_camera_capture,
        )
        return

    if (
        request_type == CAMERA_CAPTURE_STOP_REQUEST_TYPE
        or message.topic == settings.robot_camera_capture_stop_request_topic
    ):
        handle_camera_capture_stop_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            stop_camera_capture=stop_camera_capture,
        )
        return

    if (
        request_type == CAMERA_CALIBRATION_REQUEST_TYPE
        or message.topic == settings.robot_camera_calibration_request_topic
    ):
        handle_camera_calibration_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            collect_snapshot=collect_camera_calibration,
        )
        return

    if (
        request_type == CAMERA_FRAME_REQUEST_TYPE
        or message.topic == settings.robot_camera_frame_request_topic
    ):
        handle_camera_frame_request(
            message,
            settings=settings,
            publish_response=publish_response,
            event_writer=event_writer,
            collect_snapshot=collect_camera_frame,
        )
        return

    # 默认：当前位姿查询
    handle_current_pose_request(
        message,
        settings=settings,
        publish_response=publish_response,
        event_writer=event_writer,
        collect_snapshot=collect_current_pose,
    )


def handle_camera_capture_start_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    start_camera_capture: CameraCaptureStartCollector | None = None,
) -> None:
    """处理相机连续采集开始请求。"""

    try:
        request = parse_camera_capture_start_request(
            message.payload,
            default_reply_topic=settings.robot_camera_capture_start_response_topic,
            default_frame_topic_template=settings.robot_camera_capture_frame_topic_template,
        )
    except Exception as error:
        response = error_response(
            response_type=CAMERA_CAPTURE_START_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.robot_camera_capture_start_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="robot_camera_capture_start_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    result: Mapping[str, object]
    if start_camera_capture is None:
        result = {
            "started": False,
            "errorCode": "CAMERA_CAPTURE_UNAVAILABLE",
            "errorMsg": "executor 未配置相机连续采集服务",
        }
    else:
        result = start_camera_capture(request.params)

    response = build_camera_capture_start_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="robot_camera_capture_start_response_published",
        message="camera capture start response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_capture_start_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    start_qr_capture: QrCaptureStartCollector | None = None,
) -> None:
    """处理二维码建图远端采集开始请求。"""

    try:
        request = parse_qr_capture_start_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_capture_start_response_topic,
            default_frame_topic_template=settings.qr_mapping_capture_frame_topic_template,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_CAPTURE_START_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_capture_start_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_capture_start_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    result: Mapping[str, object]
    if start_qr_capture is None:
        result = {
            "started": False,
            "errorCode": "QR_CAPTURE_UNAVAILABLE",
            "errorMsg": "executor 未配置二维码建图采集服务",
        }
    else:
        result = start_qr_capture(request.params)

    response = build_qr_capture_start_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_capture_start_response_published",
        message="QR capture start response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_capture_stop_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    stop_qr_capture: QrCaptureStopCollector | None = None,
) -> None:
    """处理二维码建图远端采集停止请求。"""

    try:
        request = parse_qr_capture_stop_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_capture_stop_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_CAPTURE_STOP_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_capture_stop_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_capture_stop_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    result: Mapping[str, object]
    if stop_qr_capture is None:
        result = {
            "stopped": False,
            "errorCode": "QR_CAPTURE_UNAVAILABLE",
            "errorMsg": "executor 未配置二维码建图采集服务",
        }
    else:
        result = stop_qr_capture(request.session_id)

    response = build_qr_capture_stop_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_capture_stop_response_published",
        message="QR capture stop response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_build_map_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    build_qr_map: QrBuildMapCollector | None = None,
) -> None:
    """处理二维码建图 SDK 执行请求。"""

    try:
        request = parse_qr_build_map_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_build_map_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_BUILD_MAP_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_build_map_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_build_map_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = build_qr_map or default_qr_build_map_collector(settings)
    result = collector(
        request.robot_serial,
        request.project_name,
        request.map_name,
        request.camera_id,
        request.marker_type,
        request.marker_size_meters,
    )
    response = build_qr_build_map_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_build_map_response_published",
        message="QR build map response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_delete_map_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    delete_qr_map: QrDeleteMapCollector | None = None,
) -> None:
    """处理二维码建图地图删除请求。"""

    try:
        request = parse_qr_delete_map_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_delete_map_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_DELETE_MAP_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_delete_map_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_delete_map_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = delete_qr_map or default_qr_delete_map_collector(settings)
    result = collector(request.robot_serial, request.project_name, request.map_name)
    response = build_qr_delete_map_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_delete_map_response_published",
        message="QR delete map response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_pcd_preview_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    read_qr_pcd_preview: QrPcdPreviewCollector | None = None,
) -> None:
    """处理二维码建图 PCD 远端抽样预览请求。"""

    try:
        request = parse_qr_pcd_preview_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_pcd_preview_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_PCD_PREVIEW_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_pcd_preview_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_pcd_preview_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = read_qr_pcd_preview or default_qr_pcd_preview_collector(settings)
    result = collector(
        request.robot_serial,
        request.project_name,
        request.map_name,
        request.max_points,
    )
    response = build_qr_pcd_preview_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_pcd_preview_response_published",
        message="QR PCD preview response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_point_recording_save_target_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    save_target: PointRecordingSaveTargetCollector | None = None,
) -> None:
    """处理点位录制保存目标点位请求。"""

    try:
        request = parse_point_recording_save_target_request(
            message.payload,
            default_reply_topic=settings.point_recording_save_target_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.point_recording_save_target_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="point_recording_save_target_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = save_target or default_point_recording_service(settings).save_target_point
    result = collector(request.params)
    response = build_point_recording_save_target_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="point_recording_save_target_response_published",
        message="point recording save target response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_point_recording_save_initial_photo_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    save_initial_photo: PointRecordingSaveInitialPhotoCollector | None = None,
) -> None:
    """处理点位录制保存初始拍照点位请求。"""

    try:
        request = parse_point_recording_save_initial_photo_request(
            message.payload,
            default_reply_topic=settings.point_recording_save_initial_photo_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.point_recording_save_initial_photo_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="point_recording_save_initial_photo_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = (
        save_initial_photo
        or default_point_recording_service(settings).save_initial_photo_point
    )
    result = collector(request.params)
    response = build_point_recording_save_initial_photo_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="point_recording_save_initial_photo_response_published",
        message="point recording save initial photo response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_point_recording_submit_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    submit_recording: PointRecordingSubmitCollector | None = None,
) -> None:
    """处理点位录制提交请求。"""

    try:
        request = parse_point_recording_submit_request(
            message.payload,
            default_reply_topic=settings.point_recording_submit_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=POINT_RECORDING_SUBMIT_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.point_recording_submit_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="point_recording_submit_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = submit_recording or default_point_recording_service(settings).submit_recording
    result = collector(request.params)
    response = build_point_recording_submit_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="point_recording_submit_response_published",
        message="point recording submit response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_project_path_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_snapshot: QrProjectPathCollector | None = None,
) -> None:
    """处理二维码建图项目远端路径查询。"""

    try:
        request = parse_qr_project_path_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_project_path_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_PROJECT_PATH_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_project_path_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_project_path_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = collect_snapshot or default_qr_project_path_collector(settings)
    snapshot = collector(request.robot_serial, request.project_name)
    response = build_qr_project_path_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        snapshot=snapshot,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_project_path_response_published",
        message="QR project path response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_qr_project_snapshot_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_snapshot: QrProjectSnapshotCollector | None = None,
) -> None:
    """处理二维码建图项目快照查询。"""

    try:
        request = parse_qr_project_snapshot_request(
            message.payload,
            default_reply_topic=settings.qr_mapping_project_snapshot_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=QR_PROJECT_SNAPSHOT_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.qr_mapping_project_snapshot_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="qr_project_snapshot_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    collector = collect_snapshot or default_qr_project_snapshot_collector(settings)
    snapshot = collector(request.robot_serial, request.project_name, request.image_limit)
    response = build_qr_project_snapshot_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        snapshot=snapshot,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="qr_project_snapshot_response_published",
        message="QR project snapshot response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_camera_capture_stop_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    stop_camera_capture: CameraCaptureStopCollector | None = None,
) -> None:
    """处理相机连续采集停止请求。"""

    try:
        request = parse_camera_capture_stop_request(
            message.payload,
            default_reply_topic=settings.robot_camera_capture_stop_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=CAMERA_CAPTURE_STOP_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.robot_camera_capture_stop_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="robot_camera_capture_stop_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    result: Mapping[str, object]
    if stop_camera_capture is None:
        result = {
            "stopped": False,
            "errorCode": "CAMERA_CAPTURE_UNAVAILABLE",
            "errorMsg": "executor 未配置相机连续采集服务",
        }
    else:
        result = stop_camera_capture(request.session_id)

    response = build_camera_capture_stop_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        result=result,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="robot_camera_capture_stop_response_published",
        message="camera capture stop response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_current_pose_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_snapshot: CurrentPoseCollector = run_gdk_current_pose_snapshot,
) -> None:
    """处理当前位姿查询。解析请求 → 采集快照 → 构建并发布响应。"""

    try:
        request = parse_current_pose_request(
            message.payload,
            default_reply_topic=settings.robot_current_pose_response_topic,
        )
    except Exception as error:
        response = error_response(
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.robot_current_pose_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="robot_current_pose_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    snapshot = collect_snapshot()
    response = build_current_pose_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        snapshot=snapshot,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="robot_current_pose_response_published",
        message="current pose response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_camera_frame_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_snapshot: CameraFrameCollector = run_gdk_camera_frame_snapshot,
) -> None:
    """处理相机单帧查询。解析请求 → 采集帧 → 构建并发布响应。"""

    try:
        request = parse_camera_frame_request(
            message.payload,
            default_reply_topic=settings.robot_camera_frame_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=CAMERA_FRAME_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.robot_camera_frame_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="robot_camera_frame_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    snapshot = collect_snapshot(request.camera_id, request.timeout_ms)
    response = build_camera_frame_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        snapshot=snapshot,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="robot_camera_frame_response_published",
        message="camera frame response published",
        topic=request.reply_topic,
        response=response,
    )


def handle_camera_calibration_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_snapshot: CameraCalibrationCollector = run_gdk_camera_calibration_snapshot,
) -> None:
    """处理相机标定查询。解析请求 → 读取内参/外参探针 → 构建并发布响应。"""

    try:
        request = parse_camera_calibration_request(
            message.payload,
            default_reply_topic=settings.robot_camera_calibration_response_topic,
        )
    except Exception as error:
        response = error_response(
            response_type=CAMERA_CALIBRATION_REQUEST_TYPE,
            request_id="",
            executor_aid=settings.executor_aid,
            code="INVALID_REQUEST",
            message=str(error),
        )
        publish_response(settings.robot_camera_calibration_response_topic, response)
        write_robot_state_event(
            event_writer,
            event_type="robot_camera_calibration_request_error",
            message=str(error),
            topic=message.topic,
            response=response,
        )
        return

    snapshot = collect_snapshot(
        request.camera_ids,
        request.timeout_ms,
        request.include_extrinsics,
    )
    response = build_camera_calibration_response(
        request_id=request.request_id,
        executor_aid=settings.executor_aid,
        snapshot=snapshot,
    )
    publish_response(request.reply_topic, response)
    write_robot_state_event(
        event_writer,
        event_type="robot_camera_calibration_response_published",
        message="camera calibration response published",
        topic=request.reply_topic,
        response=response,
    )


def default_qr_project_path_collector(settings: ExecutorSettings) -> QrProjectPathCollector:
    store = QrProjectStore(settings.gsa_data_root)

    def collect(robot_serial: str, project_name: str) -> Mapping[str, object]:
        try:
            return store.get_project_path(robot_serial=robot_serial, project_name=project_name)
        except Exception as error:
            return qr_project_error_payload(ACTION_GET_QR_PROJECT_PATH, error)

    return collect


def default_qr_project_snapshot_collector(settings: ExecutorSettings) -> QrProjectSnapshotCollector:
    store = QrProjectStore(settings.gsa_data_root)

    def collect(robot_serial: str, project_name: str, image_limit: int) -> Mapping[str, object]:
        try:
            return store.get_project_snapshot(
                robot_serial=robot_serial,
                project_name=project_name,
                image_limit=image_limit,
            )
        except Exception as error:
            return qr_project_error_payload(ACTION_GET_QR_PROJECT_SNAPSHOT, error)

    return collect


def default_qr_build_service(settings: ExecutorSettings) -> QrBuildService:
    return QrBuildService(
        project_store=QrProjectStore(settings.gsa_data_root),
        sdk_path=settings.qr_mapping_sdk_path,
        sdk_python=settings.qr_mapping_sdk_python,
        build_timeout_seconds=settings.qr_mapping_build_timeout_seconds,
    )


def default_point_recording_service(settings: ExecutorSettings) -> PointRecordingService:
    return PointRecordingService(
        project_store=QrProjectStore(settings.gsa_data_root),
        session_manager=GdkSessionManager(),
        localize_sdk_path=settings.qr_localize_sdk_path,
        localize_sdk_python=settings.qr_localize_sdk_python,
        localize_timeout_seconds=settings.qr_localize_timeout_seconds,
    )


def default_qr_build_map_collector(settings: ExecutorSettings) -> QrBuildMapCollector:
    service = default_qr_build_service(settings)

    def collect(
        robot_serial: str,
        project_name: str,
        map_name: str,
        camera_id: str,
        marker_type: str,
        marker_size_meters: float,
    ) -> Mapping[str, object]:
        return service.build_map(
            robot_serial=robot_serial,
            project_name=project_name,
            map_name=map_name,
            camera_id=camera_id,
            marker_type=marker_type,
            marker_size_meters=marker_size_meters,
        )

    return collect


def default_qr_delete_map_collector(settings: ExecutorSettings) -> QrDeleteMapCollector:
    service = default_qr_build_service(settings)

    def collect(robot_serial: str, project_name: str, map_name: str) -> Mapping[str, object]:
        return service.delete_map(
            robot_serial=robot_serial,
            project_name=project_name,
            map_name=map_name,
        )

    return collect


def default_qr_pcd_preview_collector(settings: ExecutorSettings) -> QrPcdPreviewCollector:
    service = default_qr_build_service(settings)

    def collect(
        robot_serial: str,
        project_name: str,
        map_name: str,
        max_points: int,
    ) -> Mapping[str, object]:
        return service.read_pcd_preview(
            robot_serial=robot_serial,
            project_name=project_name,
            map_name=map_name,
            max_points=max_points,
        )

    return collect


def qr_project_error_payload(action: str, error: Exception) -> dict[str, object]:
    return {
        "available": False,
        "backend": "executor.filesystem",
        "action": action,
        "errorCode": "QR_PROJECT_INVALID",
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }


def write_robot_state_event(
    event_writer: JsonlEventWriter | None,
    *,
    event_type: str,
    message: str,
    topic: str,
    response: Mapping[str, object],
) -> None:
    """写入机器人状态事件日志。event_writer 为 None 时跳过。"""

    if event_writer is None:
        return
    event_writer.write(
        RuntimeEvent(
            event_type=event_type,
            level="info" if response.get("ok") is True else "warning",
            message=message,
            topic=topic,
            payload={
                "request_id": response.get("requestId"),
                "ok": response.get("ok"),
            },
        )
    )
