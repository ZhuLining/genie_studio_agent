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
from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.mqtt.robot_state_models import (
    CAMERA_CALIBRATION_REQUEST_TYPE,
    CAMERA_CAPTURE_START_REQUEST_TYPE,
    CAMERA_CAPTURE_STOP_REQUEST_TYPE,
    CAMERA_FRAME_REQUEST_TYPE,
    CURRENT_POSE_REQUEST_TYPE,
    CameraCalibrationRequest,
    CameraCaptureStartRequest,
    CameraCaptureStopRequest,
    CameraFrameRequest,
    CurrentPoseRequest,
    parse_camera_calibration_request,
    parse_camera_capture_start_request,
    parse_camera_capture_stop_request,
    parse_camera_frame_request,
    parse_current_pose_request,
    parse_json_object,
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
    error_response,
    read_error_code,
    read_error_message,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent

__all__ = [
    "CAMERA_BUSY_ERROR_MESSAGE",
    "CAMERA_CALIBRATION_BUSY_ERROR_MESSAGE",
    "CAMERA_CALIBRATION_REQUEST_TYPE",
    "CAMERA_CAPTURE_START_REQUEST_TYPE",
    "CAMERA_CAPTURE_STOP_REQUEST_TYPE",
    "CAMERA_FRAME_REQUEST_TYPE",
    "CURRENT_POSE_REQUEST_TYPE",
    "ROBOT_BUSY_ERROR_CODE",
    "ROBOT_BUSY_ERROR_MESSAGE",
    "CameraCalibrationRequest",
    "CameraCaptureStartRequest",
    "CameraCaptureStopRequest",
    "CameraFrameRequest",
    "CurrentPoseRequest",
    "build_camera_calibration_response",
    "build_camera_capture_start_response",
    "build_camera_capture_stop_response",
    "build_camera_frame_response",
    "build_current_pose_response",
    "error_response",
    "handle_camera_calibration_request",
    "handle_camera_capture_start_request",
    "handle_camera_capture_stop_request",
    "handle_camera_frame_request",
    "handle_current_pose_request",
    "handle_robot_state_request",
    "parse_camera_calibration_request",
    "parse_camera_capture_start_request",
    "parse_camera_capture_stop_request",
    "parse_camera_frame_request",
    "parse_current_pose_request",
    "parse_json_object",
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
) -> None:
    """按请求类型分发到对应 handler。通过 payload type 字段和 MQTT topic 双重匹配。"""

    request_type = read_request_type(message.payload)
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
