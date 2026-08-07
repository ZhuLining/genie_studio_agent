from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from gsa_taskflow_executor.gdk.camera_capture import (
    ACTION_START_CAMERA_CAPTURE,
    ACTION_STOP_CAMERA_CAPTURE,
    CAMERA_CAPTURE_BUSY_ERROR_MESSAGE,
    DEFAULT_CAPTURE_RATE_FPS,
    CameraCaptureStartParams,
)
from gsa_taskflow_executor.gdk.camera_frame import (
    DEFAULT_CAMERA_TIMEOUT_MS,
    run_gdk_camera_frame_snapshot,
)
from gsa_taskflow_executor.gdk.current_pose import run_gdk_current_pose_snapshot
from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent

CURRENT_POSE_REQUEST_TYPE = "get_current_pose"
CAMERA_FRAME_REQUEST_TYPE = "get_camera_frame"
CAMERA_CAPTURE_START_REQUEST_TYPE = ACTION_START_CAMERA_CAPTURE
CAMERA_CAPTURE_STOP_REQUEST_TYPE = ACTION_STOP_CAMERA_CAPTURE
ROBOT_BUSY_ERROR_CODE = "ROBOT_BUSY"
ROBOT_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，当前位姿读取已拒绝"
CAMERA_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，相机图像读取已拒绝"

RobotStatePublisher = Callable[[str, Mapping[str, Any]], None]
CurrentPoseCollector = Callable[[], Mapping[str, object]]
CameraFrameCollector = Callable[[str, int], Mapping[str, object]]
CameraCaptureStartCollector = Callable[[CameraCaptureStartParams], Mapping[str, object]]
CameraCaptureStopCollector = Callable[[str], Mapping[str, object]]


@dataclass(frozen=True)
class CurrentPoseRequest:
    request_id: str
    reply_topic: str


@dataclass(frozen=True)
class CameraFrameRequest:
    request_id: str
    reply_topic: str
    camera_id: str
    timeout_ms: int


@dataclass(frozen=True)
class CameraCaptureStartRequest:
    request_id: str
    reply_topic: str
    params: CameraCaptureStartParams


@dataclass(frozen=True)
class CameraCaptureStopRequest:
    request_id: str
    reply_topic: str
    session_id: str


def handle_robot_state_request(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    event_writer: JsonlEventWriter | None = None,
    collect_current_pose: CurrentPoseCollector = run_gdk_current_pose_snapshot,
    collect_camera_frame: CameraFrameCollector = run_gdk_camera_frame_snapshot,
    start_camera_capture: CameraCaptureStartCollector | None = None,
    stop_camera_capture: CameraCaptureStopCollector | None = None,
) -> None:
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
    """Handle a desktop current-pose query outside the taskflow scheduler."""

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
    """Handle a desktop camera frame query outside the taskflow scheduler."""

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


def parse_current_pose_request(payload: str, *, default_reply_topic: str) -> CurrentPoseRequest:
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError(f"当前位姿请求不是有效 JSON: {error.msg}") from error

    if not isinstance(decoded, Mapping):
        raise ValueError("当前位姿请求必须是 JSON object")

    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != CURRENT_POSE_REQUEST_TYPE:
        raise ValueError(f"不支持的机器人状态请求类型: {request_type}")

    request_id = read_optional_string(decoded, "requestId") or read_optional_string(
        decoded,
        "request_id",
    )
    if not request_id:
        raise ValueError("当前位姿请求缺少 requestId")

    reply_topic = (
        read_optional_string(decoded, "replyTopic")
        or read_optional_string(decoded, "reply_topic")
        or default_reply_topic
    )
    if not reply_topic:
        raise ValueError("当前位姿请求缺少 replyTopic")

    return CurrentPoseRequest(request_id=request_id, reply_topic=reply_topic)


def parse_camera_frame_request(payload: str, *, default_reply_topic: str) -> CameraFrameRequest:
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError(f"相机图像请求不是有效 JSON: {error.msg}") from error

    if not isinstance(decoded, Mapping):
        raise ValueError("相机图像请求必须是 JSON object")

    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != CAMERA_FRAME_REQUEST_TYPE:
        raise ValueError(f"不支持的机器人状态请求类型: {request_type}")

    request_id = read_optional_string(decoded, "requestId") or read_optional_string(
        decoded,
        "request_id",
    )
    if not request_id:
        raise ValueError("相机图像请求缺少 requestId")

    reply_topic = (
        read_optional_string(decoded, "replyTopic")
        or read_optional_string(decoded, "reply_topic")
        or default_reply_topic
    )
    if not reply_topic:
        raise ValueError("相机图像请求缺少 replyTopic")

    camera_id = read_optional_string(decoded, "cameraId") or read_optional_string(
        decoded,
        "camera_id",
    )
    if not camera_id:
        camera_id = "hand_left_color"

    timeout_ms = read_positive_int(decoded.get("timeoutMs"), DEFAULT_CAMERA_TIMEOUT_MS)
    return CameraFrameRequest(
        request_id=request_id,
        reply_topic=reply_topic,
        camera_id=camera_id,
        timeout_ms=timeout_ms,
    )


def parse_camera_capture_start_request(
    payload: str,
    *,
    default_reply_topic: str,
    default_frame_topic_template: str,
) -> CameraCaptureStartRequest:
    decoded = parse_json_object(payload, "相机连续采集开始请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != CAMERA_CAPTURE_START_REQUEST_TYPE:
        raise ValueError(f"不支持的机器人状态请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("相机连续采集开始请求缺少 requestId")

    session_id = read_optional_string(decoded, "sessionId") or read_optional_string(
        decoded,
        "session_id",
    )
    if not session_id:
        raise ValueError("相机连续采集开始请求缺少 sessionId")

    reply_topic = read_reply_topic(decoded, default_reply_topic)
    frame_topic = (
        read_optional_string(decoded, "frameTopic")
        or read_optional_string(decoded, "frame_topic")
        or default_frame_topic_template.replace("{sessionId}", session_id)
    )
    camera_id = read_optional_string(decoded, "cameraId") or read_optional_string(
        decoded,
        "camera_id",
    )
    if not camera_id:
        camera_id = "hand_left_color"

    capture_rate_fps = read_positive_int(decoded.get("captureRateFps"), DEFAULT_CAPTURE_RATE_FPS)
    timeout_ms = read_positive_int(decoded.get("timeoutMs"), DEFAULT_CAMERA_TIMEOUT_MS)
    return CameraCaptureStartRequest(
        request_id=request_id,
        reply_topic=reply_topic,
        params=CameraCaptureStartParams(
            session_id=session_id,
            frame_topic=frame_topic,
            camera_id=camera_id,
            capture_rate_fps=capture_rate_fps,
            timeout_ms=timeout_ms,
        ),
    )


def parse_camera_capture_stop_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> CameraCaptureStopRequest:
    decoded = parse_json_object(payload, "相机连续采集停止请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != CAMERA_CAPTURE_STOP_REQUEST_TYPE:
        raise ValueError(f"不支持的机器人状态请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("相机连续采集停止请求缺少 requestId")

    session_id = read_optional_string(decoded, "sessionId") or read_optional_string(
        decoded,
        "session_id",
    )
    if not session_id:
        raise ValueError("相机连续采集停止请求缺少 sessionId")

    return CameraCaptureStopRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        session_id=session_id,
    )


def build_current_pose_response(
    *,
    request_id: str,
    executor_aid: str,
    snapshot: Mapping[str, object],
) -> dict[str, object]:
    if snapshot.get("available") is True:
        return {
            "type": CURRENT_POSE_REQUEST_TYPE,
            "requestId": request_id,
            "ok": True,
            "executorAid": executor_aid,
            "data": dict(snapshot),
        }

    if snapshot.get("busy") is True:
        return error_response(
            response_type=CURRENT_POSE_REQUEST_TYPE,
            request_id=request_id,
            executor_aid=executor_aid,
            code=ROBOT_BUSY_ERROR_CODE,
            message=ROBOT_BUSY_ERROR_MESSAGE,
            details=dict(snapshot),
        )

    return error_response(
        response_type=CURRENT_POSE_REQUEST_TYPE,
        request_id=request_id,
        executor_aid=executor_aid,
        code="GDK_UNAVAILABLE",
        message=read_error_message(snapshot),
        details=dict(snapshot),
    )


def build_camera_frame_response(
    *,
    request_id: str,
    executor_aid: str,
    snapshot: Mapping[str, object],
) -> dict[str, object]:
    if snapshot.get("available") is True:
        return {
            "type": CAMERA_FRAME_REQUEST_TYPE,
            "requestId": request_id,
            "ok": True,
            "executorAid": executor_aid,
            "data": dict(snapshot),
        }

    if snapshot.get("busy") is True:
        return error_response(
            response_type=CAMERA_FRAME_REQUEST_TYPE,
            request_id=request_id,
            executor_aid=executor_aid,
            code=ROBOT_BUSY_ERROR_CODE,
            message=CAMERA_BUSY_ERROR_MESSAGE,
            details=dict(snapshot),
        )

    return error_response(
        response_type=CAMERA_FRAME_REQUEST_TYPE,
        request_id=request_id,
        executor_aid=executor_aid,
        code=read_error_code(snapshot, "GDK_CAMERA_UNAVAILABLE"),
        message=read_error_message(snapshot, fallback="GDK 相机图像读取失败"),
        details=dict(snapshot),
    )


def build_camera_capture_start_response(
    *,
    request_id: str,
    executor_aid: str,
    result: Mapping[str, object],
) -> dict[str, object]:
    if result.get("started") is True:
        return {
            "type": CAMERA_CAPTURE_START_REQUEST_TYPE,
            "requestId": request_id,
            "ok": True,
            "executorAid": executor_aid,
            "data": dict(result),
        }

    if result.get("busy") is True:
        return error_response(
            response_type=CAMERA_CAPTURE_START_REQUEST_TYPE,
            request_id=request_id,
            executor_aid=executor_aid,
            code=ROBOT_BUSY_ERROR_CODE,
            message=CAMERA_CAPTURE_BUSY_ERROR_MESSAGE,
            details=dict(result),
        )

    return error_response(
        response_type=CAMERA_CAPTURE_START_REQUEST_TYPE,
        request_id=request_id,
        executor_aid=executor_aid,
        code=read_error_code(result, "CAMERA_CAPTURE_START_FAILED"),
        message=read_error_message(result, fallback="相机连续采集启动失败"),
        details=dict(result),
    )


def build_camera_capture_stop_response(
    *,
    request_id: str,
    executor_aid: str,
    result: Mapping[str, object],
) -> dict[str, object]:
    if result.get("stopped") is True:
        return {
            "type": CAMERA_CAPTURE_STOP_REQUEST_TYPE,
            "requestId": request_id,
            "ok": True,
            "executorAid": executor_aid,
            "data": dict(result),
        }

    return error_response(
        response_type=CAMERA_CAPTURE_STOP_REQUEST_TYPE,
        request_id=request_id,
        executor_aid=executor_aid,
        code=read_error_code(result, "CAMERA_CAPTURE_STOP_FAILED"),
        message=read_error_message(result, fallback="相机连续采集停止失败"),
        details=dict(result),
    )


def error_response(
    *,
    response_type: str = CURRENT_POSE_REQUEST_TYPE,
    request_id: str,
    executor_aid: str,
    code: str,
    message: str,
    details: Mapping[str, object] | None = None,
) -> dict[str, object]:
    error: dict[str, object] = {
        "code": code,
        "message": message,
    }
    if details is not None:
        error["details"] = dict(details)
    return {
        "type": response_type,
        "requestId": request_id,
        "ok": False,
        "executorAid": executor_aid,
        "error": error,
    }


def read_optional_string(value: Mapping[str, Any], key: str) -> str | None:
    raw = value.get(key)
    if not isinstance(raw, str):
        return None
    stripped = raw.strip()
    return stripped or None


def parse_json_object(payload: str, label: str) -> Mapping[str, Any]:
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError(f"{label}不是有效 JSON: {error.msg}") from error
    if not isinstance(decoded, Mapping):
        raise ValueError(f"{label}必须是 JSON object")
    return decoded


def read_request_id_from_object(value: Mapping[str, Any]) -> str | None:
    return read_optional_string(value, "requestId") or read_optional_string(value, "request_id")


def read_reply_topic(value: Mapping[str, Any], default_reply_topic: str) -> str:
    reply_topic = (
        read_optional_string(value, "replyTopic")
        or read_optional_string(value, "reply_topic")
        or default_reply_topic
    )
    if not reply_topic:
        raise ValueError("请求缺少 replyTopic")
    return reply_topic


def read_request_type(payload: str) -> str | None:
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(decoded, Mapping):
        return None
    return read_optional_string(decoded, "type")


def read_positive_int(value: Any, fallback: int) -> int:
    if isinstance(value, bool):
        return fallback
    if isinstance(value, int) and value > 0:
        return value
    return fallback


def read_error_code(snapshot: Mapping[str, object], fallback: str) -> str:
    error_code = snapshot.get("errorCode") or snapshot.get("error_code")
    if isinstance(error_code, str) and error_code.strip():
        return error_code.strip()
    return fallback


def read_error_message(
    snapshot: Mapping[str, object],
    fallback: str = "GDK 当前位姿读取失败",
) -> str:
    error_msg = snapshot.get("errorMsg") or snapshot.get("error_msg")
    if isinstance(error_msg, str) and error_msg.strip():
        return error_msg.strip()
    error_stage = snapshot.get("errorStage") or snapshot.get("error_stage")
    if isinstance(error_stage, str) and error_stage.strip():
        return f"{fallback}: {error_stage}"
    return fallback


def write_robot_state_event(
    event_writer: JsonlEventWriter | None,
    *,
    event_type: str,
    message: str,
    topic: str,
    response: Mapping[str, object],
) -> None:
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
