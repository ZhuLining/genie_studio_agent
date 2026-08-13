"""robot_state MQTT 请求模型与 JSON 解析。

robot_state 是桌面端发起的只读/采集请求协议层；这里不触碰 GDK，只把外部
payload 收敛成明确的数据模型，handler 再决定调用哪个 collector。
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from gsa_taskflow_executor.gdk.camera_calibration import (
    ACTION_GET_CAMERA_CALIBRATION,
    DEFAULT_CALIBRATION_CAMERA_IDS,
)
from gsa_taskflow_executor.gdk.camera_capture import (
    ACTION_START_CAMERA_CAPTURE,
    ACTION_STOP_CAMERA_CAPTURE,
    DEFAULT_CAPTURE_RATE_FPS,
    CameraCaptureStartParams,
)
from gsa_taskflow_executor.gdk.camera_frame import DEFAULT_CAMERA_TIMEOUT_MS

# 请求类型常量
CURRENT_POSE_REQUEST_TYPE = "get_current_pose"
CAMERA_FRAME_REQUEST_TYPE = "get_camera_frame"
CAMERA_CALIBRATION_REQUEST_TYPE = ACTION_GET_CAMERA_CALIBRATION
CAMERA_CAPTURE_START_REQUEST_TYPE = ACTION_START_CAMERA_CAPTURE
CAMERA_CAPTURE_STOP_REQUEST_TYPE = ACTION_STOP_CAMERA_CAPTURE


@dataclass(frozen=True)
class CurrentPoseRequest:
    """当前位姿查询请求。"""

    request_id: str
    reply_topic: str


@dataclass(frozen=True)
class CameraFrameRequest:
    """相机单帧查询请求。"""

    request_id: str
    reply_topic: str
    camera_id: str
    timeout_ms: int


@dataclass(frozen=True)
class CameraCalibrationRequest:
    """相机标定查询请求。"""

    request_id: str
    reply_topic: str
    camera_ids: tuple[str, ...]
    include_extrinsics: bool
    timeout_ms: int


@dataclass(frozen=True)
class CameraCaptureStartRequest:
    """相机连续采集开始请求。"""

    request_id: str
    reply_topic: str
    params: CameraCaptureStartParams


@dataclass(frozen=True)
class CameraCaptureStopRequest:
    """相机连续采集停止请求。"""

    request_id: str
    reply_topic: str
    session_id: str


def parse_current_pose_request(payload: str, *, default_reply_topic: str) -> CurrentPoseRequest:
    """解析当前位姿查询请求 JSON。"""

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
    """解析相机单帧查询请求 JSON。"""

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
        camera_id = "hand_left_color"  # 默认左手彩色相机

    timeout_ms = read_positive_int(decoded.get("timeoutMs"), DEFAULT_CAMERA_TIMEOUT_MS)
    return CameraFrameRequest(
        request_id=request_id,
        reply_topic=reply_topic,
        camera_id=camera_id,
        timeout_ms=timeout_ms,
    )


def parse_camera_calibration_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> CameraCalibrationRequest:
    """解析相机标定查询请求 JSON。"""

    decoded = parse_json_object(payload, "相机标定请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != CAMERA_CALIBRATION_REQUEST_TYPE:
        raise ValueError(f"不支持的机器人状态请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("相机标定请求缺少 requestId")

    reply_topic = read_reply_topic(decoded, default_reply_topic)
    camera_ids = read_camera_ids(decoded.get("cameraIds") or decoded.get("camera_ids"))
    include_extrinsics = read_bool(decoded.get("includeExtrinsics"), False)
    timeout_ms = read_positive_int(decoded.get("timeoutMs"), DEFAULT_CAMERA_TIMEOUT_MS)
    return CameraCalibrationRequest(
        request_id=request_id,
        reply_topic=reply_topic,
        camera_ids=camera_ids,
        include_extrinsics=include_extrinsics,
        timeout_ms=timeout_ms,
    )


def parse_camera_capture_start_request(
    payload: str,
    *,
    default_reply_topic: str,
    default_frame_topic_template: str,
) -> CameraCaptureStartRequest:
    """解析相机连续采集开始请求 JSON。"""

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
    """解析相机连续采集停止请求 JSON。"""

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


def read_optional_string(value: Mapping[str, Any], key: str) -> str | None:
    """从 Mapping 读取可选字符串。空字符串返回 None。"""

    raw = value.get(key)
    if not isinstance(raw, str):
        return None
    stripped = raw.strip()
    return stripped or None


def parse_json_object(payload: str, label: str) -> Mapping[str, Any]:
    """解析 JSON 字符串为 Mapping。失败抛 ValueError（含中文标签）。"""

    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError(f"{label}不是有效 JSON: {error.msg}") from error
    if not isinstance(decoded, Mapping):
        raise ValueError(f"{label}必须是 JSON object")
    return decoded


def read_request_id_from_object(value: Mapping[str, Any]) -> str | None:
    """读取 requestId 或 request_id 字段。"""

    return read_optional_string(value, "requestId") or read_optional_string(value, "request_id")


def read_reply_topic(value: Mapping[str, Any], default_reply_topic: str) -> str:
    """读取 replyTopic/reply_topic，无则用默认值。"""

    reply_topic = (
        read_optional_string(value, "replyTopic")
        or read_optional_string(value, "reply_topic")
        or default_reply_topic
    )
    if not reply_topic:
        raise ValueError("请求缺少 replyTopic")
    return reply_topic


def read_request_type(payload: str) -> str | None:
    """从 JSON payload 提取 type 字段。"""

    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(decoded, Mapping):
        return None
    return read_optional_string(decoded, "type")


def read_bool(value: Any, fallback: bool) -> bool:
    """读取布尔值。支持 bool 和常见字符串，其他返回 fallback。"""

    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    return fallback


def read_camera_ids(value: Any) -> tuple[str, ...]:
    """读取 cameraIds。为空时默认左手彩色相机，去重并保留顺序。"""

    if value is None:
        return DEFAULT_CALIBRATION_CAMERA_IDS
    if isinstance(value, str):
        candidates: list[Any] = [value]
    elif isinstance(value, list):
        candidates = value
    else:
        raise ValueError("相机标定请求 cameraIds 必须是字符串数组")

    camera_ids: list[str] = []
    for item in candidates:
        if not isinstance(item, str):
            raise ValueError("相机标定请求 cameraIds 只能包含字符串")
        camera_id = item.strip()
        if camera_id and camera_id not in camera_ids:
            camera_ids.append(camera_id)
    return tuple(camera_ids) or DEFAULT_CALIBRATION_CAMERA_IDS


def read_positive_int(value: Any, fallback: int) -> int:
    """读取正整数。bool/非正数/非 int 返回 fallback。"""

    if isinstance(value, bool):
        return fallback
    if isinstance(value, int) and value > 0:
        return value
    return fallback
