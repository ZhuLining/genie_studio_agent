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
from gsa_taskflow_executor.gdk.robot_identity import (
    ACTION_GET_ROBOT_IDENTITY,
    DEFAULT_ROBOT_IDENTITY_TIMEOUT_MS,
)
from gsa_taskflow_executor.qr_mapping.build_service import (
    ACTION_BUILD_QR_MAP,
    ACTION_DELETE_QR_MAP,
    ACTION_READ_QR_PCD_PREVIEW,
    DEFAULT_PCD_MAX_POINTS,
    PCD_MAX_POINTS_LIMIT,
)
from gsa_taskflow_executor.qr_mapping.capture_service import (
    ACTION_START_QR_CAPTURE,
    ACTION_STOP_QR_CAPTURE,
    DEFAULT_QR_CAMERA_TIMEOUT_MS,
    DEFAULT_QR_CAPTURE_RATE_FPS,
    QrCaptureStartParams,
)
from gsa_taskflow_executor.qr_mapping.point_recording_service import (
    ACTION_SAVE_QR_INITIAL_PHOTO_POINT,
    ACTION_SAVE_QR_TARGET_POINT,
    ACTION_SUBMIT_POINT_RECORDING,
    DEFAULT_MIN_MARKERS,
    DEFAULT_POINT_RECORDING_TIMEOUT_MS,
    PointRecordingSaveInitialPhotoParams,
    PointRecordingSaveTargetParams,
    PointRecordingSubmitParams,
)
from gsa_taskflow_executor.qr_mapping.project_store import (
    ACTION_GET_QR_PROJECT_PATH,
    ACTION_GET_QR_PROJECT_SNAPSHOT,
    ACTION_LIST_QR_PROJECTS,
    DEFAULT_IMAGE_LIMIT,
    MAX_IMAGE_LIMIT,
)

# 请求类型常量
CURRENT_POSE_REQUEST_TYPE = "get_current_pose"
ROBOT_IDENTITY_REQUEST_TYPE = ACTION_GET_ROBOT_IDENTITY
CAMERA_FRAME_REQUEST_TYPE = "get_camera_frame"
CAMERA_CALIBRATION_REQUEST_TYPE = ACTION_GET_CAMERA_CALIBRATION
CAMERA_CAPTURE_START_REQUEST_TYPE = ACTION_START_CAMERA_CAPTURE
CAMERA_CAPTURE_STOP_REQUEST_TYPE = ACTION_STOP_CAMERA_CAPTURE
QR_PROJECT_PATH_REQUEST_TYPE = ACTION_GET_QR_PROJECT_PATH
QR_PROJECT_SNAPSHOT_REQUEST_TYPE = ACTION_GET_QR_PROJECT_SNAPSHOT
QR_PROJECT_LIST_REQUEST_TYPE = ACTION_LIST_QR_PROJECTS
QR_CAPTURE_START_REQUEST_TYPE = ACTION_START_QR_CAPTURE
QR_CAPTURE_STOP_REQUEST_TYPE = ACTION_STOP_QR_CAPTURE
QR_BUILD_MAP_REQUEST_TYPE = ACTION_BUILD_QR_MAP
QR_DELETE_MAP_REQUEST_TYPE = ACTION_DELETE_QR_MAP
QR_PCD_PREVIEW_REQUEST_TYPE = ACTION_READ_QR_PCD_PREVIEW
POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE = ACTION_SAVE_QR_TARGET_POINT
POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE = ACTION_SAVE_QR_INITIAL_PHOTO_POINT
POINT_RECORDING_SUBMIT_REQUEST_TYPE = ACTION_SUBMIT_POINT_RECORDING


@dataclass(frozen=True)
class CurrentPoseRequest:
    """当前位姿查询请求。"""

    request_id: str
    reply_topic: str


@dataclass(frozen=True)
class RobotIdentityRequest:
    """机器人身份查询请求。"""

    request_id: str
    reply_topic: str
    timeout_ms: int


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


@dataclass(frozen=True)
class QrProjectPathRequest:
    """二维码建图项目远端路径查询请求。"""

    request_id: str
    reply_topic: str
    robot_serial: str
    project_name: str


@dataclass(frozen=True)
class QrProjectSnapshotRequest:
    """二维码建图项目快照查询请求。"""

    request_id: str
    reply_topic: str
    robot_serial: str
    project_name: str
    image_limit: int


@dataclass(frozen=True)
class QrProjectListRequest:
    """二维码建图项目列表请求。"""

    request_id: str
    reply_topic: str
    robot_serial: str


@dataclass(frozen=True)
class QrCaptureStartRequest:
    """二维码建图远端采集开始请求。"""

    request_id: str
    reply_topic: str
    params: QrCaptureStartParams


@dataclass(frozen=True)
class QrCaptureStopRequest:
    """二维码建图远端采集停止请求。"""

    request_id: str
    reply_topic: str
    session_id: str


@dataclass(frozen=True)
class QrBuildMapRequest:
    """二维码建图 SDK 执行请求。"""

    request_id: str
    reply_topic: str
    robot_serial: str
    project_name: str
    map_name: str
    camera_id: str
    marker_type: str
    marker_size_meters: float


@dataclass(frozen=True)
class QrDeleteMapRequest:
    """二维码建图地图删除请求。"""

    request_id: str
    reply_topic: str
    robot_serial: str
    project_name: str
    map_name: str


@dataclass(frozen=True)
class QrPcdPreviewRequest:
    """二维码建图 PCD 远端抽样预览请求。"""

    request_id: str
    reply_topic: str
    robot_serial: str
    project_name: str
    map_name: str
    max_points: int


@dataclass(frozen=True)
class PointRecordingSaveTargetRequest:
    """点位录制保存目标点位请求。"""

    request_id: str
    reply_topic: str
    params: PointRecordingSaveTargetParams


@dataclass(frozen=True)
class PointRecordingSaveInitialPhotoRequest:
    """点位录制保存初始拍照点位请求。"""

    request_id: str
    reply_topic: str
    params: PointRecordingSaveInitialPhotoParams


@dataclass(frozen=True)
class PointRecordingSubmitRequest:
    """点位录制提交请求。"""

    request_id: str
    reply_topic: str
    params: PointRecordingSubmitParams


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


def parse_robot_identity_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> RobotIdentityRequest:
    """解析机器人身份查询请求 JSON。"""

    decoded = parse_json_object(payload, "机器人身份请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != ROBOT_IDENTITY_REQUEST_TYPE:
        raise ValueError(f"不支持的机器人状态请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("机器人身份请求缺少 requestId")

    timeout_ms = read_positive_int(
        decoded.get("timeoutMs"),
        DEFAULT_ROBOT_IDENTITY_TIMEOUT_MS,
    )
    return RobotIdentityRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        timeout_ms=timeout_ms,
    )


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


def parse_qr_project_path_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrProjectPathRequest:
    """解析二维码建图远端项目路径请求。"""

    decoded = parse_json_object(payload, "二维码建图项目路径请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_PROJECT_PATH_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图项目路径请求缺少 requestId")

    robot_serial = read_optional_string(decoded, "robotSerial") or read_optional_string(
        decoded,
        "robot_serial",
    )
    if not robot_serial:
        raise ValueError("二维码建图项目路径请求缺少 robotSerial")

    project_name = read_optional_string(decoded, "projectName") or read_optional_string(
        decoded,
        "project_name",
    )
    if not project_name:
        raise ValueError("二维码建图项目路径请求缺少 projectName")

    return QrProjectPathRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        robot_serial=robot_serial,
        project_name=project_name,
    )


def parse_qr_project_snapshot_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrProjectSnapshotRequest:
    """解析二维码建图项目快照请求。"""

    decoded = parse_json_object(payload, "二维码建图项目快照请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_PROJECT_SNAPSHOT_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图项目快照请求缺少 requestId")

    robot_serial = read_optional_string(decoded, "robotSerial") or read_optional_string(
        decoded,
        "robot_serial",
    )
    if not robot_serial:
        raise ValueError("二维码建图项目快照请求缺少 robotSerial")

    project_name = read_optional_string(decoded, "projectName") or read_optional_string(
        decoded,
        "project_name",
    )
    if not project_name:
        raise ValueError("二维码建图项目快照请求缺少 projectName")

    image_limit = read_bounded_positive_int(
        decoded.get("imageLimit"),
        fallback=DEFAULT_IMAGE_LIMIT,
        max_value=MAX_IMAGE_LIMIT,
    )
    return QrProjectSnapshotRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        robot_serial=robot_serial,
        project_name=project_name,
        image_limit=image_limit,
    )


def parse_qr_project_list_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrProjectListRequest:
    """解析二维码项目列表请求。"""

    decoded = parse_json_object(payload, "二维码建图项目列表请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_PROJECT_LIST_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图项目列表请求缺少 requestId")

    robot_serial = read_optional_string(decoded, "robotSerial") or read_optional_string(
        decoded,
        "robot_serial",
    )
    if not robot_serial:
        raise ValueError("二维码建图项目列表请求缺少 robotSerial")

    return QrProjectListRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        robot_serial=robot_serial,
    )


def parse_qr_capture_start_request(
    payload: str,
    *,
    default_reply_topic: str,
    default_frame_topic_template: str,
) -> QrCaptureStartRequest:
    """解析二维码建图远端采集开始请求。"""

    decoded = parse_json_object(payload, "二维码建图采集开始请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_CAPTURE_START_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图采集开始请求缺少 requestId")

    session_id = read_optional_string(decoded, "sessionId") or read_optional_string(
        decoded,
        "session_id",
    )
    if not session_id:
        raise ValueError("二维码建图采集开始请求缺少 sessionId")

    robot_serial, project_name = read_qr_project_identity(decoded, "二维码建图采集开始请求")
    frame_topic = (
        read_optional_string(decoded, "frameTopic")
        or read_optional_string(decoded, "frame_topic")
        or default_frame_topic_template.replace("{sessionId}", session_id)
    )
    camera_id = read_optional_string(decoded, "cameraId") or read_optional_string(
        decoded,
        "camera_id",
    ) or "hand_left_color"
    marker_type = (
        read_optional_string(decoded, "markerType")
        or read_optional_string(decoded, "qrType")
        or read_optional_string(decoded, "dictName")
        or "ARUCO_MIP_36h12"
    )
    marker_size_meters = read_positive_float(
        decoded.get("markerSizeMeters") or decoded.get("marker_len_m"),
        0.04,
    )
    capture_rate_fps = read_positive_int(
        decoded.get("captureRateFps") or decoded.get("fps"),
        DEFAULT_QR_CAPTURE_RATE_FPS,
    )
    timeout_ms = read_positive_int(decoded.get("timeoutMs"), DEFAULT_QR_CAMERA_TIMEOUT_MS)
    return QrCaptureStartRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        params=QrCaptureStartParams(
            session_id=session_id,
            frame_topic=frame_topic,
            robot_serial=robot_serial,
            project_name=project_name,
            camera_id=camera_id,
            marker_type=marker_type,
            marker_size_meters=marker_size_meters,
            capture_rate_fps=capture_rate_fps,
            timeout_ms=timeout_ms,
            reset_existing=read_bool(decoded.get("resetExisting"), False),
        ),
    )


def parse_qr_capture_stop_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrCaptureStopRequest:
    """解析二维码建图远端采集停止请求。"""

    decoded = parse_json_object(payload, "二维码建图采集停止请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_CAPTURE_STOP_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图采集停止请求缺少 requestId")

    session_id = read_optional_string(decoded, "sessionId") or read_optional_string(
        decoded,
        "session_id",
    )
    if not session_id:
        raise ValueError("二维码建图采集停止请求缺少 sessionId")

    return QrCaptureStopRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        session_id=session_id,
    )


def parse_qr_build_map_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrBuildMapRequest:
    """解析二维码建图 SDK 执行请求。"""

    decoded = parse_json_object(payload, "二维码建图 SDK 请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_BUILD_MAP_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")

    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图 SDK 请求缺少 requestId")

    robot_serial, project_name = read_qr_project_identity(decoded, "二维码建图 SDK 请求")
    map_name = read_optional_string(decoded, "mapName") or read_optional_string(
        decoded,
        "map_name",
    )
    if not map_name:
        raise ValueError("二维码建图 SDK 请求缺少 mapName")
    camera_id = read_optional_string(decoded, "cameraId") or read_optional_string(
        decoded,
        "camera_id",
    ) or "hand_left_color"
    marker_type = (
        read_optional_string(decoded, "markerType")
        or read_optional_string(decoded, "qrType")
        or "ARUCO_MIP_36h12"
    )
    marker_size_meters = read_positive_float(
        decoded.get("markerSizeMeters") or decoded.get("marker_len_m"),
        0.04,
    )
    return QrBuildMapRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        robot_serial=robot_serial,
        project_name=project_name,
        map_name=map_name,
        camera_id=camera_id,
        marker_type=marker_type,
        marker_size_meters=marker_size_meters,
    )


def parse_qr_delete_map_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrDeleteMapRequest:
    """解析二维码建图地图删除请求。"""

    decoded = parse_json_object(payload, "二维码建图删除请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_DELETE_MAP_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")
    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图删除请求缺少 requestId")
    robot_serial, project_name = read_qr_project_identity(decoded, "二维码建图删除请求")
    map_name = read_optional_string(decoded, "mapName") or read_optional_string(
        decoded,
        "map_name",
    )
    if not map_name:
        raise ValueError("二维码建图删除请求缺少 mapName")
    return QrDeleteMapRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        robot_serial=robot_serial,
        project_name=project_name,
        map_name=map_name,
    )


def parse_qr_pcd_preview_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> QrPcdPreviewRequest:
    """解析二维码建图 PCD 远端预览请求。"""

    decoded = parse_json_object(payload, "二维码建图 PCD 预览请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != QR_PCD_PREVIEW_REQUEST_TYPE:
        raise ValueError(f"不支持的二维码建图请求类型: {request_type}")
    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("二维码建图 PCD 预览请求缺少 requestId")
    robot_serial, project_name = read_qr_project_identity(decoded, "二维码建图 PCD 预览请求")
    map_name = read_optional_string(decoded, "mapName") or read_optional_string(
        decoded,
        "map_name",
    )
    if not map_name:
        raise ValueError("二维码建图 PCD 预览请求缺少 mapName")
    max_points = read_bounded_positive_int(
        decoded.get("maxPoints"),
        fallback=DEFAULT_PCD_MAX_POINTS,
        max_value=PCD_MAX_POINTS_LIMIT,
    )
    return QrPcdPreviewRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        robot_serial=robot_serial,
        project_name=project_name,
        map_name=map_name,
        max_points=max_points,
    )


def parse_point_recording_save_target_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> PointRecordingSaveTargetRequest:
    """解析点位录制保存目标点位请求。"""

    decoded = parse_json_object(payload, "点位录制目标点位请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE:
        raise ValueError(f"不支持的点位录制请求类型: {request_type}")
    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("点位录制目标点位请求缺少 requestId")
    robot_serial, project_name = read_qr_project_identity(decoded, "点位录制目标点位请求")
    point_name = read_point_recording_point_name(decoded, "点位录制目标点位请求")
    arm = read_point_recording_arm(decoded)
    camera_id = read_point_recording_camera_id(decoded, arm)
    return PointRecordingSaveTargetRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        params=PointRecordingSaveTargetParams(
            robot_serial=robot_serial,
            project_name=project_name,
            point_name=point_name,
            arm=arm,
            camera_id=camera_id,
            map_name=read_optional_string(decoded, "mapName")
            or read_optional_string(decoded, "map_name"),
            timeout_ms=read_positive_int(
                decoded.get("timeoutMs"),
                DEFAULT_POINT_RECORDING_TIMEOUT_MS,
            ),
        ),
    )


def parse_point_recording_save_initial_photo_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> PointRecordingSaveInitialPhotoRequest:
    """解析点位录制保存初始拍照点位请求。"""

    decoded = parse_json_object(payload, "点位录制初始拍照点位请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE:
        raise ValueError(f"不支持的点位录制请求类型: {request_type}")
    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("点位录制初始拍照点位请求缺少 requestId")
    robot_serial, project_name = read_qr_project_identity(decoded, "点位录制初始拍照点位请求")
    point_name = read_point_recording_point_name(decoded, "点位录制初始拍照点位请求")
    arm = read_point_recording_arm(decoded)
    camera_id = read_point_recording_camera_id(decoded, arm)
    return PointRecordingSaveInitialPhotoRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        params=PointRecordingSaveInitialPhotoParams(
            robot_serial=robot_serial,
            project_name=project_name,
            point_name=point_name,
            arm=arm,
            camera_id=camera_id,
            map_name=read_optional_string(decoded, "mapName")
            or read_optional_string(decoded, "map_name"),
            timeout_ms=read_positive_int(
                decoded.get("timeoutMs"),
                DEFAULT_POINT_RECORDING_TIMEOUT_MS,
            ),
            min_markers=read_positive_int(decoded.get("minMarkers"), DEFAULT_MIN_MARKERS),
        ),
    )


def parse_point_recording_submit_request(
    payload: str,
    *,
    default_reply_topic: str,
) -> PointRecordingSubmitRequest:
    """解析点位录制提交请求。"""

    decoded = parse_json_object(payload, "点位录制提交请求")
    request_type = read_optional_string(decoded, "type")
    if request_type is not None and request_type != POINT_RECORDING_SUBMIT_REQUEST_TYPE:
        raise ValueError(f"不支持的点位录制请求类型: {request_type}")
    request_id = read_request_id_from_object(decoded)
    if not request_id:
        raise ValueError("点位录制提交请求缺少 requestId")
    robot_serial, project_name = read_qr_project_identity(decoded, "点位录制提交请求")
    return PointRecordingSubmitRequest(
        request_id=request_id,
        reply_topic=read_reply_topic(decoded, default_reply_topic),
        params=PointRecordingSubmitParams(
            robot_serial=robot_serial,
            project_name=project_name,
        ),
    )


def read_qr_project_identity(
    value: Mapping[str, Any],
    label: str,
) -> tuple[str, str]:
    robot_serial = read_optional_string(value, "robotSerial") or read_optional_string(
        value,
        "robot_serial",
    )
    if not robot_serial:
        raise ValueError(f"{label}缺少 robotSerial")

    project_name = read_optional_string(value, "projectName") or read_optional_string(
        value,
        "project_name",
    )
    if not project_name:
        raise ValueError(f"{label}缺少 projectName")
    return robot_serial, project_name


def read_point_recording_point_name(value: Mapping[str, Any], label: str) -> str:
    point_name = read_optional_string(value, "pointName") or read_optional_string(
        value,
        "point_name",
    )
    if not point_name:
        raise ValueError(f"{label}缺少 pointName")
    return point_name


def read_point_recording_arm(value: Mapping[str, Any]) -> str:
    arm = read_optional_string(value, "arm") or read_optional_string(value, "targetArm")
    if arm in {"left", "left_arm"}:
        return "left_arm"
    if arm in {"right", "right_arm"}:
        return "right_arm"
    raise ValueError("点位录制执行手臂必须是 left_arm 或 right_arm")


def read_point_recording_camera_id(value: Mapping[str, Any], arm: str) -> str:
    camera_id = read_optional_string(value, "cameraId") or read_optional_string(
        value,
        "camera_id",
    )
    if camera_id:
        return camera_id
    return "hand_right_color" if arm == "right_arm" else "hand_left_color"


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


def read_positive_float(value: Any, fallback: float) -> float:
    """读取正浮点数。非法输入返回 fallback。"""

    if isinstance(value, bool):
        return fallback
    if isinstance(value, (int, float)) and float(value) > 0:
        return float(value)
    return fallback


def read_bounded_positive_int(value: Any, *, fallback: int, max_value: int) -> int:
    """读取带上限的正整数，避免一次快照返回过多图片元数据。"""

    return min(read_positive_int(value, fallback), max_value)
