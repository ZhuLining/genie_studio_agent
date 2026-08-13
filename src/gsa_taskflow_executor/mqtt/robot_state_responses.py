"""robot_state MQTT 响应 builder。

这里集中处理 snapshot/result 到 MQTT 响应 payload 的映射，避免 handler 里混入
大量 error_code/error_message 协议细节。
"""

from __future__ import annotations

from collections.abc import Mapping

from gsa_taskflow_executor.gdk.camera_capture import CAMERA_CAPTURE_BUSY_ERROR_MESSAGE
from gsa_taskflow_executor.mqtt.robot_state_models import (
    CAMERA_CALIBRATION_REQUEST_TYPE,
    CAMERA_CAPTURE_START_REQUEST_TYPE,
    CAMERA_CAPTURE_STOP_REQUEST_TYPE,
    CAMERA_FRAME_REQUEST_TYPE,
    CURRENT_POSE_REQUEST_TYPE,
)

ROBOT_BUSY_ERROR_CODE = "ROBOT_BUSY"
ROBOT_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，当前位姿读取已拒绝"
CAMERA_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，相机图像读取已拒绝"
CAMERA_CALIBRATION_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，相机标定读取已拒绝"


def build_current_pose_response(
    *,
    request_id: str,
    executor_aid: str,
    snapshot: Mapping[str, object],
) -> dict[str, object]:
    """构建当前位姿响应。available → ok, busy → ROBOT_BUSY, 其他 → GDK_UNAVAILABLE。"""

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
    """构建相机帧响应。"""

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


def build_camera_calibration_response(
    *,
    request_id: str,
    executor_aid: str,
    snapshot: Mapping[str, object],
) -> dict[str, object]:
    """构建相机标定响应。"""

    if snapshot.get("available") is True:
        return {
            "type": CAMERA_CALIBRATION_REQUEST_TYPE,
            "requestId": request_id,
            "ok": True,
            "executorAid": executor_aid,
            "data": dict(snapshot),
        }

    if snapshot.get("busy") is True:
        return error_response(
            response_type=CAMERA_CALIBRATION_REQUEST_TYPE,
            request_id=request_id,
            executor_aid=executor_aid,
            code=ROBOT_BUSY_ERROR_CODE,
            message=CAMERA_CALIBRATION_BUSY_ERROR_MESSAGE,
            details=dict(snapshot),
        )

    return error_response(
        response_type=CAMERA_CALIBRATION_REQUEST_TYPE,
        request_id=request_id,
        executor_aid=executor_aid,
        code=read_error_code(snapshot, "GDK_CAMERA_CALIBRATION_UNAVAILABLE"),
        message=read_error_message(snapshot, fallback="GDK 相机标定读取失败"),
        details=dict(snapshot),
    )


def build_camera_capture_start_response(
    *,
    request_id: str,
    executor_aid: str,
    result: Mapping[str, object],
) -> dict[str, object]:
    """构建相机连续采集开始响应。"""

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
    """构建相机连续采集停止响应。"""

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
    """构建标准错误响应。ok=False，含 code/message/details。"""

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


def read_error_code(snapshot: Mapping[str, object], fallback: str) -> str:
    """从 snapshot 提取 errorCode/error_code。"""

    error_code = snapshot.get("errorCode") or snapshot.get("error_code")
    if isinstance(error_code, str) and error_code.strip():
        return error_code.strip()
    return fallback


def read_error_message(
    snapshot: Mapping[str, object],
    fallback: str = "GDK 当前位姿读取失败",
) -> str:
    """从 snapshot 提取 errorMsg/error_msg。失败时含 error_stage。"""

    error_msg = snapshot.get("errorMsg") or snapshot.get("error_msg")
    if isinstance(error_msg, str) and error_msg.strip():
        return error_msg.strip()
    error_stage = snapshot.get("errorStage") or snapshot.get("error_stage")
    if isinstance(error_stage, str) and error_stage.strip():
        return f"{fallback}: {error_stage}"
    return fallback
