from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from gsa_taskflow_executor.gdk.current_pose import run_gdk_current_pose_snapshot
from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.runtime.event_log import JsonlEventWriter, RuntimeEvent

CURRENT_POSE_REQUEST_TYPE = "get_current_pose"
ROBOT_BUSY_ERROR_CODE = "ROBOT_BUSY"
ROBOT_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，当前位姿读取已拒绝"

RobotStatePublisher = Callable[[str, Mapping[str, Any]], None]
CurrentPoseCollector = Callable[[], Mapping[str, object]]


@dataclass(frozen=True)
class CurrentPoseRequest:
    request_id: str
    reply_topic: str


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
            request_id=request_id,
            executor_aid=executor_aid,
            code=ROBOT_BUSY_ERROR_CODE,
            message=ROBOT_BUSY_ERROR_MESSAGE,
            details=dict(snapshot),
        )

    return error_response(
        request_id=request_id,
        executor_aid=executor_aid,
        code="GDK_UNAVAILABLE",
        message=read_error_message(snapshot),
        details=dict(snapshot),
    )


def error_response(
    *,
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
        "type": CURRENT_POSE_REQUEST_TYPE,
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


def read_error_message(snapshot: Mapping[str, object]) -> str:
    error_msg = snapshot.get("errorMsg")
    if isinstance(error_msg, str) and error_msg.strip():
        return error_msg.strip()
    error_stage = snapshot.get("errorStage")
    if isinstance(error_stage, str) and error_stage.strip():
        return f"GDK 当前位姿读取失败: {error_stage}"
    return "GDK 当前位姿读取失败"


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
