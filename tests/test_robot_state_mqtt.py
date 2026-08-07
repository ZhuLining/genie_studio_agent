import json

from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.mqtt.robot_state import (
    build_camera_capture_start_response,
    build_camera_capture_stop_response,
    build_camera_frame_response,
    build_current_pose_response,
    handle_camera_capture_start_request,
    handle_camera_capture_stop_request,
    handle_camera_frame_request,
    handle_current_pose_request,
    handle_robot_state_request,
    parse_camera_capture_start_request,
    parse_camera_capture_stop_request,
    parse_camera_frame_request,
    parse_current_pose_request,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings


def make_message(payload: str) -> TaskflowMessage:
    return TaskflowMessage(
        topic="gsa/self/robot/state/get_current_pose/request",
        payload=payload,
        received_at="2026-07-27T00:00:00+00:00",
    )


def test_parse_current_pose_request_accepts_camel_case_and_reply_topic() -> None:
    request = parse_current_pose_request(
        json.dumps(
            {
                "type": "get_current_pose",
                "requestId": "req-1",
                "replyTopic": "robot/custom/response",
            }
        ),
        default_reply_topic="gsa/self/robot/state/get_current_pose/response",
    )

    assert request.request_id == "req-1"
    assert request.reply_topic == "robot/custom/response"


def test_handle_current_pose_request_publishes_success_response() -> None:
    published: list[tuple[str, dict[str, object]]] = []
    snapshot = {
        "available": True,
        "backend": "agibot_gdk.Robot",
        "groups": {},
    }

    handle_current_pose_request(
        make_message(json.dumps({"requestId": "req-1"})),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_snapshot=lambda: snapshot,
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_current_pose/response"
    assert payload["type"] == "get_current_pose"
    assert payload["requestId"] == "req-1"
    assert payload["ok"] is True
    assert payload["executorAid"] == "aid-1"
    assert payload["data"] == snapshot


def test_handle_current_pose_request_publishes_invalid_request_error() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_current_pose_request(
        make_message("{broken"),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_snapshot=lambda: {"available": True},
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_current_pose/response"
    assert payload["ok"] is False
    assert payload["requestId"] == ""
    assert payload["error"]["code"] == "INVALID_REQUEST"


def test_build_current_pose_response_maps_unavailable_snapshot_to_error() -> None:
    response = build_current_pose_response(
        request_id="req-2",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "errorStage": "import_agibot_gdk",
            "errorMsg": "No module named agibot_gdk",
        },
    )

    assert response["ok"] is False
    assert response["requestId"] == "req-2"
    assert response["error"]["code"] == "GDK_UNAVAILABLE"
    assert response["error"]["message"] == "No module named agibot_gdk"


def test_build_current_pose_response_maps_busy_snapshot_to_robot_busy() -> None:
    response = build_current_pose_response(
        request_id="req-3",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "busy": True,
            "errorStage": "gdk_session_busy",
            "errorMsg": "ignored busy detail",
        },
    )

    assert response["ok"] is False
    assert response["requestId"] == "req-3"
    assert response["error"]["code"] == "ROBOT_BUSY"
    assert response["error"]["message"] == "GDK 正在执行控制动作，当前位姿读取已拒绝"


def test_parse_camera_frame_request_defaults_camera_and_timeout() -> None:
    request = parse_camera_frame_request(
        json.dumps(
            {
                "type": "get_camera_frame",
                "requestId": "req-camera",
                "replyTopic": "robot/camera/response",
            }
        ),
        default_reply_topic="gsa/self/robot/state/get_camera_frame/response",
    )

    assert request.request_id == "req-camera"
    assert request.reply_topic == "robot/camera/response"
    assert request.camera_id == "hand_left_color"
    assert request.timeout_ms == 3000


def test_parse_camera_capture_start_request_defaults_frame_topic() -> None:
    request = parse_camera_capture_start_request(
        json.dumps(
            {
                "type": "start_camera_capture",
                "requestId": "req-start",
                "sessionId": "session-1",
                "cameraId": "head_color",
                "captureRateFps": 5,
                "timeoutMs": 2500,
            }
        ),
        default_reply_topic="gsa/self/robot/state/camera_capture/start/response",
        default_frame_topic_template="gsa/self/robot/state/camera_capture/{sessionId}/frame",
    )

    assert request.request_id == "req-start"
    assert request.reply_topic == "gsa/self/robot/state/camera_capture/start/response"
    assert request.params.session_id == "session-1"
    assert request.params.frame_topic == "gsa/self/robot/state/camera_capture/session-1/frame"
    assert request.params.camera_id == "head_color"
    assert request.params.capture_rate_fps == 5
    assert request.params.timeout_ms == 2500


def test_parse_camera_capture_stop_request_reads_session_id() -> None:
    request = parse_camera_capture_stop_request(
        json.dumps(
            {
                "type": "stop_camera_capture",
                "requestId": "req-stop",
                "sessionId": "session-1",
                "replyTopic": "robot/capture/stop/response/session-1",
            }
        ),
        default_reply_topic="gsa/self/robot/state/camera_capture/stop/response",
    )

    assert request.request_id == "req-stop"
    assert request.reply_topic == "robot/capture/stop/response/session-1"
    assert request.session_id == "session-1"


def test_handle_camera_frame_request_publishes_success_response() -> None:
    published: list[tuple[str, dict[str, object]]] = []
    snapshot = {
        "available": True,
        "backend": "agibot_gdk.Robot",
        "cameraId": "hand_left_color",
        "mimeType": "image/jpeg",
        "imageBase64": "abc",
    }

    handle_camera_frame_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/get_camera_frame/request",
            payload=json.dumps({"requestId": "req-camera"}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_snapshot=lambda camera_id, timeout_ms: {
            **snapshot,
            "cameraId": camera_id,
            "timeoutMs": timeout_ms,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_camera_frame/response"
    assert payload["type"] == "get_camera_frame"
    assert payload["requestId"] == "req-camera"
    assert payload["ok"] is True
    assert payload["executorAid"] == "aid-1"
    assert payload["data"]["cameraId"] == "hand_left_color"
    assert payload["data"]["timeoutMs"] == 3000


def test_handle_robot_state_request_dispatches_camera_frame_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/get_camera_frame/request",
            payload=json.dumps({"requestId": "req-camera", "cameraId": "head_color"}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_camera_frame=lambda camera_id, _timeout_ms: {
            "available": True,
            "cameraId": camera_id,
            "mimeType": "image/jpeg",
            "imageBase64": "abc",
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_camera_frame/response"
    assert payload["type"] == "get_camera_frame"
    assert payload["data"]["cameraId"] == "head_color"


def test_handle_camera_capture_start_request_publishes_success_response() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_camera_capture_start_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/camera_capture/start/request",
            payload=json.dumps({"requestId": "req-start", "sessionId": "session-1"}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        start_camera_capture=lambda params: {
            "started": True,
            "sessionId": params.session_id,
            "frameTopic": params.frame_topic,
            "cameraId": params.camera_id,
            "captureRateFps": params.capture_rate_fps,
            "timeoutMs": params.timeout_ms,
            "startedAt": "2026-07-27T00:00:00+00:00",
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/camera_capture/start/response"
    assert payload["type"] == "start_camera_capture"
    assert payload["requestId"] == "req-start"
    assert payload["ok"] is True
    assert payload["data"]["sessionId"] == "session-1"
    assert (
        payload["data"]["frameTopic"]
        == "gsa/self/robot/state/camera_capture/session-1/frame"
    )


def test_handle_camera_capture_stop_request_publishes_success_response() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_camera_capture_stop_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/camera_capture/stop/request",
            payload=json.dumps({"requestId": "req-stop", "sessionId": "session-1"}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        stop_camera_capture=lambda session_id: {
            "stopped": True,
            "sessionId": session_id,
            "framesCaptured": 3,
            "framesPublished": 3,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/camera_capture/stop/response"
    assert payload["type"] == "stop_camera_capture"
    assert payload["requestId"] == "req-stop"
    assert payload["ok"] is True
    assert payload["data"]["sessionId"] == "session-1"
    assert payload["data"]["framesPublished"] == 3


def test_handle_robot_state_request_dispatches_camera_capture_start_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/camera_capture/start/request",
            payload=json.dumps({"requestId": "req-start", "sessionId": "session-1"}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        start_camera_capture=lambda params: {
            "started": True,
            "sessionId": params.session_id,
            "frameTopic": params.frame_topic,
            "cameraId": params.camera_id,
            "captureRateFps": params.capture_rate_fps,
            "timeoutMs": params.timeout_ms,
            "startedAt": "2026-07-27T00:00:00+00:00",
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/camera_capture/start/response"
    assert payload["type"] == "start_camera_capture"
    assert payload["data"]["cameraId"] == "hand_left_color"


def test_build_camera_frame_response_maps_busy_snapshot_to_robot_busy() -> None:
    response = build_camera_frame_response(
        request_id="req-camera",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "busy": True,
            "errorStage": "gdk_session_busy",
        },
    )

    assert response["ok"] is False
    assert response["type"] == "get_camera_frame"
    assert response["error"]["code"] == "ROBOT_BUSY"
    assert response["error"]["message"] == "GDK 正在执行控制动作，相机图像读取已拒绝"


def test_build_camera_capture_start_response_maps_busy_to_robot_busy() -> None:
    response = build_camera_capture_start_response(
        request_id="req-start",
        executor_aid="aid-1",
        result={"started": False, "busy": True},
    )

    assert response["ok"] is False
    assert response["type"] == "start_camera_capture"
    assert response["error"]["code"] == "ROBOT_BUSY"
    assert response["error"]["message"] == "GDK 正在执行控制动作，相机连续采集已拒绝"


def test_build_camera_capture_stop_response_maps_not_found_to_error() -> None:
    response = build_camera_capture_stop_response(
        request_id="req-stop",
        executor_aid="aid-1",
        result={
            "stopped": False,
            "errorCode": "CAMERA_CAPTURE_NOT_FOUND",
            "errorMsg": "未找到正在运行的相机采集会话",
        },
    )

    assert response["ok"] is False
    assert response["type"] == "stop_camera_capture"
    assert response["error"]["code"] == "CAMERA_CAPTURE_NOT_FOUND"


def test_build_camera_frame_response_preserves_subprocess_timeout_error() -> None:
    response = build_camera_frame_response(
        request_id="req-camera-timeout",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "error_code": "GDK_OPERATION_TIMEOUT",
            "error_msg": "GDK operation get_camera_frame exceeded timeout 6.500s",
        },
    )

    assert response["ok"] is False
    assert response["type"] == "get_camera_frame"
    assert response["error"]["code"] == "GDK_OPERATION_TIMEOUT"
    assert response["error"]["message"] == "GDK operation get_camera_frame exceeded timeout 6.500s"
