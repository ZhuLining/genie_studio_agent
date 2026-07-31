import json

from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.mqtt.robot_state import (
    build_current_pose_response,
    handle_current_pose_request,
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
