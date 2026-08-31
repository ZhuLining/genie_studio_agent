import json

from gsa_taskflow_executor.mqtt.gateway import TaskflowMessage
from gsa_taskflow_executor.mqtt.robot_state import (
    build_camera_calibration_response,
    build_camera_capture_start_response,
    build_camera_capture_stop_response,
    build_camera_frame_response,
    build_current_pose_response,
    build_robot_identity_response,
    handle_camera_calibration_request,
    handle_camera_capture_start_request,
    handle_camera_capture_stop_request,
    handle_camera_frame_request,
    handle_current_pose_request,
    handle_robot_identity_request,
    handle_robot_state_request,
    parse_camera_calibration_request,
    parse_camera_capture_start_request,
    parse_camera_capture_stop_request,
    parse_camera_frame_request,
    parse_current_pose_request,
    parse_point_recording_delete_initial_photo_request,
    parse_point_recording_delete_target_request,
    parse_point_recording_save_initial_photo_request,
    parse_point_recording_save_target_request,
    parse_qr_build_map_request,
    parse_qr_capture_start_request,
    parse_qr_capture_stop_request,
    parse_qr_pcd_preview_request,
    parse_qr_project_snapshot_request,
    parse_robot_identity_request,
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


def test_parse_robot_identity_request_defaults_timeout() -> None:
    request = parse_robot_identity_request(
        json.dumps(
            {
                "type": "get_robot_identity",
                "requestId": "req-identity",
                "replyTopic": "robot/identity/response",
            }
        ),
        default_reply_topic="gsa/self/robot/state/get_robot_identity/response",
    )

    assert request.request_id == "req-identity"
    assert request.reply_topic == "robot/identity/response"
    assert request.timeout_ms == 3000


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


def test_handle_robot_identity_request_publishes_success_response() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_identity_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/get_robot_identity/request",
            payload=json.dumps({"requestId": "req-identity", "timeoutMs": 1500}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_snapshot=lambda timeout_ms: {
            "available": True,
            "backend": "agibot_gdk",
            "action": "get_robot_identity",
            "robotAid": "G2A0004BC01053",
            "robotSerial": "G2A0004BC01053",
            "suggestedRobotSerial": "G2A0004BC01053",
            "timeoutMs": timeout_ms,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_robot_identity/response"
    assert payload["type"] == "get_robot_identity"
    assert payload["requestId"] == "req-identity"
    assert payload["ok"] is True
    assert payload["data"]["robotSerial"] == "G2A0004BC01053"
    assert payload["data"]["timeoutMs"] == 1500


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


def test_build_robot_identity_response_maps_busy_snapshot_to_robot_busy() -> None:
    response = build_robot_identity_response(
        request_id="req-identity",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "busy": True,
            "errorStage": "gdk_session_busy",
        },
    )

    assert response["ok"] is False
    assert response["type"] == "get_robot_identity"
    assert response["error"]["code"] == "ROBOT_BUSY"
    assert response["error"]["message"] == "GDK 正在执行控制动作，机器人身份读取已拒绝"


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


def test_parse_camera_calibration_request_reads_camera_ids_and_extrinsics_flag() -> None:
    request = parse_camera_calibration_request(
        json.dumps(
            {
                "type": "get_camera_calibration",
                "requestId": "req-calibration",
                "replyTopic": "robot/calibration/response",
                "cameraIds": ["hand_left_color", "hand_right_color"],
                "includeExtrinsics": True,
                "timeoutMs": 2500,
            }
        ),
        default_reply_topic="gsa/self/robot/state/get_camera_calibration/response",
    )

    assert request.request_id == "req-calibration"
    assert request.reply_topic == "robot/calibration/response"
    assert request.camera_ids == ("hand_left_color", "hand_right_color")
    assert request.include_extrinsics is True
    assert request.timeout_ms == 2500


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


def test_parse_qr_project_snapshot_request_reads_project_identity() -> None:
    request = parse_qr_project_snapshot_request(
        json.dumps(
            {
                "type": "get_qr_project_snapshot",
                "requestId": "req-project",
                "replyTopic": "qr/project/snapshot/response",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "imageLimit": 20,
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/get_qr_project_snapshot/response",
    )

    assert request.request_id == "req-project"
    assert request.reply_topic == "qr/project/snapshot/response"
    assert request.robot_serial == "G2A0004BC01053"
    assert request.project_name == "test10"
    assert request.image_limit == 20


def test_parse_qr_capture_start_request_reads_remote_project_params() -> None:
    request = parse_qr_capture_start_request(
        json.dumps(
            {
                "type": "start_qr_capture",
                "requestId": "req-qr-start",
                "sessionId": "session-qr",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "cameraId": "hand_right_color",
                "markerType": "ARUCO_MIP_36h12",
                "markerSizeMeters": 0.04,
                "captureRateFps": 5,
                "timeoutMs": 2500,
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/start_capture/response",
        default_frame_topic_template="gsa/self/robot/qr_mapping/capture/{sessionId}/frame",
    )

    assert request.request_id == "req-qr-start"
    assert request.reply_topic == "gsa/self/robot/qr_mapping/start_capture/response"
    assert request.params.session_id == "session-qr"
    assert request.params.frame_topic == "gsa/self/robot/qr_mapping/capture/session-qr/frame"
    assert request.params.robot_serial == "G2A0004BC01053"
    assert request.params.project_name == "test10"
    assert request.params.camera_id == "hand_right_color"
    assert request.params.marker_size_meters == 0.04
    assert request.params.capture_rate_fps == 5


def test_parse_qr_capture_stop_request_reads_session_id() -> None:
    request = parse_qr_capture_stop_request(
        json.dumps(
            {
                "type": "stop_qr_capture",
                "requestId": "req-qr-stop",
                "sessionId": "session-qr",
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/stop_capture/response",
    )

    assert request.request_id == "req-qr-stop"
    assert request.session_id == "session-qr"
    assert request.reply_topic == "gsa/self/robot/qr_mapping/stop_capture/response"


def test_parse_qr_build_map_request_reads_sdk_params() -> None:
    request = parse_qr_build_map_request(
        json.dumps(
            {
                "type": "build_qr_map",
                "requestId": "req-build",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "mapName": "map01",
                "cameraId": "hand_left_color",
                "markerType": "ARUCO_MIP_36h12",
                "markerSizeMeters": 0.04,
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/build_map/response",
    )

    assert request.request_id == "req-build"
    assert request.robot_serial == "G2A0004BC01053"
    assert request.project_name == "test10"
    assert request.map_name == "map01"
    assert request.camera_id == "hand_left_color"
    assert request.marker_type == "ARUCO_MIP_36h12"
    assert request.marker_size_meters == 0.04


def test_parse_qr_pcd_preview_request_caps_max_points() -> None:
    request = parse_qr_pcd_preview_request(
        json.dumps(
            {
                "type": "read_qr_pcd_preview",
                "requestId": "req-pcd",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "mapName": "map01",
                "maxPoints": 999999,
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/read_pcd_preview/response",
    )

    assert request.max_points == 50000


def test_parse_point_recording_save_target_request_reads_params() -> None:
    request = parse_point_recording_save_target_request(
        json.dumps(
            {
                "type": "save_qr_target_point",
                "requestId": "req-target",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "pointName": "grasp_1",
                "arm": "right_arm",
                "timeoutMs": 2500,
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/save_target_point/response",
    )

    assert request.request_id == "req-target"
    assert request.params.robot_serial == "G2A0004BC01053"
    assert request.params.project_name == "test10"
    assert request.params.point_name == "grasp_1"
    assert request.params.arm == "right_arm"
    assert request.params.camera_id == "hand_right_color"
    assert request.params.timeout_ms == 2500


def test_parse_point_recording_save_initial_photo_request_reads_params() -> None:
    request = parse_point_recording_save_initial_photo_request(
        json.dumps(
            {
                "type": "save_qr_initial_photo_point",
                "requestId": "req-photo",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "pointName": "photo_001",
                "arm": "left_arm",
                "cameraId": "hand_left_color",
                "mapName": "map01",
                "minMarkers": 5,
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/save_initial_photo_point/response",
    )

    assert request.request_id == "req-photo"
    assert request.params.point_name == "photo_001"
    assert request.params.arm == "left_arm"
    assert request.params.camera_id == "hand_left_color"
    assert request.params.map_name == "map01"
    assert request.params.min_markers == 5


def test_parse_point_recording_delete_target_request_reads_params() -> None:
    request = parse_point_recording_delete_target_request(
        json.dumps(
            {
                "type": "delete_qr_target_point",
                "requestId": "req-delete-target",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "pointName": "grasp_1",
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/delete_target_point/response",
    )

    assert request.request_id == "req-delete-target"
    assert request.params.robot_serial == "G2A0004BC01053"
    assert request.params.project_name == "test10"
    assert request.params.point_name == "grasp_1"


def test_parse_point_recording_delete_initial_photo_request_reads_params() -> None:
    request = parse_point_recording_delete_initial_photo_request(
        json.dumps(
            {
                "type": "delete_qr_initial_photo_point",
                "requestId": "req-delete-photo",
                "robotSerial": "G2A0004BC01053",
                "projectName": "test10",
                "pointName": "photo_001",
            }
        ),
        default_reply_topic="gsa/self/robot/qr_mapping/delete_initial_photo_point/response",
    )

    assert request.request_id == "req-delete-photo"
    assert request.params.robot_serial == "G2A0004BC01053"
    assert request.params.project_name == "test10"
    assert request.params.point_name == "photo_001"


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


def test_handle_robot_state_request_dispatches_point_recording_target_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/save_target_point/request",
            payload=json.dumps(
                {
                    "requestId": "req-target",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                    "pointName": "grasp_1",
                    "arm": "left_arm",
                    "cameraId": "hand_left_color",
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        save_point_recording_target=lambda params: {
            "available": True,
            "backend": "executor.point_recording",
            "action": "save_qr_target_point",
            "robotSerial": params.robot_serial,
            "projectName": params.project_name,
            "pointKind": "target",
            "pointName": params.point_name,
            "arm": params.arm,
            "cameraId": params.camera_id,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/save_target_point/response"
    assert payload["type"] == "save_qr_target_point"
    assert payload["ok"] is True
    assert payload["data"]["pointName"] == "grasp_1"


def test_handle_robot_state_request_dispatches_point_recording_initial_photo_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/save_initial_photo_point/request",
            payload=json.dumps(
                {
                    "requestId": "req-photo",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                    "pointName": "photo_001",
                    "arm": "left_arm",
                    "cameraId": "hand_left_color",
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        save_point_recording_initial_photo=lambda params: {
            "available": True,
            "backend": "executor.qr_localize_sdk",
            "action": "save_qr_initial_photo_point",
            "robotSerial": params.robot_serial,
            "projectName": params.project_name,
            "pointKind": "initial_photo",
            "pointName": params.point_name,
            "arm": params.arm,
            "cameraId": params.camera_id,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/save_initial_photo_point/response"
    assert payload["type"] == "save_qr_initial_photo_point"
    assert payload["ok"] is True
    assert payload["data"]["pointKind"] == "initial_photo"


def test_handle_robot_state_request_dispatches_point_recording_delete_target_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/delete_target_point/request",
            payload=json.dumps(
                {
                    "requestId": "req-delete-target",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                    "pointName": "grasp_1",
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        delete_point_recording_target=lambda params: {
            "available": True,
            "backend": "executor.filesystem",
            "action": "delete_qr_target_point",
            "robotSerial": params.robot_serial,
            "projectName": params.project_name,
            "pointKind": "target",
            "pointName": params.point_name,
            "deletedPaths": [],
            "targetPointCount": 0,
            "initialPhotoPointCount": 1,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/delete_target_point/response"
    assert payload["type"] == "delete_qr_target_point"
    assert payload["ok"] is True
    assert payload["data"]["pointKind"] == "target"


def test_handle_robot_state_request_dispatches_point_recording_delete_initial_photo_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/delete_initial_photo_point/request",
            payload=json.dumps(
                {
                    "requestId": "req-delete-photo",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                    "pointName": "photo_001",
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        delete_point_recording_initial_photo=lambda params: {
            "available": True,
            "backend": "executor.filesystem",
            "action": "delete_qr_initial_photo_point",
            "robotSerial": params.robot_serial,
            "projectName": params.project_name,
            "pointKind": "initial_photo",
            "pointName": params.point_name,
            "deletedPaths": ["/data/gsa/waypoints/photo_001"],
            "targetPointCount": 1,
            "initialPhotoPointCount": 0,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/delete_initial_photo_point/response"
    assert payload["type"] == "delete_qr_initial_photo_point"
    assert payload["ok"] is True
    assert payload["data"]["pointName"] == "photo_001"


def test_handle_robot_state_request_dispatches_point_recording_submit_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/submit_point_recording/request",
            payload=json.dumps(
                {
                    "requestId": "req-submit",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        submit_point_recording=lambda params: {
            "available": True,
            "backend": "executor.filesystem",
            "action": "submit_point_recording",
            "robotSerial": params.robot_serial,
            "projectName": params.project_name,
            "targetPointCount": 1,
            "initialPhotoPointCount": 1,
            "submittedAt": "2026-08-20T00:00:00+00:00",
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/submit_point_recording/response"
    assert payload["type"] == "submit_point_recording"
    assert payload["ok"] is True
    assert payload["data"]["targetPointCount"] == 1


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


def test_handle_robot_state_request_dispatches_qr_project_snapshot_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/get_qr_project_snapshot/request",
            payload=json.dumps(
                {
                    "requestId": "req-project",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                    "imageLimit": 5,
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        get_qr_project_snapshot=lambda robot_serial, project_name, image_limit: {
            "available": True,
            "backend": "executor.filesystem",
            "action": "get_qr_project_snapshot",
            "robotSerial": robot_serial,
            "projectName": project_name,
            "projectRoot": "/data/gsa/G2A0004BC01053/qr_pose_skill_conf/test10",
            "images": [],
            "maps": [],
            "imageLimit": image_limit,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/get_qr_project_snapshot/response"
    assert payload["type"] == "get_qr_project_snapshot"
    assert payload["data"]["robotSerial"] == "G2A0004BC01053"
    assert payload["data"]["imageLimit"] == 5


def test_handle_robot_state_request_dispatches_qr_build_map_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/qr_mapping/build_map/request",
            payload=json.dumps(
                {
                    "requestId": "req-build",
                    "robotSerial": "G2A0004BC01053",
                    "projectName": "test10",
                    "mapName": "map01",
                    "cameraId": "hand_left_color",
                    "markerType": "ARUCO_MIP_36h12",
                    "markerSizeMeters": 0.04,
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        build_qr_map=lambda robot_serial,
        project_name,
        map_name,
        camera_id,
        marker_type,
        marker_size: {
            "available": True,
            "backend": "executor.qr_mapping_sdk",
            "action": "build_qr_map",
            "robotSerial": robot_serial,
            "projectName": project_name,
            "mapName": map_name,
            "cameraId": camera_id,
            "markerType": marker_type,
            "markerSizeMeters": marker_size,
            "mapYmlPath": "/data/gsa/G2A0004BC01053/qr_pose_skill_conf/test10/maps/map01.yml",
            "status": "success",
            "updatedAt": "2026-08-19T00:00:00+00:00",
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/qr_mapping/build_map/response"
    assert payload["type"] == "build_qr_map"
    assert payload["ok"] is True
    assert payload["data"]["mapName"] == "map01"
    assert payload["data"]["markerSizeMeters"] == 0.04


def test_handle_camera_calibration_request_publishes_success_response() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_camera_calibration_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/get_camera_calibration/request",
            payload=json.dumps(
                {
                    "requestId": "req-calibration",
                    "cameraIds": ["hand_left_color"],
                    "includeExtrinsics": True,
                }
            ),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_snapshot=lambda camera_ids, timeout_ms, include_extrinsics: {
            "available": True,
            "cameraIds": list(camera_ids),
            "timeoutMs": timeout_ms,
            "includeExtrinsics": include_extrinsics,
            "calibrations": [
                {
                    "cameraId": "hand_left_color",
                    "fx": 1,
                    "fy": 2,
                    "cx": 3,
                    "cy": 4,
                }
            ],
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_camera_calibration/response"
    assert payload["type"] == "get_camera_calibration"
    assert payload["requestId"] == "req-calibration"
    assert payload["ok"] is True
    assert payload["data"]["cameraIds"] == ["hand_left_color"]
    assert payload["data"]["includeExtrinsics"] is True


def test_handle_robot_state_request_dispatches_camera_calibration_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/get_camera_calibration/request",
            payload=json.dumps({"requestId": "req-calibration", "cameraIds": ["head_color"]}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_camera_calibration=lambda camera_ids, _timeout_ms, _include_extrinsics: {
            "available": True,
            "cameraIds": list(camera_ids),
            "calibrations": [{"cameraId": camera_ids[0]}],
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_camera_calibration/response"
    assert payload["type"] == "get_camera_calibration"
    assert payload["data"]["cameraIds"] == ["head_color"]


def test_handle_robot_state_request_dispatches_robot_identity_by_topic() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    handle_robot_state_request(
        TaskflowMessage(
            topic="gsa/self/robot/state/get_robot_identity/request",
            payload=json.dumps({"requestId": "req-identity"}),
            received_at="2026-07-27T00:00:00+00:00",
        ),
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_response=lambda topic, payload: published.append((topic, dict(payload))),
        collect_robot_identity=lambda timeout_ms: {
            "available": True,
            "backend": "agibot_gdk",
            "action": "get_robot_identity",
            "robotAid": "G2A0004BC01053",
            "robotSerial": "G2A0004BC01053",
            "suggestedRobotSerial": "G2A0004BC01053",
            "timeoutMs": timeout_ms,
        },
    )

    [(topic, payload)] = published
    assert topic == "gsa/self/robot/state/get_robot_identity/response"
    assert payload["type"] == "get_robot_identity"
    assert payload["data"]["robotSerial"] == "G2A0004BC01053"


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


def test_build_camera_calibration_response_maps_busy_snapshot_to_robot_busy() -> None:
    response = build_camera_calibration_response(
        request_id="req-calibration",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "busy": True,
            "errorStage": "gdk_session_busy",
        },
    )

    assert response["ok"] is False
    assert response["type"] == "get_camera_calibration"
    assert response["error"]["code"] == "ROBOT_BUSY"
    assert response["error"]["message"] == "GDK 正在执行控制动作，相机标定读取已拒绝"


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


def test_build_camera_calibration_response_preserves_subprocess_timeout_error() -> None:
    response = build_camera_calibration_response(
        request_id="req-calibration-timeout",
        executor_aid="aid-1",
        snapshot={
            "available": False,
            "error_code": "GDK_OPERATION_TIMEOUT",
            "error_msg": "GDK operation get_camera_calibration exceeded timeout 6.500s",
        },
    )

    assert response["ok"] is False
    assert response["type"] == "get_camera_calibration"
    assert response["error"]["code"] == "GDK_OPERATION_TIMEOUT"
    assert (
        response["error"]["message"]
        == "GDK operation get_camera_calibration exceeded timeout 6.500s"
    )
