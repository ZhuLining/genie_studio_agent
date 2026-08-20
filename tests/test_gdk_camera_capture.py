from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.gdk.camera_capture import (
    CameraCaptureService,
    CameraCaptureStartParams,
    build_frame_payload,
    validate_start_params,
)


class FakeLease:
    def __init__(self, purpose: str) -> None:
        self.purpose = purpose
        self.released = False

    def to_payload(self) -> dict[str, object]:
        return {"purpose": self.purpose}

    def release(self) -> None:
        self.released = True


class FakeSessionManager:
    def __init__(self, *, busy: bool = False) -> None:
        self.busy = busy
        self.active_purpose: str | None = "motion_plan" if busy else None
        self.leases: list[FakeLease] = []

    def acquire(self, **kwargs: object) -> FakeLease | None:
        if self.busy:
            return None
        lease = FakeLease(str(kwargs["purpose"]))
        self.leases.append(lease)
        self.active_purpose = lease.purpose
        return lease


def fake_camera_capture_child(
    summary_queue: Any,
    stop_event: Any,
    payload: dict[str, object],
) -> None:
    stop_event.wait(5.0)
    summary_queue.put(
        {
            "stopped": True,
            "sessionId": payload["sessionId"],
            "cameraId": payload["cameraId"],
            "captureRateFps": payload["captureRateFps"],
            "framesCaptured": 2,
            "framesPublished": 2,
        }
    )
    summary_queue.cancel_join_thread()


def test_camera_capture_service_start_and_stop_releases_parent_lease() -> None:
    session_manager = FakeSessionManager()
    service = CameraCaptureService(
        session_manager=session_manager,  # type: ignore[arg-type]
        mqtt_broker_url="mqtt://127.0.0.1:1883",
        mqtt_client_id="executor-test",
        executor_aid="aid-1",
        child_target=fake_camera_capture_child,
    )

    start_result = service.start(
        CameraCaptureStartParams(
            session_id="session-1",
            frame_topic="gsa/self/robot/state/camera_capture/session-1/frame",
            camera_id="hand_left_color",
            capture_rate_fps=5,
            timeout_ms=3000,
        )
    )
    stop_result = service.stop("session-1")

    assert start_result["started"] is True
    assert stop_result["stopped"] is True
    assert stop_result["sessionId"] == "session-1"
    assert stop_result["framesPublished"] == 2
    assert "stopKilled" in stop_result
    assert session_manager.leases[0].released is True


def test_camera_capture_service_returns_busy_when_gdk_session_is_busy() -> None:
    service = CameraCaptureService(
        session_manager=FakeSessionManager(busy=True),  # type: ignore[arg-type]
        mqtt_broker_url="mqtt://127.0.0.1:1883",
        mqtt_client_id="executor-test",
        executor_aid="aid-1",
        child_target=fake_camera_capture_child,
    )

    result = service.start(
        CameraCaptureStartParams(
            session_id="session-1",
            frame_topic="gsa/self/robot/state/camera_capture/session-1/frame",
            camera_id="hand_left_color",
            capture_rate_fps=5,
            timeout_ms=3000,
        )
    )

    assert result["started"] is False
    assert result["busy"] is True
    assert result["errorCode"] == "ROBOT_BUSY"


def test_validate_start_params_rejects_invalid_capture_rate() -> None:
    result = validate_start_params(
        CameraCaptureStartParams(
            session_id="session-1",
            frame_topic="gsa/self/robot/state/camera_capture/session-1/frame",
            camera_id="hand_left_color",
            capture_rate_fps=11,
            timeout_ms=3000,
        )
    )

    assert result is not None
    assert result["started"] is False
    assert result["errorCode"] == "INVALID_REQUEST"


def test_build_frame_payload_uses_capture_metadata() -> None:
    payload = build_frame_payload(
        {
            "cameraId": "hand_left_color",
            "mimeType": "image/jpeg",
            "imageBase64": "abc",
            "width": 1280,
            "height": 1056,
            "encoding": "JPEG",
            "imageSha256": "sha",
            "timestampNs": "123",
            "collectedAt": "2026-08-07T00:00:00+00:00",
        },
        session_id="session-1",
        frame_index=7,
        capture_rate_fps=5,
        executor_aid="aid-1",
        duplicate_timestamp_count=2,
        reopened_camera_count=1,
    )

    assert payload["type"] == "camera_capture_frame"
    assert payload["sessionId"] == "session-1"
    assert payload["frameIndex"] == 7
    assert payload["captureRateFps"] == 5
    assert payload["cameraId"] == "hand_left_color"
    assert payload["imageSha256"] == "sha"
    assert payload["duplicateTimestampCount"] == 2
    assert payload["reopenedCameraCount"] == 1
