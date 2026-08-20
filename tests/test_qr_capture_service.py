import builtins
from threading import Event
from typing import Any

from gsa_taskflow_executor.qr_mapping.capture_service import (
    ActiveQrCaptureSession,
    QrCaptureService,
    QrCaptureStartParams,
    build_child_payload,
    reconcile_stop_result_with_filesystem,
    qr_capture_child,
    validate_start_params,
)
from gsa_taskflow_executor.qr_mapping.project_store import QrProjectStore


class FakeSessionManager:
    active_purpose: str | None = None

    def acquire(self, **_kwargs: object) -> None:
        raise AssertionError("existing project should be rejected before acquiring GDK")


class FakeQueue:
    def __init__(self) -> None:
        self.items: list[dict[str, object]] = []

    def put(self, item: dict[str, object], **_kwargs: object) -> None:
        self.items.append(item)

    def cancel_join_thread(self) -> None:
        pass


class StopAfterOneFrame:
    def __init__(self) -> None:
        self.calls = 0

    def is_set(self) -> bool:
        self.calls += 1
        return self.calls > 1

    def wait(self, _timeout: float) -> None:
        pass


class FakePublishInfo:
    def wait_for_publish(self, **_kwargs: object) -> None:
        pass


class FakeMqttClient:
    def __init__(self) -> None:
        self.published: list[tuple[str, str]] = []

    def publish(self, topic: str, payload: str, **_kwargs: object) -> FakePublishInfo:
        self.published.append((topic, payload))
        return FakePublishInfo()

    def loop_stop(self) -> None:
        pass

    def disconnect(self) -> None:
        pass


class FakeCameraType:
    kHandLeftColor = "FakeHandLeftColor"


class FakeImage:
    width = 2
    height = 1
    encoding = "JPEG"
    color_format = "RGB"
    bit_depth = 8
    timestamp_ns = 1785315256907000000
    data = b"\xff\xd8test-jpeg"


class FakeCamera:
    closed = False

    def get_latest_image(self, _camera_type: Any, _timeout: float) -> FakeImage:
        return FakeImage()

    def close_camera(self) -> None:
        type(self).closed = True


class FakeAgibotGdk:
    CameraType = FakeCameraType
    Camera = FakeCamera


def test_qr_capture_service_rejects_existing_project_without_gdk_acquire(tmp_path) -> None:
    project_root = tmp_path / "G2A0004BC01053" / "qr_pose_skill_conf" / "test10"
    images_dir = project_root / "images"
    images_dir.mkdir(parents=True)
    (images_dir / "1785315256907.jpg").write_bytes(b"fake-jpg")

    service = QrCaptureService(
        session_manager=FakeSessionManager(),  # type: ignore[arg-type]
        project_store=QrProjectStore(tmp_path),
        mqtt_broker_url="mqtt://127.0.0.1:1883",
        mqtt_client_id="executor-test",
        executor_aid="aid-1",
    )
    result = service.start(
        QrCaptureStartParams(
            session_id="session-qr",
            frame_topic="gsa/self/robot/qr_mapping/capture/session-qr/frame",
            robot_serial="G2A0004BC01053",
            project_name="test10",
            camera_id="hand_left_color",
            marker_type="ARUCO_MIP_36h12",
            marker_size_meters=0.04,
            capture_rate_fps=5,
            timeout_ms=3000,
        )
    )

    assert result["started"] is False
    assert result["errorCode"] == "PROJECT_ALREADY_EXISTS"
    assert "已存在" in result["errorMsg"]


def test_validate_qr_capture_start_params_rejects_invalid_camera() -> None:
    result = validate_start_params(
        QrCaptureStartParams(
            session_id="session-qr",
            frame_topic="gsa/self/robot/qr_mapping/capture/session-qr/frame",
            robot_serial="G2A0004BC01053",
            project_name="test10",
            camera_id="bad_camera",
            marker_type="ARUCO_MIP_36h12",
            marker_size_meters=0.04,
            capture_rate_fps=5,
            timeout_ms=3000,
        )
    )

    assert result is not None
    assert result["started"] is False
    assert result["errorCode"] == "INVALID_REQUEST"


def test_qr_capture_child_uses_payload_calibration_in_ready_result(tmp_path, monkeypatch) -> None:
    paths = QrProjectStore(tmp_path).ensure_project_layout(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )
    calibration = {
        "calibrationFilePath": str(paths.sensor_root / "intrinsic_hand_left_rgb.json"),
        "extrinsicFilePath": str(paths.sensor_root / "extrinsic_end_T_hand_left_rgbd.json"),
        "calibrationWarnings": [],
    }
    payload = build_child_payload(
        QrCaptureStartParams(
            session_id="session-qr",
            frame_topic="gsa/self/robot/qr_mapping/capture/session-qr/frame",
            robot_serial="G2A0004BC01053",
            project_name="test10",
            camera_id="hand_left_color",
            marker_type="ARUCO_MIP_36h12",
            marker_size_meters=0.04,
            capture_rate_fps=5,
            timeout_ms=3000,
        ),
        paths=paths,
        calibration=calibration,
        mqtt_broker_url="mqtt://127.0.0.1:1883",
        mqtt_client_id="executor-test",
        executor_aid="aid-1",
    )

    original_import = builtins.__import__

    def fake_import(name: str, *args: object, **kwargs: object) -> object:
        if name == "agibot_gdk":
            return FakeAgibotGdk
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.capture_service.connect_frame_mqtt_client",
        lambda _payload: FakeMqttClient(),
    )
    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.capture_service.initialize_gdk",
        lambda _gdk: {"called": False, "success": True, "return": None},
    )
    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.capture_service.release_gdk",
        lambda _gdk: {"called": False, "success": True, "return": None},
    )
    monkeypatch.setattr("gsa_taskflow_executor.qr_mapping.capture_service.time.sleep", lambda _seconds: None)

    ready_queue = FakeQueue()
    summary_queue = FakeQueue()
    qr_capture_child(ready_queue, summary_queue, StopAfterOneFrame(), payload)

    assert ready_queue.items[0]["started"] is True
    assert ready_queue.items[0]["calibrationSaved"] is True
    assert ready_queue.items[0]["calibrationFilePath"] == calibration["calibrationFilePath"]
    assert ready_queue.items[0]["extrinsicFilePath"] == calibration["extrinsicFilePath"]


def test_qr_capture_stop_result_uses_filesystem_image_count_when_summary_is_zero(tmp_path) -> None:
    paths = QrProjectStore(tmp_path).ensure_project_layout(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )
    (paths.images_dir / "1785315256907.jpg").write_bytes(b"one")
    (paths.images_dir / "1785315257907.jpg").write_bytes(b"two")
    params = QrCaptureStartParams(
        session_id="session-qr",
        frame_topic="gsa/self/robot/qr_mapping/capture/session-qr/frame",
        robot_serial="G2A0004BC01053",
        project_name="test10",
        camera_id="hand_left_color",
        marker_type="ARUCO_MIP_36h12",
        marker_size_meters=0.04,
        capture_rate_fps=5,
        timeout_ms=3000,
    )
    session = ActiveQrCaptureSession(
        session_id="session-qr",
        params=params,
        paths=paths,
        lease=object(),  # type: ignore[arg-type]
        process=object(),
        stop_event=object(),
        summary_queue=object(),
        started_at="2026-08-20T00:00:00+00:00",
        completed=Event(),
    )

    result = reconcile_stop_result_with_filesystem(
        {"sessionId": "session-qr", "framesSaved": 0, "imageCount": 0},
        session,
    )

    assert result["framesSaved"] == 2
    assert result["imageCount"] == 2
    assert result["framesSavedSource"] == "filesystem_reconciled"
