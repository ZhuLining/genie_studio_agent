from gsa_taskflow_executor.qr_mapping.capture_service import (
    QrCaptureService,
    QrCaptureStartParams,
    validate_start_params,
)
from gsa_taskflow_executor.qr_mapping.project_store import QrProjectStore


class FakeSessionManager:
    active_purpose: str | None = None

    def acquire(self, **_kwargs: object) -> None:
        raise AssertionError("existing project should be rejected before acquiring GDK")


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
