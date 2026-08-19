from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.gdk.camera_calibration import (
    run_gdk_camera_calibration_snapshot,
)
from gsa_taskflow_executor.gdk.session import GdkSessionManager


class FakeCameraType:
    kHandLeftColor = "FakeHandLeftColor"
    kHandRightColor = "FakeHandRightColor"
    kHeadColor = "FakeHeadColor"


class FakeSensorExtrinsicType:
    kLeftHandDepthToLeftHandColor = "FakeLeftDepthToColor"
    kLeftHandRGBDToArmLEndLink = "FakeLeftRgbdToEnd"
    kRightHandDepthToRightHandColor = "FakeRightDepthToColor"
    kRightHandRGBDToArmREndLink = "FakeRightRgbdToEnd"


class FakeIntrinsic:
    intrinsic = [481.0, 482.0, 640.0, 528.0]
    distortion = [-0.05, -0.01, 0.001, -0.002, 0.003]


class FakeCamera:
    requested: list[Any] = []
    close_called = 0

    def get_camera_intrinsic(self, camera_type: Any) -> FakeIntrinsic:
        self.requested.append(camera_type)
        return FakeIntrinsic()

    def close_camera(self) -> None:
        type(self).close_called += 1


class FakeTranslation:
    x = 1.0
    y = 2.0
    z = 3.0


class FakeRotation:
    x = 0.1
    y = 0.2
    z = 0.3
    w = 0.4


class FakeTransform:
    translation = FakeTranslation()
    rotation = FakeRotation()


class FakeTF:
    def get_tf_from_sensor(self, _sensor_type: Any) -> FakeTransform:
        return FakeTransform()


class FakeAgibotGdk:
    CameraType = FakeCameraType
    SensorExtrinsicType = FakeSensorExtrinsicType
    Camera = FakeCamera
    TF = FakeTF


def fake_import_module(_name: str) -> type[FakeAgibotGdk]:
    return FakeAgibotGdk


def reset_fake_camera() -> None:
    FakeCamera.requested = []
    FakeCamera.close_called = 0


def test_camera_calibration_returns_intrinsic_payload() -> None:
    reset_fake_camera()

    result = run_gdk_camera_calibration_snapshot(
        camera_ids=("hand_left_color",),
        timeout_ms=1500,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is True
    assert result["action"] == "get_camera_calibration"
    assert result["cameraIds"] == ["hand_left_color"]
    [calibration] = result["calibrations"]
    assert calibration["cameraId"] == "hand_left_color"
    assert calibration["gdkCameraType"] == "FakeHandLeftColor"
    assert calibration["fx"] == 481.0
    assert calibration["fy"] == 482.0
    assert calibration["cx"] == 640.0
    assert calibration["cy"] == 528.0
    assert calibration["distortionOrder"] == ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
    assert calibration["distortionCoefficients"]["p1"] == 0.001
    assert FakeCamera.requested == ["FakeHandLeftColor"]
    assert FakeCamera.close_called == 1


def test_camera_calibration_rejects_unsupported_camera_id() -> None:
    result = run_gdk_camera_calibration_snapshot(
        camera_ids=("unsupported",),
        timeout_ms=1500,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is False
    assert result["action"] == "get_camera_calibration"
    assert result["cameraIds"] == ["unsupported"]
    assert result["errorStage"] == "validate_camera_id"


def test_camera_calibration_returns_busy_when_session_active() -> None:
    manager = GdkSessionManager(import_module=fake_import_module)
    lease = manager.acquire(blocking=False, initialize=False, purpose="motion")
    assert lease is not None
    try:
        result = run_gdk_camera_calibration_snapshot(
            camera_ids=("hand_left_color",),
            timeout_ms=1500,
            warmup_seconds=0,
            import_module=fake_import_module,
            session_manager=manager,
        )
    finally:
        lease.release()

    assert result["available"] is False
    assert result["busy"] is True
    assert result["activePurpose"] == "motion"


def test_camera_calibration_include_extrinsics_marks_verified_qr_hand_extrinsic() -> None:
    result = run_gdk_camera_calibration_snapshot(
        camera_ids=("hand_left_color", "hand_right_color"),
        timeout_ms=1500,
        include_extrinsics=True,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is True
    extrinsics = result["extrinsics"]
    assert len(extrinsics) == 4
    verified_types = {
        item["gdkSensorExtrinsicType"]
        for item in extrinsics
        if item["directionVerified"] is True
    }
    assert verified_types == {"FakeLeftRgbdToEnd", "FakeRightRgbdToEnd"}
