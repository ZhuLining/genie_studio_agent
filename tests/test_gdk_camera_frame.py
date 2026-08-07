from __future__ import annotations

import base64
from typing import Any

from gsa_taskflow_executor.gdk.camera_frame import (
    CAMERA_ENCODING_UNSUPPORTED,
    build_bmp_bytes,
    build_subprocess_timeout_seconds,
    resolve_gdk_camera_type,
    run_gdk_camera_frame_snapshot,
)
from gsa_taskflow_executor.gdk.session import GdkSessionManager


class FakeCameraType:
    kHandLeftColor = "FakeHandLeftColor"
    kHandRightColor = "FakeHandRightColor"
    kHeadColor = "FakeHeadColor"


class FakeImage:
    def __init__(
        self,
        *,
        width: int = 2,
        height: int = 1,
        encoding: str = "JPEG",
        data: Any = b"\xff\xd8jpeg",
    ) -> None:
        self.width = width
        self.height = height
        self.encoding = encoding
        self.data = data
        self.timestamp_ns = 123


class FakeArrayData:
    def __init__(self, data: bytes) -> None:
        self.data = data

    def __bool__(self) -> bool:
        raise ValueError("array truth value is ambiguous")

    def tobytes(self) -> bytes:
        return self.data


class FakeCamera:
    image = FakeImage()
    requested: list[tuple[Any, float]] = []
    close_called = 0

    def get_latest_image(self, camera_type: Any, timeout: float) -> FakeImage:
        self.requested.append((camera_type, timeout))
        return self.image

    def close_camera(self) -> None:
        type(self).close_called += 1


class FakeAgibotGdk:
    CameraType = FakeCameraType
    Camera = FakeCamera


def fake_import_module(_name: str) -> type[FakeAgibotGdk]:
    return FakeAgibotGdk


def reset_fake_camera(image: FakeImage) -> None:
    FakeCamera.image = image
    FakeCamera.requested = []
    FakeCamera.close_called = 0


def test_resolve_gdk_camera_type_maps_left_hand_alias() -> None:
    assert resolve_gdk_camera_type(FakeAgibotGdk, "hand_left_color") == "FakeHandLeftColor"
    assert resolve_gdk_camera_type(FakeAgibotGdk, "hand_left_upper_color") == "FakeHandLeftColor"


def test_subprocess_timeout_includes_camera_warmup() -> None:
    assert build_subprocess_timeout_seconds(3000, warmup_seconds=3.0) == 12.0


def test_gdk_camera_frame_snapshot_returns_jpeg_base64() -> None:
    reset_fake_camera(FakeImage(data=b"\xff\xd8hello"))

    result = run_gdk_camera_frame_snapshot(
        camera_id="hand_left_color",
        timeout_ms=1500,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is True
    assert result["cameraId"] == "hand_left_color"
    assert result["gdkCameraType"] == "FakeHandLeftColor"
    assert result["mimeType"] == "image/jpeg"
    assert result["imageBase64"] == base64.b64encode(b"\xff\xd8hello").decode("ascii")
    assert result["cameraWarmupSeconds"] == 0
    assert FakeCamera.requested == [("FakeHandLeftColor", 1500.0)]
    assert FakeCamera.close_called == 1


def test_gdk_camera_frame_snapshot_converts_rgb_to_bmp() -> None:
    reset_fake_camera(
        FakeImage(
            width=1,
            height=1,
            encoding="RGB",
            data=bytes([10, 20, 30]),
        )
    )

    result = run_gdk_camera_frame_snapshot(
        camera_id="head_color",
        timeout_ms=1000,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is True
    assert result["mimeType"] == "image/bmp"
    assert base64.b64decode(str(result["imageBase64"])) == build_bmp_bytes(
        bytes([10, 20, 30]),
        width=1,
        height=1,
        encoding="RGB",
    )


def test_gdk_camera_frame_snapshot_rejects_unsupported_encoding() -> None:
    reset_fake_camera(FakeImage(encoding="YUV420", data=b"12345678"))

    result = run_gdk_camera_frame_snapshot(
        camera_id="hand_left_color",
        timeout_ms=1500,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is False
    assert result["errorCode"] == CAMERA_ENCODING_UNSUPPORTED
    assert result["encoding"] == "YUV420"


def test_gdk_camera_frame_snapshot_returns_busy_when_session_active() -> None:
    manager = GdkSessionManager(import_module=fake_import_module)
    lease = manager.acquire(blocking=False, initialize=False, purpose="motion")
    assert lease is not None
    try:
        result = run_gdk_camera_frame_snapshot(
            camera_id="hand_left_color",
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


def test_gdk_camera_frame_snapshot_accepts_array_like_image_data() -> None:
    reset_fake_camera(FakeImage(data=FakeArrayData(b"\xff\xd8array")))

    result = run_gdk_camera_frame_snapshot(
        camera_id="hand_left_color",
        timeout_ms=1500,
        warmup_seconds=0,
        import_module=fake_import_module,
    )

    assert result["available"] is True
    assert result["mimeType"] == "image/jpeg"
    assert result["imageBase64"] == base64.b64encode(b"\xff\xd8array").decode("ascii")
