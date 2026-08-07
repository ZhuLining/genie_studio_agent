from __future__ import annotations

import base64
import importlib
import json
import os
import tempfile
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any
from uuid import uuid4

from .control_probe import initialize_gdk, release_gdk, utc_now_iso
from .readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from .session import GdkSessionImportError, GdkSessionInitError, GdkSessionManager
from .subprocess_runtime import run_gdk_subprocess

ACTION_GET_CAMERA_FRAME = "get_camera_frame"
DEFAULT_CAMERA_TIMEOUT_MS = 3000
DEFAULT_CAMERA_WARMUP_SECONDS = 3.0
SUBPROCESS_TIMEOUT_MARGIN_SECONDS = 6.0
CAMERA_ENCODING_UNSUPPORTED = "CAMERA_ENCODING_UNSUPPORTED"

CAMERA_TYPE_ATTRS: dict[str, tuple[str, ...]] = {
    "hand_left_color": ("kHandLeftColor",),
    "hand_left_upper_color": ("kHandLeftColor",),
    "left_hand_color": ("kHandLeftColor",),
    "hand_right_color": ("kHandRightColor",),
    "hand_right_upper_color": ("kHandRightColor",),
    "right_hand_color": ("kHandRightColor",),
    "head_color": ("kHeadColor",),
}


def run_gdk_camera_frame_snapshot(
    camera_id: str = "hand_left_color",
    timeout_ms: int = DEFAULT_CAMERA_TIMEOUT_MS,
    *,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """读取一帧相机图像。

    相机图像是只读能力，但 GDK/DDS 同步调用仍可能阻塞；生产路径只在父进程拿
    非阻塞互斥锁，真正取帧放到可杀掉的子进程，避免卡住 MQTT 请求 worker。
    """

    camera_id_result = validate_camera_id(camera_id)
    if camera_id_result is not None:
        return camera_id_result

    timeout_result = validate_timeout_ms(timeout_ms)
    if timeout_result is not None:
        return timeout_result

    if should_use_in_process_runtime(import_module, session_manager):
        return run_gdk_camera_frame_snapshot_in_process(
            camera_id=camera_id,
            timeout_ms=timeout_ms,
            warmup_seconds=warmup_seconds,
            import_module=import_module,
            session_manager=session_manager,
        )

    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=False,
            purpose=ACTION_GET_CAMERA_FRAME,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, camera_id=camera_id)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose, camera_id=camera_id)

    progress_path = build_camera_child_artifact_path("progress")
    result_path = build_camera_child_artifact_path("result")
    with lease:
        try:
            result = run_gdk_subprocess(
                operation="camera_frame",
                action=ACTION_GET_CAMERA_FRAME,
                backend=GDK_BACKEND,
                timeout_seconds=build_subprocess_timeout_seconds(
                    timeout_ms,
                    warmup_seconds=warmup_seconds,
                ),
                child_target=camera_frame_child,
                child_args=(
                    camera_id,
                    timeout_ms,
                    warmup_seconds,
                    str(progress_path),
                    str(result_path),
                ),
                safety_gate={
                    "enabled": False,
                    "confirmed": True,
                    "reason": "read_only_camera_frame",
                },
            )
            result = load_camera_child_result(result, result_path, camera_id=camera_id)
            attach_camera_child_progress(result, progress_path)
            result["gdk_parent_lock"] = lease.to_payload()
            return result
        finally:
            remove_camera_child_artifact(progress_path)
            remove_camera_child_artifact(result_path)


def run_gdk_camera_frame_snapshot_in_process(
    *,
    camera_id: str,
    timeout_ms: int,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=True,
            purpose=ACTION_GET_CAMERA_FRAME,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error, camera_id=camera_id)
    except GdkSessionInitError as error:
        return unavailable_result(
            "gdk_init",
            RuntimeError(str(error)),
            camera_id=camera_id,
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, camera_id=camera_id)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose, camera_id=camera_id)

    with lease:
        if lease.agibot_gdk is None:
            return unavailable_result(
                "gdk_session_acquire",
                RuntimeError("GDK session lease missing initialized module"),
                camera_id=camera_id,
            )

        result = collect_camera_frame(
            agibot_gdk=lease.agibot_gdk,
            camera_id=camera_id,
            timeout_ms=timeout_ms,
            warmup_seconds=warmup_seconds,
        )
        result["gdk_init"] = lease.init_result
        result["gdk_session"] = lease.to_payload()
        return result


def camera_frame_child(
    result_queue: Any,
    camera_id: str,
    timeout_ms: int,
    warmup_seconds: float,
    progress_path: str | None = None,
    result_path: str | None = None,
) -> None:
    agibot_gdk = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    result: dict[str, object]
    try:
        write_camera_child_progress(progress_path, "child_started", camera_id=camera_id)
        agibot_gdk = importlib.import_module(GDK_MODULE_NAME)
        write_camera_child_progress(progress_path, "agibot_gdk_imported", camera_id=camera_id)
        init_result = initialize_gdk(agibot_gdk)
        write_camera_child_progress(
            progress_path,
            "gdk_initialized",
            camera_id=camera_id,
            gdk_init=init_result,
        )
        if init_result.get("called") is True and init_result.get("success") is not True:
            result = unavailable_result(
                "gdk_init",
                RuntimeError("agibot_gdk.gdk_init() did not return success"),
                camera_id=camera_id,
                extra={"gdk_init": init_result},
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            result = collect_camera_frame(
                agibot_gdk=agibot_gdk,
                camera_id=camera_id,
                timeout_ms=timeout_ms,
                warmup_seconds=warmup_seconds,
                progress_path=progress_path,
            )
    except Exception as error:
        result = unavailable_result("import_or_initialize_gdk", error, camera_id=camera_id)
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            write_camera_child_progress(progress_path, "gdk_release_started", camera_id=camera_id)
            result["gdk_release"] = release_gdk(agibot_gdk)
            write_camera_child_progress(progress_path, "gdk_release_finished", camera_id=camera_id)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        attach_camera_child_progress(result, Path(progress_path) if progress_path else None)
        queue_payload = write_camera_child_result_reference(
            result,
            result_path=result_path,
            camera_id=camera_id,
            progress_path=progress_path,
        )
        write_camera_child_progress(progress_path, "result_put_started", camera_id=camera_id)
        result_queue.put(queue_payload)
        write_camera_child_progress(progress_path, "result_put_finished", camera_id=camera_id)


def collect_camera_frame(
    *,
    agibot_gdk: Any,
    camera_id: str,
    timeout_ms: int,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
    progress_path: str | None = None,
) -> dict[str, object]:
    camera = None
    try:
        gdk_camera_type = resolve_gdk_camera_type(agibot_gdk, camera_id)
        gdk_camera_type_name = read_camera_type_name(gdk_camera_type)
        write_camera_child_progress(
            progress_path,
            "camera_type_resolved",
            camera_id=camera_id,
            gdk_camera_type=gdk_camera_type_name,
        )
        camera = agibot_gdk.Camera()
        write_camera_child_progress(progress_path, "camera_created", camera_id=camera_id)
        if warmup_seconds > 0:
            # 真机 GDK 示例明确要求 Camera() 后等待初始化；否则首帧常返回失败。
            write_camera_child_progress(
                progress_path,
                "camera_warmup_started",
                camera_id=camera_id,
                warmup_seconds=warmup_seconds,
            )
            time.sleep(warmup_seconds)
            write_camera_child_progress(
                progress_path,
                "camera_warmup_finished",
                camera_id=camera_id,
            )
        write_camera_child_progress(
            progress_path,
            "get_latest_image_started",
            camera_id=camera_id,
            timeout_ms=timeout_ms,
        )
        image = camera.get_latest_image(gdk_camera_type, float(timeout_ms))
        write_camera_child_progress(progress_path, "get_latest_image_returned", camera_id=camera_id)
        if image is None:
            return unavailable_result(
                "get_latest_image",
                RuntimeError("camera.get_latest_image() returned None"),
                camera_id=camera_id,
                extra={
                    "gdkCameraType": gdk_camera_type_name,
                    "cameraWarmupSeconds": warmup_seconds,
                },
            )
        result = build_camera_frame_snapshot(
            image,
            camera_id=camera_id,
            gdk_camera_type=gdk_camera_type_name,
        )
        result["cameraWarmupSeconds"] = warmup_seconds
        write_camera_child_progress(progress_path, "frame_encoded", camera_id=camera_id)
        return result
    except Exception as error:
        return unavailable_result(
            "get_latest_image",
            error,
            camera_id=camera_id,
            extra={"cameraWarmupSeconds": warmup_seconds},
        )
    finally:
        if camera is not None:
            close_camera = getattr(camera, "close_camera", None)
            if callable(close_camera):
                write_camera_child_progress(
                    progress_path,
                    "close_camera_started",
                    camera_id=camera_id,
                )
                close_camera()
                write_camera_child_progress(
                    progress_path,
                    "close_camera_finished",
                    camera_id=camera_id,
                )


def build_camera_frame_snapshot(
    image: Any,
    *,
    camera_id: str,
    gdk_camera_type: str,
) -> dict[str, object]:
    width = read_positive_int(getattr(image, "width", None), "image.width")
    height = read_positive_int(getattr(image, "height", None), "image.height")
    encoding = normalize_encoding(getattr(image, "encoding", ""))
    data = image_data_to_bytes(getattr(image, "data", b""))
    encoded = encode_image_for_display(
        data,
        width=width,
        height=height,
        encoding=encoding,
    )
    if encoded is None:
        return unsupported_encoding_result(
            camera_id=camera_id,
            gdk_camera_type=gdk_camera_type,
            width=width,
            height=height,
            encoding=encoding,
            data_bytes=len(data),
        )

    mime_type, output_bytes = encoded
    return {
        "available": True,
        "backend": GDK_BACKEND,
        "action": ACTION_GET_CAMERA_FRAME,
        "cameraId": camera_id,
        "gdkCameraType": gdk_camera_type,
        "mimeType": mime_type,
        "imageBase64": base64.b64encode(output_bytes).decode("ascii"),
        "width": width,
        "height": height,
        "encoding": encoding,
        "timestampNs": to_jsonable(getattr(image, "timestamp_ns", None)),
        "collectedAt": utc_now_iso(),
        "raw": {
            "dataBytes": len(data),
        },
    }


def encode_image_for_display(
    data: bytes,
    *,
    width: int,
    height: int,
    encoding: str,
) -> tuple[str, bytes] | None:
    if encoding in {"JPEG", "JPG", "MJPEG"}:
        return "image/jpeg", data
    if encoding == "PNG":
        return "image/png", data
    if encoding in {"RGB", "BGR", "RGBA", "BGRA", "GRAY8"}:
        return "image/bmp", build_bmp_bytes(data, width=width, height=height, encoding=encoding)
    return None


def build_bmp_bytes(data: bytes, *, width: int, height: int, encoding: str) -> bytes:
    channels = {"RGB": 3, "BGR": 3, "RGBA": 4, "BGRA": 4, "GRAY8": 1}[encoding]
    expected = width * height * channels
    if len(data) < expected:
        raise ValueError(
            f"image.data too short for {encoding} {width}x{height}: {len(data)} < {expected}"
        )

    row_stride = width * 3
    padding = (4 - row_stride % 4) % 4
    pixel_bytes = bytearray()
    for y in range(height - 1, -1, -1):
        row_start = y * width * channels
        for x in range(width):
            offset = row_start + x * channels
            if encoding == "RGB":
                r, g, b = data[offset : offset + 3]
            elif encoding == "BGR":
                b, g, r = data[offset : offset + 3]
            elif encoding == "RGBA":
                r, g, b = data[offset : offset + 3]
            elif encoding == "BGRA":
                b, g, r = data[offset : offset + 3]
            else:
                gray = data[offset]
                r = g = b = gray
            pixel_bytes.extend((b, g, r))
        pixel_bytes.extend(b"\x00" * padding)

    header_size = 54
    file_size = header_size + len(pixel_bytes)
    header = bytearray()
    header.extend(b"BM")
    header.extend(file_size.to_bytes(4, "little"))
    header.extend((0).to_bytes(4, "little"))
    header.extend(header_size.to_bytes(4, "little"))
    header.extend((40).to_bytes(4, "little"))
    header.extend(width.to_bytes(4, "little", signed=True))
    header.extend(height.to_bytes(4, "little", signed=True))
    header.extend((1).to_bytes(2, "little"))
    header.extend((24).to_bytes(2, "little"))
    header.extend((0).to_bytes(4, "little"))
    header.extend(len(pixel_bytes).to_bytes(4, "little"))
    header.extend((2835).to_bytes(4, "little", signed=True))
    header.extend((2835).to_bytes(4, "little", signed=True))
    header.extend((0).to_bytes(4, "little"))
    header.extend((0).to_bytes(4, "little"))
    return bytes(header) + bytes(pixel_bytes)


def resolve_gdk_camera_type(agibot_gdk: Any, camera_id: str) -> Any:
    attrs = CAMERA_TYPE_ATTRS.get(camera_id)
    if attrs is None:
        raise ValueError(f"unsupported camera_id: {camera_id}")
    camera_type = getattr(agibot_gdk, "CameraType", None)
    for attr in attrs:
        if camera_type is not None and hasattr(camera_type, attr):
            return getattr(camera_type, attr)
        if hasattr(agibot_gdk, attr):
            return getattr(agibot_gdk, attr)
    raise AttributeError(f"agibot_gdk.CameraType missing any of {attrs!r}")


def validate_camera_id(camera_id: str) -> dict[str, object] | None:
    if camera_id not in CAMERA_TYPE_ATTRS:
        return unavailable_result(
            "validate_camera_id",
            ValueError(f"unsupported camera_id: {camera_id}"),
            camera_id=camera_id,
            extra={"supportedCameraIds": sorted(CAMERA_TYPE_ATTRS)},
        )
    return None


def validate_timeout_ms(timeout_ms: int) -> dict[str, object] | None:
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
        return unavailable_result(
            "validate_timeout_ms",
            ValueError("timeout_ms must be a positive integer"),
            camera_id="",
        )
    return None


def build_subprocess_timeout_seconds(
    timeout_ms: int,
    *,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
) -> float:
    # 子进程总超时覆盖：GDK 初始化 + Camera warmup + get_latest_image 自身超时 + 兜底余量。
    # 否则 warmup 还没结束，父进程就会误杀只读取帧子进程。
    return max(
        1.0,
        max(0.0, warmup_seconds) + timeout_ms / 1000.0 + SUBPROCESS_TIMEOUT_MARGIN_SECONDS,
    )


def build_camera_child_artifact_path(kind: str) -> Path:
    return Path(tempfile.gettempdir()) / f"gsa_camera_frame_{kind}_{uuid4().hex}.json"


def write_camera_child_result_reference(
    result: Mapping[str, object],
    *,
    result_path: str | None,
    camera_id: str,
    progress_path: str | None = None,
) -> dict[str, object]:
    if not result_path:
        return dict(result)

    try:
        encoded = json.dumps(to_jsonable(dict(result)), ensure_ascii=False)
        Path(result_path).write_text(encoded, encoding="utf-8")
    except Exception as error:
        return unavailable_result("write_camera_child_result", error, camera_id=camera_id)

    byte_count = len(encoded.encode("utf-8"))
    write_camera_child_progress(
        progress_path,
        "result_file_written",
        camera_id=camera_id,
        result_bytes=byte_count,
    )
    return {
        "cameraResultFile": result_path,
        "cameraResultBytes": byte_count,
    }


def load_camera_child_result(
    result: Mapping[str, object],
    result_path: Path,
    *,
    camera_id: str,
) -> dict[str, object]:
    loaded = read_camera_child_result(result, result_path)
    if loaded is None:
        return dict(result)

    for key in ("subprocess", "cameraResultBytes"):
        if key in result:
            loaded[key] = result[key]
    return loaded


def read_camera_child_result(
    result: Mapping[str, object],
    result_path: Path,
) -> dict[str, object] | None:
    raw_path = result.get("cameraResultFile")
    path = Path(raw_path) if isinstance(raw_path, str) and raw_path else result_path
    if not path.exists():
        return None

    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return decoded if isinstance(decoded, dict) else None


def write_camera_child_progress(
    progress_path: str | None,
    stage: str,
    **payload: object,
) -> None:
    if not progress_path:
        return
    progress = {
        "stage": stage,
        "pid": os.getpid(),
        "updatedAt": utc_now_iso(),
        **payload,
    }
    try:
        Path(progress_path).write_text(
            json.dumps(to_jsonable(progress), ensure_ascii=False),
            encoding="utf-8",
        )
    except Exception:
        # 进度文件只用于现场诊断，不能影响相机只读主流程。
        return


def attach_camera_child_progress(
    result: dict[str, object],
    progress_path: Path | None,
) -> None:
    progress = read_camera_child_progress(progress_path)
    if progress is not None:
        result["cameraChildProgress"] = progress


def read_camera_child_progress(progress_path: Path | None) -> dict[str, object] | None:
    if progress_path is None or not progress_path.exists():
        return None
    try:
        decoded = json.loads(progress_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return decoded if isinstance(decoded, dict) else None


def remove_camera_child_artifact(path: Path) -> None:
    try:
        path.unlink(missing_ok=True)
    except Exception:
        return


def should_use_in_process_runtime(
    import_module: Callable[[str], Any],
    session_manager: GdkSessionManager | None,
) -> bool:
    if import_module is not importlib.import_module:
        return True
    return (
        session_manager is not None
        and session_manager.import_module is not importlib.import_module
    )


def read_positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def normalize_encoding(value: Any) -> str:
    text = str(value or "").strip()
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text.upper()


def image_data_to_bytes(value: Any) -> bytes:
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, memoryview):
        return value.tobytes()
    tobytes = getattr(value, "tobytes", None)
    if callable(tobytes):
        raw = tobytes()
        if isinstance(raw, bytes):
            return raw
    try:
        return bytes(value)
    except Exception as error:
        raise TypeError("image.data cannot be converted to bytes") from error


def read_camera_type_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str) and name:
        return name
    return str(value)


def busy_result(*, active_purpose: str | None, camera_id: str) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_BACKEND,
        "action": ACTION_GET_CAMERA_FRAME,
        "cameraId": camera_id,
        "collectedAt": utc_now_iso(),
        "busy": True,
        "errorStage": "gdk_session_busy",
        "errorType": "GdkSessionBusy",
        "errorMsg": "GDK 正在执行控制动作，相机图像读取已拒绝",
        "activePurpose": active_purpose,
    }


def unsupported_encoding_result(
    *,
    camera_id: str,
    gdk_camera_type: str,
    width: int,
    height: int,
    encoding: str,
    data_bytes: int,
) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_BACKEND,
        "action": ACTION_GET_CAMERA_FRAME,
        "cameraId": camera_id,
        "gdkCameraType": gdk_camera_type,
        "collectedAt": utc_now_iso(),
        "errorStage": "encode_image_for_display",
        "errorCode": CAMERA_ENCODING_UNSUPPORTED,
        "errorType": "UnsupportedCameraEncoding",
        "errorMsg": f"首版暂不支持相机图像编码: {encoding}",
        "width": width,
        "height": height,
        "encoding": encoding,
        "raw": {"dataBytes": data_bytes},
    }


def unavailable_result(
    stage: str,
    error: Exception,
    *,
    camera_id: str,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": GDK_BACKEND,
        "action": ACTION_GET_CAMERA_FRAME,
        "cameraId": camera_id,
        "collectedAt": utc_now_iso(),
        "errorStage": stage,
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }
    if extra:
        result.update(dict(extra))
    return result
