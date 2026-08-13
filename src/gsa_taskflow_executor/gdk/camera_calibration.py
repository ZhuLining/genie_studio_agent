"""GDK 相机标定只读快照。

读取相机内参属于建图数据采集前置能力。GDK/DDS 调用仍可能阻塞，因此生产
路径与相机取帧一致：父进程只持有互斥锁，实际 GDK 调用放进可终止子进程。
"""

from __future__ import annotations

import importlib
import time
from collections.abc import Callable, Mapping
from typing import Any

from .camera_frame import (
    DEFAULT_CAMERA_TIMEOUT_MS,
    DEFAULT_CAMERA_WARMUP_SECONDS,
    build_subprocess_timeout_seconds,
    read_camera_type_name,
    resolve_gdk_camera_type,
    should_use_in_process_runtime,
    validate_camera_id,
)
from .control_probe import initialize_gdk, release_gdk, utc_now_iso
from .readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from .session import GdkSessionImportError, GdkSessionInitError, GdkSessionManager
from .subprocess_runtime import run_gdk_subprocess

ACTION_GET_CAMERA_CALIBRATION = "get_camera_calibration"
DEFAULT_CALIBRATION_CAMERA_IDS = ("hand_left_color",)
DISTORTION_ORDER = ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")

SENSOR_EXTRINSIC_ATTRS: dict[str, tuple[tuple[str, str], ...]] = {
    "hand_left_color": (
        ("kLeftHandDepthToLeftHandColor", "左手深度相机到彩色相机"),
        ("kLeftHandRGBDToArmLEndLink", "左手RGBD到左臂末端链接"),
    ),
    "hand_right_color": (
        ("kRightHandDepthToRightHandColor", "右手深度相机到彩色相机"),
        ("kRightHandRGBDToArmREndLink", "右手RGBD到右臂末端链接"),
    ),
    "head_color": (
        ("kHeadDepthToHeadColor", "头部深度相机到彩色相机"),
        ("kHeadRGBDToHeadLink3", "头部RGBD到头部链接3"),
    ),
}


def run_gdk_camera_calibration_snapshot(
    camera_ids: tuple[str, ...] = DEFAULT_CALIBRATION_CAMERA_IDS,
    timeout_ms: int = DEFAULT_CAMERA_TIMEOUT_MS,
    include_extrinsics: bool = False,
    *,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """读取一组相机内参，必要时尝试读取外参探针。"""

    camera_ids = normalize_camera_ids(camera_ids)
    validation_result = validate_camera_ids(camera_ids)
    if validation_result is not None:
        return validation_result

    timeout_result = validate_timeout_ms(timeout_ms)
    if timeout_result is not None:
        return timeout_result

    if should_use_in_process_runtime(import_module, session_manager):
        return run_gdk_camera_calibration_snapshot_in_process(
            camera_ids=camera_ids,
            timeout_ms=timeout_ms,
            include_extrinsics=include_extrinsics,
            warmup_seconds=warmup_seconds,
            import_module=import_module,
            session_manager=session_manager,
        )

    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=False,
            purpose=ACTION_GET_CAMERA_CALIBRATION,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, camera_ids=camera_ids)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose, camera_ids=camera_ids)

    with lease:
        result = run_gdk_subprocess(
            operation="camera_calibration",
            action=ACTION_GET_CAMERA_CALIBRATION,
            backend=GDK_BACKEND,
            timeout_seconds=build_subprocess_timeout_seconds(
                timeout_ms,
                warmup_seconds=warmup_seconds,
            ),
            child_target=camera_calibration_child,
            child_args=(camera_ids, timeout_ms, include_extrinsics, warmup_seconds),
            safety_gate={
                "enabled": False,
                "confirmed": True,
                "reason": "read_only_camera_calibration",
            },
        )
        result["gdk_parent_lock"] = lease.to_payload()
        return result


def run_gdk_camera_calibration_snapshot_in_process(
    *,
    camera_ids: tuple[str, ...],
    timeout_ms: int,
    include_extrinsics: bool,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=True,
            purpose=ACTION_GET_CAMERA_CALIBRATION,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error, camera_ids=camera_ids)
    except GdkSessionInitError as error:
        return unavailable_result(
            "gdk_init",
            RuntimeError(str(error)),
            camera_ids=camera_ids,
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, camera_ids=camera_ids)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose, camera_ids=camera_ids)

    with lease:
        if lease.agibot_gdk is None:
            return unavailable_result(
                "gdk_session_acquire",
                RuntimeError("GDK session lease missing initialized module"),
                camera_ids=camera_ids,
            )

        result = collect_camera_calibration(
            agibot_gdk=lease.agibot_gdk,
            camera_ids=camera_ids,
            timeout_ms=timeout_ms,
            include_extrinsics=include_extrinsics,
            warmup_seconds=warmup_seconds,
        )
        result["gdk_init"] = lease.init_result
        result["gdk_session"] = lease.to_payload()
        return result


def camera_calibration_child(
    result_queue: Any,
    camera_ids: tuple[str, ...],
    timeout_ms: int,
    include_extrinsics: bool,
    warmup_seconds: float,
) -> None:
    agibot_gdk = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    result: dict[str, object]
    try:
        agibot_gdk = importlib.import_module(GDK_MODULE_NAME)
        init_result = initialize_gdk(agibot_gdk)
        if init_result.get("called") is True and init_result.get("success") is not True:
            result = unavailable_result(
                "gdk_init",
                RuntimeError("agibot_gdk.gdk_init() did not return success"),
                camera_ids=camera_ids,
                extra={"gdk_init": init_result},
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            result = collect_camera_calibration(
                agibot_gdk=agibot_gdk,
                camera_ids=camera_ids,
                timeout_ms=timeout_ms,
                include_extrinsics=include_extrinsics,
                warmup_seconds=warmup_seconds,
            )
    except Exception as error:
        result = unavailable_result("import_or_initialize_gdk", error, camera_ids=camera_ids)
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            result["gdk_release"] = release_gdk(agibot_gdk)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        result_queue.put(result)


def collect_camera_calibration(
    *,
    agibot_gdk: Any,
    camera_ids: tuple[str, ...],
    timeout_ms: int,
    include_extrinsics: bool,
    warmup_seconds: float = DEFAULT_CAMERA_WARMUP_SECONDS,
) -> dict[str, object]:
    camera = None
    warnings: list[dict[str, object]] = []
    calibrations: list[dict[str, object]] = []
    try:
        camera = agibot_gdk.Camera()
        if warmup_seconds > 0:
            # GDK 文档建议 Camera 创建后等待初始化，避免首次只读调用误判失败。
            time.sleep(warmup_seconds)

        for camera_id in camera_ids:
            gdk_camera_type = resolve_gdk_camera_type(agibot_gdk, camera_id)
            gdk_camera_type_name = read_camera_type_name(gdk_camera_type)
            intrinsic = camera.get_camera_intrinsic(gdk_camera_type)
            calibrations.append(
                build_camera_intrinsic_payload(
                    intrinsic,
                    camera_id=camera_id,
                    gdk_camera_type=gdk_camera_type_name,
                )
            )

        result: dict[str, object] = {
            "available": True,
            "backend": GDK_BACKEND,
            "action": ACTION_GET_CAMERA_CALIBRATION,
            "cameraIds": list(camera_ids),
            "timeoutMs": timeout_ms,
            "calibrations": calibrations,
            "collectedAt": utc_now_iso(),
        }

        if include_extrinsics:
            extrinsics, extrinsic_warnings = collect_camera_extrinsics(
                agibot_gdk,
                camera_ids=camera_ids,
            )
            result["extrinsics"] = extrinsics
            warnings.extend(extrinsic_warnings)

        if warnings:
            result["warnings"] = warnings
        return result
    except Exception as error:
        return unavailable_result("get_camera_intrinsic", error, camera_ids=camera_ids)
    finally:
        if camera is not None:
            close_camera = getattr(camera, "close_camera", None)
            if callable(close_camera):
                close_camera()


def collect_camera_extrinsics(
    agibot_gdk: Any,
    *,
    camera_ids: tuple[str, ...],
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    extrinsics: list[dict[str, object]] = []
    warnings: list[dict[str, object]] = []
    tf_class = getattr(agibot_gdk, "TF", None)
    sensor_type = getattr(agibot_gdk, "SensorExtrinsicType", None)
    if tf_class is None or sensor_type is None:
        return extrinsics, [
            {
                "stage": "get_tf_from_sensor",
                "message": "agibot_gdk.TF or SensorExtrinsicType is unavailable",
            }
        ]

    tf = None
    try:
        tf = tf_class()
        for camera_id in camera_ids:
            for attr, description in SENSOR_EXTRINSIC_ATTRS.get(camera_id, ()):
                if not hasattr(sensor_type, attr):
                    warnings.append(
                        {
                            "cameraId": camera_id,
                            "sensorExtrinsicType": attr,
                            "stage": "resolve_sensor_extrinsic_type",
                            "message": f"SensorExtrinsicType missing {attr}",
                        }
                    )
                    continue
                gdk_sensor_type = getattr(sensor_type, attr)
                try:
                    transform = tf.get_tf_from_sensor(gdk_sensor_type)
                    extrinsics.append(
                        build_extrinsic_payload(
                            transform,
                            camera_id=camera_id,
                            gdk_sensor_type=read_camera_type_name(gdk_sensor_type),
                            description=description,
                        )
                    )
                except Exception as error:
                    warnings.append(
                        {
                            "cameraId": camera_id,
                            "sensorExtrinsicType": attr,
                            "stage": "get_tf_from_sensor",
                            "errorType": type(error).__name__,
                            "message": str(error),
                        }
                    )
    except Exception as error:
        warnings.append(
            {
                "stage": "create_tf",
                "errorType": type(error).__name__,
                "message": str(error),
            }
        )
    return extrinsics, warnings


def build_camera_intrinsic_payload(
    intrinsic: Any,
    *,
    camera_id: str,
    gdk_camera_type: str,
) -> dict[str, object]:
    values = read_float_list(getattr(intrinsic, "intrinsic", None), "intrinsic")
    if len(values) < 4:
        raise ValueError(f"camera intrinsic must contain fx/fy/cx/cy, got {len(values)}")
    distortion = read_float_list(getattr(intrinsic, "distortion", None), "distortion")
    coefficients = {
        name: distortion[index]
        for index, name in enumerate(DISTORTION_ORDER)
        if index < len(distortion)
    }
    return {
        "cameraId": camera_id,
        "gdkCameraType": gdk_camera_type,
        "fx": values[0],
        "fy": values[1],
        "cx": values[2],
        "cy": values[3],
        "intrinsic": values,
        "distortion": distortion,
        "distortionOrder": list(DISTORTION_ORDER),
        "distortionCoefficients": coefficients,
        "collectedAt": utc_now_iso(),
        "raw": {
            "intrinsic": to_jsonable(getattr(intrinsic, "intrinsic", None)),
            "distortion": to_jsonable(getattr(intrinsic, "distortion", None)),
        },
    }


def build_extrinsic_payload(
    transform: Any,
    *,
    camera_id: str,
    gdk_sensor_type: str,
    description: str,
) -> dict[str, object]:
    translation = getattr(transform, "translation", None)
    rotation = getattr(transform, "rotation", None)
    return {
        "cameraId": camera_id,
        "gdkSensorExtrinsicType": gdk_sensor_type,
        "description": description,
        "translation": {
            "x": read_float_attr(translation, "x"),
            "y": read_float_attr(translation, "y"),
            "z": read_float_attr(translation, "z"),
        },
        "rotation": {
            "x": read_float_attr(rotation, "x"),
            "y": read_float_attr(rotation, "y"),
            "z": read_float_attr(rotation, "z"),
            "w": read_float_attr(rotation, "w"),
        },
        "directionVerified": False,
        "collectedAt": utc_now_iso(),
    }


def normalize_camera_ids(camera_ids: tuple[str, ...]) -> tuple[str, ...]:
    normalized: list[str] = []
    for camera_id in camera_ids:
        if camera_id and camera_id not in normalized:
            normalized.append(camera_id)
    return tuple(normalized) or DEFAULT_CALIBRATION_CAMERA_IDS


def validate_camera_ids(camera_ids: tuple[str, ...]) -> dict[str, object] | None:
    for camera_id in camera_ids:
        result = validate_camera_id(camera_id)
        if result is not None:
            result["action"] = ACTION_GET_CAMERA_CALIBRATION
            result["cameraIds"] = list(camera_ids)
            return result
    return None


def validate_timeout_ms(timeout_ms: int) -> dict[str, object] | None:
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
        return unavailable_result(
            "validate_timeout_ms",
            ValueError("timeout_ms must be a positive integer"),
            camera_ids=(),
        )
    return None


def read_float_list(value: Any, label: str) -> list[float]:
    if value is None:
        raise ValueError(f"camera {label} is missing")
    try:
        return [float(item) for item in value]
    except TypeError as error:
        raise TypeError(f"camera {label} is not iterable") from error
    except ValueError as error:
        raise ValueError(f"camera {label} contains non-numeric value") from error


def read_float_attr(value: Any, attr: str) -> float | None:
    raw = getattr(value, attr, None)
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def busy_result(
    *,
    active_purpose: str | None,
    camera_ids: tuple[str, ...],
) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_BACKEND,
        "action": ACTION_GET_CAMERA_CALIBRATION,
        "cameraIds": list(camera_ids),
        "collectedAt": utc_now_iso(),
        "busy": True,
        "errorStage": "gdk_session_busy",
        "errorType": "GdkSessionBusy",
        "errorMsg": "GDK 正在执行控制动作，相机标定读取已拒绝",
        "activePurpose": active_purpose,
    }


def unavailable_result(
    stage: str,
    error: Exception,
    *,
    camera_ids: tuple[str, ...],
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": GDK_BACKEND,
        "action": ACTION_GET_CAMERA_CALIBRATION,
        "cameraIds": list(camera_ids),
        "collectedAt": utc_now_iso(),
        "errorStage": stage,
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }
    if extra:
        result.update(dict(extra))
    return result
