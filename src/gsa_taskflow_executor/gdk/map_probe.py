"""GDK 高精度地图只读探针。

用于现场验证 ``agibot_gdk.Map.get_all_map()`` 与 ``get_map(map_id)``。
这里只读取地图列表和地图详情摘要，不切换地图、不发导航或控制指令。
"""

from __future__ import annotations

import importlib
from collections.abc import Callable, Iterable, Mapping, Sequence
from itertools import islice
from typing import Any

from .camera_frame import should_use_in_process_runtime
from .control_probe import initialize_gdk, release_gdk, utc_now_iso
from .readonly import GDK_MODULE_NAME, to_jsonable
from .session import GdkSessionImportError, GdkSessionInitError, GdkSessionManager
from .subprocess_runtime import run_gdk_subprocess

ACTION_GET_HIGH_PRECISION_MAPS = "get_high_precision_maps"
GDK_MAP_BACKEND = "agibot_gdk.Map"
DEFAULT_MAP_TIMEOUT_MS = 5000
SUBPROCESS_TIMEOUT_MARGIN_SECONDS = 3.0
GRID_DATA_SAMPLE_SIZE = 64
POINT_SAMPLE_SIZE = 50


def run_gdk_map_probe(
    map_id: int | None = None,
    timeout_ms: int = DEFAULT_MAP_TIMEOUT_MS,
    *,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """读取高精度地图列表，并按 map_id/当前地图/首个地图读取详情摘要。

    地图详情可能包含大栅格或点云数据，MQTT/UI 路径只返回摘要和小样本，避免把
    DDS 大对象直接塞进状态通道。真机排障需要完整字段时，可在现场扩展本探针。
    """

    timeout_result = validate_timeout_ms(timeout_ms)
    if timeout_result is not None:
        return timeout_result

    map_id_result = validate_map_id(map_id)
    if map_id_result is not None:
        return map_id_result

    if should_use_in_process_runtime(import_module, session_manager):
        return run_gdk_map_probe_in_process(
            map_id=map_id,
            timeout_ms=timeout_ms,
            import_module=import_module,
            session_manager=session_manager,
        )

    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=False,
            purpose=ACTION_GET_HIGH_PRECISION_MAPS,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, map_id=map_id)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose, map_id=map_id)

    with lease:
        result = run_gdk_subprocess(
            operation="high_precision_map_probe",
            action=ACTION_GET_HIGH_PRECISION_MAPS,
            backend=GDK_MAP_BACKEND,
            timeout_seconds=build_subprocess_timeout_seconds(timeout_ms),
            child_target=map_probe_child,
            child_args=(map_id, timeout_ms),
            safety_gate={
                "enabled": False,
                "confirmed": True,
                "reason": "read_only_high_precision_map",
            },
        )
        result["gdk_parent_lock"] = lease.to_payload()
        return result


def run_gdk_map_probe_in_process(
    *,
    map_id: int | None,
    timeout_ms: int,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=True,
            purpose=ACTION_GET_HIGH_PRECISION_MAPS,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error, map_id=map_id)
    except GdkSessionInitError as error:
        return unavailable_result(
            "gdk_init",
            RuntimeError(str(error)),
            map_id=map_id,
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error, map_id=map_id)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose, map_id=map_id)

    with lease:
        if lease.agibot_gdk is None:
            return unavailable_result(
                "gdk_session_acquire",
                RuntimeError("GDK session lease missing initialized module"),
                map_id=map_id,
            )
        result = collect_high_precision_maps(
            agibot_gdk=lease.agibot_gdk,
            requested_map_id=map_id,
            timeout_ms=timeout_ms,
        )
        result["gdk_init"] = lease.init_result
        result["gdk_session"] = lease.to_payload()
        return result


def map_probe_child(result_queue: Any, map_id: int | None, timeout_ms: int) -> None:
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
                map_id=map_id,
                extra={"gdk_init": init_result},
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            result = collect_high_precision_maps(
                agibot_gdk=agibot_gdk,
                requested_map_id=map_id,
                timeout_ms=timeout_ms,
            )
    except Exception as error:
        result = unavailable_result("import_or_initialize_gdk", error, map_id=map_id)
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            result["gdk_release"] = release_gdk(agibot_gdk)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        result_queue.put(result)


def collect_high_precision_maps(
    *,
    agibot_gdk: Any,
    requested_map_id: int | None,
    timeout_ms: int,
) -> dict[str, object]:
    try:
        map_factory = agibot_gdk.Map
    except AttributeError as error:
        return unavailable_result("get_map_factory", error, map_id=requested_map_id)

    try:
        map_manager = map_factory()
    except Exception as error:
        return unavailable_result("create_map_manager", error, map_id=requested_map_id)

    try:
        all_maps_raw = map_manager.get_all_map()
    except Exception as error:
        return unavailable_result("get_all_map", error, map_id=requested_map_id)

    maps = normalize_map_list(all_maps_raw)
    selected_map_id = choose_map_id(requested_map_id, maps)
    selected_map_detail: dict[str, object] | None = None
    detail_error: dict[str, object] | None = None
    if selected_map_id is not None:
        try:
            selected_map_detail = summarize_map_info(map_manager.get_map(selected_map_id))
        except Exception as error:
            detail_error = {
                "stage": "get_map",
                "mapId": selected_map_id,
                "errorType": type(error).__name__,
                "errorMsg": str(error),
            }

    result: dict[str, object] = {
        "available": detail_error is None,
        "backend": GDK_MAP_BACKEND,
        "action": ACTION_GET_HIGH_PRECISION_MAPS,
        "timeoutMs": timeout_ms,
        "requestedMapId": requested_map_id,
        "selectedMapId": selected_map_id,
        "mapCount": len(maps),
        "maps": maps,
        "mapDetail": selected_map_detail,
        "collectedAt": utc_now_iso(),
    }
    if detail_error is not None:
        result.update(
            {
                "errorStage": detail_error["stage"],
                "errorType": detail_error["errorType"],
                "errorMsg": detail_error["errorMsg"],
                "detailError": detail_error,
            }
        )
    return result


def normalize_map_list(value: Any) -> list[dict[str, object]]:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return []
    return [summarize_map_name(item) for item in value]


def summarize_map_name(value: Any) -> dict[str, object]:
    map_id = read_int_attr(value, "id")
    name = read_string_attr(value, "name")
    is_curr_map = read_bool_attr(value, "is_curr_map")
    return {
        "id": map_id,
        "name": name if name is not None else "",
        "isCurrMap": is_curr_map is True,
        "raw": compact_object(value),
    }


def choose_map_id(requested_map_id: int | None, maps: Sequence[Mapping[str, object]]) -> int | None:
    if requested_map_id is not None:
        return requested_map_id
    for item in maps:
        if item.get("isCurrMap") is True and isinstance(item.get("id"), int):
            return int(item["id"])
    for item in maps:
        if isinstance(item.get("id"), int):
            return int(item["id"])
    return None


def summarize_map_info(value: Any) -> dict[str, object]:
    grid_map = getattr(value, "grid_map", None)
    walls = read_sequence_attr(value, "walls")
    infeasible_areas = read_sequence_attr(value, "infeasible_areas")
    guide_pts = read_sequence_attr(value, "guide_pts")
    point_cloud = read_first_existing_attr(value, ("point_cloud", "points", "cloud"))
    return {
        "id": read_int_attr(value, "id"),
        "name": read_string_attr(value, "name") or "",
        "gravity": summarize_vector3(getattr(value, "gravity", None)),
        "gridMap": summarize_grid_map(grid_map),
        "walls": summarize_sequence(walls),
        "infeasibleAreas": summarize_sequence(infeasible_areas),
        "guidePoints": summarize_sequence(guide_pts),
        "pointCloud": summarize_sequence(point_cloud),
        "rawKeys": list_public_attrs(value),
    }


def summarize_grid_map(grid_map: Any) -> dict[str, object] | None:
    if grid_map is None:
        return None
    data = read_first_existing_attr(grid_map, ("data", "cells"))
    width = read_int_attr(grid_map, "width")
    height = read_int_attr(grid_map, "height")
    return {
        "width": width,
        "height": height,
        "resolution": read_float_attr(grid_map, "resolution"),
        "origin": summarize_pose(getattr(grid_map, "origin", None)),
        "expectedDataLength": expected_grid_data_length(width, height),
        "dataType": type_name(data),
        "dataLength": read_sequence_length(data),
        "dataSample": sample_sequence(data, GRID_DATA_SAMPLE_SIZE),
        "rawKeys": list_public_attrs(grid_map),
    }


def summarize_sequence(value: Any) -> dict[str, object]:
    return {
        "type": type_name(value),
        "count": read_sequence_length(value),
        "sample": sample_sequence(value, POINT_SAMPLE_SIZE),
    }


def summarize_pose(value: Any) -> dict[str, object] | None:
    if value is None:
        return None
    return {
        "position": summarize_vector3(getattr(value, "position", None)),
        "orientation": summarize_quaternion(getattr(value, "orientation", None)),
    }


def summarize_vector3(value: Any) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {
        "x": read_float_attr(value, "x"),
        "y": read_float_attr(value, "y"),
        "z": read_float_attr(value, "z"),
    }


def summarize_quaternion(value: Any) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {
        "x": read_float_attr(value, "x"),
        "y": read_float_attr(value, "y"),
        "z": read_float_attr(value, "z"),
        "w": read_float_attr(value, "w"),
    }


def sample_sequence(value: Any, limit: int) -> list[object]:
    if limit <= 0 or value is None or isinstance(value, str) or isinstance(value, Mapping):
        return []
    if isinstance(value, bytes | bytearray):
        return list(value[:limit])
    if isinstance(value, memoryview):
        return sample_memoryview(value, limit)

    iterable = read_iterable(value)
    if iterable is not None:
        try:
            return [compact_object(item) for item in islice(iterable, limit)]
        except Exception:
            pass

    count = read_sequence_length(value)
    if count <= 0 or not hasattr(value, "__getitem__"):
        return []

    sample: list[object] = []
    for index in range(min(count, limit)):
        try:
            sample.append(compact_object(value[index]))
        except Exception:
            break
    return sample


def read_sequence_attr(value: Any, name: str) -> Any:
    return getattr(value, name, None)


def read_first_existing_attr(value: Any, names: Sequence[str]) -> Any:
    for name in names:
        if hasattr(value, name):
            return getattr(value, name)
    return None


def read_sequence_length(value: Any) -> int:
    if value is None or isinstance(value, str) or isinstance(value, Mapping):
        return 0
    if isinstance(value, bytes | bytearray):
        return len(value)
    if isinstance(value, memoryview):
        return value.nbytes
    size = read_size_attr(value)
    if size is not None:
        return size
    try:
        length = len(value)
    except Exception:
        return 0
    return length if isinstance(length, int) and length >= 0 else 0


def read_int_attr(value: Any, name: str) -> int | None:
    raw = getattr(value, name, None)
    if isinstance(raw, bool):
        return None
    if isinstance(raw, int):
        return raw
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def read_float_attr(value: Any, name: str) -> float | None:
    raw = getattr(value, name, None)
    if isinstance(raw, bool):
        return None
    if isinstance(raw, int | float):
        return float(raw)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def read_bool_attr(value: Any, name: str) -> bool | None:
    raw = getattr(value, name, None)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, int):
        return raw != 0
    return None


def read_string_attr(value: Any, name: str) -> str | None:
    raw = getattr(value, name, None)
    if raw is None:
        return None
    text = str(raw).strip()
    return text or None


def compact_object(value: Any) -> object:
    if value is None or isinstance(value, str | int | float | bool):
        return value
    if isinstance(value, Mapping):
        return {str(key): compact_object(item) for key, item in value.items()}
    if is_sequence_like(value):
        return {
            "type": type_name(value),
            "count": read_sequence_length(value),
            "sample": sample_sequence(value, 5),
        }
    attrs = list_public_attrs(value)
    if not attrs:
        return to_jsonable(value)
    return {name: compact_object(getattr(value, name)) for name in attrs[:20]}


def list_public_attrs(value: Any) -> list[str]:
    try:
        return [
            name
            for name in dir(value)
            if not name.startswith("_") and not callable(getattr(value, name, None))
        ]
    except Exception:
        return []


def expected_grid_data_length(width: int | None, height: int | None) -> int | None:
    if width is None or height is None or width < 0 or height < 0:
        return None
    return width * height


def type_name(value: Any) -> str | None:
    if value is None:
        return None
    value_type = type(value)
    if value_type.__module__ == "builtins":
        return value_type.__qualname__
    return f"{value_type.__module__}.{value_type.__qualname__}"


def read_size_attr(value: Any) -> int | None:
    raw = getattr(value, "size", None)
    if isinstance(raw, bool) or callable(raw):
        return None
    try:
        size = int(raw)
    except (TypeError, ValueError):
        return None
    return size if size >= 0 else None


def read_iterable(value: Any) -> Iterable[Any] | None:
    try:
        flat = getattr(value, "flat", None)
    except Exception:
        flat = None
    if flat is not None:
        try:
            return iter(flat)
        except Exception:
            pass
    try:
        return iter(value)
    except Exception:
        return None


def sample_memoryview(value: memoryview, limit: int) -> list[object]:
    try:
        return list(value[:limit])
    except Exception:
        try:
            return list(value.cast("B")[:limit])
        except Exception:
            return []


def is_sequence_like(value: Any) -> bool:
    if value is None or isinstance(value, str) or isinstance(value, Mapping):
        return False
    if isinstance(value, bytes | bytearray | memoryview):
        return True
    return hasattr(value, "__len__") and (
        hasattr(value, "__iter__") or hasattr(value, "__getitem__")
    )


def validate_map_id(map_id: int | None) -> dict[str, object] | None:
    if map_id is None:
        return None
    if isinstance(map_id, bool) or not isinstance(map_id, int) or not 0 <= map_id <= 255:
        return unavailable_result(
            "validate_map_id",
            ValueError("map_id must be an integer between 0 and 255"),
            map_id=map_id,
        )
    return None


def validate_timeout_ms(timeout_ms: int) -> dict[str, object] | None:
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
        return unavailable_result(
            "validate_timeout_ms",
            ValueError("timeout_ms must be a positive integer"),
        )
    return None


def build_subprocess_timeout_seconds(timeout_ms: int) -> float:
    return max(1.0, timeout_ms / 1000.0 + SUBPROCESS_TIMEOUT_MARGIN_SECONDS)


def busy_result(*, active_purpose: str | None, map_id: int | None) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_MAP_BACKEND,
        "action": ACTION_GET_HIGH_PRECISION_MAPS,
        "requestedMapId": map_id,
        "collectedAt": utc_now_iso(),
        "busy": True,
        "activePurpose": active_purpose,
        "errorStage": "gdk_session_busy",
        "errorMsg": "GDK session is busy",
    }


def unavailable_result(
    stage: str,
    error: Exception,
    *,
    map_id: int | None = None,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": GDK_MAP_BACKEND,
        "action": ACTION_GET_HIGH_PRECISION_MAPS,
        "requestedMapId": map_id,
        "mapCount": 0,
        "maps": [],
        "mapDetail": None,
        "collectedAt": utc_now_iso(),
        "errorStage": stage,
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }
    if extra:
        result.update(extra)
    return result
