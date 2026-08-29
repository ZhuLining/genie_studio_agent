"""GDK 机器人身份只读快照。

当前真机验证显示 ``agibot_gdk.get_robot_aid()`` 返回值与二维码建图产物目录使用的
机器人 SN 一致。这里把它封装为独立只读能力，供客户端自动填充 SN；它不创建
Robot()，但仍进入 GDK 生命周期和全局互斥，避免 DDS/GDK 初始化边界被并发打穿。
"""

from __future__ import annotations

import importlib
from collections.abc import Callable
from typing import Any

from .camera_frame import should_use_in_process_runtime
from .control_probe import initialize_gdk, release_gdk, utc_now_iso
from .readonly import GDK_MODULE_NAME, to_jsonable
from .session import GdkSessionImportError, GdkSessionInitError, GdkSessionManager
from .subprocess_runtime import run_gdk_subprocess

ACTION_GET_ROBOT_IDENTITY = "get_robot_identity"
GDK_IDENTITY_BACKEND = "agibot_gdk"
DEFAULT_ROBOT_IDENTITY_TIMEOUT_MS = 3000
SUBPROCESS_TIMEOUT_MARGIN_SECONDS = 3.0


def run_gdk_robot_identity_snapshot(
    timeout_ms: int = DEFAULT_ROBOT_IDENTITY_TIMEOUT_MS,
    *,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    """读取机器人身份。

    身份读取是只读能力，但 ``gdk_init/get_robot_aid`` 仍可能被底层 DDS 阻塞；生产
    路径使用父进程互斥 + 子进程超时，确保 robot_state 队列不会被永久卡住。
    """

    timeout_result = validate_timeout_ms(timeout_ms)
    if timeout_result is not None:
        return timeout_result

    if should_use_in_process_runtime(import_module, session_manager):
        return run_gdk_robot_identity_snapshot_in_process(
            timeout_ms=timeout_ms,
            import_module=import_module,
            session_manager=session_manager,
        )

    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=False,
            purpose=ACTION_GET_ROBOT_IDENTITY,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose)

    with lease:
        result = run_gdk_subprocess(
            operation="robot_identity",
            action=ACTION_GET_ROBOT_IDENTITY,
            backend=GDK_IDENTITY_BACKEND,
            timeout_seconds=build_subprocess_timeout_seconds(timeout_ms),
            child_target=robot_identity_child,
            child_args=(timeout_ms,),
            safety_gate={
                "enabled": False,
                "confirmed": True,
                "reason": "read_only_robot_identity",
            },
        )
        result["gdk_parent_lock"] = lease.to_payload()
        return result


def run_gdk_robot_identity_snapshot_in_process(
    *,
    timeout_ms: int,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
) -> dict[str, object]:
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=False,
            initialize=True,
            purpose=ACTION_GET_ROBOT_IDENTITY,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error)
    except GdkSessionInitError as error:
        return unavailable_result(
            "gdk_init",
            RuntimeError(str(error)),
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return busy_result(active_purpose=manager.active_purpose)

    with lease:
        if lease.agibot_gdk is None:
            return unavailable_result(
                "gdk_session_acquire",
                RuntimeError("GDK session lease missing initialized module"),
            )
        result = collect_robot_identity(
            agibot_gdk=lease.agibot_gdk,
            timeout_ms=timeout_ms,
        )
        result["gdk_init"] = lease.init_result
        result["gdk_session"] = lease.to_payload()
        return result


def robot_identity_child(result_queue: Any, timeout_ms: int) -> None:
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
                extra={"gdk_init": init_result},
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            result = collect_robot_identity(
                agibot_gdk=agibot_gdk,
                timeout_ms=timeout_ms,
            )
    except Exception as error:
        result = unavailable_result("import_or_initialize_gdk", error)
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            result["gdk_release"] = release_gdk(agibot_gdk)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        result_queue.put(result)


def collect_robot_identity(*, agibot_gdk: Any, timeout_ms: int) -> dict[str, object]:
    try:
        get_robot_aid = agibot_gdk.get_robot_aid
    except AttributeError as error:
        return unavailable_result("get_robot_aid_attr", error)

    if not callable(get_robot_aid):
        return unavailable_result(
            "get_robot_aid_attr",
            TypeError("agibot_gdk.get_robot_aid is not callable"),
        )

    try:
        robot_aid_raw = get_robot_aid()
    except Exception as error:
        return unavailable_result("get_robot_aid", error)

    robot_aid = normalize_robot_aid(robot_aid_raw)
    if not robot_aid:
        return unavailable_result(
            "get_robot_aid",
            RuntimeError("agibot_gdk.get_robot_aid() returned an empty value"),
            extra={"raw": {"get_robot_aid": to_jsonable(robot_aid_raw)}},
        )

    return {
        "available": True,
        "backend": GDK_IDENTITY_BACKEND,
        "action": ACTION_GET_ROBOT_IDENTITY,
        "robotAid": robot_aid,
        "robotSerial": robot_aid,
        "suggestedRobotSerial": robot_aid,
        "identitySource": "agibot_gdk.get_robot_aid",
        "timeoutMs": timeout_ms,
        "collectedAt": utc_now_iso(),
        "raw": {"get_robot_aid": to_jsonable(robot_aid_raw)},
    }


def normalize_robot_aid(value: object) -> str:
    if isinstance(value, bytes | bytearray):
        return bytes(value).decode("utf-8", errors="replace").strip()
    return str(value or "").strip()


def validate_timeout_ms(timeout_ms: int) -> dict[str, object] | None:
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
        return unavailable_result(
            "validate_timeout_ms",
            ValueError("timeout_ms must be a positive integer"),
        )
    return None


def build_subprocess_timeout_seconds(timeout_ms: int) -> float:
    return max(1.0, timeout_ms / 1000.0 + SUBPROCESS_TIMEOUT_MARGIN_SECONDS)


def busy_result(*, active_purpose: str | None) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_IDENTITY_BACKEND,
        "action": ACTION_GET_ROBOT_IDENTITY,
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
    extra: dict[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": GDK_IDENTITY_BACKEND,
        "action": ACTION_GET_ROBOT_IDENTITY,
        "collectedAt": utc_now_iso(),
        "errorStage": stage,
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }
    if extra:
        result.update(extra)
    return result
