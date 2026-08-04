from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass
from threading import Lock
from typing import Any

from .control_probe import initialize_gdk, release_gdk
from .readonly import GDK_MODULE_NAME

PROCESS_MANAGED_RELEASE_RESULT = {
    "called": False,
    "success": True,
    "reason": "process_managed_session",
}


class GdkSessionError(RuntimeError):
    """GDK 进程级 session 进入失败。"""


class GdkSessionImportError(GdkSessionError):
    """导入 agibot_gdk 失败。"""

    def __init__(self, error: Exception) -> None:
        super().__init__(str(error))
        self.error = error


class GdkSessionInitError(GdkSessionError):
    """gdk_init 返回失败。"""

    def __init__(self, init_result: dict[str, object]) -> None:
        super().__init__("agibot_gdk.gdk_init() did not return success")
        self.init_result = init_result


@dataclass
class GdkSessionLease:
    manager: GdkSessionManager
    purpose: str
    initialize: bool
    agibot_gdk: Any | None
    init_result: dict[str, object]

    def __enter__(self) -> GdkSessionLease:
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.release()

    def release(self) -> None:
        self.manager.release(self)

    def to_payload(self) -> dict[str, object]:
        return {
            "policy": "process_managed_session",
            "purpose": self.purpose,
            "initialize": self.initialize,
            "busy": self.manager.busy,
            "active_purpose": self.manager.active_purpose,
            "init_result": dict(self.init_result),
        }


class GdkSessionManager:
    """进程级 GDK 生命周期与访问互斥管理。

    agibot_gdk 的 IPC/DDS 线程安全边界未知，因此所有 GDK 读写都必须先获取
    同一把 operation lock。控制动作阻塞等待，当前位姿读取使用非阻塞获取，
    忙时直接返回 ROBOT_BUSY，避免 release/use-after-release 竞态。
    对带 timeout 的正式控制 runtime，父进程只用 initialize=False lease 做调度互斥；
    真正的 GDK 初始化、控制调用和 release 放在可被杀掉的子进程里。
    """

    def __init__(
        self,
        *,
        import_module: Callable[[str], Any] = importlib.import_module,
        module_name: str = GDK_MODULE_NAME,
    ) -> None:
        self.import_module = import_module
        self.module_name = module_name
        self._operation_lock = Lock()
        self._state_lock = Lock()
        self._agibot_gdk: Any | None = None
        self._initialized = False
        self._init_result: dict[str, object] = {
            "called": False,
            "success": True,
            "return": None,
        }
        self._active_lease: GdkSessionLease | None = None

    @property
    def busy(self) -> bool:
        return self._active_lease is not None

    @property
    def active_purpose(self) -> str | None:
        lease = self._active_lease
        return lease.purpose if lease is not None else None

    def diagnostics(self) -> dict[str, object]:
        return {
            "policy": "process_managed_session",
            "busy": self.busy,
            "active_purpose": self.active_purpose,
            "initialized": self._initialized,
            "init_result": dict(self._init_result),
        }

    def acquire(
        self,
        *,
        blocking: bool,
        initialize: bool,
        purpose: str,
    ) -> GdkSessionLease | None:
        lock_acquired = self._operation_lock.acquire(blocking=blocking)
        if not lock_acquired:
            return None

        try:
            if initialize:
                agibot_gdk, init_result = self._ensure_initialized()
            else:
                agibot_gdk = None
                init_result = {"called": False, "success": True, "return": None}
            lease = GdkSessionLease(
                manager=self,
                purpose=purpose,
                initialize=initialize,
                agibot_gdk=agibot_gdk,
                init_result=init_result,
            )
            self._active_lease = lease
            return lease
        except Exception:
            self._operation_lock.release()
            raise

    def release(self, lease: GdkSessionLease) -> None:
        if self._active_lease is lease:
            self._active_lease = None
            self._operation_lock.release()

    def shutdown(self, timeout: float = 5.0) -> dict[str, object]:
        # shutdown 只在没有活跃 GDK 操作时 release；拿不到锁就宁可跳过 release。
        acquired = self._operation_lock.acquire(timeout=timeout)
        if not acquired:
            return {
                "called": False,
                "success": False,
                "busy": True,
                "reason": "gdk_operation_active",
                "active_purpose": self.active_purpose,
            }

        try:
            with self._state_lock:
                if not self._initialized or self._agibot_gdk is None:
                    return {
                        "called": False,
                        "success": True,
                        "busy": False,
                        "reason": "not_initialized",
                    }

                result = release_gdk(self._agibot_gdk)
                self._initialized = False
                self._agibot_gdk = None
                self._init_result = {
                    "called": False,
                    "success": True,
                    "return": None,
                }
                return {**result, "busy": False}
        finally:
            self._operation_lock.release()

    def _ensure_initialized(self) -> tuple[Any, dict[str, object]]:
        with self._state_lock:
            if self._agibot_gdk is None:
                try:
                    self._agibot_gdk = self.import_module(self.module_name)
                except Exception as error:
                    raise GdkSessionImportError(error) from error

            if self._initialized:
                return self._agibot_gdk, {**self._init_result, "reused": True}

            init_result = initialize_gdk(self._agibot_gdk)
            self._init_result = init_result
            if init_result.get("called") is True and init_result.get("success") is not True:
                raise GdkSessionInitError(init_result)

            self._initialized = True
            return self._agibot_gdk, {**init_result, "reused": False}
