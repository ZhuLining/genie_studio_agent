"""GDK 持久 worker 子进程管理器。

通过复用单个 multiprocessing.Process 避免每个节点重复 spawn/import/init agibot_gdk。
命令超时时直接 kill worker，下条命令懒启动新 worker。

数据流::

    父进程                              worker 子进程
    ────────                            ────────────
    run_command() ──command_queue──▶   gdk_worker_main()
                                      │
                                      ├─ motion_abs_joint → Robot.move_arm_joint()
                                      │                    / end_effector_pose_control()
                                      ├─ end_effector    → Robot.move_ee_pos()
                                      ├─ code_script     → run_script_safely()
                                      └─ shutdown        → release_gdk()
                                      │
    _wait_for_result() ◀──result_queue── result

    cancel_active_command() → terminate worker → 下次命令懒重启

父进程仍通过 GdkSessionManager 做访问互斥；worker 只承担 GDK C 扩展的阻塞边界。
"""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping
from multiprocessing import get_context
from queue import Empty
from threading import Lock
from typing import Any
from uuid import uuid4

from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND
from gsa_taskflow_executor.gdk.subprocess_runtime import (
    DEFAULT_TERMINATE_GRACE_SECONDS,
    build_subprocess_failed_result,
    build_timeout_result,
    close_process,
    close_queue,
    read_timeout_seconds,
)
from gsa_taskflow_executor.gdk.worker_commands import (
    GDK_WORKER_COMMAND_TIMEOUT_SEMANTICS,
    GDK_WORKER_POLICY,
    GDK_WORKER_SHUTDOWN_ACTION,
    build_cancelled_result,
    build_worker_process_payload,
    default_gdk_release_result,
    gdk_worker_main,
    normalize_string_mapping,
    read_mapping_field,
    read_string_field,
)

DEFAULT_WORKER_SHUTDOWN_TIMEOUT_SECONDS = 5.0

WorkerTarget = Callable[[Any, Any], None]


class GdkWorkerProcessManager:
    """复用单个 GDK 子进程的命令管理器。

    父进程通过 GdkSessionManager 做访问互斥；本类只管理 worker 生命周期。
    命令超时时直接 terminate worker → 下次命令懒启动新 worker。
    """

    def __init__(
        self,
        *,
        worker_target: WorkerTarget | None = None,
        start_method: str = "spawn",
        terminate_grace_seconds: float = DEFAULT_TERMINATE_GRACE_SECONDS,
    ) -> None:
        self._ctx: Any = get_context(start_method)
        self._worker_target: WorkerTarget = worker_target or gdk_worker_main
        self._terminate_grace_seconds = terminate_grace_seconds
        self._lock = Lock()            # 保护进程/队列状态
        self._command_lock = Lock()    # 串行化命令（同一时刻只有一个命令在飞）
        self._process: Any | None = None
        self._command_queue: Any | None = None
        self._result_queue: Any | None = None
        # 未认领的结果（command_id 匹配但延迟到达）。
        self._pending_results: dict[str, dict[str, object]] = {}
        # cancel_active_command 注入给等待线程的取消结果。
        self._cancelled_results: dict[str, dict[str, object]] = {}
        self._active_command_id: str | None = None
        self._active_command_payload: dict[str, object] | None = None

    def run_command(
        self,
        *,
        kind: str,
        payload: Mapping[str, object],
        action: str,
        backend: str,
        timeout_seconds: float,
        safety_gate: Mapping[str, object],
    ) -> dict[str, object]:
        """发送命令到 worker，阻塞等待结果。超时时 terminate worker 并返回 timeout 结果。"""
        command_id = str(uuid4())
        with self._command_lock:
            process: Any | None = None
            worker_started = False
            try:
                with self._lock:
                    process, command_queue, result_queue, worker_started = (
                        self._ensure_started_locked()
                    )
                    self._active_command_id = command_id
                    self._active_command_payload = {
                        "action": action,
                        "backend": backend,
                        "safety_gate": dict(safety_gate),
                    }
                    command_queue.put(
                        {
                            "command_id": command_id,
                            "kind": kind,
                            "payload": dict(payload),
                            "action": action,
                            "backend": backend,
                            "safety_gate": dict(safety_gate),
                        }
                    )
            except Exception as error:
                with self._lock:
                    subprocess_payload = build_worker_process_payload(
                        self._process,
                        command_id=command_id,
                        timed_out=False,
                        terminated=False,
                        killed=False,
                        worker_started=False,
                        worker_reused=False,
                    )
                    self._reset_locked()
                return build_subprocess_failed_result(
                    action=action,
                    backend=backend,
                    stage="gdk_worker_start_or_send_command",
                    message=str(error),
                    safety_gate=safety_gate,
                    subprocess_payload=subprocess_payload,
                )

            status, envelope = self._wait_for_result(
                command_id=command_id,
                process=process,
                result_queue=result_queue,
                timeout_seconds=timeout_seconds,
            )

            if status == "timeout":
                with self._lock:
                    subprocess_payload = self._terminate_worker_locked(
                        command_id=command_id,
                        timeout_seconds=timeout_seconds,
                        worker_started=worker_started,
                    )
                result = build_timeout_result(
                    action=action,
                    backend=backend,
                    timeout_seconds=timeout_seconds,
                    safety_gate=safety_gate,
                    subprocess_payload=subprocess_payload,
                )
                result["timeout_semantics"] = GDK_WORKER_COMMAND_TIMEOUT_SEMANTICS
                return result

            if status != "ok" or envelope is None:
                with self._lock:
                    subprocess_payload = build_worker_process_payload(
                        process,
                        command_id=command_id,
                        timed_out=False,
                        terminated=False,
                        killed=False,
                        worker_started=worker_started,
                        worker_reused=not worker_started,
                    )
                    self._reset_locked()
                return build_subprocess_failed_result(
                    action=action,
                    backend=backend,
                    stage=f"gdk_worker_{status}",
                    message="GDK worker process exited or returned an invalid envelope",
                    safety_gate=safety_gate,
                    subprocess_payload=subprocess_payload,
                )

            # 提取并校验 command result
            raw_result = envelope.get("result")
            command_result = normalize_string_mapping(raw_result)
            if command_result is None:
                with self._lock:
                    subprocess_payload = build_worker_process_payload(
                        process,
                        command_id=command_id,
                        timed_out=False,
                        terminated=False,
                        killed=False,
                        worker_started=worker_started,
                        worker_reused=not worker_started,
                    )
                    if self._active_command_id == command_id:
                        self._active_command_id = None
                        self._active_command_payload = None
                return build_subprocess_failed_result(
                    action=action,
                    backend=backend,
                    stage="gdk_worker_result_invalid",
                    message="GDK worker returned a non-dict command result",
                    safety_gate=safety_gate,
                    subprocess_payload=subprocess_payload,
                )

            if "subprocess" not in command_result:
                command_result["subprocess"] = build_worker_process_payload(
                    process,
                    command_id=command_id,
                    timed_out=False,
                    terminated=False,
                    killed=False,
                    worker_started=worker_started,
                    worker_reused=not worker_started,
                )
            with self._lock:
                if self._active_command_id == command_id:
                    self._active_command_id = None
                    self._active_command_payload = None
            return command_result

    def cancel_active_command(self, reason: str) -> dict[str, object]:
        """终止当前正在执行的 worker 命令（来自 MQTT 取消回调线程）。

        这不是机器人控制器急停；只切断 executor 侧阻塞的 GDK worker。
        后续控制命令会被恢复门控挡住，直到只读状态确认机器人安全。
        """
        command_id = str(uuid4())
        with self._lock:
            active_command_id = self._active_command_id
            active_payload = dict(self._active_command_payload or {})
            if active_command_id is None:
                return {
                    "called": False,
                    "success": True,
                    "reason": "no_active_command",
                    "request_reason": reason,
                    "subprocess": build_worker_process_payload(
                        self._process,
                        command_id=command_id,
                        timed_out=False,
                        terminated=False,
                        killed=False,
                        worker_started=False,
                        worker_reused=self._process is not None,
                    ),
                }

            subprocess_payload = self._terminate_worker_locked(
                command_id=active_command_id,
                timeout_seconds=0.0,
                worker_started=False,
                timed_out=False,
            )
            result = build_cancelled_result(
                action=read_string_field(active_payload, "action") or "gdk_worker_command",
                backend=read_string_field(active_payload, "backend") or GDK_BACKEND,
                reason=reason,
                safety_gate=read_mapping_field(active_payload, "safety_gate") or {},
                subprocess_payload=subprocess_payload,
            )
            # 注入取消结果，_wait_for_result 中会优先返回
            self._cancelled_results[active_command_id] = {
                "command_id": active_command_id,
                "result": result,
            }
            return {
                "called": True,
                "success": True,
                "reason": "worker_terminated",
                "request_reason": reason,
                "active_command_id": active_command_id,
                "result": result,
                "subprocess": subprocess_payload,
            }

    def shutdown(
        self,
        timeout_seconds: float = DEFAULT_WORKER_SHUTDOWN_TIMEOUT_SECONDS,
    ) -> dict[str, object]:
        """优雅关闭 GDK worker。发送 shutdown 命令 → 等待 → 超时则 kill。"""
        command_id = str(uuid4())
        with self._command_lock:
            with self._lock:
                if self._process is None:
                    return {
                        "called": False,
                        "success": True,
                        "reason": "not_started",
                        "subprocess": build_worker_process_payload(
                            None,
                            command_id=command_id,
                            timed_out=False,
                            terminated=False,
                            killed=False,
                            worker_started=False,
                            worker_reused=False,
                        ),
                    }

                process = self._process
                command_queue = self._command_queue
                result_queue = self._result_queue
                if command_queue is None or result_queue is None or not process.is_alive():
                    subprocess_payload = build_worker_process_payload(
                        process,
                        command_id=command_id,
                        timed_out=False,
                        terminated=False,
                        killed=False,
                        worker_started=False,
                        worker_reused=True,
                    )
                    self._reset_locked()
                    return {
                        "called": False,
                        "success": True,
                        "reason": "worker_not_alive",
                        "subprocess": subprocess_payload,
                    }

                self._active_command_id = command_id
                self._active_command_payload = {
                    "action": GDK_WORKER_SHUTDOWN_ACTION,
                    "backend": GDK_BACKEND,
                    "safety_gate": {"enabled": True, "confirmed": True},
                }
                try:
                    command_queue.put(
                        {
                            "command_id": command_id,
                            "kind": "shutdown",
                            "payload": {},
                            "action": GDK_WORKER_SHUTDOWN_ACTION,
                            "backend": GDK_BACKEND,
                            "safety_gate": {"enabled": True, "confirmed": True},
                        }
                    )
                except Exception as error:
                    subprocess_payload = self._terminate_worker_locked(
                        command_id=command_id,
                        timeout_seconds=timeout_seconds,
                        worker_started=False,
                    )
                    return {
                        "called": True,
                        "success": False,
                        "reason": "send_shutdown_failed",
                        "error_type": type(error).__name__,
                        "error_msg": str(error),
                        "subprocess": subprocess_payload,
                    }

            status, envelope = self._wait_for_result(
                command_id=command_id,
                process=process,
                result_queue=result_queue,
                timeout_seconds=timeout_seconds,
            )
            process.join(0.1)
            with self._lock:
                if process.is_alive():
                    subprocess_payload = self._terminate_worker_locked(
                        command_id=command_id,
                        timeout_seconds=timeout_seconds,
                        worker_started=False,
                    )
                    return {
                        "called": True,
                        "success": False,
                        "reason": "shutdown_timeout",
                        "gdk_release": default_gdk_release_result(reason="shutdown_timeout"),
                        "subprocess": subprocess_payload,
                    }

                subprocess_payload = build_worker_process_payload(
                    process,
                    command_id=command_id,
                    timed_out=status == "timeout",
                    terminated=False,
                    killed=False,
                    worker_started=False,
                    worker_reused=True,
                )
                self._reset_locked()

            result = normalize_string_mapping(envelope.get("result")) if envelope else None
            release_result = read_mapping_field(result, "gdk_release")
            return {
                "called": True,
                "success": status == "ok",
                "reason": status,
                "gdk_release": release_result
                or default_gdk_release_result(reason="release_result_missing"),
                "subprocess": subprocess_payload,
            }

    def diagnostics(self) -> dict[str, object]:
        """返回 worker 进程快照；只读，不 spawn、不发命令、不导入 GDK。"""
        with self._lock:
            process_alive = False
            if self._process is not None:
                try:
                    process_alive = bool(self._process.is_alive())
                except ValueError:
                    process_alive = False

            return {
                "policy": GDK_WORKER_POLICY,
                "started": self._process is not None,
                "process_alive": process_alive,
                "process": build_worker_process_payload(
                    self._process,
                    command_id=self._active_command_id,
                    timed_out=False,
                    terminated=False,
                    killed=False,
                    worker_started=False,
                    worker_reused=self._process is not None,
                ),
                "active_command_id": self._active_command_id,
                "active_command": (
                    dict(self._active_command_payload)
                    if self._active_command_payload is not None
                    else None
                ),
                "pending_result_count": len(self._pending_results),
                "cancelled_result_count": len(self._cancelled_results),
            }

    # ---- 内部方法（均在持有 _lock 时调用） ----

    def _ensure_started_locked(self) -> tuple[Any, Any, Any, bool]:
        """确保 worker 进程在运行。返回 (process, cmd_queue, result_queue, is_new)。

        若当前 worker 存活则复用；否则 spawn 新进程。
        """
        if (
            self._process is not None
            and self._command_queue is not None
            and self._result_queue is not None
            and self._process.is_alive()
        ):
            return self._process, self._command_queue, self._result_queue, False

        self._reset_locked()
        command_queue = self._ctx.Queue(maxsize=1)
        result_queue = self._ctx.Queue()
        process = self._ctx.Process(
            target=self._worker_target,
            args=(command_queue, result_queue),
            name="gdk-persistent-worker",
        )
        process.daemon = True
        process.start()
        self._process = process
        self._command_queue = command_queue
        self._result_queue = result_queue
        return process, command_queue, result_queue, True

    def _wait_for_result(
        self,
        *,
        command_id: str,
        process: Any,
        result_queue: Any,
        timeout_seconds: float,
    ) -> tuple[str, dict[str, object] | None]:
        """阻塞等待 worker 返回结果。支持 timeout、取消注入、pending result 认领。

        每 200ms 轮询 result_queue，同时检查:
        - cancelled_results（cancel_active_command 注入）
        - process.is_alive()（worker 异常退出）
        - pending_results（之前到达但 command_id 不匹配的结果）
        """
        pending = self._pop_pending_result(command_id)
        if pending is not None:
            return "ok", pending

        deadline = time.monotonic() + timeout_seconds
        while True:
            cancelled = self._pop_cancelled_result(command_id)
            if cancelled is not None:
                return "ok", cancelled

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return "timeout", None

            try:
                raw_envelope = result_queue.get(timeout=min(remaining, 0.2))
            except Empty:
                cancelled = self._pop_cancelled_result(command_id)
                if cancelled is not None:
                    return "ok", cancelled
                if not process.is_alive():
                    return "worker_exited", None
                continue

            envelope = normalize_string_mapping(raw_envelope)
            if envelope is None:
                return "invalid_envelope", None

            raw_command_id = envelope.get("command_id")
            if not isinstance(raw_command_id, str):
                return "invalid_envelope", envelope

            if raw_command_id == command_id:
                return "ok", envelope
            # command_id 不匹配 → 存储为 pending（可能来自之前的命令）
            self._store_pending_result(raw_command_id, envelope)

    def _terminate_worker_locked(
        self,
        *,
        command_id: str,
        timeout_seconds: float,
        worker_started: bool,
        timed_out: bool = True,
    ) -> dict[str, object]:
        """强制终止 worker：先 terminate()，未退出则 kill()。"""
        process = self._process
        terminated = False
        killed = False
        if process is not None and process.is_alive():
            process.terminate()
            terminated = True
            process.join(self._terminate_grace_seconds)
            if process.is_alive() and hasattr(process, "kill"):
                process.kill()
                killed = True
                process.join(self._terminate_grace_seconds)

        payload = build_worker_process_payload(
            process,
            command_id=command_id,
            timed_out=timed_out,
            terminated=terminated,
            killed=killed,
            worker_started=worker_started,
            worker_reused=not worker_started,
        )
        payload["timeout_seconds"] = timeout_seconds
        payload["terminate_grace_seconds"] = self._terminate_grace_seconds
        self._reset_locked()
        return payload

    def _reset_locked(self) -> None:
        """清空所有 worker 状态（队列、进程、pending/cancelled results）。"""
        if self._command_queue is not None:
            close_queue_safely(self._command_queue)
        if self._result_queue is not None:
            close_queue_safely(self._result_queue)
        if self._process is not None:
            close_process_safely(self._process)
        self._process = None
        self._command_queue = None
        self._result_queue = None
        self._pending_results.clear()
        self._active_command_id = None
        self._active_command_payload = None

    def _pop_pending_result(self, command_id: str) -> dict[str, object] | None:
        with self._lock:
            return self._pending_results.pop(command_id, None)

    def _store_pending_result(self, command_id: str, envelope: dict[str, object]) -> None:
        with self._lock:
            self._pending_results[command_id] = envelope

    def _pop_cancelled_result(self, command_id: str) -> dict[str, object] | None:
        with self._lock:
            return self._cancelled_results.pop(command_id, None)


# ============================================================
# 顶层便捷函数（供 motion_runtime / end_effector_runtime / code_script 调用）
# ============================================================


def run_motion_abs_joint_in_worker(
    motion_params: object,
    *,
    action: str,
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
    """通过持久 worker 执行运动规划。

    kind 沿用 motion_abs_joint 以兼容父进程队列协议；实际 action 会区分
    taskflow_abs_joint / taskflow_abs_pose，便于 timeout/recovery 日志排查。
    """
    return get_default_gdk_worker_manager().run_command(
        kind="motion_abs_joint",
        payload={"motion_params": motion_params},
        action=action,
        backend=GDK_BACKEND,
        timeout_seconds=read_timeout_seconds(motion_params),
        safety_gate=safety_gate,
    )


def run_end_effector_in_worker(
    end_effector_params: object,
    *,
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
    """通过持久 worker 执行末端控制。"""
    return get_default_gdk_worker_manager().run_command(
        kind="end_effector",
        payload={"end_effector_params": end_effector_params},
        action="taskflow_end_effector",
        backend=GDK_BACKEND,
        timeout_seconds=read_timeout_seconds(end_effector_params),
        safety_gate=safety_gate,
    )


def run_code_script_in_worker(
    script_params: object,
    *,
    script_id: str,
    inputs: Mapping[str, object],
    environ: Mapping[str, str],
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
    """通过持久 worker 执行代码脚本。"""
    return get_default_gdk_worker_manager().run_command(
        kind="code_script",
        payload={
            "script_params": script_params,
            "script_id": script_id,
            "inputs": dict(inputs),
            "environ": dict(environ),
        },
        action=script_id,
        backend="executor_code_script",
        timeout_seconds=read_timeout_seconds(script_params),
        safety_gate=safety_gate,
    )


def run_point_recording_snapshot_in_worker(
    payload: Mapping[str, object],
    *,
    action: str,
    timeout_seconds: float,
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
    """通过持久 worker 执行点位录制只读采样。

    点位录制需要 Robot 读取末端位姿和关节；G2 实测 Robot() 首次构造可接近 30s，
    因此复用常驻 worker 内的 Robot，避免每个点位都重新等待 DDS/控制栈初始化。
    """

    return get_default_gdk_worker_manager().run_command(
        kind="point_recording_snapshot",
        payload=payload,
        action=action,
        backend=GDK_BACKEND,
        timeout_seconds=timeout_seconds,
        safety_gate=safety_gate,
    )


def shutdown_default_gdk_worker(
    timeout_seconds: float = DEFAULT_WORKER_SHUTDOWN_TIMEOUT_SECONDS,
) -> dict[str, object]:
    """关闭全局默认 GDK worker。"""
    global _DEFAULT_GDK_WORKER_MANAGER
    with _DEFAULT_GDK_WORKER_MANAGER_LOCK:
        manager = _DEFAULT_GDK_WORKER_MANAGER
    if manager is None:
        return {
            "called": False,
            "success": True,
            "reason": "not_started",
            "subprocess": build_worker_process_payload(
                None,
                command_id=None,
                timed_out=False,
                terminated=False,
                killed=False,
                worker_started=False,
                worker_reused=False,
            ),
        }
    result = manager.shutdown(timeout_seconds=timeout_seconds)
    with _DEFAULT_GDK_WORKER_MANAGER_LOCK:
        if _DEFAULT_GDK_WORKER_MANAGER is manager:
            _DEFAULT_GDK_WORKER_MANAGER = None
    return result


def cancel_default_gdk_worker_command(reason: str) -> dict[str, object]:
    """取消全局默认 GDK worker 的当前命令。"""
    with _DEFAULT_GDK_WORKER_MANAGER_LOCK:
        manager = _DEFAULT_GDK_WORKER_MANAGER
    if manager is None:
        return {
            "called": False,
            "success": True,
            "reason": "worker_not_started",
            "request_reason": reason,
        }
    return manager.cancel_active_command(reason)


def diagnostics_default_gdk_worker() -> dict[str, object]:
    """返回全局默认 GDK worker 快照；不启动 worker，不触碰机器人。"""
    with _DEFAULT_GDK_WORKER_MANAGER_LOCK:
        manager = _DEFAULT_GDK_WORKER_MANAGER
    if manager is None:
        return {
            "policy": GDK_WORKER_POLICY,
            "started": False,
            "process": build_worker_process_payload(
                None,
                command_id=None,
                timed_out=False,
                terminated=False,
                killed=False,
                worker_started=False,
                worker_reused=False,
            ),
            "active_command_id": None,
            "active_command": None,
            "pending_result_count": 0,
            "cancelled_result_count": 0,
        }
    return manager.diagnostics()


def get_default_gdk_worker_manager() -> GdkWorkerProcessManager:
    """获取/懒创建全局默认 GDK worker manager。"""
    global _DEFAULT_GDK_WORKER_MANAGER
    with _DEFAULT_GDK_WORKER_MANAGER_LOCK:
        if _DEFAULT_GDK_WORKER_MANAGER is None:
            _DEFAULT_GDK_WORKER_MANAGER = GdkWorkerProcessManager()
        return _DEFAULT_GDK_WORKER_MANAGER


def close_queue_safely(queue: Any) -> None:
    try:
        close_queue(queue)
    except Exception:
        pass


def close_process_safely(process: Any) -> None:
    try:
        close_process(process)
    except Exception:
        pass


# 全局默认 worker manager（懒初始化）
_DEFAULT_GDK_WORKER_MANAGER: GdkWorkerProcessManager | None = None
_DEFAULT_GDK_WORKER_MANAGER_LOCK = Lock()
