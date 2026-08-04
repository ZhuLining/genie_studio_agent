from __future__ import annotations

import importlib
import os
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from multiprocessing import get_context
from queue import Empty
from threading import Lock
from typing import Any, cast
from uuid import uuid4

from gsa_taskflow_executor.gdk.control_probe import initialize_gdk, release_gdk, utc_now_iso
from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from gsa_taskflow_executor.gdk.subprocess_runtime import (
    DEFAULT_TERMINATE_GRACE_SECONDS,
    GDK_SUBPROCESS_FAILED_CODE,
    build_subprocess_failed_result,
    build_timeout_result,
    close_process,
    close_queue,
    read_timeout_seconds,
)

GDK_WORKER_POLICY = "persistent_gdk_worker"
GDK_WORKER_COMMAND_TIMEOUT_SEMANTICS = (
    "worker_process_restarted; robot_controller_cancel_not_guaranteed"
)
GDK_WORKER_SHUTDOWN_ACTION = "gdk_worker_shutdown"
DEFAULT_WORKER_SHUTDOWN_TIMEOUT_SECONDS = 5.0

WorkerTarget = Callable[[Any, Any], None]


@dataclass
class WorkerGdkState:
    agibot_gdk: Any | None = None
    init_attempted: bool = False
    gdk_initialized: bool = False
    init_result: dict[str, object] = field(default_factory=lambda: default_gdk_init_result())

    def require_agibot_gdk(self) -> Any:
        if self.agibot_gdk is None:
            raise RuntimeError("GDK worker missing initialized agibot_gdk module")
        return self.agibot_gdk


class GdkWorkerProcessManager:
    """复用一个可杀掉的 GDK 子进程，避免每个节点重复 spawn/import/init。

    父进程仍依赖 GdkSessionManager 做访问互斥；这里的 worker 只承担 GDK C 扩展
    的阻塞边界。命令超时时直接终止 worker，并在下一条命令懒启动新 worker。
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
        self._lock = Lock()
        self._process: Any | None = None
        self._command_queue: Any | None = None
        self._result_queue: Any | None = None
        self._pending_results: dict[str, dict[str, object]] = {}

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
        command_id = str(uuid4())
        with self._lock:
            try:
                process, command_queue, result_queue, worker_started = self._ensure_started_locked()
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

            status, envelope = self._wait_for_result_locked(
                command_id=command_id,
                process=process,
                result_queue=result_queue,
                timeout_seconds=timeout_seconds,
            )

            if status == "timeout":
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

            raw_result = envelope.get("result")
            command_result = normalize_string_mapping(raw_result)
            if command_result is None:
                subprocess_payload = build_worker_process_payload(
                    process,
                    command_id=command_id,
                    timed_out=False,
                    terminated=False,
                    killed=False,
                    worker_started=worker_started,
                    worker_reused=not worker_started,
                )
                return build_subprocess_failed_result(
                    action=action,
                    backend=backend,
                    stage="gdk_worker_result_invalid",
                    message="GDK worker returned a non-dict command result",
                    safety_gate=safety_gate,
                    subprocess_payload=subprocess_payload,
                )

            command_result["subprocess"] = build_worker_process_payload(
                process,
                command_id=command_id,
                timed_out=False,
                terminated=False,
                killed=False,
                worker_started=worker_started,
                worker_reused=not worker_started,
            )
            return command_result

    def shutdown(
        self,
        timeout_seconds: float = DEFAULT_WORKER_SHUTDOWN_TIMEOUT_SECONDS,
    ) -> dict[str, object]:
        """优雅释放常驻 GDK worker；若 worker 正卡在 GDK 内部，则超时后杀掉。"""

        command_id = str(uuid4())
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

            status, envelope = self._wait_for_result_locked(
                command_id=command_id,
                process=process,
                result_queue=result_queue,
                timeout_seconds=timeout_seconds,
            )
            process.join(0.1)
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

    def _ensure_started_locked(self) -> tuple[Any, Any, Any, bool]:
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

    def _wait_for_result_locked(
        self,
        *,
        command_id: str,
        process: Any,
        result_queue: Any,
        timeout_seconds: float,
    ) -> tuple[str, dict[str, object] | None]:
        pending = self._pending_results.pop(command_id, None)
        if pending is not None:
            return "ok", pending

        deadline = time.monotonic() + timeout_seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return "timeout", None

            try:
                raw_envelope = result_queue.get(timeout=min(remaining, 0.2))
            except Empty:
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
            self._pending_results[raw_command_id] = envelope

    def _terminate_worker_locked(
        self,
        *,
        command_id: str,
        timeout_seconds: float,
        worker_started: bool,
    ) -> dict[str, object]:
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
            timed_out=True,
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


def run_motion_abs_joint_in_worker(
    motion_params: object,
    *,
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
    return get_default_gdk_worker_manager().run_command(
        kind="motion_abs_joint",
        payload={"motion_params": motion_params},
        action="taskflow_abs_joint",
        backend=GDK_BACKEND,
        timeout_seconds=read_timeout_seconds(motion_params),
        safety_gate=safety_gate,
    )


def run_end_effector_in_worker(
    end_effector_params: object,
    *,
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
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


def shutdown_default_gdk_worker(
    timeout_seconds: float = DEFAULT_WORKER_SHUTDOWN_TIMEOUT_SECONDS,
) -> dict[str, object]:
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
    return manager.shutdown(timeout_seconds=timeout_seconds)


def get_default_gdk_worker_manager() -> GdkWorkerProcessManager:
    global _DEFAULT_GDK_WORKER_MANAGER
    if _DEFAULT_GDK_WORKER_MANAGER is None:
        _DEFAULT_GDK_WORKER_MANAGER = GdkWorkerProcessManager()
    return _DEFAULT_GDK_WORKER_MANAGER


def gdk_worker_main(command_queue: Any, result_queue: Any) -> None:
    state = WorkerGdkState()
    while True:
        raw_command = command_queue.get()
        command = normalize_string_mapping(raw_command)
        if command is None:
            continue

        command_id = command.get("command_id")
        if not isinstance(command_id, str):
            continue

        kind = command.get("kind")
        if not isinstance(kind, str):
            result_queue.put(
                {
                    "command_id": command_id,
                    "result": worker_command_failed_result(
                        command=command,
                        stage="validate_worker_command",
                        message="GDK worker command missing string kind",
                    ),
                }
            )
            continue

        if kind == "shutdown":
            result_queue.put(
                {
                    "command_id": command_id,
                    "result": execute_shutdown_command(state),
                }
            )
            return

        try:
            result = execute_worker_command(kind, command, state)
        except Exception as error:
            result = worker_command_failed_result(
                command=command,
                stage="execute_worker_command",
                message=str(error),
                error_type=type(error).__name__,
            )
        result_queue.put({"command_id": command_id, "result": result})


def execute_worker_command(
    kind: str,
    command: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    payload = read_mapping_field(command, "payload") or {}
    if kind == "motion_abs_joint":
        return execute_motion_abs_joint_command(payload, state)
    if kind == "end_effector":
        return execute_end_effector_command(payload, state)
    if kind == "code_script":
        return execute_code_script_command(payload, state)
    return worker_command_failed_result(
        command=command,
        stage="validate_worker_command",
        message=f"unsupported GDK worker command kind: {kind}",
    )


def execute_motion_abs_joint_command(
    payload: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    from gsa_taskflow_executor.gdk import motion_runtime
    from gsa_taskflow_executor.taskflow.parser import MotionPlanParams

    init_error = ensure_gdk_ready_for_motion(state)
    if init_error is not None:
        result = init_error
    else:
        try:
            agibot_gdk = state.require_agibot_gdk()
            robot = agibot_gdk.Robot()
            result = motion_runtime.execute_abs_joint_targets(
                robot,
                cast(MotionPlanParams, payload["motion_params"]),
                agibot_gdk=agibot_gdk,
            )
        except motion_runtime.UnsupportedGdkControlModeError as error:
            result = motion_runtime.refused_control_mode_result(error)
        except Exception as error:
            result = motion_runtime.unavailable_result("execute_abs_joint_targets", error)

    attach_worker_gdk_payload(
        result,
        purpose="taskflow_abs_joint",
        init_result=state.init_result,
    )
    return result


def execute_end_effector_command(
    payload: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    from gsa_taskflow_executor.gdk import end_effector_runtime
    from gsa_taskflow_executor.taskflow.parser import EndEffectorParams

    init_error = ensure_gdk_ready_for_end_effector(state)
    if init_error is not None:
        result = init_error
    else:
        try:
            agibot_gdk = state.require_agibot_gdk()
            robot = agibot_gdk.Robot()
            result = end_effector_runtime.execute_end_effector_control(
                robot,
                cast(EndEffectorParams, payload["end_effector_params"]),
                agibot_gdk=agibot_gdk,
            )
        except Exception as error:
            result = end_effector_runtime.unavailable_result(
                "execute_end_effector_control",
                error,
            )

    attach_worker_gdk_payload(
        result,
        purpose="taskflow_end_effector",
        init_result=state.init_result,
    )
    return result


def execute_code_script_command(
    payload: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    from gsa_taskflow_executor.code_scripts.api import CodeScriptContext
    from gsa_taskflow_executor.code_scripts.registry import CODE_SCRIPT_DEFINITIONS
    from gsa_taskflow_executor.code_scripts.runtime import load_script_runner, run_script_safely
    from gsa_taskflow_executor.taskflow.parser import ScriptParams

    script_id = payload.get("script_id")
    if not isinstance(script_id, str):
        return worker_command_failed_result(
            command={
                "action": "code_script",
                "backend": "executor_code_script",
                "safety_gate": {"enabled": True, "confirmed": True},
            },
            stage="validate_worker_command",
            message="code script worker command missing script_id",
        )

    environ = read_string_mapping(payload.get("environ"))
    os.environ.update(environ)
    definition = CODE_SCRIPT_DEFINITIONS[script_id]
    script_runner = load_script_runner(definition, import_module=importlib.import_module)
    if isinstance(script_runner, dict):
        return script_runner

    typed_script_params = cast(ScriptParams, payload["script_params"])
    inputs = read_mapping_field(payload, "inputs") or {}
    init_error = ensure_gdk_ready_for_code_script(script_id, state)
    if init_error is not None:
        result = init_error
    else:
        try:
            agibot_gdk = state.require_agibot_gdk()
            robot = agibot_gdk.Robot()
            context = CodeScriptContext(
                script_id=definition.script_id,
                description=definition.description,
                timeout=read_timeout_seconds(typed_script_params),
                output_variables=typed_script_params.output_variables,
                environ=environ,
                agibot_gdk=agibot_gdk,
                robot=robot,
                gdk_init=state.init_result,
                gdk_session=build_worker_session_payload(
                    purpose=f"code_script:{script_id}",
                    init_result=state.init_result,
                ),
            )
            result = run_script_safely(
                definition=definition,
                script_runner=script_runner,
                inputs=inputs,
                context=context,
                safety_gate_enabled=True,
                safety_confirmed=True,
            )
        except Exception as error:
            from gsa_taskflow_executor.code_scripts.api import unavailable_result

            result = unavailable_result(
                script_id=script_id,
                stage="import_or_execute_gdk_script",
                error=error,
                safety_gate_enabled=True,
                safety_confirmed=True,
            )

    attach_worker_gdk_payload(
        result,
        purpose=f"code_script:{script_id}",
        init_result=state.init_result,
    )
    return result


def execute_shutdown_command(state: WorkerGdkState) -> dict[str, object]:
    release_result = default_gdk_release_result(reason="not_initialized")
    if state.agibot_gdk is not None and state.gdk_initialized:
        release_result = release_gdk(state.agibot_gdk)
    return {
        "available": True,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": GDK_WORKER_SHUTDOWN_ACTION,
        "collected_at": utc_now_iso(),
        "gdk_init": dict(state.init_result),
        "gdk_release": release_result,
        "gdk_session": build_worker_session_payload(
            purpose=GDK_WORKER_SHUTDOWN_ACTION,
            init_result=state.init_result,
        ),
    }


def ensure_gdk_ready_for_motion(state: WorkerGdkState) -> dict[str, object] | None:
    from gsa_taskflow_executor.gdk import motion_runtime

    init_error = ensure_gdk_ready(state)
    if init_error is None:
        return None
    if init_error["stage"] == "gdk_init":
        return motion_runtime.refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            extra={"gdk_init": state.init_result},
        )
    return motion_runtime.unavailable_result(
        "import_or_initialize_gdk",
        cast(Exception, init_error["error"]),
    )


def ensure_gdk_ready_for_end_effector(state: WorkerGdkState) -> dict[str, object] | None:
    from gsa_taskflow_executor.gdk import end_effector_runtime

    init_error = ensure_gdk_ready(state)
    if init_error is None:
        return None
    if init_error["stage"] == "gdk_init":
        return end_effector_runtime.refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            safety_confirmed=True,
            extra={"gdk_init": state.init_result},
        )
    return end_effector_runtime.unavailable_result(
        "import_or_execute_end_effector",
        cast(Exception, init_error["error"]),
    )


def ensure_gdk_ready_for_code_script(
    script_id: str,
    state: WorkerGdkState,
) -> dict[str, object] | None:
    from gsa_taskflow_executor.code_scripts.api import refused_result, unavailable_result

    init_error = ensure_gdk_ready(state)
    if init_error is None:
        return None
    if init_error["stage"] == "gdk_init":
        return refused_result(
            script_id=script_id,
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            extra={"gdk_init": state.init_result},
            safety_gate_enabled=True,
            safety_confirmed=True,
        )
    return unavailable_result(
        script_id=script_id,
        stage="import_or_execute_gdk_script",
        error=cast(Exception, init_error["error"]),
        safety_gate_enabled=True,
        safety_confirmed=True,
    )


def ensure_gdk_ready(state: WorkerGdkState) -> dict[str, object] | None:
    if state.agibot_gdk is None:
        try:
            state.agibot_gdk = importlib.import_module(GDK_MODULE_NAME)
        except Exception as error:
            return {"stage": "import_agibot_gdk", "error": error}

    if not state.init_attempted:
        try:
            state.init_result = initialize_gdk(state.agibot_gdk)
        except Exception as error:
            state.init_result = default_gdk_init_result()
            return {"stage": "gdk_init_exception", "error": error}

        if (
            state.init_result.get("called") is True
            and state.init_result.get("success") is not True
        ):
            state.init_attempted = False
            state.gdk_initialized = False
            return {"stage": "gdk_init", "error": RuntimeError("gdk_init failed")}

        state.init_attempted = True
        state.gdk_initialized = bool(state.init_result.get("called"))

    return None


def attach_worker_gdk_payload(
    result: dict[str, object],
    *,
    purpose: str,
    init_result: Mapping[str, object],
) -> None:
    result.setdefault("gdk_init", dict(init_result))
    result.setdefault(
        "gdk_release",
        default_gdk_release_result(reason="persistent_worker_releases_on_shutdown"),
    )
    result["gdk_session"] = build_worker_session_payload(
        purpose=purpose,
        init_result=init_result,
    )


def build_worker_session_payload(
    *,
    purpose: str,
    init_result: Mapping[str, object],
) -> dict[str, object]:
    return {
        "policy": GDK_WORKER_POLICY,
        "purpose": purpose,
        "pid": os.getpid(),
        "initialize": True,
        "init_result": dict(init_result),
    }


def build_worker_process_payload(
    process: Any | None,
    *,
    command_id: str | None,
    timed_out: bool,
    terminated: bool,
    killed: bool,
    worker_started: bool,
    worker_reused: bool,
) -> dict[str, object]:
    return {
        "policy": GDK_WORKER_POLICY,
        "pid": None if process is None else process.pid,
        "exitcode": None if process is None else to_jsonable(process.exitcode),
        "command_id": command_id,
        "timed_out": timed_out,
        "terminated": terminated,
        "killed": killed,
        "worker_started": worker_started,
        "worker_reused": worker_reused,
    }


def worker_command_failed_result(
    *,
    command: Mapping[str, object],
    stage: str,
    message: str,
    error_type: str = "GdkWorkerCommandError",
) -> dict[str, object]:
    action = command.get("action")
    backend = command.get("backend")
    safety_gate = read_mapping_field(command, "safety_gate") or {}
    result = build_subprocess_failed_result(
        action=action if isinstance(action, str) else "gdk_worker_command",
        backend=backend if isinstance(backend, str) else GDK_BACKEND,
        stage=stage,
        message=message,
        safety_gate=safety_gate,
        subprocess_payload={},
    )
    result["error_code"] = GDK_SUBPROCESS_FAILED_CODE
    result["error_type"] = error_type
    result.pop("subprocess", None)
    return result


def default_gdk_init_result() -> dict[str, object]:
    return {"called": False, "success": True, "return": None}


def default_gdk_release_result(*, reason: str) -> dict[str, object]:
    return {
        "called": False,
        "success": True,
        "return": None,
        "reason": reason,
    }


def normalize_string_mapping(value: object) -> dict[str, object] | None:
    if not isinstance(value, Mapping):
        return None
    result: dict[str, object] = {}
    for key, item in value.items():
        if isinstance(key, str):
            result[key] = item
    return result


def read_mapping_field(
    mapping: Mapping[str, object] | None,
    key: str,
) -> dict[str, object] | None:
    if mapping is None:
        return None
    return normalize_string_mapping(mapping.get(key))


def read_string_mapping(value: object) -> dict[str, str]:
    if not isinstance(value, Mapping):
        return {}
    result: dict[str, str] = {}
    for key, item in value.items():
        if isinstance(key, str) and isinstance(item, str):
            result[key] = item
    return result


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


_DEFAULT_GDK_WORKER_MANAGER: GdkWorkerProcessManager | None = None
