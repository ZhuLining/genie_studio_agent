"""GDK 子进程运行时 — 在可 kill 的子进程中执行 GDK C 扩展调用。

GDK 控制调用是同步 C 扩展，Python 线程超时无法安全中断。
父进程维持 MQTT/调度器存活，子进程作为故障隔离边界。
timeout 时 terminate → kill → 返回 timeout 结果。

注: 此文件用于 camera/frame/probe 等一次性操作；常驻 control 操作已迁移到 worker_runtime。
"""

from __future__ import annotations

import importlib
import os
from collections.abc import Callable, Mapping
from multiprocessing import get_context
from queue import Empty
from typing import Any, cast

from gsa_taskflow_executor.gdk.control_probe import initialize_gdk, release_gdk, utc_now_iso
from gsa_taskflow_executor.gdk.readonly import GDK_MODULE_NAME, to_jsonable

GDK_OPERATION_TIMEOUT_CODE = "GDK_OPERATION_TIMEOUT"
GDK_SUBPROCESS_FAILED_CODE = "GDK_SUBPROCESS_FAILED"
GDK_SUBPROCESS_POLICY = "subprocess_per_operation"
GDK_PARENT_LOCK_POLICY = "parent_process_operation_lock"
DEFAULT_TERMINATE_GRACE_SECONDS = 2.0

ChildTarget = Callable[..., None]


def run_gdk_subprocess(
    *,
    operation: str,
    action: str,
    backend: str,
    timeout_seconds: float,
    child_target: ChildTarget,
    child_args: tuple[object, ...],
    safety_gate: Mapping[str, object],
    terminate_grace_seconds: float = DEFAULT_TERMINATE_GRACE_SECONDS,
) -> dict[str, object]:
    """Run one GDK operation in a killable child process.

    GDK control calls are C-extension synchronous calls. A Python thread timeout
    cannot safely interrupt them, so the parent keeps MQTT/scheduler alive and
    treats the child process as the failure boundary.
    """

    ctx = get_context("spawn")
    result_queue = ctx.Queue(maxsize=1)
    process = ctx.Process(
        target=child_target,
        args=(result_queue, *child_args),
        name=f"gdk-{operation}",
    )
    process.daemon = True
    process.start()
    process.join(timeout_seconds)

    if process.is_alive():
        subprocess_payload = terminate_timed_out_process(
            process,
            timeout_seconds=timeout_seconds,
            terminate_grace_seconds=terminate_grace_seconds,
        )
        close_queue(result_queue)
        return build_timeout_result(
            action=action,
            backend=backend,
            timeout_seconds=timeout_seconds,
            safety_gate=safety_gate,
            subprocess_payload=subprocess_payload,
        )

    subprocess_payload = build_subprocess_payload(
        process,
        timed_out=False,
        terminated=False,
        killed=False,
    )
    try:
        result = result_queue.get(timeout=1.0)
    except Empty:
        result = build_subprocess_failed_result(
            action=action,
            backend=backend,
            stage="gdk_subprocess_result_missing",
            message="GDK child process exited without returning a result",
            safety_gate=safety_gate,
            subprocess_payload=subprocess_payload,
        )
    finally:
        close_queue(result_queue)
        close_process(process)

    if not isinstance(result, dict):
        return build_subprocess_failed_result(
            action=action,
            backend=backend,
            stage="gdk_subprocess_result_invalid",
            message="GDK child process returned a non-dict result",
            safety_gate=safety_gate,
            subprocess_payload=subprocess_payload,
        )

    payload = dict(result)
    payload["subprocess"] = subprocess_payload
    return payload


def run_motion_abs_joint_in_subprocess(
    motion_params: object,
    *,
    action: str,
    safety_gate: Mapping[str, object],
    abs_pose_limits: object | None = None,
) -> dict[str, object]:
    from gsa_taskflow_executor.gdk.worker_runtime import run_motion_abs_joint_in_worker

    return run_motion_abs_joint_in_worker(
        motion_params,
        action=action,
        safety_gate=safety_gate,
        abs_pose_limits=abs_pose_limits,
    )


def run_end_effector_in_subprocess(
    end_effector_params: object,
    *,
    safety_gate: Mapping[str, object],
    prefer_servo: bool = False,
) -> dict[str, object]:
    from gsa_taskflow_executor.gdk.worker_runtime import run_end_effector_in_worker

    return run_end_effector_in_worker(
        end_effector_params,
        safety_gate=safety_gate,
        prefer_servo=prefer_servo,
    )


def run_code_script_in_subprocess(
    script_params: object,
    *,
    script_id: str,
    inputs: Mapping[str, object],
    environ: Mapping[str, str],
    safety_gate: Mapping[str, object],
) -> dict[str, object]:
    from gsa_taskflow_executor.gdk.worker_runtime import run_code_script_in_worker

    return run_code_script_in_worker(
        script_params,
        script_id=script_id,
        inputs=inputs,
        environ=environ,
        safety_gate=safety_gate,
    )


def motion_abs_joint_child(result_queue: Any, motion_params: object) -> None:
    from gsa_taskflow_executor.gdk import motion_runtime
    from gsa_taskflow_executor.taskflow.models import MotionPlanParams

    agibot_gdk = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    result: dict[str, object]
    action = motion_runtime.motion_action(cast(MotionPlanParams, motion_params))
    try:
        agibot_gdk = importlib.import_module(GDK_MODULE_NAME)
        init_result = initialize_gdk(agibot_gdk)
        if init_result.get("called") is True and init_result.get("success") is not True:
            result = motion_runtime.refused_result(
                stage="gdk_init",
                message="agibot_gdk.gdk_init() did not return success",
                extra={"gdk_init": init_result},
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            robot = agibot_gdk.Robot()
            try:
                result = motion_runtime.execute_motion_plan_targets(
                    robot,
                    cast(MotionPlanParams, motion_params),
                    agibot_gdk=agibot_gdk,
                )
            except motion_runtime.UnsupportedGdkControlModeError as error:
                result = motion_runtime.refused_control_mode_result(error)
            except Exception as error:
                result = motion_runtime.unavailable_result(
                    "execute_motion_plan_targets",
                    error,
                    action=action,
                )
    except Exception as error:
        result = motion_runtime.unavailable_result(
            "import_or_initialize_gdk",
            error,
            action=action,
        )
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            result["gdk_release"] = release_gdk(agibot_gdk)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        result["gdk_session"] = build_child_session_payload(
            purpose=str(result.get("action") or "taskflow_abs_joint"),
            init_result=init_result,
        )
        result_queue.put(result)


def end_effector_child(
    result_queue: Any,
    end_effector_params: object,
    prefer_servo: bool = False,
) -> None:
    from gsa_taskflow_executor.gdk import end_effector_runtime
    from gsa_taskflow_executor.taskflow.models import EndEffectorParams

    agibot_gdk = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    result: dict[str, object]
    try:
        agibot_gdk = importlib.import_module(GDK_MODULE_NAME)
        init_result = initialize_gdk(agibot_gdk)
        if init_result.get("called") is True and init_result.get("success") is not True:
            result = end_effector_runtime.refused_result(
                stage="gdk_init",
                message="agibot_gdk.gdk_init() did not return success",
                safety_confirmed=True,
                extra={"gdk_init": init_result},
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            robot = agibot_gdk.Robot()
            result = end_effector_runtime.execute_end_effector_control(
                robot,
                cast(EndEffectorParams, end_effector_params),
                agibot_gdk=agibot_gdk,
                prefer_servo=prefer_servo,
            )
    except Exception as error:
        result = end_effector_runtime.unavailable_result(
            "import_or_execute_end_effector",
            error,
        )
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            result["gdk_release"] = release_gdk(agibot_gdk)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        result["gdk_session"] = build_child_session_payload(
            purpose="taskflow_end_effector",
            init_result=init_result,
        )
        result_queue.put(result)


def code_script_child(
    result_queue: Any,
    script_params: object,
    script_id: str,
    inputs: Mapping[str, object],
    environ: Mapping[str, str],
) -> None:
    from gsa_taskflow_executor.code_scripts.api import CodeScriptContext, refused_result
    from gsa_taskflow_executor.code_scripts.registry import CODE_SCRIPT_DEFINITIONS
    from gsa_taskflow_executor.code_scripts.runtime import load_script_runner, run_script_safely
    from gsa_taskflow_executor.taskflow.models import ScriptParams

    os.environ.update(dict(environ))
    typed_script_params = cast(ScriptParams, script_params)
    definition = CODE_SCRIPT_DEFINITIONS[script_id]
    script_runner = load_script_runner(definition, import_module=importlib.import_module)
    if isinstance(script_runner, dict):
        result_queue.put(script_runner)
        return

    agibot_gdk = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    result: dict[str, object]
    try:
        agibot_gdk = importlib.import_module(GDK_MODULE_NAME)
        init_result = initialize_gdk(agibot_gdk)
        if init_result.get("called") is True and init_result.get("success") is not True:
            result = refused_result(
                script_id=script_id,
                stage="gdk_init",
                message="agibot_gdk.gdk_init() did not return success",
                extra={"gdk_init": init_result},
                safety_gate_enabled=True,
                safety_confirmed=True,
            )
        else:
            gdk_initialized = bool(init_result.get("called"))
            robot = agibot_gdk.Robot()
            context = CodeScriptContext(
                script_id=definition.script_id,
                description=definition.description,
                timeout=read_timeout_seconds(typed_script_params),
                output_variables=typed_script_params.output_variables,
                environ=environ,
                agibot_gdk=agibot_gdk,
                robot=robot,
                gdk_init=init_result,
                gdk_session=build_child_session_payload(
                    purpose=f"code_script:{script_id}",
                    init_result=init_result,
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
    finally:
        result.setdefault("gdk_init", init_result)
        if agibot_gdk is not None and gdk_initialized:
            result["gdk_release"] = release_gdk(agibot_gdk)
        result.setdefault("gdk_release", {"called": False, "success": True, "return": None})
        result["gdk_session"] = build_child_session_payload(
            purpose=f"code_script:{script_id}",
            init_result=init_result,
        )
        result_queue.put(result)


def terminate_timed_out_process(
    process: Any,
    *,
    timeout_seconds: float,
    terminate_grace_seconds: float,
) -> dict[str, object]:
    terminated = False
    killed = False

    process.terminate()
    terminated = True
    process.join(terminate_grace_seconds)
    if process.is_alive() and hasattr(process, "kill"):
        process.kill()
        killed = True
        process.join(terminate_grace_seconds)

    payload = build_subprocess_payload(
        process,
        timed_out=True,
        terminated=terminated,
        killed=killed,
    )
    payload["timeout_seconds"] = timeout_seconds
    payload["terminate_grace_seconds"] = terminate_grace_seconds
    close_process(process)
    return payload


def build_timeout_result(
    *,
    action: str,
    backend: str,
    timeout_seconds: float,
    safety_gate: Mapping[str, object],
    subprocess_payload: Mapping[str, object],
) -> dict[str, object]:
    return {
        "available": False,
        "executed": False,
        "backend": backend,
        "action": action,
        "collected_at": utc_now_iso(),
        "error_stage": "gdk_operation_timeout",
        "error_code": GDK_OPERATION_TIMEOUT_CODE,
        "error_type": "GdkOperationTimeout",
        "error_msg": (
            f"GDK operation {action} exceeded timeout {timeout_seconds:.3f}s; "
            "child process was terminated"
        ),
        "timeout_seconds": timeout_seconds,
        "timeout_semantics": "executor_worker_recovered; robot_controller_cancel_not_guaranteed",
        "safety_gate": dict(safety_gate),
        "raw": {},
        "subprocess": dict(subprocess_payload),
    }


def build_subprocess_failed_result(
    *,
    action: str,
    backend: str,
    stage: str,
    message: str,
    safety_gate: Mapping[str, object],
    subprocess_payload: Mapping[str, object],
) -> dict[str, object]:
    return {
        "available": False,
        "executed": False,
        "backend": backend,
        "action": action,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_code": GDK_SUBPROCESS_FAILED_CODE,
        "error_type": "GdkSubprocessError",
        "error_msg": message,
        "safety_gate": dict(safety_gate),
        "raw": {},
        "subprocess": dict(subprocess_payload),
    }


def build_child_session_payload(
    *,
    purpose: str,
    init_result: Mapping[str, object],
) -> dict[str, object]:
    return {
        "policy": GDK_SUBPROCESS_POLICY,
        "purpose": purpose,
        "pid": os.getpid(),
        "initialize": True,
        "init_result": dict(init_result),
    }


def build_subprocess_payload(
    process: Any,
    *,
    timed_out: bool,
    terminated: bool,
    killed: bool,
) -> dict[str, object]:
    return {
        "policy": GDK_SUBPROCESS_POLICY,
        "pid": process.pid,
        "exitcode": to_jsonable(process.exitcode),
        "timed_out": timed_out,
        "terminated": terminated,
        "killed": killed,
    }


def read_timeout_seconds(params: Any) -> float:
    value = params.timeout
    if isinstance(value, int | float) and not isinstance(value, bool) and value > 0:
        return float(value)
    raise ValueError("GDK operation timeout must be a positive number")


def close_queue(result_queue: Any) -> None:
    result_queue.close()
    result_queue.join_thread()


def close_process(process: Any) -> None:
    close = getattr(process, "close", None)
    if callable(close):
        close()
