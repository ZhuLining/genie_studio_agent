"""GDK worker 子进程内的命令执行逻辑。

worker_runtime 管父进程里的进程/队列/timeout/cancel；本模块只在 worker 子进程
里处理命令分发、GDK 懒初始化和结果 payload 构建，边界更清楚，也降低误用真机
控制入口的风险。
"""

from __future__ import annotations

import importlib
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, cast

from gsa_taskflow_executor.gdk.control_probe import initialize_gdk, release_gdk, utc_now_iso
from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from gsa_taskflow_executor.gdk.recovery import GDK_OPERATION_CANCELLED_CODE
from gsa_taskflow_executor.gdk.subprocess_runtime import (
    GDK_SUBPROCESS_FAILED_CODE,
    build_subprocess_failed_result,
    read_timeout_seconds,
)

GDK_WORKER_POLICY = "persistent_gdk_worker"
GDK_WORKER_COMMAND_TIMEOUT_SEMANTICS = (
    "worker_process_restarted; robot_controller_cancel_not_guaranteed"
)
GDK_WORKER_SHUTDOWN_ACTION = "gdk_worker_shutdown"


@dataclass
class WorkerGdkState:
    """worker 子进程内的 GDK 状态。懒初始化，失败可重试。"""

    agibot_gdk: Any | None = None
    robot: Any | None = None
    init_attempted: bool = False
    gdk_initialized: bool = False
    init_result: dict[str, object] = field(default_factory=lambda: default_gdk_init_result())

    def require_agibot_gdk(self) -> Any:
        if self.agibot_gdk is None:
            raise RuntimeError("GDK worker missing initialized agibot_gdk module")
        return self.agibot_gdk

    def require_robot(self, progress_path: str | None, action: str) -> Any:
        if self.robot is None:
            from gsa_taskflow_executor.qr_mapping.point_recording_service import (
                write_point_recording_child_progress,
            )

            write_point_recording_child_progress(
                progress_path,
                "robot_create_started",
                action=action,
                reusedWorker=True,
            )
            self.robot = self.require_agibot_gdk().Robot()
            write_point_recording_child_progress(
                progress_path,
                "robot_created",
                action=action,
                reusedWorker=True,
            )
        return self.robot


def gdk_worker_main(command_queue: Any, result_queue: Any) -> None:
    """worker 子进程主循环：从 command_queue 取命令 → 执行 → 结果写入 result_queue。

    支持: motion_abs_joint, end_effector, code_script, shutdown。
    shutdown 命令退出循环并释放 GDK。
    """

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
    """按 kind 分发到具体命令处理函数。"""

    payload = read_mapping_field(command, "payload") or {}
    if kind == "motion_abs_joint":
        return execute_motion_abs_joint_command(payload, state)
    if kind == "end_effector":
        return execute_end_effector_command(payload, state)
    if kind == "code_script":
        return execute_code_script_command(payload, state)
    if kind == "point_recording_snapshot":
        return execute_point_recording_snapshot_command(payload, command, state)
    return worker_command_failed_result(
        command=command,
        stage="validate_worker_command",
        message=f"unsupported GDK worker command kind: {kind}",
    )


def execute_motion_abs_joint_command(
    payload: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    """执行运动规划命令：初始化 GDK → 创建 Robot → 调用 motion_runtime。

    command kind 为历史兼容名；这里允许 motion_runtime 根据 control_type 分派
    ABS_JOINT/ABS_POSE。真机控制边界仍由父进程安全门和 worker timeout 兜住。
    """

    from gsa_taskflow_executor.gdk import motion_runtime
    from gsa_taskflow_executor.taskflow.models import MotionPlanParams

    motion_params = cast(MotionPlanParams, payload["motion_params"])
    action = motion_runtime.motion_action(motion_params)
    init_error = ensure_gdk_ready_for_motion(state, action=action)
    if init_error is not None:
        result = init_error
    else:
        try:
            agibot_gdk = state.require_agibot_gdk()
            robot = agibot_gdk.Robot()
            abs_pose_limits = cast(Mapping[str, object] | None, payload.get("abs_pose_limits"))
            result = motion_runtime.execute_motion_plan_targets(
                robot,
                motion_params,
                agibot_gdk=agibot_gdk,
                abs_pose_limits=abs_pose_limits,
            )
        except motion_runtime.UnsupportedGdkControlModeError as error:
            result = motion_runtime.refused_control_mode_result(error)
        except Exception as error:
            result = motion_runtime.unavailable_result(
                "execute_motion_plan_targets",
                error,
                action=action,
            )

    attach_worker_gdk_payload(
        result,
        purpose=str(result.get("action") or action),
        init_result=state.init_result,
    )
    return result


def execute_end_effector_command(
    payload: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    """执行末端控制命令：初始化 GDK → 创建 Robot → 调用 end_effector_runtime。"""

    from gsa_taskflow_executor.gdk import end_effector_runtime
    from gsa_taskflow_executor.taskflow.models import EndEffectorParams

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
                prefer_servo=payload.get("prefer_servo") is True,
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
    """执行代码脚本命令：加载脚本模块 → 初始化 GDK → 创建 Robot/Context → run_script_safely。"""

    from gsa_taskflow_executor.code_scripts.api import CodeScriptContext
    from gsa_taskflow_executor.code_scripts.registry import CODE_SCRIPT_DEFINITIONS
    from gsa_taskflow_executor.code_scripts.runtime import load_script_runner, run_script_safely
    from gsa_taskflow_executor.taskflow.models import ScriptParams

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
        return script_runner  # load_script_runner 返回错误 dict

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


def execute_point_recording_snapshot_command(
    payload: Mapping[str, object],
    command: Mapping[str, object],
    state: WorkerGdkState,
) -> dict[str, object]:
    """执行点位录制只读采样：复用 worker 内 Robot，按需读取相机帧。

    该命令不执行机器人控制，但读取 Robot 状态仍可能被 GDK C 扩展阻塞；父进程通过
    worker timeout/cancel 负责恢复 executor，不保证取消机器人侧正在等待的 DDS 调用。
    """

    from gsa_taskflow_executor.qr_mapping import point_recording_service

    action = read_string_field(payload, "action") or "point_recording_snapshot"
    progress_path = read_string_field(payload, "progress_path")
    init_error = ensure_gdk_ready(state)
    if init_error is not None:
        error = cast(Exception, init_error["error"])
        result = point_recording_service.unavailable_result(action, str(init_error["stage"]), error)
    else:
        try:
            agibot_gdk = state.require_agibot_gdk()
            robot = state.require_robot(progress_path, action)
            result = point_recording_service.execute_point_recording_snapshot(
                agibot_gdk=agibot_gdk,
                robot=robot,
                action=action,
                arm=read_required_string(payload, "arm"),
                camera_id=read_required_string(payload, "camera_id"),
                timeout_ms=read_int_field(payload, "timeout_ms"),
                include_image=read_bool_field(payload, "include_image"),
                temp_dir=read_required_string(payload, "temp_dir"),
                warmup_seconds=read_float_field(payload, "warmup_seconds"),
                max_motion_mm=read_float_field(payload, "max_motion_mm"),
                max_rotation_deg=read_float_field(payload, "max_rotation_deg"),
                progress_path=progress_path,
            )
        except Exception as error:
            result = point_recording_service.unavailable_result(
                action,
                "execute_point_recording_snapshot",
                error,
            )

    attach_worker_gdk_payload(
        result,
        purpose=action,
        init_result=state.init_result,
    )
    # 保留原始 command 里的安全门，方便现场看清这是只读采样，不是控制动作。
    result.setdefault("safety_gate", read_mapping_field(command, "safety_gate") or {})
    return result


def execute_shutdown_command(state: WorkerGdkState) -> dict[str, object]:
    """执行 shutdown 命令：调用 release_gdk() 释放 GDK 资源。"""

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


def ensure_gdk_ready_for_motion(
    state: WorkerGdkState,
    *,
    action: str,
) -> dict[str, object] | None:
    """确保 GDK 就绪，失败返回 motion_runtime 格式的错误结果。"""

    from gsa_taskflow_executor.gdk import motion_runtime

    init_error = ensure_gdk_ready(state)
    if init_error is None:
        return None
    if init_error["stage"] == "gdk_init":
        return motion_runtime.refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            action=action,
            extra={"gdk_init": state.init_result},
        )
    return motion_runtime.unavailable_result(
        "import_or_initialize_gdk",
        cast(Exception, init_error["error"]),
        action=action,
    )


def ensure_gdk_ready_for_end_effector(state: WorkerGdkState) -> dict[str, object] | None:
    """确保 GDK 就绪，失败返回 end_effector_runtime 格式的错误结果。"""

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
    """确保 GDK 就绪，失败返回 code_scripts 格式的错误结果。"""

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
    """通用 GDK 初始化守卫：import agibot_gdk → gdk_init()。

    初始化失败时 init_attempted 保持 False，允许下次命令重试。
    返回 None 表示就绪；返回 dict 表示失败（含 stage 和 error）。
    """

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
    """给结果 dict 附加 GDK 初始化/释放/会话信息。"""

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
    """构建 worker 会话信息 payload。"""

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
    """构建 worker 进程状态 payload（pid/exitcode/终止标记）。"""

    pid: object | None = None
    exitcode: object | None = None
    if process is not None:
        try:
            pid = process.pid
            exitcode = to_jsonable(process.exitcode)
        except ValueError:
            # process.close() 后 pid/exitcode 拒绝访问（cancel 路径已记录了 terminated/killed）
            pid = None
            exitcode = None
    return {
        "policy": GDK_WORKER_POLICY,
        "pid": pid,
        "exitcode": exitcode,
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
    """构建 worker 命令失败的标准结果。"""

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


def build_cancelled_result(
    *,
    action: str,
    backend: str,
    reason: str,
    safety_gate: Mapping[str, object],
    subprocess_payload: Mapping[str, object],
) -> dict[str, object]:
    """构建取消结果（GDK_OPERATION_CANCELLED_CODE 错误码）。"""

    return {
        "available": False,
        "executed": False,
        "backend": backend,
        "action": action,
        "collected_at": utc_now_iso(),
        "error_code": GDK_OPERATION_CANCELLED_CODE,
        "error_stage": "gdk_worker_cancelled",
        "error_type": "GdkWorkerCommandCancelled",
        "error_msg": f"GDK worker command cancelled: {reason}",
        "cancelled": True,
        "cancel_reason": reason,
        "timeout_semantics": GDK_WORKER_COMMAND_TIMEOUT_SEMANTICS,
        "safety_gate": dict(safety_gate),
        "subprocess": dict(subprocess_payload),
    }


def default_gdk_init_result() -> dict[str, object]:
    """默认 GDK 初始化结果（未调用）。"""

    return {"called": False, "success": True, "return": None}


def default_gdk_release_result(*, reason: str) -> dict[str, object]:
    """默认 GDK 释放结果（未调用）。"""

    return {
        "called": False,
        "success": True,
        "return": None,
        "reason": reason,
    }


def normalize_string_mapping(value: object) -> dict[str, object] | None:
    """将 Mapping 转为纯字符串 key 的 dict。非 Mapping 返回 None。"""

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
    """从 Mapping 读取并归一化指定 key 的 dict 值。"""

    if mapping is None:
        return None
    return normalize_string_mapping(mapping.get(key))


def read_string_field(
    mapping: Mapping[str, object] | None,
    key: str,
) -> str | None:
    """从 Mapping 读取字符串字段。"""

    if mapping is None:
        return None
    value = mapping.get(key)
    return value if isinstance(value, str) and value else None


def read_string_mapping(value: object) -> dict[str, str]:
    """提取纯 str→str 的映射。"""

    if not isinstance(value, Mapping):
        return {}
    result: dict[str, str] = {}
    for key, item in value.items():
        if isinstance(key, str) and isinstance(item, str):
            result[key] = item
    return result


def read_required_string(mapping: Mapping[str, object], key: str) -> str:
    value = read_string_field(mapping, key)
    if value is None:
        raise ValueError(f"worker payload missing string field: {key}")
    return value


def read_int_field(mapping: Mapping[str, object], key: str) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"worker payload field {key} must be int")
    return value


def read_float_field(mapping: Mapping[str, object], key: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"worker payload field {key} must be number")
    return float(value)


def read_bool_field(mapping: Mapping[str, object], key: str) -> bool:
    value = mapping.get(key)
    if not isinstance(value, bool):
        raise ValueError(f"worker payload field {key} must be bool")
    return value
