"""代码脚本执行运行时。

只接受白名单（CODE_SCRIPT_DEFINITIONS）内的脚本模块，不接受任意 Python 文件路径。
GDK 脚本走 worker 子进程；非 GDK 脚本在 executor 进程内执行。

run_code_script() 是主入口: 查白名单 → load_script_runner → 安全门/恢复门 → 执行。
"""

from __future__ import annotations

import importlib
import os
from collections.abc import Callable, Mapping
from typing import Any, cast

from gsa_taskflow_executor.gdk.motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from gsa_taskflow_executor.gdk.recovery import (
    maybe_mark_gdk_recovery_required,
    recovery_refused_payload,
)
from gsa_taskflow_executor.gdk.session import (
    PROCESS_MANAGED_RELEASE_RESULT,
    GdkSessionImportError,
    GdkSessionInitError,
    GdkSessionManager,
)
from gsa_taskflow_executor.gdk.subprocess_runtime import (
    GDK_PARENT_LOCK_POLICY,
    run_code_script_in_subprocess,
)
from gsa_taskflow_executor.taskflow.models import ScriptParams

from .api import CodeScriptContext, refused_result, unavailable_result
from .registry import CODE_SCRIPT_DEFINITIONS, CodeScriptDefinition

CodeScriptRunner = Callable[[Mapping[str, object], CodeScriptContext], dict[str, object]]
ModuleImporter = Callable[[str], Any]


def run_code_script(
    script_params: ScriptParams,
    *,
    environ: Mapping[str, str] | None = None,
    runner: Callable[..., object] | None = None,
    python_executable: str | None = None,
    session_manager: GdkSessionManager | None = None,
    inputs: Mapping[str, object] | None = None,
    import_module: ModuleImporter = importlib.import_module,
) -> dict[str, object]:
    """执行代码脚本（主入口）。查白名单 → 加载脚本模块 → 按需走 GDK worker。"""
    del runner, python_executable  # 兼容旧签名，不接受任意路径

    definition = CODE_SCRIPT_DEFINITIONS.get(script_params.script_id)
    if definition is None:
        return refused_result(
            script_id=script_params.script_id,
            stage="validate_script",
            message=f"unsupported script_id: {script_params.script_id}",
            safety_gate_enabled=False,
            safety_confirmed=True,
        )

    script_runner = load_script_runner(definition, import_module=import_module)
    if isinstance(script_runner, dict):
        return script_runner  # 加载失败，返回错误 dict

    runtime_inputs = inputs or {}
    if definition.requires_gdk_control:
        return run_gdk_code_script(
            script_params,
            definition=definition,
            script_runner=script_runner,
            environ=environ,
            session_manager=session_manager,
            inputs=runtime_inputs,
        )

    # 非 GDK 脚本：在 executor 进程内直接执行
    context = CodeScriptContext(
        script_id=definition.script_id,
        description=definition.description,
        timeout=script_params.timeout,
        output_variables=script_params.output_variables,
        environ=environ,
    )
    return run_script_safely(
        definition=definition,
        script_runner=script_runner,
        inputs=runtime_inputs,
        context=context,
        safety_gate_enabled=False,
        safety_confirmed=True,
    )


def load_script_runner(
    definition: CodeScriptDefinition,
    *,
    import_module: ModuleImporter,
) -> CodeScriptRunner | dict[str, object]:
    """从白名单模块加载 run() 函数。失败返回错误 dict。"""
    try:
        module = import_module(definition.module)
    except Exception as error:
        return unavailable_result(
            script_id=definition.script_id,
            stage="load_script",
            error=error,
            extra={"script_module": definition.module},
            safety_gate_enabled=definition.requires_gdk_control,
        )

    script_runner = getattr(module, "run", None)
    if not callable(script_runner):
        return refused_result(
            script_id=definition.script_id,
            stage="load_script",
            message=f"code script module missing callable run(): {definition.module}",
            extra={"script_module": definition.module},
            safety_gate_enabled=definition.requires_gdk_control,
        )
    return cast(CodeScriptRunner, script_runner)


def run_gdk_code_script(
    script_params: ScriptParams,
    *,
    definition: CodeScriptDefinition,
    script_runner: CodeScriptRunner,
    environ: Mapping[str, str] | None,
    session_manager: GdkSessionManager | None,
    inputs: Mapping[str, object],
) -> dict[str, object]:
    """执行 GDK 代码脚本：安全门 → 恢复门 → worker 子进程（或测试模式进程内）。"""
    env = environ if environ is not None else os.environ
    gate_result = check_code_script_safety_gate(definition.script_id, env)
    if gate_result is not None:
        return gate_result

    recovery_result = build_recovery_refused_result(definition.script_id, env)
    if recovery_result is not None:
        return recovery_result

    if should_use_in_process_runtime(session_manager):
        return run_gdk_code_script_in_process(
            script_params,
            definition=definition,
            script_runner=script_runner,
            environ=env,
            session_manager=session_manager,
            inputs=inputs,
        )

    # 父进程只做白名单和互斥，GDK 调用在常驻 worker 子进程完成
    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=False,
            purpose=f"code_script:{definition.script_id}",
        )
    except Exception as error:
        return unavailable_result(
            script_id=definition.script_id,
            stage="gdk_session_acquire",
            error=error,
            safety_gate_enabled=True,
            safety_confirmed=True,
        )

    if lease is None:
        return refused_result(
            script_id=definition.script_id,
            stage="gdk_session_busy",
            message="GDK session is busy",
            safety_gate_enabled=True,
            safety_confirmed=True,
        )

    with lease:
        result = run_code_script_in_subprocess(
            script_params,
            script_id=definition.script_id,
            inputs=inputs,
            environ=env,
            safety_gate=confirmed_code_script_safety_gate(),
        )
        maybe_mark_gdk_recovery_required(
            result,
            operation=f"code_script:{definition.script_id}",
        )
        result["gdk_parent_lock"] = {
            **lease.to_payload(),
            "policy": GDK_PARENT_LOCK_POLICY,
        }
        return result


def run_gdk_code_script_in_process(
    script_params: ScriptParams,
    *,
    definition: CodeScriptDefinition,
    script_runner: CodeScriptRunner,
    environ: Mapping[str, str] | None,
    session_manager: GdkSessionManager | None,
    inputs: Mapping[str, object],
) -> dict[str, object]:
    """进程内执行 GDK 代码脚本（测试模式）。"""
    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=True,
            purpose=f"code_script:{definition.script_id}",
        )
    except GdkSessionImportError as error:
        return unavailable_result(
            script_id=definition.script_id,
            stage="import_agibot_gdk",
            error=error.error,
            safety_gate_enabled=True,
            safety_confirmed=True,
        )
    except GdkSessionInitError as error:
        return refused_result(
            script_id=definition.script_id,
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            extra={"gdk_init": error.init_result},
            safety_gate_enabled=True,
            safety_confirmed=True,
        )
    except Exception as error:
        return unavailable_result(
            script_id=definition.script_id,
            stage="gdk_session_acquire",
            error=error,
            safety_gate_enabled=True,
            safety_confirmed=True,
        )

    if lease is None:
        return refused_result(
            script_id=definition.script_id,
            stage="gdk_session_busy",
            message="GDK session is busy",
            safety_gate_enabled=True,
            safety_confirmed=True,
        )

    with lease:
        if lease.agibot_gdk is None:
            return refused_result(
                script_id=definition.script_id,
                stage="gdk_session_acquire",
                message="GDK session lease missing initialized module",
                safety_gate_enabled=True,
                safety_confirmed=True,
            )

        try:
            robot = lease.agibot_gdk.Robot()
        except Exception as error:
            return unavailable_result(
                script_id=definition.script_id,
                stage="create_robot",
                error=error,
                safety_gate_enabled=True,
                safety_confirmed=True,
                extra={"gdk_session": lease.to_payload()},
            )

        context = CodeScriptContext(
            script_id=definition.script_id,
            description=definition.description,
            timeout=script_params.timeout,
            output_variables=script_params.output_variables,
            environ=environ,
            agibot_gdk=lease.agibot_gdk,
            robot=robot,
            gdk_init=lease.init_result,
            gdk_session=lease.to_payload(),
        )
        result = run_script_safely(
            definition=definition,
            script_runner=script_runner,
            inputs=inputs,
            context=context,
            safety_gate_enabled=True,
            safety_confirmed=True,
        )
        result.setdefault("gdk_init", lease.init_result)
        result.setdefault("gdk_release", dict(PROCESS_MANAGED_RELEASE_RESULT))
        result.setdefault("gdk_session", lease.to_payload())
        maybe_mark_gdk_recovery_required(
            result,
            operation=f"code_script:{definition.script_id}",
        )
        return result


def should_use_in_process_runtime(session_manager: GdkSessionManager | None) -> bool:
    """测试 fixture 注入 mock session_manager 时走进程内路径。"""
    return (
        session_manager is not None
        and session_manager.import_module is not importlib.import_module
    )


def confirmed_code_script_safety_gate() -> dict[str, object]:
    return {
        "enabled": True,
        "confirmed": True,
        "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }


def run_script_safely(
    *,
    definition: CodeScriptDefinition,
    script_runner: CodeScriptRunner,
    inputs: Mapping[str, object],
    context: CodeScriptContext,
    safety_gate_enabled: bool,
    safety_confirmed: bool,
) -> dict[str, object]:
    """安全执行脚本：捕获异常并返回标准化结果。校验返回值必须为 dict。"""
    try:
        result = script_runner(inputs, context)
    except Exception as error:
        return unavailable_result(
            script_id=definition.script_id,
            stage="execute_script",
            error=error,
            safety_gate_enabled=safety_gate_enabled,
            safety_confirmed=safety_confirmed,
        )

    if not isinstance(result, dict):
        return refused_result(
            script_id=definition.script_id,
            stage="execute_script",
            message="code script run() did not return a dict",
            safety_gate_enabled=safety_gate_enabled,
            safety_confirmed=safety_confirmed,
        )
    result.setdefault("script_id", definition.script_id)
    result.setdefault("script_description", definition.description)
    result.setdefault("script_module", definition.module)
    return result


def check_code_script_safety_gate(
    script_id: str,
    env: Mapping[str, str],
) -> dict[str, object] | None:
    """检查 GDK 脚本安全门。"""
    if env.get("ENABLE_GDK_CONTROL") != "1":
        return refused_result(
            script_id=script_id,
            stage="safety_gate",
            message="ENABLE_GDK_CONTROL must be 1",
            safety_gate_enabled=True,
            safety_confirmed=False,
        )
    if env.get("CONFIRM_GDK_CONTROL") != TASKFLOW_ABS_JOINT_CONFIRMATION:
        return refused_result(
            script_id=script_id,
            stage="safety_gate",
            message="CONFIRM_GDK_CONTROL mismatch",
            safety_gate_enabled=True,
            safety_confirmed=False,
        )
    return None


def build_recovery_refused_result(
    script_id: str,
    env: Mapping[str, str],
) -> dict[str, object] | None:
    """检查恢复门。"""
    recovery_result = recovery_refused_payload(env)
    if recovery_result is None:
        return None
    return refused_result(
        script_id=script_id,
        stage="gdk_recovery_required",
        message=str(recovery_result["error_msg"]),
        extra={
            **recovery_result,
            "safety_gate": confirmed_code_script_safety_gate(),
        },
        safety_gate_enabled=True,
        safety_confirmed=True,
    )
