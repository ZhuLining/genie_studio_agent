from __future__ import annotations

import json
import os
import subprocess
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from gsa_taskflow_executor.taskflow.parser import ScriptParams

from .motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION

SubprocessRunner = Callable[..., subprocess.CompletedProcess[str]]


@dataclass(frozen=True)
class GdkScriptDefinition:
    script_id: str
    module: str
    action: str
    confirmation: str
    description: str


GDK_SCRIPT_DEFINITIONS: dict[str, GdkScriptDefinition] = {
    "gdk_hold_current_dual_arm": GdkScriptDefinition(
        script_id="gdk_hold_current_dual_arm",
        module="gsa_taskflow_executor.gdk.scripts.hold_current_dual_arm",
        action="hold_current",
        confirmation="HOLD_CURRENT_DUAL_ARM",
        description="Hold current dual-arm joint positions.",
    ),
    "gdk_nudge_left_j7_0p005": GdkScriptDefinition(
        script_id="gdk_nudge_left_j7_0p005",
        module="gsa_taskflow_executor.gdk.scripts.nudge_left_j7_0p005",
        action="nudge_left_j7_0p005",
        confirmation="NUDGE_LEFT_J7_0P005",
        description="Nudge left arm J7 by +0.005 rad, then return to origin.",
    ),
    "gdk_nudge_right_j7_0p005": GdkScriptDefinition(
        script_id="gdk_nudge_right_j7_0p005",
        module="gsa_taskflow_executor.gdk.scripts.nudge_right_j7_0p005",
        action="nudge_right_j7_0p005",
        confirmation="NUDGE_RIGHT_J7_0P005",
        description="Nudge right arm J7 by +0.005 rad, then return to origin.",
    ),
}


def run_gdk_script(
    script_params: ScriptParams,
    *,
    environ: Mapping[str, str] | None = None,
    runner: SubprocessRunner = subprocess.run,
    python_executable: str | None = None,
) -> dict[str, object]:
    definition = GDK_SCRIPT_DEFINITIONS.get(script_params.script_id)
    if definition is None:
        return refused_result(
            script_id=script_params.script_id,
            stage="validate_script",
            message=f"unsupported script_id: {script_params.script_id}",
        )

    env = environ if environ is not None else os.environ
    gate_result = check_taskflow_script_safety_gate(script_params.script_id, env)
    if gate_result is not None:
        return gate_result

    runtime_env = dict(env)
    runtime_env["ENABLE_GDK_CONTROL"] = "1"
    runtime_env["CONFIRM_GDK_CONTROL"] = definition.confirmation
    add_source_tree_to_pythonpath(runtime_env)
    command = [
        python_executable or sys.executable,
        "-m",
        definition.module,
    ]

    try:
        completed = runner(
            command,
            env=runtime_env,
            capture_output=True,
            text=True,
            timeout=script_params.timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        return unavailable_result(
            script_id=definition.script_id,
            definition=definition,
            stage="execute_script",
            error=error,
            extra={"timeout": script_params.timeout},
        )
    except Exception as error:
        return unavailable_result(
            script_id=definition.script_id,
            definition=definition,
            stage="execute_script",
            error=error,
        )

    if completed.returncode != 0:
        return refused_result(
            script_id=definition.script_id,
            stage="execute_script",
            message=f"script process exited with code {completed.returncode}",
            extra={
                "script_module": definition.module,
                "script_action": definition.action,
                "timeout": script_params.timeout,
                "process": process_summary(completed),
            },
        )

    result = parse_script_stdout(
        stdout=completed.stdout,
        script_id=definition.script_id,
        definition=definition,
        timeout=script_params.timeout,
        process=completed,
    )
    result["script_id"] = definition.script_id
    result["script_module"] = definition.module
    result["script_description"] = definition.description
    result["script_action"] = definition.action
    result["timeout"] = script_params.timeout
    result["safety_gate_source"] = "script_whitelist"
    return result


def check_taskflow_script_safety_gate(
    script_id: str,
    env: Mapping[str, str],
) -> dict[str, object] | None:
    if env.get("ENABLE_GDK_CONTROL") != "1":
        return refused_result(
            script_id=script_id,
            stage="safety_gate",
            message="ENABLE_GDK_CONTROL must be 1",
        )
    if env.get("CONFIRM_GDK_CONTROL") != TASKFLOW_ABS_JOINT_CONFIRMATION:
        return refused_result(
            script_id=script_id,
            stage="safety_gate",
            message="CONFIRM_GDK_CONTROL mismatch",
        )
    return None


def parse_script_stdout(
    *,
    stdout: str,
    script_id: str,
    definition: GdkScriptDefinition,
    timeout: float,
    process: subprocess.CompletedProcess[str],
) -> dict[str, object]:
    for line in reversed(stdout.splitlines()):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            result = dict(parsed)
            result["process"] = process_summary(process)
            return result

    return refused_result(
        script_id=script_id,
        stage="parse_script_output",
        message="script stdout did not contain a JSON object line",
        extra={
            "script_module": definition.module,
            "script_action": definition.action,
            "timeout": timeout,
            "process": process_summary(process),
        },
    )


def process_summary(process: subprocess.CompletedProcess[str]) -> dict[str, object]:
    return {
        "returncode": process.returncode,
        "stdout_tail": tail_lines(process.stdout),
        "stderr_tail": tail_lines(process.stderr),
    }


def tail_lines(value: str, limit: int = 20) -> list[str]:
    return value.splitlines()[-limit:]


def add_source_tree_to_pythonpath(env: dict[str, str]) -> None:
    source_root = str(Path(__file__).resolve().parents[1])
    current = env.get("PYTHONPATH")
    if not current:
        env["PYTHONPATH"] = source_root
        return

    paths = current.split(os.pathsep)
    if source_root not in paths:
        env["PYTHONPATH"] = os.pathsep.join([source_root, *paths])


def unavailable_result(
    *,
    script_id: str,
    definition: GdkScriptDefinition,
    stage: str,
    error: Exception,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    payload = refused_result(
        script_id=script_id,
        stage=stage,
        message=str(error),
        extra={
            "script_module": definition.module,
            "script_action": definition.action,
            "error_type": type(error).__name__,
        },
    )
    if extra:
        payload.update(dict(extra))
    return payload


def refused_result(
    *,
    script_id: str,
    stage: str,
    message: str,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": "gdk_script",
        "script_id": script_id,
        "error_stage": stage,
        "error_type": "GdkScriptRefused",
        "error_msg": message,
        "safety_gate": {
            "enabled": True,
            "confirmed": False,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
    }
    if extra:
        payload.update(dict(extra))
    return payload


def script_modules() -> Sequence[str]:
    return tuple(definition.module for definition in GDK_SCRIPT_DEFINITIONS.values())
