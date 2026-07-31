from __future__ import annotations

import subprocess

import gsa_taskflow_executor.gdk.script_runtime as gdk_script_runtime
from gsa_taskflow_executor.taskflow.parser import ScriptParams


def test_gdk_script_refuses_before_gdk_import_without_taskflow_safety_gate() -> None:
    result = gdk_script_runtime.run_gdk_script(
        ScriptParams(script_id="gdk_hold_current_dual_arm", timeout=20),
        environ={},
    )

    assert result["executed"] is False
    assert result["error_stage"] == "safety_gate"
    assert result["error_msg"] == "ENABLE_GDK_CONTROL must be 1"


def test_gdk_script_executes_whitelisted_script_module() -> None:
    calls: list[tuple[list[str], dict[str, str], float]] = []

    def fake_runner(
        command: list[str],
        *,
        env: dict[str, str],
        capture_output: bool,
        text: bool,
        timeout: float,
        check: bool,
    ) -> subprocess.CompletedProcess[str]:
        assert capture_output is True
        assert text is True
        assert check is False
        calls.append((command, dict(env), timeout))
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout=(
                "Init glog with processor name:python3.10\n"
                '{"available":true,"executed":true,"action":"hold_current"}\n'
            ),
            stderr="",
        )

    result = gdk_script_runtime.run_gdk_script(
        ScriptParams(script_id="gdk_hold_current_dual_arm", timeout=20),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
        },
        runner=fake_runner,
        python_executable="/opt/gsa_taskflow_executor/.venv/bin/python",
    )

    assert result["executed"] is True
    assert result["script_id"] == "gdk_hold_current_dual_arm"
    assert result["script_module"] == "gsa_taskflow_executor.gdk.scripts.hold_current_dual_arm"
    assert result["script_action"] == "hold_current"
    assert result["timeout"] == 20
    assert calls[0][0] == [
        "/opt/gsa_taskflow_executor/.venv/bin/python",
        "-m",
        "gsa_taskflow_executor.gdk.scripts.hold_current_dual_arm",
    ]
    assert calls[0][1]["ENABLE_GDK_CONTROL"] == "1"
    assert calls[0][1]["CONFIRM_GDK_CONTROL"] == "HOLD_CURRENT_DUAL_ARM"
    assert "gsa_taskflow_executor/src" in calls[0][1]["PYTHONPATH"]
    assert calls[0][2] == 20


def test_gdk_script_rejects_non_json_script_output() -> None:
    def fake_runner(
        command: list[str],
        *,
        env: dict[str, str],
        capture_output: bool,
        text: bool,
        timeout: float,
        check: bool,
    ) -> subprocess.CompletedProcess[str]:
        del env, capture_output, text, timeout, check
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="script completed without json\n",
            stderr="",
        )

    result = gdk_script_runtime.run_gdk_script(
        ScriptParams(script_id="gdk_hold_current_dual_arm", timeout=20),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
        },
        runner=fake_runner,
    )

    assert result["executed"] is False
    assert result["error_stage"] == "parse_script_output"
    assert result["script_module"] == "gsa_taskflow_executor.gdk.scripts.hold_current_dual_arm"


def test_gdk_script_refuses_mismatched_taskflow_confirmation() -> None:
    result = gdk_script_runtime.run_gdk_script(
        ScriptParams(script_id="gdk_hold_current_dual_arm", timeout=20),
        environ={
            "ENABLE_GDK_CONTROL": "1",
            "CONFIRM_GDK_CONTROL": "HOLD_CURRENT_DUAL_ARM",
        },
    )

    assert result["executed"] is False
    assert result["error_msg"] == "CONFIRM_GDK_CONTROL mismatch"
    assert result["safety_gate"]["expected_confirmation"] == "TASKFLOW_ABS_JOINT"
