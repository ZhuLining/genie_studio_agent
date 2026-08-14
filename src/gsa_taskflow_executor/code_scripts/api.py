"""代码脚本 API：上下文、结果工厂和类型安全输入读取器。

脚本通过 CodeScriptContext 访问 agibot_gdk/Robot/环境变量。
success_result/refused_result/unavailable_result 构建标准化结果 dict。
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
from math import isfinite
from typing import Any

from gsa_taskflow_executor.gdk.motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from gsa_taskflow_executor.taskflow.models import ScriptOutputVariable


@dataclass(frozen=True)
class CodeScriptContext:
    """脚本执行上下文。提供 GDK 模块、Robot 实例、超时和环境变量。"""
    script_id: str
    description: str
    timeout: float
    output_variables: Sequence[ScriptOutputVariable]
    environ: Mapping[str, str] | None = None
    agibot_gdk: Any | None = None
    robot: Any | None = None
    gdk_init: Mapping[str, object] | None = None
    gdk_session: Mapping[str, object] | None = None

    def require_agibot_gdk(self) -> Any:
        if self.agibot_gdk is None:
            raise RuntimeError("代码脚本需要 GDK module，但当前上下文未初始化 GDK")
        return self.agibot_gdk

    def require_robot(self) -> Any:
        if self.robot is None:
            raise RuntimeError("代码脚本需要 Robot 实例，但当前上下文未创建 Robot")
        return self.robot


def success_result(
    context: CodeScriptContext,
    *,
    action: str,
    inputs: Mapping[str, object],
    outputs: Mapping[str, object],
    backend: str,
    safety_gate_enabled: bool = False,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """构建脚本执行成功结果。executed=True, available=True。"""
    payload: dict[str, object] = {
        "available": True,
        "executed": True,
        "backend": backend,
        "script_id": context.script_id,
        "script_description": context.description,
        "script_action": action,
        "timeout": context.timeout,
        "inputs": deepcopy(dict(inputs)),
        "outputs": deepcopy(dict(outputs)),
        "safety_gate": build_safety_gate_payload(
            safety_gate_enabled,
            confirmed=True,
        ),
        "raw": {},
    }
    if extra:
        payload.update(deepcopy(dict(extra)))
    return payload


def refused_result(
    *,
    script_id: str,
    stage: str,
    message: str,
    extra: Mapping[str, object] | None = None,
    safety_gate_enabled: bool = True,
    safety_confirmed: bool = False,
) -> dict[str, object]:
    """构建脚本拒绝结果（安全门/校验/白名单拒绝）。executed=False。"""
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": "executor_code_script",
        "script_id": script_id,
        "error_stage": stage,
        "error_type": "CodeScriptRefused",
        "error_msg": message,
        "safety_gate": build_safety_gate_payload(
            safety_gate_enabled,
            confirmed=safety_confirmed,
        ),
    }
    if extra:
        payload.update(dict(extra))
    return payload


def unavailable_result(
    *,
    script_id: str,
    stage: str,
    error: Exception,
    extra: Mapping[str, object] | None = None,
    safety_gate_enabled: bool = True,
    safety_confirmed: bool = False,
) -> dict[str, object]:
    """构建脚本异常结果（执行中异常）。"""
    payload = refused_result(
        script_id=script_id,
        stage=stage,
        message=str(error),
        extra={"error_type": type(error).__name__},
        safety_gate_enabled=safety_gate_enabled,
        safety_confirmed=safety_confirmed,
    )
    if extra:
        payload.update(dict(extra))
    return payload


def build_safety_gate_payload(
    enabled: bool,
    *,
    confirmed: bool,
) -> dict[str, object]:
    """构建安全门 payload。"""
    if not enabled:
        return {
            "enabled": False,
            "confirmed": True,
            "reason": "no_gdk_control",
        }
    return {
        "enabled": True,
        "confirmed": confirmed,
        "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }


# ============================================================
# 类型安全输入读取器（供脚本内部使用）
# ============================================================


def read_actual_openness_input(inputs: Mapping[str, object]) -> float | None:
    """从 inputs 读取 actual_openness（数值序列的第一个元素）。"""
    value = inputs.get("actual_openness")
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray) or not value:
        return None
    return read_finite_float(value[0])


def read_number_input(inputs: Mapping[str, object], name: str) -> float | None:
    """按名称读取数值输入。"""
    return read_finite_float(inputs.get(name))


def read_finite_float(value: Any) -> float | None:
    """将值转为有限浮点数。拒绝 bool、NaN、inf。"""
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    number = float(value)
    return number if isfinite(number) else None


def clamp_opening(value: float) -> float:
    """将开度值限制在 [0, 1] 范围内。"""
    return min(1.0, max(0.0, value))
