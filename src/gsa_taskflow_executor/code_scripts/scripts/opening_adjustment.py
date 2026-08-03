from __future__ import annotations

from collections.abc import Mapping

from gsa_taskflow_executor.code_scripts.api import (
    CodeScriptContext,
    clamp_opening,
    read_actual_openness_input,
    refused_result,
    success_result,
)


def run_opening_adjustment(
    inputs: Mapping[str, object],
    context: CodeScriptContext,
    *,
    delta: float,
) -> dict[str, object]:
    source_opening = read_actual_openness_input(inputs)
    if source_opening is None:
        return refused_result(
            script_id=context.script_id,
            stage="validate_inputs",
            message="开合度计算代码要求输入参数 actual_openness 为非空数字数组",
            extra={"backend": "executor_builtin_code"},
            safety_gate_enabled=False,
            safety_confirmed=True,
        )

    raw_opening = source_opening + delta
    adjusted_opening = clamp_opening(raw_opening)
    return success_result(
        context,
        action="adjust_opening_plus" if delta > 0 else "adjust_opening_minus",
        inputs=inputs,
        outputs={"adjusted_opening": adjusted_opening},
        backend="executor_builtin_code",
        extra={
            "source_opening": source_opening,
            "delta": delta,
            "clamped": adjusted_opening != raw_opening,
        },
    )
