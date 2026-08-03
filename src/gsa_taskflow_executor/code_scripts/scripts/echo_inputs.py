from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy

from gsa_taskflow_executor.code_scripts.api import (
    CodeScriptContext,
    refused_result,
    success_result,
)


def run(inputs: Mapping[str, object], context: CodeScriptContext) -> dict[str, object]:
    outputs: dict[str, object] = {}
    for output_variable in context.output_variables:
        if output_variable.name not in inputs:
            return refused_result(
                script_id=context.script_id,
                stage="validate_outputs",
                message=(
                    "code_echo_inputs 要求输出变量名能在输入映射中找到同名参数: "
                    f"{output_variable.name}"
                ),
                extra={"backend": "executor_builtin_code"},
                safety_gate_enabled=False,
                safety_confirmed=True,
            )
        outputs[output_variable.name] = deepcopy(inputs[output_variable.name])

    return success_result(
        context,
        action="echo_inputs",
        inputs=inputs,
        outputs=outputs,
        backend="executor_builtin_code",
    )
