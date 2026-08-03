from __future__ import annotations

from collections.abc import Mapping

from gsa_taskflow_executor.code_scripts.api import CodeScriptContext

from .opening_adjustment import run_opening_adjustment


def run(inputs: Mapping[str, object], context: CodeScriptContext) -> dict[str, object]:
    return run_opening_adjustment(inputs, context, delta=-0.1)
