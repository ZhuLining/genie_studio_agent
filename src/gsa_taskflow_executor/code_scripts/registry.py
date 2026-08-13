from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CodeScriptDefinition:
    script_id: str
    module: str
    description: str
    requires_gdk_control: bool = False


CODE_SCRIPT_DEFINITIONS: dict[str, CodeScriptDefinition] = {
    # 白名单只声明固定模块名，代码节点不能接收任意脚本路径。
    "code_echo_inputs": CodeScriptDefinition(
        script_id="code_echo_inputs",
        module="gsa_taskflow_executor.code_scripts.scripts.echo_inputs",
        description="Echo mapped inputs to declared outputs without touching GDK.",
    ),
    "code_opening_plus_0p1": CodeScriptDefinition(
        script_id="code_opening_plus_0p1",
        module="gsa_taskflow_executor.code_scripts.scripts.opening_plus_0p1",
        description="Read actual_openness[0], add 0.1, and output adjusted_opening.",
    ),
    "code_opening_minus_0p1": CodeScriptDefinition(
        script_id="code_opening_minus_0p1",
        module="gsa_taskflow_executor.code_scripts.scripts.opening_minus_0p1",
        description="Read actual_openness[0], subtract 0.1, and output adjusted_opening.",
    ),
    "code_move_end_effector": CodeScriptDefinition(
        script_id="code_move_end_effector",
        module="gsa_taskflow_executor.code_scripts.scripts.move_end_effector",
        description="Move end effector with mapped opening by directly calling move_ee_pos.",
        requires_gdk_control=True,
    ),
}

CODE_SCRIPT_IDS = frozenset(CODE_SCRIPT_DEFINITIONS)
