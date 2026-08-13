from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy

from gsa_taskflow_executor.code_scripts.api import (
    CodeScriptContext,
    read_number_input,
    refused_result,
    success_result,
)
from gsa_taskflow_executor.gdk.control_probe import is_zero_error, utc_now_iso
from gsa_taskflow_executor.gdk.end_effector_runtime import (
    GDK_END_EFFECTOR_TYPE_UNKNOWN,
    GDK_END_EFFECTOR_TYPE_UNSUPPORTED,
    MULTI_JOINT_END_EFFECTOR_TYPES,
    SINGLE_JOINT_END_EFFECTOR_RANGES,
    build_gdk_joint_states,
    build_positions_for_opening,
    extract_actual_openness,
    read_end_state,
    resolve_end_effector_type,
)
from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND, to_jsonable
from gsa_taskflow_executor.taskflow.models import (
    EndEffectorParams,
    TaskflowParseError,
)
from gsa_taskflow_executor.taskflow.skill_params import (
    parse_end_effector_params,
)


def run(inputs: Mapping[str, object], context: CodeScriptContext) -> dict[str, object]:
    end_effector_params = build_end_effector_params_from_inputs(inputs, context)
    if isinstance(end_effector_params, dict):
        return end_effector_params

    agibot_gdk = context.require_agibot_gdk()
    robot = context.require_robot()
    before_end_state = read_end_state(robot)
    end_effector_type = resolve_end_effector_type(
        end_effector_params.end_effector_type,
        end_effector_params.target_end,
        before_end_state,
    )
    if end_effector_type is None:
        return refused_result(
            script_id=context.script_id,
            stage="resolve_end_effector_type",
            message="末端型号为空，且 get_end_state() 未返回可识别的末端执行器类型",
            safety_gate_enabled=True,
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_TYPE_UNKNOWN,
                "target_end": end_effector_params.target_end,
                "before_end_state": to_jsonable(before_end_state),
            },
        )

    positions = build_positions_for_opening(
        end_effector_type,
        end_effector_params.opening,
    )
    if positions is None:
        return refused_result(
            script_id=context.script_id,
            stage="validate_end_effector_type",
            message=(
                "当前开合度 0~1 映射仅支持 omnipicker/dahuan/ctek90d；"
                f"{end_effector_type} 需要补充多关节映射后再开放"
            ),
            safety_gate_enabled=True,
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_TYPE_UNSUPPORTED,
                "target_end": end_effector_params.target_end,
                "end_effector_type": end_effector_type,
                "supported_end_effector_types": sorted(SINGLE_JOINT_END_EFFECTOR_RANGES),
                "known_multi_joint_end_effector_types": sorted(MULTI_JOINT_END_EFFECTOR_TYPES),
                "before_end_state": to_jsonable(before_end_state),
            },
        )

    # 这里是代码节点脚本真正的 GDK 控制主体：独立脚本文件负责构造 JointStates
    # 并调用 move_ee_pos；runtime 只负责白名单、变量注入和进程级 GDK session。
    joint_states = build_gdk_joint_states(
        agibot_gdk,
        group=end_effector_params.target_end,
        target_type=end_effector_type,
        positions=positions,
    )
    move_return = robot.move_ee_pos(joint_states)
    if not is_zero_error(move_return):
        raise RuntimeError(f"move_ee_pos returned {move_return!r}")

    after_end_state = read_end_state(robot)
    actual_openness = extract_actual_openness(after_end_state, end_effector_params.target_end)
    actual_openness_source = "gdk_after_end_state"
    if actual_openness is None:
        # 真机 get_end_state() 字段形态仍需继续现场归档；解析不到时保留请求值用于下游联调。
        actual_openness = [end_effector_params.opening]
        actual_openness_source = "requested_opening_fallback"

    end_effector_detail = {
        "target_end": end_effector_params.target_end,
        "end_effector_type": end_effector_type,
        "opening": end_effector_params.opening,
        "actual_openness": deepcopy(actual_openness),
        "actual_openness_source": actual_openness_source,
        "target_positions": positions,
        "positions_len": len(positions),
        "move_return": to_jsonable(move_return),
    }
    return success_result(
        context,
        action="move_end_effector",
        inputs=inputs,
        outputs={
            "opening": end_effector_params.opening,
            "actual_openness": deepcopy(actual_openness),
            "target_end": end_effector_params.target_end,
            "end_effector_type": end_effector_type,
            "end_effector_result": deepcopy(end_effector_detail),
        },
        backend=GDK_BACKEND,
        safety_gate_enabled=True,
        extra={
            "code_backend": "executor_file_gdk_script",
            "action": "code_move_end_effector",
            "collected_at": utc_now_iso(),
            "method": "move_ee_pos",
            "target_end": end_effector_params.target_end,
            "group": end_effector_params.target_end,
            "end_effector_type": end_effector_type,
            "target_type": end_effector_type,
            "opening": end_effector_params.opening,
            "actual_openness": deepcopy(actual_openness),
            "actual_openness_source": actual_openness_source,
            "target_positions": positions,
            "positions_len": len(positions),
            "joint_states": {
                "group": end_effector_params.target_end,
                "target_type": end_effector_type,
                "nums": len(positions),
                "positions": positions,
            },
            "move_return": to_jsonable(move_return),
            "raw": {
                "before_end_state": to_jsonable(before_end_state),
                "after_end_state": to_jsonable(after_end_state),
            },
        },
    )


def build_end_effector_params_from_inputs(
    inputs: Mapping[str, object],
    context: CodeScriptContext,
) -> EndEffectorParams | dict[str, object]:
    opening = read_number_input(inputs, "opening")
    if opening is None:
        return refused_result(
            script_id=context.script_id,
            stage="validate_inputs",
            message="末端控制代码要求输入参数 opening 为数字",
            extra={"backend": "executor_file_gdk_script"},
            safety_gate_enabled=True,
            safety_confirmed=True,
        )

    target_end = inputs.get("target_end", "left_tool")
    end_effector_type = inputs.get("end_effector_type", "")
    try:
        return parse_end_effector_params(
            {
                "target_end": target_end,
                "end_effector_type": end_effector_type,
                "opening": opening,
                "timeout": context.timeout,
            },
            "code_move_end_effector",
        )
    except TaskflowParseError as error:
        return refused_result(
            script_id=context.script_id,
            stage="validate_inputs",
            message=str(error),
            extra={"backend": "executor_file_gdk_script"},
            safety_gate_enabled=True,
            safety_confirmed=True,
        )
