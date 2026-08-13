"""GDK 力控运行时（硬阻断）。

力控 GDK API 在真机上未验证，所有调用直接返回 refused 结果。
保留参数解析和校验逻辑，开放前只需移除硬阻断。
"""

from __future__ import annotations

from copy import deepcopy

from gsa_taskflow_executor.taskflow.models import ForceControlParams

ACTION_TASKFLOW_FORCE_CONTROL = "taskflow_force_control"
TASKFLOW_FORCE_CONTROL_CONFIRMATION = "TASKFLOW_FORCE_CONTROL"
GDK_FORCE_CONTROL_UNVERIFIED = "GDK_FORCE_CONTROL_UNVERIFIED"


def run_gdk_force_control_unverified(
    force_params: ForceControlParams,
) -> dict[str, object]:
    """Return a hard refusal until the GDK force-control path is verified on robot."""

    # 力控会在闭环中持续读力并发运动指令；在 direct_move/force_position_control
    # 和力读数接口完成真机验证前，runtime 必须只返回错误，不能导入或调用 GDK 控制接口。
    return {
        "available": False,
        "executed": False,
        "backend": "agibot_gdk.Robot",
        "action": ACTION_TASKFLOW_FORCE_CONTROL,
        "error_code": GDK_FORCE_CONTROL_UNVERIFIED,
        "error_stage": "force_control_runtime_unverified",
        "error_msg": "GDK force control runtime is not verified on robot",
        "reason": GDK_FORCE_CONTROL_UNVERIFIED,
        "params": force_params_to_payload(force_params),
        "suggested_gdk_verification": [
            "确认 force_position_control() 或 direct_move() 是否存在且可用于小步相对移动",
            (
                "确认 get_motion_control_status().frame_poses/wrenches "
                "或 get_end_force_state() 的力读数语义"
            ),
            (
                "在独立安全门 ENABLE_GDK_CONTROL=1 + "
                "CONFIRM_GDK_CONTROL=TASKFLOW_FORCE_CONTROL 下做真机小步验证"
            ),
        ],
        "safety_gate": {
            "required": True,
            "expected_confirmation": TASKFLOW_FORCE_CONTROL_CONFIRMATION,
            "shares_abs_joint_gate": False,
        },
    }


def force_params_to_payload(force_params: ForceControlParams) -> dict[str, object]:
    return {
        "method": force_params.method,
        "arm": force_params.arm,
        "delta_xyz": deepcopy(list(force_params.delta_xyz)),
        "force_threshold": force_params.force_threshold,
        "timeout_s": force_params.timeout_s,
        "control_hz": force_params.control_hz,
        "step": force_params.step,
    }
