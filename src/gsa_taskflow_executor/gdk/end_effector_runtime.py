"""GDK 末端执行器控制运行时。

通过 agibot_gdk.Robot.move_ee_pos() 控制夹爪开合。
支持 omnipicker/dahuan/ctek90d 三种单关节末端（opening 0~1 线性映射）；
o10_t2/o12_t2 等多关节末端暂未适配。
"""

from __future__ import annotations

import importlib
import os
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from gsa_taskflow_executor.taskflow.models import EndEffectorParams

from .control_probe import is_zero_error, utc_now_iso
from .motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from .readonly import GDK_BACKEND, to_jsonable
from .recovery import maybe_mark_gdk_recovery_required, recovery_refused_payload
from .session import (
    PROCESS_MANAGED_RELEASE_RESULT,
    GdkSessionImportError,
    GdkSessionInitError,
    GdkSessionManager,
)
from .subprocess_runtime import GDK_PARENT_LOCK_POLICY, run_end_effector_in_subprocess

ACTION_TASKFLOW_END_EFFECTOR = "taskflow_end_effector"
GDK_END_EFFECTOR_TYPE_UNKNOWN = "GDK_END_EFFECTOR_TYPE_UNKNOWN"
GDK_END_EFFECTOR_TYPE_UNSUPPORTED = "GDK_END_EFFECTOR_TYPE_UNSUPPORTED"

# PDF 示例给出了这些单关节末端的开/合范围；opening=1 表示打开，0 表示闭合。
SINGLE_JOINT_END_EFFECTOR_RANGES = {
    "omnipicker": {"open": -0.785, "closed": 0.0},
    "dahuan": {"open": 0.0, "closed": 0.025},
    "ctek90d": {"open": -0.91, "closed": 0.0},
}
MULTI_JOINT_END_EFFECTOR_TYPES = frozenset({"o10_t2", "o12_t2"})


def run_gdk_end_effector_control(
    end_effector_params: EndEffectorParams,
    *,
    environ: Mapping[str, str] | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """通过 GDK move_ee_pos 控制末端执行器开合。"""

    env = environ if environ is not None else os.environ
    gate_result = check_taskflow_end_effector_safety_gate(env)
    if gate_result is not None:
        return gate_result

    recovery_result = build_recovery_refused_result(env)
    if recovery_result is not None:
        return recovery_result

    if should_use_in_process_runtime(import_module, session_manager, sleep):
        return run_gdk_end_effector_control_in_process(
            end_effector_params,
            import_module=import_module,
            session_manager=session_manager,
            sleep=sleep,
        )

    # 末端 move_ee_pos 是同步 C 扩展调用；父进程只保留互斥锁。
    # 常驻 worker 复用 GDK 初始化，超时时再杀掉并重启该 worker。
    manager = session_manager or GdkSessionManager()
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=False,
            purpose=ACTION_TASKFLOW_END_EFFECTOR,
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
            safety_confirmed=True,
        )

    with lease:
        result = run_end_effector_in_subprocess(
            end_effector_params,
            safety_gate=confirmed_taskflow_safety_gate(),
        )
        maybe_mark_gdk_recovery_required(result, operation=ACTION_TASKFLOW_END_EFFECTOR)
        result["gdk_parent_lock"] = {
            **lease.to_payload(),
            "policy": GDK_PARENT_LOCK_POLICY,
        }
        return result


def run_gdk_end_effector_control_in_process(
    end_effector_params: EndEffectorParams,
    *,
    import_module: Callable[[str], Any] = importlib.import_module,
    session_manager: GdkSessionManager | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    manager = session_manager or GdkSessionManager(import_module=import_module)
    try:
        lease = manager.acquire(
            blocking=True,
            initialize=True,
            purpose=ACTION_TASKFLOW_END_EFFECTOR,
        )
    except GdkSessionImportError as error:
        return unavailable_result("import_agibot_gdk", error.error)
    except GdkSessionInitError as error:
        return refused_result(
            stage="gdk_init",
            message="agibot_gdk.gdk_init() did not return success",
            safety_confirmed=True,
            extra={"gdk_init": error.init_result},
        )
    except Exception as error:
        return unavailable_result("gdk_session_acquire", error)

    if lease is None:
        return refused_result(
            stage="gdk_session_busy",
            message="GDK session is busy",
            safety_confirmed=True,
        )

    with lease:
        result: dict[str, object]
        try:
            if lease.agibot_gdk is None:
                raise RuntimeError("GDK session lease missing initialized module")
            robot = lease.agibot_gdk.Robot()
            result = execute_end_effector_control(
                robot,
                end_effector_params,
                agibot_gdk=lease.agibot_gdk,
                sleep=sleep,
            )
        except Exception as error:
            result = unavailable_result("execute_end_effector_control", error)

        result["gdk_init"] = lease.init_result
        result["gdk_release"] = dict(PROCESS_MANAGED_RELEASE_RESULT)
        result["gdk_session"] = lease.to_payload()
        maybe_mark_gdk_recovery_required(result, operation=ACTION_TASKFLOW_END_EFFECTOR)

    return result


def should_use_in_process_runtime(
    import_module: Callable[[str], Any],
    session_manager: GdkSessionManager | None,
    sleep: Callable[[float], None],
) -> bool:
    if import_module is not importlib.import_module:
        return True
    if sleep is not time.sleep:
        return True
    return (
        session_manager is not None
        and session_manager.import_module is not importlib.import_module
    )


def confirmed_taskflow_safety_gate() -> dict[str, object]:
    return {
        "enabled": True,
        "confirmed": True,
        "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }


def execute_end_effector_control(
    robot: Any,
    end_effector_params: EndEffectorParams,
    *,
    agibot_gdk: Any,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    before_end_state = read_end_state(robot)
    end_effector_type = resolve_end_effector_type(
        end_effector_params.end_effector_type,
        end_effector_params.target_end,
        before_end_state,
    )
    if end_effector_type is None:
        return refused_result(
            stage="resolve_end_effector_type",
            message=(
                "末端型号为空，且 get_end_state() 未返回可识别的末端执行器类型"
            ),
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
            stage="validate_end_effector_type",
            message=(
                "当前开合度 0~1 映射仅支持 omnipicker/dahuan/ctek90d；"
                f"{end_effector_type} 需要补充多关节映射后再开放"
            ),
            safety_confirmed=True,
            extra={
                "error_code": GDK_END_EFFECTOR_TYPE_UNSUPPORTED,
                "target_end": end_effector_params.target_end,
                "end_effector_type": end_effector_type,
                "supported_end_effector_types": sorted(SINGLE_JOINT_END_EFFECTOR_RANGES),
                "known_multi_joint_end_effector_types": sorted(
                    MULTI_JOINT_END_EFFECTOR_TYPES
                ),
                "before_end_state": to_jsonable(before_end_state),
            },
        )

    joint_states = build_gdk_joint_states(
        agibot_gdk,
        group=end_effector_params.target_end,
        target_type=end_effector_type,
        positions=positions,
    )
    move_return = robot.move_ee_pos(joint_states)
    if not is_zero_error(move_return):
        raise RuntimeError(f"move_ee_pos returned {move_return!r}")

    wait_after_command = end_effector_params.post_wait_seconds > 0
    if wait_after_command:
        # move_ee_pos 的返回值只代表命令被 GDK 接收，不保证末端已经完成动作；
        # 这里复刻原 GSA 的指令后等待，避免下游位控立即抢占末端开合动作。
        sleep(end_effector_params.post_wait_seconds)

    after_end_state = read_end_state(robot)
    actual_openness = extract_actual_openness(after_end_state, end_effector_params.target_end)
    actual_openness_source = "gdk_after_end_state"
    if actual_openness is None:
        # 真机 get_end_state() 字段形态仍需继续现场归档；解析不到时保留请求值用于下游联调。
        actual_openness = [end_effector_params.opening]
        actual_openness_source = "requested_opening_fallback"
    return {
        "available": True,
        "executed": True,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
        "collected_at": utc_now_iso(),
        "method": "move_ee_pos",
        "target_end": end_effector_params.target_end,
        "group": end_effector_params.target_end,
        "end_effector_type": end_effector_type,
        "target_type": end_effector_type,
        "opening": end_effector_params.opening,
        "post_wait_seconds": end_effector_params.post_wait_seconds,
        "wait_after_command": wait_after_command,
        "actual_openness": actual_openness,
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
        "timeout": end_effector_params.timeout,
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {
            "before_end_state": to_jsonable(before_end_state),
            "after_end_state": to_jsonable(after_end_state),
        },
    }


def check_taskflow_end_effector_safety_gate(
    env: Mapping[str, str],
) -> dict[str, object] | None:
    if env.get("ENABLE_GDK_CONTROL") != "1":
        return refused_result(
            stage="safety_gate",
            message="ENABLE_GDK_CONTROL must be 1",
            safety_confirmed=False,
        )
    if env.get("CONFIRM_GDK_CONTROL") != TASKFLOW_ABS_JOINT_CONFIRMATION:
        return refused_result(
            stage="safety_gate",
            message="CONFIRM_GDK_CONTROL mismatch",
            safety_confirmed=False,
        )
    return None


def build_recovery_refused_result(env: Mapping[str, str]) -> dict[str, object] | None:
    recovery_result = recovery_refused_payload(env)
    if recovery_result is None:
        return None
    return refused_result(
        stage="gdk_recovery_required",
        message=str(recovery_result["error_msg"]),
        safety_confirmed=True,
        extra={
            **recovery_result,
            "safety_gate": confirmed_taskflow_safety_gate(),
        },
    )


def read_end_state(robot: Any) -> Mapping[str, Any]:
    end_state = robot.get_end_state()
    if not isinstance(end_state, Mapping):
        raise TypeError("robot.get_end_state() did not return a mapping")
    return end_state


def resolve_end_effector_type(
    configured_type: str | None,
    target_end: str,
    end_state: Mapping[str, Any],
) -> str | None:
    """解析末端执行器型号：优先配置值，否则从 get_end_state() 推断。"""
    if configured_type:
        return normalize_end_effector_type(configured_type)
    return infer_end_effector_type(end_state, target_end)


def infer_end_effector_type(
    end_state: Mapping[str, Any],
    target_end: str,
) -> str | None:
    """从 get_end_state() 返回值推断末端型号。按 top-level → nested → list 顺序查找。"""
    side = "left" if target_end == "left_tool" else "right"
    top_level_keys = (
        f"{side}_end_effector_type",
        f"{side}_end_effector_model",
        f"{side}_tool_type",
        f"{side}_tool_model",
        f"{side}_end_type",
        f"{side}_end_model",
    )
    value = first_string_value(end_state, top_level_keys)
    if value is not None:
        return normalize_end_effector_type(value)

    nested_keys = (
        target_end,
        f"{side}_tool",
        f"{side}_end",
        f"{side}_end_effector",
        side,
    )
    for key in nested_keys:
        nested = end_state.get(key)
        if isinstance(nested, Mapping):
            value = first_string_value(
                nested,
                (
                    "target_type",
                    "end_effector_type",
                    "type",
                    "model",
                    "end_effector_model",
                ),
            )
            if value is not None:
                return normalize_end_effector_type(value)

    for list_key in ("states", "end_states", "tools"):
        value = infer_end_effector_type_from_items(end_state.get(list_key), target_end, side)
        if value is not None:
            return value

    return None


def infer_end_effector_type_from_items(
    value: Any,
    target_end: str,
    side: str,
) -> str | None:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return None

    aliases = {target_end, side, f"{side}_tool", f"{side}_end"}
    for item in value:
        if not isinstance(item, Mapping):
            continue
        item_group = first_string_value(item, ("group", "name", "target_end", "end"))
        if item_group not in aliases:
            continue
        item_type = first_string_value(
            item,
            ("target_type", "end_effector_type", "type", "model", "end_effector_model"),
        )
        if item_type is not None:
            return normalize_end_effector_type(item_type)
    return None


def extract_actual_openness(
    end_state: Mapping[str, Any],
    target_end: str,
) -> list[float] | None:
    side = "left" if target_end == "left_tool" else "right"
    nested_keys = (
        target_end,
        f"{side}_tool",
        f"{side}_end",
        f"{side}_end_effector",
        side,
    )
    for key in nested_keys:
        nested = end_state.get(key)
        if isinstance(nested, Mapping):
            value = first_numeric_sequence(
                nested,
                ("actual_openness", "openness", "opening", "open_degree"),
            )
            if value is not None:
                return value

    value = first_numeric_sequence(
        end_state,
        (
            f"{side}_actual_openness",
            f"{side}_openness",
            f"{side}_opening",
            "actual_openness",
        ),
    )
    if value is not None:
        return value

    for list_key in ("states", "end_states", "tools"):
        value = extract_actual_openness_from_items(end_state.get(list_key), target_end, side)
        if value is not None:
            return value
    return None


def extract_actual_openness_from_items(
    value: Any,
    target_end: str,
    side: str,
) -> list[float] | None:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return None

    aliases = {target_end, side, f"{side}_tool", f"{side}_end"}
    for item in value:
        if not isinstance(item, Mapping):
            continue
        item_group = first_string_value(item, ("group", "name", "target_end", "end"))
        if item_group not in aliases:
            continue
        openness = first_numeric_sequence(
            item,
            ("actual_openness", "openness", "opening", "open_degree"),
        )
        if openness is not None:
            return openness
    return None


def first_numeric_sequence(mapping: Mapping[str, Any], keys: Sequence[str]) -> list[float] | None:
    for key in keys:
        value = mapping.get(key)
        numbers = normalize_numeric_sequence(value)
        if numbers is not None:
            return numbers
    return None


def normalize_numeric_sequence(value: Any) -> list[float] | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return [float(value)]
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return None

    numbers: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int | float):
            return None
        numbers.append(float(item))
    return numbers if numbers else None


def first_string_value(mapping: Mapping[str, Any], keys: Sequence[str]) -> str | None:
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def normalize_end_effector_type(value: str) -> str:
    return value.strip().lower()


def build_positions_for_opening(
    end_effector_type: str,
    opening: float,
) -> list[float] | None:
    joint_range = SINGLE_JOINT_END_EFFECTOR_RANGES.get(end_effector_type)
    if joint_range is None:
        return None

    open_position = joint_range["open"]
    closed_position = joint_range["closed"]
    return [closed_position + (open_position - closed_position) * opening]


def build_gdk_joint_states(
    agibot_gdk: Any,
    *,
    group: str,
    target_type: str,
    positions: Sequence[float],
) -> Any:
    joint_states = agibot_gdk.JointStates()
    joint_states.group = group
    joint_states.target_type = target_type
    states = []
    for position in positions:
        state = agibot_gdk.JointState()
        state.position = float(position)
        states.append(state)
    joint_states.states = states
    joint_states.nums = len(states)
    return joint_states


def refused_result(
    *,
    stage: str,
    message: str,
    safety_confirmed: bool,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_type": "GdkEndEffectorRuntimeRefused",
        "error_msg": message,
        "safety_gate": {
            "enabled": True,
            "confirmed": safety_confirmed,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {},
    }
    if extra:
        payload.update(dict(extra))
    return payload


def unavailable_result(stage: str, error: Exception) -> dict[str, object]:
    return {
        "available": False,
        "executed": False,
        "backend": GDK_BACKEND,
        "action": ACTION_TASKFLOW_END_EFFECTOR,
        "collected_at": utc_now_iso(),
        "error_stage": stage,
        "error_type": type(error).__name__,
        "error_msg": str(error),
        "safety_gate": {
            "enabled": True,
            "confirmed": True,
            "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
        },
        "raw": {},
    }
