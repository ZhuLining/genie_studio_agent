"""GDK 恢复门 — timeout/cancel 后阻断控制命令，直到只读位姿确认安全。

工作流程:
1. worker timeout/cancel → mark_gdk_recovery_required()
2. 后续控制命令 → recovery_refused_payload() 返回拒绝
3. get_current_pose 成功 → confirm_gdk_recovery_from_snapshot() 清除恢复标记
4. 后续控制命令放行

可通过 GDK_RECOVERY_REQUIRED_BLOCKS_CONTROL=0 环境变量跳过（调试用）。
"""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from threading import Lock
from typing import Any

from gsa_taskflow_executor.gdk.control_probe import utc_now_iso

# 恢复门相关错误码
GDK_OPERATION_TIMEOUT_CODE = "GDK_OPERATION_TIMEOUT"
GDK_OPERATION_CANCELLED_CODE = "GDK_OPERATION_CANCELLED"
GDK_RECOVERY_REQUIRED_CODE = "GDK_RECOVERY_REQUIRED"
GDK_RECOVERY_CONFIRMATION_CODE = "GDK_RECOVERY_CONFIRMED"


@dataclass(frozen=True)
class GdkRecoveryRequirement:
    """恢复需求记录。标记原因、触发操作、时间戳和来源结果。"""
    reason: str
    operation: str
    marked_at: str
    source_result: dict[str, object]

    def to_payload(self) -> dict[str, object]:
        return {
            "required": True,
            "reason": self.reason,
            "operation": self.operation,
            "marked_at": self.marked_at,
            "source_result": deepcopy(self.source_result),
        }


# 进程级全局状态（线程安全）
_LOCK = Lock()
_RECOVERY_REQUIREMENT: GdkRecoveryRequirement | None = None


def mark_gdk_recovery_required(
    *,
    operation: str,
    reason: str,
    source_result: Mapping[str, object],
) -> GdkRecoveryRequirement:
    """标记需要恢复确认。后续控制命令将被拒绝。"""
    global _RECOVERY_REQUIREMENT
    requirement = GdkRecoveryRequirement(
        reason=reason,
        operation=operation,
        marked_at=utc_now_iso(),
        source_result=dict(source_result),
    )
    with _LOCK:
        _RECOVERY_REQUIREMENT = requirement
    return requirement


def maybe_mark_gdk_recovery_required(
    result: Mapping[str, object],
    *,
    operation: str,
) -> GdkRecoveryRequirement | None:
    """检查 GDK 结果：若为 cancel/timeout 则自动标记恢复需求。"""
    error_code = result.get("error_code")
    subprocess_payload = result.get("subprocess")
    timed_out = (
        isinstance(subprocess_payload, Mapping)
        and subprocess_payload.get("timed_out") is True
    )
    if error_code == GDK_OPERATION_CANCELLED_CODE:
        return mark_gdk_recovery_required(
            operation=operation,
            reason="worker_cancelled",
            source_result=result,
        )
    if error_code == GDK_OPERATION_TIMEOUT_CODE or timed_out:
        return mark_gdk_recovery_required(
            operation=operation,
            reason="worker_timeout",
            source_result=result,
        )
    return None


def current_gdk_recovery_requirement() -> GdkRecoveryRequirement | None:
    """返回当前恢复需求（线程安全）。"""
    with _LOCK:
        return _RECOVERY_REQUIREMENT


def clear_gdk_recovery_requirement(*, reason: str) -> dict[str, object]:
    """清除恢复需求。返回清除结果（含之前的恢复信息）。"""
    global _RECOVERY_REQUIREMENT
    with _LOCK:
        previous = _RECOVERY_REQUIREMENT
        _RECOVERY_REQUIREMENT = None
    return {
        "cleared": previous is not None,
        "reason": reason,
        "cleared_at": utc_now_iso(),
        "previous": previous.to_payload() if previous is not None else None,
    }


def recovery_refused_payload(
    environ: Mapping[str, str] | None = None,
) -> dict[str, object] | None:
    """若当前需要恢复确认，返回拒绝 payload；否则返回 None。

    环境变量 GDK_RECOVERY_REQUIRED_BLOCKS_CONTROL=0 可跳过检查。
    """
    if environ is not None and environ.get("GDK_RECOVERY_REQUIRED_BLOCKS_CONTROL") == "0":
        return None
    requirement = current_gdk_recovery_requirement()
    if requirement is None:
        return None
    return {
        "available": False,
        "executed": False,
        "error_code": GDK_RECOVERY_REQUIRED_CODE,
        "error_stage": "gdk_recovery_required",
        "error_type": "GdkRecoveryRequired",
        "error_msg": (
            "GDK worker was interrupted or timed out; "
            "read-only recovery confirmation is required before the next control command"
        ),
        "gdk_recovery": requirement.to_payload(),
    }


def confirm_gdk_recovery_from_snapshot(snapshot: Mapping[str, Any]) -> dict[str, object] | None:
    """用位姿快照确认恢复安全。snapshot 通过校验时清除恢复标记。"""
    requirement = current_gdk_recovery_requirement()
    if requirement is None:
        return None
    if not is_recovery_safe_snapshot(snapshot):
        return None
    cleared = clear_gdk_recovery_requirement(reason="current_pose_snapshot_available")
    return {
        "confirmed": True,
        "code": GDK_RECOVERY_CONFIRMATION_CODE,
        "confirmed_at": utc_now_iso(),
        "clear_result": cleared,
    }


def is_recovery_safe_snapshot(snapshot: Mapping[str, Any]) -> bool:
    """校验位姿快照是否满足恢复安全条件。

    要求: available=True, busy=False, 无 nonzeroErrorJoints, motionStatus.errorCode 为 0。
    """
    if snapshot.get("available") is not True:
        return False
    if snapshot.get("busy") is True:
        return False
    nonzero_error_joints = snapshot.get("nonzeroErrorJoints")
    if isinstance(nonzero_error_joints, list) and nonzero_error_joints:
        return False

    motion_status = snapshot.get("motionStatus")
    if isinstance(motion_status, Mapping):
        error_code = motion_status.get("errorCode")
        if error_code not in {None, 0, "0"}:
            return False
    return True


def attach_gdk_recovery_payload(
    result: dict[str, object],
    confirmation: Mapping[str, object] | None,
) -> None:
    """将恢复确认信息附加到结果中。"""
    if confirmation is not None:
        result["gdk_recovery"] = deepcopy(dict(confirmation))
