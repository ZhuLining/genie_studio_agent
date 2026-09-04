"""GDK 恢复门 — timeout/cancel 后阻断控制命令，直到显式多帧确认安全。

工作流程:
1. worker timeout/cancel → mark_gdk_recovery_required()
2. 后续控制命令 → recovery_refused_payload() 返回拒绝
3. confirm_gdk_recovery 多帧只读确认通过 → 清除恢复标记
4. 后续控制命令放行

可通过 GDK_RECOVERY_REQUIRED_BLOCKS_CONTROL=0 环境变量跳过（调试用）。
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Any

from gsa_taskflow_executor.gdk.control_probe import utc_now_iso

# 恢复门相关错误码
GDK_OPERATION_TIMEOUT_CODE = "GDK_OPERATION_TIMEOUT"
GDK_OPERATION_CANCELLED_CODE = "GDK_OPERATION_CANCELLED"
GDK_RECOVERY_REQUIRED_CODE = "GDK_RECOVERY_REQUIRED"
GDK_RECOVERY_CONFIRMATION_CODE = "GDK_RECOVERY_CONFIRMED"
GDK_RECOVERY_NOT_CONFIRMED_CODE = "GDK_RECOVERY_NOT_CONFIRMED"
STOP_UNCONFIRMED_STATE = "STOP_UNCONFIRMED"

DEFAULT_RECOVERY_CONFIRM_SAMPLE_COUNT = 3
DEFAULT_RECOVERY_CONFIRM_SAMPLE_INTERVAL_SECONDS = 0.2
DEFAULT_RECOVERY_MAX_JOINT_VELOCITY = 0.005
DEFAULT_RECOVERY_MAX_POSITION_DELTA = 0.002


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
            "state": STOP_UNCONFIRMED_STATE,
            "robot_stop_confirmed": False,
            "reason": self.reason,
            "operation": self.operation,
            "marked_at": self.marked_at,
            "source_result": deepcopy(self.source_result),
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, object]) -> GdkRecoveryRequirement:
        reason = payload.get("reason")
        operation = payload.get("operation")
        marked_at = payload.get("marked_at")
        source_result = payload.get("source_result")
        if not isinstance(reason, str) or not reason:
            raise ValueError("recovery payload missing reason")
        if not isinstance(operation, str) or not operation:
            raise ValueError("recovery payload missing operation")
        if not isinstance(marked_at, str) or not marked_at:
            raise ValueError("recovery payload missing marked_at")
        return cls(
            reason=reason,
            operation=operation,
            marked_at=marked_at,
            source_result=dict(source_result) if isinstance(source_result, Mapping) else {},
        )


@dataclass(frozen=True)
class GdkRecoveryStabilityPolicy:
    """多帧恢复确认策略。

    取消/超时后，不能用单次 get_current_pose 当作机器人停稳证明。这里要求多帧
    只读快照同时满足无错误、无 active control、低速度和位置稳定，才允许清恢复门。
    """

    sample_count: int = DEFAULT_RECOVERY_CONFIRM_SAMPLE_COUNT
    sample_interval_seconds: float = DEFAULT_RECOVERY_CONFIRM_SAMPLE_INTERVAL_SECONDS
    max_joint_velocity: float = DEFAULT_RECOVERY_MAX_JOINT_VELOCITY
    max_position_delta: float = DEFAULT_RECOVERY_MAX_POSITION_DELTA

    def to_payload(self) -> dict[str, object]:
        return {
            "sample_count": self.sample_count,
            "sample_interval_seconds": self.sample_interval_seconds,
            "max_joint_velocity": self.max_joint_velocity,
            "max_position_delta": self.max_position_delta,
        }


# 进程级全局状态（线程安全）。配置持久化路径后，重启也不会遗失 STOP_UNCONFIRMED。
_LOCK = Lock()
_RECOVERY_REQUIREMENT: GdkRecoveryRequirement | None = None
_RECOVERY_STATE_PATH: Path | None = None
_RECOVERY_STORE_LOADED = False


def configure_gdk_recovery_store(path: str | Path | None) -> None:
    """配置恢复门持久化文件路径，并加载已有 STOP_UNCONFIRMED 状态。

    path=None 用于测试或纯内存模式。真实 executor 启动时由 CLI 指向
    logs/gdk_recovery_state.json，避免进程重启后误放行控制命令。
    """

    global _RECOVERY_REQUIREMENT, _RECOVERY_STATE_PATH, _RECOVERY_STORE_LOADED
    with _LOCK:
        _RECOVERY_STATE_PATH = Path(path).expanduser() if path is not None else None
        _RECOVERY_REQUIREMENT = None
        _RECOVERY_STORE_LOADED = True
        if _RECOVERY_STATE_PATH is not None:
            _RECOVERY_REQUIREMENT = _read_recovery_requirement_from_disk(_RECOVERY_STATE_PATH)


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
        _write_recovery_requirement_to_disk(requirement)
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
    global _RECOVERY_REQUIREMENT, _RECOVERY_STORE_LOADED
    with _LOCK:
        if not _RECOVERY_STORE_LOADED:
            _RECOVERY_REQUIREMENT = (
                _read_recovery_requirement_from_disk(_RECOVERY_STATE_PATH)
                if _RECOVERY_STATE_PATH is not None
                else None
            )
            _RECOVERY_STORE_LOADED = True
        return _RECOVERY_REQUIREMENT


def clear_gdk_recovery_requirement(*, reason: str) -> dict[str, object]:
    """清除恢复需求。返回清除结果（含之前的恢复信息）。"""
    global _RECOVERY_REQUIREMENT
    with _LOCK:
        previous = _RECOVERY_REQUIREMENT
        _RECOVERY_REQUIREMENT = None
        _delete_recovery_requirement_from_disk()
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
        "stop_state": STOP_UNCONFIRMED_STATE,
        "robot_stop_confirmed": False,
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
    """兼容旧调用：单帧只能做基础校验，不再清除恢复标记。"""

    requirement = current_gdk_recovery_requirement()
    if requirement is None:
        return None
    safety = evaluate_recovery_stability(
        [snapshot],
        GdkRecoveryStabilityPolicy(sample_count=1),
    )
    return {
        "confirmed": False,
        "code": GDK_RECOVERY_NOT_CONFIRMED_CODE,
        "state": STOP_UNCONFIRMED_STATE,
        "reason": "single_snapshot_does_not_clear_recovery",
        "requirement": requirement.to_payload(),
        "stability": safety,
    }


def confirm_gdk_recovery_from_snapshots(
    snapshots: list[Mapping[str, Any]],
    policy: GdkRecoveryStabilityPolicy | None = None,
) -> dict[str, object] | None:
    """用多帧位姿快照确认恢复安全。全部通过才清除恢复门。"""

    requirement = current_gdk_recovery_requirement()
    if requirement is None:
        return {
            "confirmed": True,
            "code": GDK_RECOVERY_CONFIRMATION_CODE,
            "state": "CONFIRMED",
            "required": False,
            "already_confirmed": True,
            "confirmed_at": utc_now_iso(),
        }

    resolved_policy = policy or GdkRecoveryStabilityPolicy()
    stability = evaluate_recovery_stability(snapshots, resolved_policy)
    if stability["stable"] is not True:
        return {
            "confirmed": False,
            "code": GDK_RECOVERY_NOT_CONFIRMED_CODE,
            "state": STOP_UNCONFIRMED_STATE,
            "requirement": requirement.to_payload(),
            "stability": stability,
        }

    cleared = clear_gdk_recovery_requirement(reason="multi_frame_current_pose_stable")
    return {
        "confirmed": True,
        "code": GDK_RECOVERY_CONFIRMATION_CODE,
        "state": "CONFIRMED",
        "required": False,
        "robot_stop_confirmed": True,
        "confirmed_at": utc_now_iso(),
        "requirement": requirement.to_payload(),
        "stability": stability,
        "clear_result": cleared,
    }


def is_recovery_safe_snapshot(snapshot: Mapping[str, Any]) -> bool:
    """校验位姿快照是否满足恢复安全条件。

    要求: available=True, busy=False, 无 error/estop/control-active。
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
    whole_body_status = snapshot.get("wholeBodyStatus")
    if isinstance(whole_body_status, Mapping):
        for key in (
            "left_arm_error",
            "right_arm_error",
            "left_end_error",
            "right_end_error",
            "waist_error",
        ):
            value = whole_body_status.get(key)
            if value not in {None, 0, "0"}:
                return False
        for key in ("left_arm_estop", "right_arm_estop"):
            if whole_body_status.get(key) is True:
                return False
        # 控制标志为 True 说明控制器仍处在活跃控制态；缺失字段按旧 GDK 兼容处理。
        for key in ("left_arm_control", "right_arm_control"):
            if whole_body_status.get(key) is True:
                return False
    return True


def evaluate_recovery_stability(
    snapshots: list[Mapping[str, Any]],
    policy: GdkRecoveryStabilityPolicy,
) -> dict[str, object]:
    """评估多帧快照是否满足停稳恢复条件。"""

    reasons: list[dict[str, object]] = []
    expected_count = max(1, policy.sample_count)
    if len(snapshots) < expected_count:
        reasons.append(
            {
                "code": "INSUFFICIENT_SAMPLES",
                "expected": expected_count,
                "actual": len(snapshots),
            }
        )

    safe_snapshot_count = 0
    max_abs_velocity = 0.0
    max_position_delta = 0.0
    previous_positions: dict[str, float] | None = None
    velocity_samples = 0

    for index, snapshot in enumerate(snapshots):
        if is_recovery_safe_snapshot(snapshot):
            safe_snapshot_count += 1
        else:
            reasons.append({"code": "UNSAFE_SNAPSHOT", "sample_index": index})

        positions: dict[str, float] = {}
        for joint in iter_recovery_joint_readings(snapshot):
            name = joint.get("name")
            position = joint.get("position")
            velocity = joint.get("velocity")
            if isinstance(name, str) and isinstance(position, int | float):
                positions[name] = float(position)
            if isinstance(name, str) and isinstance(velocity, int | float):
                velocity_samples += 1
                max_abs_velocity = max(max_abs_velocity, abs(float(velocity)))
                if abs(float(velocity)) > policy.max_joint_velocity:
                    reasons.append(
                        {
                            "code": "JOINT_VELOCITY_NOT_STABLE",
                            "sample_index": index,
                            "joint": name,
                            "velocity": float(velocity),
                            "max_joint_velocity": policy.max_joint_velocity,
                        }
                    )

        if previous_positions is not None:
            for name, position in positions.items():
                if name not in previous_positions:
                    continue
                delta = abs(position - previous_positions[name])
                max_position_delta = max(max_position_delta, delta)
                if delta > policy.max_position_delta:
                    reasons.append(
                        {
                            "code": "JOINT_POSITION_NOT_STABLE",
                            "sample_index": index,
                            "joint": name,
                            "delta": delta,
                            "max_position_delta": policy.max_position_delta,
                        }
                    )
        previous_positions = positions

    if velocity_samples == 0:
        reasons.append({"code": "NO_JOINT_VELOCITY_SAMPLES"})

    return {
        "stable": not reasons,
        "sample_count": len(snapshots),
        "safe_snapshot_count": safe_snapshot_count,
        "policy": policy.to_payload(),
        "max_abs_velocity": max_abs_velocity,
        "max_position_delta": max_position_delta,
        "reasons": reasons,
    }


def iter_recovery_joint_readings(snapshot: Mapping[str, Any]) -> list[dict[str, object]]:
    """提取 arm/waist 关节位置和速度，用于多帧停稳确认。"""

    groups = snapshot.get("groups")
    if not isinstance(groups, Mapping):
        return []

    readings: list[dict[str, object]] = []
    for group_name in ("left_arm", "right_arm", "waist"):
        group = groups.get(group_name)
        if not isinstance(group, Mapping):
            continue
        joints = group.get("joints")
        if not isinstance(joints, list):
            continue
        for joint in joints:
            if not isinstance(joint, Mapping):
                continue
            readings.append(
                {
                    "name": joint.get("name"),
                    "position": joint.get("position"),
                    "velocity": joint.get("velocity"),
                }
            )
    return readings


def attach_gdk_recovery_payload(
    result: dict[str, object],
    payload: Mapping[str, object] | None,
) -> None:
    """将恢复确认信息附加到结果中。"""
    if payload is not None:
        result["gdk_recovery"] = deepcopy(dict(payload))


def current_gdk_recovery_payload() -> dict[str, object] | None:
    """返回当前 STOP_UNCONFIRMED payload；普通只读查询只展示，不清除。"""

    requirement = current_gdk_recovery_requirement()
    if requirement is None:
        return None
    return {
        "confirmed": False,
        "code": GDK_RECOVERY_REQUIRED_CODE,
        "state": STOP_UNCONFIRMED_STATE,
        "required": True,
        "robot_stop_confirmed": False,
        "requirement": requirement.to_payload(),
    }


def _write_recovery_requirement_to_disk(requirement: GdkRecoveryRequirement) -> None:
    if _RECOVERY_STATE_PATH is None:
        return
    _RECOVERY_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": 1,
        "saved_at": utc_now_iso(),
        "requirement": requirement.to_payload(),
    }
    _RECOVERY_STATE_PATH.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )


def _delete_recovery_requirement_from_disk() -> None:
    if _RECOVERY_STATE_PATH is None:
        return
    try:
        _RECOVERY_STATE_PATH.unlink()
    except FileNotFoundError:
        return


def _read_recovery_requirement_from_disk(path: Path | None) -> GdkRecoveryRequirement | None:
    if path is None or not path.exists():
        return None
    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(decoded, Mapping):
            raise ValueError("recovery state root must be an object")
        requirement = decoded.get("requirement")
        if not isinstance(requirement, Mapping):
            raise ValueError("recovery state missing requirement object")
        return GdkRecoveryRequirement.from_payload(requirement)
    except Exception as error:
        return GdkRecoveryRequirement(
            reason="recovery_state_file_unreadable",
            operation="unknown",
            marked_at=utc_now_iso(),
            source_result={
                "error": str(error),
                "state_file": str(path),
            },
        )
