"""Taskflow executor 能力清单。

客户端有同名能力表用于展示和发布前拦截；executor 这里仍是最终安全边界。
这份清单集中声明 skill_name 与 implementation 的固定映射，避免 registry、runtime
和测试各自维护一套容易漂移的白名单。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

SkillImplementationName = Literal[
    "motion_plan",
    "script",
    "end_effector",
    "force_control",
    "qr_pose",
]


@dataclass(frozen=True)
class ExecutorSkillCapability:
    skill_name: str
    implementation: SkillImplementationName
    description: str
    registry_enabled: bool
    client_publishable: bool
    reason: str = ""


EXECUTOR_SKILL_CAPABILITIES: tuple[ExecutorSkillCapability, ...] = (
    ExecutorSkillCapability(
        skill_name="motion_plan_skill",
        implementation="motion_plan",
        description="GDK motion planning skill.",
        registry_enabled=True,
        client_publishable=True,
    ),
    ExecutorSkillCapability(
        skill_name="script_skill",
        implementation="script",
        description="GDK whitelisted script skill.",
        registry_enabled=True,
        client_publishable=True,
    ),
    ExecutorSkillCapability(
        skill_name="control_end_effector_skill",
        implementation="end_effector",
        description="GDK end-effector open/close skill.",
        registry_enabled=True,
        client_publishable=True,
    ),
    ExecutorSkillCapability(
        skill_name="force_control_skill",
        implementation="force_control",
        description="GDK force-control skill, blocked until robot verification.",
        registry_enabled=True,
        client_publishable=False,
        reason="GDK 力控接口尚未完成真机验证",
    ),
    ExecutorSkillCapability(
        skill_name="qr_pose_skill",
        implementation="qr_pose",
        description="GDK QR pose localization skill.",
        registry_enabled=True,
        client_publishable=True,
    ),
)

EXECUTOR_SKILL_CAPABILITY_BY_NAME = {
    capability.skill_name: capability for capability in EXECUTOR_SKILL_CAPABILITIES
}

EXECUTOR_SKILL_IMPLEMENTATIONS: dict[str, SkillImplementationName] = {
    capability.skill_name: capability.implementation
    for capability in EXECUTOR_SKILL_CAPABILITIES
    if capability.registry_enabled
}


def iter_default_skill_capabilities() -> tuple[ExecutorSkillCapability, ...]:
    return tuple(
        capability
        for capability in EXECUTOR_SKILL_CAPABILITIES
        if capability.registry_enabled
    )


def supported_skill_message() -> str:
    supported = tuple(EXECUTOR_SKILL_IMPLEMENTATIONS)
    if len(supported) <= 1:
        return f"MVP 当前只支持 {supported[0]}" if supported else "MVP 当前未开放 skill"
    return f"MVP 当前只支持 {'、'.join(supported[:-1])} 和 {supported[-1]}"
