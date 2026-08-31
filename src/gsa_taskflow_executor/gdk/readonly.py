"""GDK 只读探针。

纯只读 GDK 接口封装：import agibot_gdk → gdk_init → Robot.get_joint_states()。
不执行任何控制命令，用于诊断和预检查。
"""

from __future__ import annotations

import importlib
from collections.abc import Callable, Mapping, Sequence
from datetime import datetime, timezone
from typing import Any

GDK_BACKEND = "agibot_gdk.Robot"
GDK_MODULE_NAME = "agibot_gdk"
GDK_IMPORT_REQUIRED_ATTRIBUTES = ("Robot",)
GDK_ENV_KEY_HINTS = (
    "GDK",
    "DDS",
    "CYCLONEDDS",
    "FASTRTPS",
    "RMW",
    "ROS",
    "AMENT",
    "LD_LIBRARY_PATH",
    "PYTHONPATH",
)


def run_gdk_env_check(
    import_module: Callable[[str], Any] = importlib.import_module,
    *,
    environ: Mapping[str, str] | None = None,
) -> dict[str, object]:
    """只验证 systemd 进程环境是否足以导入 GDK，不创建 Robot。

    LD_LIBRARY_PATH/PYTHONPATH 必须在 Python 进程启动前由 systemd 注入；
    这里不尝试运行 env.sh，避免交互 shell 和正式服务看到两套环境。
    """

    resolved_environ = environ or {}
    environment = summarize_gdk_environment(resolved_environ)
    try:
        agibot_gdk = import_module(GDK_MODULE_NAME)
    except Exception as error:
        return gdk_env_check_error("import_agibot_gdk", error, environment=environment)

    missing_attributes = [
        name for name in GDK_IMPORT_REQUIRED_ATTRIBUTES if not hasattr(agibot_gdk, name)
    ]
    if missing_attributes:
        return gdk_env_check_error(
            "validate_agibot_gdk_module",
            AttributeError(
                "agibot_gdk missing required attributes: "
                + ", ".join(missing_attributes)
            ),
            environment=environment,
            extra={"missing_attributes": missing_attributes},
        )

    return {
        "available": True,
        "backend": GDK_MODULE_NAME,
        "checked_at": utc_now_iso(),
        "environment": environment,
        "module": {
            "required_attributes": list(GDK_IMPORT_REQUIRED_ATTRIBUTES),
            "has_robot": hasattr(agibot_gdk, "Robot"),
            "has_gdk_init": callable(getattr(agibot_gdk, "gdk_init", None)),
        },
    }


def run_gdk_readonly_probe(
    import_module: Callable[[str], Any] = importlib.import_module,
) -> dict[str, object]:
    """Collect a read-only GDK snapshot without issuing any control command."""

    try:
        agibot_gdk = import_module(GDK_MODULE_NAME)
    except Exception as error:
        return unavailable_result("import_agibot_gdk", error)

    try:
        robot_factory = agibot_gdk.Robot
    except AttributeError as error:
        return unavailable_result("get_robot_factory", error)

    try:
        robot = robot_factory()
    except Exception as error:
        return unavailable_result("create_robot", error)

    try:
        joint_states = robot.get_joint_states()
    except Exception as error:
        return unavailable_result("get_joint_states", error)

    if not isinstance(joint_states, Mapping):
        return unavailable_result(
            "parse_joint_states",
            TypeError("robot.get_joint_states() did not return a mapping"),
        )

    snapshot = build_joint_state_snapshot(joint_states)
    return {
        "available": True,
        "backend": GDK_BACKEND,
        "collected_at": utc_now_iso(),
        **snapshot,
        "raw": {"get_joint_states": to_jsonable(joint_states)},
    }


def build_joint_state_snapshot(joint_states: Mapping[str, Any]) -> dict[str, object]:
    states = normalize_joint_states(joint_states.get("states"))
    joint_names = [name for name in (read_joint_name(state) for state in states) if name]
    return {
        "joint_count": read_joint_count(joint_states.get("nums"), states),
        "joint_names": joint_names,
        "left_arm_joint_names": [
            name for name in joint_names if is_left_arm_joint_name(name)
        ],
        "right_arm_joint_names": [
            name for name in joint_names if is_right_arm_joint_name(name)
        ],
        "nonzero_error_joints": build_nonzero_error_joints(states),
    }


def normalize_joint_states(value: Any) -> list[Mapping[str, Any]]:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes | bytearray):
        return []
    return [state for state in value if isinstance(state, Mapping)]


def read_joint_count(raw_count: Any, states: Sequence[Mapping[str, Any]]) -> int:
    if isinstance(raw_count, bool):
        return len(states)
    if isinstance(raw_count, int):
        return raw_count
    return len(states)


def read_joint_name(state: Mapping[str, Any]) -> str | None:
    name = state.get("name")
    if isinstance(name, str) and name.strip():
        return name.strip()
    return None


def is_left_arm_joint_name(name: str) -> bool:
    return "_arm_l_" in name or "arm_l" in name


def is_right_arm_joint_name(name: str) -> bool:
    return "_arm_r_" in name or "arm_r" in name


def build_nonzero_error_joints(states: Sequence[Mapping[str, Any]]) -> list[dict[str, object]]:
    failed: list[dict[str, object]] = []
    for state in states:
        error_code = state.get("error_code")
        if is_zero_error(error_code):
            continue
        name = read_joint_name(state) or "<unknown>"
        failed.append(
            {
                "name": name,
                "error_code": to_jsonable(error_code),
            }
        )
    return failed


def is_zero_error(error_code: Any) -> bool:
    if error_code is None:
        return True
    if isinstance(error_code, bool):
        return not error_code
    if isinstance(error_code, int | float):
        return error_code == 0
    if isinstance(error_code, str):
        return error_code.strip() in {"", "0"}
    return False


def summarize_gdk_environment(environ: Mapping[str, str]) -> dict[str, object]:
    """输出 GDK/DDS 环境摘要，只暴露 key，不回显路径值或潜在敏感内容。"""

    gdk_related_keys = sorted(
        key for key in environ if any(hint in key.upper() for hint in GDK_ENV_KEY_HINTS)
    )
    return {
        "pythonpath_configured": bool(environ.get("PYTHONPATH")),
        "ld_library_path_configured": bool(environ.get("LD_LIBRARY_PATH")),
        "gdk_related_keys": gdk_related_keys,
    }


def gdk_env_check_error(
    stage: str,
    error: Exception,
    *,
    environment: Mapping[str, object],
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "available": False,
        "backend": GDK_MODULE_NAME,
        "checked_at": utc_now_iso(),
        "environment": dict(environment),
        "error_stage": stage,
        "error_type": type(error).__name__,
        "error_msg": str(error),
    }
    if extra:
        payload.update(extra)
    return payload


def unavailable_result(stage: str, error: Exception) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_BACKEND,
        "collected_at": utc_now_iso(),
        "joint_count": 0,
        "joint_names": [],
        "left_arm_joint_names": [],
        "right_arm_joint_names": [],
        "nonzero_error_joints": [],
        "raw": {},
        "error_stage": stage,
        "error_type": type(error).__name__,
        "error_msg": str(error),
    }


def to_jsonable(value: Any) -> object:
    if value is None or isinstance(value, str | int | float | bool):
        return value
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray):
        return [to_jsonable(item) for item in value]
    return repr(value)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()
