#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse, urlunparse

DEFAULT_REQUIRED_ENV_KEYS = (
    "MQTT_BROKER_URL",
    "EXECUTOR_AID",
    "EXECUTOR_MODE",
    "GSA_DATA_ROOT",
    "QR_MAPPING_SDK_PATH",
    "QR_MAPPING_SDK_PYTHON",
    "QR_LOCALIZE_SDK_PATH",
    "QR_LOCALIZE_SDK_PYTHON",
)
ALLOW_LOCAL_MQTT_BROKER_ENV = "ALLOW_LOCAL_MQTT_BROKER"
DEVELOPMENT_EXECUTOR_AIDS = {"gsa-dev"}
LOCAL_MQTT_HOSTS = {"127.0.0.1", "localhost", "::1"}


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    severity: str
    message: str
    details: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "name": self.name,
            "ok": self.ok,
            "severity": self.severity,
            "message": self.message,
        }
        if self.details:
            payload["details"] = self.details
        return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="二维码建图 / 点位录制 / 二维码定位交付预检，不调用 GDK 运动接口。",
    )
    parser.add_argument("--env-file", type=Path, required=True, help="executor env 文件路径")
    parser.add_argument("--robot-serial", default="", help="可选：校验指定机器人 SN 的产物目录")
    parser.add_argument("--project-name", default="", help="可选：校验指定二维码项目的产物目录")
    parser.add_argument(
        "--strict-safety-gate",
        action="store_true",
        help="把 ENABLE_GDK_CONTROL/CONFIRM_GDK_CONTROL 缺失视为错误",
    )
    parser.add_argument(
        "--check-mqtt",
        action="store_true",
        help="额外检查 MQTT broker TCP 连接，不订阅/发布消息",
    )
    args = parser.parse_args(argv)

    checks: list[CheckResult] = []
    env = dict(os.environ)
    env_file_values, env_file_check = load_env_file(args.env_file)
    checks.append(env_file_check)
    env.update(env_file_values)

    for key in DEFAULT_REQUIRED_ENV_KEYS:
        checks.append(check_required_env(env, key))

    checks.append(check_executor_aid_not_development_default(env))
    checks.append(check_mqtt_broker_not_unconfirmed_localhost(env))
    checks.append(check_executor_mode(env))
    checks.append(check_safety_gate(env, args.strict_safety_gate))
    checks.extend(check_sdk_paths(env))
    checks.extend(check_python_executable(env))
    checks.extend(check_data_root(env))

    if args.robot_serial and args.project_name:
        checks.extend(check_project_artifacts(env, args.robot_serial, args.project_name))
    elif args.robot_serial or args.project_name:
        checks.append(
            CheckResult(
                name="project_artifacts",
                ok=False,
                severity="warning",
                message="只提供了 robotSerial 或 projectName，跳过项目产物检查。",
            )
        )

    if args.check_mqtt:
        checks.append(check_mqtt_connectivity(env))

    failures = [check for check in checks if not check.ok and check.severity == "error"]
    warnings = [check for check in checks if not check.ok and check.severity == "warning"]
    result = {
        "ok": not failures,
        "errorCount": len(failures),
        "warningCount": len(warnings),
        "checks": [check.to_dict() for check in checks],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


def load_env_file(path: Path) -> tuple[dict[str, str], CheckResult]:
    if not path.exists():
        return {}, CheckResult(
            name="env_file",
            ok=False,
            severity="error",
            message=f"env 文件不存在：{path}",
        )
    if not path.is_file():
        return {}, CheckResult(
            name="env_file",
            ok=False,
            severity="error",
            message=f"env 路径不是文件：{path}",
        )

    values: dict[str, str] = {}
    invalid_lines: list[int] = []
    for index, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            invalid_lines.append(index)
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = strip_env_value(value.strip())
        if not key:
            invalid_lines.append(index)
            continue
        values[key] = value

    if invalid_lines:
        return values, CheckResult(
            name="env_file",
            ok=False,
            severity="warning",
            message="env 文件存在但包含无法解析的行，已跳过这些行。",
            details={"path": str(path), "invalidLines": invalid_lines},
        )
    return values, CheckResult(
        name="env_file",
        ok=True,
        severity="error",
        message="env 文件可读取。",
        details={"path": str(path), "keyCount": len(values)},
    )


def strip_env_value(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def check_required_env(env: dict[str, str], key: str) -> CheckResult:
    if env.get(key, "").strip():
        return CheckResult(
            name=f"env:{key}",
            ok=True,
            severity="error",
            message=f"{key} 已配置。",
        )
    return CheckResult(
        name=f"env:{key}",
        ok=False,
        severity="error",
        message=f"{key} 未配置。",
    )


def check_executor_aid_not_development_default(env: dict[str, str]) -> CheckResult:
    aid = env.get("EXECUTOR_AID", "").strip()
    if not aid:
        return CheckResult(
            name="executor_aid_default",
            ok=True,
            severity="error",
            message="EXECUTOR_AID 空值由必填检查处理。",
        )
    ok = aid.lower() not in DEVELOPMENT_EXECUTOR_AIDS
    return CheckResult(
        name="executor_aid_default",
        ok=ok,
        severity="error",
        message=(
            "EXECUTOR_AID 已替换开发默认值。"
            if ok
            else "EXECUTOR_AID 仍是开发默认值 gsa-dev，状态 topic 会和客户端目标 AID 错配。"
        ),
        details={"executorAid": aid},
    )


def check_mqtt_broker_not_unconfirmed_localhost(env: dict[str, str]) -> CheckResult:
    broker_url = env.get("MQTT_BROKER_URL", "").strip()
    if not broker_url:
        return CheckResult(
            name="mqtt_broker_localhost_confirmation",
            ok=True,
            severity="error",
            message="MQTT_BROKER_URL 空值由必填检查处理。",
        )

    parsed = urlparse(broker_url)
    host = (parsed.hostname or "").lower()
    local_allowed = env.get(ALLOW_LOCAL_MQTT_BROKER_ENV, "").strip() == "1"
    ok = host not in LOCAL_MQTT_HOSTS or local_allowed
    return CheckResult(
        name="mqtt_broker_localhost_confirmation",
        ok=ok,
        severity="error",
        message=(
            "MQTT broker 地址已通过发布确认。"
            if ok
            else "MQTT_BROKER_URL 指向本机地址，需确认 broker 与 executor 同机并设置 "
            f"{ALLOW_LOCAL_MQTT_BROKER_ENV}=1。"
        ),
        details={
            "brokerUrl": redact_url(broker_url),
            "allowLocalMqttBroker": local_allowed,
        },
    )


def check_executor_mode(env: dict[str, str]) -> CheckResult:
    mode = env.get("EXECUTOR_MODE", "").strip()
    return CheckResult(
        name="executor_mode",
        ok=mode == "gdk",
        severity="error",
        message="EXECUTOR_MODE 为 gdk。" if mode == "gdk" else "EXECUTOR_MODE 必须为 gdk。",
        details={"mode": mode or None},
    )


def check_safety_gate(env: dict[str, str], strict: bool) -> CheckResult:
    enabled = env.get("ENABLE_GDK_CONTROL", "").strip() == "1"
    confirmation = env.get("CONFIRM_GDK_CONTROL", "").strip()
    ok = enabled and confirmation == "TASKFLOW_ABS_JOINT"
    severity = "error" if strict else "warning"
    return CheckResult(
        name="taskflow_gdk_safety_gate",
        ok=ok,
        severity=severity,
        message=(
            "Taskflow GDK 控制安全门已确认。"
            if ok
            else "Taskflow GDK 控制安全门未确认；应用位姿调整节点会拒绝真机控制。"
        ),
        details={
            "enabled": enabled,
            "expectedConfirmation": "TASKFLOW_ABS_JOINT",
            "confirmed": confirmation == "TASKFLOW_ABS_JOINT",
        },
    )


def check_sdk_paths(env: dict[str, str]) -> list[CheckResult]:
    return [
        check_directory_env(env, "QR_MAPPING_SDK_PATH", "二维码建图 SDK 目录"),
        check_directory_env(env, "QR_LOCALIZE_SDK_PATH", "二维码定位 / 点位录制 SDK 目录"),
    ]


def check_directory_env(env: dict[str, str], key: str, label: str) -> CheckResult:
    value = env.get(key, "").strip()
    path = Path(value) if value else Path()
    ok = bool(value) and path.is_dir()
    return CheckResult(
        name=f"path:{key}",
        ok=ok,
        severity="error",
        message=f"{label}存在。" if ok else f"{label}不存在或未配置。",
        details={"path": value or None},
    )


def check_python_executable(env: dict[str, str]) -> list[CheckResult]:
    return [
        check_python_command(env.get("QR_MAPPING_SDK_PYTHON", ""), "QR_MAPPING_SDK_PYTHON"),
        check_python_command(env.get("QR_LOCALIZE_SDK_PYTHON", ""), "QR_LOCALIZE_SDK_PYTHON"),
    ]


def check_python_command(command: str, key: str) -> CheckResult:
    command = command.strip()
    if not command:
        return CheckResult(
            name=f"python:{key}",
            ok=False,
            severity="error",
            message=f"{key} 未配置。",
        )
    try:
        completed = subprocess.run(
            [command, "-c", "import sys; print(sys.version.split()[0])"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return CheckResult(
            name=f"python:{key}",
            ok=False,
            severity="error",
            message=f"{key} 无法启动。",
            details={"command": command, "error": str(error)},
        )

    ok = completed.returncode == 0
    return CheckResult(
        name=f"python:{key}",
        ok=ok,
        severity="error",
        message=f"{key} 可启动。" if ok else f"{key} 启动失败。",
        details={
            "command": command,
            "returncode": completed.returncode,
            "pythonVersion": completed.stdout.strip() if ok else None,
            "stderr": completed.stderr.strip()[:500] if completed.stderr else None,
        },
    )


def check_data_root(env: dict[str, str]) -> list[CheckResult]:
    value = env.get("GSA_DATA_ROOT", "").strip()
    if not value:
        return [
            CheckResult(
                name="path:GSA_DATA_ROOT",
                ok=False,
                severity="error",
                message="GSA_DATA_ROOT 未配置。",
            )
        ]
    root = Path(value)
    checks = [
        CheckResult(
            name="path:GSA_DATA_ROOT",
            ok=root.is_absolute() and root.is_dir(),
            severity="error",
            message=(
                "GSA_DATA_ROOT 是已存在的绝对目录。"
                if root.is_absolute() and root.is_dir()
                else "GSA_DATA_ROOT 必须是已存在的绝对目录。"
            ),
            details={"path": str(root)},
        )
    ]
    if root.is_dir():
        checks.append(check_data_root_writable(root))
    if root.parts[:2] == ("/", "home"):
        checks.append(
            CheckResult(
                name="systemd:data_root_home",
                ok=False,
                severity="warning",
                message=(
                    "GSA_DATA_ROOT 位于 /home；默认 systemd service 的 ProtectHome "
                    "会阻止访问。"
                ),
                details={"path": str(root), "recommendation": "正式部署建议改用 /data/gsa"},
            )
        )
    return checks


def check_data_root_writable(root: Path) -> CheckResult:
    probe = root / f".gsa_qr_pose_smoke_write_test_{os.getpid()}"
    try:
        probe.write_text("ok\n", encoding="utf-8")
        probe.unlink(missing_ok=True)
    except OSError as error:
        return CheckResult(
            name="path:GSA_DATA_ROOT:writable",
            ok=False,
            severity="error",
            message="GSA_DATA_ROOT 不可写。",
            details={"path": str(root), "error": str(error)},
        )
    return CheckResult(
        name="path:GSA_DATA_ROOT:writable",
        ok=True,
        severity="error",
        message="GSA_DATA_ROOT 可写。",
        details={"path": str(root)},
    )


def check_project_artifacts(
    env: dict[str, str],
    robot_serial: str,
    project_name: str,
) -> list[CheckResult]:
    data_root = Path(env.get("GSA_DATA_ROOT", "").strip())
    robot_root = data_root / robot_serial
    sensor_root = robot_root / "sensor"
    project_root = robot_root / "qr_pose_skill_conf" / project_name
    images_dir = project_root / "images"
    maps_dir = project_root / "maps"
    point_dir = project_root / "point"
    waypoints_dir = project_root / "waypoints"

    checks = [
        check_dir(project_root, "artifact:project_root", "二维码项目目录"),
        check_dir(sensor_root, "artifact:sensor_root", "机器人 sensor 目录"),
        check_dir(images_dir, "artifact:images_dir", "采集图片目录"),
        check_dir(maps_dir, "artifact:maps_dir", "地图目录"),
        check_dir(point_dir, "artifact:point_dir", "点位目录"),
        check_dir(waypoints_dir, "artifact:waypoints_dir", "waypoints 目录"),
    ]
    checks.extend(
        [
            check_glob(sensor_root, "intrinsic_hand_*_rgb.json", "artifact:intrinsics", "相机内参"),
            check_glob(
                sensor_root,
                "extrinsic_end_T_hand_*_rgbd.json",
                "artifact:extrinsics",
                "相机外参",
            ),
            check_any_glob(
                images_dir,
                ("*.jpg", "*.jpeg", "*.png"),
                "artifact:images",
                "采集图片",
            ),
            check_glob(maps_dir, "*.pcd", "artifact:maps_pcd", "PCD 地图"),
            check_glob(maps_dir, "*.yml", "artifact:maps_yml", "YAML 地图文件"),
            check_glob(maps_dir, "*-cam.yml", "artifact:maps_cam_yml", "相机地图文件"),
            check_nonempty_tree(point_dir, "artifact:point_records", "点位记录"),
            check_nonempty_tree(waypoints_dir, "artifact:waypoint_records", "waypoint 记录"),
        ]
    )
    return checks


def check_dir(path: Path, name: str, label: str) -> CheckResult:
    return CheckResult(
        name=name,
        ok=path.is_dir(),
        severity="error",
        message=f"{label}存在。" if path.is_dir() else f"{label}不存在。",
        details={"path": str(path)},
    )


def check_glob(path: Path, pattern: str, name: str, label: str) -> CheckResult:
    matches = sorted(item for item in path.glob(pattern) if item.is_file()) if path.is_dir() else []
    return CheckResult(
        name=name,
        ok=bool(matches),
        severity="error",
        message=f"已找到{label}。" if matches else f"未找到{label}。",
        details={"path": str(path), "pattern": pattern, "count": len(matches)},
    )


def check_any_glob(
    path: Path,
    patterns: tuple[str, ...],
    name: str,
    label: str,
) -> CheckResult:
    matches: list[Path] = []
    if path.is_dir():
        for pattern in patterns:
            matches.extend(item for item in path.glob(pattern) if item.is_file())
    return CheckResult(
        name=name,
        ok=bool(matches),
        severity="error",
        message=f"已找到{label}。" if matches else f"未找到{label}。",
        details={"path": str(path), "patterns": patterns, "count": len(matches)},
    )


def check_nonempty_tree(path: Path, name: str, label: str) -> CheckResult:
    matches = sorted(item for item in path.rglob("*") if item.is_file()) if path.is_dir() else []
    return CheckResult(
        name=name,
        ok=bool(matches),
        severity="error",
        message=f"已找到{label}。" if matches else f"未找到{label}。",
        details={"path": str(path), "count": len(matches)},
    )


def check_mqtt_connectivity(env: dict[str, str]) -> CheckResult:
    broker_url = env.get("MQTT_BROKER_URL", "").strip()
    parsed = urlparse(broker_url)
    if parsed.scheme not in {"mqtt", "tcp"} or not parsed.hostname:
        return CheckResult(
            name="mqtt:tcp_connect",
            ok=False,
            severity="error",
            message="MQTT_BROKER_URL 不是有效的 mqtt://host:port 地址。",
            details={"brokerUrl": redact_url(broker_url)},
        )
    port = parsed.port or 1883
    try:
        with socket.create_connection((parsed.hostname, port), timeout=3):
            pass
    except OSError as error:
        return CheckResult(
            name="mqtt:tcp_connect",
            ok=False,
            severity="error",
            message="无法连接 MQTT broker TCP 端口。",
            details={"brokerUrl": redact_url(broker_url), "error": str(error)},
        )
    return CheckResult(
        name="mqtt:tcp_connect",
        ok=True,
        severity="error",
        message="MQTT broker TCP 端口可连接。",
        details={"brokerUrl": redact_url(broker_url)},
    )


def redact_url(value: str) -> str:
    parsed = urlparse(value)
    if not parsed.username and not parsed.password:
        return value
    host = parsed.hostname or ""
    if parsed.port:
        host = f"{host}:{parsed.port}"
    return urlunparse(
        (parsed.scheme, host, parsed.path, parsed.params, parsed.query, parsed.fragment)
    )


if __name__ == "__main__":
    sys.exit(main())
