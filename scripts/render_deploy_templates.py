#!/usr/bin/env python3
"""Render deploy templates from a site deployment profile."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

DEPLOY_FILES = (
    "gsa-taskflow-executor.service",
    "gsa-taskflow-executor.env.example",
    "gsa-taskflow-executor.gdk.env.example",
)

REQUIRED_PROFILE_KEYS = (
    "EXECUTOR_USER",
    "EXECUTOR_INSTALL_DIR",
    "EXECUTOR_CONFIG_DIR",
    "EXECUTOR_LOG_DIR",
    "GSA_DATA_ROOT",
    "QR_SDK_PATH",
    "GDK_APP_DIR",
    "PROTECT_HOME",
)

GDK_ENV_KEY_PARTS = (
    "PATH",
    "PYTHONPATH",
    "LD_LIBRARY_PATH",
    "CYCLONEDDS_URI",
    "CYCLONEDDS_[A-Za-z0-9_]*",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "FASTRTPS_[A-Za-z0-9_]*",
    "DDS_[A-Za-z0-9_]*",
    "RMW_IMPLEMENTATION",
    "ROS_DOMAIN_ID",
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "AORTA_[A-Za-z0-9_]*",
    "AGIBOT_[A-Za-z0-9_]*",
    "GDK_[A-Za-z0-9_]*",
)


@dataclass(frozen=True)
class DeploymentProfile:
    executor_user: str
    executor_install_dir: str
    executor_config_dir: str
    executor_log_dir: str
    gsa_data_root: str
    qr_sdk_path: str
    gdk_app_dir: str
    protect_home: str

    @property
    def executor_bin(self) -> str:
        return f"{self.executor_install_dir}/.venv/bin/gsa-taskflow-executor"

    @property
    def executor_python(self) -> str:
        return f"{self.executor_install_dir}/.venv/bin/python"

    @property
    def runtime_env_file(self) -> str:
        return f"{self.executor_config_dir}/gsa-taskflow-executor.env"

    @property
    def gdk_env_file(self) -> str:
        return f"{self.executor_config_dir}/gdk.env"

    @property
    def skill_registry_file(self) -> str:
        return f"{self.executor_config_dir}/skills.yaml"

    def validate(self) -> None:
        if not self.executor_user:
            raise ValueError("EXECUTOR_USER 不能为空")
        for key, value in {
            "EXECUTOR_INSTALL_DIR": self.executor_install_dir,
            "EXECUTOR_CONFIG_DIR": self.executor_config_dir,
            "EXECUTOR_LOG_DIR": self.executor_log_dir,
            "GSA_DATA_ROOT": self.gsa_data_root,
            "QR_SDK_PATH": self.qr_sdk_path,
            "GDK_APP_DIR": self.gdk_app_dir,
        }.items():
            if not value.startswith("/"):
                raise ValueError(f"{key} 必须是绝对路径: {value!r}")
        if self.protect_home not in {"true", "false"}:
            raise ValueError("PROTECT_HOME 只能是 true 或 false")


def read_profile(path: Path) -> DeploymentProfile:
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{line_number}: profile 行必须是 KEY=VALUE")
        key, value = line.split("=", 1)
        values[key.strip()] = strip_optional_quotes(value.strip())

    missing = [key for key in REQUIRED_PROFILE_KEYS if not values.get(key)]
    if missing:
        raise ValueError("deployment profile 缺少字段: " + ", ".join(missing))

    profile = DeploymentProfile(
        executor_user=values["EXECUTOR_USER"],
        executor_install_dir=values["EXECUTOR_INSTALL_DIR"].rstrip("/"),
        executor_config_dir=values["EXECUTOR_CONFIG_DIR"].rstrip("/"),
        executor_log_dir=values["EXECUTOR_LOG_DIR"].rstrip("/"),
        gsa_data_root=values["GSA_DATA_ROOT"].rstrip("/"),
        qr_sdk_path=values["QR_SDK_PATH"].rstrip("/"),
        gdk_app_dir=values["GDK_APP_DIR"].rstrip("/"),
        protect_home=values["PROTECT_HOME"].lower(),
    )
    profile.validate()
    return profile


def strip_optional_quotes(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]
    return value


def render_service(profile: DeploymentProfile) -> str:
    return "\n".join(
        [
            "[Unit]",
            "Description=GSA Taskflow Executor",
            "Wants=network-online.target",
            "After=network-online.target",
            "",
            "[Service]",
            "Type=simple",
            f"User={profile.executor_user}",
            f"WorkingDirectory={profile.executor_install_dir}",
            "Environment=PYTHONUNBUFFERED=1",
            f"EnvironmentFile={profile.runtime_env_file}",
            (
                "# GDK/DDS 依赖 LD_LIBRARY_PATH/PYTHONPATH 等进程启动前环境；"
                "不要依赖人工 shell source。"
            ),
            f"EnvironmentFile=-{profile.gdk_env_file}",
            f"ExecStartPre={profile.executor_bin} --deployment-config-check",
            f"ExecStartPre={profile.executor_bin} --gdk-env-check",
            f"ExecStart={profile.executor_bin} --listen",
            "Restart=on-failure",
            "RestartSec=3",
            "# 日志目录和 GSA_DATA_ROOT 必须和部署 profile/env 文件保持一致，",
            "# 否则 systemd 沙箱会在启动或运行时阻止 executor 写入产物。",
            f"ReadWritePaths={profile.executor_log_dir} {profile.gsa_data_root}",
            "NoNewPrivileges=true",
            "PrivateTmp=true",
            f"ProtectHome={profile.protect_home}",
            (
                "# GDK 环境目录只读暴露；GDK/DDS 变量仍由 gdk.env "
                "在进程启动前注入。"
            ),
            f"BindReadOnlyPaths=-{profile.gdk_app_dir}",
            "ProtectSystem=full",
            "",
            "[Install]",
            "WantedBy=multi-user.target",
            "",
        ]
    )


def render_runtime_env_example(base_text: str, profile: DeploymentProfile) -> str:
    replacements = {
        "GSA_DATA_ROOT": profile.gsa_data_root,
        "QR_MAPPING_SDK_PATH": profile.qr_sdk_path,
        "QR_MAPPING_SDK_PYTHON": profile.executor_python,
        "QR_LOCALIZE_SDK_PATH": profile.qr_sdk_path,
        "QR_LOCALIZE_SDK_PYTHON": profile.executor_python,
        "EXECUTOR_LOG_DIR": profile.executor_log_dir,
        "SKILL_REGISTRY_FILE": profile.skill_registry_file,
    }
    rendered = base_text
    for key, value in replacements.items():
        rendered = replace_env_assignment(rendered, key, value)
    return rendered if rendered.endswith("\n") else rendered + "\n"


def replace_env_assignment(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"^{re.escape(key)}=.*$", flags=re.MULTILINE)
    if not pattern.search(text):
        raise ValueError(f"env template 缺少 {key}= 配置行")
    return pattern.sub(lambda _: f"{key}={value}", text)


def render_gdk_env_example(profile: DeploymentProfile) -> str:
    env_key_pattern = "^(" + "|".join(GDK_ENV_KEY_PARTS) + ")="
    return "\n".join(
        [
            "# GSA Taskflow Executor GDK/DDS runtime environment for systemd.",
            "#",
            (
                "# systemd EnvironmentFile 只接受静态 KEY=VALUE，"
                "不会执行 source、命令替换或 shell 变量展开。"
            ),
            f"# 在 G2 现场先用交互 shell source {profile.gdk_app_dir}/env.sh，",
            (
                "# 确认可 import agibot_gdk 后，再把其中和 GDK/DDS "
                "相关的最终值固化到"
            ),
            f"# {profile.gdk_env_file}。不要在这里写入 token、密码或其他凭据。",
            "#",
            "# 生成候选文件示例：",
            f"# GDK_ENV_KEYS='{env_key_pattern}'",
            f"# bash -lc 'set -a; source {profile.gdk_app_dir}/env.sh; env' \\",
            f"#   | grep -E \"$GDK_ENV_KEYS\" | sudo tee {profile.gdk_env_file} >/dev/null",
            f"# sudo chmod 640 {profile.gdk_env_file}",
            "",
            (
                "# Python 包路径：当 agibot_gdk 不在 executor venv 内时，"
                "必须在 Python 进程启动前注入。"
            ),
            f"# PYTHONPATH={profile.gdk_app_dir}/python",
            "",
            (
                "# GDK 原生库路径：导入 agibot_gdk 依赖的 .so "
                "必须在 Python 进程启动前可见。"
            ),
            f"# LD_LIBRARY_PATH={profile.gdk_app_dir}/lib",
            "",
            (
                "# 如果 env.sh 修改了 PATH，也需要固化最终 PATH，"
                "确保 systemd 启动时能找到 GDK 依赖命令。"
            ),
            (
                f"# PATH={profile.gdk_app_dir}/bin:"
                "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
            ),
            "",
            (
                "# DDS / ROS2 相关变量请按目标 G2 机器 env.sh "
                "的实际输出填写；下面只是常见 key 提示。"
            ),
            f"# CYCLONEDDS_URI=file://{profile.gdk_app_dir}/config/cyclonedds.xml",
            f"# FASTRTPS_DEFAULT_PROFILES_FILE={profile.gdk_app_dir}/config/fastdds.xml",
            "# RMW_IMPLEMENTATION=",
            "# ROS_DOMAIN_ID=",
            "# AMENT_PREFIX_PATH=",
            "# CMAKE_PREFIX_PATH=",
            "# COLCON_PREFIX_PATH=",
            "# AORTA_<按 env.sh 实际输出填写>",
            "# DDS_<按 env.sh 实际输出填写>",
            "",
        ]
    )


def render_files(template_dir: Path, profile: DeploymentProfile) -> dict[str, str]:
    runtime_template = (template_dir / "gsa-taskflow-executor.env.example").read_text(
        encoding="utf-8"
    )
    return {
        "gsa-taskflow-executor.service": render_service(profile),
        "gsa-taskflow-executor.env.example": render_runtime_env_example(
            runtime_template,
            profile,
        ),
        "gsa-taskflow-executor.gdk.env.example": render_gdk_env_example(profile),
    }


def write_files(output_dir: Path, rendered_files: dict[str, str]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for file_name, content in rendered_files.items():
        (output_dir / file_name).write_text(content, encoding="utf-8")


def check_files(output_dir: Path, rendered_files: dict[str, str]) -> int:
    mismatches: list[str] = []
    for file_name, expected in rendered_files.items():
        path = output_dir / file_name
        if not path.exists() or path.read_text(encoding="utf-8") != expected:
            mismatches.append(file_name)
    if mismatches:
        print("deploy templates are not in sync: " + ", ".join(mismatches), file=sys.stderr)
        return 1
    print("deploy templates are in sync")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path("deploy/deployment.profile.example"),
        help="Deployment profile file to read.",
    )
    parser.add_argument(
        "--template-dir",
        type=Path,
        default=None,
        help="Directory containing the base env template. Defaults to profile parent.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("deploy"),
        help="Directory to write rendered deploy files.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check rendered output against files in output-dir without writing.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    profile = read_profile(args.profile)
    template_dir = args.template_dir or args.profile.parent
    rendered_files = render_files(template_dir=template_dir, profile=profile)
    if args.check:
        return check_files(args.output_dir, rendered_files)
    write_files(args.output_dir, rendered_files)
    print(f"rendered deploy templates to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
