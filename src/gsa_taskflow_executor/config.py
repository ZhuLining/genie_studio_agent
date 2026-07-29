from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from os import environ
from pathlib import Path
from urllib.parse import urlparse


class ConfigError(ValueError):
    """Raised when executor settings are incomplete or unsafe."""


EnvMapping = Mapping[str, str]


@dataclass(frozen=True)
class ExecutorSettings:
    """Runtime settings for the GDK taskflow executor."""

    mqtt_broker_url: str = "mqtt://127.0.0.1:1883"
    mqtt_client_id: str = "gsa-taskflow-executor-dev"
    taskflow_input_topic: str = "gsa/self/taskflow_yaml"
    taskflow_status_topic_template: str = "gsa/self/{aid}/status"
    robot_current_pose_request_topic: str = "gsa/self/robot/state/get_current_pose/request"
    robot_current_pose_response_topic: str = "gsa/self/robot/state/get_current_pose/response"
    executor_aid: str = "gsa-dev"
    executor_mode: str = "gdk"
    executor_log_dir: str = "logs"
    skill_registry_file: str = ""

    @classmethod
    def from_env(
        cls,
        env: EnvMapping | None = None,
        env_file: str | Path | None = None,
    ) -> ExecutorSettings:
        source = dict(environ if env is None else env)
        if env_file is not None:
            source = {**read_env_file(Path(env_file)), **source}

        settings = cls(
            mqtt_broker_url=source.get("MQTT_BROKER_URL", cls.mqtt_broker_url).strip(),
            mqtt_client_id=source.get("MQTT_CLIENT_ID", cls.mqtt_client_id).strip(),
            taskflow_input_topic=source.get(
                "TASKFLOW_INPUT_TOPIC",
                cls.taskflow_input_topic,
            ).strip(),
            taskflow_status_topic_template=source.get(
                "TASKFLOW_STATUS_TOPIC_TEMPLATE",
                cls.taskflow_status_topic_template,
            ).strip(),
            robot_current_pose_request_topic=source.get(
                "ROBOT_CURRENT_POSE_REQUEST_TOPIC",
                cls.robot_current_pose_request_topic,
            ).strip(),
            robot_current_pose_response_topic=source.get(
                "ROBOT_CURRENT_POSE_RESPONSE_TOPIC",
                cls.robot_current_pose_response_topic,
            ).strip(),
            executor_aid=source.get("EXECUTOR_AID", cls.executor_aid).strip(),
            executor_mode=source.get("EXECUTOR_MODE", cls.executor_mode).strip(),
            executor_log_dir=source.get("EXECUTOR_LOG_DIR", cls.executor_log_dir).strip(),
            skill_registry_file=source.get(
                "SKILL_REGISTRY_FILE",
                cls.skill_registry_file,
            ).strip(),
        )
        settings.validate()
        return settings

    @classmethod
    def from_env_file(cls, env_file: str | Path) -> ExecutorSettings:
        return cls.from_env(env={}, env_file=env_file)

    @property
    def status_topic(self) -> str:
        return self.taskflow_status_topic_template.format(aid=self.executor_aid)

    @property
    def log_dir_path(self) -> Path:
        return Path(self.executor_log_dir).expanduser()

    @property
    def execution_log_dir(self) -> Path:
        return self.log_dir_path / "executions"

    def validate(self) -> None:
        require_non_empty("MQTT_BROKER_URL", self.mqtt_broker_url)
        require_non_empty("MQTT_CLIENT_ID", self.mqtt_client_id)
        require_non_empty("TASKFLOW_INPUT_TOPIC", self.taskflow_input_topic)
        require_non_empty("TASKFLOW_STATUS_TOPIC_TEMPLATE", self.taskflow_status_topic_template)
        require_non_empty(
            "ROBOT_CURRENT_POSE_REQUEST_TOPIC",
            self.robot_current_pose_request_topic,
        )
        require_non_empty(
            "ROBOT_CURRENT_POSE_RESPONSE_TOPIC",
            self.robot_current_pose_response_topic,
        )
        require_non_empty("EXECUTOR_AID", self.executor_aid)
        require_non_empty("EXECUTOR_MODE", self.executor_mode)
        require_non_empty("EXECUTOR_LOG_DIR", self.executor_log_dir)

        broker = urlparse(self.mqtt_broker_url)
        if broker.scheme not in {"mqtt", "mqtts", "ws", "wss"}:
            raise ConfigError(
                "MQTT_BROKER_URL 只支持 mqtt、mqtts、ws 或 wss scheme"
            )
        if not broker.hostname:
            raise ConfigError("MQTT_BROKER_URL 必须包含 broker host")
        if "{aid}" not in self.taskflow_status_topic_template:
            raise ConfigError("TASKFLOW_STATUS_TOPIC_TEMPLATE 必须包含 {aid}")
        if self.executor_mode != "gdk":
            raise ConfigError("EXECUTOR_MODE 只支持 gdk")

    def to_dict(self) -> dict[str, str]:
        data = asdict(self)
        data["status_topic"] = self.status_topic
        data["execution_log_dir"] = str(self.execution_log_dir)
        return data


def require_non_empty(name: str, value: str) -> None:
    if not value:
        raise ConfigError(f"{name} 不能为空")


def read_env_file(path: Path) -> dict[str, str]:
    if not path.exists():
        raise ConfigError(f"env 文件不存在: {path}")
    if not path.is_file():
        raise ConfigError(f"env 路径不是文件: {path}")

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            raise ConfigError(f"{path}:{line_number} 缺少 KEY=VALUE 格式")

        key, value = line.split("=", 1)
        key = key.strip()
        if not key:
            raise ConfigError(f"{path}:{line_number} env key 不能为空")
        values[key] = strip_env_value(value.strip())

    return values


def strip_env_value(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value
