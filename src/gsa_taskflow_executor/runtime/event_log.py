"""运行时事件日志。

RuntimeEvent + JsonlEventWriter 将结构化事件写入 {log_dir}/executions/{YYYYMMDD}.jsonl。
configure_stdout_logging() 配置应用级 logger。
"""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any

from .config import ExecutorSettings
from .payload_sanitizer import PayloadSanitizerConfig, sanitize_event_payload

LOGGER_NAME = "gsa_taskflow_executor"


@dataclass(frozen=True)
class RuntimeEvent:
    """A JSONL-friendly executor event."""

    event_type: str
    level: str = "info"
    message: str = ""
    app_execution_id: str | None = None
    node_id: str | None = None
    topic: str | None = None
    payload: dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())

    def to_json_line(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, separators=(",", ":"))


class JsonlEventWriter:
    """Append-only JSONL event writer used for execution replay and debugging."""

    def __init__(
        self,
        execution_log_dir: Path,
        sanitizer_config: PayloadSanitizerConfig | None = None,
    ) -> None:
        self.execution_log_dir = execution_log_dir
        self._lock = Lock()
        self.sanitizer_config = sanitizer_config or PayloadSanitizerConfig()
        self.execution_log_dir.mkdir(parents=True, exist_ok=True)

    @classmethod
    def from_settings(cls, settings: ExecutorSettings) -> JsonlEventWriter:
        return cls(
            settings.execution_log_dir,
            sanitizer_config=PayloadSanitizerConfig.from_settings(settings),
        )

    def write(self, event: RuntimeEvent) -> Path:
        target = self.execution_log_dir / f"{datetime.now(timezone.utc):%Y%m%d}.jsonl"
        safe_event = RuntimeEvent(
            event_type=event.event_type,
            level=event.level,
            message=event.message,
            app_execution_id=event.app_execution_id,
            node_id=event.node_id,
            topic=event.topic,
            payload=sanitize_event_payload(
                event.payload,
                config=self.sanitizer_config,
            ),
            timestamp=event.timestamp,
        )
        with self._lock:
            with target.open("a", encoding="utf-8") as file:
                file.write(safe_event.to_json_line())
                file.write("\n")
        return target


def configure_stdout_logging(level: str = "INFO") -> logging.Logger:
    logger = logging.getLogger(LOGGER_NAME)
    logger.setLevel(level.upper())
    logger.propagate = False

    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter(
                fmt="%(asctime)s %(levelname)s [%(name)s] %(message)s",
                datefmt="%Y-%m-%dT%H:%M:%S%z",
            )
        )
        logger.addHandler(handler)

    return logger
