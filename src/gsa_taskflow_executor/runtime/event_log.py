from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any

from .config import ExecutorSettings

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

    def __init__(self, execution_log_dir: Path) -> None:
        self.execution_log_dir = execution_log_dir
        self._lock = Lock()
        self.execution_log_dir.mkdir(parents=True, exist_ok=True)

    @classmethod
    def from_settings(cls, settings: ExecutorSettings) -> JsonlEventWriter:
        return cls(settings.execution_log_dir)

    def write(self, event: RuntimeEvent) -> Path:
        target = self.execution_log_dir / f"{datetime.now(timezone.utc):%Y%m%d}.jsonl"
        with self._lock:
            with target.open("a", encoding="utf-8") as file:
                file.write(event.to_json_line())
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
