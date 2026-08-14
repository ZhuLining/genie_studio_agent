"""状态与日志 payload 瘦身/脱敏。

MQTT status 和 JSONL 日志都可能经过不可信或高成本链路；这里统一把大字段、
敏感字段和不可 JSON 化对象压成摘要，避免把大图、GDK raw 或凭据直接写入外部通道。
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Final

DEFAULT_MAX_STRING_LENGTH: Final = 512
DEFAULT_MAX_COLLECTION_ITEMS: Final = 20
DEFAULT_MAX_DEPTH: Final = 6
REDACTED_VALUE: Final = "[REDACTED]"
OMITTED_VALUE: Final = "[OMITTED]"
TRUNCATED_SUFFIX: Final = "...[truncated]"

SENSITIVE_KEY_PARTS: Final = (
    "password",
    "passwd",
    "secret",
    "token",
    "api_key",
    "apikey",
    "credential",
    "authorization",
    "auth",
    "private_key",
)
HEAVY_KEYS: Final = (
    "raw",
    "imageBase64",
    "image_base64",
    "dataBase64",
    "data_base64",
    "frameBase64",
    "frame_base64",
    "bytes",
    "buffer",
    "binary",
)


@dataclass(frozen=True)
class PayloadSanitizerConfig:
    max_string_length: int = DEFAULT_MAX_STRING_LENGTH
    max_collection_items: int = DEFAULT_MAX_COLLECTION_ITEMS
    max_depth: int = DEFAULT_MAX_DEPTH
    include_full_variables: bool = False

    @classmethod
    def from_settings(cls, settings: Any) -> PayloadSanitizerConfig:
        return cls(
            max_string_length=settings.payload_max_string_length,
            max_collection_items=settings.payload_max_collection_items,
            max_depth=settings.payload_max_depth,
            include_full_variables=settings.payload_include_full_variables,
        )


def sanitize_payload(
    value: Any,
    *,
    config: PayloadSanitizerConfig | None = None,
) -> Any:
    """返回 JSON-safe、脱敏、限长后的 payload。"""
    return sanitize_value(
        value,
        config=config or PayloadSanitizerConfig(),
        path=(),
        depth=0,
    )


def payload_preview(
    value: str,
    *,
    config: PayloadSanitizerConfig | None = None,
) -> dict[str, object]:
    """构建收包预览。只保留前缀和长度，不把完整 YAML/JSON 写进日志。"""
    sanitizer_config = config or PayloadSanitizerConfig()
    encoded_bytes = len(value.encode("utf-8"))
    return {
        "preview": truncate_string(value, sanitizer_config.max_string_length),
        "characters": len(value),
        "bytes": encoded_bytes,
        "truncated": len(value) > sanitizer_config.max_string_length,
    }


def sanitize_value(
    value: Any,
    *,
    config: PayloadSanitizerConfig,
    path: tuple[str, ...],
    depth: int,
) -> Any:
    if path and is_sensitive_key(path[-1]):
        return REDACTED_VALUE
    if path and is_heavy_key(path[-1]) and is_heavy_value(value):
        return summarize_omitted_value(value)
    if path and path[-1] == "variables" and not config.include_full_variables:
        return summarize_variables(value)
    if depth >= config.max_depth:
        return summarize_omitted_value(value, reason="max_depth")

    if isinstance(value, str):
        return truncate_string(value, config.max_string_length)
    if isinstance(value, bool | int | float) or value is None:
        return value
    if isinstance(value, bytes | bytearray | memoryview):
        return {"omitted": True, "reason": "binary", "bytes": len(value)}
    if isinstance(value, Mapping):
        return sanitize_mapping(value, config=config, path=path, depth=depth)
    if isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray):
        return sanitize_sequence(value, config=config, path=path, depth=depth)
    return truncate_string(repr(value), config.max_string_length)


def sanitize_mapping(
    value: Mapping[Any, Any],
    *,
    config: PayloadSanitizerConfig,
    path: tuple[str, ...],
    depth: int,
) -> dict[str, object]:
    items = list(value.items())
    limited = items[: config.max_collection_items]
    result: dict[str, object] = {}
    for raw_key, raw_child in limited:
        key = str(raw_key)
        result[key] = sanitize_value(
            raw_child,
            config=config,
            path=(*path, key),
            depth=depth + 1,
        )
    omitted_count = len(items) - len(limited)
    if omitted_count > 0:
        result["_omitted_items"] = omitted_count
    return result


def sanitize_sequence(
    value: Sequence[Any],
    *,
    config: PayloadSanitizerConfig,
    path: tuple[str, ...],
    depth: int,
) -> list[object] | dict[str, object]:
    limited = list(value[: config.max_collection_items])
    sanitized = [
        sanitize_value(
            child,
            config=config,
            path=(*path, str(index)),
            depth=depth + 1,
        )
        for index, child in enumerate(limited)
    ]
    omitted_count = len(value) - len(limited)
    if omitted_count <= 0:
        return sanitized
    return {
        "items": sanitized,
        "omitted_items": omitted_count,
        "total_items": len(value),
    }


def sanitize_status_payload(
    payload: Mapping[str, Any],
    *,
    config: PayloadSanitizerConfig | None = None,
) -> dict[str, Any]:
    """瘦身 MQTT status payload，保留客户端必需字段和小型 outputs/detail。"""
    sanitized = sanitize_payload(payload, config=config)
    if not isinstance(sanitized, dict):
        return {"payload": sanitized}
    return sanitized


def sanitize_event_payload(
    payload: Mapping[str, Any],
    *,
    config: PayloadSanitizerConfig | None = None,
) -> dict[str, Any]:
    """瘦身 JSONL 事件 payload；日志永远不写完整 variables/raw/base64。"""
    sanitizer_config = config or PayloadSanitizerConfig()
    safe_config = PayloadSanitizerConfig(
        max_string_length=sanitizer_config.max_string_length,
        max_collection_items=sanitizer_config.max_collection_items,
        max_depth=sanitizer_config.max_depth,
        include_full_variables=False,
    )
    sanitized = sanitize_payload(payload, config=safe_config)
    if not isinstance(sanitized, dict):
        return {"payload": sanitized}
    return sanitized


def summarize_variables(value: Any) -> dict[str, object]:
    if not isinstance(value, Mapping):
        return summarize_omitted_value(value, reason="variables")

    node_ids = [str(key) for key in value.keys()]
    node_summaries: dict[str, object] = {}
    for raw_node_id, raw_record in value.items():
        node_id = str(raw_node_id)
        if isinstance(raw_record, Mapping):
            detail = raw_record.get("detail")
            if isinstance(detail, Mapping):
                node_summaries[node_id] = {
                    "status": detail.get("status"),
                    "node_type": detail.get("node_type"),
                    "output_keys": sorted(str(key) for key in read_output_keys(detail)),
                }
                continue
        node_summaries[node_id] = {"type": type(raw_record).__name__}

    return {
        "summary_only": True,
        "node_count": len(value),
        "node_ids": node_ids,
        "nodes": node_summaries,
    }


def read_output_keys(detail: Mapping[Any, Any]) -> list[str]:
    outputs = detail.get("outputs")
    if not isinstance(outputs, Mapping):
        return []
    return [str(key) for key in outputs.keys()]


def truncate_string(value: str, max_length: int) -> str:
    if len(value) <= max_length:
        return value
    return f"{value[:max_length]}{TRUNCATED_SUFFIX}"


def is_sensitive_key(key: str) -> bool:
    normalized = key.lower().replace("-", "_")
    return any(part in normalized for part in SENSITIVE_KEY_PARTS)


def is_heavy_key(key: str) -> bool:
    normalized = key.lower()
    return any(normalized == heavy.lower() for heavy in HEAVY_KEYS)


def is_heavy_value(value: Any) -> bool:
    return isinstance(
        value,
        str | bytes | bytearray | memoryview | Mapping | Sequence,
    ) and not isinstance(
        value,
        bool | int | float,
    )


def summarize_omitted_value(value: Any, *, reason: str = "heavy_field") -> dict[str, object]:
    summary: dict[str, object] = {
        "omitted": True,
        "reason": reason,
        "type": type(value).__name__,
    }
    if isinstance(value, str):
        summary["characters"] = len(value)
        summary["bytes"] = len(value.encode("utf-8"))
    elif isinstance(value, bytes | bytearray | memoryview):
        summary["bytes"] = len(value)
    elif isinstance(value, Mapping | Sequence):
        summary["items"] = len(value)
    return summary
