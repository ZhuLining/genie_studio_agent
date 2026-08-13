"""Taskflow YAML 基础读取器。

这些函数是 parser 和 skill 参数解析共用的低层校验工具；错误统一抛
TaskflowParseError，确保桌面端收到的校验信息格式一致。
"""

from __future__ import annotations

from collections.abc import Mapping
from math import isfinite
from typing import Any

from gsa_taskflow_executor.taskflow.models import TaskflowParseError, YamlMapping


def expect_mapping(value: Any, path: str) -> YamlMapping:
    """断言 value 为 dict。"""

    if not isinstance(value, Mapping):
        raise TaskflowParseError(f"{path} 必须是对象")
    return value


def read_required_string(mapping: YamlMapping, key: str, path: str) -> str:
    """读取必填非空字符串。"""

    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise TaskflowParseError(f"{path}.{key} 必须是非空字符串")
    return value.strip()


def read_required_string_list(value: Any, path: str) -> list[str]:
    """读取必填非空去重字符串数组（loop.children 用）。"""

    if not isinstance(value, list) or not value:
        raise TaskflowParseError(f"{path} 必须是非空字符串数组")

    result: list[str] = []
    seen: set[str] = set()
    for index, item in enumerate(value):
        if not isinstance(item, str) or not item.strip():
            raise TaskflowParseError(f"{path}[{index}] 必须是非空字符串")
        child_id = item.strip()
        if child_id in seen:
            raise TaskflowParseError(f"{path}[{index}] 重复: {child_id}")
        seen.add(child_id)
        result.append(child_id)
    return result


def read_optional_string(value: Any, path: str) -> str | None:
    """读取可选字符串。None 或空字符串返回 None。"""

    if value is None:
        return None
    if not isinstance(value, str):
        raise TaskflowParseError(f"{path} 必须是字符串")
    return value.strip() or None


def read_bool(value: Any, fallback: bool) -> bool:
    """读取可选布尔值，None 时返回默认值。"""

    if value is None:
        return fallback
    if isinstance(value, bool):
        return value
    return fallback


def read_positive_number(value: Any, path: str) -> float:
    """读取必填正数（> 0）。"""

    number = to_float(value, path)
    if number <= 0:
        raise TaskflowParseError(f"{path} 必须大于 0")
    return number


def read_non_negative_number(value: Any, path: str) -> float:
    """读取必填非负数（>= 0）。"""

    number = to_float(value, path)
    if number < 0:
        raise TaskflowParseError(f"{path} 必须大于等于 0")
    return number


def read_positive_int(value: Any, path: str, *, maximum: int) -> int:
    """读取必填正整数，范围 [1, maximum]。"""

    if isinstance(value, bool) or not isinstance(value, int):
        raise TaskflowParseError(f"{path} 必须是整数")
    if value < 1 or value > maximum:
        raise TaskflowParseError(f"{path} 必须在 1 到 {maximum} 之间")
    return int(value)


def read_int_in_range(value: Any, path: str, *, minimum: int, maximum: int) -> int:
    """读取必填整数，范围 [minimum, maximum]。接受整数值浮点数（如 50.0）。"""

    if isinstance(value, bool):
        raise TaskflowParseError(f"{path} 必须是整数")
    if isinstance(value, int):
        number = value
    elif isinstance(value, float) and value.is_integer():
        number = int(value)
    else:
        raise TaskflowParseError(f"{path} 必须是整数")
    if number < minimum or number > maximum:
        raise TaskflowParseError(f"{path} 必须在 {minimum} 到 {maximum} 之间")
    return number


def read_positive_number_with_default(value: Any, path: str, fallback: float) -> float:
    """读取可选正数，None 时返回默认值。"""

    if value is None:
        return fallback
    return read_positive_number(value, path)


def read_non_negative_number_with_default(value: Any, path: str, fallback: float) -> float:
    """读取可选非负数，None 时返回默认值。"""

    if value is None:
        return fallback
    number = to_float(value, path)
    if number < 0:
        raise TaskflowParseError(f"{path} 必须大于等于 0")
    return number


def read_int_in_range_with_default(
    value: Any,
    path: str,
    *,
    fallback: int,
    minimum: int,
    maximum: int,
) -> int:
    """读取可选范围整数，None 时返回默认值。"""

    if value is None:
        return fallback
    return read_int_in_range(value, path, minimum=minimum, maximum=maximum)


def read_number_in_range_with_default(
    value: Any,
    path: str,
    *,
    fallback: float,
    minimum: float,
    maximum: float,
) -> float:
    """读取可选范围数字，None 时返回默认值。"""

    if value is None:
        return fallback
    number = to_float(value, path)
    if number < minimum or number > maximum:
        raise TaskflowParseError(f"{path} 必须在 {minimum} 到 {maximum} 之间")
    return number


def to_float(value: Any, path: str) -> float:
    """将 YAML 值转为有限浮点数。拒绝 bool（Python 中 bool 是 int 子类）、NaN、inf。"""

    if isinstance(value, bool):
        raise TaskflowParseError(f"{path} 必须是数字")
    if isinstance(value, int | float):
        number = float(value)
        if isfinite(number):
            return number
    raise TaskflowParseError(f"{path} 必须是数字")
