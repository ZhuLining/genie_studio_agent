"""运行时变量存储 — 支持 $.variables.<node_id>.<path> 引用解析。

VariableStore 为每个 taskflow 执行维护一个变量命名空间。
节点结果按 node_id（或 output_var）存储为 {output_var: {detail: {...}}}。
下游节点通过 $.variables.<node_id>.detail.<field> 引用上游产物。
"""

from __future__ import annotations

from collections.abc import Mapping, MutableMapping, Sequence
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

VARIABLE_PREFIX = "$.variables."


class VariableStoreError(KeyError):
    """变量引用无法解析。"""


@dataclass
class VariableStore:
    """单个 taskflow 执行的运行时变量空间。"""

    variables: dict[str, Any] = field(default_factory=dict)

    def resolve(self, reference: str) -> Any:
        """解析 $.variables.<node>.<path> 引用，沿路径逐级查找。返回深拷贝。"""
        if not is_variable_reference(reference):
            raise VariableStoreError(f"不支持的变量引用: {reference}")

        path = reference[len(VARIABLE_PREFIX) :]
        segments = [segment for segment in path.split(".") if segment]
        if not segments:
            raise VariableStoreError(f"变量引用路径为空: {reference}")

        value: Any = self.variables
        for segment in segments:
            if isinstance(value, Mapping):
                if segment not in value:
                    raise VariableStoreError(f"变量路径不存在: {reference}")
                value = value[segment]
                continue

            if is_indexable_sequence(value):
                index = read_sequence_index(segment, reference)
                if index < 0 or index >= len(value):
                    raise VariableStoreError(f"变量路径不存在: {reference}")
                value = value[index]
                continue

            # loop v2 的 iterations 是数组；除数组索引外，变量路径仍只允许对象字段，
            # 避免把字符串等标量误当成可继续展开的路径。
            if not isinstance(value, Mapping):
                raise VariableStoreError(f"变量路径不存在: {reference}")
        return deepcopy(value)

    def resolve_value(self, value: Any) -> Any:
        """递归解析值中的所有 $.variables.* 引用。

        - 字符串引用 → resolve()
        - Mapping → 递归每个 value
        - list/tuple → 递归每个元素
        - 其他 → 深拷贝
        """
        if isinstance(value, str) and is_variable_reference(value):
            return self.resolve(value)
        if isinstance(value, Mapping):
            return {key: self.resolve_value(item) for key, item in value.items()}
        if isinstance(value, list):
            return [self.resolve_value(item) for item in value]
        if isinstance(value, tuple):
            return tuple(self.resolve_value(item) for item in value)
        return deepcopy(value)

    def set_node_detail(self, output_var: str, detail: Mapping[str, Any]) -> None:
        """写入节点结果。覆盖已有 detail。"""
        node_scope = self.ensure_node_scope(output_var)
        node_scope["detail"] = deepcopy(dict(detail))

    def merge_node_detail(self, output_var: str, detail: Mapping[str, Any]) -> None:
        """合并节点结果。保留已有字段，新增/覆盖传入字段。"""
        node_scope = self.ensure_node_scope(output_var)
        current = node_scope.get("detail")
        if not isinstance(current, MutableMapping):
            current = {}
            node_scope["detail"] = current
        current.update(deepcopy(dict(detail)))

    def ensure_node_scope(self, output_var: str) -> dict[str, Any]:
        """获取或创建节点命名空间。防止与已有非 dict 值冲突。"""
        if not output_var.strip():
            raise VariableStoreError("output_var 不能为空")
        existing = self.variables.setdefault(output_var, {})
        if not isinstance(existing, dict):
            raise VariableStoreError(f"变量命名冲突: {output_var}")
        return existing

    def snapshot(self) -> dict[str, Any]:
        """返回完整变量空间快照（深拷贝）。"""
        return {"variables": deepcopy(self.variables)}


def is_variable_reference(value: str) -> bool:
    """判断字符串是否为 $.variables. 开头的变量引用。"""
    return value.strip().startswith(VARIABLE_PREFIX)


def is_indexable_sequence(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray)


def read_sequence_index(segment: str, reference: str) -> int:
    try:
        return int(segment)
    except ValueError as error:
        raise VariableStoreError(f"变量数组索引无效: {reference}") from error


def collect_variable_references(value: Any) -> tuple[str, ...]:
    """递归收集值中所有 $.variables.* 引用。"""
    references: list[str] = []
    collect_variable_references_into(value, references)
    return tuple(references)


def collect_variable_references_into(value: Any, references: list[str]) -> None:
    """递归收集变量引用到 references 列表。"""
    if isinstance(value, str):
        if is_variable_reference(value):
            references.append(value.strip())
        return
    if isinstance(value, Mapping):
        for item in value.values():
            collect_variable_references_into(item, references)
        return
    if isinstance(value, Sequence) and not isinstance(value, str | bytes | bytearray):
        for item in value:
            collect_variable_references_into(item, references)
