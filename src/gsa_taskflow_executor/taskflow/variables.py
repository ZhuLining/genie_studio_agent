from __future__ import annotations

from collections.abc import Mapping, MutableMapping, Sequence
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

VARIABLE_PREFIX = "$.variables."


class VariableStoreError(KeyError):
    """Raised when variable references cannot be resolved."""


@dataclass
class VariableStore:
    """Runtime variable space for one taskflow execution."""

    variables: dict[str, Any] = field(default_factory=dict)

    def resolve(self, reference: str) -> Any:
        if not is_variable_reference(reference):
            raise VariableStoreError(f"不支持的变量引用: {reference}")

        path = reference[len(VARIABLE_PREFIX) :]
        segments = [segment for segment in path.split(".") if segment]
        if not segments:
            raise VariableStoreError(f"变量引用路径为空: {reference}")

        value: Any = self.variables
        for segment in segments:
            if not isinstance(value, Mapping) or segment not in value:
                raise VariableStoreError(f"变量路径不存在: {reference}")
            value = value[segment]
        return deepcopy(value)

    def resolve_value(self, value: Any) -> Any:
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
        node_scope = self.ensure_node_scope(output_var)
        node_scope["detail"] = deepcopy(dict(detail))

    def merge_node_detail(self, output_var: str, detail: Mapping[str, Any]) -> None:
        node_scope = self.ensure_node_scope(output_var)
        current = node_scope.get("detail")
        if not isinstance(current, MutableMapping):
            current = {}
            node_scope["detail"] = current
        current.update(deepcopy(dict(detail)))

    def ensure_node_scope(self, output_var: str) -> dict[str, Any]:
        if not output_var.strip():
            raise VariableStoreError("output_var 不能为空")
        existing = self.variables.setdefault(output_var, {})
        if not isinstance(existing, dict):
            raise VariableStoreError(f"变量命名冲突: {output_var}")
        return existing

    def snapshot(self) -> dict[str, Any]:
        return {"variables": deepcopy(self.variables)}


def is_variable_reference(value: str) -> bool:
    return value.strip().startswith(VARIABLE_PREFIX)


def collect_variable_references(value: Any) -> tuple[str, ...]:
    references: list[str] = []
    collect_variable_references_into(value, references)
    return tuple(references)


def collect_variable_references_into(value: Any, references: list[str]) -> None:
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
