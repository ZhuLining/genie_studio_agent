import pytest

from gsa_taskflow_executor.variable_store import (
    VariableStore,
    VariableStoreError,
    collect_variable_references,
    is_variable_reference,
)


def test_resolve_variable_reference() -> None:
    store = VariableStore(
        variables={
            "二维码定位": {
                "detail": {
                    "action_data": {
                        "抓取点A": [0.1, 0.2, 0.3, 0, 0, 0, 1],
                    }
                }
            }
        }
    )

    value = store.resolve("$.variables.二维码定位.detail.action_data.抓取点A")

    assert value == [0.1, 0.2, 0.3, 0, 0, 0, 1]


def test_resolve_value_recursively() -> None:
    store = VariableStore(variables={"上游": {"detail": {"joints": [1, 2, 3]}}})

    value = store.resolve_value(
        {
            "right_arm": {
                "control_type": "ABS_JOINT",
                "action_data": "$.variables.上游.detail.joints",
            }
        }
    )

    assert value == {"right_arm": {"control_type": "ABS_JOINT", "action_data": [1, 2, 3]}}


def test_missing_variable_path_raises() -> None:
    store = VariableStore()

    with pytest.raises(VariableStoreError, match="变量路径不存在"):
        store.resolve("$.variables.不存在.detail")


def test_set_and_merge_node_detail() -> None:
    store = VariableStore()

    store.set_node_detail("位姿调整-位控", {"status": "running"})
    store.merge_node_detail("位姿调整-位控", {"final_joint": [1, 2, 3]})

    assert store.snapshot() == {
        "variables": {
            "位姿调整-位控": {
                "detail": {
                    "status": "running",
                    "final_joint": [1, 2, 3],
                }
            }
        }
    }


def test_collect_variable_references() -> None:
    value = {
        "a": "$.variables.A.detail.x",
        "b": ["literal", "$.variables.B.detail.y"],
    }

    assert collect_variable_references(value) == (
        "$.variables.A.detail.x",
        "$.variables.B.detail.y",
    )


def test_is_variable_reference() -> None:
    assert is_variable_reference("$.variables.A.detail")
    assert not is_variable_reference("not-a-reference")
