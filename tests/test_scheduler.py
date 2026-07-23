import pytest

from fixtures import VALID_RIGHT_ARM_YAML
from gsa_taskflow_executor.scheduler import (
    NodeExecutionEvent,
    NodeRunResult,
    TaskflowScheduleError,
    TaskflowScheduler,
)
from gsa_taskflow_executor.taskflow_parser import parse_taskflow_yaml
from gsa_taskflow_executor.variable_store import VariableStore


def test_dry_run_scheduler_walks_linear_taskflow() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "success"
    assert result.terminal_node_id == "结束"
    assert result.visited_node_ids == ("开始", "位姿调整-位控", "结束")
    assert result.summary()["step_count"] == 3
    assert result.variables["位姿调整-位控"]["detail"]["status"] == "success"
    assert result.variables["位姿调整-位控"]["detail"]["mode"] == "dry-run"


def test_scheduler_stops_on_node_error() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    def runner(node, _variable_store):
        if node.node_id == "位姿调整-位控":
            return NodeRunResult(outcome="error")
        return NodeRunResult(outcome="success")

    result = TaskflowScheduler(taskflow, node_runner=runner).run()

    assert result.outcome == "error"
    assert result.terminal_node_id == "位姿调整-位控"
    assert result.visited_node_ids == ("开始", "位姿调整-位控")


def test_scheduler_resolves_variable_references_before_worker() -> None:
    yaml_payload = VALID_RIGHT_ARM_YAML.replace(
        """        action_data:
          - 0.282
          - -1.039
          - -0.304
          - -1.751
          - -0.621
          - -0.169
          - 1.122""",
        "        action_data: $.variables.二维码定位.detail.action_data.抓取点A",
    )
    taskflow = parse_taskflow_yaml(yaml_payload)
    store = VariableStore(
        variables={
            "二维码定位": {
                "detail": {
                    "action_data": {
                        "抓取点A": [0.282, -1.039, -0.304, -1.751, -0.621, -0.169, 1.122],
                    }
                }
            }
        }
    )

    result = TaskflowScheduler(taskflow, variable_store=store).run()
    worker_detail = result.variables["位姿调整-位控"]["detail"]

    assert worker_detail["outputs"]["resolved_params_template"]["right_arm"]["action_data"] == [
        0.282,
        -1.039,
        -0.304,
        -1.751,
        -0.621,
        -0.169,
        1.122,
    ]


def test_scheduler_emits_node_execution_events() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    events: list[NodeExecutionEvent] = []

    TaskflowScheduler(taskflow, node_event_handler=events.append).run()

    assert [event.status for event in events] == [
        "running",
        "success",
        "running",
        "success",
        "running",
        "success",
    ]
    assert events[2].node.node_id == "位姿调整-位控"
    assert events[3].result is not None
    assert events[3].variables is not None
    assert "位姿调整-位控" in events[3].variables


def test_scheduler_rejects_cycle() -> None:
    yaml_payload = VALID_RIGHT_ARM_YAML.replace(
        """  - from: 位姿调整-位控
    outcome: success
    to: 结束""",
        """  - from: 位姿调整-位控
    outcome: success
    to: 开始""",
    )
    taskflow = parse_taskflow_yaml(yaml_payload)

    with pytest.raises(TaskflowScheduleError, match="循环执行节点"):
        TaskflowScheduler(taskflow).run()


def test_scheduler_rejects_duplicate_success_transition() -> None:
    yaml_payload = (
        VALID_RIGHT_ARM_YAML
        + """
  - from: 开始
    outcome: success
    to: 结束
"""
    )
    taskflow = parse_taskflow_yaml(yaml_payload)

    with pytest.raises(TaskflowScheduleError, match="transition 不唯一"):
        TaskflowScheduler(taskflow).run()


def test_scheduler_rejects_max_steps() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    with pytest.raises(TaskflowScheduleError, match="超过最大步数"):
        TaskflowScheduler(taskflow, max_steps=1).run()
