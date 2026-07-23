from fixtures import VALID_RIGHT_ARM_YAML
from gsa_taskflow_executor.config import ExecutorSettings
from gsa_taskflow_executor.scheduler import NodeRunResult, TaskflowScheduler
from gsa_taskflow_executor.status_reporter import TaskflowStatusReporter
from gsa_taskflow_executor.taskflow_parser import parse_taskflow_yaml


def test_status_reporter_publishes_execution_started_payload() -> None:
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1", executor_mode="mock"),
        publish_status=payloads.append,
    )

    reporter.publish_execution_started(taskflow)

    assert payloads[0]["aid"] == "aid-1"
    assert payloads[0]["app_execution_id"] == taskflow.app_execution_id
    assert payloads[0]["task_state"] == "RUNNING"
    assert payloads[0]["status"] == "RUNNING"
    assert payloads[0]["executor_mode"] == "mock"


def test_status_reporter_publishes_node_running_and_over_payloads() -> None:
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1", executor_mode="mock"),
        publish_status=payloads.append,
    )

    result = TaskflowScheduler(
        taskflow,
        node_event_handler=reporter.publish_node_event,
    ).run()
    reporter.publish_execution_finished(result)

    sub_tasks = [payload["sub_task"] for payload in payloads if "sub_task" in payload]
    assert len(sub_tasks) == 6
    assert sub_tasks[0]["node_id"] == "开始"
    assert sub_tasks[0]["state"] == "RUNNING"
    assert sub_tasks[1]["node_id"] == "开始"
    assert sub_tasks[1]["state"] == "OVER"
    assert sub_tasks[3]["node_id"] == "位姿调整-位控"
    assert sub_tasks[3]["state"] == "OVER"
    assert sub_tasks[3]["outputs"]["final_joint"] == [
        0.282,
        -1.039,
        -0.304,
        -1.751,
        -0.621,
        -0.169,
        1.122,
    ]
    assert "variables" in sub_tasks[3]
    assert payloads[-1]["task_state"] == "OVER"
    assert payloads[-1]["terminal_node_id"] == "结束"


def test_status_reporter_publishes_node_error_payload() -> None:
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    def runner(node, _variable_store):
        if node.node_id == "位姿调整-位控":
            return NodeRunResult(outcome="error", detail={"error": "mock failure"})
        return NodeRunResult(outcome="success")

    result = TaskflowScheduler(
        taskflow,
        node_runner=runner,
        node_event_handler=reporter.publish_node_event,
    ).run()
    reporter.publish_execution_finished(result)

    error_payload = payloads[3]["sub_task"]
    assert error_payload["node_id"] == "位姿调整-位控"
    assert error_payload["state"] == "ERROR"
    assert error_payload["error_msg"] == "mock failure"
    assert payloads[-1]["task_state"] == "ERROR"


def test_status_reporter_publishes_parse_error_payload_without_app_execution_id() -> None:
    payloads: list[dict[str, object]] = []
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    reporter.publish_execution_error(message="YAML 解析失败")

    assert payloads == [
        {
            "aid": "aid-1",
            "task_state": "ERROR",
            "status": "ERROR",
            "timestamp": payloads[0]["timestamp"],
            "timestamp_ms": payloads[0]["timestamp_ms"],
            "executor_mode": "mock",
            "error_msg": "YAML 解析失败",
            "error": "YAML 解析失败",
        }
    ]
