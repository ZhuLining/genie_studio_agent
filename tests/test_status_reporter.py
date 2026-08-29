import gsa_taskflow_executor.skills.runtime as skill_runtime
from fixtures import VALID_RIGHT_ARM_YAML
from gsa_taskflow_executor.mqtt.status_reporter import TaskflowStatusReporter
from gsa_taskflow_executor.runtime.config import ExecutorSettings
from gsa_taskflow_executor.taskflow.control import TaskflowCancellation
from gsa_taskflow_executor.taskflow.parser import parse_taskflow_yaml
from gsa_taskflow_executor.taskflow.scheduler import NodeRunResult, TaskflowScheduler


def test_status_reporter_publishes_execution_started_payload() -> None:
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    reporter.publish_execution_started(taskflow)

    assert payloads[0]["aid"] == "aid-1"
    assert payloads[0]["app_execution_id"] == taskflow.app_execution_id
    assert payloads[0]["task_state"] == "RUNNING"
    assert payloads[0]["status"] == "RUNNING"
    assert payloads[0]["status_seq"] == 1
    assert payloads[0]["executor_mode"] == "gdk"


def test_status_reporter_publishes_node_running_and_sub_task_over_payloads(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    result = TaskflowScheduler(
        taskflow,
        node_event_handler=reporter.publish_node_event,
    ).run()
    reporter.publish_execution_finished(result)

    node_payloads = [payload for payload in payloads if "sub_task" in payload]
    sub_tasks = [payload["sub_task"] for payload in node_payloads]
    assert len(sub_tasks) == 6
    assert all(payload["task_state"] == "RUNNING" for payload in node_payloads)
    assert all(payload["status"] == "RUNNING" for payload in node_payloads)
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
    assert sub_tasks[3]["variables"]["summary_only"] is True
    assert sub_tasks[3]["variables"]["node_count"] >= 2
    assert payloads[-1]["task_state"] == "OVER"
    assert payloads[-1]["terminal_node_id"] == "结束"
    assert payloads[-1]["variables"]["summary_only"] is True


def test_status_reporter_publishes_node_error_payload() -> None:
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    def runner(node, _variable_store):
        if node.node_id == "位姿调整-位控":
            return NodeRunResult(outcome="error", detail={"error": "gdk failure"})
        return NodeRunResult(outcome="success")

    result = TaskflowScheduler(
        taskflow,
        node_runner=runner,
        node_event_handler=reporter.publish_node_event,
    ).run()
    reporter.publish_execution_finished(result)

    error_payload = payloads[3]["sub_task"]
    assert payloads[3]["task_state"] == "ERROR"
    assert payloads[3]["status"] == "ERROR"
    assert error_payload["node_id"] == "位姿调整-位控"
    assert error_payload["state"] == "ERROR"
    assert error_payload["error_msg"] == "gdk failure"
    assert payloads[-1]["task_state"] == "ERROR"


def test_status_reporter_marks_cancelled_execution_as_canceled_terminal_state() -> None:
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    cancellation = TaskflowCancellation(
        app_execution_id=taskflow.app_execution_id,
        request_id="cancel-1",
        reason="operator stop",
        requested_at="2026-08-11T00:00:00+00:00",
    )
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    result = TaskflowScheduler(
        taskflow,
        node_event_handler=reporter.publish_node_event,
        cancel_checker=lambda: cancellation,
    ).run()
    reporter.publish_execution_finished(result)

    assert result.outcome == "cancelled"
    cancel_payload = payloads[1]["sub_task"]
    assert cancel_payload["state"] == "ERROR"
    assert cancel_payload["cancel_state"] == "CANCELED"
    assert cancel_payload["error_code"] == "TASKFLOW_CANCELLED"
    assert payloads[-1]["task_state"] == "CANCELED"
    assert payloads[-1]["status"] == "CANCELED"
    assert payloads[-1]["cancelled"] is True
    assert payloads[-1]["cancel_state"] == "CANCELED"
    assert payloads[-1]["error_code"] == "TASKFLOW_CANCELLED"


def test_status_reporter_publishes_gdk_control_mode_error_code(monkeypatch) -> None:
    error_message = "当前为笛卡尔阻抗模式，请切换到关节位置/规划控制模式后重试"
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {
            "available": False,
            "executed": False,
            "backend": "agibot_gdk.Robot",
            "action": "taskflow_abs_joint",
            "error_stage": "gdk_control_mode_unsupported",
            "error_code": "GDK_CONTROL_MODE_UNSUPPORTED",
            "error_msg": error_message,
        },
    )
    payloads: list[dict[str, object]] = []
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    reporter = TaskflowStatusReporter(
        settings=ExecutorSettings(executor_aid="aid-1"),
        publish_status=payloads.append,
    )

    result = TaskflowScheduler(
        taskflow,
        node_event_handler=reporter.publish_node_event,
    ).run()
    reporter.publish_execution_finished(result)

    error_payload = payloads[3]["sub_task"]
    assert error_payload["state"] == "ERROR"
    assert error_payload["error_code"] == "GDK_CONTROL_MODE_UNSUPPORTED"
    assert error_payload["error_stage"] == "gdk_control_mode_unsupported"
    assert error_payload["error_msg"] == error_message
    assert error_payload["detail"]["gdk_result"]["error_code"] == (
        "GDK_CONTROL_MODE_UNSUPPORTED"
    )
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
            "status_seq": 1,
            "executor_mode": "gdk",
            "error_msg": "YAML 解析失败",
            "error": "YAML 解析失败",
        }
    ]
