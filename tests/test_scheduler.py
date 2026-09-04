import pytest

import gsa_taskflow_executor.skills.runtime as skill_runtime
from fixtures import (
    VALID_CODE_AND_MOTION_YAML,
    VALID_CODE_CHAIN_YAML,
    VALID_END_EFFECTOR_CODE_FLOW_YAML,
    VALID_FORCE_CONTROL_YAML,
    VALID_LOOP_TIMER_YAML,
    VALID_RIGHT_ARM_YAML,
)
from gsa_taskflow_executor.taskflow.control import TaskflowCancellation
from gsa_taskflow_executor.taskflow.parser import TaskflowParseError, parse_taskflow_yaml
from gsa_taskflow_executor.taskflow.scheduler import (
    OUTPUT_CONTRACT_VIOLATION_CODE,
    NodeExecutionEvent,
    NodeRunResult,
    TaskflowScheduleError,
    TaskflowScheduler,
)
from gsa_taskflow_executor.taskflow.variables import VariableStore


def test_gdk_scheduler_walks_linear_taskflow(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "success"
    assert result.terminal_node_id == "结束"
    assert result.visited_node_ids == ("开始", "位姿调整-位控", "结束")
    assert result.summary()["step_count"] == 3
    assert result.variables["位姿调整-位控"]["detail"]["status"] == "success"
    assert result.variables["位姿调整-位控"]["detail"]["mode"] == "gdk"


def test_scheduler_treats_explicit_end_node_as_terminal_without_node_runner() -> None:
    taskflow = parse_taskflow_yaml(
        """
start_node: 开始
app_execution_id: explicit-end-run
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 正常结束
    type: end
transitions:
  - from: 开始
    outcome: success
    to: 正常结束
"""
    )
    called_nodes: list[str] = []

    def runner(node, _variable_store):
        called_nodes.append(node.node_id)
        return NodeRunResult(outcome="success")

    result = TaskflowScheduler(taskflow, node_runner=runner).run()

    assert result.outcome == "success"
    assert result.terminal_node_id == "正常结束"
    assert result.visited_node_ids == ("开始", "正常结束")
    assert called_nodes == ["开始"]
    assert result.variables["正常结束"]["detail"]["terminal"] is True


def test_gdk_scheduler_walks_mixed_code_and_motion_taskflow(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_code_script",
        lambda _script_params, **_kwargs: {
            "available": True,
            "executed": True,
            "script_id": "code_echo_inputs",
            "outputs": {"out_1": "code-and-motion-run"},
        },
    )
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
    taskflow = parse_taskflow_yaml(VALID_CODE_AND_MOTION_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "success"
    assert result.terminal_node_id == "结束"
    assert result.visited_node_ids == ("开始", "代码", "位姿调整-位控", "结束")
    assert result.summary()["step_count"] == 4
    assert result.variables["代码"]["detail"]["status"] == "success"
    assert result.variables["代码"]["detail"]["outputs"]["script_id"] == "code_echo_inputs"
    assert result.variables["代码"]["detail"]["outputs"]["out_1"] == "code-and-motion-run"
    assert result.variables["位姿调整-位控"]["detail"]["status"] == "success"


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


def test_scheduler_stops_on_cooperative_cancellation() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    cancellation = TaskflowCancellation(
        app_execution_id=taskflow.app_execution_id,
        request_id="cancel-1",
        reason="operator stop",
        requested_at="2026-08-11T00:00:00+00:00",
    )

    result = TaskflowScheduler(
        taskflow,
        cancel_checker=lambda: cancellation,
    ).run()

    assert result.outcome == "cancelled"
    assert result.terminal_node_id == "开始"
    assert result.visited_node_ids == ("开始",)
    assert result.variables["开始"]["detail"]["status"] == "cancelled"
    assert result.variables["开始"]["detail"]["error_code"] == "TASKFLOW_CANCELLED"


def test_scheduler_maps_gdk_cancel_error_to_cancelled(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {
            "available": False,
            "executed": False,
            "error_code": "GDK_OPERATION_CANCELLED",
            "error_msg": "cancelled",
        },
    )
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "cancelled"
    assert result.terminal_node_id == "位姿调整-位控"
    assert result.visited_node_ids == ("开始", "位姿调整-位控")
    assert result.variables["位姿调整-位控"]["detail"]["status"] == "cancelled"
    assert result.variables["位姿调整-位控"]["detail"]["error_code"] == (
        "GDK_OPERATION_CANCELLED"
    )


def test_scheduler_resolves_variable_references_before_worker(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
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


def test_scheduler_rejects_missing_output_contract_path(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
    yaml_payload = VALID_RIGHT_ARM_YAML.replace(
        "$.variables.位姿调整-位控.detail",
        "$.variables.位姿调整-位控.detail.outputs.missing_value",
    )
    taskflow = parse_taskflow_yaml(yaml_payload)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "error"
    assert result.terminal_node_id == "位姿调整-位控"
    assert result.visited_node_ids == ("开始", "位姿调整-位控")
    worker_detail = result.variables["位姿调整-位控"]["detail"]
    assert worker_detail["status"] == "error"
    assert worker_detail["error_code"] == OUTPUT_CONTRACT_VIOLATION_CODE
    assert worker_detail["error_stage"] == "output_contract"
    assert worker_detail["missing_paths"] == [
        "$.variables.位姿调整-位控.detail.outputs.missing_value"
    ]
    assert "结束" not in result.variables


def test_scheduler_skips_output_contract_validation_for_failed_node() -> None:
    yaml_payload = VALID_RIGHT_ARM_YAML.replace(
        "$.variables.位姿调整-位控.detail",
        "$.variables.位姿调整-位控.detail.outputs.missing_value",
    )
    taskflow = parse_taskflow_yaml(yaml_payload)

    def runner(node, _variable_store):
        if node.node_id == "位姿调整-位控":
            return NodeRunResult(
                outcome="error",
                detail={"error": "worker failed", "error_code": "WORKER_FAILED"},
            )
        return NodeRunResult(outcome="success")

    result = TaskflowScheduler(taskflow, node_runner=runner).run()

    assert result.outcome == "error"
    worker_detail = result.variables["位姿调整-位控"]["detail"]
    assert worker_detail["error_code"] == "WORKER_FAILED"
    assert worker_detail["error"] == "worker failed"
    assert "missing_paths" not in worker_detail


def test_scheduler_code_nodes_pass_declared_outputs_downstream() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "success"
    assert result.visited_node_ids == ("开始", "代码1", "代码2", "结束")
    assert result.variables["system"]["detail"]["outputs"]["app_execution_id"] == "code-chain-run"
    assert result.variables["代码1"]["detail"]["outputs"]["out_1"] == "code-chain-run"
    assert result.variables["代码2"]["detail"]["outputs"]["out_2"] == "code-chain-run"


def test_scheduler_timer_and_count_loop_execute_in_order() -> None:
    taskflow = parse_taskflow_yaml(VALID_LOOP_TIMER_YAML)
    sleep_calls: list[float] = []
    worker_counts: dict[str, int] = {}

    def runner(node, _variable_store):
        worker_counts[node.node_id] = worker_counts.get(node.node_id, 0) + 1
        count = worker_counts[node.node_id]
        output_name = "out_1" if node.node_id == "代码1" else "out_2"
        return NodeRunResult(
            outcome="success",
            outputs={output_name: f"{node.node_id}-{count}"},
        )

    result = TaskflowScheduler(
        taskflow,
        node_runner=runner,
        sleep=sleep_calls.append,
    ).run()

    assert result.outcome == "success"
    assert result.terminal_node_id == "结束"
    assert result.visited_node_ids == (
        "开始",
        "定时器",
        "代码1",
        "循环内定时器",
        "代码2",
        "代码1",
        "循环内定时器",
        "代码2",
        "代码1",
        "循环内定时器",
        "代码2",
        "循环",
        "结束",
    )
    assert sleep_calls == [0.2, 0.1, 0.1, 0.1]
    assert result.variables["代码1"]["detail"]["outputs"]["out_1"] == "代码1-3"
    assert result.variables["代码2"]["detail"]["outputs"]["out_2"] == "代码2-3"
    loop_outputs = result.variables["循环"]["detail"]["outputs"]
    assert loop_outputs["contract_version"] == 2
    assert loop_outputs["completed_iterations"] == 3
    assert loop_outputs["iteration_max"] == 3
    assert [item["outcome"] for item in loop_outputs["iteration_results"]] == [
        "success",
        "success",
        "success",
    ]
    assert [
        item["nodes"]["代码1"]["detail"]["outputs"]["out_1"]
        for item in loop_outputs["iterations"]
    ] == ["代码1-1", "代码1-2", "代码1-3"]
    assert loop_outputs["last_iteration"]["nodes"]["代码2"]["detail"]["outputs"]["out_2"] == (
        "代码2-3"
    )
    assert result.variables["循环"]["detail"]["outputs"]["iterations"][0]["nodes"]["代码1"][
        "detail"
    ]["outputs"]["out_1"] == "代码1-1"


def test_scheduler_timer_can_be_interrupted_by_cancellation() -> None:
    taskflow = parse_taskflow_yaml(VALID_LOOP_TIMER_YAML)
    cancellation = TaskflowCancellation(
        app_execution_id=taskflow.app_execution_id,
        request_id="cancel-timer",
        reason="operator stop",
        requested_at="2026-08-11T00:00:00+00:00",
    )
    cancel_enabled = False

    def checker() -> TaskflowCancellation | None:
        return cancellation if cancel_enabled else None

    def sleep(_duration: float) -> None:
        nonlocal cancel_enabled
        cancel_enabled = True

    result = TaskflowScheduler(
        taskflow,
        sleep=sleep,
        cancel_checker=checker,
    ).run()

    assert result.outcome == "cancelled"
    assert result.terminal_node_id == "定时器"
    assert result.visited_node_ids == ("开始", "定时器")
    assert result.variables["定时器"]["detail"]["timer_interrupted"] is True


def test_scheduler_loop_failure_marks_parent_loop_error() -> None:
    taskflow = parse_taskflow_yaml(VALID_LOOP_TIMER_YAML)

    def runner(node, _variable_store):
        if node.node_id == "代码2":
            return NodeRunResult(outcome="error", detail={"error": "child failed"})
        return NodeRunResult(outcome="success")

    result = TaskflowScheduler(
        taskflow,
        node_runner=runner,
        sleep=lambda _duration: None,
    ).run()

    assert result.outcome == "error"
    assert result.terminal_node_id == "循环"
    assert result.visited_node_ids == ("开始", "定时器", "代码1", "循环内定时器", "代码2", "循环")
    loop_detail = result.variables["循环"]["detail"]
    assert loop_detail["status"] == "error"
    assert loop_detail["failed_iteration"] == 1
    assert loop_detail["failed_child_node"] == "代码2"
    assert loop_detail["outputs"]["completed_iterations"] == 0
    assert loop_detail["outputs"]["contract_version"] == 2
    assert loop_detail["outputs"]["iterations"][0]["failed_child_node"] == "代码2"


def test_scheduler_force_control_stub_returns_unverified_error() -> None:
    taskflow = parse_taskflow_yaml(VALID_FORCE_CONTROL_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "error"
    assert result.terminal_node_id == "位姿调整-力控"
    assert result.visited_node_ids == ("开始", "位姿调整-力控")
    force_detail = result.variables["位姿调整-力控"]["detail"]
    assert force_detail["status"] == "error"
    assert force_detail["error_code"] == "GDK_FORCE_CONTROL_UNVERIFIED"
    assert force_detail["gdk_result"]["reason"] == "GDK_FORCE_CONTROL_UNVERIFIED"
    assert force_detail["gdk_result"]["params"]["arm"] == "left_arm"


def test_scheduler_end_effector_code_flow_passes_adjusted_opening(monkeypatch) -> None:
    end_effector_openings: list[float] = []
    real_run_code_script = skill_runtime.run_code_script

    def fake_end_effector_runtime(
        end_effector_params: object,
        **_kwargs: object,
    ) -> dict[str, object]:
        end_effector_openings.append(end_effector_params.opening)
        return {
            "available": True,
            "executed": True,
            "backend": "agibot_gdk.Robot",
            "target_end": end_effector_params.target_end,
            "end_effector_type": end_effector_params.end_effector_type or "omnipicker",
            "opening": end_effector_params.opening,
            "actual_openness": [end_effector_params.opening],
        }

    def fake_code_script(
        script_params: object,
        *,
        inputs: dict[str, object] | None = None,
        **kwargs: object,
    ) -> dict[str, object]:
        if script_params.script_id != "code_move_end_effector":
            return real_run_code_script(script_params, inputs=inputs, **kwargs)

        opening = float((inputs or {})["opening"])
        end_effector_openings.append(opening)
        return {
            "available": True,
            "executed": True,
            "backend": "agibot_gdk.Robot",
            "script_id": "code_move_end_effector",
            "script_action": "move_end_effector",
            "outputs": {
                "actual_openness": [opening],
                "target_end": (inputs or {}).get("target_end", "left_tool"),
                "end_effector_type": (inputs or {}).get("end_effector_type", ""),
            },
        }

    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_end_effector_control",
        fake_end_effector_runtime,
    )
    monkeypatch.setattr(skill_runtime, "run_code_script", fake_code_script)

    taskflow = parse_taskflow_yaml(VALID_END_EFFECTOR_CODE_FLOW_YAML)
    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "success"
    assert result.visited_node_ids == ("开始", "末端控制", "代码1", "代码2", "结束")
    assert result.variables["末端控制"]["detail"]["outputs"]["actual_openness"] == [0.5]
    assert result.variables["末端控制"]["detail"]["outputs"]["target_end"] == "left_tool"
    assert result.variables["末端控制"]["detail"]["outputs"]["end_effector_type"] == "omnipicker"
    assert result.variables["代码1"]["detail"]["outputs"]["adjusted_opening"] == pytest.approx(0.6)
    assert result.variables["代码2"]["detail"]["outputs"]["actual_openness"] == pytest.approx([0.6])
    assert end_effector_openings == pytest.approx([0.5, 0.6])


def test_scheduler_emits_node_execution_events(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
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


def test_scheduler_rejects_cycle(monkeypatch) -> None:
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
    )
    yaml_payload = VALID_RIGHT_ARM_YAML.replace(
        """  - from: 位姿调整-位控
    outcome: success
    to: 结束""",
        """  - from: 位姿调整-位控
    outcome: success
    to: 开始""",
    )

    with pytest.raises(TaskflowParseError, match="start_node 不能有入边"):
        parse_taskflow_yaml(yaml_payload)


def test_scheduler_rejects_duplicate_success_transition() -> None:
    yaml_payload = (
        VALID_RIGHT_ARM_YAML
        + """
  - from: 开始
    outcome: success
    to: 结束
"""
    )
    with pytest.raises(TaskflowParseError, match="transition 不唯一"):
        parse_taskflow_yaml(yaml_payload)


def test_scheduler_rejects_max_steps() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    with pytest.raises(TaskflowScheduleError, match="超过最大步数"):
        TaskflowScheduler(taskflow, max_steps=1).run()
