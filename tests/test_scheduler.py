import pytest

import gsa_taskflow_executor.skills.runtime as skill_runtime
from fixtures import (
    VALID_CODE_AND_MOTION_YAML,
    VALID_CODE_CHAIN_YAML,
    VALID_END_EFFECTOR_CODE_FLOW_YAML,
    VALID_RIGHT_ARM_YAML,
)
from gsa_taskflow_executor.taskflow.parser import parse_taskflow_yaml
from gsa_taskflow_executor.taskflow.scheduler import (
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


def test_scheduler_code_nodes_pass_declared_outputs_downstream() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML)

    result = TaskflowScheduler(taskflow).run()

    assert result.outcome == "success"
    assert result.visited_node_ids == ("开始", "代码1", "代码2", "结束")
    assert result.variables["system"]["detail"]["outputs"]["app_execution_id"] == "code-chain-run"
    assert result.variables["代码1"]["detail"]["outputs"]["out_1"] == "code-chain-run"
    assert result.variables["代码2"]["detail"]["outputs"]["out_2"] == "code-chain-run"


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
