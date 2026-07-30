import pytest

import gsa_taskflow_executor.skill_runtime as skill_runtime
from fixtures import VALID_RIGHT_ARM_YAML
from gsa_taskflow_executor.skill_registry import SkillRegistry
from gsa_taskflow_executor.skill_runtime import (
    SkillExecutionContext,
    SkillRuntime,
    SkillRuntimeError,
)
from gsa_taskflow_executor.taskflow_parser import TaskflowNode, parse_taskflow_yaml
from gsa_taskflow_executor.variable_store import VariableStore


def test_assign_skill_resolves_assignments() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    node = taskflow.nodes[0]
    store = VariableStore(variables={"上游": {"detail": {"value": 7}}})
    runtime = SkillRuntime()

    result = runtime.run(
        TaskflowNode(
            node_id=node.node_id,
            node_type=node.node_type,
            assignments={"value": "$.variables.上游.detail.value"},
            skill_name=node.skill_name,
            params_template=node.params_template,
            capture_state_detail=node.capture_state_detail,
            output_var=node.output_var,
            output_contract=node.output_contract,
        ),
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=store,
            mode="gdk",
        ),
    )

    assert result.outcome == "success"
    assert result.outputs == {"assignments": {"value": 7}}


def test_motion_plan_skill_gdk_outputs_structured_motion_result(monkeypatch) -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    node = taskflow.worker_nodes[0]
    runtime = SkillRuntime()

    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params: {"available": True, "executed": True},
    )

    result = runtime.run(
        node,
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=VariableStore(),
            mode="gdk",
        ),
    )

    assert result.outcome == "success"
    assert result.outputs is not None
    assert result.outputs["app_execution_id"] == taskflow.app_execution_id
    assert result.outputs["mode"] == "gdk"
    assert result.outputs["primary_body_part"] == "right_arm"
    assert result.outputs["primary_control_type"] == "ABS_JOINT"
    assert result.outputs["speed"] == 0.05
    assert result.outputs["requested_speed"] == 0.05
    assert result.outputs["requested_speed_unit"] == "gdk_velocity"
    assert result.outputs["timeout"] == 50.0
    assert result.outputs["final_joint"] == [
        0.282,
        -1.039,
        -0.304,
        -1.751,
        -0.621,
        -0.169,
        1.122,
    ]


def test_motion_plan_skill_gdk_resolves_variable_reference(monkeypatch) -> None:
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
    runtime = SkillRuntime()
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params: {"available": True, "executed": True},
    )

    result = runtime.run(
        taskflow.worker_nodes[0],
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=store,
            mode="gdk",
        ),
    )

    assert result.outputs is not None
    assert result.outputs["final_joint"] == [
        0.282,
        -1.039,
        -0.304,
        -1.751,
        -0.621,
        -0.169,
        1.122,
    ]


def test_motion_plan_skill_gdk_adapter_calls_gdk_runtime(monkeypatch) -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    node = taskflow.worker_nodes[0]
    calls: list[tuple[object, dict[str, str] | None]] = []
    runtime_env = {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
    }

    def fake_gdk_runtime(
        motion_params: object,
        *,
        environ: dict[str, str] | None = None,
    ) -> dict[str, object]:
        calls.append((motion_params, environ))
        return {
            "available": True,
            "executed": True,
            "backend": "agibot_gdk.Robot",
            "action": "taskflow_abs_joint",
        }

    monkeypatch.setattr(skill_runtime, "run_gdk_motion_plan_abs_joint", fake_gdk_runtime)
    runtime = SkillRuntime(
        registry=SkillRegistry.from_mapping(
            {
                "skills": {
                    "motion_plan_skill": {
                        "adapter": "gdk",
                        "implementation": "motion_plan",
                    }
                }
            }
        ),
        environ=runtime_env,
    )

    result = runtime.run(
        node,
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=VariableStore(),
            mode="gdk",
        ),
    )

    assert len(calls) == 1
    assert calls[0][1] == runtime_env
    assert result.outcome == "success"
    assert result.detail is not None
    assert result.detail["adapter"] == "gdk"
    assert result.outputs is not None
    assert result.outputs["mode"] == "gdk"
    assert result.outputs["gdk_result"] == {
        "available": True,
        "executed": True,
        "backend": "agibot_gdk.Robot",
        "action": "taskflow_abs_joint",
    }


def test_non_mvp_worker_skill_is_rejected() -> None:
    node = TaskflowNode(
        node_id="二维码定位",
        node_type="worker",
        assignments={},
        skill_name="qr_detect_skill",
        params_template={"camera_id": "wrist", "marker_size": 0.04},
        capture_state_detail=True,
        output_var="二维码定位",
        output_contract={},
    )
    runtime = SkillRuntime()

    with pytest.raises(SkillRuntimeError, match="未注册 skill_name"):
        runtime.run(
            node,
            SkillExecutionContext(
                app_execution_id="run-1",
                variable_store=VariableStore(),
            ),
        )


def test_unsupported_worker_skill_raises_runtime_error() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    node = taskflow.worker_nodes[0]
    runtime = SkillRuntime()

    unsupported_node = TaskflowNode(
        node_id=node.node_id,
        node_type=node.node_type,
        assignments=node.assignments,
        skill_name="unknown_skill",
        params_template=node.params_template,
        capture_state_detail=node.capture_state_detail,
        output_var=node.output_var,
        output_contract=node.output_contract,
    )

    with pytest.raises(SkillRuntimeError, match="未注册 skill_name"):
        runtime.run(
            unsupported_node,
            SkillExecutionContext(
                app_execution_id=taskflow.app_execution_id,
                variable_store=VariableStore(),
            ),
        )
