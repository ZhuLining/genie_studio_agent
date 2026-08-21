import pytest

import gsa_taskflow_executor.skills.runtime as skill_runtime
from fixtures import (
    VALID_CODE_AND_MOTION_YAML,
    VALID_CODE_CHAIN_YAML,
    VALID_END_EFFECTOR_YAML,
    VALID_RIGHT_ARM_YAML,
)
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.skills.registry import SkillRegistry
from gsa_taskflow_executor.skills.runtime import (
    SkillExecutionContext,
    SkillRuntime,
    SkillRuntimeError,
)
from gsa_taskflow_executor.taskflow.parser import TaskflowNode, parse_taskflow_yaml
from gsa_taskflow_executor.taskflow.variables import VariableStore


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
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
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
        "        action_data: $.variables.二维码定位.detail.outputs.action_data.抓取点A",
    )
    taskflow = parse_taskflow_yaml(yaml_payload)
    store = VariableStore(
        variables={
            "二维码定位": {
                "detail": {
                    "outputs": {
                        "action_data": {
                            "抓取点A": [
                                0.282,
                                -1.039,
                                -0.304,
                                -1.751,
                                -0.621,
                                -0.169,
                                1.122,
                            ],
                        }
                    }
                }
            }
        }
    )
    runtime = SkillRuntime()
    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_motion_plan_abs_joint",
        lambda _motion_params, **_kwargs: {"available": True, "executed": True},
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


def test_qr_pose_skill_gdk_outputs_action_data() -> None:
    node = TaskflowNode(
        node_id="二维码定位",
        node_type="worker",
        assignments={},
        skill_name="qr_pose_skill",
        params_template={
            "robot_serial": "G2A0004BC01053",
            "project_name": "test10",
            "map_name": "test10",
            "initial_photo_point_name": "paizhao001",
            "arm": "left_arm",
            "camera_id": "hand_left_color",
            "timeout": 60,
            "min_markers": 3,
        },
        capture_state_detail=True,
        output_var="二维码定位",
        output_contract={},
    )

    class FakeQrPoseService:
        def locate(self, _params: object) -> dict[str, object]:
            return {
                "available": True,
                "mapName": "test10",
                "targetPointNames": ["zhua1"],
                "pose": [1, 2, 3, 0, 0, 0, 1],
                "poses": {"zhua1": [1, 2, 3, 0, 0, 0, 1]},
                "action_data": {"zhua1": [1, 2, 3, 0, 0, 0, 1]},
                "currentTagPose": [0, 0, 0, 0, 0, 0, 1],
                "quality": {"ok": True},
                "artifactPaths": {"projectRoot": "/home/u/gsa_data/G2A0004BC01053"},
            }

    runtime = SkillRuntime(qr_pose_service=FakeQrPoseService())
    result = runtime.run(
        node,
        SkillExecutionContext(
            app_execution_id="qr-pose-run",
            variable_store=VariableStore(),
            mode="gdk",
        ),
    )

    assert result.outcome == "success"
    assert result.outputs is not None
    assert result.outputs["project_name"] == "test10"
    assert result.outputs["action_data"] == {"zhua1": [1, 2, 3, 0, 0, 0, 1]}
    assert result.outputs["pose"] == [1, 2, 3, 0, 0, 0, 1]


def test_motion_plan_skill_gdk_adapter_calls_gdk_runtime(monkeypatch) -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    node = taskflow.worker_nodes[0]
    calls: list[
        tuple[object, dict[str, str] | None, GdkSessionManager | None, dict[str, object]]
    ] = []
    runtime_env = {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
    }
    gdk_session_manager = GdkSessionManager()

    def fake_gdk_runtime(
        motion_params: object,
        *,
        environ: dict[str, str] | None = None,
        session_manager: GdkSessionManager | None = None,
    ) -> dict[str, object]:
        calls.append((motion_params, environ, session_manager))
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
        gdk_session_manager=gdk_session_manager,
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
    assert calls[0][2] is gdk_session_manager
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


def test_script_skill_gdk_adapter_calls_code_script_runtime(monkeypatch) -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_AND_MOTION_YAML)
    node = taskflow.worker_nodes[0]
    calls: list[
        tuple[object, dict[str, str] | None, GdkSessionManager | None, dict[str, object]]
    ] = []
    runtime_env = {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
    }
    gdk_session_manager = GdkSessionManager()

    def fake_script_runtime(
        script_params: object,
        *,
        environ: dict[str, str] | None = None,
        session_manager: GdkSessionManager | None = None,
        inputs: dict[str, object] | None = None,
    ) -> dict[str, object]:
        calls.append((script_params, environ, session_manager, dict(inputs or {})))
        return {
            "available": True,
            "executed": True,
            "backend": "executor_builtin_code",
            "script_id": "code_echo_inputs",
            "script_action": "echo_inputs",
            "outputs": {"out_1": "code-and-motion-run"},
        }

    monkeypatch.setattr(skill_runtime, "run_code_script", fake_script_runtime)
    runtime = SkillRuntime(environ=runtime_env, gdk_session_manager=gdk_session_manager)
    store = VariableStore(
        variables={
            "system": {
                "detail": {
                    "outputs": {
                        "app_execution_id": taskflow.app_execution_id,
                    }
                }
            }
        }
    )

    result = runtime.run(
        node,
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=store,
            mode="gdk",
        ),
    )

    assert len(calls) == 1
    assert calls[0][1] == runtime_env
    assert calls[0][2] is gdk_session_manager
    assert calls[0][3] == {"out_1": "code-and-motion-run"}
    assert result.outcome == "success"
    assert result.detail is not None
    assert result.detail["adapter"] == "gdk"
    assert result.outputs is not None
    assert result.outputs["script_id"] == "code_echo_inputs"
    assert result.outputs["timeout"] == 50.0
    assert result.outputs["out_1"] == "code-and-motion-run"
    assert result.outputs["script_result"] == {
        "available": True,
        "executed": True,
        "backend": "executor_builtin_code",
        "script_id": "code_echo_inputs",
        "script_action": "echo_inputs",
        "outputs": {"out_1": "code-and-motion-run"},
    }


def test_code_echo_inputs_resolves_and_exports_declared_outputs() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML)
    runtime = SkillRuntime()
    store = VariableStore(
        variables={
            "system": {
                "detail": {
                    "outputs": {
                        "timestamp": "2026-08-02T00:00:00+00:00",
                        "app_execution_id": taskflow.app_execution_id,
                    }
                }
            }
        }
    )

    first_result = runtime.run(
        taskflow.worker_nodes[0],
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=store,
            mode="gdk",
        ),
    )
    assert first_result.outputs is not None
    store.set_node_detail(
        "代码1",
        {
            "status": first_result.outcome,
            "outputs": first_result.outputs,
        },
    )

    second_result = runtime.run(
        taskflow.worker_nodes[1],
        SkillExecutionContext(
            app_execution_id=taskflow.app_execution_id,
            variable_store=store,
            mode="gdk",
        ),
    )

    assert first_result.outputs["out_1"] == taskflow.app_execution_id
    assert first_result.outputs["script_result"]["backend"] == "executor_builtin_code"
    assert second_result.outputs is not None
    assert second_result.outputs["out_2"] == taskflow.app_execution_id


def test_code_node_rejects_missing_variable_reference() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML)
    runtime = SkillRuntime()

    with pytest.raises(SkillRuntimeError, match="变量路径不存在"):
        runtime.run(
            taskflow.worker_nodes[0],
            SkillExecutionContext(
                app_execution_id=taskflow.app_execution_id,
                variable_store=VariableStore(),
                mode="gdk",
            ),
        )


def test_code_node_rejects_input_type_mismatch() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML.replace("type: string", "type: number", 1))
    runtime = SkillRuntime()
    store = VariableStore(
        variables={
            "system": {
                "detail": {
                    "outputs": {
                        "timestamp": "2026-08-02T00:00:00+00:00",
                        "app_execution_id": taskflow.app_execution_id,
                    }
                }
            }
        }
    )

    with pytest.raises(SkillRuntimeError, match="输入参数 out_1 类型不匹配"):
        runtime.run(
            taskflow.worker_nodes[0],
            SkillExecutionContext(
                app_execution_id=taskflow.app_execution_id,
                variable_store=store,
                mode="gdk",
            ),
        )


def test_code_node_rejects_declared_output_type_mismatch(monkeypatch) -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML)
    runtime = SkillRuntime()
    store = VariableStore(
        variables={
            "system": {
                "detail": {
                    "outputs": {
                        "timestamp": "2026-08-02T00:00:00+00:00",
                        "app_execution_id": taskflow.app_execution_id,
                    }
                }
            }
        }
    )

    def fake_script_runtime(
        script_params: object,
        *,
        environ: dict[str, str] | None = None,
        session_manager: GdkSessionManager | None = None,
        inputs: dict[str, object] | None = None,
    ) -> dict[str, object]:
        del script_params, environ, session_manager, inputs
        return {
            "available": True,
            "executed": True,
            "backend": "executor_builtin_code",
            "script_id": "code_echo_inputs",
            "outputs": {"out_1": 7},
        }

    monkeypatch.setattr(skill_runtime, "run_code_script", fake_script_runtime)

    with pytest.raises(SkillRuntimeError, match="输出变量 out_1 类型不匹配"):
        runtime.run(
            taskflow.worker_nodes[0],
            SkillExecutionContext(
                app_execution_id=taskflow.app_execution_id,
                variable_store=store,
                mode="gdk",
            ),
        )


def test_end_effector_skill_gdk_adapter_calls_gdk_runtime(monkeypatch) -> None:
    taskflow = parse_taskflow_yaml(VALID_END_EFFECTOR_YAML)
    node = taskflow.worker_nodes[0]
    calls: list[tuple[object, dict[str, str] | None, GdkSessionManager | None]] = []
    runtime_env = {
        "ENABLE_GDK_CONTROL": "1",
        "CONFIRM_GDK_CONTROL": "TASKFLOW_ABS_JOINT",
    }
    gdk_session_manager = GdkSessionManager()

    def fake_end_effector_runtime(
        end_effector_params: object,
        *,
        environ: dict[str, str] | None = None,
        session_manager: GdkSessionManager | None = None,
    ) -> dict[str, object]:
        calls.append((end_effector_params, environ, session_manager))
        return {
            "available": True,
            "executed": True,
            "backend": "agibot_gdk.Robot",
            "action": "taskflow_end_effector",
        }

    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_end_effector_control",
        fake_end_effector_runtime,
    )
    runtime = SkillRuntime(environ=runtime_env, gdk_session_manager=gdk_session_manager)

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
    assert calls[0][2] is gdk_session_manager
    assert result.outcome == "success"
    assert result.detail is not None
    assert result.detail["adapter"] == "gdk"
    assert result.outputs is not None
    assert result.outputs["target_end"] == "left_tool"
    assert result.outputs["end_effector_type"] == "omnipicker"
    assert result.outputs["opening"] == 0.5
    assert result.outputs["actual_openness"] == [0.5]
    assert result.outputs["actual_openness_source"] == "requested_opening_fallback"
    assert result.outputs["timeout"] == 20.0
    assert result.outputs["end_effector_result"] == {
        "available": True,
        "executed": True,
        "backend": "agibot_gdk.Robot",
        "action": "taskflow_end_effector",
    }


def test_end_effector_skill_gdk_outputs_resolved_dual_side_fields(monkeypatch) -> None:
    node = TaskflowNode(
        node_id="末端控制",
        node_type="worker",
        assignments={},
        skill_name="control_end_effector_skill",
        params_template={
            "target_end": "dual_tool",
            "left_opening": 0.25,
            "right_opening": 0.75,
            "timeout": 20,
        },
        capture_state_detail=True,
        output_var="末端控制",
        output_contract={},
    )

    def fake_end_effector_runtime(
        end_effector_params: object,
        *,
        environ: dict[str, str] | None = None,
        session_manager: GdkSessionManager | None = None,
    ) -> dict[str, object]:
        del environ, session_manager
        return {
            "available": True,
            "executed": True,
            "backend": "agibot_gdk.Robot",
            "action": "taskflow_end_effector",
            "target_end": "dual_tool",
            "end_effector_type": "omnipicker",
            "opening": end_effector_params.opening,
            "left_opening": 0.25,
            "right_opening": 0.75,
            "left_end_effector_type": "omnipicker",
            "right_end_effector_type": "omnipicker",
            "actual_openness_source": "requested_opening_fallback",
        }

    monkeypatch.setattr(
        skill_runtime,
        "run_gdk_end_effector_control",
        fake_end_effector_runtime,
    )

    result = SkillRuntime().run(
        node,
        SkillExecutionContext(
            app_execution_id="dual-end-run",
            variable_store=VariableStore(),
            mode="gdk",
        ),
    )

    assert result.outputs is not None
    assert result.outputs["target_end"] == "dual_tool"
    assert result.outputs["opening"] is None
    assert result.outputs["left_opening"] == pytest.approx(0.25)
    assert result.outputs["right_opening"] == pytest.approx(0.75)
    assert result.outputs["left_end_effector_type"] == "omnipicker"
    assert result.outputs["right_end_effector_type"] == "omnipicker"
    assert result.outputs["end_effector_type"] == "omnipicker"
    assert result.outputs["actual_openness"] == pytest.approx([0.25, 0.75])


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
