import pytest

from fixtures import (
    VALID_CODE_AND_MOTION_YAML,
    VALID_CODE_CHAIN_YAML,
    VALID_END_EFFECTOR_CODE_FLOW_YAML,
    VALID_END_EFFECTOR_YAML,
    VALID_LOOP_TIMER_YAML,
    VALID_RIGHT_ARM_YAML,
)
from gsa_taskflow_executor.taskflow.parser import (
    TaskflowParseError,
    parse_end_effector_params,
    parse_motion_plan_params,
    parse_script_params,
    parse_taskflow_yaml,
)


def test_parse_valid_right_arm_abs_joint_yaml() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)

    assert taskflow.start_node == "开始"
    assert taskflow.app_execution_id == "977ddeb3-3a42-4027-9e6f-5a11bbb6ced9"
    assert taskflow.node_ids == ("开始", "位姿调整-位控", "结束")
    assert len(taskflow.worker_nodes) == 1
    assert taskflow.summary()["transition_count"] == 2


def test_parse_motion_plan_params() -> None:
    taskflow = parse_taskflow_yaml(VALID_RIGHT_ARM_YAML)
    worker = taskflow.worker_nodes[0]
    params = parse_motion_plan_params(worker.params_template, "params_template")

    assert params.speed == 0.05
    assert params.timeout == 50
    assert params.targets[0].body_part == "right_arm"
    assert params.targets[0].control_type == "ABS_JOINT"
    assert params.targets[0].action_data == [
        0.282,
        -1.039,
        -0.304,
        -1.751,
        -0.621,
        -0.169,
        1.122,
    ]


def test_variable_reference_action_data_is_allowed() -> None:
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
    params = parse_motion_plan_params(taskflow.worker_nodes[0].params_template, "params_template")

    assert params.targets[0].action_data == "$.variables.二维码定位.detail.action_data.抓取点A"


def test_parse_script_skill_params() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_AND_MOTION_YAML)
    worker = taskflow.worker_nodes[0]
    params = parse_script_params(worker.params_template, "params_template")

    assert worker.skill_name == "script_skill"
    assert params.script_id == "code_echo_inputs"
    assert params.timeout == 50
    assert params.input_mappings[0].name == "out_1"
    assert params.output_variables[0].name == "out_1"


def test_parse_code_node_input_mapping_and_outputs() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML)
    worker = taskflow.worker_nodes[0]
    params = parse_script_params(worker.params_template, "params_template")

    assert worker.skill_name == "script_skill"
    assert params.script_id == "code_echo_inputs"
    assert params.timeout == 50
    assert params.input_mappings[0].name == "out_1"
    assert params.input_mappings[0].value_type == "string"
    assert params.input_mappings[0].variable_ref == (
        "$.variables.system.detail.outputs.app_execution_id"
    )
    assert params.output_variables[0].name == "out_1"
    assert params.output_variables[0].value_type == "string"


def test_parse_end_effector_code_flow_script_ids() -> None:
    taskflow = parse_taskflow_yaml(VALID_END_EFFECTOR_CODE_FLOW_YAML)
    code_1 = taskflow.worker_nodes[1]
    code_2 = taskflow.worker_nodes[2]
    code_1_params = parse_script_params(code_1.params_template, "params_template")
    code_2_params = parse_script_params(code_2.params_template, "params_template")

    assert code_1_params.script_id == "code_opening_plus_0p1"
    assert code_1_params.input_mappings[0].name == "actual_openness"
    assert code_1_params.output_variables[0].name == "adjusted_opening"
    assert code_2_params.script_id == "code_move_end_effector"
    assert [mapping.name for mapping in code_2_params.input_mappings] == [
        "opening",
        "target_end",
        "end_effector_type",
    ]


def test_parse_code_node_allows_label_style_type() -> None:
    taskflow = parse_taskflow_yaml(VALID_CODE_CHAIN_YAML.replace("type: string", "type: String", 1))
    params = parse_script_params(taskflow.worker_nodes[0].params_template, "params_template")

    assert params.input_mappings[0].value_type == "string"


def test_parse_end_effector_skill_params() -> None:
    taskflow = parse_taskflow_yaml(VALID_END_EFFECTOR_YAML)
    worker = taskflow.worker_nodes[0]
    params = parse_end_effector_params(worker.params_template, "params_template")

    assert worker.skill_name == "control_end_effector_skill"
    assert params.target_end == "left_tool"
    assert params.end_effector_type == "omnipicker"
    assert params.opening == 0.5
    assert params.timeout == 20
    assert params.post_wait_seconds == 1.0


def test_parse_end_effector_skill_params_accepts_post_wait_seconds() -> None:
    taskflow = parse_taskflow_yaml(
        VALID_END_EFFECTOR_YAML.replace(
            "      timeout: 20\n",
            "      timeout: 20\n      post_wait_seconds: 0.5\n",
        )
    )
    params = parse_end_effector_params(taskflow.worker_nodes[0].params_template, "params_template")

    assert params.post_wait_seconds == 0.5


def test_parse_loop_and_timer_nodes() -> None:
    taskflow = parse_taskflow_yaml(VALID_LOOP_TIMER_YAML)
    timer = next(node for node in taskflow.nodes if node.node_id == "定时器")
    loop = next(node for node in taskflow.nodes if node.node_id == "循环")

    assert timer.is_timer
    assert timer.timer_mode == "rel"
    assert timer.duration == 0.2
    assert loop.is_loop
    assert loop.loop_mode == "count"
    assert loop.children == ("代码1", "循环内定时器", "代码2")
    assert loop.iteration_max == 3


def test_reject_loop_cross_boundary_transition() -> None:
    yaml_payload = VALID_LOOP_TIMER_YAML.replace(
        """  - from: 定时器
    outcome: success
    to: 循环""",
        """  - from: 定时器
    outcome: success
    to: 代码1""",
    )

    with pytest.raises(TaskflowParseError, match="循环内部节点只能连接"):
        parse_taskflow_yaml(yaml_payload)


def test_parse_end_effector_params_allows_robot_reported_type() -> None:
    yaml_payload = VALID_END_EFFECTOR_YAML.replace(
        "      end_effector_type: omnipicker\n",
        "      end_effector_type: \"\"\n",
    )

    taskflow = parse_taskflow_yaml(yaml_payload)
    params = parse_end_effector_params(taskflow.worker_nodes[0].params_template, "params_template")

    assert params.end_effector_type is None


def test_reject_end_effector_opening_outside_range() -> None:
    with pytest.raises(TaskflowParseError, match="必须在 0 到 1 之间"):
        parse_taskflow_yaml(VALID_END_EFFECTOR_YAML.replace("opening: 0.5", "opening: 1.2"))


def test_reject_script_id_outside_whitelist() -> None:
    with pytest.raises(TaskflowParseError, match="script_id 未在白名单中"):
        parse_taskflow_yaml(
            VALID_CODE_AND_MOTION_YAML.replace("code_echo_inputs", "run_any_python")
        )


def test_reject_legacy_probe_script_id_in_code_node() -> None:
    with pytest.raises(TaskflowParseError, match="script_id 未在白名单中"):
        parse_taskflow_yaml(
            VALID_CODE_AND_MOTION_YAML.replace(
                "code_echo_inputs",
                "gdk_hold_current_dual_arm",
            )
        )


def test_reject_code_node_duplicate_output_name() -> None:
    yaml_payload = VALID_CODE_CHAIN_YAML.replace(
        """      output_variables:
        - name: out_1
          type: string""",
        """      output_variables:
        - name: out_1
          type: string
        - name: out_1
          type: string""",
        1,
    )

    with pytest.raises(TaskflowParseError, match="name 重复: out_1"):
        parse_taskflow_yaml(yaml_payload)


def test_reject_code_node_variable_ref_outside_variable_store() -> None:
    yaml_payload = VALID_CODE_CHAIN_YAML.replace(
        "$.variables.system.detail.outputs.app_execution_id",
        "$.context.app_execution_id",
        1,
    )

    with pytest.raises(TaskflowParseError, match="必须是 \\$\\.variables\\. 开头"):
        parse_taskflow_yaml(yaml_payload)


def test_reject_missing_start_node() -> None:
    with pytest.raises(TaskflowParseError, match="start_node 不存在"):
        parse_taskflow_yaml(VALID_RIGHT_ARM_YAML.replace("start_node: 开始", "start_node: 不存在"))


def test_reject_duplicate_node_id() -> None:
    with pytest.raises(TaskflowParseError, match="节点 ID 重复"):
        parse_taskflow_yaml(VALID_RIGHT_ARM_YAML.replace("id: 结束", "id: 开始"))


def test_reject_transition_to_unknown_node() -> None:
    with pytest.raises(TaskflowParseError, match="transition.to 不存在"):
        parse_taskflow_yaml(VALID_RIGHT_ARM_YAML.replace("to: 结束", "to: 不存在"))


def test_reject_non_abs_joint() -> None:
    with pytest.raises(TaskflowParseError, match="只支持 ABS_JOINT"):
        parse_taskflow_yaml(
            VALID_RIGHT_ARM_YAML.replace("control_type: ABS_JOINT", "control_type: ABS_POSE")
        )


def test_reject_wrong_joint_count() -> None:
    yaml_payload = VALID_RIGHT_ARM_YAML.replace("          - 1.122\n", "")

    with pytest.raises(TaskflowParseError, match="长度必须是 7"):
        parse_taskflow_yaml(yaml_payload)


def test_reject_motion_speed_outside_gdk_velocity_range() -> None:
    with pytest.raises(TaskflowParseError, match="必须在 0.001 到 0.1 之间"):
        parse_taskflow_yaml(VALID_RIGHT_ARM_YAML.replace("speed: 0.05", "speed: 0.5"))


def test_reject_non_finite_motion_speed() -> None:
    with pytest.raises(TaskflowParseError, match="必须是数字"):
        parse_taskflow_yaml(VALID_RIGHT_ARM_YAML.replace("speed: 0.05", "speed: .nan"))


def test_parser_allows_structural_worker_skill_name() -> None:
    taskflow = parse_taskflow_yaml(
        VALID_RIGHT_ARM_YAML.replace("motion_plan_skill", "qr_pose_skill")
    )

    assert taskflow.worker_nodes[0].skill_name == "qr_pose_skill"
