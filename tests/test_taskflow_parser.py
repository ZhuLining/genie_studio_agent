import pytest

from fixtures import VALID_RIGHT_ARM_YAML
from gsa_taskflow_executor.taskflow_parser import (
    TaskflowParseError,
    parse_motion_plan_params,
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
