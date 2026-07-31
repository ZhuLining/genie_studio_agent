from __future__ import annotations

from pathlib import Path

from gsa_taskflow_executor.taskflow.parser import parse_taskflow_yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_systemd_service_uses_gdk_executor_entrypoint() -> None:
    service = (PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.service").read_text(
        encoding="utf-8"
    )

    assert (
        "ExecStart=/opt/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor --listen"
        in service
    )
    assert "EnvironmentFile=/etc/gsa-taskflow-executor/gsa-taskflow-executor.env" in service
    assert "ReadWritePaths=/var/log/gsa-taskflow-executor" in service


def test_deploy_env_template_uses_gdk_mode() -> None:
    env_file = (PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.env.example").read_text(
        encoding="utf-8"
    )

    assert "TASKFLOW_INPUT_TOPIC=gsa/self/taskflow_yaml" in env_file
    assert "TASKFLOW_STATUS_TOPIC_TEMPLATE=gsa/self/{aid}/status" in env_file
    assert (
        "ROBOT_CURRENT_POSE_REQUEST_TOPIC=gsa/self/robot/state/get_current_pose/request"
        in env_file
    )
    assert (
        "ROBOT_CURRENT_POSE_RESPONSE_TOPIC=gsa/self/robot/state/get_current_pose/response"
        in env_file
    )
    assert "EXECUTOR_MODE=gdk" in env_file
    assert "# ENABLE_GDK_CONTROL=1" in env_file
    assert "# CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT" in env_file
    assert "SKILL_REGISTRY_FILE=/etc/gsa-taskflow-executor/skills.yaml" in env_file


def test_example_taskflow_keeps_abs_joint_gdk_yaml() -> None:
    yaml_file = (PROJECT_ROOT / "examples" / "right_arm_abs_joint.yaml").read_text(
        encoding="utf-8"
    )
    taskflow = parse_taskflow_yaml(yaml_file)

    assert "skill_name: motion_plan_skill" in yaml_file
    assert "control_type: ABS_JOINT" in yaml_file
    assert "speed: 0.05" in yaml_file
    assert "app_execution_id: sample-e2e-run" in yaml_file
    assert taskflow.node_ids == ("开始", "位姿调整-位控", "结束")


def test_script_and_motion_example_keeps_whitelisted_script_yaml() -> None:
    yaml_file = (PROJECT_ROOT / "examples" / "script_and_motion.yaml").read_text(
        encoding="utf-8"
    )
    taskflow = parse_taskflow_yaml(yaml_file)

    assert "skill_name: script_skill" in yaml_file
    assert "script_id: gdk_hold_current_dual_arm" in yaml_file
    assert "skill_name: motion_plan_skill" in yaml_file
    assert "speed: 0.05" in yaml_file
    assert taskflow.node_ids == ("开始", "脚本控制", "位姿调整-位控", "结束")


def test_example_skill_registry_includes_whitelisted_script_skill() -> None:
    skills_file = (PROJECT_ROOT / "skills.example.yaml").read_text(encoding="utf-8")

    assert "motion_plan_skill:" in skills_file
    assert "implementation: motion_plan" in skills_file
    assert "script_skill:" in skills_file
    assert "implementation: script" in skills_file
