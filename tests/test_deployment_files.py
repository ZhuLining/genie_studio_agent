from __future__ import annotations

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_systemd_service_uses_mock_executor_entrypoint() -> None:
    service = (PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.service").read_text(
        encoding="utf-8"
    )

    assert (
        "ExecStart=/opt/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor --listen"
        in service
    )
    assert "EnvironmentFile=/etc/gsa-taskflow-executor/gsa-taskflow-executor.env" in service
    assert "ReadWritePaths=/var/log/gsa-taskflow-executor" in service


def test_deploy_env_template_keeps_mock_mode() -> None:
    env_file = (PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.env.example").read_text(
        encoding="utf-8"
    )

    assert "TASKFLOW_INPUT_TOPIC=taskflow/taskflow_yaml" in env_file
    assert "TASKFLOW_STATUS_TOPIC_TEMPLATE=taskflow/{aid}/status" in env_file
    assert "EXECUTOR_MODE=mock" in env_file
    assert "SKILL_REGISTRY_FILE=/etc/gsa-taskflow-executor/skills.yaml" in env_file


def test_example_taskflow_keeps_abs_joint_mock_yaml() -> None:
    yaml_file = (PROJECT_ROOT / "examples" / "right_arm_abs_joint.yaml").read_text(
        encoding="utf-8"
    )

    assert "skill_name: motion_plan_skill" in yaml_file
    assert "control_type: ABS_JOINT" in yaml_file
    assert "app_execution_id: sample-e2e-run" in yaml_file
