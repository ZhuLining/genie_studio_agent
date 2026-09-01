from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from gsa_taskflow_executor.taskflow.parser import parse_taskflow_yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_systemd_service_uses_gdk_executor_entrypoint() -> None:
    service = (PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.service").read_text(
        encoding="utf-8"
    )

    assert (
        "ExecStart=/home/u/project/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor --listen"
        in service
    )
    assert "User=u" in service
    assert "User=gsa" not in service
    assert "Group=gsa" not in service
    assert "WorkingDirectory=/home/u/project/gsa_taskflow_executor" in service
    assert "EnvironmentFile=/etc/gsa-taskflow-executor/gsa-taskflow-executor.env" in service
    assert "EnvironmentFile=-/etc/gsa-taskflow-executor/gdk.env" in service
    assert (
        "ExecStartPre=/home/u/project/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor "
        "--deployment-config-check"
    ) in service
    assert (
        "ExecStartPre=/home/u/project/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor "
        "--gdk-env-check"
    ) in service
    assert "ReadWritePaths=/home/u/project/gsa_taskflow_executor/logs /data/gsa" in service
    assert "/data/gsa" in service
    assert "ProtectHome=false" in service
    assert "BindReadOnlyPaths=-/home/u/.cache/agibot/app" in service


def test_runtime_dependencies_do_not_include_unused_pydantic_stack() -> None:
    pyproject = (PROJECT_ROOT / "pyproject.toml").read_text(encoding="utf-8")

    assert '"pydantic' not in pyproject
    assert '"pydantic-settings' not in pyproject


def test_deploy_env_template_uses_gdk_mode() -> None:
    env_file = (PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.env.example").read_text(
        encoding="utf-8"
    )

    assert "TASKFLOW_INPUT_TOPIC=gsa/self/taskflow_yaml" in env_file
    assert "TASKFLOW_CANCEL_TOPIC_FILTER=gsa/self/taskflow/+/cancel" in env_file
    assert "TASKFLOW_STATUS_TOPIC_TEMPLATE=gsa/self/{aid}/status" in env_file
    assert "MQTT_TERMINAL_STATUS_QOS=1" in env_file
    assert "TASKFLOW_QUEUE_MAXSIZE=16" in env_file
    assert "TASKFLOW_QUEUE_FULL_POLICY=reject" in env_file
    assert "MQTT_BROKER_URL=mqtt://127.0.0.1:1883" not in env_file
    assert "\nMQTT_BROKER_URL=\n" in env_file
    assert "ALLOW_LOCAL_MQTT_BROKER=" in env_file
    assert "ROBOT_STATE_QUEUE_MAXSIZE=8" in env_file
    assert "ROBOT_STATE_QUEUE_FULL_POLICY=reject" in env_file
    assert "DIAGNOSTICS_MQTT_CONNECT_TIMEOUT=2.0" in env_file
    assert "PAYLOAD_MAX_STRING_LENGTH=512" in env_file
    assert "PAYLOAD_MAX_COLLECTION_ITEMS=20" in env_file
    assert "PAYLOAD_MAX_DEPTH=6" in env_file
    assert "PAYLOAD_INCLUDE_FULL_VARIABLES=false" in env_file
    assert (
        "ROBOT_CURRENT_POSE_REQUEST_TOPIC=gsa/self/robot/state/get_current_pose/request"
        in env_file
    )
    assert (
        "ROBOT_CURRENT_POSE_RESPONSE_TOPIC=gsa/self/robot/state/get_current_pose/response"
        in env_file
    )
    assert (
        "ROBOT_IDENTITY_REQUEST_TOPIC=gsa/self/robot/state/get_robot_identity/request"
        in env_file
    )
    assert (
        "ROBOT_IDENTITY_RESPONSE_TOPIC=gsa/self/robot/state/get_robot_identity/response"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_FRAME_REQUEST_TOPIC=gsa/self/robot/state/get_camera_frame/request"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_FRAME_RESPONSE_TOPIC=gsa/self/robot/state/get_camera_frame/response"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_CALIBRATION_REQUEST_TOPIC=gsa/self/robot/state/get_camera_calibration/request"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_CALIBRATION_RESPONSE_TOPIC=gsa/self/robot/state/get_camera_calibration/response"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_CAPTURE_START_REQUEST_TOPIC=gsa/self/robot/state/camera_capture/start/request"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_CAPTURE_STOP_REQUEST_TOPIC=gsa/self/robot/state/camera_capture/stop/request"
        in env_file
    )
    assert (
        "ROBOT_CAMERA_CAPTURE_FRAME_TOPIC_TEMPLATE=gsa/self/robot/state/camera_capture/{sessionId}/frame"
        in env_file
    )
    assert "EXECUTOR_MODE=gdk" in env_file
    assert "EXECUTOR_AID=gsa-dev" not in env_file
    assert "\nEXECUTOR_AID=\n" in env_file
    assert "# ENABLE_GDK_CONTROL=1" in env_file
    assert "# CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT" in env_file
    assert "QR_MAPPING_SDK_PATH=/home/u/project/gsa_taskflow_executor/sdk" in env_file
    assert (
        "QR_MAPPING_SDK_PYTHON=/home/u/project/gsa_taskflow_executor/.venv/bin/python"
        in env_file
    )
    assert "QR_LOCALIZE_SDK_PATH=/home/u/project/gsa_taskflow_executor/sdk" in env_file
    assert (
        "QR_LOCALIZE_SDK_PYTHON=/home/u/project/gsa_taskflow_executor/.venv/bin/python"
        in env_file
    )
    assert "EXECUTOR_LOG_DIR=/home/u/project/gsa_taskflow_executor/logs" in env_file
    assert "SKILL_REGISTRY_FILE=/etc/gsa-taskflow-executor/skills.yaml" in env_file
    assert "gsa-taskflow-executor.gdk.env.example" in env_file


def test_deploy_gdk_env_template_documents_systemd_startup_env() -> None:
    env_file = (
        PROJECT_ROOT / "deploy" / "gsa-taskflow-executor.gdk.env.example"
    ).read_text(encoding="utf-8")

    assert "systemd EnvironmentFile" in env_file
    assert "PYTHONPATH" in env_file
    assert "LD_LIBRARY_PATH" in env_file
    assert "CYCLONEDDS_URI" in env_file
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" in env_file
    assert "source /home/u/.cache/agibot/app/env.sh" in env_file


def test_qr_pose_delivery_smoke_script_accepts_valid_baseline(tmp_path: Path) -> None:
    data_root = tmp_path / "gsa_data"
    mapping_sdk = tmp_path / "sdk" / "qr_mapping_sdk"
    localize_sdk = tmp_path / "sdk" / "qr_localize_sdk"
    robot_serial = "G2A0004BC01053"
    project_name = "test10"
    sensor_root = data_root / robot_serial / "sensor"
    project_root = data_root / robot_serial / "qr_pose_skill_conf" / project_name
    images_dir = project_root / "images"
    maps_dir = project_root / "maps"
    point_dir = project_root / "point"
    waypoints_dir = project_root / "waypoints"
    for directory in (
        mapping_sdk,
        localize_sdk,
        sensor_root,
        images_dir,
        maps_dir,
        point_dir,
        waypoints_dir,
    ):
        directory.mkdir(parents=True)

    (sensor_root / "intrinsic_hand_right_rgb.json").write_text("{}", encoding="utf-8")
    (sensor_root / "extrinsic_end_T_hand_right_rgbd.json").write_text("{}", encoding="utf-8")
    (images_dir / "1785315256907.jpg").write_bytes(b"jpeg")
    (maps_dir / "test10.pcd").write_text("VERSION .7\n", encoding="utf-8")
    (maps_dir / "test10.yml").write_text("map: test10\n", encoding="utf-8")
    (maps_dir / "test10-cam.yml").write_text("cam: test10\n", encoding="utf-8")
    (point_dir / "grasp_1.json").write_text("{}", encoding="utf-8")
    (waypoints_dir / "paizhao001.json").write_text("{}", encoding="utf-8")
    env_file = tmp_path / ".env.local"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://127.0.0.1:1883",
                "ALLOW_LOCAL_MQTT_BROKER=1",
                "EXECUTOR_AID=G2A0004BC01053",
                "EXECUTOR_MODE=gdk",
                f"GSA_DATA_ROOT={data_root}",
                f"QR_MAPPING_SDK_PATH={mapping_sdk}",
                f"QR_MAPPING_SDK_PYTHON={sys.executable}",
                f"QR_LOCALIZE_SDK_PATH={localize_sdk}",
                f"QR_LOCALIZE_SDK_PYTHON={sys.executable}",
                "ENABLE_GDK_CONTROL=1",
                "CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT",
            ]
        ),
        encoding="utf-8",
    )
    script = PROJECT_ROOT / "scripts" / "qr_pose_delivery_smoke.py"

    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--env-file",
            str(env_file),
            "--robot-serial",
            robot_serial,
            "--project-name",
            project_name,
            "--strict-safety-gate",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    payload = json.loads(completed.stdout)
    assert payload["ok"] is True
    assert payload["errorCount"] == 0


def test_qr_pose_delivery_smoke_script_rejects_dev_executor_aid(
    tmp_path: Path,
) -> None:
    data_root = tmp_path / "gsa_data"
    mapping_sdk = tmp_path / "sdk" / "qr_mapping_sdk"
    localize_sdk = tmp_path / "sdk" / "qr_localize_sdk"
    for directory in (data_root, mapping_sdk, localize_sdk):
        directory.mkdir(parents=True)
    env_file = tmp_path / ".env.local"
    env_file.write_text(
        "\n".join(
            [
                "MQTT_BROKER_URL=mqtt://broker.internal:1883",
                "EXECUTOR_AID=gsa-dev",
                "EXECUTOR_MODE=gdk",
                f"GSA_DATA_ROOT={data_root}",
                f"QR_MAPPING_SDK_PATH={mapping_sdk}",
                f"QR_MAPPING_SDK_PYTHON={sys.executable}",
                f"QR_LOCALIZE_SDK_PATH={localize_sdk}",
                f"QR_LOCALIZE_SDK_PYTHON={sys.executable}",
                "ENABLE_GDK_CONTROL=1",
                "CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT",
            ]
        ),
        encoding="utf-8",
    )
    script = PROJECT_ROOT / "scripts" / "qr_pose_delivery_smoke.py"

    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--env-file",
            str(env_file),
            "--strict-safety-gate",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 1
    payload = json.loads(completed.stdout)
    checks = {check["name"]: check for check in payload["checks"]}
    assert checks["executor_aid_default"]["ok"] is False


def test_deploy_docs_reference_qr_pose_delivery_baseline() -> None:
    deploy_readme = (PROJECT_ROOT / "deploy" / "README.md").read_text(encoding="utf-8")
    baseline = (
        PROJECT_ROOT.parent / "docs" / "qr_pose_delivery_baseline.md"
    ).read_text(encoding="utf-8")

    assert "qr_pose_delivery_smoke.py" in deploy_readme
    assert "gsa-taskflow-executor.gdk.env.example" in deploy_readme
    assert "--deployment-config-check" in deploy_readme
    assert "--gdk-env-check" in deploy_readme
    assert "BindReadOnlyPaths" in deploy_readme
    assert "docs/qr_pose_delivery_baseline.md" in deploy_readme
    assert "二维码建图工具" in baseline
    assert "点位录制" in baseline
    assert "开始 -> 二维码定位 -> 位姿调整-位控 -> 结束" in baseline


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


def test_script_and_motion_example_keeps_code_script_yaml() -> None:
    yaml_file = (PROJECT_ROOT / "examples" / "script_and_motion.yaml").read_text(
        encoding="utf-8"
    )
    taskflow = parse_taskflow_yaml(yaml_file)

    assert "skill_name: script_skill" in yaml_file
    assert "script_id: code_echo_inputs" in yaml_file
    assert "$.variables.system.detail.outputs.app_execution_id" in yaml_file
    assert "skill_name: motion_plan_skill" in yaml_file
    assert "speed: 0.05" in yaml_file
    assert taskflow.node_ids == ("开始", "代码", "位姿调整-位控", "结束")


def test_end_effector_example_keeps_move_ee_pos_yaml() -> None:
    yaml_file = (PROJECT_ROOT / "examples" / "end_effector_open.yaml").read_text(
        encoding="utf-8"
    )
    taskflow = parse_taskflow_yaml(yaml_file)

    assert "skill_name: control_end_effector_skill" in yaml_file
    assert "target_end: left_tool" in yaml_file
    assert "end_effector_type: omnipicker" in yaml_file
    assert "opening: 0.5" in yaml_file
    assert taskflow.node_ids == ("开始", "末端控制", "结束")


def test_example_skill_registry_includes_gdk_skills() -> None:
    skills_file = (PROJECT_ROOT / "skills.example.yaml").read_text(encoding="utf-8")

    assert "motion_plan_skill:" in skills_file
    assert "implementation: motion_plan" in skills_file
    assert "script_skill:" in skills_file
    assert "implementation: script" in skills_file
    assert "control_end_effector_skill:" in skills_file
    assert "implementation: end_effector" in skills_file
    assert "force_control_skill:" in skills_file
    assert "implementation: force_control" in skills_file
