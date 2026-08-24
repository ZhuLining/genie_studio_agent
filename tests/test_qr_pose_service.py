import json
from pathlib import Path

from gsa_taskflow_executor.qr_mapping.pose_service import QrPoseService
from gsa_taskflow_executor.qr_mapping.project_store import QrProjectStore
from gsa_taskflow_executor.taskflow.models import QrPoseParams


def test_qr_pose_service_outputs_target_action_data(tmp_path, monkeypatch) -> None:
    store = QrProjectStore(tmp_path)
    paths = store.ensure_project_layout(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )
    (paths.maps_dir / "test10.yml").write_text("markers: []\n", encoding="utf-8")
    (paths.maps_dir / "test10-cam.yml").write_text("camera: {}\n", encoding="utf-8")
    (paths.sensor_root / "extrinsic_end_T_hand_left_rgbd.json").write_text(
        json.dumps({"transform": [1, 0, 0, 0]}),
        encoding="utf-8",
    )
    (paths.manifest_path).write_text(
        json.dumps({"activeMapName": "test10"}),
        encoding="utf-8",
    )
    waypoint_dir = paths.waypoints_dir / "paizhao001"
    waypoint_dir.mkdir(parents=True, exist_ok=True)
    (waypoint_dir / "joints.json").write_text(
        json.dumps(
            {
                "recorded_commands": [
                    {
                        "index_name": "scan",
                        "joint_names": [
                            "idx21_arm_l_joint1",
                            "idx22_arm_l_joint2",
                            "idx23_arm_l_joint3",
                            "idx24_arm_l_joint4",
                            "idx25_arm_l_joint5",
                            "idx26_arm_l_joint6",
                            "idx27_arm_l_joint7",
                        ],
                        "joint_positions": [0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    (waypoint_dir / "tf_tag_ee.json").write_text(
        json.dumps(
            {
                "zhua1": [
                    [1, 0, 0, 0.1],
                    [0, 1, 0, 0.2],
                    [0, 0, 1, 0.3],
                    [0, 0, 0, 1],
                ]
            }
        ),
        encoding="utf-8",
    )

    calls: list[str] = []

    def fake_return_motion(motion_params, **_kwargs) -> dict[str, object]:
        calls.append("return_motion")
        assert motion_params.targets[0].body_part == "left_arm"
        assert motion_params.targets[0].action_data == [
            0.11,
            0.12,
            0.13,
            0.14,
            0.15,
            0.16,
            0.17,
        ]
        assert motion_params.speed == 0.03
        assert motion_params.timeout == 50
        return {"available": True, "executed": True, "groups": []}

    def fake_collect_snapshot(**kwargs) -> dict[str, object]:
        calls.append("collect_snapshot")
        temp_dir = Path(kwargs["temp_dir"])
        temp_dir.mkdir(parents=True, exist_ok=True)
        image_path = temp_dir / ".scan_image_test.jpeg"
        image_path.write_bytes(b"jpeg")
        return {
            "available": True,
            "imageTempPath": str(image_path),
            "scanImageFileName": "scan_image.jpeg",
            "absPose": {
                "position": [0, 0, 0],
                "orientation": [0, 0, 0, 1],
            },
            "absJoints": [0.0] * 22,
            "cameraId": "hand_left_color",
            "timestampNs": 1786071364284463632,
            "imageSha256": "fake",
            "stationarity": {"motionMm": 0.0, "rotationDeg": 0.0},
            "collectedAt": "2026-08-21T00:00:00+00:00",
        }

    def fake_run_sdk(
        _python: str,
        _sdk_path: Path | None,
        _point_dir: Path,
        _map_yml: Path,
        _calibration: Path,
        _extrinsic: Path,
        out_dir: Path,
        _min_markers: int,
        stats_path: Path,
        _timeout_seconds: float,
    ) -> dict[str, object]:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "tf_baselink_tag.json").write_text(
            json.dumps(
                [
                    [1, 0, 0, 1.0],
                    [0, 1, 0, 2.0],
                    [0, 0, 1, 3.0],
                    [0, 0, 0, 1.0],
                ]
            ),
            encoding="utf-8",
        )
        stats_path.write_text(
            json.dumps({"ok": True, "n_markers": 4, "used_ids": [1, 2, 3, 4]}),
            encoding="utf-8",
        )
        return {"returnCode": 0}

    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.pose_service.run_gdk_motion_plan_abs_joint",
        fake_return_motion,
    )
    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.pose_service.collect_point_recording_gdk_snapshot",
        fake_collect_snapshot,
    )
    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.pose_service.run_qr_localize_sdk",
        fake_run_sdk,
    )

    service = QrPoseService(
        project_store=store,
        session_manager=None,  # type: ignore[arg-type]
    )
    result = service.locate(
        QrPoseParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            initial_photo_point_name="paizhao001",
            map_name="test10",
            arm="left_arm",
            camera_id="hand_left_color",
            timeout=60,
            min_markers=3,
            return_to_initial_photo_pose=True,
            return_pose_speed=0.03,
            return_pose_timeout=50,
        )
    )

    assert result["available"] is True, result
    assert calls == ["return_motion", "collect_snapshot"]
    assert result["targetPointNames"] == ["zhua1"]
    assert result["initialPhotoReturn"]["executed"] is True
    assert result["action_data"] == {"zhua1": [1.1, 2.2, 3.3, 0.0, 0.0, 0.0, 1.0]}
    assert result["pose"] == {
        "position": [1.1, 2.2, 3.3],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }


def test_qr_pose_service_reports_initial_photo_return_failure(tmp_path, monkeypatch) -> None:
    store = QrProjectStore(tmp_path)
    paths = store.ensure_project_layout(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )
    (paths.maps_dir / "test10.yml").write_text("markers: []\n", encoding="utf-8")
    (paths.maps_dir / "test10-cam.yml").write_text("camera: {}\n", encoding="utf-8")
    (paths.sensor_root / "extrinsic_end_T_hand_right_rgbd.json").write_text(
        json.dumps({"transform": [1, 0, 0, 0]}),
        encoding="utf-8",
    )
    paths.manifest_path.write_text(
        json.dumps({"activeMapName": "test10"}),
        encoding="utf-8",
    )
    waypoint_dir = paths.waypoints_dir / "paizhao001"
    waypoint_dir.mkdir(parents=True, exist_ok=True)
    (waypoint_dir / "joints.json").write_text(
        json.dumps(
            {
                "recorded_commands": [
                    {
                        "index_name": "scan",
                        "joint_names": [
                            "idx61_arm_r_joint1",
                            "idx62_arm_r_joint2",
                            "idx63_arm_r_joint3",
                            "idx64_arm_r_joint4",
                            "idx65_arm_r_joint5",
                            "idx66_arm_r_joint6",
                            "idx67_arm_r_joint7",
                        ],
                        "joint_positions": [0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    (waypoint_dir / "tf_tag_ee.json").write_text(
        json.dumps(
            {
                "zhua1": [
                    [1, 0, 0, 0.1],
                    [0, 1, 0, 0.2],
                    [0, 0, 1, 0.3],
                    [0, 0, 0, 1],
                ]
            }
        ),
        encoding="utf-8",
    )

    def fake_return_motion(_motion_params, **_kwargs) -> dict[str, object]:
        return {
            "available": False,
            "executed": False,
            "error_msg": "CONFIRM_GDK_CONTROL mismatch",
        }

    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.pose_service.run_gdk_motion_plan_abs_joint",
        fake_return_motion,
    )

    service = QrPoseService(
        project_store=store,
        session_manager=None,  # type: ignore[arg-type]
    )
    result = service.locate(
        QrPoseParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            initial_photo_point_name="paizhao001",
            map_name="test10",
            arm="right_arm",
            camera_id="hand_right_color",
            timeout=60,
            min_markers=3,
            return_to_initial_photo_pose=True,
            return_pose_speed=0.03,
            return_pose_timeout=50,
        )
    )

    assert result["available"] is False
    assert result["errorCode"] == "QR_POSE_INITIAL_RETURN_FAILED"
    assert result["initialPhotoReturn"]["bodyPart"] == "right_arm"
    assert result["initialPhotoReturn"]["executed"] is False


def test_qr_pose_service_reports_missing_initial_photo_joints_as_return_failure(
    tmp_path,
) -> None:
    store = QrProjectStore(tmp_path)
    paths = store.ensure_project_layout(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )
    (paths.maps_dir / "test10.yml").write_text("markers: []\n", encoding="utf-8")
    (paths.maps_dir / "test10-cam.yml").write_text("camera: {}\n", encoding="utf-8")
    (paths.sensor_root / "extrinsic_end_T_hand_left_rgbd.json").write_text(
        json.dumps({"transform": [1, 0, 0, 0]}),
        encoding="utf-8",
    )
    paths.manifest_path.write_text(
        json.dumps({"activeMapName": "test10"}),
        encoding="utf-8",
    )
    waypoint_dir = paths.waypoints_dir / "paizhao001"
    waypoint_dir.mkdir(parents=True, exist_ok=True)
    (waypoint_dir / "tf_tag_ee.json").write_text(
        json.dumps(
            {
                "zhua1": [
                    [1, 0, 0, 0.1],
                    [0, 1, 0, 0.2],
                    [0, 0, 1, 0.3],
                    [0, 0, 0, 1],
                ]
            }
        ),
        encoding="utf-8",
    )

    service = QrPoseService(
        project_store=store,
        session_manager=None,  # type: ignore[arg-type]
    )
    result = service.locate(
        QrPoseParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            initial_photo_point_name="paizhao001",
            map_name="test10",
            arm="left_arm",
            camera_id="hand_left_color",
            timeout=60,
            min_markers=3,
            return_to_initial_photo_pose=True,
            return_pose_speed=0.03,
            return_pose_timeout=50,
        )
    )

    assert result["available"] is False
    assert result["errorCode"] == "QR_POSE_INITIAL_RETURN_FAILED"
    assert result["initialPhotoReturn"]["errorStage"] == "load_initial_photo_joints"
