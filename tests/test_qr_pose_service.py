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

    def fake_collect_snapshot(**kwargs) -> dict[str, object]:
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
        )
    )

    assert result["available"] is True, result
    assert result["targetPointNames"] == ["zhua1"]
    assert result["action_data"] == {"zhua1": [1.1, 2.2, 3.3, 0.0, 0.0, 0.0, 1.0]}
    assert result["pose"] == {
        "position": [1.1, 2.2, 3.3],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
