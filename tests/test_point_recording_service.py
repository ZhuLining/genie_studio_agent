import json
from pathlib import Path

import pytest

from gsa_taskflow_executor.qr_mapping.point_recording_service import (
    PointRecordingSaveInitialPhotoParams,
    PointRecordingSaveTargetParams,
    PointRecordingService,
    QrLocalizeSdkError,
    QrLocalizeQualityError,
    run_qr_localize_sdk,
    validate_qr_localize_quality,
)
from gsa_taskflow_executor.qr_mapping.project_store import QrProjectStore


def test_point_recording_service_saves_target_point(tmp_path) -> None:
    store, service = make_service(tmp_path)
    make_mapped_project(store)

    result = service.save_target_point(
        PointRecordingSaveTargetParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="grasp_1",
            arm="left_arm",
            camera_id="hand_left_color",
        )
    )

    paths = store.build_paths(robot_serial="G2A0004BC01053", project_name="test10")
    points = json.loads((paths.point_dir / "grasp_points.json").read_text(encoding="utf-8"))
    assert result["available"] is True
    assert result["action"] == "save_qr_target_point"
    assert result["pointName"] == "grasp_1"
    assert points[0]["grasp_name"] == "grasp_1"
    assert len(points[0]["abs_joints"]) == 22


def test_point_recording_service_saves_initial_photo_and_runs_sdk(tmp_path) -> None:
    store, service = make_service(tmp_path)
    make_mapped_project(store)

    result = service.save_initial_photo_point(
        PointRecordingSaveInitialPhotoParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="photo_001",
            arm="left_arm",
            camera_id="hand_left_color",
            min_markers=4,
        )
    )

    paths = store.build_paths(robot_serial="G2A0004BC01053", project_name="test10")
    waypoint_dir = paths.waypoints_dir / "photo_001"
    assert result["available"] is True
    assert result["action"] == "save_qr_initial_photo_point"
    assert result["pointKind"] == "initial_photo"
    assert (paths.point_dir / "scan_image.jpeg").exists()
    assert (paths.point_dir / "scan_abs_pose.json").exists()
    assert (paths.point_dir / "scan_abs_joints.json").exists()
    assert (waypoint_dir / "tf_baselink_tag.json").exists()
    assert (waypoint_dir / "tf_tag_ee.json").exists()
    manifest = json.loads(paths.manifest_path.read_text(encoding="utf-8"))
    assert manifest["projectStatus"] == "recorded"
    assert manifest["activeWaypointName"] == "photo_001"


def test_project_snapshot_lists_point_recording_records(tmp_path) -> None:
    store, service = make_service(tmp_path)
    make_mapped_project(store)
    service.save_target_point(
        PointRecordingSaveTargetParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="grasp_1",
            arm="left_arm",
            camera_id="hand_left_color",
        )
    )
    service.save_initial_photo_point(
        PointRecordingSaveInitialPhotoParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="photo_001",
            arm="left_arm",
            camera_id="hand_left_color",
        )
    )

    snapshot = store.get_project_snapshot(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )

    assert snapshot["targetPoints"][0]["pointName"] == "grasp_1"
    assert snapshot["initialPhotoPoints"][0]["pointName"] == "photo_001"
    assert snapshot["initialPhotoPoints"][0]["tfBaselinkTagPath"].endswith(
        "tf_baselink_tag.json"
    )


def test_qr_localize_sdk_failure_keeps_stats(tmp_path, monkeypatch: pytest.MonkeyPatch) -> None:
    stats_path = tmp_path / "locate_stats.json"

    class Completed:
        returncode = 1
        stdout = ""
        stderr = "定位失败: 可见 marker 少于 4 或 PnP 失败"

    def fake_run(*_args, **_kwargs) -> Completed:
        stats_path.write_text(
            json.dumps({"ok": False, "n_markers": 2, "reproj_px": None}),
            encoding="utf-8",
        )
        return Completed()

    monkeypatch.setattr(
        "gsa_taskflow_executor.qr_mapping.point_recording_service.subprocess.run",
        fake_run,
    )

    with pytest.raises(QrLocalizeSdkError) as raised:
        run_qr_localize_sdk(
            "python3",
            None,
            tmp_path,
            tmp_path / "map.yml",
            tmp_path / "camera.yml",
            tmp_path / "extrinsic.json",
            tmp_path / "waypoints",
            4,
            stats_path,
            5.0,
        )

    error = raised.value
    assert error.return_code == 1
    assert error.stats_path == stats_path
    assert error.stats["n_markers"] == 2


def test_qr_localize_quality_gate_rejects_high_reprojection(tmp_path) -> None:
    stats_path = tmp_path / "locate_stats.json"
    output_dir = tmp_path / "waypoints" / "photo_001"

    with pytest.raises(QrLocalizeQualityError) as raised:
        validate_qr_localize_quality(
            {
                "ok": True,
                "n_markers": 6,
                "reproj_px": 399.2342529296875,
                "used_ids": [1, 2, 4, 7, 8, 10],
            },
            min_markers=4,
            max_reproj_px=2.0,
            stats_path=stats_path,
            output_dir=output_dir,
        )

    error = raised.value
    assert "重投影误差 399.234px" in str(error)
    assert error.stats_path == stats_path
    assert error.max_reproj_px == 2.0


def make_service(tmp_path) -> tuple[QrProjectStore, PointRecordingService]:
    store = QrProjectStore(tmp_path)
    service = PointRecordingService(
        project_store=store,
        session_manager=None,  # type: ignore[arg-type]
        snapshot_collector=fake_snapshot_collector,
        sdk_runner=fake_sdk_runner,
    )
    return store, service


def make_mapped_project(store: QrProjectStore) -> None:
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
        json.dumps({"activeMapName": "test10", "projectStatus": "mapped"}),
        encoding="utf-8",
    )


def fake_snapshot_collector(
    action: str,
    arm: str,
    camera_id: str,
    _timeout_ms: int,
    include_image: bool,
    temp_dir: Path,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": True,
        "backend": "fake.gdk",
        "action": action,
        "arm": arm,
        "cameraId": camera_id,
        "absPose": {
            "position": [0.1, 0.2, 0.3],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        "absJoints": [float(index) for index in range(22)],
        "jointNames": [f"joint_{index}" for index in range(22)],
        "collectedAt": "2026-08-20T00:00:00+00:00",
    }
    if include_image:
        temp_dir.mkdir(parents=True, exist_ok=True)
        temp_image = temp_dir / ".scan_image_test.jpeg"
        temp_image.write_bytes(b"fake-jpeg")
        result.update(
            {
                "imageTempPath": str(temp_image),
                "scanImageFileName": "scan_image.jpeg",
                "mimeType": "image/jpeg",
                "width": 1280,
                "height": 1056,
                "timestampNs": 1786071364284463632,
                "imageSha256": "fake-sha256",
                "stationarity": {"motionMm": 0.1, "rotationDeg": 0.1, "accepted": True},
            }
        )
    return result


def fake_sdk_runner(
    _sdk_python: str,
    _sdk_path: Path | None,
    _point_dir: Path,
    _map_yml: Path,
    _calibration: Path,
    _extrinsic: Path,
    out_dir: Path,
    min_markers: int,
    stats_path: Path,
    _timeout_seconds: float,
) -> dict[str, object]:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "tf_baselink_tag.json").write_text(
        json.dumps({"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}),
        encoding="utf-8",
    )
    (out_dir / "tf_tag_ee.json").write_text(
        json.dumps({"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}),
        encoding="utf-8",
    )
    (out_dir / "joints.json").write_text(json.dumps([0.0] * 22), encoding="utf-8")
    stats_path.write_text(
        json.dumps(
            {
                "ok": True,
                "n_markers": min_markers,
                "used_ids": [1, 2, 3, 4],
                "reproj_px": 1.0,
            }
        ),
        encoding="utf-8",
    )
    return {"returnCode": 0, "stdout": "", "stderr": ""}
