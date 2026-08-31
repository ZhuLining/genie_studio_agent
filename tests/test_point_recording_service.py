import json
from pathlib import Path

import pytest

from gsa_taskflow_executor.qr_mapping.point_recording_service import (
    PointRecordingDeletePointParams,
    PointRecordingSaveInitialPhotoParams,
    PointRecordingSaveTargetParams,
    PointRecordingService,
    PointRecordingSubmitParams,
    QrLocalizeSdkError,
    run_qr_localize_sdk,
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


def test_point_recording_service_saves_initial_photo_as_draft(tmp_path) -> None:
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
    assert result["submitted"] is False
    assert result["localizationReady"] is False
    assert (waypoint_dir / "scan_image.jpeg").exists()
    assert (waypoint_dir / "scan_abs_pose.json").exists()
    assert (waypoint_dir / "scan_abs_joints.json").exists()
    assert not (waypoint_dir / "tf_baselink_tag.json").exists()
    assert not (waypoint_dir / "tf_tag_ee.json").exists()
    manifest = json.loads(paths.manifest_path.read_text(encoding="utf-8"))
    assert manifest["projectStatus"] == "recording_dirty"
    assert manifest["activeWaypointName"] == "photo_001"


def test_point_recording_submit_builds_waypoint_after_any_save_order(tmp_path) -> None:
    store, service = make_service(tmp_path)
    make_mapped_project(store)
    service.save_initial_photo_point(
        PointRecordingSaveInitialPhotoParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="photo_001",
            arm="left_arm",
            camera_id="hand_left_color",
        )
    )
    service.save_target_point(
        PointRecordingSaveTargetParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="grasp_1",
            arm="left_arm",
            camera_id="hand_left_color",
        )
    )

    submit = service.submit_recording(
        PointRecordingSubmitParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
        )
    )

    paths = store.build_paths(robot_serial="G2A0004BC01053", project_name="test10")
    waypoint_dir = paths.waypoints_dir / "photo_001"
    manifest = json.loads(paths.manifest_path.read_text(encoding="utf-8"))
    assert submit["available"] is True
    assert submit["targetPointCount"] == 1
    assert submit["initialPhotoPointCount"] == 1
    assert (waypoint_dir / "tf_baselink_tag.json").exists()
    assert (waypoint_dir / "tf_tag_ee.json").exists()
    assert (waypoint_dir / "joints.json").exists()
    assert manifest["projectStatus"] == "recorded"
    assert manifest["pointRecording"]["initialPhotoPoints"][0]["targetPointNames"] == ["grasp_1"]


def test_project_snapshot_lists_point_recording_records(tmp_path) -> None:
    store, service = make_service(tmp_path)
    make_mapped_project(store)
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

    assert snapshot["initialPhotoPoints"][0]["pointName"] == "photo_001"
    assert snapshot["initialPhotoPoints"][0]["scanReady"] is True
    assert snapshot["initialPhotoPoints"][0]["localizationReady"] is False
    assert snapshot["initialPhotoPoints"][0]["tfBaselinkTagPath"] is None


def test_point_recording_service_deletes_target_and_prunes_waypoints(tmp_path) -> None:
    store, service = make_service(tmp_path)
    make_mapped_project(store)
    for name in ("grasp_1", "grasp_2"):
        service.save_target_point(
            PointRecordingSaveTargetParams(
                robot_serial="G2A0004BC01053",
                project_name="test10",
                point_name=name,
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
    service.submit_recording(
        PointRecordingSubmitParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
        )
    )

    result = service.delete_target_point(
        PointRecordingDeletePointParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="grasp_1",
        )
    )

    paths = store.build_paths(robot_serial="G2A0004BC01053", project_name="test10")
    points = json.loads((paths.point_dir / "grasp_points.json").read_text(encoding="utf-8"))
    tag_ee = json.loads((paths.waypoints_dir / "photo_001" / "tf_tag_ee.json").read_text(encoding="utf-8"))
    manifest = json.loads(paths.manifest_path.read_text(encoding="utf-8"))
    [initial] = manifest["pointRecording"]["initialPhotoPoints"]
    assert result["available"] is True
    assert result["action"] == "delete_qr_target_point"
    assert result["affectedInitialPhotoPoints"] == ["photo_001"]
    assert [item["grasp_name"] for item in points] == ["grasp_2"]
    assert "grasp_1" not in tag_ee
    assert initial["targetPointNames"] == ["grasp_2"]
    assert initial["targetPointCount"] == 1
    assert manifest["projectStatus"] == "recording_dirty"


def test_point_recording_service_deletes_initial_photo_waypoint(tmp_path) -> None:
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

    result = service.delete_initial_photo_point(
        PointRecordingDeletePointParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
            point_name="photo_001",
        )
    )

    paths = store.build_paths(robot_serial="G2A0004BC01053", project_name="test10")
    manifest = json.loads(paths.manifest_path.read_text(encoding="utf-8"))
    assert result["available"] is True
    assert result["action"] == "delete_qr_initial_photo_point"
    assert not (paths.waypoints_dir / "photo_001").exists()
    assert manifest["pointRecording"]["initialPhotoPoints"] == []
    assert manifest["activeWaypointName"] is None


def test_point_recording_service_overwrites_existing_target_and_invalidates_old_waypoint(tmp_path) -> None:
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
    service.submit_recording(
        PointRecordingSubmitParams(
            robot_serial="G2A0004BC01053",
            project_name="test10",
        )
    )

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
    tag_ee = json.loads((paths.waypoints_dir / "photo_001" / "tf_tag_ee.json").read_text(encoding="utf-8"))
    manifest = json.loads(paths.manifest_path.read_text(encoding="utf-8"))
    [initial] = manifest["pointRecording"]["initialPhotoPoints"]
    assert result["overwritten"] is True
    assert len(points) == 1
    assert "grasp_1" not in tag_ee
    assert initial["targetPointNames"] == []
    assert initial["targetPointCount"] == 0
    assert manifest["projectStatus"] == "recording_dirty"


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
    grasp_points_path = _point_dir / "grasp_points.json"
    grasp_points = json.loads(grasp_points_path.read_text(encoding="utf-8"))
    tag_ee = {
        item["grasp_name"]: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        for item in grasp_points
    }
    (out_dir / "tf_tag_ee.json").write_text(
        json.dumps(tag_ee),
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
