import json

import pytest

from gsa_taskflow_executor.qr_mapping.project_store import (
    QrProjectStore,
    QrProjectStoreError,
)


def test_project_store_returns_not_found_snapshot(tmp_path) -> None:
    store = QrProjectStore(tmp_path)

    snapshot = store.get_project_snapshot(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )

    assert snapshot["available"] is True
    assert snapshot["exists"] is False
    assert snapshot["projectStatus"] == "not_found"
    assert snapshot["projectRoot"] == str(
        tmp_path / "G2A0004BC01053" / "qr_pose_skill_conf" / "test10"
    )
    assert snapshot["imageCount"] == 0
    assert snapshot["images"] == []
    assert snapshot["maps"] == []


def test_project_store_scans_images_maps_and_active_map(tmp_path) -> None:
    project_root = tmp_path / "G2A0004BC01053" / "qr_pose_skill_conf" / "test10"
    images_dir = project_root / "images"
    maps_dir = project_root / "maps"
    images_dir.mkdir(parents=True)
    maps_dir.mkdir(parents=True)
    (images_dir / "1785315256907.jpg").write_bytes(b"jpg")
    (maps_dir / "test10.yml").write_text("markers: []\n", encoding="utf-8")
    (maps_dir / "test10.pcd").write_text("# pcd\n", encoding="utf-8")
    (maps_dir / "test10_stats.json").write_text(
        json.dumps(
            {
                "origin_marker_id": 5,
                "marker_ids": [1, 2],
                "n_markers": 2,
                "n_images": 3,
                "n_frames_used": 2,
                "reproj_rms_px_final": 1.25,
                "message": "ok",
            }
        ),
        encoding="utf-8",
    )
    (project_root / "manifest.json").write_text(
        json.dumps({"activeMapName": "test10", "cameraId": "hand_left_color"}),
        encoding="utf-8",
    )

    store = QrProjectStore(tmp_path)
    snapshot = store.get_project_snapshot(
        robot_serial="G2A0004BC01053",
        project_name="test10",
    )

    assert snapshot["exists"] is True
    assert snapshot["projectStatus"] == "mapped"
    assert snapshot["imageCount"] == 1
    assert snapshot["images"][0]["fileName"] == "1785315256907.jpg"
    assert snapshot["maps"][0]["mapName"] == "test10"
    assert snapshot["maps"][0]["active"] is True
    assert snapshot["maps"][0]["quality"]["nMarkers"] == 2
    assert snapshot["maps"][0]["quality"]["reprojRmsPxFinal"] == 1.25


def test_project_store_rejects_path_traversal_segments(tmp_path) -> None:
    store = QrProjectStore(tmp_path)

    with pytest.raises(QrProjectStoreError):
        store.get_project_path(robot_serial="G2A0004BC01053", project_name="../bad")

