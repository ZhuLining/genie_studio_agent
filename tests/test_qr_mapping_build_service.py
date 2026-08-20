import json

from gsa_taskflow_executor.qr_mapping.build_service import QrBuildService
from gsa_taskflow_executor.qr_mapping.project_store import QrProjectStore


def test_qr_build_service_rejects_existing_map_name(tmp_path) -> None:
    project_root = tmp_path / "G2A0004BC01053" / "qr_pose_skill_conf" / "test10"
    (project_root / "images").mkdir(parents=True)
    (project_root / "maps").mkdir()
    (project_root / "images" / "1785315256907.jpg").write_bytes(b"fake-jpg")
    (project_root / "maps" / "test10.yml").write_text("markers: []\n", encoding="utf-8")

    service = QrBuildService(project_store=QrProjectStore(tmp_path))
    result = service.build_map(
        robot_serial="G2A0004BC01053",
        project_name="test10",
        map_name="test10",
        camera_id="hand_left_color",
        marker_type="ARUCO_MIP_36h12",
        marker_size_meters=0.04,
    )

    assert result["available"] is False
    assert result["errorCode"] == "QR_MAPPING_FAILED"
    assert "同名地图已存在" in result["errorMsg"]


def test_qr_build_service_reads_ascii_pcd_preview(tmp_path) -> None:
    project_root = tmp_path / "G2A0004BC01053" / "qr_pose_skill_conf" / "test10"
    maps_dir = project_root / "maps"
    maps_dir.mkdir(parents=True)
    (maps_dir / "test10.pcd").write_text(
        "\n".join(
            [
                "# .PCD v0.7",
                "VERSION 0.7",
                "FIELDS x y z rgb",
                "SIZE 4 4 4 4",
                "TYPE F F F F",
                "COUNT 1 1 1 1",
                "WIDTH 3",
                "HEIGHT 1",
                "POINTS 3",
                "DATA ascii",
                "0.0 0.0 0.0 0",
                "1.0 0.0 0.0 0",
                "2.0 0.0 0.0 0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    service = QrBuildService(project_store=QrProjectStore(tmp_path))
    result = service.read_pcd_preview(
        robot_serial="G2A0004BC01053",
        project_name="test10",
        map_name="test10",
        max_points=2,
    )

    assert result["available"] is True
    assert result["action"] == "read_qr_pcd_preview"
    assert result["pointCount"] == 3
    assert result["truncated"] is True
    assert result["points"] == [
        {"x": 0.0, "y": 0.0, "z": 0.0},
        {"x": 2.0, "y": 0.0, "z": 0.0},
    ]


def test_qr_build_service_deletes_map_artifacts_and_updates_manifest(tmp_path) -> None:
    project_root = tmp_path / "G2A0004BC01053" / "qr_pose_skill_conf" / "test10"
    images_dir = project_root / "images"
    maps_dir = project_root / "maps"
    images_dir.mkdir(parents=True)
    maps_dir.mkdir()
    (images_dir / "1785315256907.jpg").write_bytes(b"fake-jpg")
    (maps_dir / "test10.yml").write_text("markers: []\n", encoding="utf-8")
    (maps_dir / "test10.pcd").write_text("# pcd\n", encoding="utf-8")
    (project_root / "manifest.json").write_text(
        json.dumps({"activeMapName": "test10", "projectStatus": "mapped"}),
        encoding="utf-8",
    )

    service = QrBuildService(project_store=QrProjectStore(tmp_path))
    result = service.delete_map(
        robot_serial="G2A0004BC01053",
        project_name="test10",
        map_name="test10",
    )

    manifest = json.loads((project_root / "manifest.json").read_text(encoding="utf-8"))
    assert result["available"] is True
    assert sorted(path.split("/")[-1] for path in result["deletedPaths"]) == [
        "test10.pcd",
        "test10.yml",
    ]
    assert "activeMapName" not in manifest
    assert manifest["projectStatus"] == "captured"
