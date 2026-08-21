"""二维码建图项目路径和快照索引。

方案 2 中客户端不能浏览 Ubuntu 文件系统，也不能传入绝对路径。这里把
`robotSerial/projectName` 收敛为 executor 本机 `GSA_DATA_ROOT` 下的安全路径，
并只返回项目状态、图片/地图元数据等轻量信息。
"""

from __future__ import annotations

import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ACTION_GET_QR_PROJECT_PATH = "get_qr_project_path"
ACTION_GET_QR_PROJECT_SNAPSHOT = "get_qr_project_snapshot"
ACTION_LIST_QR_PROJECTS = "list_qr_projects"

DEFAULT_IMAGE_LIMIT = 50
MAX_IMAGE_LIMIT = 200
SAFE_SEGMENT_PATTERN = re.compile(r"^[A-Za-z0-9_\-\u4e00-\u9fff]{1,64}$")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}


class QrProjectStoreError(ValueError):
    """二维码建图资源路径或文件索引错误。"""


@dataclass(frozen=True)
class QrProjectPaths:
    data_root: Path
    robot_root: Path
    sensor_root: Path
    project_root: Path
    images_dir: Path
    maps_dir: Path
    point_dir: Path
    waypoints_dir: Path
    captures_file: Path
    manifest_path: Path


class QrProjectStore:
    """读取 executor 本机二维码建图项目索引。"""

    def __init__(self, data_root: str | Path) -> None:
        self._data_root = Path(data_root).expanduser()

    def build_paths(self, *, robot_serial: str, project_name: str) -> QrProjectPaths:
        """计算项目安全路径，供采集、建图、点位录制等写入侧复用。"""

        return self._build_paths(robot_serial=robot_serial, project_name=project_name)

    def ensure_project_layout(self, *, robot_serial: str, project_name: str) -> QrProjectPaths:
        """创建原版兼容目录结构。所有路径仍受 GSA_DATA_ROOT 防逃逸校验保护。"""

        paths = self._build_paths(robot_serial=robot_serial, project_name=project_name)
        for directory in (
            paths.sensor_root,
            paths.images_dir,
            paths.maps_dir,
            paths.point_dir,
            paths.waypoints_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)
        return paths

    def has_project_data(self, *, robot_serial: str, project_name: str) -> bool:
        """判断项目是否已有会被新采集覆盖语义影响的数据。"""

        paths = self._build_paths(robot_serial=robot_serial, project_name=project_name)
        return has_project_data(paths)

    def get_project_path(self, *, robot_serial: str, project_name: str) -> dict[str, object]:
        paths = self._build_paths(robot_serial=robot_serial, project_name=project_name)
        return {
            "available": True,
            "backend": "executor.filesystem",
            "action": ACTION_GET_QR_PROJECT_PATH,
            "robotSerial": robot_serial.strip(),
            "projectName": project_name.strip(),
            "exists": paths.project_root.exists(),
            "projectStatus": read_project_status(paths),
            **paths_to_payload(paths),
            "collectedAt": utc_now_iso(),
        }

    def list_projects(self, *, robot_serial: str) -> dict[str, object]:
        """列出指定机器人 SN 下的二维码项目。只返回轻量索引，供应用节点下拉使用。"""

        robot_segment = validate_safe_segment(robot_serial, "robotSerial")
        data_root = self._data_root
        robot_root = data_root / robot_segment
        project_parent = robot_root / "qr_pose_skill_conf"
        ensure_project_parent_under_data_root(data_root, robot_root, project_parent)
        projects = list_project_records(self, robot_segment, project_parent)
        return {
            "available": True,
            "backend": "executor.filesystem",
            "action": ACTION_LIST_QR_PROJECTS,
            "robotSerial": robot_segment,
            "dataRoot": str(data_root),
            "robotRoot": str(robot_root),
            "projectParent": str(project_parent),
            "projects": projects,
            "projectCount": len(projects),
            "collectedAt": utc_now_iso(),
        }

    def get_project_snapshot(
        self,
        *,
        robot_serial: str,
        project_name: str,
        image_limit: int = DEFAULT_IMAGE_LIMIT,
    ) -> dict[str, object]:
        paths = self._build_paths(robot_serial=robot_serial, project_name=project_name)
        manifest = read_json_object(paths.manifest_path)
        images = list_image_records(paths.images_dir, limit=image_limit)
        maps = list_map_records(paths.maps_dir, manifest=manifest)
        target_points = list_target_point_records(paths)
        initial_photo_points = list_initial_photo_records(paths, manifest=manifest)
        for item in maps:
            item.setdefault("robotSerial", robot_serial.strip())
            item.setdefault("projectName", project_name.strip())
        active_map_name = read_optional_string(manifest, "activeMapName")
        if not active_map_name and maps:
            active_map_name = maps[0]["mapName"] if isinstance(maps[0].get("mapName"), str) else None
        project_status = infer_project_status(paths, images=images, maps=maps, manifest=manifest)

        return {
            "available": True,
            "backend": "executor.filesystem",
            "action": ACTION_GET_QR_PROJECT_SNAPSHOT,
            "robotSerial": robot_serial.strip(),
            "projectName": project_name.strip(),
            "exists": paths.project_root.exists(),
            "projectStatus": project_status,
            **paths_to_payload(paths),
            "imageCount": count_images(paths.images_dir),
            "images": images,
            "maps": maps,
            "targetPoints": target_points,
            "initialPhotoPoints": initial_photo_points,
            "activeMapName": active_map_name,
            "manifest": manifest,
            "warnings": build_snapshot_warnings(paths, manifest=manifest, maps=maps),
            "collectedAt": utc_now_iso(),
        }

    def _build_paths(self, *, robot_serial: str, project_name: str) -> QrProjectPaths:
        robot_segment = validate_safe_segment(robot_serial, "robotSerial")
        project_segment = validate_safe_segment(project_name, "projectName")
        data_root = self._data_root
        robot_root = data_root / robot_segment
        project_root = robot_root / "qr_pose_skill_conf" / project_segment
        paths = QrProjectPaths(
            data_root=data_root,
            robot_root=robot_root,
            sensor_root=robot_root / "sensor",
            project_root=project_root,
            images_dir=project_root / "images",
            maps_dir=project_root / "maps",
            point_dir=project_root / "point",
            waypoints_dir=project_root / "waypoints",
            captures_file=project_root / "captures.jsonl",
            manifest_path=project_root / "manifest.json",
        )
        ensure_under_data_root(paths)
        return paths


def ensure_project_parent_under_data_root(
    data_root: Path,
    robot_root: Path,
    project_parent: Path,
) -> None:
    root = data_root.resolve(strict=False)
    for path in (robot_root, project_parent):
        try:
            path.resolve(strict=False).relative_to(root)
        except ValueError as error:
            raise QrProjectStoreError("二维码项目列表路径越过 GSA_DATA_ROOT，已拒绝") from error


def list_project_records(
    store: QrProjectStore,
    robot_serial: str,
    project_parent: Path,
) -> list[dict[str, object]]:
    if not project_parent.exists() or not project_parent.is_dir():
        return []
    records: list[dict[str, object]] = []
    for project_dir in sorted(project_parent.iterdir(), key=lambda item: item.name):
        if not project_dir.is_dir():
            continue
        project_name = project_dir.name
        try:
            paths = store.build_paths(robot_serial=robot_serial, project_name=project_name)
        except QrProjectStoreError:
            continue
        manifest = read_json_object(paths.manifest_path)
        maps = list_map_records(paths.maps_dir, manifest=manifest)
        targets = list_target_point_records(paths)
        initial_photos = list_initial_photo_records(paths, manifest=manifest)
        records.append(
            {
                "projectName": project_name,
                "projectRoot": str(paths.project_root),
                "projectStatus": infer_project_status(
                    paths,
                    images=[],
                    maps=maps,
                    manifest=manifest,
                ),
                "mapCount": len(maps),
                "targetPointCount": len(targets),
                "initialPhotoPointCount": len(initial_photos),
                "activeMapName": read_optional_string(manifest, "activeMapName"),
                "updatedAt": file_mtime_iso(project_dir),
            }
        )
    return records


def validate_safe_segment(value: str, field_name: str) -> str:
    segment = value.strip()
    if not SAFE_SEGMENT_PATTERN.fullmatch(segment):
        raise QrProjectStoreError(
            f"{field_name} 只能包含中文、英文、数字、下划线和中划线，长度 1~64"
        )
    return segment


def ensure_under_data_root(paths: QrProjectPaths) -> None:
    """防止路径穿越。这里不要求目录已存在，只校验解析后的路径前缀。"""

    data_root = paths.data_root.resolve(strict=False)
    for path in (
        paths.robot_root,
        paths.sensor_root,
        paths.project_root,
        paths.images_dir,
        paths.maps_dir,
        paths.point_dir,
        paths.waypoints_dir,
        paths.captures_file,
        paths.manifest_path,
    ):
        try:
            path.resolve(strict=False).relative_to(data_root)
        except ValueError as error:
            raise QrProjectStoreError("二维码建图路径越过 GSA_DATA_ROOT，已拒绝") from error


def paths_to_payload(paths: QrProjectPaths) -> dict[str, str]:
    return {
        "dataRoot": str(paths.data_root),
        "robotRoot": str(paths.robot_root),
        "sensorRoot": str(paths.sensor_root),
        "projectRoot": str(paths.project_root),
        "imagesDir": str(paths.images_dir),
        "mapsDir": str(paths.maps_dir),
        "pointDir": str(paths.point_dir),
        "waypointsDir": str(paths.waypoints_dir),
        "capturesFile": str(paths.captures_file),
        "manifestPath": str(paths.manifest_path),
    }


def infer_project_status(
    paths: QrProjectPaths,
    *,
    images: list[dict[str, object]],
    maps: list[dict[str, object]],
    manifest: Mapping[str, Any],
) -> str:
    status = read_optional_string(manifest, "projectStatus")
    if status:
        return status
    if not paths.project_root.exists():
        return "not_found"
    if has_waypoints(paths) or paths.point_dir.exists() and any(paths.point_dir.iterdir()):
        return "recorded"
    if maps:
        return "mapped"
    if images or paths.images_dir.exists() and any_image_file(paths.images_dir):
        return "captured"
    return "empty"


def read_project_status(paths: QrProjectPaths) -> str:
    if not paths.project_root.exists():
        return "not_found"
    manifest = read_json_object(paths.manifest_path)
    return infer_project_status(
        paths,
        images=[],
        maps=list_map_records(paths.maps_dir, manifest=manifest),
        manifest=manifest,
    )


def list_image_records(images_dir: Path, *, limit: int) -> list[dict[str, object]]:
    if not images_dir.exists() or not images_dir.is_dir():
        return []
    normalized_limit = max(1, min(int(limit), MAX_IMAGE_LIMIT))
    files = sorted(
        (path for path in images_dir.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS),
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )
    return [image_record(path) for path in files[:normalized_limit]]


def image_record(path: Path) -> dict[str, object]:
    timestamp_ms = parse_timestamp_ms(path.stem)
    stat = path.stat()
    return {
        "fileName": path.name,
        "remotePath": str(path),
        "timestampMs": timestamp_ms,
        "sizeBytes": stat.st_size,
        "updatedAt": datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat(),
    }


def count_images(images_dir: Path) -> int:
    if not images_dir.exists() or not images_dir.is_dir():
        return 0
    return sum(1 for path in images_dir.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS)


def any_image_file(images_dir: Path) -> bool:
    return any(path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS for path in images_dir.iterdir())


def has_project_data(paths: QrProjectPaths) -> bool:
    """已有图片、地图、点位或 manifest 时默认视为不可覆盖。"""

    if paths.images_dir.exists() and any_image_file(paths.images_dir):
        return True
    if paths.maps_dir.exists() and any(paths.maps_dir.iterdir()):
        return True
    if paths.point_dir.exists() and any(paths.point_dir.iterdir()):
        return True
    if paths.waypoints_dir.exists() and any(paths.waypoints_dir.iterdir()):
        return True
    if paths.captures_file.exists() and paths.captures_file.stat().st_size > 0:
        return True
    return paths.manifest_path.exists()


def list_map_records(
    maps_dir: Path,
    *,
    manifest: Mapping[str, Any],
) -> list[dict[str, object]]:
    if not maps_dir.exists() or not maps_dir.is_dir():
        return []
    active_map_name = read_optional_string(manifest, "activeMapName")
    records = [
        map_record(path, active_map_name=active_map_name)
        for path in maps_dir.iterdir()
        if path.is_file() and path.suffix == ".yml" and not path.stem.endswith("-cam")
    ]
    records.sort(key=lambda item: str(item.get("updatedAt") or ""), reverse=True)
    if not active_map_name and records:
        records[0]["active"] = True
    return records


def map_record(map_yml_path: Path, *, active_map_name: str | None) -> dict[str, object]:
    map_name = map_yml_path.stem
    maps_dir = map_yml_path.parent
    pcd_path = maps_dir / f"{map_name}.pcd"
    cam_yml_path = maps_dir / f"{map_name}-cam.yml"
    log_path = maps_dir / f"{map_name}.log"
    stats_path = maps_dir / f"{map_name}_stats.json"
    stat = map_yml_path.stat()
    quality = read_map_quality(stats_path)
    return {
        "mapName": map_name,
        "mapYmlPath": str(map_yml_path),
        "camYmlPath": str(cam_yml_path) if cam_yml_path.exists() else None,
        "pcdPath": str(pcd_path) if pcd_path.exists() else None,
        "logPath": str(log_path) if log_path.exists() else None,
        "statsPath": str(stats_path) if stats_path.exists() else None,
        "status": "success",
        "active": active_map_name == map_name,
        "updatedAt": datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat(),
        "quality": quality,
    }


def read_map_quality(stats_path: Path) -> dict[str, object] | None:
    stats = read_json_object(stats_path)
    if not stats:
        return None
    return {
        "originMarkerId": stats.get("origin_marker_id"),
        "markerIds": stats.get("marker_ids"),
        "nMarkers": stats.get("n_markers"),
        "nImages": stats.get("n_images"),
        "nFramesUsed": stats.get("n_frames_used"),
        "nObservations": stats.get("n_observations"),
        "reprojRmsPxInitial": stats.get("reproj_rms_px_initial"),
        "reprojRmsPxFinal": stats.get("reproj_rms_px_final"),
        "message": stats.get("message"),
    }


def list_target_point_records(paths: QrProjectPaths) -> list[dict[str, object]]:
    """列出点位录制目标点。只读索引函数，供桌面端刷新远端项目快照使用。"""

    file_path = paths.point_dir / "grasp_points.json"
    points = read_json_list(file_path)
    records: list[dict[str, object]] = []
    for item in points:
        point_name = item.get("grasp_name")
        if not isinstance(point_name, str) or not point_name.strip():
            continue
        abs_joints = item.get("abs_joints")
        records.append(
            {
                "pointKind": "target",
                "pointName": point_name.strip(),
                "arm": item.get("arm"),
                "cameraId": item.get("camera_id"),
                "mapName": item.get("map_name"),
                "savedAt": item.get("recorded_at") or file_mtime_iso(file_path),
                "pointFilePath": str(file_path),
                "absJointsCount": len(abs_joints) if isinstance(abs_joints, list) else 0,
            }
        )
    return records


def list_initial_photo_records(
    paths: QrProjectPaths,
    *,
    manifest: Mapping[str, Any],
) -> list[dict[str, object]]:
    """列出 qr_localize SDK 已生成的初始拍照点 waypoint 目录。"""

    if not paths.waypoints_dir.exists() or not paths.waypoints_dir.is_dir():
        return []
    point_recording = manifest.get("pointRecording")
    metadata_by_name: dict[str, Mapping[str, Any]] = {}
    if isinstance(point_recording, Mapping):
        raw_initial = point_recording.get("initialPhotoPoints")
        if isinstance(raw_initial, list):
            for item in raw_initial:
                if isinstance(item, Mapping) and isinstance(item.get("pointName"), str):
                    metadata_by_name[str(item["pointName"])] = item

    records: list[dict[str, object]] = []
    for waypoint_dir in paths.waypoints_dir.iterdir():
        if not waypoint_dir.is_dir():
            continue
        point_name = waypoint_dir.name
        stats_path = waypoint_dir / "locate_stats.json"
        stats = read_json_object(stats_path)
        metadata = metadata_by_name.get(point_name, {})
        records.append(
            {
                "pointKind": "initial_photo",
                "pointName": point_name,
                "arm": metadata.get("arm"),
                "cameraId": metadata.get("cameraId"),
                "mapName": metadata.get("mapName"),
                "savedAt": metadata.get("savedAt") or file_mtime_iso(waypoint_dir),
                "waypointDir": str(waypoint_dir),
                "tfBaselinkTagPath": str(waypoint_dir / "tf_baselink_tag.json")
                if (waypoint_dir / "tf_baselink_tag.json").exists()
                else None,
                "tfTagEePath": str(waypoint_dir / "tf_tag_ee.json")
                if (waypoint_dir / "tf_tag_ee.json").exists()
                else None,
                "jointsPath": str(waypoint_dir / "joints.json")
                if (waypoint_dir / "joints.json").exists()
                else None,
                "statsPath": str(stats_path) if stats_path.exists() else None,
                "quality": build_localize_quality(stats),
            }
        )
    records.sort(key=lambda item: str(item.get("savedAt") or ""), reverse=True)
    return records


def read_json_list(path: Path) -> list[Mapping[str, Any]]:
    if not path.exists() or not path.is_file():
        return []
    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    if not isinstance(decoded, list):
        return []
    return [item for item in decoded if isinstance(item, Mapping)]


def build_localize_quality(stats: Mapping[str, Any]) -> dict[str, object] | None:
    if not stats:
        return None
    return {
        "ok": stats.get("ok"),
        "nMarkers": stats.get("n_markers"),
        "usedIds": stats.get("used_ids"),
        "reprojPx": stats.get("reproj_px"),
        "dictName": stats.get("dict_name"),
        "message": stats.get("message"),
    }


def file_mtime_iso(path: Path) -> str:
    try:
        stat_target = path if path.is_file() else next(path.iterdir())
        return datetime.fromtimestamp(stat_target.stat().st_mtime, timezone.utc).isoformat()
    except Exception:
        return utc_now_iso()


def build_snapshot_warnings(
    paths: QrProjectPaths,
    *,
    manifest: Mapping[str, Any],
    maps: list[dict[str, object]],
) -> list[dict[str, str]]:
    warnings: list[dict[str, str]] = []
    if paths.project_root.exists() and not paths.manifest_path.exists():
        warnings.append({"stage": "manifest", "message": "项目目录存在但缺少 manifest.json"})
    if maps and not (read_optional_string(manifest, "activeMapName") or any(item.get("active") for item in maps)):
        warnings.append({"stage": "maps", "message": "存在地图文件但未设置 activeMapName"})
    return warnings


def has_waypoints(paths: QrProjectPaths) -> bool:
    if not paths.waypoints_dir.exists() or not paths.waypoints_dir.is_dir():
        return False
    return any(path.is_dir() for path in paths.waypoints_dir.iterdir())


def read_json_object(path: Path) -> Mapping[str, Any]:
    if not path.exists() or not path.is_file():
        return {}
    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return decoded if isinstance(decoded, Mapping) else {}


def read_optional_string(value: Mapping[str, Any], key: str) -> str | None:
    raw = value.get(key)
    if isinstance(raw, str) and raw.strip():
        return raw.strip()
    return None


def parse_timestamp_ms(value: str) -> int | None:
    if value.isdigit():
        try:
            return int(value[:13]) if len(value) > 13 else int(value)
        except ValueError:
            return None
    return None


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()
