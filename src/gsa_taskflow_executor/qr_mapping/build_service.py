"""二维码建图 SDK 与地图资源服务。

建图 SDK 运行在 executor 所在 Ubuntu 主机，输入输出都限定在 GSA_DATA_ROOT 下。
客户端只通过 MQTT 请求建图/删除/预览，不能传入任意本机或远端绝对路径。
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from gsa_taskflow_executor.gdk.control_probe import utc_now_iso
from gsa_taskflow_executor.qr_mapping.calibration_store import (
    build_sdk_calibration_json,
    intrinsic_file_name_from_camera_id,
)
from gsa_taskflow_executor.qr_mapping.capture_service import read_manifest
from gsa_taskflow_executor.qr_mapping.project_store import (
    IMAGE_EXTENSIONS,
    QrProjectPaths,
    QrProjectStore,
    map_record,
    paths_to_payload,
    read_json_object,
    validate_safe_segment,
)

ACTION_BUILD_QR_MAP = "build_qr_map"
ACTION_DELETE_QR_MAP = "delete_qr_map"
ACTION_READ_QR_PCD_PREVIEW = "read_qr_pcd_preview"
DEFAULT_BUILD_TIMEOUT_SECONDS = 300.0
MAP_STATS_SUFFIX = "_stats.json"
SDK_INTRINSIC_SUFFIX = "-sdk-intrinsic.json"
DEFAULT_PCD_MAX_POINTS = 10_000
PCD_MAX_POINTS_LIMIT = 50_000


class QrBuildService:
    """封装 executor 本机二维码建图产物操作。"""

    def __init__(
        self,
        *,
        project_store: QrProjectStore,
        sdk_path: str = "",
        sdk_python: str = "python3",
        build_timeout_seconds: float = DEFAULT_BUILD_TIMEOUT_SECONDS,
    ) -> None:
        self._project_store = project_store
        self._sdk_path = sdk_path.strip()
        self._sdk_python = sdk_python.strip() or "python3"
        self._build_timeout_seconds = build_timeout_seconds

    def build_map(
        self,
        *,
        robot_serial: str,
        project_name: str,
        map_name: str,
        camera_id: str,
        marker_type: str,
        marker_size_meters: float,
    ) -> dict[str, object]:
        try:
            paths = self._project_store.ensure_project_layout(
                robot_serial=robot_serial,
                project_name=project_name,
            )
            normalized_map_name = validate_safe_segment(map_name, "mapName")
            if marker_size_meters <= 0:
                raise ValueError("二维码尺寸必须大于 0")
            if not any_image(paths.images_dir):
                raise ValueError("当前项目还没有采集图片，请先完成采集")
            if map_artifact_exists(paths, normalized_map_name):
                raise ValueError("同名地图已存在，请更换建图名称")

            calibration_file = resolve_project_calibration_file(paths, camera_id)
            sdk_calibration_file = prepare_sdk_calibration_file(
                calibration_file,
                paths.maps_dir,
                normalized_map_name,
            )
            stats_path = paths.maps_dir / f"{normalized_map_name}{MAP_STATS_SUFFIX}"
            sdk_result = run_qr_mapping_sdk(
                sdk_python=self._sdk_python,
                sdk_path=resolve_sdk_path(self._sdk_path),
                images_dir=paths.images_dir,
                calibration_file=sdk_calibration_file,
                out_dir=paths.maps_dir,
                marker_size_meters=marker_size_meters,
                map_name=normalized_map_name,
                marker_type=marker_type,
                stats_path=stats_path,
                timeout_seconds=self._build_timeout_seconds,
            )
            record = map_record(paths.maps_dir / f"{normalized_map_name}.yml", active_map_name=normalized_map_name)
            update_manifest_after_build(
                paths,
                map_name=normalized_map_name,
                marker_type=marker_type,
                marker_size_meters=marker_size_meters,
                sdk_calibration_file=sdk_calibration_file,
                sdk_result=sdk_result,
            )
            return {
                "available": True,
                "backend": "executor.qr_mapping_sdk",
                "action": ACTION_BUILD_QR_MAP,
                "robotSerial": robot_serial.strip(),
                "projectName": project_name.strip(),
                **record,
                **paths_to_payload(paths),
                "sdk": sdk_result,
                "collectedAt": utc_now_iso(),
            }
        except Exception as error:
            return error_payload(ACTION_BUILD_QR_MAP, error)

    def delete_map(
        self,
        *,
        robot_serial: str,
        project_name: str,
        map_name: str,
    ) -> dict[str, object]:
        try:
            paths = self._project_store.build_paths(robot_serial=robot_serial, project_name=project_name)
            normalized_map_name = validate_safe_segment(map_name, "mapName")
            deleted_paths: list[str] = []
            for path in map_artifact_paths(paths, normalized_map_name):
                if path.exists() and path.is_file():
                    path.unlink()
                    deleted_paths.append(str(path))
            update_manifest_after_delete(paths, normalized_map_name)
            return {
                "available": True,
                "backend": "executor.filesystem",
                "action": ACTION_DELETE_QR_MAP,
                "robotSerial": robot_serial.strip(),
                "projectName": project_name.strip(),
                "mapName": normalized_map_name,
                "deletedPaths": deleted_paths,
                **paths_to_payload(paths),
                "collectedAt": utc_now_iso(),
            }
        except Exception as error:
            return error_payload(ACTION_DELETE_QR_MAP, error)

    def read_pcd_preview(
        self,
        *,
        robot_serial: str,
        project_name: str,
        map_name: str,
        max_points: int,
    ) -> dict[str, object]:
        try:
            paths = self._project_store.build_paths(robot_serial=robot_serial, project_name=project_name)
            normalized_map_name = validate_safe_segment(map_name, "mapName")
            pcd_path = paths.maps_dir / f"{normalized_map_name}.pcd"
            preview = parse_pcd_preview(
                pcd_path,
                map_name=normalized_map_name,
                max_points=normalize_pcd_max_points(max_points),
            )
            return {
                "available": True,
                "backend": "executor.filesystem",
                "action": ACTION_READ_QR_PCD_PREVIEW,
                "robotSerial": robot_serial.strip(),
                "projectName": project_name.strip(),
                **preview,
                "collectedAt": utc_now_iso(),
            }
        except Exception as error:
            return error_payload(ACTION_READ_QR_PCD_PREVIEW, error)


def any_image(images_dir: Path) -> bool:
    return images_dir.exists() and any(
        path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        for path in images_dir.iterdir()
    )


def map_artifact_exists(paths: QrProjectPaths, map_name: str) -> bool:
    return any(path.exists() for path in map_artifact_paths(paths, map_name))


def map_artifact_paths(paths: QrProjectPaths, map_name: str) -> list[Path]:
    return [
        paths.maps_dir / f"{map_name}.yml",
        paths.maps_dir / f"{map_name}-cam.yml",
        paths.maps_dir / f"{map_name}.pcd",
        paths.maps_dir / f"{map_name}.log",
        paths.maps_dir / f"{map_name}{MAP_STATS_SUFFIX}",
        paths.maps_dir / f"{map_name}{SDK_INTRINSIC_SUFFIX}",
    ]


def resolve_project_calibration_file(paths: QrProjectPaths, camera_id: str) -> Path:
    manifest = read_manifest(paths.manifest_path)
    calibration = manifest.get("calibration")
    if isinstance(calibration, Mapping):
        file_path = calibration.get("calibrationFilePath")
        if isinstance(file_path, str) and file_path.strip():
            path = Path(file_path)
            if path.exists():
                return path

    fallback = paths.sensor_root / intrinsic_file_name_from_camera_id(camera_id)
    if fallback.exists():
        return fallback
    raise FileNotFoundError(f"未找到相机内参文件: {fallback}")


def prepare_sdk_calibration_file(
    calibration_file: Path,
    maps_dir: Path,
    map_name: str,
) -> Path:
    if calibration_file.suffix.lower() in {".yml", ".yaml"}:
        return calibration_file
    if calibration_file.suffix.lower() != ".json":
        raise ValueError("建图 SDK 仅支持 json / yml / yaml 内参文件")
    sdk_calibration = build_sdk_calibration_json(calibration_file)
    sdk_calibration_file = maps_dir / f"{map_name}{SDK_INTRINSIC_SUFFIX}"
    sdk_calibration_file.write_text(
        json.dumps(sdk_calibration, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return sdk_calibration_file


def resolve_sdk_path(configured_sdk_path: str) -> Path | None:
    candidates = [
        configured_sdk_path,
        str(Path.cwd() / "sdk" / "qr_mapping_sdk"),
        str(Path.cwd().parent / "sdk" / "qr_mapping_sdk"),
        str(Path(__file__).resolve().parents[4] / "sdk" / "qr_mapping_sdk"),
        str(Path(__file__).resolve().parents[5] / "sdk" / "qr_mapping_sdk"),
    ]
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser()
        if (path / "qr_mapping" / "cli.py").exists():
            return path
    return None


def run_qr_mapping_sdk(
    *,
    sdk_python: str,
    sdk_path: Path | None,
    images_dir: Path,
    calibration_file: Path,
    out_dir: Path,
    marker_size_meters: float,
    map_name: str,
    marker_type: str,
    stats_path: Path,
    timeout_seconds: float,
) -> dict[str, object]:
    args = [
        sdk_python,
        "-m",
        "qr_mapping.cli",
        "--images",
        str(images_dir),
        "--calibration",
        str(calibration_file),
        "--out",
        str(out_dir),
        "--marker-len",
        str(marker_size_meters),
        "--name",
        map_name,
        "--dict",
        marker_type,
        "--json-result",
        str(stats_path),
    ]
    env = dict(os.environ)
    cwd = str(sdk_path) if sdk_path is not None else None
    if sdk_path is not None:
        env["PYTHONPATH"] = os.pathsep.join(
            part
            for part in (str(sdk_path), env.get("PYTHONPATH", ""))
            if part
        )
    try:
        completed = subprocess.run(
            args,
            cwd=cwd,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise TimeoutError(f"二维码建图 SDK 执行超时 ({timeout_seconds:.1f}s)") from error

    stdout = append_bounded("", completed.stdout or "")
    stderr = append_bounded("", completed.stderr or "")
    if completed.returncode != 0:
        details = "\n".join(item for item in (stderr.strip(), stdout.strip()) if item)
        raise RuntimeError(
            f"二维码建图 SDK 执行失败，退出码 {completed.returncode}"
            + (f": {details}" if details else "")
        )
    return {
        "python": sdk_python,
        "sdkPath": str(sdk_path) if sdk_path is not None else None,
        "returnCode": completed.returncode,
        "stdout": stdout,
        "stderr": stderr,
    }


def append_bounded(current: str, next_text: str) -> str:
    combined = current + next_text
    return combined if len(combined) <= 12_000 else combined[-12_000:]


def update_manifest_after_build(
    paths: QrProjectPaths,
    *,
    map_name: str,
    marker_type: str,
    marker_size_meters: float,
    sdk_calibration_file: Path,
    sdk_result: Mapping[str, object],
) -> None:
    manifest = read_manifest(paths.manifest_path)
    builds = manifest.get("builds")
    build_items = list(builds) if isinstance(builds, list) else []
    build_items.append(
        {
            "mapName": map_name,
            "markerType": marker_type,
            "markerSizeMeters": marker_size_meters,
            "sdkCalibrationFilePath": str(sdk_calibration_file),
            "statsPath": str(paths.maps_dir / f"{map_name}{MAP_STATS_SUFFIX}"),
            "builtAt": utc_now_iso(),
            "sdk": sdk_result,
        }
    )
    manifest.update(
        {
            "projectStatus": "mapped",
            "activeMapName": map_name,
            "builds": build_items,
            "updatedAt": utc_now_iso(),
        }
    )
    paths.manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def update_manifest_after_delete(paths: QrProjectPaths, deleted_map_name: str) -> None:
    manifest = read_manifest(paths.manifest_path)
    if manifest.get("activeMapName") == deleted_map_name:
        manifest.pop("activeMapName", None)
    manifest["updatedAt"] = utc_now_iso()
    maps_remaining = [
        path
        for path in paths.maps_dir.glob("*.yml")
        if path.is_file() and not path.stem.endswith("-cam")
    ]
    manifest["projectStatus"] = "mapped" if maps_remaining else ("captured" if any_image(paths.images_dir) else "empty")
    paths.manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def parse_pcd_preview(
    path: Path,
    *,
    map_name: str,
    max_points: int,
) -> dict[str, object]:
    data = path.read_bytes()
    header = parse_pcd_header(data)
    if header["dataFormat"] == "binary_compressed":
        raise ValueError("暂不支持 binary_compressed PCD，请先转换为 ascii 或未压缩 binary PCD")
    points, parsed_count = (
        parse_ascii_pcd_points(data, header, max_points)
        if header["dataFormat"] == "ascii"
        else parse_binary_pcd_points(data, header, max_points)
    )
    point_count = header.get("declaredPointCount") or parsed_count
    return {
        "mapName": map_name,
        "filePath": str(path),
        "format": header["dataFormat"],
        "fields": header["fields"],
        "pointCount": point_count,
        "points": points,
        "truncated": len(points) < int(point_count),
    }


def parse_pcd_header(data: bytes) -> dict[str, Any]:
    lines: list[str] = []
    offset = 0
    data_offset = -1
    while offset < len(data):
        newline_index = data.find(b"\n", offset)
        if newline_index < 0:
            break
        line = data[offset:newline_index].decode("ascii", errors="replace").rstrip("\r")
        lines.append(line)
        offset = newline_index + 1
        if line.strip().split(maxsplit=1)[0].lower() == "data":
            data_offset = offset
            break
    if data_offset < 0:
        raise ValueError("PCD 文件缺少 DATA 行")

    data_words = read_header_words(lines, "DATA")
    data_format = data_words[0].lower() if data_words else ""
    if data_format not in {"ascii", "binary", "binary_compressed"}:
        raise ValueError(f"不支持的 PCD DATA 格式: {data_format or '未知'}")
    fields = [item.lower() for item in read_header_words(lines, "FIELDS")]
    if not fields:
        raise ValueError("PCD 文件缺少 FIELDS 行")
    sizes = read_header_numbers(lines, "SIZE", [4] * len(fields))
    types = [item.upper() for item in read_header_words(lines, "TYPE", ["F"] * len(fields))]
    counts = read_header_numbers(lines, "COUNT", [1] * len(fields))
    if not (len(sizes) == len(types) == len(counts) == len(fields)):
        raise ValueError("PCD header 的 FIELDS/SIZE/TYPE/COUNT 数量不一致")

    byte_offset = 0
    token_index = 0
    required: dict[str, dict[str, object]] = {}
    for index, field in enumerate(fields):
        size = int(sizes[index])
        count = int(counts[index])
        if size < 1 or count < 1:
            raise ValueError("PCD header 中 SIZE/COUNT 必须大于 0")
        if field in {"x", "y", "z"}:
            required[field] = {
                "byteOffset": byte_offset,
                "tokenIndex": token_index,
                "size": size,
                "type": types[index],
            }
        byte_offset += size * count
        token_index += count
    if set(required) != {"x", "y", "z"}:
        raise ValueError("PCD 文件缺少 x/y/z 坐标字段")

    return {
        "lines": lines,
        "dataOffset": data_offset,
        "dataFormat": data_format,
        "fields": fields,
        "declaredPointCount": read_declared_point_count(lines),
        "pointStep": byte_offset,
        "requiredFields": required,
    }


def read_header_words(lines: list[str], key: str, fallback: list[str] | None = None) -> list[str]:
    lower_key = key.lower()
    for line in lines:
        words = line.strip().split()
        if words and words[0].lower() == lower_key:
            return words[1:]
    return fallback or []


def read_header_numbers(lines: list[str], key: str, fallback: list[int]) -> list[int]:
    words = read_header_words(lines, key)
    if not words:
        return fallback
    return [int(float(word)) for word in words]


def read_declared_point_count(lines: list[str]) -> int | None:
    words = read_header_words(lines, "POINTS")
    if words:
        return int(float(words[0]))
    width_words = read_header_words(lines, "WIDTH")
    height_words = read_header_words(lines, "HEIGHT")
    if width_words and height_words:
        return int(float(width_words[0])) * int(float(height_words[0]))
    return None


def parse_ascii_pcd_points(
    data: bytes,
    header: Mapping[str, Any],
    max_points: int,
) -> tuple[list[dict[str, float]], int]:
    text = data[int(header["dataOffset"]) :].decode("utf-8", errors="replace")
    lines = [line for line in text.splitlines() if line.strip()]
    estimated_count = int(header.get("declaredPointCount") or len(lines))
    sample_step = max(1, (estimated_count + max_points - 1) // max_points)
    fields = header["requiredFields"]
    points: list[dict[str, float]] = []
    valid_count = 0
    for line in lines:
        columns = line.strip().split()
        try:
            point = {
                "x": float(columns[int(fields["x"]["tokenIndex"])]),
                "y": float(columns[int(fields["y"]["tokenIndex"])]),
                "z": float(columns[int(fields["z"]["tokenIndex"])]),
            }
        except (IndexError, ValueError):
            continue
        if valid_count % sample_step == 0 and len(points) < max_points:
            points.append(point)
        valid_count += 1
    return points, valid_count


def parse_binary_pcd_points(
    data: bytes,
    header: Mapping[str, Any],
    max_points: int,
) -> tuple[list[dict[str, float]], int]:
    point_step = int(header["pointStep"])
    data_offset = int(header["dataOffset"])
    available = max(0, (len(data) - data_offset) // point_step)
    point_count = min(int(header.get("declaredPointCount") or available), available)
    sample_step = max(1, (point_count + max_points - 1) // max_points)
    fields = header["requiredFields"]
    points: list[dict[str, float]] = []
    for index in range(0, point_count, sample_step):
        point_offset = data_offset + index * point_step
        point = {
            "x": read_binary_scalar(data, point_offset, fields["x"]),
            "y": read_binary_scalar(data, point_offset, fields["y"]),
            "z": read_binary_scalar(data, point_offset, fields["z"]),
        }
        if all(isinstance(value, float) for value in point.values()):
            points.append(point)
        if len(points) >= max_points:
            break
    return points, point_count


def read_binary_scalar(data: bytes, point_offset: int, field: Mapping[str, object]) -> float:
    offset = point_offset + int(field["byteOffset"])
    size = int(field["size"])
    field_type = str(field["type"])
    if offset + size > len(data):
        return float("nan")
    if field_type == "F":
        if size == 4:
            return float(struct.unpack_from("<f", data, offset)[0])
        if size == 8:
            return float(struct.unpack_from("<d", data, offset)[0])
    if field_type == "I":
        formats = {1: "<b", 2: "<h", 4: "<i"}
    elif field_type == "U":
        formats = {1: "<B", 2: "<H", 4: "<I"}
    else:
        formats = {}
    if size in formats:
        return float(struct.unpack_from(formats[size], data, offset)[0])
    raise ValueError(f"暂不支持 PCD binary 字段类型 {field_type}{size}")


def normalize_pcd_max_points(value: int) -> int:
    return max(1, min(int(value), PCD_MAX_POINTS_LIMIT))


def error_payload(action: str, error: Exception) -> dict[str, object]:
    return {
        "available": False,
        "backend": "executor.qr_mapping",
        "action": action,
        "errorCode": error_code_from_exception(error),
        "errorType": type(error).__name__,
        "errorMsg": str(error),
        "collectedAt": utc_now_iso(),
    }


def error_code_from_exception(error: Exception) -> str:
    if isinstance(error, FileNotFoundError):
        return "QR_RESOURCE_NOT_FOUND"
    if isinstance(error, TimeoutError):
        return "QR_MAPPING_SDK_TIMEOUT"
    return "QR_MAPPING_FAILED"
