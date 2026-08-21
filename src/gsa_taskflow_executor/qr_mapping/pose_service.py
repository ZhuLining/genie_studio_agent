"""应用工作流二维码定位服务。

二维码建图和点位录制产物都保存在 executor 所在 Ubuntu 主机。应用运行时
`qr_pose_skill` 只做只读定位：重新拍照计算当前二维码/tag 在 base 下的位姿，
再乘以点位录制阶段保存的 `T_tag_ee`，输出目标末端 pose/action_data。
这里不调用任何运动控制，避免把“定位结果”误用成“自动执行运动”。
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any
from uuid import uuid4

from gsa_taskflow_executor.gdk.readonly import to_jsonable
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.qr_mapping.point_recording_service import (
    build_localize_quality,
    cleanup_temp_image,
    collect_point_recording_gdk_snapshot,
    normalize_min_markers,
    resolve_qr_localize_sdk_path,
    resolve_recording_resources,
    run_qr_localize_sdk,
    validate_arm_camera,
    validate_timeout_ms,
)
from gsa_taskflow_executor.qr_mapping.project_store import (
    QrProjectStore,
    paths_to_payload,
    read_json_object,
    validate_safe_segment,
)
from gsa_taskflow_executor.taskflow.models import QrPoseParams

ACTION_QR_POSE = "qr_pose"
DEFAULT_QR_POSE_LOCALIZE_TIMEOUT_SECONDS = 120.0


class QrPoseService:
    """封装二维码定位节点的远端资源读取、GDK 采样和 SDK 调用。"""

    def __init__(
        self,
        *,
        project_store: QrProjectStore,
        session_manager: GdkSessionManager,
        localize_sdk_path: str = "",
        localize_sdk_python: str = "python3",
        localize_timeout_seconds: float = DEFAULT_QR_POSE_LOCALIZE_TIMEOUT_SECONDS,
    ) -> None:
        self._project_store = project_store
        self._session_manager = session_manager
        self._localize_sdk_path = localize_sdk_path.strip()
        self._localize_sdk_python = localize_sdk_python.strip() or "python3"
        self._localize_timeout_seconds = localize_timeout_seconds

    def locate(self, params: QrPoseParams) -> dict[str, object]:
        """执行一次二维码定位，返回可写入 taskflow 变量空间的 outputs。"""

        runtime_root: Path | None = None
        snapshot: Mapping[str, object] | None = None
        try:
            paths = self._project_store.build_paths(
                robot_serial=params.robot_serial,
                project_name=params.project_name,
            )
            validate_arm_camera(params.arm, params.camera_id)
            validate_timeout_ms(int(params.timeout * 1000))
            initial_photo_point = validate_safe_segment(
                params.initial_photo_point_name,
                "initialPhotoPointName",
            )
            resolved = resolve_recording_resources(paths, params.map_name, params.camera_id)
            waypoint_dir = paths.waypoints_dir / initial_photo_point
            tf_tag_ee_path = waypoint_dir / "tf_tag_ee.json"
            if not tf_tag_ee_path.exists():
                raise FileNotFoundError(f"未找到初始拍照点位 tf_tag_ee.json: {tf_tag_ee_path}")
            tag_ee_by_target = read_tag_ee_targets(tf_tag_ee_path)
            if not tag_ee_by_target:
                raise FileNotFoundError("初始拍照点位没有可用目标点位，请先保存目标点位")

            runtime_root = paths.project_root / ".runtime" / "qr_pose" / uuid4().hex
            runtime_root.mkdir(parents=True, exist_ok=True)
            snapshot = collect_point_recording_gdk_snapshot(
                action=ACTION_QR_POSE,
                arm=params.arm,
                camera_id=params.camera_id,
                timeout_ms=int(params.timeout * 1000),
                include_image=True,
                temp_dir=runtime_root,
                session_manager=self._session_manager,
            )
            if snapshot.get("available") is not True:
                cleanup_temp_image(snapshot)
                return dict(snapshot)

            runtime_point_dir = materialize_qr_pose_scan_files(
                runtime_root=runtime_root,
                snapshot=snapshot,
            )
            stats_path = runtime_root / "locate_stats.json"
            current_waypoint_dir = runtime_root / "current_waypoint"
            sdk_result = run_qr_localize_sdk(
                self._localize_sdk_python,
                resolve_qr_localize_sdk_path(self._localize_sdk_path),
                runtime_point_dir,
                resolved["mapYmlPath"],
                resolved["calibrationPath"],
                resolved["extrinsicPath"],
                current_waypoint_dir,
                normalize_min_markers(params.min_markers),
                stats_path,
                self._localize_timeout_seconds,
            )
            current_base_tag = read_matrix_4x4(current_waypoint_dir / "tf_baselink_tag.json")
            stats = read_json_object(stats_path)
            poses, action_data = build_target_outputs(current_base_tag, tag_ee_by_target)
            primary_target_name = next(iter(action_data))

            return {
                "available": True,
                "backend": "executor.qr_localize_sdk",
                "action": ACTION_QR_POSE,
                "robotSerial": params.robot_serial,
                "projectName": params.project_name,
                "mapName": resolved["mapName"],
                "initialPhotoPointName": initial_photo_point,
                "arm": params.arm,
                "cameraId": params.camera_id,
                "targetPointNames": list(action_data.keys()),
                "pose": poses[primary_target_name],
                "action_data": action_data,
                "poses": poses,
                "currentTagPose": matrix_to_pose(current_base_tag),
                "quality": build_localize_quality(stats),
                "sdk": sdk_result,
                "stats": stats,
                "artifactPaths": {
                    "runtimeRoot": str(runtime_root),
                    "runtimePointDir": str(runtime_point_dir),
                    "currentWaypointDir": str(current_waypoint_dir),
                    "statsPath": str(stats_path),
                    "tfBaselinkTagPath": str(current_waypoint_dir / "tf_baselink_tag.json"),
                    "tfTagEePath": str(tf_tag_ee_path),
                    "mapYmlPath": str(resolved["mapYmlPath"]),
                    "calibrationPath": str(resolved["calibrationPath"]),
                    "extrinsicPath": str(resolved["extrinsicPath"]),
                },
                "gdk": {
                    "absPose": snapshot.get("absPose"),
                    "absJointsCount": len(snapshot.get("absJoints"))
                    if isinstance(snapshot.get("absJoints"), Sequence)
                    else 0,
                    "stationarity": snapshot.get("stationarity"),
                    "timestampNs": snapshot.get("timestampNs"),
                    "imageSha256": snapshot.get("imageSha256"),
                },
                **paths_to_payload(paths),
            }
        except Exception as error:
            if snapshot is not None:
                cleanup_temp_image(snapshot)
            return qr_pose_error_payload(error, runtime_root=runtime_root)


def materialize_qr_pose_scan_files(
    *,
    runtime_root: Path,
    snapshot: Mapping[str, object],
) -> Path:
    point_dir = runtime_root / "point"
    point_dir.mkdir(parents=True, exist_ok=True)
    temp_image_path = Path(str(snapshot["imageTempPath"]))
    scan_image_name = str(snapshot["scanImageFileName"])
    scan_image_path = point_dir / scan_image_name
    temp_image_path.replace(scan_image_path)
    (point_dir / "scan_abs_pose.json").write_text(
        json.dumps(to_jsonable(snapshot["absPose"]), ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    (point_dir / "scan_abs_joints.json").write_text(
        json.dumps(to_jsonable(snapshot["absJoints"]), ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    (point_dir / "scan_metadata.json").write_text(
        json.dumps(
            to_jsonable(
                {
                    "action": ACTION_QR_POSE,
                    "cameraId": snapshot.get("cameraId"),
                    "timestampNs": snapshot.get("timestampNs"),
                    "imageSha256": snapshot.get("imageSha256"),
                    "stationarity": snapshot.get("stationarity"),
                    "collectedAt": snapshot.get("collectedAt"),
                }
            ),
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return point_dir


def read_tag_ee_targets(path: Path) -> dict[str, list[list[float]]]:
    decoded = read_json_object(path)
    targets: dict[str, list[list[float]]] = {}
    for name, matrix in decoded.items():
        if isinstance(name, str) and name:
            targets[name] = read_matrix_4x4_value(matrix, f"{path}:{name}")
    return targets


def read_matrix_4x4(path: Path) -> list[list[float]]:
    return read_matrix_4x4_value(json.loads(path.read_text(encoding="utf-8")), str(path))


def read_matrix_4x4_value(value: object, label: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError(f"{label} 必须是 4x4 矩阵")
    matrix: list[list[float]] = []
    for row_index, row in enumerate(value):
        if not isinstance(row, list) or len(row) != 4:
            raise ValueError(f"{label}[{row_index}] 必须是长度 4 的数组")
        matrix.append([float(item) for item in row])
    return matrix


def build_target_outputs(
    current_base_tag: Sequence[Sequence[float]],
    tag_ee_by_target: Mapping[str, Sequence[Sequence[float]]],
) -> tuple[dict[str, dict[str, list[float]]], dict[str, list[float]]]:
    poses: dict[str, dict[str, list[float]]] = {}
    action_data: dict[str, list[float]] = {}
    for target_name in sorted(tag_ee_by_target):
        target_matrix = matmul4(current_base_tag, tag_ee_by_target[target_name])
        pose = matrix_to_pose(target_matrix)
        poses[target_name] = pose
        action_data[target_name] = [*pose["position"], *pose["orientation"]]
    return poses, action_data


def matmul4(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> list[list[float]]:
    return [
        [
            sum(float(left[row][k]) * float(right[k][col]) for k in range(4))
            for col in range(4)
        ]
        for row in range(4)
    ]


def matrix_to_pose(matrix: Sequence[Sequence[float]]) -> dict[str, list[float]]:
    return {
        "position": [
            float(matrix[0][3]),
            float(matrix[1][3]),
            float(matrix[2][3]),
        ],
        "orientation": rotation_matrix_to_quaternion_xyzw(matrix),
    }


def rotation_matrix_to_quaternion_xyzw(matrix: Sequence[Sequence[float]]) -> list[float]:
    m00, m01, m02 = float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2])
    m10, m11, m12 = float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2])
    m20, m21, m22 = float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2])
    trace = m00 + m11 + m22
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    return normalize_quaternion([qx, qy, qz, qw])


def normalize_quaternion(values: Sequence[float]) -> list[float]:
    norm = math.sqrt(sum(float(value) * float(value) for value in values))
    if norm <= 0:
        raise ValueError("二维码定位结果四元数为零")
    return [float(value) / norm for value in values]


def qr_pose_error_payload(
    error: Exception,
    *,
    runtime_root: Path | None,
) -> dict[str, object]:
    return {
        "available": False,
        "backend": "executor.qr_pose",
        "action": ACTION_QR_POSE,
        "errorCode": "QR_POSE_FAILED",
        "errorType": type(error).__name__,
        "errorMsg": str(error),
        "runtimeRoot": str(runtime_root) if runtime_root is not None else None,
    }
