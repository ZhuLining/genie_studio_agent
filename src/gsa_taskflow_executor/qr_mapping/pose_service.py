"""应用工作流二维码定位服务。

二维码建图和点位录制产物都保存在 executor 所在 Ubuntu 主机。应用运行时
`qr_pose_skill` 默认先把执行手臂回到初始拍照点位，再重新拍照计算当前
二维码/tag 在 base 下的位姿，最后乘以点位录制阶段保存的 `T_tag_ee`，
输出目标末端 pose/action_data。
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from uuid import uuid4

from gsa_taskflow_executor.gdk.motion_runtime import run_gdk_motion_plan_abs_joint
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
from gsa_taskflow_executor.taskflow.models import (
    MotionPlanParams,
    MotionPlanTarget,
    QrPoseParams,
)

ACTION_QR_POSE = "qr_pose"
DEFAULT_QR_POSE_LOCALIZE_TIMEOUT_SECONDS = 120.0
INITIAL_PHOTO_SCAN_INDEX_NAME = "scan"
ARM_JOINTS_BY_PART = {
    "left_arm": (
        "idx21_arm_l_joint1",
        "idx22_arm_l_joint2",
        "idx23_arm_l_joint3",
        "idx24_arm_l_joint4",
        "idx25_arm_l_joint5",
        "idx26_arm_l_joint6",
        "idx27_arm_l_joint7",
    ),
    "right_arm": (
        "idx61_arm_r_joint1",
        "idx62_arm_r_joint2",
        "idx63_arm_r_joint3",
        "idx64_arm_r_joint4",
        "idx65_arm_r_joint5",
        "idx66_arm_r_joint6",
        "idx67_arm_r_joint7",
    ),
}


class QrPoseInitialPhotoReturnError(RuntimeError):
    """二维码定位前回初始拍照点位失败，保留 GDK 结果供状态上报排查。"""

    def __init__(
        self,
        message: str,
        *,
        initial_photo_return: Mapping[str, object],
    ) -> None:
        super().__init__(message)
        self.initial_photo_return = dict(initial_photo_return)


class QrPoseService:
    """封装二维码定位节点的远端资源读取、回位、GDK 采样和 SDK 调用。"""

    def __init__(
        self,
        *,
        project_store: QrProjectStore,
        session_manager: GdkSessionManager,
        localize_sdk_path: str = "",
        localize_sdk_python: str = "python3",
        localize_timeout_seconds: float = DEFAULT_QR_POSE_LOCALIZE_TIMEOUT_SECONDS,
        environ: Mapping[str, str] | None = None,
    ) -> None:
        self._project_store = project_store
        self._session_manager = session_manager
        self._localize_sdk_path = localize_sdk_path.strip()
        self._localize_sdk_python = localize_sdk_python.strip() or "python3"
        self._localize_timeout_seconds = localize_timeout_seconds
        self._environ = environ

    def locate(self, params: QrPoseParams) -> dict[str, object]:
        """执行一次二维码定位，返回可写入 taskflow 变量空间的 outputs。"""

        runtime_root: Path | None = None
        snapshot: Mapping[str, object] | None = None
        initial_photo_return: Mapping[str, object] | None = None
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

            initial_photo_return = self._return_to_initial_photo_pose(
                waypoint_dir=waypoint_dir,
                params=params,
            )

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
                return {
                    **dict(snapshot),
                    "initialPhotoReturn": initial_photo_return,
                }

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
            abs_joints = snapshot.get("absJoints")

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
                "initialPhotoReturn": initial_photo_return,
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
                    "absJointsCount": len(abs_joints)
                    if isinstance(abs_joints, Sequence)
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
            return qr_pose_error_payload(
                error,
                runtime_root=runtime_root,
                initial_photo_return=initial_photo_return,
            )

    def _return_to_initial_photo_pose(
        self,
        *,
        waypoint_dir: Path,
        params: QrPoseParams,
    ) -> Mapping[str, object]:
        if not params.return_to_initial_photo_pose:
            return {
                "enabled": False,
                "executed": False,
                "reason": "disabled_by_params",
            }

        try:
            motion_params, joints_path = build_initial_photo_return_motion_params(
                waypoint_dir=waypoint_dir,
                params=params,
            )
        except Exception as error:
            raise QrPoseInitialPhotoReturnError(
                f"回初始拍照点位准备失败: {error}",
                initial_photo_return={
                    "enabled": True,
                    "executed": False,
                    "errorStage": "load_initial_photo_joints",
                    "errorType": type(error).__name__,
                    "errorMsg": str(error),
                    "waypointDir": str(waypoint_dir),
                    "sourceIndexName": INITIAL_PHOTO_SCAN_INDEX_NAME,
                    "bodyPart": params.arm,
                },
            ) from error
        # 二维码定位节点内部会触发真实手臂运动；这里不绕过安全门，
        # 统一复用 Taskflow ABS_JOINT 的 ENABLE/CONFIRM 双确认和恢复门。
        result = run_gdk_motion_plan_abs_joint(
            motion_params,
            environ=self._environ,
            session_manager=self._session_manager,
        )
        json_result = to_jsonable(result)
        result_payload = dict(json_result) if isinstance(json_result, Mapping) else {}
        payload = {
            **result_payload,
            "enabled": True,
            "jointsPath": str(joints_path),
            "sourceIndexName": INITIAL_PHOTO_SCAN_INDEX_NAME,
            "bodyPart": params.arm,
        }
        if result.get("executed") is not True:
            message = str(result.get("error_msg") or "回初始拍照点位失败")
            raise QrPoseInitialPhotoReturnError(
                f"回初始拍照点位失败: {message}",
                initial_photo_return=payload,
            )
        return payload


def build_initial_photo_return_motion_params(
    *,
    waypoint_dir: Path,
    params: QrPoseParams,
) -> tuple[MotionPlanParams, Path]:
    joints_path = waypoint_dir / "joints.json"
    if not joints_path.exists():
        raise FileNotFoundError(f"未找到初始拍照点位 joints.json: {joints_path}")

    decoded = read_json_object(joints_path)
    command = read_recorded_command(decoded, INITIAL_PHOTO_SCAN_INDEX_NAME, str(joints_path))
    action_data = read_arm_joint_positions_from_command(
        command,
        body_part=params.arm,
        label=f"{joints_path}:{INITIAL_PHOTO_SCAN_INDEX_NAME}",
    )
    return (
        MotionPlanParams(
            targets=(
                MotionPlanTarget(
                    body_part=params.arm,
                    control_type="ABS_JOINT",
                    action_data=action_data,
                ),
            ),
            speed=params.return_pose_speed,
            timeout=params.return_pose_timeout,
        ),
        joints_path,
    )


def read_recorded_command(
    decoded: Mapping[str, object],
    index_name: str,
    label: str,
) -> Mapping[str, object]:
    raw_commands = decoded.get("recorded_commands")
    if not isinstance(raw_commands, list):
        raise ValueError(f"{label}.recorded_commands 必须是数组")
    for item in raw_commands:
        if isinstance(item, Mapping) and item.get("index_name") == index_name:
            return item
    raise FileNotFoundError(f"{label} 未找到 index_name={index_name!r} 的关节记录")


def read_arm_joint_positions_from_command(
    command: Mapping[str, object],
    *,
    body_part: str,
    label: str,
) -> list[float]:
    joint_names = read_non_empty_string_list(command.get("joint_names"), f"{label}.joint_names")
    joint_positions = read_number_list(
        command.get("joint_positions"),
        f"{label}.joint_positions",
    )
    if len(joint_names) != len(joint_positions):
        raise ValueError(f"{label}.joint_names 与 joint_positions 长度不一致")

    position_by_name = {
        joint_name: joint_positions[index]
        for index, joint_name in enumerate(joint_names)
    }
    expected_joints = ARM_JOINTS_BY_PART.get(body_part)
    if expected_joints is None:
        raise ValueError(f"{label} 不支持的执行手臂: {body_part}")
    missing = [joint_name for joint_name in expected_joints if joint_name not in position_by_name]
    if missing:
        raise ValueError(f"{label} 缺少 {body_part} 关节: {', '.join(missing)}")
    return [position_by_name[joint_name] for joint_name in expected_joints]


def read_non_empty_string_list(value: object, label: str) -> list[str]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{label} 必须是非空字符串数组")
    result: list[str] = []
    for index, item in enumerate(value):
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f"{label}[{index}] 必须是非空字符串")
        result.append(item.strip())
    return result


def read_number_list(value: object, label: str) -> list[float]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{label} 必须是非空数字数组")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, int | float):
            raise ValueError(f"{label}[{index}] 必须是数字")
        result.append(float(item))
    return result


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
    initial_photo_return: Mapping[str, object] | None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "available": False,
        "backend": "executor.qr_pose",
        "action": ACTION_QR_POSE,
        "errorCode": "QR_POSE_FAILED",
        "errorType": type(error).__name__,
        "errorMsg": str(error),
        "runtimeRoot": str(runtime_root) if runtime_root is not None else None,
    }
    if isinstance(error, QrPoseInitialPhotoReturnError):
        payload["errorCode"] = "QR_POSE_INITIAL_RETURN_FAILED"
        payload["initialPhotoReturn"] = error.initial_photo_return
    elif initial_photo_return is not None:
        payload["initialPhotoReturn"] = dict(initial_photo_return)
    return payload
