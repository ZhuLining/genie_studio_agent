"""二维码点位录制远端资源服务。

方案 2 下点位录制产物必须落在 executor 所在 Ubuntu 主机。客户端只传
robotSerial/projectName/mapName/pointName 等业务参数，executor 负责只读采样
GDK 当前末端位姿、关节和相机帧，并调用离线 qr_localize SDK 生成 waypoint。
"""

from __future__ import annotations

import base64
import json
import math
import os
import shutil
import subprocess
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from uuid import uuid4

from gsa_taskflow_executor.gdk.camera_frame import (
    DEFAULT_CAMERA_WARMUP_SECONDS,
    build_camera_child_artifact_path,
    build_camera_frame_snapshot,
    remove_camera_child_artifact,
    resolve_gdk_camera_type,
    validate_camera_id,
)
from gsa_taskflow_executor.gdk.control_probe import (
    initialize_gdk,
    is_zero_error,
    read_joint_position,
    release_gdk,
    utc_now_iso,
)
from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from gsa_taskflow_executor.gdk.session import GdkSessionManager
from gsa_taskflow_executor.qr_mapping.build_service import (
    append_bounded,
    resolve_project_calibration_file,
)
from gsa_taskflow_executor.qr_mapping.calibration_store import (
    extrinsic_file_name_from_camera_id,
)
from gsa_taskflow_executor.qr_mapping.capture_service import (
    ensure_project_layout,
    read_manifest,
)
from gsa_taskflow_executor.qr_mapping.project_store import (
    QrProjectPaths,
    QrProjectStore,
    list_map_records,
    paths_to_payload,
    read_json_object,
    validate_safe_segment,
)

ACTION_SAVE_QR_TARGET_POINT = "save_qr_target_point"
ACTION_SAVE_QR_INITIAL_PHOTO_POINT = "save_qr_initial_photo_point"
ACTION_SUBMIT_POINT_RECORDING = "submit_point_recording"
POINT_RECORDING_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，点位录制已拒绝"
DEFAULT_POINT_RECORDING_TIMEOUT_MS = 60000
DEFAULT_LOCALIZE_TIMEOUT_SECONDS = 120.0
DEFAULT_MIN_MARKERS = 4
DEFAULT_LOCALIZE_MAX_REPROJ_PX = 2.0
MAX_MIN_MARKERS = 32
DEFAULT_MAX_MOTION_DURING_CAPTURE_MM = 1.0
DEFAULT_MAX_ROTATION_DURING_CAPTURE_DEG = 0.5

POINT_RECORDING_JOINT_NAMES = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

ARM_FRAME_NAMES = {
    "left_arm": "arm_l_end_link",
    "right_arm": "arm_r_end_link",
}

ARM_CAMERA_IDS = {
    "left_arm": "hand_left_color",
    "right_arm": "hand_right_color",
}


@dataclass(frozen=True)
class PointRecordingSaveTargetParams:
    robot_serial: str
    project_name: str
    point_name: str
    arm: str
    camera_id: str
    map_name: str | None = None
    timeout_ms: int = DEFAULT_POINT_RECORDING_TIMEOUT_MS


@dataclass(frozen=True)
class PointRecordingSaveInitialPhotoParams:
    robot_serial: str
    project_name: str
    point_name: str
    arm: str
    camera_id: str
    map_name: str | None = None
    timeout_ms: int = DEFAULT_POINT_RECORDING_TIMEOUT_MS
    min_markers: int = DEFAULT_MIN_MARKERS


@dataclass(frozen=True)
class PointRecordingSubmitParams:
    robot_serial: str
    project_name: str


PointSnapshotCollector = Callable[
    [str, str, str, int, bool, Path],
    Mapping[str, object],
]
LocalizeSdkRunner = Callable[
    [str, Path | None, Path, Path, Path, Path, Path, int, Path, float],
    Mapping[str, object],
]


class QrLocalizeSdkError(RuntimeError):
    """二维码定位 SDK 失败时保留诊断信息，便于现场判断是识别还是 PnP 问题。"""

    def __init__(
        self,
        message: str,
        *,
        return_code: int,
        stdout: str,
        stderr: str,
        stats_path: Path,
        stats: Mapping[str, object],
    ) -> None:
        super().__init__(message)
        self.return_code = return_code
        self.stdout = stdout
        self.stderr = stderr
        self.stats_path = stats_path
        self.stats = dict(stats)


class QrLocalizeQualityError(RuntimeError):
    """二维码定位 SDK 返回成功，但质量指标未达到可用于点位录制的门禁。"""

    def __init__(
        self,
        message: str,
        *,
        stats_path: Path,
        stats: Mapping[str, object],
        min_markers: int,
        max_reproj_px: float,
        output_dir: Path,
    ) -> None:
        super().__init__(message)
        self.stats_path = stats_path
        self.stats = dict(stats)
        self.min_markers = min_markers
        self.max_reproj_px = max_reproj_px
        self.output_dir = output_dir


class PointRecordingService:
    """封装点位录制落盘和 qr_localize SDK 调用。"""

    def __init__(
        self,
        *,
        project_store: QrProjectStore,
        session_manager: GdkSessionManager,
        localize_sdk_path: str = "",
        localize_sdk_python: str = "python3",
        localize_timeout_seconds: float = DEFAULT_LOCALIZE_TIMEOUT_SECONDS,
        snapshot_collector: PointSnapshotCollector | None = None,
        sdk_runner: LocalizeSdkRunner | None = None,
    ) -> None:
        self._project_store = project_store
        self._session_manager = session_manager
        self._localize_sdk_path = localize_sdk_path.strip()
        self._localize_sdk_python = localize_sdk_python.strip() or "python3"
        self._localize_timeout_seconds = localize_timeout_seconds
        self._snapshot_collector = snapshot_collector
        self._sdk_runner = sdk_runner or run_qr_localize_sdk

    def save_target_point(self, params: PointRecordingSaveTargetParams) -> dict[str, object]:
        try:
            paths = self._project_store.build_paths(
                robot_serial=params.robot_serial,
                project_name=params.project_name,
            )
            point_name = validate_safe_segment(params.point_name, "pointName")
            resolved = resolve_recording_resources(paths, params.map_name, params.camera_id)
            validate_arm_camera(params.arm, params.camera_id)
            validate_timeout_ms(params.timeout_ms)

            snapshot = self._collect_snapshot(
                ACTION_SAVE_QR_TARGET_POINT,
                params.arm,
                params.camera_id,
                params.timeout_ms,
                include_image=False,
                temp_dir=paths.point_dir,
            )
            if snapshot.get("available") is not True:
                return dict(snapshot)

            ensure_project_layout(paths)
            record = build_grasp_point_record(
                point_name=point_name,
                arm=params.arm,
                camera_id=params.camera_id,
                map_name=resolved["mapName"],
                snapshot=snapshot,
            )
            save_grasp_point(paths, record)
            update_manifest_after_target(paths, record)
            return {
                "available": True,
                "backend": GDK_BACKEND,
                "action": ACTION_SAVE_QR_TARGET_POINT,
                "robotSerial": params.robot_serial.strip(),
                "projectName": params.project_name.strip(),
                "pointKind": "target",
                "pointName": point_name,
                "arm": params.arm,
                "cameraId": params.camera_id,
                "mapName": resolved["mapName"],
                "savedAt": record["recorded_at"],
                "pointFilePath": str(paths.point_dir / "grasp_points.json"),
                "absJointsCount": len(record["abs_joints"]),
                **paths_to_payload(paths),
                "gdk": snapshot,
                "collectedAt": utc_now_iso(),
            }
        except Exception as error:
            return error_payload(ACTION_SAVE_QR_TARGET_POINT, error)

    def save_initial_photo_point(
        self,
        params: PointRecordingSaveInitialPhotoParams,
    ) -> dict[str, object]:
        try:
            paths = self._project_store.build_paths(
                robot_serial=params.robot_serial,
                project_name=params.project_name,
            )
            point_name = validate_safe_segment(params.point_name, "pointName")
            resolved = resolve_recording_resources(paths, params.map_name, params.camera_id)
            validate_arm_camera(params.arm, params.camera_id)
            validate_timeout_ms(params.timeout_ms)
            min_markers = normalize_min_markers(params.min_markers)
            waypoint_dir = paths.waypoints_dir / point_name
            if waypoint_dir.exists() and any(waypoint_dir.iterdir()):
                raise ValueError("同名初始拍照点位已存在，请更换点位名称")

            ensure_project_layout(paths)
            snapshot = self._collect_snapshot(
                ACTION_SAVE_QR_INITIAL_PHOTO_POINT,
                params.arm,
                params.camera_id,
                params.timeout_ms,
                include_image=True,
                temp_dir=paths.point_dir,
            )
            if snapshot.get("available") is not True:
                cleanup_temp_image(snapshot)
                return dict(snapshot)

            scan_files = materialize_scan_files(
                paths=paths,
                snapshot=snapshot,
                point_name=point_name,
                arm=params.arm,
                camera_id=params.camera_id,
                map_name=resolved["mapName"],
            )
            stats_path = waypoint_dir / "locate_stats.json"
            sdk_result = self._sdk_runner(
                self._localize_sdk_python,
                resolve_qr_localize_sdk_path(self._localize_sdk_path),
                paths.point_dir,
                resolved["mapYmlPath"],
                resolved["calibrationPath"],
                resolved["extrinsicPath"],
                waypoint_dir,
                min_markers,
                stats_path,
                self._localize_timeout_seconds,
            )
            validate_qr_localize_quality(
                read_json_object(stats_path),
                min_markers=min_markers,
                max_reproj_px=DEFAULT_LOCALIZE_MAX_REPROJ_PX,
                stats_path=stats_path,
                output_dir=waypoint_dir,
            )
            record = build_initial_photo_record(
                paths=paths,
                point_name=point_name,
                arm=params.arm,
                camera_id=params.camera_id,
                map_name=resolved["mapName"],
                scan_files=scan_files,
                waypoint_dir=waypoint_dir,
                stats_path=stats_path,
                sdk_result=sdk_result,
            )
            update_manifest_after_initial(paths, record)
            return {
                "available": True,
                "backend": "executor.qr_localize_sdk",
                "action": ACTION_SAVE_QR_INITIAL_PHOTO_POINT,
                "robotSerial": params.robot_serial.strip(),
                "projectName": params.project_name.strip(),
                **record,
                **paths_to_payload(paths),
                "gdk": summarize_gdk_snapshot(snapshot),
                "sdk": sdk_result,
                "collectedAt": utc_now_iso(),
            }
        except Exception as error:
            if isinstance(error, QrLocalizeQualityError):
                cleanup_waypoint_dir(error.output_dir)
            return error_payload(ACTION_SAVE_QR_INITIAL_PHOTO_POINT, error)

    def submit_recording(self, params: PointRecordingSubmitParams) -> dict[str, object]:
        try:
            paths = self._project_store.build_paths(
                robot_serial=params.robot_serial,
                project_name=params.project_name,
            )
            target_points = read_grasp_points(paths)
            initial_points = list_initial_photo_records(paths)
            missing = missing_submit_resources(paths, target_points, initial_points)
            if missing:
                raise ValueError("点位录制产物不完整: " + "、".join(missing))
            manifest = read_manifest(paths.manifest_path)
            manifest.update(
                {
                    "projectStatus": "recorded",
                    "pointRecordingSubmittedAt": utc_now_iso(),
                    "targetPointCount": len(target_points),
                    "initialPhotoPointCount": len(initial_points),
                    "updatedAt": utc_now_iso(),
                }
            )
            paths.manifest_path.write_text(
                json.dumps(to_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            return {
                "available": True,
                "backend": "executor.filesystem",
                "action": ACTION_SUBMIT_POINT_RECORDING,
                "robotSerial": params.robot_serial.strip(),
                "projectName": params.project_name.strip(),
                "targetPointCount": len(target_points),
                "initialPhotoPointCount": len(initial_points),
                "submittedAt": manifest["pointRecordingSubmittedAt"],
                **paths_to_payload(paths),
                "collectedAt": utc_now_iso(),
            }
        except Exception as error:
            return error_payload(ACTION_SUBMIT_POINT_RECORDING, error)

    def _collect_snapshot(
        self,
        action: str,
        arm: str,
        camera_id: str,
        timeout_ms: int,
        *,
        include_image: bool,
        temp_dir: Path,
    ) -> Mapping[str, object]:
        collector = self._snapshot_collector
        if collector is not None:
            return collector(action, arm, camera_id, timeout_ms, include_image, temp_dir)
        return collect_point_recording_gdk_snapshot(
            action=action,
            arm=arm,
            camera_id=camera_id,
            timeout_ms=timeout_ms,
            include_image=include_image,
            temp_dir=temp_dir,
            session_manager=self._session_manager,
        )


def resolve_recording_resources(
    paths: QrProjectPaths,
    map_name: str | None,
    camera_id: str,
) -> dict[str, Any]:
    normalized_map_name, map_yml_path = resolve_map_yml_path(paths, map_name)
    calibration_path = resolve_map_calibration_path(paths, normalized_map_name, camera_id)
    extrinsic_file_name = extrinsic_file_name_from_camera_id(camera_id)
    if not extrinsic_file_name:
        raise ValueError(f"点位录制暂不支持相机 {camera_id}，请选择手部彩色相机")
    extrinsic_path = paths.sensor_root / extrinsic_file_name
    if not extrinsic_path.exists():
        raise FileNotFoundError(f"未找到相机外参文件: {extrinsic_path}")
    return {
        "mapName": normalized_map_name,
        "mapYmlPath": map_yml_path,
        "calibrationPath": calibration_path,
        "extrinsicPath": extrinsic_path,
    }


def resolve_map_yml_path(paths: QrProjectPaths, map_name: str | None) -> tuple[str, Path]:
    if map_name and map_name.strip():
        normalized_map_name = validate_safe_segment(map_name, "mapName")
    else:
        manifest = read_manifest(paths.manifest_path)
        active = manifest.get("activeMapName")
        if isinstance(active, str) and active.strip():
            normalized_map_name = validate_safe_segment(active, "activeMapName")
        else:
            maps = list_map_records(paths.maps_dir, manifest=manifest)
            if not maps:
                raise FileNotFoundError("当前项目还没有可用地图，请先完成二维码建图")
            normalized_map_name = str(maps[0]["mapName"])
    map_yml_path = paths.maps_dir / f"{normalized_map_name}.yml"
    if not map_yml_path.exists():
        raise FileNotFoundError(f"未找到地图文件: {map_yml_path}")
    return normalized_map_name, map_yml_path


def resolve_map_calibration_path(paths: QrProjectPaths, map_name: str, camera_id: str) -> Path:
    cam_yml_path = paths.maps_dir / f"{map_name}-cam.yml"
    if cam_yml_path.exists():
        return cam_yml_path
    return resolve_project_calibration_file(paths, camera_id)


def validate_arm_camera(arm: str, camera_id: str) -> None:
    if arm not in ARM_FRAME_NAMES:
        raise ValueError("执行手臂必须是 left_arm 或 right_arm")
    expected_camera = ARM_CAMERA_IDS[arm]
    if camera_id != expected_camera:
        raise ValueError(f"{arm} 点位录制需要使用 {expected_camera}")
    camera_result = validate_camera_id(camera_id)
    if camera_result is not None:
        raise ValueError(str(camera_result.get("errorMsg") or "不支持的相机"))


def validate_timeout_ms(timeout_ms: int) -> None:
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
        raise ValueError("timeoutMs 必须是正整数")


def normalize_min_markers(value: int) -> int:
    if isinstance(value, bool):
        return DEFAULT_MIN_MARKERS
    return max(1, min(int(value), MAX_MIN_MARKERS))


def collect_point_recording_gdk_snapshot(
    *,
    action: str,
    arm: str,
    camera_id: str,
    timeout_ms: int,
    include_image: bool,
    temp_dir: Path,
    session_manager: GdkSessionManager,
) -> dict[str, object]:
    try:
        lease = session_manager.acquire(
            blocking=False,
            initialize=False,
            purpose=action,
        )
    except Exception as error:
        return unavailable_result(action, "gdk_session_acquire", error)
    if lease is None:
        return busy_result(action, active_purpose=session_manager.active_purpose)

    progress_path = build_camera_child_artifact_path("point_recording_progress")
    with lease:
        try:
            from gsa_taskflow_executor.gdk.worker_runtime import (
                run_point_recording_snapshot_in_worker,
            )

            # 点位录制会频繁读取 Robot 状态。G2 实测 Robot() 构造可能接近 30s，
            # 因此这里走常驻 GDK worker，并在 worker 内复用 Robot，避免每个点位
            # 都重新初始化 DDS/控制状态。
            result = run_point_recording_snapshot_in_worker(
                {
                    "action": action,
                    "arm": arm,
                    "camera_id": camera_id,
                    "timeout_ms": timeout_ms,
                    "include_image": include_image,
                    "temp_dir": str(temp_dir),
                    "warmup_seconds": DEFAULT_CAMERA_WARMUP_SECONDS,
                    "max_motion_mm": DEFAULT_MAX_MOTION_DURING_CAPTURE_MM,
                    "max_rotation_deg": DEFAULT_MAX_ROTATION_DURING_CAPTURE_DEG,
                    "progress_path": str(progress_path),
                },
                action=action,
                timeout_seconds=build_gdk_snapshot_timeout_seconds(
                    timeout_ms,
                    include_image=include_image,
                ),
                safety_gate={
                    "enabled": False,
                    "confirmed": True,
                    "reason": "read_only_point_recording",
                },
            )
            attach_point_recording_child_progress(result, progress_path)
            result["gdk_parent_lock"] = lease.to_payload()
            return result
        finally:
            remove_camera_child_artifact(progress_path)


def build_gdk_snapshot_timeout_seconds(timeout_ms: int, *, include_image: bool) -> float:
    warmup = DEFAULT_CAMERA_WARMUP_SECONDS if include_image else 0.0
    return max(6.0, timeout_ms / 1000.0 + warmup + 6.0)


def execute_point_recording_snapshot(
    *,
    agibot_gdk: Any,
    robot: Any,
    action: str,
    arm: str,
    camera_id: str,
    timeout_ms: int,
    include_image: bool,
    temp_dir: str,
    warmup_seconds: float,
    max_motion_mm: float,
    max_rotation_deg: float,
    progress_path: str | None = None,
) -> dict[str, object]:
    """在已初始化的 GDK worker 内执行一次点位录制采样。

    该函数只读 Robot 状态和相机图像，不调用运动控制。它被常驻 worker 调用，
    这样可以复用 worker 内的 Robot()，减少连续保存点位时的初始化等待。
    """

    camera = None
    write_point_recording_child_progress(
        progress_path,
        "validate_robot_status_started",
        action=action,
        arm=arm,
    )
    validate_robot_status_for_arm(robot, arm)
    write_point_recording_child_progress(
        progress_path,
        "validate_robot_status_finished",
        action=action,
        arm=arm,
    )

    try:
        if include_image:
            gdk_camera_type = resolve_gdk_camera_type(agibot_gdk, camera_id)
            write_point_recording_child_progress(
                progress_path,
                "camera_create_started",
                action=action,
                camera_id=camera_id,
            )
            camera = agibot_gdk.Camera()
            write_point_recording_child_progress(
                progress_path,
                "camera_created",
                action=action,
                camera_id=camera_id,
            )
            if warmup_seconds > 0:
                # Camera 初始化后首帧容易失败，沿用二维码建图采集的保守 warmup。
                import time

                write_point_recording_child_progress(
                    progress_path,
                    "camera_warmup_started",
                    action=action,
                    camera_id=camera_id,
                    warmup_seconds=warmup_seconds,
                )
                time.sleep(warmup_seconds)
                write_point_recording_child_progress(
                    progress_path,
                    "camera_warmup_finished",
                    action=action,
                    camera_id=camera_id,
                )
            write_point_recording_child_progress(
                progress_path,
                "read_before_pose_started",
                action=action,
                arm=arm,
            )
            before_pose = read_arm_end_pose(robot, arm)
            write_point_recording_child_progress(
                progress_path,
                "get_latest_image_started",
                action=action,
                camera_id=camera_id,
            )
            image = camera.get_latest_image(gdk_camera_type, float(timeout_ms))
            write_point_recording_child_progress(
                progress_path,
                "get_latest_image_returned",
                action=action,
                camera_id=camera_id,
            )
            if image is None:
                raise RuntimeError("camera.get_latest_image() returned None")
            write_point_recording_child_progress(
                progress_path,
                "read_after_pose_started",
                action=action,
                arm=arm,
            )
            after_pose = read_arm_end_pose(robot, arm)
            motion = build_stationarity_payload(before_pose, after_pose)
            if motion["motionMm"] > max_motion_mm or motion["rotationDeg"] > max_rotation_deg:
                raise RuntimeError(
                    "拍照期间末端发生移动: "
                    f"{motion['motionMm']:.3f}mm/{motion['rotationDeg']:.3f}deg"
                )
            pose = mean_pose(before_pose, after_pose)
            write_point_recording_child_progress(
                progress_path,
                "read_joint_positions_started",
                action=action,
            )
            joints = read_joint_positions(robot)
            write_point_recording_child_progress(
                progress_path,
                "encode_frame_started",
                action=action,
                camera_id=camera_id,
            )
            frame_snapshot = build_camera_frame_snapshot(
                image,
                camera_id=camera_id,
                gdk_camera_type=str(gdk_camera_type),
            )
            if frame_snapshot.get("available") is not True:
                raise RuntimeError(str(frame_snapshot.get("errorMsg") or "相机图像编码失败"))
            image_file = write_temp_scan_image(frame_snapshot, Path(temp_dir))
            return build_snapshot_result(
                action,
                arm=arm,
                camera_id=camera_id,
                pose=pose,
                joints=joints,
                extra={
                    "imageTempPath": image_file["imageTempPath"],
                    "scanImageFileName": image_file["scanImageFileName"],
                    "mimeType": frame_snapshot.get("mimeType"),
                    "width": frame_snapshot.get("width"),
                    "height": frame_snapshot.get("height"),
                    "encoding": frame_snapshot.get("encoding"),
                    "timestampNs": frame_snapshot.get("timestampNs"),
                    "imageSha256": frame_snapshot.get("imageSha256"),
                    "stationarity": motion,
                },
            )

        write_point_recording_child_progress(
            progress_path,
            "read_pose_started",
            action=action,
            arm=arm,
        )
        pose = read_arm_end_pose(robot, arm)
        write_point_recording_child_progress(
            progress_path,
            "read_joint_positions_started",
            action=action,
        )
        joints = read_joint_positions(robot)
        return build_snapshot_result(
            action,
            arm=arm,
            camera_id=camera_id,
            pose=pose,
            joints=joints,
        )
    finally:
        if camera is not None:
            close_camera = getattr(camera, "close_camera", None)
            if callable(close_camera):
                try:
                    close_camera()
                except Exception:
                    pass


def point_recording_gdk_child(
    result_queue: Any,
    action: str,
    arm: str,
    camera_id: str,
    timeout_ms: int,
    include_image: bool,
    temp_dir: str,
    warmup_seconds: float,
    max_motion_mm: float,
    max_rotation_deg: float,
    progress_path: str | None = None,
) -> None:
    agibot_gdk = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    release_result: dict[str, object] = {"called": False, "success": True, "return": None}
    result: dict[str, object]
    camera = None
    try:
        write_point_recording_child_progress(progress_path, "child_started", action=action)
        agibot_gdk = __import__(GDK_MODULE_NAME)
        write_point_recording_child_progress(progress_path, "agibot_gdk_imported", action=action)
        write_point_recording_child_progress(progress_path, "gdk_init_started", action=action)
        init_result = initialize_gdk(agibot_gdk)
        write_point_recording_child_progress(
            progress_path,
            "gdk_init_finished",
            action=action,
            gdk_init=init_result,
        )
        if init_result.get("called") is True and init_result.get("success") is not True:
            raise RuntimeError("agibot_gdk.gdk_init() did not return success")
        gdk_initialized = bool(init_result.get("called"))
        write_point_recording_child_progress(progress_path, "robot_create_started", action=action)
        robot = agibot_gdk.Robot()
        write_point_recording_child_progress(progress_path, "robot_created", action=action)
        write_point_recording_child_progress(
            progress_path,
            "validate_robot_status_started",
            action=action,
            arm=arm,
        )
        validate_robot_status_for_arm(robot, arm)
        write_point_recording_child_progress(
            progress_path,
            "validate_robot_status_finished",
            action=action,
            arm=arm,
        )

        if include_image:
            gdk_camera_type = resolve_gdk_camera_type(agibot_gdk, camera_id)
            write_point_recording_child_progress(
                progress_path,
                "camera_create_started",
                action=action,
                camera_id=camera_id,
            )
            camera = agibot_gdk.Camera()
            write_point_recording_child_progress(
                progress_path,
                "camera_created",
                action=action,
                camera_id=camera_id,
            )
            if warmup_seconds > 0:
                # Camera 初始化后首帧容易失败，沿用二维码建图采集的保守 warmup。
                import time

                write_point_recording_child_progress(
                    progress_path,
                    "camera_warmup_started",
                    action=action,
                    camera_id=camera_id,
                    warmup_seconds=warmup_seconds,
                )
                time.sleep(warmup_seconds)
                write_point_recording_child_progress(
                    progress_path,
                    "camera_warmup_finished",
                    action=action,
                    camera_id=camera_id,
                )
            write_point_recording_child_progress(
                progress_path,
                "read_before_pose_started",
                action=action,
                arm=arm,
            )
            before_pose = read_arm_end_pose(robot, arm)
            write_point_recording_child_progress(
                progress_path,
                "get_latest_image_started",
                action=action,
                camera_id=camera_id,
            )
            image = camera.get_latest_image(gdk_camera_type, float(timeout_ms))
            write_point_recording_child_progress(
                progress_path,
                "get_latest_image_returned",
                action=action,
                camera_id=camera_id,
            )
            if image is None:
                raise RuntimeError("camera.get_latest_image() returned None")
            write_point_recording_child_progress(
                progress_path,
                "read_after_pose_started",
                action=action,
                arm=arm,
            )
            after_pose = read_arm_end_pose(robot, arm)
            motion = build_stationarity_payload(before_pose, after_pose)
            if motion["motionMm"] > max_motion_mm or motion["rotationDeg"] > max_rotation_deg:
                raise RuntimeError(
                    "拍照期间末端发生移动: "
                    f"{motion['motionMm']:.3f}mm/{motion['rotationDeg']:.3f}deg"
            )
            pose = mean_pose(before_pose, after_pose)
            write_point_recording_child_progress(
                progress_path,
                "read_joint_positions_started",
                action=action,
            )
            joints = read_joint_positions(robot)
            write_point_recording_child_progress(
                progress_path,
                "encode_frame_started",
                action=action,
                camera_id=camera_id,
            )
            frame_snapshot = build_camera_frame_snapshot(
                image,
                camera_id=camera_id,
                gdk_camera_type=str(gdk_camera_type),
            )
            if frame_snapshot.get("available") is not True:
                raise RuntimeError(str(frame_snapshot.get("errorMsg") or "相机图像编码失败"))
            image_file = write_temp_scan_image(frame_snapshot, Path(temp_dir))
            result = build_snapshot_result(
                action,
                arm=arm,
                camera_id=camera_id,
                pose=pose,
                joints=joints,
                extra={
                    "imageTempPath": image_file["imageTempPath"],
                    "scanImageFileName": image_file["scanImageFileName"],
                    "mimeType": frame_snapshot.get("mimeType"),
                    "width": frame_snapshot.get("width"),
                    "height": frame_snapshot.get("height"),
                    "encoding": frame_snapshot.get("encoding"),
                    "timestampNs": frame_snapshot.get("timestampNs"),
                    "imageSha256": frame_snapshot.get("imageSha256"),
                    "stationarity": motion,
                },
            )
        else:
            write_point_recording_child_progress(
                progress_path,
                "read_pose_started",
                action=action,
                arm=arm,
            )
            pose = read_arm_end_pose(robot, arm)
            write_point_recording_child_progress(
                progress_path,
                "read_joint_positions_started",
                action=action,
            )
            joints = read_joint_positions(robot)
            result = build_snapshot_result(
                action,
                arm=arm,
                camera_id=camera_id,
                pose=pose,
                joints=joints,
            )
    except Exception as error:
        result = unavailable_result(action, "point_recording_gdk_child", error)
    finally:
        if camera is not None:
            close_camera = getattr(camera, "close_camera", None)
            if callable(close_camera):
                try:
                    close_camera()
                except Exception:
                    pass
        if agibot_gdk is not None and gdk_initialized:
            write_point_recording_child_progress(progress_path, "gdk_release_started", action=action)
            release_result = release_gdk(agibot_gdk)
            write_point_recording_child_progress(
                progress_path,
                "gdk_release_finished",
                action=action,
                gdk_release=release_result,
            )
        result.setdefault("gdk_init", init_result)
        result.setdefault("gdk_release", release_result)
        attach_point_recording_child_progress(result, Path(progress_path) if progress_path else None)
        write_point_recording_child_progress(progress_path, "result_put_started", action=action)
        result_queue.put(result)


def validate_robot_status_for_arm(robot: Any, arm: str) -> None:
    whole_body_status = robot.get_whole_body_status()
    if not isinstance(whole_body_status, Mapping):
        raise TypeError("robot.get_whole_body_status() did not return a mapping")
    arm_prefix = "left" if arm == "left_arm" else "right"
    for key in (f"{arm_prefix}_arm_error", f"{arm_prefix}_end_error"):
        if not is_zero_error(whole_body_status.get(key)):
            raise RuntimeError(f"{key}={whole_body_status.get(key)}")
    if whole_body_status.get(f"{arm_prefix}_arm_estop"):
        raise RuntimeError(f"{arm} estop is active")


def read_arm_end_pose(robot: Any, arm: str) -> dict[str, list[float]]:
    status = robot.get_motion_control_status()
    error_code = getattr(status, "error_code", 0)
    if not is_zero_error(error_code):
        raise RuntimeError(
            f"motion control error_code={error_code} "
            f"error_msg={getattr(status, 'error_msg', '')!r}"
        )
    frame_name = ARM_FRAME_NAMES[arm]
    names = [str(value) for value in getattr(status, "frame_names", [])]
    poses = list(getattr(status, "frame_poses", []))
    try:
        index = names.index(frame_name)
    except ValueError as error:
        raise RuntimeError(f"No {frame_name} in frame_names: {names}") from error
    if index >= len(poses):
        raise RuntimeError(f"No pose for {frame_name}")
    pose = poses[index]
    return {
        "position": [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ],
        "orientation": normalize_quaternion(
            [
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ]
        ),
    }


def read_joint_positions(robot: Any) -> list[float]:
    joint_states = robot.get_joint_states()
    if not isinstance(joint_states, Mapping):
        raise TypeError("robot.get_joint_states() did not return a mapping")
    states = joint_states.get("states")
    if not isinstance(states, Sequence) or isinstance(states, str | bytes | bytearray):
        raise TypeError("joint_states['states'] is not a sequence")
    states_by_name = {
        state["name"]: state
        for state in states
        if isinstance(state, Mapping) and isinstance(state.get("name"), str)
    }
    positions: list[float] = []
    for joint_name in POINT_RECORDING_JOINT_NAMES:
        state = states_by_name.get(joint_name)
        if state is None:
            raise RuntimeError(f"missing joint state: {joint_name}")
        if not is_zero_error(state.get("error_code")):
            raise RuntimeError(f"{joint_name} error_code={state.get('error_code')}")
        positions.append(read_joint_position(state))
    return positions


def write_temp_scan_image(frame_snapshot: Mapping[str, object], temp_dir: Path) -> dict[str, str]:
    mime_type = str(frame_snapshot.get("mimeType") or "")
    if mime_type == "image/jpeg":
        scan_file_name = "scan_image.jpeg"
        suffix = "jpeg"
    elif mime_type == "image/png":
        scan_file_name = "scan_image.png"
        suffix = "png"
    else:
        raise RuntimeError(f"qr_localize SDK 暂只支持 JPEG/PNG 扫描图，当前为 {mime_type or '未知'}")
    image_base64 = str(frame_snapshot.get("imageBase64") or "")
    image_bytes = base64.b64decode(strip_data_url_prefix(image_base64), validate=False)
    if not image_bytes:
        raise RuntimeError("相机图像为空，不能保存扫描图")
    temp_dir.mkdir(parents=True, exist_ok=True)
    temp_path = temp_dir / f".scan_image_{uuid4().hex}.{suffix}"
    temp_path.write_bytes(image_bytes)
    return {
        "imageTempPath": str(temp_path),
        "scanImageFileName": scan_file_name,
    }


def strip_data_url_prefix(value: str) -> str:
    comma_index = value.find(",")
    return value[comma_index + 1 :] if value.startswith("data:") and comma_index >= 0 else value


def build_snapshot_result(
    action: str,
    *,
    arm: str,
    camera_id: str,
    pose: Mapping[str, object],
    joints: Sequence[float],
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": True,
        "backend": GDK_BACKEND,
        "action": action,
        "arm": arm,
        "cameraId": camera_id,
        "absPose": dict(pose),
        "absJoints": [float(value) for value in joints],
        "jointNames": list(POINT_RECORDING_JOINT_NAMES),
        "collectedAt": utc_now_iso(),
    }
    if extra:
        result.update(dict(extra))
    return result


def materialize_scan_files(
    *,
    paths: QrProjectPaths,
    snapshot: Mapping[str, object],
    point_name: str,
    arm: str,
    camera_id: str,
    map_name: str,
) -> dict[str, object]:
    temp_image_path = Path(str(snapshot["imageTempPath"]))
    scan_image_file_name = str(snapshot["scanImageFileName"])
    remove_existing_scan_images(paths.point_dir)
    scan_image_path = paths.point_dir / scan_image_file_name
    temp_image_path.replace(scan_image_path)
    scan_pose_path = paths.point_dir / "scan_abs_pose.json"
    scan_joints_path = paths.point_dir / "scan_abs_joints.json"
    scan_metadata_path = paths.point_dir / "scan_metadata.json"
    scan_pose_path.write_text(
        json.dumps(to_jsonable(snapshot["absPose"]), ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    scan_joints_path.write_text(
        json.dumps(to_jsonable(snapshot["absJoints"]), ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    scan_metadata_path.write_text(
        json.dumps(
            to_jsonable(
                {
                    "pointName": point_name,
                    "arm": arm,
                    "cameraId": camera_id,
                    "mapName": map_name,
                    "timestampNs": snapshot.get("timestampNs"),
                    "imageSha256": snapshot.get("imageSha256"),
                    "stationarity": snapshot.get("stationarity"),
                    "recordedAt": snapshot.get("collectedAt") or utc_now_iso(),
                }
            ),
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return {
        "scanImagePath": str(scan_image_path),
        "scanAbsPosePath": str(scan_pose_path),
        "scanAbsJointsPath": str(scan_joints_path),
        "scanMetadataPath": str(scan_metadata_path),
    }


def remove_existing_scan_images(point_dir: Path) -> None:
    for file_name in ("scan_image.jpeg", "scan_image.jpg", "scan_image.png"):
        path = point_dir / file_name
        if path.exists() and path.is_file():
            path.unlink()


def cleanup_temp_image(snapshot: Mapping[str, object]) -> None:
    raw_path = snapshot.get("imageTempPath")
    if not isinstance(raw_path, str) or not raw_path:
        return
    try:
        path = Path(raw_path)
        if path.exists() and path.is_file():
            path.unlink()
    except OSError:
        pass


def cleanup_waypoint_dir(path: Path) -> None:
    try:
        if path.exists() and path.is_dir():
            shutil.rmtree(path)
    except OSError:
        pass


def build_grasp_point_record(
    *,
    point_name: str,
    arm: str,
    camera_id: str,
    map_name: str,
    snapshot: Mapping[str, object],
) -> dict[str, object]:
    return {
        "grasp_name": point_name,
        "abs_joints": list(snapshot["absJoints"]),
        "abs_pose": dict(snapshot["absPose"]),
        "recorded_at": snapshot.get("collectedAt") or utc_now_iso(),
        "arm": arm,
        "camera_id": camera_id,
        "map_name": map_name,
    }


def save_grasp_point(paths: QrProjectPaths, record: Mapping[str, object]) -> None:
    grasp_points_path = paths.point_dir / "grasp_points.json"
    points = read_grasp_points(paths)
    point_name = str(record["grasp_name"])
    if any(item.get("grasp_name") == point_name for item in points):
        raise ValueError("同名目标点位已存在，请更换点位名称")
    points.append(dict(record))
    grasp_points_path.write_text(
        json.dumps(to_jsonable(points), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def read_grasp_points(paths: QrProjectPaths) -> list[dict[str, object]]:
    path = paths.point_dir / "grasp_points.json"
    if not path.exists() or not path.is_file():
        return []
    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    return [dict(item) for item in decoded if isinstance(item, Mapping)] if isinstance(decoded, list) else []


def list_target_point_records(paths: QrProjectPaths) -> list[dict[str, object]]:
    records = []
    file_path = paths.point_dir / "grasp_points.json"
    for item in read_grasp_points(paths):
        point_name = item.get("grasp_name")
        if not isinstance(point_name, str) or not point_name:
            continue
        records.append(
            {
                "pointKind": "target",
                "pointName": point_name,
                "arm": item.get("arm"),
                "cameraId": item.get("camera_id"),
                "mapName": item.get("map_name"),
                "savedAt": item.get("recorded_at") or file_mtime_iso(file_path),
                "pointFilePath": str(file_path),
                "absJointsCount": len(item.get("abs_joints")) if isinstance(item.get("abs_joints"), list) else 0,
            }
        )
    return records


def build_initial_photo_record(
    *,
    paths: QrProjectPaths,
    point_name: str,
    arm: str,
    camera_id: str,
    map_name: str,
    scan_files: Mapping[str, object],
    waypoint_dir: Path,
    stats_path: Path,
    sdk_result: Mapping[str, object],
) -> dict[str, object]:
    stats = read_json_object(stats_path)
    return {
        "pointKind": "initial_photo",
        "pointName": point_name,
        "arm": arm,
        "cameraId": camera_id,
        "mapName": map_name,
        "savedAt": utc_now_iso(),
        "waypointDir": str(waypoint_dir),
        "tfBaselinkTagPath": str(waypoint_dir / "tf_baselink_tag.json"),
        "tfTagEePath": str(waypoint_dir / "tf_tag_ee.json"),
        "jointsPath": str(waypoint_dir / "joints.json") if (waypoint_dir / "joints.json").exists() else None,
        "statsPath": str(stats_path) if stats_path.exists() else None,
        "quality": build_localize_quality(stats),
        "scan": dict(scan_files),
        "sdk": dict(sdk_result),
    }


def list_initial_photo_records(paths: QrProjectPaths) -> list[dict[str, object]]:
    if not paths.waypoints_dir.exists() or not paths.waypoints_dir.is_dir():
        return []
    records: list[dict[str, object]] = []
    manifest = read_manifest(paths.manifest_path)
    point_recording = manifest.get("pointRecording")
    metadata_by_name: dict[str, Mapping[str, Any]] = {}
    if isinstance(point_recording, Mapping):
        raw_initial = point_recording.get("initialPhotoPoints")
        if isinstance(raw_initial, list):
            for item in raw_initial:
                if isinstance(item, Mapping) and isinstance(item.get("pointName"), str):
                    metadata_by_name[str(item["pointName"])] = item
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


def validate_qr_localize_quality(
    stats: Mapping[str, Any],
    *,
    min_markers: int,
    max_reproj_px: float,
    stats_path: Path,
    output_dir: Path,
) -> None:
    """校验初始拍照定位质量，避免低质量 waypoint 进入正式应用产物。

    qr_localize SDK README 建议用 n_markers 和 reproj_px 做第一道门禁；它不能覆盖
    位姿-图像不同步的全部问题，但能挡住明显 PnP/检测质量异常的结果。
    """

    reasons: list[str] = []
    if stats.get("ok") is not True:
        reasons.append("SDK stats.ok 不是 true")

    marker_count = read_finite_number(stats.get("n_markers"))
    if marker_count is None:
        reasons.append("缺少 n_markers")
    elif marker_count < min_markers:
        reasons.append(f"可见 marker {marker_count:g} 个，小于门槛 {min_markers}")

    reproj_px = read_finite_number(stats.get("reproj_px"))
    if reproj_px is None:
        reasons.append("缺少 reproj_px")
    elif reproj_px > max_reproj_px:
        reasons.append(f"重投影误差 {reproj_px:.3f}px，大于门槛 {max_reproj_px:.3f}px")

    if reasons:
        raise QrLocalizeQualityError(
            "初始拍照定位质量不合格: " + "；".join(reasons),
            stats_path=stats_path,
            stats=stats,
            min_markers=min_markers,
            max_reproj_px=max_reproj_px,
            output_dir=output_dir,
        )


def read_finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def update_manifest_after_target(paths: QrProjectPaths, record: Mapping[str, object]) -> None:
    manifest = read_manifest(paths.manifest_path)
    point_recording = dict(manifest.get("pointRecording")) if isinstance(manifest.get("pointRecording"), Mapping) else {}
    targets = list(point_recording.get("targetPoints")) if isinstance(point_recording.get("targetPoints"), list) else []
    targets.append(build_manifest_point_record(record, kind="target"))
    point_recording["targetPoints"] = targets
    manifest.update({"pointRecording": point_recording, "updatedAt": utc_now_iso()})
    paths.manifest_path.write_text(
        json.dumps(to_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def update_manifest_after_initial(paths: QrProjectPaths, record: Mapping[str, object]) -> None:
    manifest = read_manifest(paths.manifest_path)
    point_recording = dict(manifest.get("pointRecording")) if isinstance(manifest.get("pointRecording"), Mapping) else {}
    initial = list(point_recording.get("initialPhotoPoints")) if isinstance(point_recording.get("initialPhotoPoints"), list) else []
    initial.append(build_manifest_point_record(record, kind="initial_photo"))
    point_recording["initialPhotoPoints"] = initial
    manifest.update(
        {
            "projectStatus": "recorded",
            "pointRecording": point_recording,
            "activeWaypointName": record.get("pointName"),
            "updatedAt": utc_now_iso(),
        }
    )
    paths.manifest_path.write_text(
        json.dumps(to_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_manifest_point_record(record: Mapping[str, object], *, kind: str) -> dict[str, object]:
    return {
        "pointKind": kind,
        "pointName": record.get("pointName") or record.get("grasp_name"),
        "arm": record.get("arm"),
        "cameraId": record.get("cameraId") or record.get("camera_id"),
        "mapName": record.get("mapName") or record.get("map_name"),
        "savedAt": record.get("savedAt") or record.get("recorded_at") or utc_now_iso(),
        "waypointDir": record.get("waypointDir"),
        "statsPath": record.get("statsPath"),
        "pointFilePath": record.get("pointFilePath"),
    }


def missing_submit_resources(
    paths: QrProjectPaths,
    target_points: Sequence[Mapping[str, object]],
    initial_points: Sequence[Mapping[str, object]],
) -> list[str]:
    missing: list[str] = []
    if not target_points:
        missing.append("目标点位")
    if not initial_points:
        missing.append("初始拍照点位")
    for file_name in ("scan_abs_pose.json", "scan_abs_joints.json"):
        if not (paths.point_dir / file_name).exists():
            missing.append(file_name)
    if not any((paths.point_dir / file_name).exists() for file_name in ("scan_image.jpeg", "scan_image.jpg", "scan_image.png")):
        missing.append("scan_image")
    return missing


def run_qr_localize_sdk(
    sdk_python: str,
    sdk_path: Path | None,
    point_dir: Path,
    map_yml: Path,
    calibration: Path,
    extrinsic: Path,
    out_dir: Path,
    min_markers: int,
    stats_path: Path,
    timeout_seconds: float,
) -> dict[str, object]:
    args = [
        sdk_python,
        "-m",
        "qr_localize.cli",
        "--point",
        str(point_dir),
        "--map",
        str(map_yml),
        "--calibration",
        str(calibration),
        "--extrinsic",
        str(extrinsic),
        "--out",
        str(out_dir),
        "--min-markers",
        str(min_markers),
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
        raise TimeoutError(f"二维码定位 SDK 执行超时 ({timeout_seconds:.1f}s)") from error

    stdout = append_bounded("", completed.stdout or "")
    stderr = append_bounded("", completed.stderr or "")
    if completed.returncode != 0:
        details = "\n".join(item for item in (stderr.strip(), stdout.strip()) if item)
        message = (
            f"二维码定位 SDK 执行失败，退出码 {completed.returncode}"
            + (f": {details}" if details else "")
        )
        raise QrLocalizeSdkError(
            message,
            return_code=completed.returncode,
            stdout=stdout,
            stderr=stderr,
            stats_path=stats_path,
            stats=read_json_object(stats_path),
        )
    return {
        "python": sdk_python,
        "sdkPath": str(sdk_path) if sdk_path is not None else None,
        "returnCode": completed.returncode,
        "stdout": stdout,
        "stderr": stderr,
    }


def resolve_qr_localize_sdk_path(configured_sdk_path: str) -> Path | None:
    candidates = [
        configured_sdk_path,
        str(Path.cwd() / "sdk" / "qr_localize_sdk"),
        str(Path.cwd().parent / "sdk" / "qr_localize_sdk"),
        str(Path(__file__).resolve().parents[4] / "sdk" / "qr_localize_sdk"),
        str(Path(__file__).resolve().parents[5] / "sdk" / "qr_localize_sdk"),
    ]
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser()
        if (path / "qr_localize" / "cli.py").exists():
            return path
    return None


def summarize_gdk_snapshot(snapshot: Mapping[str, object]) -> dict[str, object]:
    return {
        key: value
        for key, value in snapshot.items()
        if key not in {"absJoints", "absPose", "imageBase64"}
    }


def normalize_quaternion(quaternion: Sequence[float]) -> list[float]:
    norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
    if norm < 1e-12:
        raise RuntimeError("Quaternion norm is zero")
    return [float(value) / norm for value in quaternion]


def quaternion_angle_deg(first: Sequence[float], second: Sequence[float]) -> float:
    first_norm = normalize_quaternion(first)
    second_norm = normalize_quaternion(second)
    dot = abs(sum(a * b for a, b in zip(first_norm, second_norm, strict=True)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def build_stationarity_payload(
    before_pose: Mapping[str, Sequence[float]],
    after_pose: Mapping[str, Sequence[float]],
) -> dict[str, float | bool]:
    before_position = before_pose["position"]
    after_position = after_pose["position"]
    motion_mm = math.sqrt(
        sum((float(after) - float(before)) ** 2 for before, after in zip(before_position, after_position, strict=True))
    ) * 1000.0
    rotation_deg = quaternion_angle_deg(before_pose["orientation"], after_pose["orientation"])
    return {
        "motionMm": motion_mm,
        "rotationDeg": rotation_deg,
        "accepted": True,
    }


def mean_pose(
    before_pose: Mapping[str, Sequence[float]],
    after_pose: Mapping[str, Sequence[float]],
) -> dict[str, list[float]]:
    before_quaternion = list(before_pose["orientation"])
    after_quaternion = list(after_pose["orientation"])
    if sum(a * b for a, b in zip(before_quaternion, after_quaternion, strict=True)) < 0.0:
        after_quaternion = [-value for value in after_quaternion]
    return {
        "position": [
            0.5 * (float(before) + float(after))
            for before, after in zip(before_pose["position"], after_pose["position"], strict=True)
        ],
        "orientation": normalize_quaternion(
            [before + after for before, after in zip(before_quaternion, after_quaternion, strict=True)]
        ),
    }


def file_mtime_iso(path: Path) -> str:
    try:
        stat_target = path if path.is_file() else next(path.iterdir())
        return utc_now_from_timestamp(stat_target.stat().st_mtime)
    except Exception:
        return utc_now_iso()


def utc_now_from_timestamp(timestamp: float) -> str:
    from datetime import datetime, timezone

    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat()


def unavailable_result(
    action: str,
    stage: str,
    error: Exception,
    *,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": GDK_BACKEND,
        "action": action,
        "errorStage": stage,
        "errorCode": "POINT_RECORDING_GDK_UNAVAILABLE",
        "errorType": type(error).__name__,
        "errorMsg": str(error),
        "collectedAt": utc_now_iso(),
    }
    if extra:
        result.update(dict(extra))
    return result


def busy_result(action: str, *, active_purpose: str | None) -> dict[str, object]:
    return {
        "available": False,
        "backend": GDK_BACKEND,
        "action": action,
        "busy": True,
        "errorCode": "ROBOT_BUSY",
        "errorMsg": POINT_RECORDING_BUSY_ERROR_MESSAGE,
        "activePurpose": active_purpose,
        "collectedAt": utc_now_iso(),
    }


def error_payload(action: str, error: Exception) -> dict[str, object]:
    result: dict[str, object] = {
        "available": False,
        "backend": "executor.point_recording",
        "action": action,
        "errorCode": error_code_from_exception(error),
        "errorType": type(error).__name__,
        "errorMsg": str(error),
        "collectedAt": utc_now_iso(),
    }
    if isinstance(error, QrLocalizeSdkError):
        result["sdk"] = {
            "returnCode": error.return_code,
            "stdout": error.stdout,
            "stderr": error.stderr,
            "statsPath": str(error.stats_path),
            "stats": dict(error.stats),
            "quality": build_localize_quality(error.stats),
        }
    if isinstance(error, QrLocalizeQualityError):
        result["sdk"] = {
            "statsPath": str(error.stats_path),
            "stats": dict(error.stats),
            "quality": build_localize_quality(error.stats),
            "qualityGate": {
                "minMarkers": error.min_markers,
                "maxReprojPx": error.max_reproj_px,
                "passed": False,
            },
        }
    return result


def error_code_from_exception(error: Exception) -> str:
    if isinstance(error, FileNotFoundError):
        return "QR_RESOURCE_NOT_FOUND"
    if isinstance(error, TimeoutError):
        return "QR_LOCALIZE_SDK_TIMEOUT"
    if isinstance(error, QrLocalizeSdkError):
        return "QR_LOCALIZE_SDK_FAILED"
    if isinstance(error, QrLocalizeQualityError):
        return "QR_LOCALIZE_QUALITY_FAILED"
    if isinstance(error, ValueError):
        return "POINT_RECORDING_INVALID"
    return "POINT_RECORDING_FAILED"


def write_point_recording_child_progress(
    progress_path: str | None,
    stage: str,
    **payload: object,
) -> None:
    if not progress_path:
        return
    progress = {
        "stage": stage,
        "pid": os.getpid(),
        "updatedAt": utc_now_iso(),
        **payload,
    }
    try:
        Path(progress_path).write_text(
            json.dumps(to_jsonable(progress), ensure_ascii=False),
            encoding="utf-8",
        )
    except Exception:
        # 进度文件只用于现场定位 GDK 阻塞阶段，写失败不能影响点位录制主流程。
        return


def attach_point_recording_child_progress(
    result: dict[str, object],
    progress_path: Path | None,
) -> None:
    progress = read_point_recording_child_progress(progress_path)
    if progress is not None:
        result["pointRecordingChildProgress"] = progress


def read_point_recording_child_progress(progress_path: Path | None) -> dict[str, object] | None:
    if progress_path is None or not progress_path.exists():
        return None
    try:
        decoded = json.loads(progress_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return decoded if isinstance(decoded, dict) else None
