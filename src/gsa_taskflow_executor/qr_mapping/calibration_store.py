"""二维码建图相机标定文件落盘。

GDK 返回的是运行态相机模型；原版二维码定位链路需要固定文件名的内参/外参
JSON。本模块只做格式转换和写文件，不直接调用 GDK。
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from gsa_taskflow_executor.qr_mapping.project_store import QrProjectPaths


class QrCalibrationStoreError(ValueError):
    """相机标定文件转换失败。"""


def save_camera_calibration_files(
    *,
    paths: QrProjectPaths,
    camera_id: str,
    calibration_snapshot: Mapping[str, Any],
) -> dict[str, object]:
    """写入原版兼容内参/外参文件，并返回 manifest 可记录的元数据。"""

    calibration = find_camera_calibration(calibration_snapshot, camera_id)
    intrinsic_path = paths.sensor_root / intrinsic_file_name_from_camera_id(camera_id)
    warnings: list[str] = ["GDK get_camera_intrinsic 未返回相机 SN，已按原版字段写入空字符串"]
    intrinsic_json = build_original_compatible_intrinsic_json(calibration)
    intrinsic_path.write_text(
        json.dumps(intrinsic_json, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    extrinsic_path = save_verified_original_compatible_extrinsic_if_any(
        paths=paths,
        camera_id=camera_id,
        calibration_snapshot=calibration_snapshot,
        warnings=warnings,
    )
    snapshot_warnings = calibration_snapshot.get("warnings")
    if isinstance(snapshot_warnings, list):
        warnings.extend(
            str(item.get("message"))
            for item in snapshot_warnings
            if isinstance(item, Mapping) and item.get("message")
        )

    return {
        "calibrationFilePath": str(intrinsic_path),
        "extrinsicFilePath": str(extrinsic_path) if extrinsic_path else None,
        "calibrationCollectedAt": calibration.get("collectedAt"),
        "calibrationWarnings": warnings,
    }


def find_camera_calibration(
    calibration_snapshot: Mapping[str, Any],
    camera_id: str,
) -> Mapping[str, Any]:
    calibrations = calibration_snapshot.get("calibrations")
    if not isinstance(calibrations, list):
        raise QrCalibrationStoreError("相机标定结果缺少 calibrations")
    for item in calibrations:
        if isinstance(item, Mapping) and item.get("cameraId") == camera_id:
            return item
    raise QrCalibrationStoreError(f"相机标定结果缺少 {camera_id}")


def intrinsic_file_name_from_camera_id(camera_id: str) -> str:
    if camera_id == "hand_left_color":
        return "intrinsic_hand_left_rgb.json"
    if camera_id == "hand_right_color":
        return "intrinsic_hand_right_rgb.json"
    if camera_id == "head_color":
        return "intrinsic_head_rgb.json"
    return f"intrinsic_{safe_file_part(camera_id)}.json"


def extrinsic_file_name_from_camera_id(camera_id: str) -> str | None:
    if camera_id == "hand_left_color":
        return "extrinsic_end_T_hand_left_rgbd.json"
    if camera_id == "hand_right_color":
        return "extrinsic_end_T_hand_right_rgbd.json"
    return None


def expected_verified_extrinsic_type_from_camera_id(camera_id: str) -> str | None:
    if camera_id == "hand_left_color":
        return "kLeftHandRGBDToArmLEndLink"
    if camera_id == "hand_right_color":
        return "kRightHandRGBDToArmREndLink"
    return None


def build_original_compatible_intrinsic_json(calibration: Mapping[str, Any]) -> dict[str, object]:
    coefficients = calibration.get("distortionCoefficients")
    if not isinstance(coefficients, Mapping):
        coefficients = {}
    return {
        "Cx": read_required_number(calibration, "cx"),
        "Cy": read_required_number(calibration, "cy"),
        "Fx": read_required_number(calibration, "fx"),
        "Fy": read_required_number(calibration, "fy"),
        "SN": "",
        "k1": read_optional_number(coefficients, "k1"),
        "k2": read_optional_number(coefficients, "k2"),
        "k3": read_optional_number(coefficients, "k3"),
        "p1": read_optional_number(coefficients, "p1"),
        "p2": read_optional_number(coefficients, "p2"),
    }


def save_verified_original_compatible_extrinsic_if_any(
    *,
    paths: QrProjectPaths,
    camera_id: str,
    calibration_snapshot: Mapping[str, Any],
    warnings: list[str],
) -> Path | None:
    file_name = extrinsic_file_name_from_camera_id(camera_id)
    expected_type = expected_verified_extrinsic_type_from_camera_id(camera_id)
    if not file_name or not expected_type:
        warnings.append(f"相机 {camera_id} 暂无原版兼容外参文件映射，已跳过外参落盘")
        return None

    extrinsics = calibration_snapshot.get("extrinsics")
    if not isinstance(extrinsics, list):
        raise QrCalibrationStoreError(f"相机 {camera_id} 缺少已验证外参 {expected_type}")

    for item in extrinsics:
        if (
            isinstance(item, Mapping)
            and item.get("cameraId") == camera_id
            and item.get("gdkSensorExtrinsicType") == expected_type
            and item.get("directionVerified") is True
        ):
            extrinsic_path = paths.sensor_root / file_name
            extrinsic_path.write_text(
                json.dumps(build_original_compatible_extrinsic_json(item), ensure_ascii=False, indent=2)
                + "\n",
                encoding="utf-8",
            )
            return extrinsic_path

    raise QrCalibrationStoreError(f"相机 {camera_id} 缺少已验证外参 {expected_type}")


def build_original_compatible_extrinsic_json(extrinsic: Mapping[str, Any]) -> dict[str, object]:
    translation = extrinsic.get("translation")
    rotation = extrinsic.get("rotation")
    if not isinstance(translation, Mapping) or not isinstance(rotation, Mapping):
        raise QrCalibrationStoreError("已验证外参缺少 translation/rotation")
    return {
        "rotation": {
            "w": read_required_number(rotation, "w"),
            "x": read_required_number(rotation, "x"),
            "y": read_required_number(rotation, "y"),
            "z": read_required_number(rotation, "z"),
        },
        "translation": {
            "x": read_required_number(translation, "x"),
            "y": read_required_number(translation, "y"),
            "z": read_required_number(translation, "z"),
        },
    }


def build_sdk_calibration_json(intrinsic_file_path: str | Path) -> dict[str, object]:
    """把原版 intrinsic_*.json 转成 SDK 可读的 OpenCV 顺序 JSON。"""

    value = json.loads(Path(intrinsic_file_path).read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise QrCalibrationStoreError("内参文件不是 JSON object")
    intrinsic = value.get("intrinsic") if isinstance(value.get("intrinsic"), Mapping) else value
    if not isinstance(intrinsic, Mapping):
        raise QrCalibrationStoreError("内参文件缺少 intrinsic object")
    return {
        "fx": read_required_number(intrinsic, "fx", aliases=("Fx",)),
        "fy": read_required_number(intrinsic, "fy", aliases=("Fy",)),
        "cx": read_required_number(intrinsic, "cx", aliases=("Cx",)),
        "cy": read_required_number(intrinsic, "cy", aliases=("Cy",)),
        "distortion": normalize_distortion(value),
    }


def normalize_distortion(value: Mapping[str, Any]) -> list[float]:
    raw = value.get("distortion")
    if isinstance(raw, list):
        coefficients = [read_number(item, 0.0) for item in raw]
        return [
            coefficients[0] if len(coefficients) > 0 else 0.0,
            coefficients[1] if len(coefficients) > 1 else 0.0,
            coefficients[2] if len(coefficients) > 2 else 0.0,
            coefficients[3] if len(coefficients) > 3 else 0.0,
            coefficients[4] if len(coefficients) > 4 else 0.0,
        ]
    return [
        read_optional_number(value, "k1"),
        read_optional_number(value, "k2"),
        read_optional_number(value, "p1"),
        read_optional_number(value, "p2"),
        read_optional_number(value, "k3"),
    ]


def read_required_number(
    value: Mapping[str, Any],
    key: str,
    *,
    aliases: tuple[str, ...] = (),
) -> float:
    for candidate in (key, *aliases):
        raw = value.get(candidate)
        if isinstance(raw, (int, float)) and not isinstance(raw, bool):
            return float(raw)
    label = "/".join((key, *aliases))
    raise QrCalibrationStoreError(f"标定字段 {label} 缺失或不是有效数字")


def read_optional_number(value: Mapping[str, Any], key: str) -> float:
    return read_number(value.get(key), 0.0)


def read_number(value: Any, fallback: float) -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return fallback


def safe_file_part(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in value).strip("_") or "camera"
