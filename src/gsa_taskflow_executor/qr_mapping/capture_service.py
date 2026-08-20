"""二维码建图远端图片采集服务。

方案 2 中客户端只发业务参数，executor 在 Ubuntu/G2 环境里完成 GDK 相机读取、
标定落盘、图片保存和 MQTT 预览推送。这样 Mac/Windows 客户端不需要本机文件
系统或 GDK 环境，也不会生成无法被后续应用流程使用的本地产物。
"""

from __future__ import annotations

import base64
import json
import time
from collections.abc import Mapping
from dataclasses import dataclass
from multiprocessing import get_context
from pathlib import Path
from queue import Empty
from threading import Event, Lock, Thread
from typing import Any
from uuid import uuid4

from gsa_taskflow_executor.gdk.camera_calibration import collect_camera_calibration
from gsa_taskflow_executor.gdk.camera_capture import (
    CAPTURE_RATE_FPS_MAX,
    CAPTURE_RATE_FPS_MIN,
    STOP_JOIN_TIMEOUT_SECONDS,
    TERMINATE_GRACE_SECONDS,
    build_frame_payload,
    connect_frame_mqtt_client,
    put_summary_queue,
    read_summary_queue,
)
from gsa_taskflow_executor.gdk.camera_frame import (
    DEFAULT_CAMERA_WARMUP_SECONDS,
    build_camera_frame_snapshot,
    resolve_gdk_camera_type,
    validate_camera_id,
)
from gsa_taskflow_executor.gdk.control_probe import initialize_gdk, release_gdk, utc_now_iso
from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from gsa_taskflow_executor.gdk.session import GdkSessionLease, GdkSessionManager
from gsa_taskflow_executor.qr_mapping.calibration_store import save_camera_calibration_files
from gsa_taskflow_executor.qr_mapping.project_store import (
    QrProjectPaths,
    QrProjectStore,
    paths_to_payload,
)

ACTION_START_QR_CAPTURE = "start_qr_capture"
ACTION_STOP_QR_CAPTURE = "stop_qr_capture"
QR_CAPTURE_FRAME_TYPE = "qr_capture_frame"
QR_CAPTURE_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，二维码建图采集已拒绝"
DEFAULT_QR_CAPTURE_RATE_FPS = 10
DEFAULT_QR_CAMERA_TIMEOUT_MS = 3000
START_READY_TIMEOUT_SECONDS = 30.0


@dataclass(frozen=True)
class QrCaptureStartParams:
    session_id: str
    frame_topic: str
    robot_serial: str
    project_name: str
    camera_id: str
    marker_type: str
    marker_size_meters: float
    capture_rate_fps: int
    timeout_ms: int
    reset_existing: bool = False


@dataclass
class ActiveQrCaptureSession:
    session_id: str
    params: QrCaptureStartParams
    lease: GdkSessionLease
    process: Any
    stop_event: Any
    summary_queue: Any
    started_at: str
    completed: Event
    final_result: dict[str, object] | None = None


class QrCaptureService:
    """管理二维码建图采集会话。同一时刻只允许一个 QR 采集占用 GDK。"""

    def __init__(
        self,
        *,
        session_manager: GdkSessionManager,
        project_store: QrProjectStore,
        mqtt_broker_url: str,
        mqtt_client_id: str,
        executor_aid: str,
        child_target: Any = None,
    ) -> None:
        self._session_manager = session_manager
        self._project_store = project_store
        self._mqtt_broker_url = mqtt_broker_url
        self._mqtt_client_id = mqtt_client_id
        self._executor_aid = executor_aid
        self._child_target = child_target or qr_capture_child
        self._lock = Lock()
        self._active: ActiveQrCaptureSession | None = None

    def start(self, params: QrCaptureStartParams) -> dict[str, object]:
        validation = validate_start_params(params)
        if validation is not None:
            return validation

        try:
            paths = self._project_store.build_paths(
                robot_serial=params.robot_serial,
                project_name=params.project_name,
            )
        except Exception as error:
            return refused_result("QR_PROJECT_INVALID", str(error))

        if params.reset_existing:
            return refused_result("RESET_EXISTING_UNSUPPORTED", "项目重采集归档策略尚未实现，已拒绝覆盖")
        if self._project_store.has_project_data(
            robot_serial=params.robot_serial,
            project_name=params.project_name,
        ):
            return refused_result(
                "PROJECT_ALREADY_EXISTS",
                "该机器人 SN 和项目名称已存在采集/建图数据，请更换项目名称",
                extra=paths_to_payload(paths),
            )

        with self._lock:
            self._clear_finished_session_locked()
            if self._active is not None:
                return busy_result(
                    active_session_id=self._active.session_id,
                    active_purpose=self._session_manager.active_purpose,
                )

            try:
                lease = self._session_manager.acquire(
                    blocking=False,
                    initialize=False,
                    purpose=f"qr_capture:{params.session_id}",
                )
            except Exception as error:
                return unavailable_result("gdk_session_acquire", error)
            if lease is None:
                return busy_result(
                    active_session_id=None,
                    active_purpose=self._session_manager.active_purpose,
                )

            ctx = get_context("spawn")
            stop_event = ctx.Event()
            ready_queue = ctx.Queue(maxsize=1)
            summary_queue = ctx.Queue(maxsize=1)
            process = ctx.Process(
                target=self._child_target,
                args=(
                    ready_queue,
                    summary_queue,
                    stop_event,
                    build_child_payload(
                        params,
                        paths=paths,
                        mqtt_broker_url=self._mqtt_broker_url,
                        mqtt_client_id=self._mqtt_client_id,
                        executor_aid=self._executor_aid,
                    ),
                ),
                name=f"gdk-qr-capture-{params.session_id}",
            )
            process.daemon = True
            process.start()

            ready_result = read_ready_queue(
                ready_queue,
                timeout_seconds=max(
                    START_READY_TIMEOUT_SECONDS,
                    params.timeout_ms / 1000.0 + DEFAULT_CAMERA_WARMUP_SECONDS + 8.0,
                ),
            )
            if ready_result is None:
                terminate_process(process)
                lease.release()
                return refused_result(
                    "QR_CAPTURE_START_TIMEOUT",
                    "二维码建图采集启动超时，子进程未完成相机/标定初始化",
                    extra={"sessionId": params.session_id},
                )
            if ready_result.get("started") is not True:
                terminate_process(process)
                lease.release()
                return dict(ready_result)

            session = ActiveQrCaptureSession(
                session_id=params.session_id,
                params=params,
                lease=lease,
                process=process,
                stop_event=stop_event,
                summary_queue=summary_queue,
                started_at=str(ready_result.get("startedAt") or utc_now_iso()),
                completed=Event(),
            )
            self._active = session
            Thread(
                target=self._monitor_session,
                args=(session,),
                name=f"qr-capture-monitor-{params.session_id}",
                daemon=True,
            ).start()

        ready_result["gdk_parent_lock"] = lease.to_payload()
        return dict(ready_result)

    def stop(self, session_id: str) -> dict[str, object]:
        with self._lock:
            session = self._active
            if session is None or session.session_id != session_id:
                return {
                    "stopped": False,
                    "backend": GDK_BACKEND,
                    "action": ACTION_STOP_QR_CAPTURE,
                    "sessionId": session_id,
                    "errorCode": "QR_CAPTURE_NOT_FOUND",
                    "errorMsg": "未找到正在运行的二维码建图采集会话",
                }

        session.stop_event.set()
        session.process.join(STOP_JOIN_TIMEOUT_SECONDS)
        forced = False
        killed = False
        if session.process.is_alive():
            forced = True
            # QR 采集子进程长期持有 GDK/DDS 访问权；强制退出是释放父进程互斥锁的兜底。
            session.process.terminate()
            session.process.join(TERMINATE_GRACE_SECONDS)
            if session.process.is_alive():
                killed = True
                session.process.kill()
                session.process.join(TERMINATE_GRACE_SECONDS)

        session.completed.wait(1.0)
        final_result = session.final_result or build_session_exit_result(session)
        final_result["stopForced"] = forced
        final_result["stopKilled"] = killed
        return final_result

    def shutdown(self) -> dict[str, object]:
        with self._lock:
            active = self._active
        if active is None:
            return {"called": False, "success": True, "reason": "no_active_qr_capture"}
        result = self.stop(active.session_id)
        return {"called": True, "success": result.get("stopped") is True, "result": result}

    def _monitor_session(self, session: ActiveQrCaptureSession) -> None:
        session.process.join()
        result = read_summary_queue(session.summary_queue) or build_session_exit_result(session)
        result.setdefault("stopped", True)
        result.setdefault("sessionId", session.session_id)
        result.setdefault("backend", GDK_BACKEND)
        result.setdefault("action", ACTION_STOP_QR_CAPTURE)
        result["subprocess"] = {
            "policy": "qr_capture_session",
            "pid": session.process.pid,
            "exitcode": session.process.exitcode,
            "timed_out": False,
            "terminated": session.process.exitcode not in {0, None},
        }
        result["gdk_parent_lock"] = session.lease.to_payload()
        session.final_result = result
        session.lease.release()
        with self._lock:
            if self._active is session:
                self._active = None
        session.completed.set()

    def _clear_finished_session_locked(self) -> None:
        if self._active is not None and self._active.completed.is_set():
            self._active = None


def qr_capture_child(
    ready_queue: Any,
    summary_queue: Any,
    stop_event: Any,
    payload: Mapping[str, object],
) -> None:
    session_id = str(payload["sessionId"])
    camera_id = str(payload["cameraId"])
    capture_rate_fps = int(str(payload["captureRateFps"]))
    timeout_ms = int(str(payload["timeoutMs"]))
    frame_topic = str(payload["frameTopic"])
    executor_aid = str(payload["executorAid"])
    paths = paths_from_payload(payload)
    started_at = utc_now_iso()
    frames_captured = 0
    frames_published = 0
    frames_saved = 0
    last_error: dict[str, object] | None = None
    agibot_gdk = None
    camera = None
    mqtt_client = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}
    ready_sent = False
    capture_started = False

    try:
        ensure_project_layout(paths)
        mqtt_client = connect_frame_mqtt_client(payload)
        agibot_gdk = __import__(GDK_MODULE_NAME)
        init_result = initialize_gdk(agibot_gdk)
        if init_result.get("called") is True and init_result.get("success") is not True:
            raise RuntimeError("agibot_gdk.gdk_init() did not return success")
        gdk_initialized = bool(init_result.get("called"))

        calibration_snapshot = collect_camera_calibration(
            agibot_gdk=agibot_gdk,
            camera_ids=(camera_id,),
            timeout_ms=timeout_ms,
            include_extrinsics=True,
            warmup_seconds=DEFAULT_CAMERA_WARMUP_SECONDS,
        )
        if calibration_snapshot.get("available") is not True:
            raise RuntimeError(str(calibration_snapshot.get("errorMsg") or "相机标定读取失败"))
        calibration = save_camera_calibration_files(
            paths=paths,
            camera_id=camera_id,
            calibration_snapshot=calibration_snapshot,
        )
        write_project_manifest(
            paths=paths,
            payload=payload,
            status="capturing",
            calibration=calibration,
            active_map_name=None,
        )

        gdk_camera_type = resolve_gdk_camera_type(agibot_gdk, camera_id)
        camera = agibot_gdk.Camera()
        time.sleep(DEFAULT_CAMERA_WARMUP_SECONDS)
        put_summary_queue(
            ready_queue,
            {
                "started": True,
                "backend": GDK_BACKEND,
                "action": ACTION_START_QR_CAPTURE,
                "sessionId": session_id,
                "frameTopic": frame_topic,
                "robotSerial": payload["robotSerial"],
                "projectName": payload["projectName"],
                "cameraId": camera_id,
                "markerType": payload["markerType"],
                "markerSizeMeters": payload["markerSizeMeters"],
                "captureRateFps": capture_rate_fps,
                "timeoutMs": timeout_ms,
                "startedAt": started_at,
                "calibrationSaved": True,
                **calibration,
                **paths_to_payload(paths),
            },
        )
        ready_sent = True
        capture_started = True

        interval_seconds = 1.0 / capture_rate_fps
        while not stop_event.is_set():
            loop_started_at = time.monotonic()
            try:
                image = camera.get_latest_image(gdk_camera_type, float(timeout_ms))
                if image is None:
                    raise RuntimeError("camera.get_latest_image() returned None")
                snapshot = build_camera_frame_snapshot(
                    image,
                    camera_id=camera_id,
                    gdk_camera_type=str(gdk_camera_type),
                )
                frames_captured += 1
                captured_frame = save_captured_frame(
                    paths=paths,
                    payload=payload,
                    snapshot=snapshot,
                    session_id=session_id,
                    frame_index=frames_captured,
                    capture_rate_fps=capture_rate_fps,
                )
                frames_saved += 1
                frame_payload = build_frame_payload(
                    snapshot,
                    session_id=session_id,
                    frame_index=frames_captured,
                    capture_rate_fps=capture_rate_fps,
                    executor_aid=executor_aid,
                )
                frame_payload["type"] = QR_CAPTURE_FRAME_TYPE
                frame_payload["robotSerial"] = payload["robotSerial"]
                frame_payload["projectName"] = payload["projectName"]
                frame_payload["capturedFrame"] = captured_frame
                publish_info = mqtt_client.publish(
                    frame_topic,
                    json.dumps(to_jsonable(frame_payload), ensure_ascii=False),
                    qos=0,
                )
                publish_info.wait_for_publish(timeout=2.0)
                frames_published += 1
            except Exception as error:
                last_error = {
                    "stage": "capture_frame",
                    "type": type(error).__name__,
                    "message": str(error),
                    "at": utc_now_iso(),
                }

            elapsed = time.monotonic() - loop_started_at
            sleep_seconds = interval_seconds - elapsed
            if sleep_seconds > 0:
                stop_event.wait(sleep_seconds)
    except Exception as error:
        last_error = {
            "stage": "qr_capture_child",
            "type": type(error).__name__,
            "message": str(error),
            "at": utc_now_iso(),
        }
        if not ready_sent:
            put_summary_queue(
                ready_queue,
                unavailable_result(
                    "qr_capture_child",
                    error,
                    extra={
                        "sessionId": session_id,
                        "robotSerial": payload.get("robotSerial"),
                        "projectName": payload.get("projectName"),
                    },
                ),
            )
            ready_sent = True
    finally:
        if camera is not None:
            close_camera = getattr(camera, "close_camera", None)
            if callable(close_camera):
                try:
                    close_camera()
                except Exception:
                    pass
        release_result: dict[str, object] = {"called": False, "success": True, "return": None}
        if agibot_gdk is not None and gdk_initialized:
            release_result = release_gdk(agibot_gdk)
        if mqtt_client is not None:
            try:
                mqtt_client.loop_stop()
                mqtt_client.disconnect()
            except Exception:
                pass
        if capture_started:
            try:
                write_project_manifest(
                    paths=paths,
                    payload=payload,
                    status="captured",
                    calibration=None,
                    active_map_name=None,
                )
            except Exception:
                pass
        put_summary_queue(
            summary_queue,
            {
                "stopped": True,
                "backend": GDK_BACKEND,
                "action": ACTION_STOP_QR_CAPTURE,
                "sessionId": session_id,
                "robotSerial": payload.get("robotSerial"),
                "projectName": payload.get("projectName"),
                "cameraId": camera_id,
                "captureRateFps": capture_rate_fps,
                "framesCaptured": frames_captured,
                "framesPublished": frames_published,
                "framesSaved": frames_saved,
                "imageCount": frames_saved,
                "startedAt": started_at,
                "stoppedAt": utc_now_iso(),
                "stopReason": "requested" if stop_event.is_set() else "child_finished",
                "lastError": last_error,
                "gdk_init": init_result,
                "gdk_release": release_result,
                **paths_to_payload(paths),
            },
        )


def build_child_payload(
    params: QrCaptureStartParams,
    *,
    paths: QrProjectPaths,
    mqtt_broker_url: str,
    mqtt_client_id: str,
    executor_aid: str,
) -> dict[str, object]:
    return {
        "sessionId": params.session_id,
        "frameTopic": params.frame_topic,
        "robotSerial": params.robot_serial,
        "projectName": params.project_name,
        "cameraId": params.camera_id,
        "markerType": params.marker_type,
        "markerSizeMeters": params.marker_size_meters,
        "captureRateFps": params.capture_rate_fps,
        "timeoutMs": params.timeout_ms,
        "mqttBrokerUrl": mqtt_broker_url,
        "mqttClientId": f"{mqtt_client_id}-qr-capture-{params.session_id}",
        "executorAid": executor_aid,
        **paths_to_payload(paths),
    }


def save_captured_frame(
    *,
    paths: QrProjectPaths,
    payload: Mapping[str, object],
    snapshot: Mapping[str, object],
    session_id: str,
    frame_index: int,
    capture_rate_fps: int,
) -> dict[str, object]:
    captured_at = str(snapshot.get("collectedAt") or utc_now_iso())
    mime_type = str(snapshot.get("mimeType") or "image/jpeg")
    image_base64 = str(snapshot.get("imageBase64") or "")
    image_bytes = base64.b64decode(strip_data_url_prefix(image_base64), validate=False)
    if not image_bytes:
        raise RuntimeError("相机图像为空，不能保存")
    extension = extension_from_mime_type(mime_type)
    file_name_base = build_image_file_name_base(snapshot.get("timestampNs"), captured_at)
    file_path = resolve_unique_file_path(paths.images_dir, file_name_base, extension)
    file_path.write_bytes(image_bytes)
    frame = {
        "id": str(uuid4()),
        "sessionId": session_id,
        "frameIndex": frame_index,
        "robotSerial": payload["robotSerial"],
        "projectName": payload["projectName"],
        "cameraId": snapshot.get("cameraId"),
        "markerType": payload["markerType"],
        "markerSizeMeters": payload["markerSizeMeters"],
        "captureRateFps": capture_rate_fps,
        "fileName": file_path.name,
        "filePath": str(file_path),
        "remotePath": str(file_path),
        "mimeType": mime_type,
        "width": snapshot.get("width"),
        "height": snapshot.get("height"),
        "timestampNs": snapshot.get("timestampNs"),
        "capturedAt": captured_at,
    }
    with paths.captures_file.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(to_jsonable(frame), ensure_ascii=False) + "\n")
    return frame


def write_project_manifest(
    *,
    paths: QrProjectPaths,
    payload: Mapping[str, object],
    status: str,
    calibration: Mapping[str, object] | None,
    active_map_name: str | None,
) -> None:
    manifest = read_manifest(paths.manifest_path)
    manifest.update(
        {
            "schemaVersion": 1,
            "format": "gsa_qr_mapping_dataset_v1",
            "compatibility": "agibot_qr_pose_skill_conf",
            "projectStatus": status,
            "robotSerial": payload["robotSerial"],
            "projectName": payload["projectName"],
            "cameraId": payload["cameraId"],
            "markerType": payload["markerType"],
            "markerSizeMeters": payload["markerSizeMeters"],
            "captureRateFps": payload["captureRateFps"],
            "directories": {
                "sensor": str(paths.sensor_root),
                "images": str(paths.images_dir),
                "maps": str(paths.maps_dir),
                "point": str(paths.point_dir),
                "waypoints": str(paths.waypoints_dir),
            },
            "updatedAt": utc_now_iso(),
        }
    )
    if calibration is not None:
        manifest["calibration"] = {key: value for key, value in calibration.items() if value is not None}
    if active_map_name is not None:
        manifest["activeMapName"] = active_map_name
    paths.manifest_path.write_text(
        json.dumps(to_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def read_manifest(path: Path) -> dict[str, object]:
    if not path.exists():
        return {}
    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return dict(decoded) if isinstance(decoded, Mapping) else {}


def ensure_project_layout(paths: QrProjectPaths) -> None:
    for directory in (
        paths.sensor_root,
        paths.images_dir,
        paths.maps_dir,
        paths.point_dir,
        paths.waypoints_dir,
    ):
        directory.mkdir(parents=True, exist_ok=True)


def paths_from_payload(payload: Mapping[str, object]) -> QrProjectPaths:
    return QrProjectPaths(
        data_root=Path(str(payload["dataRoot"])),
        robot_root=Path(str(payload["robotRoot"])),
        sensor_root=Path(str(payload["sensorRoot"])),
        project_root=Path(str(payload["projectRoot"])),
        images_dir=Path(str(payload["imagesDir"])),
        maps_dir=Path(str(payload["mapsDir"])),
        point_dir=Path(str(payload["pointDir"])),
        waypoints_dir=Path(str(payload["waypointsDir"])),
        captures_file=Path(str(payload["capturesFile"])),
        manifest_path=Path(str(payload["manifestPath"])),
    )


def validate_start_params(params: QrCaptureStartParams) -> dict[str, object] | None:
    if not params.session_id.strip():
        return refused_result("INVALID_REQUEST", "二维码建图采集缺少 sessionId")
    if not params.frame_topic.strip():
        return refused_result("INVALID_REQUEST", "二维码建图采集缺少 frameTopic")
    if not params.robot_serial.strip():
        return refused_result("INVALID_REQUEST", "二维码建图采集缺少 robotSerial")
    if not params.project_name.strip():
        return refused_result("INVALID_REQUEST", "二维码建图采集缺少 projectName")
    if not params.marker_type.strip():
        return refused_result("INVALID_REQUEST", "二维码建图采集缺少 markerType")
    if params.marker_size_meters <= 0:
        return refused_result("INVALID_REQUEST", "二维码尺寸必须大于 0")
    if not (CAPTURE_RATE_FPS_MIN <= params.capture_rate_fps <= CAPTURE_RATE_FPS_MAX):
        return refused_result("INVALID_REQUEST", "采集频率必须在 1~10 FPS")
    if params.timeout_ms <= 0:
        return refused_result("INVALID_REQUEST", "相机 GDK timeoutMs 必须为正整数")
    camera_result = validate_camera_id(params.camera_id)
    if camera_result is not None:
        return refused_result(
            "INVALID_REQUEST",
            str(camera_result.get("errorMsg") or "不支持的相机"),
            extra=camera_result,
        )
    return None


def read_ready_queue(ready_queue: Any, *, timeout_seconds: float) -> dict[str, object] | None:
    try:
        result = ready_queue.get(timeout=timeout_seconds)
    except Empty:
        return None
    return dict(result) if isinstance(result, Mapping) else None


def terminate_process(process: Any) -> None:
    if not process.is_alive():
        process.join(0.2)
        return
    process.terminate()
    process.join(TERMINATE_GRACE_SECONDS)
    if process.is_alive():
        process.kill()
        process.join(TERMINATE_GRACE_SECONDS)


def strip_data_url_prefix(value: str) -> str:
    comma_index = value.find(",")
    return value[comma_index + 1 :] if value.startswith("data:") and comma_index >= 0 else value


def extension_from_mime_type(mime_type: str) -> str:
    if mime_type == "image/png":
        return "png"
    if mime_type == "image/bmp":
        return "bmp"
    return "jpg"


def build_image_file_name_base(timestamp_ns: object, captured_at: str) -> str:
    timestamp_ms = timestamp_ns_to_milliseconds(timestamp_ns)
    if timestamp_ms:
        return timestamp_ms
    try:
        return str(int(time.mktime(time.strptime(captured_at[:19], "%Y-%m-%dT%H:%M:%S")) * 1000))
    except Exception:
        return "".join(ch for ch in captured_at if ch.isalnum())[:20] or "capture"


def timestamp_ns_to_milliseconds(value: object) -> str | None:
    if isinstance(value, int) and value > 0:
        return str(value // 1_000_000)
    if isinstance(value, str) and value.isdigit():
        return str(int(value) // 1_000_000)
    return None


def resolve_unique_file_path(directory: Path, file_name_base: str, extension: str) -> Path:
    candidate = directory / f"{file_name_base}.{extension}"
    index = 1
    while candidate.exists():
        candidate = directory / f"{file_name_base}_{index}.{extension}"
        index += 1
    return candidate


def refused_result(
    code: str,
    message: str,
    *,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "started": False,
        "backend": GDK_BACKEND,
        "action": ACTION_START_QR_CAPTURE,
        "errorCode": code,
        "errorMsg": message,
    }
    if extra:
        result.update(dict(extra))
    return result


def unavailable_result(
    stage: str,
    error: Exception,
    *,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "started": False,
        "backend": GDK_BACKEND,
        "action": ACTION_START_QR_CAPTURE,
        "errorStage": stage,
        "errorCode": "QR_CAPTURE_UNAVAILABLE",
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }
    if extra:
        result.update(dict(extra))
    return result


def busy_result(
    *,
    active_session_id: str | None,
    active_purpose: str | None,
) -> dict[str, object]:
    return {
        "started": False,
        "backend": GDK_BACKEND,
        "action": ACTION_START_QR_CAPTURE,
        "busy": True,
        "errorCode": "ROBOT_BUSY",
        "errorMsg": QR_CAPTURE_BUSY_ERROR_MESSAGE,
        "activeSessionId": active_session_id,
        "activePurpose": active_purpose,
    }


def build_session_exit_result(session: ActiveQrCaptureSession) -> dict[str, object]:
    return {
        "stopped": True,
        "backend": GDK_BACKEND,
        "action": ACTION_STOP_QR_CAPTURE,
        "sessionId": session.session_id,
        "robotSerial": session.params.robot_serial,
        "projectName": session.params.project_name,
        "cameraId": session.params.camera_id,
        "captureRateFps": session.params.capture_rate_fps,
        "framesCaptured": 0,
        "framesPublished": 0,
        "framesSaved": 0,
        "startedAt": session.started_at,
        "stoppedAt": utc_now_iso(),
        "stopReason": "process_exited_without_summary",
    }
