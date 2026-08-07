from __future__ import annotations

import json
import time
from collections.abc import Mapping
from dataclasses import dataclass
from multiprocessing import get_context
from queue import Empty
from threading import Event, Lock, Thread
from typing import Any
from urllib.parse import urlparse

from gsa_taskflow_executor.gdk.camera_frame import (
    DEFAULT_CAMERA_WARMUP_SECONDS,
    build_camera_frame_snapshot,
    resolve_gdk_camera_type,
)
from gsa_taskflow_executor.gdk.control_probe import initialize_gdk, release_gdk, utc_now_iso
from gsa_taskflow_executor.gdk.readonly import GDK_BACKEND, GDK_MODULE_NAME, to_jsonable
from gsa_taskflow_executor.gdk.session import GdkSessionLease, GdkSessionManager
from gsa_taskflow_executor.mqtt.gateway import (
    create_client,
    default_port_for_scheme,
    import_paho_mqtt,
)
from gsa_taskflow_executor.runtime.config import ExecutorSettings

ACTION_START_CAMERA_CAPTURE = "start_camera_capture"
ACTION_STOP_CAMERA_CAPTURE = "stop_camera_capture"
CAMERA_CAPTURE_FRAME_TYPE = "camera_capture_frame"
CAMERA_CAPTURE_BUSY_ERROR_MESSAGE = "GDK 正在执行控制动作，相机连续采集已拒绝"
DEFAULT_CAPTURE_RATE_FPS = 10
CAPTURE_RATE_FPS_MIN = 1
CAPTURE_RATE_FPS_MAX = 10
STOP_JOIN_TIMEOUT_SECONDS = 6.0
TERMINATE_GRACE_SECONDS = 2.0


@dataclass(frozen=True)
class CameraCaptureStartParams:
    session_id: str
    frame_topic: str
    camera_id: str
    capture_rate_fps: int
    timeout_ms: int


@dataclass
class ActiveCameraCaptureSession:
    session_id: str
    params: CameraCaptureStartParams
    lease: GdkSessionLease
    process: Any
    stop_event: Any
    summary_queue: Any
    started_at: str
    completed: Event
    final_result: dict[str, object] | None = None


class CameraCaptureService:
    """管理 executor 侧连续相机采集会话。

    连续采集会长期占用 GDK/DDS 访问锁；Camera 在子进程中常驻，只初始化一次。
    帧数据直接由子进程发布到 MQTT，避免大图像 base64 通过父子进程 Queue 阻塞。
    """

    def __init__(
        self,
        *,
        session_manager: GdkSessionManager,
        mqtt_broker_url: str,
        mqtt_client_id: str,
        executor_aid: str,
        child_target: Any = None,
    ) -> None:
        self._session_manager = session_manager
        self._mqtt_broker_url = mqtt_broker_url
        self._mqtt_client_id = mqtt_client_id
        self._executor_aid = executor_aid
        self._child_target = child_target or camera_capture_child
        self._lock = Lock()
        self._active: ActiveCameraCaptureSession | None = None

    def start(self, params: CameraCaptureStartParams) -> dict[str, object]:
        params_result = validate_start_params(params)
        if params_result is not None:
            return params_result

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
                    purpose=f"camera_capture:{params.session_id}",
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
            summary_queue = ctx.Queue(maxsize=1)
            process = ctx.Process(
                target=self._child_target,
                args=(
                    summary_queue,
                    stop_event,
                    build_child_payload(
                        params,
                        mqtt_broker_url=self._mqtt_broker_url,
                        mqtt_client_id=self._mqtt_client_id,
                        executor_aid=self._executor_aid,
                    ),
                ),
                name=f"gdk-camera-capture-{params.session_id}",
            )
            process.daemon = True
            process.start()

            session = ActiveCameraCaptureSession(
                session_id=params.session_id,
                params=params,
                lease=lease,
                process=process,
                stop_event=stop_event,
                summary_queue=summary_queue,
                started_at=utc_now_iso(),
                completed=Event(),
            )
            self._active = session
            Thread(
                target=self._monitor_session,
                args=(session,),
                name=f"camera-capture-monitor-{params.session_id}",
                daemon=True,
            ).start()

        return {
            "started": True,
            "backend": GDK_BACKEND,
            "action": ACTION_START_CAMERA_CAPTURE,
            "sessionId": params.session_id,
            "frameTopic": params.frame_topic,
            "cameraId": params.camera_id,
            "captureRateFps": params.capture_rate_fps,
            "timeoutMs": params.timeout_ms,
            "startedAt": session.started_at,
            "gdk_parent_lock": lease.to_payload(),
        }

    def stop(self, session_id: str) -> dict[str, object]:
        with self._lock:
            session = self._active
            if session is None or session.session_id != session_id:
                return {
                    "stopped": False,
                    "backend": GDK_BACKEND,
                    "action": ACTION_STOP_CAMERA_CAPTURE,
                    "sessionId": session_id,
                    "errorCode": "CAMERA_CAPTURE_NOT_FOUND",
                    "errorMsg": "未找到正在运行的相机采集会话",
                }

        session.stop_event.set()
        session.process.join(STOP_JOIN_TIMEOUT_SECONDS)
        forced = False
        killed = False
        if session.process.is_alive():
            forced = True
            # 子进程长期持有 GDK/DDS 访问权；terminate 无效时必须 kill，
            # 否则父进程锁不会释放，后续 Taskflow 控制和只读查询都会被挡住。
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
            return {"called": False, "success": True, "reason": "no_active_camera_capture"}
        result = self.stop(active.session_id)
        return {"called": True, "success": result.get("stopped") is True, "result": result}

    def _monitor_session(self, session: ActiveCameraCaptureSession) -> None:
        session.process.join()
        result = read_summary_queue(session.summary_queue) or build_session_exit_result(session)
        result.setdefault("stopped", True)
        result.setdefault("sessionId", session.session_id)
        result.setdefault("backend", GDK_BACKEND)
        result.setdefault("action", ACTION_STOP_CAMERA_CAPTURE)
        result["subprocess"] = {
            "policy": "camera_capture_session",
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


def camera_capture_child(
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
    started_at = utc_now_iso()
    frames_captured = 0
    frames_published = 0
    last_error: dict[str, object] | None = None
    agibot_gdk = None
    camera = None
    mqtt_client = None
    gdk_initialized = False
    init_result: dict[str, object] = {"called": False, "success": True, "return": None}

    try:
        mqtt_client = connect_frame_mqtt_client(payload)
        agibot_gdk = __import__(GDK_MODULE_NAME)
        init_result = initialize_gdk(agibot_gdk)
        if init_result.get("called") is True and init_result.get("success") is not True:
            raise RuntimeError("agibot_gdk.gdk_init() did not return success")

        gdk_initialized = bool(init_result.get("called"))
        gdk_camera_type = resolve_gdk_camera_type(agibot_gdk, camera_id)
        camera = agibot_gdk.Camera()
        time.sleep(DEFAULT_CAMERA_WARMUP_SECONDS)
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
                frame_payload = build_frame_payload(
                    snapshot,
                    session_id=session_id,
                    frame_index=frames_captured,
                    capture_rate_fps=capture_rate_fps,
                    executor_aid=executor_aid,
                )
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
            "stage": "camera_capture_child",
            "type": type(error).__name__,
            "message": str(error),
            "at": utc_now_iso(),
        }
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
        put_summary_queue(
            summary_queue,
            {
                "stopped": True,
                "backend": GDK_BACKEND,
                "action": ACTION_STOP_CAMERA_CAPTURE,
                "sessionId": session_id,
                "cameraId": camera_id,
                "captureRateFps": capture_rate_fps,
                "framesCaptured": frames_captured,
                "framesPublished": frames_published,
                "startedAt": started_at,
                "stoppedAt": utc_now_iso(),
                "stopReason": "requested" if stop_event.is_set() else "child_finished",
                "lastError": last_error,
                "gdk_init": init_result,
                "gdk_release": release_result,
            },
        )


def build_child_payload(
    params: CameraCaptureStartParams,
    *,
    mqtt_broker_url: str,
    mqtt_client_id: str,
    executor_aid: str,
) -> dict[str, object]:
    return {
        "sessionId": params.session_id,
        "frameTopic": params.frame_topic,
        "cameraId": params.camera_id,
        "captureRateFps": params.capture_rate_fps,
        "timeoutMs": params.timeout_ms,
        "mqttBrokerUrl": mqtt_broker_url,
        "mqttClientId": f"{mqtt_client_id}-camera-{params.session_id}",
        "executorAid": executor_aid,
    }


def build_frame_payload(
    snapshot: Mapping[str, object],
    *,
    session_id: str,
    frame_index: int,
    capture_rate_fps: int,
    executor_aid: str,
) -> dict[str, object]:
    return {
        "type": CAMERA_CAPTURE_FRAME_TYPE,
        "sessionId": session_id,
        "frameIndex": frame_index,
        "executorAid": executor_aid,
        "captureRateFps": capture_rate_fps,
        "cameraId": snapshot.get("cameraId"),
        "mimeType": snapshot.get("mimeType"),
        "imageBase64": snapshot.get("imageBase64"),
        "width": snapshot.get("width"),
        "height": snapshot.get("height"),
        "encoding": snapshot.get("encoding"),
        "timestampNs": snapshot.get("timestampNs"),
        "capturedAt": snapshot.get("collectedAt"),
        "raw": snapshot.get("raw"),
    }


def connect_frame_mqtt_client(payload: Mapping[str, object]) -> Any:
    broker_url = str(payload["mqttBrokerUrl"])
    parsed = urlparse(broker_url)
    if not parsed.hostname:
        raise RuntimeError("MQTT_BROKER_URL missing host")
    mqtt = import_paho_mqtt()
    settings = ExecutorSettings(mqtt_client_id=str(payload["mqttClientId"]))
    client = create_client(
        mqtt,
        settings,
        parsed.scheme,
    )
    port = parsed.port or default_port_for_scheme(parsed.scheme)
    client.connect(parsed.hostname, port, keepalive=60)
    client.loop_start()
    return client

def read_summary_queue(summary_queue: Any) -> dict[str, object] | None:
    try:
        result = summary_queue.get(timeout=0.2)
    except Empty:
        return None
    return dict(result) if isinstance(result, Mapping) else None


def put_summary_queue(summary_queue: Any, payload: Mapping[str, object]) -> None:
    """向父进程回传小型 summary，不让 Queue feeder 阻塞子进程退出。"""

    try:
        summary_queue.put(dict(payload), timeout=1.0)
        cancel_join_thread = getattr(summary_queue, "cancel_join_thread", None)
        if callable(cancel_join_thread):
            cancel_join_thread()
    except Exception:
        pass


def validate_start_params(params: CameraCaptureStartParams) -> dict[str, object] | None:
    if not params.session_id.strip():
        return refused_result("INVALID_REQUEST", "相机连续采集缺少 sessionId")
    if not params.frame_topic.strip():
        return refused_result("INVALID_REQUEST", "相机连续采集缺少 frameTopic")
    if not (CAPTURE_RATE_FPS_MIN <= params.capture_rate_fps <= CAPTURE_RATE_FPS_MAX):
        return refused_result(
            "INVALID_REQUEST",
            f"采集频率必须在 {CAPTURE_RATE_FPS_MIN}~{CAPTURE_RATE_FPS_MAX} FPS",
        )
    if params.timeout_ms <= 0:
        return refused_result("INVALID_REQUEST", "相机 GDK timeoutMs 必须为正整数")
    return None


def refused_result(code: str, message: str) -> dict[str, object]:
    return {
        "started": False,
        "backend": GDK_BACKEND,
        "action": ACTION_START_CAMERA_CAPTURE,
        "errorCode": code,
        "errorMsg": message,
    }


def unavailable_result(stage: str, error: Exception) -> dict[str, object]:
    return {
        "started": False,
        "backend": GDK_BACKEND,
        "action": ACTION_START_CAMERA_CAPTURE,
        "errorStage": stage,
        "errorCode": "GDK_CAMERA_CAPTURE_UNAVAILABLE",
        "errorType": type(error).__name__,
        "errorMsg": str(error),
    }


def busy_result(
    *,
    active_session_id: str | None,
    active_purpose: str | None,
) -> dict[str, object]:
    return {
        "started": False,
        "backend": GDK_BACKEND,
        "action": ACTION_START_CAMERA_CAPTURE,
        "busy": True,
        "errorCode": "ROBOT_BUSY",
        "errorMsg": CAMERA_CAPTURE_BUSY_ERROR_MESSAGE,
        "activeSessionId": active_session_id,
        "activePurpose": active_purpose,
    }


def build_session_exit_result(session: ActiveCameraCaptureSession) -> dict[str, object]:
    return {
        "stopped": True,
        "backend": GDK_BACKEND,
        "action": ACTION_STOP_CAMERA_CAPTURE,
        "sessionId": session.session_id,
        "cameraId": session.params.camera_id,
        "captureRateFps": session.params.capture_rate_fps,
        "framesCaptured": 0,
        "framesPublished": 0,
        "startedAt": session.started_at,
        "stoppedAt": utc_now_iso(),
        "stopReason": "process_exited_without_summary",
    }
