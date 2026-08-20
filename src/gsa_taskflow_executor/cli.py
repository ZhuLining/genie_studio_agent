"""gsa-taskflow-executor CLI 入口。

--listen 模式下的数据流::

    MQTT Broker → MqttGateway(paho 网络循环)
      ├─ taskflow 消息 → taskflow_queue(单 worker FIFO) → process_taskflow_message()
      │    解析 YAML → 校验技能白名单 → TaskflowScheduler.run() → 上报状态
      ├─ cancel 消息 → handle_taskflow_cancel_message() (绕过队列，直接中断)
      └─ robot_state 消息 → robot_state_queue(独立 worker) → 位姿/相机帧/标定/采集

taskflow 和 robot_state 分队列避免只读查询被长动作阻塞；cancel 单独 topic 绕过 FIFO。
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from . import __version__
from .gdk.camera_calibration import run_gdk_camera_calibration_snapshot
from .gdk.camera_capture import CameraCaptureService
from .gdk.camera_frame import run_gdk_camera_frame_snapshot
from .gdk.control_probe import ALLOWED_ACTIONS, run_gdk_control_probe
from .gdk.current_pose import run_gdk_current_pose_snapshot
from .gdk.motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from .gdk.readonly import run_gdk_readonly_probe
from .gdk.session import GdkSessionManager
from .gdk.worker_runtime import (
    cancel_default_gdk_worker_command,
    diagnostics_default_gdk_worker,
    shutdown_default_gdk_worker,
)
from .mqtt.gateway import MqttGateway, MqttGatewayError, TaskflowMessage
from .mqtt.message_queue import MqttMessageQueueError, MqttMessageWorkerQueue
from .mqtt.robot_state import (
    CAMERA_CALIBRATION_REQUEST_TYPE,
    CAMERA_CAPTURE_START_REQUEST_TYPE,
    CAMERA_CAPTURE_STOP_REQUEST_TYPE,
    CAMERA_FRAME_REQUEST_TYPE,
    QR_BUILD_MAP_REQUEST_TYPE,
    QR_CAPTURE_START_REQUEST_TYPE,
    QR_CAPTURE_STOP_REQUEST_TYPE,
    QR_DELETE_MAP_REQUEST_TYPE,
    QR_PCD_PREVIEW_REQUEST_TYPE,
    POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE,
    POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE,
    POINT_RECORDING_SUBMIT_REQUEST_TYPE,
    handle_robot_state_request,
)
from .mqtt.status_reporter import StatusSequence, TaskflowStatusReporter
from .runtime.config import ConfigError, ExecutorSettings, build_env_source
from .runtime.diagnostics import (
    build_health_check_payload,
    build_runtime_diagnostics_payload,
    health_check_exit_code,
)
from .runtime.event_log import JsonlEventWriter, RuntimeEvent, configure_stdout_logging
from .runtime.payload_sanitizer import summarize_variables
from .qr_mapping.build_service import QrBuildService
from .qr_mapping.capture_service import QrCaptureService
from .qr_mapping.point_recording_service import PointRecordingService
from .qr_mapping.project_store import QrProjectStore
from .skills.registry import SkillRegistry, SkillRegistryError
from .skills.runtime import SkillRuntime
from .taskflow.control import (
    TaskflowExecutionController,
    parse_taskflow_cancel_request,
)
from .taskflow.parser import (
    MOTION_SPEED_MAX,
    MOTION_SPEED_MIN,
    TaskflowParseError,
    parse_taskflow_yaml,
)
from .taskflow.scheduler import SkillRuntimeNodeRunner, TaskflowScheduleError, TaskflowScheduler

RobotStatePublisher = Callable[[str, Mapping[str, Any]], None]


def build_parser() -> argparse.ArgumentParser:
    """构建 CLI 参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="gsa-taskflow-executor",
        description="GDK taskflow executor for self-developed GSA workflows.",
    )
    parser.add_argument(
        "--print-config",
        action="store_true",
        help="Print resolved executor settings and exit.",
    )
    parser.add_argument(
        "--env-file",
        type=Path,
        help="Load settings from a KEY=VALUE env file before reading process environment.",
    )
    parser.add_argument(
        "--write-sample-log",
        action="store_true",
        help="Write one sample JSONL runtime event and exit.",
    )
    parser.add_argument(
        "--listen",
        action="store_true",
        help="Connect MQTT broker and listen for taskflow YAML messages.",
    )
    parser.add_argument(
        "--print-skills",
        action="store_true",
        help="Print resolved skill registry and exit.",
    )
    parser.add_argument(
        "--health-check",
        "--diagnostics",
        dest="health_check",
        action="store_true",
        help=(
            "Run read-only executor diagnostics, including MQTT status roundtrip, "
            "and exit with non-zero status on hard failures."
        ),
    )
    parser.add_argument(
        "--gdk-readonly-probe",
        action="store_true",
        help="Run a one-shot read-only agibot_gdk probe and exit without robot control.",
    )
    parser.add_argument(
        "--gdk-control-probe",
        choices=ALLOWED_ACTIONS,
        help=(
            "Run a manually gated agibot_gdk control probe action and exit. "
            "Requires ENABLE_GDK_CONTROL=1 and a matching CONFIRM_GDK_CONTROL token."
        ),
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {__version__}",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """主入口。支持两类模式：

    1. 一次性诊断模式（--print-config / --print-skills / --gdk-*-probe / --write-sample-log）
    2. --listen 长连接模式：连接 MQTT broker，处理 taskflow/取消/robot_state 消息
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    logger = configure_stdout_logging()

    # 加载配置和技能白名单（所有模式共用）
    try:
        runtime_env = build_env_source(env_file=args.env_file)
        settings = ExecutorSettings.from_env(env=runtime_env)
    except ConfigError as error:
        parser.error(str(error))

    try:
        skill_registry = SkillRegistry.from_settings(settings)
    except SkillRegistryError as error:
        parser.error(str(error))

    # ---- 一次性诊断模式 ----

    if args.print_config:
        print(
            json.dumps(
                build_print_config_payload(settings, runtime_env),
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    if args.print_skills:
        print(json.dumps(skill_registry.summary(), ensure_ascii=False, indent=2))
        return 0

    if args.health_check:
        gdk_session = GdkSessionManager()
        payload = build_health_check_payload(
            settings=settings,
            runtime_env=runtime_env,
            skill_registry_summary=skill_registry.summary(),
            version=__version__,
            gdk_session_diagnostics=gdk_session.diagnostics(),
            gdk_worker_diagnostics=diagnostics_default_gdk_worker(),
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return health_check_exit_code(payload)

    if args.gdk_readonly_probe:
        writer = JsonlEventWriter.from_settings(settings)
        result = run_gdk_readonly_probe()
        writer.write(
            RuntimeEvent(
                event_type="gdk_readonly_probe",
                level="info" if result.get("available") is True else "warning",
                message="GDK read-only probe completed",
                payload={"probe": result},
            )
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    if args.gdk_control_probe:
        writer = JsonlEventWriter.from_settings(settings)
        result = run_gdk_control_probe(args.gdk_control_probe, environ=runtime_env)
        writer.write(
            RuntimeEvent(
                event_type="gdk_control_probe",
                level="info" if result.get("executed") is True else "warning",
                message="GDK manual control probe completed",
                payload={"probe": result},
            )
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    if args.write_sample_log:
        writer = JsonlEventWriter.from_settings(settings)
        path = writer.write(
            RuntimeEvent(
                event_type="executor_runtime_ready",
                message="配置与日志系统已初始化",
                topic=settings.taskflow_input_topic,
                payload={"mode": settings.executor_mode, "status_topic": settings.status_topic},
            )
        )
        logger.info("sample runtime event written to %s", path)
        return 0

    # ---- --listen 长连接模式 ----

    if args.listen:
        # 共享基础设施（listen 生命周期内复用）
        writer = JsonlEventWriter.from_settings(settings)
        gdk_session = GdkSessionManager()
        qr_project_store = QrProjectStore(settings.gsa_data_root)
        camera_capture_service = CameraCaptureService(
            session_manager=gdk_session,
            mqtt_broker_url=settings.mqtt_broker_url,
            mqtt_client_id=settings.mqtt_client_id,
            executor_aid=settings.executor_aid,
        )
        qr_capture_service = QrCaptureService(
            session_manager=gdk_session,
            project_store=qr_project_store,
            mqtt_broker_url=settings.mqtt_broker_url,
            mqtt_client_id=settings.mqtt_client_id,
            executor_aid=settings.executor_aid,
        )
        qr_build_service = QrBuildService(
            project_store=qr_project_store,
            sdk_path=settings.qr_mapping_sdk_path,
            sdk_python=settings.qr_mapping_sdk_python,
            build_timeout_seconds=settings.qr_mapping_build_timeout_seconds,
        )
        point_recording_service = PointRecordingService(
            project_store=qr_project_store,
            session_manager=gdk_session,
            localize_sdk_path=settings.qr_localize_sdk_path,
            localize_sdk_python=settings.qr_localize_sdk_python,
            localize_timeout_seconds=settings.qr_localize_timeout_seconds,
        )
        skill_runtime = SkillRuntime(
            registry=skill_registry,
            environ=runtime_env,
            gdk_session_manager=gdk_session,
        )
        # 单调递增序列号，保证桌面端即使 MQTT 乱序也能正确排序状态
        status_sequence = StatusSequence()
        # 执行控制器：跟踪当前活跃 taskflow，处理取消请求
        execution_controller = TaskflowExecutionController(
            cancel_gdk_command=cancel_default_gdk_worker_command,
        )
        gateway: MqttGateway
        taskflow_queue: MqttMessageWorkerQueue
        robot_state_queue: MqttMessageWorkerQueue

        def build_listen_diagnostics() -> dict[str, object]:
            """listen 运行态只读快照；不触发 GDK 初始化或控制。"""
            return build_runtime_diagnostics_payload(
                settings=settings,
                queue_snapshots=[
                    taskflow_queue.snapshot(),
                    robot_state_queue.snapshot(),
                ],
                execution_diagnostics=execution_controller.diagnostics(),
                gdk_session_diagnostics=gdk_session.diagnostics(),
                gdk_worker_diagnostics=diagnostics_default_gdk_worker(),
            )

        def publish_status_from_paho_callback(payload: Mapping[str, Any]) -> None:
            """paho 网络线程内只发布不等待，避免 QoS ack 等待阻塞后续 MQTT 消息。"""

            gateway.publish_status(payload, wait_for_terminal=False)

        # ==== taskflow worker 线程回调 ====

        def process_taskflow_message(message: TaskflowMessage) -> None:
            """解析 → 校验 → 调度 → 执行一个 taskflow YAML 消息。

            运行在 taskflow worker 线程内（非 paho 回调），长 GDK 操作不阻塞 MQTT 网络循环。
            """
            logger.info("taskflow YAML payload:\n%s", message.payload)
            reporter = TaskflowStatusReporter(
                settings=settings,
                publish_status=gateway.publish_status,
                status_sequence=status_sequence,
            )

            # 1. 解析 YAML
            try:
                taskflow = parse_taskflow_yaml(message.payload)
            except TaskflowParseError as error:
                logger.error("invalid taskflow YAML: %s", error)
                reporter.publish_execution_error(message=str(error))
                writer.write(
                    RuntimeEvent(
                        event_type="taskflow_yaml_parse_error",
                        level="error",
                        message=str(error),
                        topic=message.topic,
                    )
                )
                return

            # 2. 校验技能白名单
            try:
                skill_registry.validate_taskflow(taskflow)
            except SkillRegistryError as error:
                logger.error("taskflow skill registry validation failed: %s", error)
                reporter.publish_execution_error(
                    message=str(error),
                    app_execution_id=taskflow.app_execution_id,
                )
                writer.write(
                    RuntimeEvent(
                        event_type="taskflow_skill_registry_error",
                        level="error",
                        message=str(error),
                        app_execution_id=taskflow.app_execution_id,
                        topic=message.topic,
                        payload=skill_registry.summary(),
                    )
                )
                return

            summary = taskflow.summary()
            logger.info(
                "parsed taskflow app_execution_id=%s nodes=%s workers=%s transitions=%s",
                taskflow.app_execution_id,
                summary["node_count"],
                summary["worker_count"],
                summary["transition_count"],
            )
            writer.write(
                RuntimeEvent(
                    event_type="taskflow_yaml_parsed",
                    message="taskflow YAML parsed",
                    app_execution_id=taskflow.app_execution_id,
                    topic=message.topic,
                    payload=summary,
                )
            )

            # 3-6. 执行（finally 保证 finish_execution 一定调用，防止控制器状态残留）
            try:
                execution_controller.start_execution(taskflow.app_execution_id)
                writer.write(
                    RuntimeEvent(
                        event_type="executor_runtime_diagnostics",
                        message="taskflow execution started diagnostics snapshot",
                        app_execution_id=taskflow.app_execution_id,
                        topic=message.topic,
                        payload={"diagnostics": build_listen_diagnostics()},
                    )
                )
                reporter.publish_execution_started(taskflow)
                schedule = TaskflowScheduler(
                    taskflow,
                    node_runner=SkillRuntimeNodeRunner(
                        app_execution_id=taskflow.app_execution_id,
                        mode=settings.executor_mode,
                        runtime=skill_runtime,
                    ),
                    node_event_handler=reporter.publish_node_event,
                    # 每个调度步长协作式检查取消；检测到取消时调度器提前返回 CANCELED
                    cancel_checker=lambda: execution_controller.current_cancellation(
                        taskflow.app_execution_id,
                    ),
                ).run()
            except TaskflowScheduleError as error:
                logger.error("taskflow schedule failed: %s", error)
                reporter.publish_execution_error(
                    message=str(error),
                    app_execution_id=taskflow.app_execution_id,
                )
                writer.write(
                    RuntimeEvent(
                        event_type="taskflow_schedule_error",
                        level="error",
                        message=str(error),
                        app_execution_id=taskflow.app_execution_id,
                        topic=message.topic,
                    )
                )
                return
            finally:
                execution_controller.finish_execution(taskflow.app_execution_id)

            # 发布最终状态和变量快照
            reporter.publish_execution_finished(schedule)
            schedule_summary = schedule.summary()
            logger.info(
                "skill runtime schedule outcome=%s terminal=%s steps=%s path=%s",
                schedule.outcome,
                schedule.terminal_node_id,
                schedule_summary["step_count"],
                " -> ".join(schedule.visited_node_ids),
            )
            writer.write(
                RuntimeEvent(
                    event_type="taskflow_skill_runtime_scheduled",
                    message="taskflow scheduled by skill runtime",
                    app_execution_id=taskflow.app_execution_id,
                    topic=message.topic,
                    payload=schedule_summary,
                )
            )
            writer.write(
                RuntimeEvent(
                    event_type="executor_runtime_diagnostics",
                    message="taskflow execution finished diagnostics snapshot",
                    app_execution_id=taskflow.app_execution_id,
                    topic=message.topic,
                    payload={"diagnostics": build_listen_diagnostics()},
                )
            )
            writer.write(
                RuntimeEvent(
                    event_type="variable_store_summary",
                    message="variable store summary after skill runtime schedule",
                    app_execution_id=taskflow.app_execution_id,
                    topic=message.topic,
                    payload={"variables": summarize_variables(schedule.variables)},
                )
            )

        # ==== robot_state worker 线程回调 ====

        def process_robot_state_message(message: TaskflowMessage) -> None:
            """处理机器人只读状态查询（位姿、相机帧、标定、采集启停）。

            使用独立 worker 队列，不被 taskflow 长动作阻塞。GDK 访问仍由 session 锁互斥；
            相机请求在控制动作持锁时直接返回 ROBOT_BUSY，不等待。
            """
            handle_robot_state_request(
                message,
                settings=settings,
                publish_response=lambda topic, payload: gateway.publish_json(
                    topic,
                    payload,
                    event_type="robot_state_response_mqtt_published",
                    message="robot state response published",
                ),
                event_writer=writer,
                collect_current_pose=lambda: run_gdk_current_pose_snapshot(
                    session_manager=gdk_session,
                ),
                collect_camera_frame=lambda camera_id, timeout_ms: run_gdk_camera_frame_snapshot(
                    camera_id=camera_id,
                    timeout_ms=timeout_ms,
                    session_manager=gdk_session,
                ),
                collect_camera_calibration=(
                    lambda camera_ids, timeout_ms, include_extrinsics:
                    run_gdk_camera_calibration_snapshot(
                        camera_ids=camera_ids,
                        timeout_ms=timeout_ms,
                        include_extrinsics=include_extrinsics,
                        session_manager=gdk_session,
                    )
                ),
                start_camera_capture=camera_capture_service.start,
                stop_camera_capture=camera_capture_service.stop,
                start_qr_capture=qr_capture_service.start,
                stop_qr_capture=qr_capture_service.stop,
                build_qr_map=(
                    lambda robot_serial, project_name, map_name, camera_id, marker_type, marker_size:
                    qr_build_service.build_map(
                        robot_serial=robot_serial,
                        project_name=project_name,
                        map_name=map_name,
                        camera_id=camera_id,
                        marker_type=marker_type,
                        marker_size_meters=marker_size,
                    )
                ),
                delete_qr_map=(
                    lambda robot_serial, project_name, map_name:
                    qr_build_service.delete_map(
                        robot_serial=robot_serial,
                        project_name=project_name,
                        map_name=map_name,
                    )
                ),
                read_qr_pcd_preview=(
                    lambda robot_serial, project_name, map_name, max_points:
                    qr_build_service.read_pcd_preview(
                        robot_serial=robot_serial,
                        project_name=project_name,
                        map_name=map_name,
                        max_points=max_points,
                    )
                ),
                save_point_recording_target=point_recording_service.save_target_point,
                save_point_recording_initial_photo=(
                    point_recording_service.save_initial_photo_point
                ),
                submit_point_recording=point_recording_service.submit_recording,
            )

        # taskflow 执行队列（单 worker FIFO）
        taskflow_queue = MqttMessageWorkerQueue(
            name="taskflow-execution-worker",
            handler=process_taskflow_message,
            maxsize=settings.taskflow_queue_maxsize,
            queue_full_policy=settings.taskflow_queue_full_policy,
            logger=logger,
            event_writer=writer,
        )
        # robot_state 查询队列（独立 worker，与控制执行互斥由 session 锁保证）
        robot_state_queue = MqttMessageWorkerQueue(
            name="robot-state-worker",
            handler=process_robot_state_message,
            maxsize=settings.robot_state_queue_maxsize,
            queue_full_policy=settings.robot_state_queue_full_policy,
            logger=logger,
            event_writer=writer,
        )

        # ==== paho 回调（运行在 paho 网络线程，必须快速返回） ====

        def handle_taskflow_message(message: TaskflowMessage) -> None:
            """paho 回调：taskflow 消息入队。队列满时立即发布 ERROR 状态。"""
            try:
                queued_count = taskflow_queue.enqueue(message)
            except MqttMessageQueueError as error:
                logger.error("taskflow message queue rejected message: %s", error)
                TaskflowStatusReporter(
                    settings=settings,
                    publish_status=publish_status_from_paho_callback,
                    status_sequence=status_sequence,
                ).publish_execution_error(message=str(error))
                writer.write(
                    RuntimeEvent(
                        event_type="taskflow_execution_queue_rejected",
                        level="error",
                        message=str(error),
                        topic=message.topic,
                        payload={"diagnostics": build_listen_diagnostics()},
                    )
                )
                return

            logger.info(
                "taskflow YAML enqueued topic=%s queued=%s",
                message.topic,
                queued_count,
            )

        def handle_taskflow_cancel_message(message: TaskflowMessage) -> None:
            """paho 回调：取消请求不经过 taskflow FIFO，直接中断当前执行。

            执行控制器在每个调度步长协作式检查取消，同时向 GDK worker 子进程发 kill 信号。
            """
            reporter = TaskflowStatusReporter(
                settings=settings,
                publish_status=publish_status_from_paho_callback,
                status_sequence=status_sequence,
            )
            try:
                request = parse_taskflow_cancel_request(
                    topic=message.topic,
                    payload=message.payload,
                    topic_filter=settings.taskflow_cancel_topic_filter,
                )
            except ValueError as error:
                logger.error("invalid taskflow cancel request: %s", error)
                reporter.publish_execution_error(message=str(error))
                writer.write(
                    RuntimeEvent(
                        event_type="taskflow_cancel_parse_error",
                        level="error",
                        message=str(error),
                        topic=message.topic,
                    )
                )
                return

            result = execution_controller.request_cancel(request)
            raw_cancel_app_execution_id = result.get("app_execution_id")
            cancel_app_execution_id = (
                raw_cancel_app_execution_id
                if isinstance(raw_cancel_app_execution_id, str)
                else None
            )
            writer.write(
                RuntimeEvent(
                    event_type="taskflow_cancel_requested",
                    level="info" if result.get("accepted") is True else "warning",
                    message="taskflow cancel request handled",
                    app_execution_id=cancel_app_execution_id,
                    topic=message.topic,
                    payload={"cancel": result},
                )
            )
            # 取消成功且进入 CANCELING 状态时通知客户端
            if result.get("accepted") is True and result.get("state") == "CANCELING":
                if cancel_app_execution_id is not None:
                    gdk_cancel_result = result.get("gdk_cancel_result")
                    reporter.publish_execution_canceling(
                        app_execution_id=cancel_app_execution_id,
                        request_id=request.request_id,
                        reason=request.reason,
                        gdk_cancel_result=gdk_cancel_result
                        if isinstance(gdk_cancel_result, Mapping)
                        else None,
                    )

        def handle_robot_state_message(message: TaskflowMessage) -> None:
            """paho 回调：robot_state 消息入队。队列满时发布 QUEUE_UNAVAILABLE 错误。"""
            try:
                queued_count = robot_state_queue.enqueue(message)
            except MqttMessageQueueError as error:
                logger.error("robot state queue rejected message: %s", error)
                publish_robot_state_queue_error(
                    message,
                    settings=settings,
                    publish_response=lambda topic, payload: gateway.publish_json(
                        topic,
                        payload,
                        event_type="robot_current_pose_queue_error_published",
                        message="current pose queue error published",
                    ),
                    error_message=str(error),
                )
                writer.write(
                    RuntimeEvent(
                        event_type="robot_state_queue_rejected",
                        level="error",
                        message=str(error),
                        topic=message.topic,
                        payload={"diagnostics": build_listen_diagnostics()},
                    )
                )
                return

            logger.info(
                "robot state request enqueued topic=%s queued=%s",
                message.topic,
                queued_count,
            )

        # 启动 MQTT gateway 和 worker 线程
        gateway = MqttGateway(
            settings=settings,
            on_taskflow_message=handle_taskflow_message,
            on_robot_state_message=handle_robot_state_message,
            on_taskflow_cancel_message=handle_taskflow_cancel_message,
            logger=logger,
            event_writer=writer,
        )
        taskflow_queue.start()
        robot_state_queue.start()
        writer.write(
            RuntimeEvent(
                event_type="executor_runtime_diagnostics",
                message="executor runtime diagnostics snapshot",
                payload={"diagnostics": build_listen_diagnostics()},
            )
        )

        try:
            gateway.run_forever()  # 阻塞直到 MQTT 断连或致命错误
        except MqttGatewayError as error:
            parser.error(str(error))
        finally:
            # 优雅关闭：先停队列，再释放 GDK 资源（顺序重要）
            taskflow_queue.stop()
            robot_state_queue.stop()

            camera_capture_shutdown = camera_capture_service.shutdown()
            qr_capture_shutdown = qr_capture_service.shutdown()
            writer.write(
                RuntimeEvent(
                    event_type="qr_capture_shutdown",
                    level="info"
                    if qr_capture_shutdown.get("success") is True
                    else "warning",
                    message="QR capture shutdown completed",
                    payload={"qr_capture": qr_capture_shutdown},
                )
            )
            writer.write(
                RuntimeEvent(
                    event_type="camera_capture_shutdown",
                    level="info"
                    if camera_capture_shutdown.get("success") is True
                    else "warning",
                    message="camera capture shutdown completed",
                    payload={"camera_capture": camera_capture_shutdown},
                )
            )
            worker_shutdown = shutdown_default_gdk_worker()
            worker_shutdown_level = (
                "info" if worker_shutdown.get("success") is True else "warning"
            )
            if worker_shutdown_level == "warning":
                logger.warning("GDK worker shutdown skipped or failed: %s", worker_shutdown)
            writer.write(
                RuntimeEvent(
                    event_type="gdk_worker_shutdown",
                    level=worker_shutdown_level,
                    message="GDK persistent worker shutdown completed",
                    payload={"gdk_worker": worker_shutdown},
                )
            )
            shutdown_result = gdk_session.shutdown()
            shutdown_level = "info" if shutdown_result.get("success") is True else "warning"
            if shutdown_level == "warning":
                logger.warning("GDK session shutdown skipped or failed: %s", shutdown_result)
            writer.write(
                RuntimeEvent(
                    event_type="gdk_session_shutdown",
                    level=shutdown_level,
                    message="GDK process session shutdown completed",
                    payload={"gdk_release": shutdown_result},
                )
            )
        return 0

    print("gsa-taskflow-executor is ready.")
    print("Run with --print-config to inspect current settings.")
    return 0


def publish_robot_state_queue_error(
    message: TaskflowMessage,
    *,
    settings: ExecutorSettings,
    publish_response: RobotStatePublisher,
    error_message: str,
) -> None:
    """队列满时，按原消息 topic 匹配对应的 response topic 回传 QUEUE_UNAVAILABLE 错误。

    确保桌面端显示明确错误而非超时。
    """
    # 默认位姿响应；根据入站 topic 覆写
    response_topic = settings.robot_current_pose_response_topic
    response_type = "get_current_pose"
    if message.topic == settings.robot_camera_frame_request_topic:
        response_topic = settings.robot_camera_frame_response_topic
        response_type = CAMERA_FRAME_REQUEST_TYPE
    elif message.topic == settings.robot_camera_calibration_request_topic:
        response_topic = settings.robot_camera_calibration_response_topic
        response_type = CAMERA_CALIBRATION_REQUEST_TYPE
    elif message.topic == settings.robot_camera_capture_start_request_topic:
        response_topic = settings.robot_camera_capture_start_response_topic
        response_type = CAMERA_CAPTURE_START_REQUEST_TYPE
    elif message.topic == settings.robot_camera_capture_stop_request_topic:
        response_topic = settings.robot_camera_capture_stop_response_topic
        response_type = CAMERA_CAPTURE_STOP_REQUEST_TYPE
    elif message.topic == settings.qr_mapping_project_path_request_topic:
        response_topic = settings.qr_mapping_project_path_response_topic
        response_type = "get_qr_project_path"
    elif message.topic == settings.qr_mapping_project_snapshot_request_topic:
        response_topic = settings.qr_mapping_project_snapshot_response_topic
        response_type = "get_qr_project_snapshot"
    elif message.topic == settings.qr_mapping_capture_start_request_topic:
        response_topic = settings.qr_mapping_capture_start_response_topic
        response_type = QR_CAPTURE_START_REQUEST_TYPE
    elif message.topic == settings.qr_mapping_capture_stop_request_topic:
        response_topic = settings.qr_mapping_capture_stop_response_topic
        response_type = QR_CAPTURE_STOP_REQUEST_TYPE
    elif message.topic == settings.qr_mapping_build_map_request_topic:
        response_topic = settings.qr_mapping_build_map_response_topic
        response_type = QR_BUILD_MAP_REQUEST_TYPE
    elif message.topic == settings.qr_mapping_delete_map_request_topic:
        response_topic = settings.qr_mapping_delete_map_response_topic
        response_type = QR_DELETE_MAP_REQUEST_TYPE
    elif message.topic == settings.qr_mapping_pcd_preview_request_topic:
        response_topic = settings.qr_mapping_pcd_preview_response_topic
        response_type = QR_PCD_PREVIEW_REQUEST_TYPE
    elif message.topic == settings.point_recording_save_target_request_topic:
        response_topic = settings.point_recording_save_target_response_topic
        response_type = POINT_RECORDING_SAVE_TARGET_REQUEST_TYPE
    elif message.topic == settings.point_recording_save_initial_photo_request_topic:
        response_topic = settings.point_recording_save_initial_photo_response_topic
        response_type = POINT_RECORDING_SAVE_INITIAL_PHOTO_REQUEST_TYPE
    elif message.topic == settings.point_recording_submit_request_topic:
        response_topic = settings.point_recording_submit_response_topic
        response_type = POINT_RECORDING_SUBMIT_REQUEST_TYPE

    publish_response(
        response_topic,
        {
            "type": response_type,
            "requestId": read_request_id(message.payload),
            "ok": False,
            "executorAid": settings.executor_aid,
            "error": {
                "code": "QUEUE_UNAVAILABLE",
                "message": error_message,
            },
        },
    )


def read_request_id(payload: str) -> str:
    """从 JSON payload 提取 requestId 或 request_id 字段。失败返回空字符串。"""
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError:
        return ""

    if not isinstance(decoded, Mapping):
        return ""

    raw = decoded.get("requestId") or decoded.get("request_id")
    if isinstance(raw, str):
        return raw.strip()
    return ""


def build_print_config_payload(
    settings: ExecutorSettings,
    runtime_env: Mapping[str, str],
) -> dict[str, Any]:
    """构建 --print-config 诊断输出，附加安全门状态和运动速度限制。"""
    payload: dict[str, Any] = settings.to_dict()
    payload["taskflow_gdk_safety_gate"] = {
        "enabled": runtime_env.get("ENABLE_GDK_CONTROL") == "1",
        "confirmed": runtime_env.get("CONFIRM_GDK_CONTROL")
        == TASKFLOW_ABS_JOINT_CONFIRMATION,
        "expected_confirmation": TASKFLOW_ABS_JOINT_CONFIRMATION,
    }
    payload["motion_speed_limits"] = {
        "unit": "gdk_velocity",
        "min": MOTION_SPEED_MIN,
        "max": MOTION_SPEED_MAX,
    }
    return payload
