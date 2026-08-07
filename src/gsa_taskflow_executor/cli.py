from __future__ import annotations

import argparse
import json
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from . import __version__
from .gdk.camera_frame import run_gdk_camera_frame_snapshot
from .gdk.control_probe import ALLOWED_ACTIONS, run_gdk_control_probe
from .gdk.current_pose import run_gdk_current_pose_snapshot
from .gdk.motion_runtime import TASKFLOW_ABS_JOINT_CONFIRMATION
from .gdk.readonly import run_gdk_readonly_probe
from .gdk.session import GdkSessionManager
from .gdk.worker_runtime import shutdown_default_gdk_worker
from .mqtt.gateway import MqttGateway, MqttGatewayError, TaskflowMessage
from .mqtt.message_queue import MqttMessageQueueError, MqttMessageWorkerQueue
from .mqtt.robot_state import CAMERA_FRAME_REQUEST_TYPE, handle_robot_state_request
from .mqtt.status_reporter import TaskflowStatusReporter
from .runtime.config import ConfigError, ExecutorSettings, build_env_source
from .runtime.event_log import JsonlEventWriter, RuntimeEvent, configure_stdout_logging
from .skills.registry import SkillRegistry, SkillRegistryError
from .skills.runtime import SkillRuntime
from .taskflow.parser import (
    MOTION_SPEED_MAX,
    MOTION_SPEED_MIN,
    TaskflowParseError,
    parse_taskflow_yaml,
)
from .taskflow.scheduler import SkillRuntimeNodeRunner, TaskflowScheduleError, TaskflowScheduler

TASKFLOW_MESSAGE_QUEUE_MAXSIZE = 16
ROBOT_STATE_MESSAGE_QUEUE_MAXSIZE = 8
RobotStatePublisher = Callable[[str, Mapping[str, Any]], None]


def build_parser() -> argparse.ArgumentParser:
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
    parser = build_parser()
    args = parser.parse_args(argv)
    logger = configure_stdout_logging()

    try:
        runtime_env = build_env_source(env_file=args.env_file)
        settings = ExecutorSettings.from_env(env=runtime_env)
    except ConfigError as error:
        parser.error(str(error))

    try:
        skill_registry = SkillRegistry.from_settings(settings)
    except SkillRegistryError as error:
        parser.error(str(error))

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

    if args.listen:
        writer = JsonlEventWriter.from_settings(settings)
        gdk_session = GdkSessionManager()
        skill_runtime = SkillRuntime(
            registry=skill_registry,
            environ=runtime_env,
            gdk_session_manager=gdk_session,
        )
        gateway: MqttGateway

        def process_taskflow_message(message: TaskflowMessage) -> None:
            # 这里是真正执行 Taskflow 的 worker 线程路径；不要在 paho on_message 中直接调用。
            logger.info("taskflow YAML payload:\n%s", message.payload)
            reporter = TaskflowStatusReporter(
                settings=settings,
                publish_status=gateway.publish_status,
            )
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
            reporter.publish_execution_started(taskflow)
            try:
                schedule = TaskflowScheduler(
                    taskflow,
                    node_runner=SkillRuntimeNodeRunner(
                        app_execution_id=taskflow.app_execution_id,
                        mode=settings.executor_mode,
                        runtime=skill_runtime,
                    ),
                    node_event_handler=reporter.publish_node_event,
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
                    event_type="variable_store_snapshot",
                    message="variable store snapshot after skill runtime schedule",
                    app_execution_id=taskflow.app_execution_id,
                    topic=message.topic,
                    payload={"variables": schedule.variables},
                )
            )

        def process_robot_state_message(message: TaskflowMessage) -> None:
            # 机器人只读请求放入单独 worker，避免被 Taskflow FIFO 队列排在长动作之后。
            # 具体 GDK 访问再由 session 锁互斥；相机请求忙时直接拒绝，不等待控制动作。
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
            )

        # taskflow_queue 保持 MVP 单任务串行；
        # 长时间 move_* 调用只阻塞该 worker，不阻塞 MQTT 网络循环。
        taskflow_queue = MqttMessageWorkerQueue(
            name="taskflow-execution-worker",
            handler=process_taskflow_message,
            maxsize=TASKFLOW_MESSAGE_QUEUE_MAXSIZE,
            logger=logger,
            event_writer=writer,
        )
        # robot_state_queue 让 get_current_pose/get_camera_frame 请求脱离 paho 回调；
        # 相机取帧同样是 GDK 只读能力，但必须和控制动作互斥。
        robot_state_queue = MqttMessageWorkerQueue(
            name="robot-state-worker",
            handler=process_robot_state_message,
            maxsize=ROBOT_STATE_MESSAGE_QUEUE_MAXSIZE,
            logger=logger,
            event_writer=writer,
        )

        def handle_taskflow_message(message: TaskflowMessage) -> None:
            try:
                # paho 回调只做入队和轻量日志，队列满时快速发布 ERROR。
                queued_count = taskflow_queue.enqueue(message)
            except MqttMessageQueueError as error:
                logger.error("taskflow message queue rejected message: %s", error)
                TaskflowStatusReporter(
                    settings=settings,
                    publish_status=gateway.publish_status,
                ).publish_execution_error(message=str(error))
                writer.write(
                    RuntimeEvent(
                        event_type="taskflow_execution_queue_rejected",
                        level="error",
                        message=str(error),
                        topic=message.topic,
                    )
                )
                return

            logger.info(
                "taskflow YAML enqueued topic=%s queued=%s",
                message.topic,
                queued_count,
            )

        def handle_robot_state_message(message: TaskflowMessage) -> None:
            try:
                # 当前位姿请求同样只入队，避免只读 GDK 查询卡住后续 MQTT 消息分发。
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
                    )
                )
                return

            logger.info(
                "robot state request enqueued topic=%s queued=%s",
                message.topic,
                queued_count,
            )

        gateway = MqttGateway(
            settings=settings,
            on_taskflow_message=handle_taskflow_message,
            on_robot_state_message=handle_robot_state_message,
            logger=logger,
            event_writer=writer,
        )
        taskflow_queue.start()
        robot_state_queue.start()
        try:
            gateway.run_forever()
        except MqttGatewayError as error:
            parser.error(str(error))
        finally:
            taskflow_queue.stop()
            robot_state_queue.stop()
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
    # 队列不可用时仍尽量按对应 response 协议回包，客户端可以展示明确错误。
    is_camera_request = message.topic == settings.robot_camera_frame_request_topic
    publish_response(
        settings.robot_camera_frame_response_topic
        if is_camera_request
        else settings.robot_current_pose_response_topic,
        {
            "type": CAMERA_FRAME_REQUEST_TYPE if is_camera_request else "get_current_pose",
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
