from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import __version__
from .config import ConfigError, ExecutorSettings
from .gdk_control_probe import ALLOWED_ACTIONS, run_gdk_control_probe
from .gdk_readonly import run_gdk_readonly_probe
from .mqtt_gateway import MqttGateway, MqttGatewayError, TaskflowMessage
from .runtime_logging import JsonlEventWriter, RuntimeEvent, configure_stdout_logging
from .scheduler import SkillRuntimeNodeRunner, TaskflowScheduleError, TaskflowScheduler
from .skill_registry import SkillRegistry, SkillRegistryError
from .skill_runtime import SkillRuntime
from .status_reporter import TaskflowStatusReporter
from .taskflow_parser import TaskflowParseError, parse_taskflow_yaml


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gsa-taskflow-executor",
        description="Dry-run taskflow executor scaffold for self-developed GSA workflows.",
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
        settings = ExecutorSettings.from_env(env_file=args.env_file)
    except ConfigError as error:
        parser.error(str(error))

    try:
        skill_registry = SkillRegistry.from_settings(settings)
    except SkillRegistryError as error:
        parser.error(str(error))

    if args.print_config:
        print(json.dumps(settings.to_dict(), ensure_ascii=False, indent=2))
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
        result = run_gdk_control_probe(args.gdk_control_probe)
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
                event_type="executor_scaffold_ready",
                message="配置与日志系统已初始化",
                topic=settings.taskflow_input_topic,
                payload={"mode": settings.executor_mode, "status_topic": settings.status_topic},
            )
        )
        logger.info("sample runtime event written to %s", path)
        return 0

    if args.listen:
        writer = JsonlEventWriter.from_settings(settings)
        gateway: MqttGateway

        def handle_taskflow_message(message: TaskflowMessage) -> None:
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
                        runtime=SkillRuntime(registry=skill_registry),
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

        gateway = MqttGateway(
            settings=settings,
            on_taskflow_message=handle_taskflow_message,
            logger=logger,
            event_writer=writer,
        )
        try:
            gateway.run_forever()
        except MqttGatewayError as error:
            parser.error(str(error))
        return 0

    print("gsa-taskflow-executor scaffold is ready.")
    print("Run with --print-config to inspect current settings.")
    return 0
