#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the node logger surface."""

from __future__ import annotations

import argparse
import importlib
import inspect
import json


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.clock import Clock, ClockType  # noqa: E402
from rclpy.impl.logging_severity import LoggingSeverity  # noqa: E402
from rclpy.impl.rcutils_logger import RcutilsLogger  # noqa: E402
from rclpy.node import Node  # noqa: E402


def poison_direct_boundaries():
    if args.backend != "direct":
        return

    def forbidden(*_args, **_kwargs):
        raise AssertionError("conversion or serialization entered logging proof")

    kit = importlib.import_module("rclcpp_kit")
    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    rclpy_serialization = importlib.import_module("rclpy.serialization")
    kit.convert_python_msg_to_cpp = forbidden
    bringup.convert_python_msg_to_cpp = forbidden
    serialization.serialize_message = forbidden
    serialization.deserialize_message = forbidden
    serialization.serialized_message_from_bytes = forbidden
    serialization.serialized_message_to_bytes = forbidden
    rclpy_serialization.serialize_message = forbidden
    rclpy_serialization.deserialize_message = forbidden


def log_once(logger):
    return [logger.info("logging differential", once=True) for _ in range(3)]


def log_once_false(logger):
    return [logger.info("logging differential", once=False) for _ in range(2)]


def log_skip_first(logger):
    return [logger.info("logging differential", skip_first=True) for _ in range(3)]


def log_throttled(logger, clock):
    return [
        logger.info(
            "logging differential",
            throttle_duration_sec=3600.0,
            throttle_time_source_type=clock,
        )
        for _ in range(3)
    ]


def log_skip_first_once(logger):
    return [
        logger.info("logging differential", skip_first=True, once=True)
        for _ in range(3)
    ]


def changing_filter(logger):
    outcomes = []
    for once in (True, False):
        try:
            outcomes.append(logger.info("changing filter", once=once))
        except Exception as exception:  # noqa: BLE001 - exception is evidence
            outcomes.append([type(exception).__name__, str(exception)])
    return outcomes


def changing_severity(logger):
    outcomes = []
    for severity in (LoggingSeverity.INFO, LoggingSeverity.WARN):
        try:
            outcomes.append(logger.log("changing severity", severity))
        except Exception as exception:  # noqa: BLE001 - exception is evidence
            outcomes.append([type(exception).__name__, str(exception)])
    return outcomes


def raised(operation):
    try:
        operation()
    except Exception as exception:  # noqa: BLE001 - exception is evidence
        return [type(exception).__name__, str(exception)]
    return None


poison_direct_boundaries()
rclpy.init(args=[
    "--ros-args",
    "--disable-stdout-logs",
    "--disable-rosout-logs",
    "--disable-external-lib-logs",
])
node = None
try:
    node = Node(
        "requested_logging_probe",
        namespace="/logging_audit",
        cli_args=["--ros-args", "-r", "__node:=resolved_logging_probe"],
    )
    logger = node.get_logger()
    same_logger = logger is node.get_logger()
    logger.set_level(LoggingSeverity.DEBUG)
    child = logger.get_child("child")
    child.set_level(LoggingSeverity.UNSET)
    clock = Clock(clock_type=ClockType.STEADY_TIME)

    methods = (
        "get_child", "set_level", "get_effective_level", "is_enabled_for",
        "log", "debug", "info", "warning", "warn", "error", "fatal",
    )
    contract = {
        "logger_type": [type(logger).__module__, type(logger).__name__],
        "is_exact_rcutils_logger": type(logger) is RcutilsLogger,
        "same_logger_identity": same_logger,
        "name": logger.name,
        "resolved_node_name": node.get_name(),
        "signatures": {
            name: str(inspect.signature(getattr(RcutilsLogger, name)))
            for name in methods
        },
        "level": int(logger.get_effective_level()),
        "enabled": {
            "debug": logger.is_enabled_for(LoggingSeverity.DEBUG),
            "info": logger.is_enabled_for(LoggingSeverity.INFO),
        },
        "threshold": {
            "below": logger.log("below threshold", LoggingSeverity.UNSET),
            "at": logger.debug("at threshold"),
        },
        "once": log_once(logger),
        "once_false": log_once_false(logger),
        "skip_first": log_skip_first(logger),
        "throttle": log_throttled(logger, clock),
        "skip_first_once": log_skip_first_once(logger),
        "changing_filter": changing_filter(logger),
        "changing_severity": changing_severity(logger),
        "invalid": {
            "empty_child": raised(lambda: logger.get_child("")),
            "unknown_option": raised(
                lambda: logger.info("invalid", unsupported=True)),
            "missing_throttle": raised(
                lambda: logger.info(
                    "invalid", throttle_time_source_type=clock)),
            "non_string_message": raised(lambda: logger.info(42)),
        },
        "child": {
            "type": [type(child).__module__, type(child).__name__],
            "name": child.name,
            "effective_level": int(child.get_effective_level()),
        },
    }

    node.destroy_node()
    contract["retained_after_destroy"] = {
        "same_identity": node.get_logger() is logger,
        "enabled": logger.is_enabled_for(LoggingSeverity.INFO),
        "logged": logger.info("retained logger"),
    }
    node = None
    print("RCLCPPYY_LOGGING_PROBE " + json.dumps({
        "backend": args.backend,
        "contract": contract,
        "boundary": {
            "conversion_forbidden": args.backend == "direct",
            "serialization_forbidden": args.backend == "direct",
        },
    }, sort_keys=True), flush=True)
finally:
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
