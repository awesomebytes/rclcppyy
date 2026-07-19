#!/usr/bin/env python3
"""Fresh-process stock/direct differential for ``wait_for_message``."""

import argparse
import importlib
import inspect
import json
import os
import threading
import time


REPORT_PREFIX = "DIRECT_WAIT_FOR_MESSAGE_REPORT="


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    return parser.parse_args()


def _wait_for_subscriber(publisher, timeout=10.0):
    deadline = time.monotonic() + timeout
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert publisher.get_subscription_count() >= 1


def _wait_for_subscription_cleanup(publisher, timeout=5.0):
    deadline = time.monotonic() + timeout
    while publisher.get_subscription_count() and time.monotonic() < deadline:
        time.sleep(0.01)
    return publisher.get_subscription_count() == 0


def _publish_after_discovery(publisher, message):
    errors = []

    def publish():
        try:
            _wait_for_subscriber(publisher)
            publisher.publish(message)
        except BaseException as exception:
            errors.append(exception)

    thread = threading.Thread(target=publish, name="wait-for-message-publisher")
    thread.start()
    return thread, errors


def _install_boundary_poison():
    calls = {"conversion": 0, "serialization": 0, "cdr": 0}

    def poison(kind):
        def forbidden(*_args, **_kwargs):
            calls[kind] += 1
            raise AssertionError(
                "wait_for_message crossed the %s boundary" % kind)

        return forbidden

    conversion = poison("conversion")
    serialization = poison("serialization")
    cdr = poison("cdr")
    kit = importlib.import_module("rclcpp_kit")
    kit_bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    product_bringup = importlib.import_module("rclcppyy.bringup_rclcpp")
    product_node = importlib.import_module("rclcppyy.node")
    kit_serialization = importlib.import_module("rclcpp_kit.serialization")
    product_serialization = importlib.import_module("rclcppyy.serialization")
    rclpy_serialization = importlib.import_module("rclpy.serialization")

    kit.convert_python_msg_to_cpp = conversion
    kit_bringup.convert_python_msg_to_cpp = conversion
    product_bringup.convert_python_msg_to_cpp = conversion
    product_node.convert_python_msg_to_cpp = conversion
    for module in (kit_serialization, product_serialization, rclpy_serialization):
        module.serialize_message = serialization
        module.deserialize_message = serialization
    for module in (kit_serialization, product_serialization):
        module.serialized_message_from_bytes = cdr
        module.serialized_message_to_bytes = cdr
    return calls


def main():
    args = _arguments()
    boundary_calls = {"conversion": 0, "serialization": 0, "cdr": 0}
    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(
            profile="direct_cpp", interfaces=("std_msgs/msg/String",))
        boundary_calls = _install_boundary_poison()

    import rclpy
    import rclpy.wait_for_message as wait_module
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    wait_for_message = wait_module.wait_for_message
    installed = inspect.unwrap(wait_for_message)
    public_contract = {
        "module": wait_for_message.__module__,
        "name": wait_for_message.__name__,
        "signature": str(inspect.signature(wait_for_message)),
        "signature_preserved": (
            inspect.signature(wait_for_message) == inspect.signature(installed)
        ),
        "installed_source_preserved": (
            inspect.getsourcefile(installed) or ""
        ).endswith("rclpy/wait_for_message.py"),
    }

    rclpy.init(args=[])
    waiter = Node("wait_for_message_waiter_%s_%d" % (args.backend, os.getpid()))
    sender = Node("wait_for_message_sender_%s_%d" % (args.backend, os.getpid()))
    prefix = "/rclcppyy/wait_for_message/%s/p%d" % (args.backend, os.getpid())
    publishers = []
    retained = []
    behavior = {}
    recursive_guard = None
    try:
        default_topic = prefix + "/default"
        default_publisher = sender.create_publisher(String, default_topic, 1)
        publishers.append(default_publisher)
        thread, errors = _publish_after_discovery(
            default_publisher, String(data="default-qos"))
        succeeded, message = wait_for_message(
            String, waiter, default_topic, time_to_wait=10.0)
        thread.join(timeout=12.0)
        assert not thread.is_alive() and errors == [], repr(errors)
        assert succeeded and message is not None
        retained.append(message)
        behavior["default"] = str(message.data)
        behavior["default_subscription_cleaned"] = (
            _wait_for_subscription_cleanup(default_publisher)
        )
        assert waiter.executor is None

        explicit_topic = prefix + "/explicit"
        explicit_qos = QoSProfile(
            depth=3,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        explicit_publisher = sender.create_publisher(
            String, explicit_topic, explicit_qos)
        publishers.append(explicit_publisher)
        thread, errors = _publish_after_discovery(
            explicit_publisher, String(data="explicit-qos"))
        succeeded, message = wait_for_message(
            String,
            waiter,
            explicit_topic,
            qos_profile=explicit_qos,
            time_to_wait=10.0,
        )
        thread.join(timeout=12.0)
        assert not thread.is_alive() and errors == [], repr(errors)
        assert succeeded and message is not None
        retained.append(message)
        behavior["explicit"] = str(message.data)
        behavior["explicit_subscription_cleaned"] = (
            _wait_for_subscription_cleanup(explicit_publisher)
        )
        assert waiter.executor is None

        behavior["timeout"] = wait_for_message(
            String, waiter, prefix + "/timeout", time_to_wait=0.02)
        behavior["zero_timeout"] = wait_for_message(
            String, waiter, prefix + "/zero", time_to_wait=0.0)
        assert behavior["timeout"] == (False, None)
        assert behavior["zero_timeout"] == (False, None)
        assert waiter.executor is None

        if args.backend == "direct":
            from rclcppyy.policy import BackendUnavailableError
            from rclpy.executors import SingleThreadedExecutor

            executor = SingleThreadedExecutor(context=waiter.context)
            assert executor.add_node(waiter)
            before_subscriptions = len(tuple(waiter.subscriptions))
            rejection = []
            timer = None

            def reject_recursive_wait():
                timer.cancel()
                try:
                    wait_for_message(
                        String,
                        waiter,
                        prefix + "/recursive",
                        time_to_wait=0.0,
                    )
                except BackendUnavailableError as exception:
                    rejection.append(str(exception))

            timer = waiter.create_timer(0.001, reject_recursive_wait)
            deadline = time.monotonic() + 2.0
            while not rejection and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            assert len(rejection) == 1
            assert "cannot recursively wait" in rejection[0]
            recursive_guard = {
                "failed_before_subscription": (
                    len(tuple(waiter.subscriptions)) == before_subscriptions
                ),
                "exception": "BackendUnavailableError",
            }
            assert waiter.destroy_timer(timer)
            executor.remove_node(waiter)
            assert waiter.executor is None
            assert executor.shutdown(timeout_sec=1.0)
    finally:
        for publisher in publishers:
            sender.destroy_publisher(publisher)
        waiter.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()

    retained_after_teardown = [str(message.data) for message in retained]
    exact_cpp_message = False
    runtime_released = True
    if args.backend == "direct":
        import cppyy

        exact_cpp_message = (
            String is cppyy.gbl.std_msgs.msg.String and
            all(type(message) is String for message in retained)
        )
        runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()
        runtime_released = runtime.session is None and runtime.nodes == []

    assert all(boundary_calls[kind] == 0 for kind in boundary_calls)
    report = {
        "backend": args.backend,
        "behavior": behavior,
        "boundary_calls": boundary_calls,
        "exact_cpp_message": exact_cpp_message,
        "public_contract": public_contract,
        "recursive_guard": recursive_guard,
        "retained_after_teardown": retained_after_teardown,
        "runtime_released": runtime_released,
    }
    print(REPORT_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
