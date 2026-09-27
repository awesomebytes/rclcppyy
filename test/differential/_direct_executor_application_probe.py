"""Single-threaded executor and callback-group application scenario."""

import argparse
import importlib
import os
import sys
import time
import traceback

from .protocol import encode_result, verify_backend_expectations


def _spin_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    if not predicate():
        raise AssertionError("executor application callback did not complete")


def _run(mode):
    backend = None
    if mode == "activated":
        import rclcppyy

        backend = rclcppyy
        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String

    class ExecutorAppNode(Node):
        pass

    if mode == "activated":
        def forbidden_boundary(*_args, **_kwargs):
            raise AssertionError("Python-message conversion/serialization ran")

        kit = importlib.import_module("rclcpp_kit")
        bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
        serialization = importlib.import_module("rclcpp_kit.serialization")
        rclpy_serialization = importlib.import_module("rclpy.serialization")
        kit.convert_python_msg_to_cpp = forbidden_boundary
        bringup.convert_python_msg_to_cpp = forbidden_boundary
        serialization.serialize_message = forbidden_boundary
        serialization.deserialize_message = forbidden_boundary
        rclpy_serialization.serialize_message = forbidden_boundary
        rclpy_serialization.deserialize_message = forbidden_boundary

    rclpy.init(args=[])
    suffix = "p%d" % os.getpid()
    topic = "/direct_executor_application/" + suffix
    node = None
    context = None
    executor = None
    group = None
    publisher = None
    subscription = None
    timer = None
    cleanup = {
        "entities_destroyed": False,
        "node_removed": False,
        "executor_shutdown": False,
        "node_destroyed": False,
        "context_shutdown": False,
    }
    observations = {}
    try:
        node = ExecutorAppNode(
            "executor_application_" + suffix,
            start_parameter_services=False,
        )
        context = node.context
        executor = SingleThreadedExecutor(context=context)
        assert executor.add_node(node)
        group = MutuallyExclusiveCallbackGroup()
        received = []
        timer_ticks = []
        publisher = node.create_publisher(
            String, topic, 10)
        subscription = node.create_subscription(
            String, topic,
            lambda message: received.append(str(message.data)),
            10,
            callback_group=group,
        )
        timer = node.create_timer(
            0.02, lambda: timer_ticks.append("tick"), callback_group=group)

        _spin_until(executor, lambda: publisher.get_subscription_count() >= 1)
        for value in ("executor-one", "executor-two", "executor-three"):
            publisher.publish(String(data=value))
        _spin_until(
            executor,
            lambda: len(received) == 3 and bool(timer_ticks),
        )

        observations = {
            "received": received,
            "timer_fired": bool(timer_ticks),
            "executor_owned_node": node.executor is executor,
            "executor_node_count": len(executor.get_nodes()),
            "subscription_in_callback_group": group.has_entity(subscription),
            "timer_in_callback_group": group.has_entity(timer),
            "context_preserved": node.context is context,
        }
        assert observations["executor_owned_node"]
        assert observations["executor_node_count"] == 1
        assert all(
            observations[key]
            for key in (
                "subscription_in_callback_group",
                "timer_in_callback_group",
                "context_preserved",
            )
        ), observations

        expectations = []
        status = None
        if backend is not None:
            status = backend.status()
            expectations = [
                {
                    "kind": "nodes",
                    "backend": "cpp",
                    "minimum": 1,
                    "metadata": {"name": node.get_name()},
                },
                *[
                    {
                        "kind": "entities",
                        "backend": "cpp",
                        "minimum": 1,
                        "metadata": metadata,
                    }
                    for metadata in (
                        {"entity_type": "publisher", "topic": topic},
                        {"entity_type": "subscription", "topic": topic},
                        {"entity_type": "timer"},
                    )
                ],
            ]
            assert not verify_backend_expectations(status, expectations)
            assert all(
                "no_conversion" in record["policies"]
                for record in status["entities"]
                if record["metadata"].get("entity_type") in
                ("publisher", "subscription", "timer")
            )

        executor.remove_node(node)
        assert executor.get_nodes() == []
        cleanup["node_removed"] = True
    finally:
        if node is not None:
            context = node.context
            destroyed = True
            for entity_name, entity in (
                ("timer", timer),
                ("subscription", subscription),
                ("publisher", publisher),
            ):
                if entity is not None:
                    destroyed = (
                        bool(getattr(node, "destroy_" + entity_name)(entity))
                        and destroyed
                    )
            cleanup["entities_destroyed"] = destroyed
            if executor is not None:
                if node in executor.get_nodes():
                    executor.remove_node(node)
                cleanup["node_removed"] = node not in executor.get_nodes()
                cleanup["executor_shutdown"] = executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
            cleanup["node_destroyed"] = True
        rclpy.shutdown()
        cleanup["context_shutdown"] = not (
            context.ok() if context is not None else rclpy.ok())

    return (
        observations,
        expectations,
        status,
        not expectations or not verify_backend_expectations(status, expectations),
        cleanup,
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("stock", "activated"), required=True)
    mode = parser.parse_args(argv).mode
    try:
        observations, expectations, status, verified, cleanup = _run(mode)
        result = {
            "schema": "rclcppyy.differential/v1",
            "scenario": "direct_executor_callback_group_application",
            "mode": mode,
            "outcome": "pass",
            "observations": observations,
            "backend_expectations": expectations,
            "backend_status": status,
            "backend_verified": verified,
            "cleanup": cleanup,
            "error": None,
        }
    except BaseException as exc:
        result = {
            "schema": "rclcppyy.differential/v1",
            "scenario": "direct_executor_callback_group_application",
            "mode": mode,
            "outcome": "error",
            "observations": {},
            "backend_expectations": [],
            "backend_status": None,
            "backend_verified": False,
            "cleanup": {},
            "error": {
                "type": type(exc).__name__,
                "message": str(exc),
                "traceback": traceback.format_exc(),
            },
        }
    print(encode_result(result), flush=True)
    return 0 if result["outcome"] == "pass" and result["backend_verified"] else 1


if __name__ == "__main__":
    sys.exit(main())
