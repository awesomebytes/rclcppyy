"""Isolated ROS probe used by the differential contract test."""

import argparse
import gc
import sys
import time
import traceback

from .protocol import encode_result, verify_backend_expectations


GRAPH_WAIT_S = 5.0
PREFIX = "diff_"


def _nodes(observer):
    return sorted(
        [
            {"name": name, "namespace": namespace}
            for name, namespace in observer.get_node_names_and_namespaces()
            if name.startswith(PREFIX) and name != "diff_observer"
        ],
        key=lambda item: (item["namespace"], item["name"]),
    )


def _wait_nodes(observer, predicate):
    deadline = time.monotonic() + GRAPH_WAIT_S
    observed = _nodes(observer)
    while not predicate(observed) and time.monotonic() < deadline:
        time.sleep(0.05)
        observed = _nodes(observer)
    return observed


def _destroy_and_collect(node):
    node.destroy_node()
    gc.collect()


def _observe_identity(rclpy, observer, context):
    node = rclpy.create_node(
        "diff_identity", namespace="/diff_identity_ns", context=context)
    expected_names = {"diff_identity"}
    graph_nodes = _wait_nodes(
        observer,
        lambda nodes: expected_names <= {item["name"] for item in nodes},
    )
    observation = {
        "facade": {
            "name": node.get_name(),
            "namespace": node.get_namespace(),
            "type": "%s.%s" % (type(node).__module__, type(node).__qualname__),
        },
        "graph_nodes": graph_nodes,
    }
    _destroy_and_collect(node)
    del node
    gc.collect()
    _wait_nodes(observer, lambda nodes: not any(
        item["name"].startswith("diff_identity") for item in nodes))
    return observation


def _observe_remapping(rclpy, observer, context):
    node = rclpy.create_node(
        "diff_remap_source",
        context=context,
        cli_args=[
            "--ros-args",
            "-r", "__node:=diff_remapped",
            "-r", "__ns:=/diff_remapped_ns",
        ],
    )
    graph_nodes = _wait_nodes(
        observer,
        lambda nodes: any(item["name"] == "diff_remapped" for item in nodes),
    )
    observation = {
        "facade": {"name": node.get_name(), "namespace": node.get_namespace()},
        "graph_nodes": graph_nodes,
    }
    _destroy_and_collect(node)
    del node
    gc.collect()
    _wait_nodes(observer, lambda nodes: not any(
        item["name"].startswith("diff_remap") for item in nodes))
    return observation


def _observe_parameters(rclpy, observer, context, Parameter):
    node = rclpy.create_node(
        "diff_parameters",
        namespace="/diff_parameters_ns",
        context=context,
        parameter_overrides=[Parameter("answer", value=42)],
        automatically_declare_parameters_from_overrides=True,
    )
    graph_nodes = _wait_nodes(
        observer,
        lambda nodes: any(item["name"] == "diff_parameters" for item in nodes),
    )
    owners = []
    deadline = time.monotonic() + GRAPH_WAIT_S
    while len(owners) != len(graph_nodes) and time.monotonic() < deadline:
        owners = []
        for item in graph_nodes:
            services = observer.get_service_names_and_types_by_node(
                item["name"], item["namespace"])
            if any(name.endswith("/get_parameters") for name, _types in services):
                owners.append(item)
        if len(owners) != len(graph_nodes):
            time.sleep(0.05)
    observation = {
        "value": node.get_parameter("answer").value,
        "parameter_service_owners": sorted(
            owners, key=lambda item: (item["namespace"], item["name"])),
    }
    _destroy_and_collect(node)
    del node
    gc.collect()
    _wait_nodes(observer, lambda nodes: not any(
        item["name"].startswith("diff_parameters") for item in nodes))
    return observation


def _qualified_type(value):
    return "%s.%s" % (type(value).__module__, type(value).__qualname__)


def _observe_entity_routing(rclpy, context):
    from std_msgs.msg import String
    from rclpy.executors import SingleThreadedExecutor

    node = rclpy.create_node(
        "diff_entities", namespace="/diff_entities_ns", context=context)
    received = []
    executor = None
    try:
        subscription = node.create_subscription(
            String, "contract_topic", lambda message: received.append(message.data), 10)
        publisher = node.create_publisher(String, "contract_topic", 10)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + GRAPH_WAIT_S
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if publisher.get_subscription_count() < 1:
            raise AssertionError("differential publisher and subscription did not match")
        publisher.publish(String(data="differential-message"))
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if received != ["differential-message"]:
            raise AssertionError("differential payload mismatch: %r" % received)
        return {
            "publisher": {
                "type": _qualified_type(publisher),
                "topic_name": publisher.topic_name,
            },
            "subscription": {
                "type": _qualified_type(subscription),
                "topic_name": subscription.topic_name,
            },
            "received": received,
        }
    finally:
        if executor is not None:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        _destroy_and_collect(node)


def _observe_teardown(rclpy, observer, context):
    node = rclpy.create_node(
        "diff_teardown", namespace="/diff_teardown_ns", context=context)
    before = _wait_nodes(
        observer,
        lambda nodes: any(item["name"] == "diff_teardown" for item in nodes),
    )
    node.destroy_node()
    after = _wait_nodes(
        observer,
        lambda nodes: not any(item["name"] == "diff_teardown" for item in nodes),
    )
    observation = {
        "graph_nodes_before_destroy": before,
        "graph_nodes_after_destroy": after,
        "public_node_present_after_destroy": any(
            item["name"] == "diff_teardown" for item in after),
    }
    del node
    gc.collect()
    _wait_nodes(observer, lambda nodes: not any(
        item["name"].startswith("diff_teardown") for item in nodes))
    return observation


def _run(mode):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node as StockNode
    from rclpy.parameter import Parameter

    cleanup = {
        "custom_context_shutdown": False,
        "default_context_shutdown": False,
        "observer_destroyed": False,
    }
    observations = {}
    custom_context = Context()
    custom_context.init(args=[])
    context_node = None
    # Construct the stock observer before activation replaces rclpy.node.Node.
    # It remains an observation-only node on the explicitly owned context.
    observer = None
    backend_module = None
    try:
        observer = StockNode(
            "diff_observer", context=custom_context, start_parameter_services=False)
        if mode == "activated":
            import rclcppyy
            backend_module = rclcppyy
            rclcppyy.enable_cpp_acceleration()
        context_node = rclpy.create_node(
            "diff_context", context=custom_context, start_parameter_services=False)
        observations["context"] = {
            "node_uses_requested_context": context_node.context is custom_context,
            "requested_context_ok": custom_context.ok(),
            "default_context_ok": rclpy.get_default_context().ok(),
        }
        _destroy_and_collect(context_node)
        context_node = None

        observations["identity"] = _observe_identity(
            rclpy, observer, custom_context)
        observations["remapping"] = _observe_remapping(
            rclpy, observer, custom_context)
        observations["parameters"] = _observe_parameters(
            rclpy, observer, custom_context, Parameter)
        observations["entity_routing"] = _observe_entity_routing(
            rclpy, custom_context)
        observations["teardown"] = _observe_teardown(
            rclpy, observer, custom_context)
    finally:
        if context_node is not None:
            context_node.destroy_node()
        if observer is not None:
            observer.destroy_node()
            cleanup["observer_destroyed"] = True
        if custom_context.ok():
            custom_context.try_shutdown()
        cleanup["custom_context_shutdown"] = not custom_context.ok()
        if rclpy.get_default_context().ok():
            rclpy.try_shutdown()
        cleanup["default_context_shutdown"] = not rclpy.get_default_context().ok()
        gc.collect()

    expectations = []
    backend_status = None
    if backend_module is not None:
        expectations = [
            {
                "kind": "operations",
                "backend": "cpp",
                "minimum": 1,
                "metadata": {"operation": "enable_cpp_acceleration"},
            },
            {
                "kind": "nodes",
                "backend": "python",
                "minimum": 1,
                "metadata": {"profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "cpp",
                "minimum": 1,
                "metadata": {
                    "entity_type": "publisher",
                    "profile": "compatible",
                },
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {
                    "entity_type": "subscription",
                    "profile": "compatible",
                },
            },
        ]
        backend_status = backend_module.status()
    backend_verified = (
        not expectations or not verify_backend_expectations(backend_status, expectations))
    return observations, expectations, backend_status, backend_verified, cleanup


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True, choices=("stock", "activated"))
    args = parser.parse_args(argv)

    try:
        observations, expectations, status, verified, cleanup = _run(args.mode)
        result = {
            "schema": "rclcppyy.differential/v1",
            "scenario": "core_node_contract",
            "mode": args.mode,
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
            "scenario": "core_node_contract",
            "mode": args.mode,
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
