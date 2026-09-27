#!/usr/bin/env python3
"""Required-C++ policy rejects uncertified operations without partial entities."""

from rclpy.task import Future

STOCK_FUTURE_METHODS = (Future.set_result, Future.set_exception, Future.cancel)

import rclcppyy  # noqa: E402
from rclcppyy import BackendUnavailableError  # noqa: E402

rclcppyy.enable_cpp_acceleration(profile="required_cpp")

import rclpy  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.lifecycle import LifecycleNode  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
from rclcppyy import monkey as monkey_module  # noqa: E402


def _must_reject_without_entity(node, collection_name, create):
    before = len(list(getattr(node, collection_name)))
    try:
        create()
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("required-C++ operation unexpectedly fell back")
    assert len(list(getattr(node, collection_name))) == before


def _must_reject(operation):
    try:
        operation()
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("required-C++ runtime operation unexpectedly fell back")


def main():
    borrowed_publish, unavailable_reason = monkey_module._load_borrowed_publish()
    assert borrowed_publish is not None, unavailable_reason
    prepare_calls = []

    class TrackingBorrowedPublish:
        @staticmethod
        def prepare(msg_type):
            prepare_calls.append(msg_type)
            return borrowed_publish.prepare(msg_type)

    monkey_module._load_borrowed_publish = lambda: (TrackingBorrowedPublish, None)
    context = rclpy.context.Context()
    context.init(args=[])
    node = rclpy.create_node("required_cpp", context=context)
    assert type(node) is Node
    assert prepare_calls == [], prepare_calls

    publisher = node.create_publisher(String, "required_cpp", 10)
    assert prepare_calls == [String], prepare_calls
    assert type(publisher) is Publisher
    assert hasattr(publisher, "_rclcppyy_publish_route")
    print("REQUIRED_PUBLISHER_OK", flush=True)
    print("REQUIRED_CONSTRUCTOR_PUBLISHER_STOCK_OK", flush=True)

    observed = []
    proof_executor = SingleThreadedExecutor(context=context)
    proof_executor.add_node(node)
    subscription = monkey_module._original_create_subscription(
        node, String, "required_cpp", lambda message: observed.append(message.data), 10)
    publisher.publish(String(data="cpp-route"))
    for _ in range(20):
        proof_executor.spin_once(timeout_sec=0.05)
        if observed:
            break
    assert observed == ["cpp-route"]

    class RejectingRoute:
        def publish(self, _publisher, _message):
            raise TypeError("deliberate route rejection")

    publisher._rclcppyy_publish_route = RejectingRoute()
    _must_reject(lambda: publisher.publish(String(data="must-not-fallback")))
    for _ in range(3):
        proof_executor.spin_once(timeout_sec=0.01)
    assert observed == ["cpp-route"]
    print("REQUIRED_PUBLISH_FAIL_CLOSED_OK", flush=True)

    _must_reject_without_entity(
        node,
        "subscriptions",
        lambda: node.create_subscription(
            String, "required_cpp", lambda message: None, 10),
    )
    print("REQUIRED_FAIL_CLOSED_OK", flush=True)

    _must_reject_without_entity(
        node, "timers", lambda: node.create_timer(1.0, lambda: None))
    _must_reject_without_entity(
        node,
        "services",
        lambda: node.create_service(
            SetBool, "required_service", lambda request, response: response),
    )
    _must_reject_without_entity(
        node, "clients", lambda: node.create_client(SetBool, "required_service"))
    _must_reject_without_entity(
        node, "guards", lambda: node.create_guard_condition(lambda: None))

    node.declare_parameter("required_value", 1)
    try:
        node.set_parameters([Parameter("required_value", value=2)])
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("required-C++ parameter mutation unexpectedly fell back")
    assert node.get_parameter("required_value").value == 1
    try:
        node.set_parameters_atomically([Parameter("required_value", value=3)])
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("required-C++ atomic parameter mutation unexpectedly fell back")
    assert node.get_parameter("required_value").value == 1

    _must_reject(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    _must_reject(lambda: rclpy.spin(node))
    multi_threaded_executor = MultiThreadedExecutor(
        num_threads=2, context=context)
    _must_reject(multi_threaded_executor.spin)
    multi_threaded_executor.shutdown(timeout_sec=1.0)

    future = Future()
    future.set_result("stock-runtime-authority")
    assert future.result() == "stock-runtime-authority"
    exception_future = Future()
    exception_future.set_exception(ValueError("stock-runtime-authority"))
    assert type(exception_future.exception()) is ValueError
    canceled_future = Future()
    assert canceled_future.cancel() is None
    assert canceled_future.cancelled()
    assert (Future.set_result, Future.set_exception, Future.cancel) == STOCK_FUTURE_METHODS

    names_before_lifecycle = sorted(node.get_node_names())
    _must_reject(lambda: LifecycleNode(
        "required_lifecycle",
        context=context,
        enable_communication_interface=False,
        start_parameter_services=False,
    ))
    assert sorted(node.get_node_names()) == names_before_lifecycle

    waitables_before_action = tuple(node.waitables)
    graph_before_action = (
        sorted(node.get_topic_names_and_types()),
        sorted(node.get_service_names_and_types()),
    )
    _must_reject(lambda: ActionClient(
        node, object, "required_action_client"))
    _must_reject(lambda: ActionServer(
        node, object, "required_action_server"))
    assert tuple(node.waitables) == waitables_before_action
    assert (
        sorted(node.get_topic_names_and_types()),
        sorted(node.get_service_names_and_types()),
    ) == graph_before_action
    print("REQUIRED_CONTROL_PLANE_OK", flush=True)

    status = rclcppyy.status()
    rejected = {
        record["metadata"].get("operation")
        for record in status["operations"]
        if record["backend"] == "unsupported"
    }
    assert {
        "create_subscription",
        "create_timer",
        "create_service",
        "create_client",
        "create_guard_condition",
        "set_parameters",
        "set_parameters_atomically",
        "spin",
        "spin_once",
        "multi_threaded_spin",
        "create_lifecycle_node",
        "create_action_client",
        "create_action_server",
        "publish",
    } <= rejected, status
    infrastructure = [
        record for record in status["entities"]
        if record["backend"] == "python"
        and "stock_node_infrastructure" in record["policies"]
    ]
    assert infrastructure, status
    assert all(record["metadata"]["profile"] == "required_cpp"
               for record in infrastructure)
    assert {"publisher", "service"} <= {
        record["metadata"].get("entity_type") for record in infrastructure
    }, status
    node.destroy_publisher(publisher)
    node.destroy_subscription(subscription)
    proof_executor.remove_node(node)
    proof_executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    context.shutdown()


if __name__ == "__main__":
    main()
