#!/usr/bin/env python3
"""Required-C++ policy rejects uncertified operations without partial entities."""

import rclcppyy
from rclcppyy import BackendUnavailableError

rclcppyy.enable_cpp_acceleration(profile="required_cpp")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


def _must_reject_without_entity(node, collection_name, create):
    before = len(list(getattr(node, collection_name)))
    try:
        create()
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("required-C++ operation unexpectedly fell back")
    assert len(list(getattr(node, collection_name))) == before


def main():
    context = rclpy.context.Context()
    context.init(args=[])
    node = rclpy.create_node("required_cpp", context=context)
    assert type(node) is Node

    publisher = node.create_publisher(String, "required_cpp", 10)
    assert type(publisher) is Publisher
    assert hasattr(publisher, "_rclcppyy_publish_route")
    print("REQUIRED_PUBLISHER_OK", flush=True)

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
    } <= rejected, status
    infrastructure = [
        record for record in status["entities"]
        if record["backend"] == "python"
        and record["metadata"].get("entity_type") == "service"
        and "stock_node_infrastructure" in record["policies"]
    ]
    assert infrastructure, status
    assert all(record["metadata"]["profile"] == "required_cpp"
               for record in infrastructure)
    node.destroy_publisher(publisher)
    node.destroy_node()
    context.shutdown()


if __name__ == "__main__":
    main()
