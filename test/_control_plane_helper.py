#!/usr/bin/env python3
"""Compatible control-plane fallbacks preserve objects and warn once."""

import inspect
import warnings

import rclcppyy
from rclcppyy import monkey

rclcppyy.enable_cpp_acceleration(profile="compatible", warn_fallback=True)

import rclpy  # noqa: E402
from rclpy.client import Client  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.guard_condition import GuardCondition  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.service import Service  # noqa: E402
from rclpy.task import Future  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


def _service(_request, response):
    response.success = True
    return response


def main():
    context = rclpy.context.Context()
    context.init(args=[])
    node = rclpy.create_node("control_plane", context=context)
    node.declare_parameter("value", 0)

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        services = [
            node.create_service(SetBool, "service_%d" % index, _service)
            for index in range(2)
        ]
        clients = [
            node.create_client(SetBool, "service_%d" % index)
            for index in range(2)
        ]
        guards = [node.create_guard_condition(lambda: None) for _index in range(2)]
        node.set_parameters([Parameter("value", value=1)])
        node.set_parameters([Parameter("value", value=2)])

    assert all(type(service) is Service for service in services)
    assert all(type(client) is Client for client in clients)
    assert all(type(guard) is GuardCondition for guard in guards)
    assert node.get_parameter("value").value == 2
    fallback_warnings = [
        warning for warning in caught
        if "rclcppyy fell back to stock rclpy" in str(warning.message)
    ]
    assert len(fallback_warnings) == 4, [str(warning.message) for warning in caught]

    original_methods = {
        "create_service": monkey._original_create_service,
        "create_client": monkey._original_create_client,
        "create_guard_condition": monkey._original_create_guard_condition,
        "set_parameters": monkey._original_set_parameters,
        "set_parameters_atomically": monkey._original_set_parameters_atomically,
    }
    assert all(
        inspect.signature(getattr(Node, name)) == inspect.signature(original)
        for name, original in original_methods.items()
    )
    runtime_functions = {
        rclpy.spin: monkey._original_spin,
        rclpy.spin_once: monkey._original_spin_once,
        MultiThreadedExecutor.spin: monkey._original_multi_threaded_spin,
        Future.set_result: monkey._original_future_set_result,
        Future.set_exception: monkey._original_future_set_exception,
        Future.cancel: monkey._original_future_cancel,
    }
    assert all(
        function.__name__ == original.__name__
        and inspect.signature(function) == inspect.signature(original)
        for function, original in runtime_functions.items()
    )

    status = rclcppyy.status()
    entity_counts = {
        entity_type: len([
            record for record in status["entities"]
            if record["backend"] == "python"
            and record["metadata"].get("entity_type") == entity_type
            and "stock_fallback" in record["policies"]
        ])
        for entity_type in ("service", "client", "guard_condition")
    }
    assert entity_counts == {"service": 2, "client": 2, "guard_condition": 2}
    parameter_records = [
        record for record in status["operations"]
        if record["backend"] == "python"
        and record["metadata"].get("operation") == "set_parameters"
    ]
    assert len(parameter_records) == 2
    assert all(record["metadata"]["successful"] for record in parameter_records)
    print("CONTROL_PLANE_STATUS_OK", flush=True)
    print("CONTROL_PLANE_WARN_ONCE_OK", flush=True)
    print("CONTROL_PLANE_SIGNATURES_OK", flush=True)

    for client in clients:
        node.destroy_client(client)
    for service in services:
        node.destroy_service(service)
    for guard in guards:
        node.destroy_guard_condition(guard)
    node.destroy_node()
    context.shutdown()


if __name__ == "__main__":
    main()
