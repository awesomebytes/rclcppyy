#!/usr/bin/env python3
"""Fresh-process stock/direct differential for local parameter reads."""

import argparse
import importlib
import json
import os
import warnings


REPORT_PREFIX = "DIRECT_PARAMETER_GET_REPORT="


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    return parser.parse_args()


def _exception_name(operation):
    try:
        operation()
    except BaseException as exception:
        return type(exception).__name__
    raise AssertionError("parameter operation unexpectedly succeeded")


def main():
    args = _arguments()
    boundary_calls = []
    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

        def forbidden_boundary(*boundary_args, **boundary_kwargs):
            boundary_calls.append((boundary_args, boundary_kwargs))
            raise AssertionError("parameter read used a conversion or serialization path")

        bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
        kit = importlib.import_module("rclcpp_kit")
        serialization = importlib.import_module("rclcpp_kit.serialization")
        rclpy_serialization = importlib.import_module("rclpy.serialization")
        bringup.convert_python_msg_to_cpp = forbidden_boundary
        kit.convert_python_msg_to_cpp = forbidden_boundary
        serialization.serialize_message = forbidden_boundary
        serialization.deserialize_message = forbidden_boundary
        rclpy_serialization.serialize_message = forbidden_boundary
        rclpy_serialization.deserialize_message = forbidden_boundary

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    node = Node("parameter_get_probe_%s_%d" % (args.backend, os.getpid()))
    permissive = Node(
        "parameter_get_permissive_%s_%d" % (args.backend, os.getpid()),
        allow_undeclared_parameters=True,
    )
    retained = None
    try:
        node.declare_parameter("initialized", 7)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            node.declare_parameter("dynamic")
        node.declare_parameter("static", Parameter.Type.INTEGER)

        initialized = node.get_parameter("initialized")
        dynamic = node.get_parameter("dynamic")
        permissive_missing = permissive.get_parameter("missing")
        retained = initialized
        behavior = {
            "initialized": {
                "name": initialized.name,
                "type": int(initialized.type_.value),
                "value": initialized.value,
            },
            "dynamic_not_set": {
                "name": dynamic.name,
                "type": int(dynamic.type_.value),
                "value": dynamic.value,
            },
            "static_uninitialized_exception": _exception_name(
                lambda: node.get_parameter("static")),
            "missing_exception": _exception_name(
                lambda: node.get_parameter("missing")),
            "permissive_missing": {
                "name": permissive_missing.name,
                "type": int(permissive_missing.type_.value),
                "value": permissive_missing.value,
                "declared_after_get": permissive.has_parameter("missing"),
            },
        }

        exact_cpp = False
        if args.backend == "direct":
            import cppyy
            from rclcpp_kit.native_parameters import NativeParameter

            native = initialized._rclcppyy_native_parameter
            exact_cpp = (
                isinstance(native, NativeParameter) and
                type(native.native) is cppyy.gbl.rclcpp.Parameter
            )
    finally:
        node.destroy_node()
        permissive.destroy_node()
        rclpy.shutdown()

    assert retained.value == 7
    assert boundary_calls == []
    report = {
        "backend": args.backend,
        "behavior": behavior,
        "exact_cpp_parameter": exact_cpp,
        "retained_after_teardown": True,
        "forbidden_boundary_calls": len(boundary_calls),
    }
    print(REPORT_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
