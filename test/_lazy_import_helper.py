#!/usr/bin/env python3
"""Fresh-process proofs for rclcppyy's lazy public API imports."""

import sys


HEAVY_MODULES = {
    "cppyy",
    "rclcpp_kit.bringup_rclcpp",
    "rclcpp_kit.native",
    "rclcppyy.bringup_rclcpp",
    "rclcppyy.node",
}


def assert_heavy_absent():
    loaded = sorted(HEAVY_MODULES & set(sys.modules))
    assert not loaded, loaded


def compatible():
    import rclcppyy

    assert_heavy_absent()
    assert set(rclcppyy.__all__) <= set(dir(rclcppyy))
    from rclcppyy import (  # noqa: F401
        AccelerationPolicy,
        BackendUnavailableError,
        status,
    )
    assert_heavy_absent()

    rclcppyy.enable_cpp_acceleration(profile="compatible")
    assert "rclcppyy.monkey" in sys.modules
    assert_heavy_absent()

    from rclpy.publisher import Publisher
    from rclpy.task import Future
    from rclcppyy import monkey
    from rclcppyy import patch_node_class, patch_ros2

    assert patch_ros2 is monkey.patch_ros2
    assert patch_node_class is monkey.patch_node_class
    assert Publisher.publish is monkey._original_publish
    assert Future.set_result is monkey._original_future_set_result
    assert Future.set_exception is monkey._original_future_set_exception
    assert Future.cancel is monkey._original_future_cancel
    assert_heavy_absent()
    print("COMPATIBLE_IMPORT_GRAPH_LIGHT_OK", flush=True)


def bringup():
    import rclcppyy  # noqa: F401

    assert_heavy_absent()
    import importlib

    module = importlib.import_module("rclcppyy.bringup_rclcpp")
    assert callable(rclcppyy.bringup_rclcpp)
    assert rclcppyy.bringup_rclcpp is module.bringup_rclcpp
    from rclcppyy import bringup_rclcpp, shutdown_rclcpp

    assert callable(bringup_rclcpp)
    assert callable(shutdown_rclcpp)
    assert "rclcppyy.bringup_rclcpp" in sys.modules
    assert "rclcpp_kit.bringup_rclcpp" in sys.modules
    print("BRINGUP_EXPORT_LAZY_OK", flush=True)


def legacy_node():
    import rclcppyy  # noqa: F401

    assert_heavy_absent()
    from rclcppyy import Node, RclcppyyNode

    assert Node is RclcppyyNode
    assert "rclcppyy.node" in sys.modules
    print("LEGACY_NODE_EXPORT_LAZY_OK", flush=True)


def native_exports():
    import rclcppyy  # noqa: F401

    assert_heavy_absent()
    from rclcppyy import (
        NativeCapabilities,
        NativeSession,
        native,
        publisher_capabilities,
    )

    assert NativeCapabilities.__module__.startswith("rclcpp_kit")
    assert NativeSession.__module__.startswith("rclcpp_kit")
    assert callable(native)
    assert callable(publisher_capabilities)
    assert "rclcpp_kit.native" in sys.modules
    print("NATIVE_EXPORTS_LAZY_OK", flush=True)


def future_identity():
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile=sys.argv[2])
    from rclpy.task import Future
    from rclcppyy import monkey

    assert Future.set_result is monkey._original_future_set_result
    assert Future.set_exception is monkey._original_future_set_exception
    assert Future.cancel is monkey._original_future_cancel
    print("FUTURE_METHOD_IDENTITY_OK", flush=True)


MODES = {
    "bringup": bringup,
    "compatible": compatible,
    "future_identity": future_identity,
    "legacy_node": legacy_node,
    "native_exports": native_exports,
}


if __name__ == "__main__":
    MODES[sys.argv[1]]()
