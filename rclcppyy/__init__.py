"""
rclcppyy - High-performance ROS 2 with a Python API and C++ internals.

As of 0.2.0, rclcppyy is the drop-in **rclpy accelerator product** built on top of
the cppyy_kit suite. The rclcpp core it used to carry (bringup, message conversion,
serialization, rosbag2, tf) now lives in the standalone ``rclcpp_kit`` package, and
the cppyy "kits" live in their own packages; rclcppyy keeps the brand and the
one-line monkeypatch (``enable_cpp_acceleration``) and re-exports the moved pieces
through deprecation shims. See the README for the new architecture.
"""
import importlib

# Public API. ``bringup_rclcpp`` / ``shutdown_rclcpp`` come from the (silent)
# bringup shim, which node.py / monkey.py also use internally.
from rclcppyy.bringup_rclcpp import bringup_rclcpp, shutdown_rclcpp
from rclcppyy.node import RclcppyyNode
from rclcppyy.monkey import patch_ros2, patch_node_class
Node = RclcppyyNode

# The moved re-export submodules are imported lazily: ``import rclcppyy`` must not
# fire their DeprecationWarnings for the product's own use, but accessing
# ``rclcppyy.tf`` (or ``from rclcppyy import tf`` / ``rclcppyy.serialization`` ...)
# imports the shim and warns, nudging callers to ``rclcpp_kit.<name>``.
_MOVED_SUBMODULES = ("serialization", "rosbag2_cpp", "rosbag2_py_compat", "tf")


def __getattr__(name):
    if name in _MOVED_SUBMODULES:
        module = importlib.import_module(f"rclcppyy.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(list(globals()) + list(_MOVED_SUBMODULES))


def enable_cpp_acceleration(patch_node=True):
    """
    Enable C++ acceleration for ROS2 Python code.

    This will:
    1. Set up automatic message conversion from Python to C++
    2. Monkey-patch rclpy.create_node to return RclcppyyNode
    3. Monkey-patch ROS2 message imports to use C++ versions

    Args:
        patch_node (bool): If True, also monkey-patch rclpy.node.Node class directly.
                          This is more aggressive but ensures all nodes use C++.

    Returns:
        bool: True if successful

    Example:
        ```python
        import rclpy
        from std_msgs.msg import String

        # Add this single line to your existing ROS2 Python code:
        import rclcppyy; rclcppyy.enable_cpp_acceleration()

        # Rest of your code remains unchanged
        rclpy.init()
        node = rclpy.create_node('my_node')  # Actually returns a RclcppyyNode
        ```
    """
    # Apply monkey patching
    result = patch_ros2()

    # Optionally patch the Node class directly
    if patch_node:
        patch_node_class()

    return result


__all__ = [
    'bringup_rclcpp',
    'shutdown_rclcpp',
    'RclcppyyNode',
    'Node',
    'enable_cpp_acceleration',
    'patch_ros2',
    'patch_node_class',
    'rosbag2_cpp',
    'serialization',
    'rosbag2_py_compat',
    'tf',
]
