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
from rclcppyy._status import status
from rclcppyy._status import record_decision
from rclcppyy.node import RclcppyyNode
from rclcppyy.monkey import patch_ros2, patch_node_class
from rclcppyy.policy import AccelerationPolicy, BackendUnavailableError
from rclcpp_kit.native import (
    NativeCapabilities,
    NativeSession,
    native as _native_session,
    publisher_capabilities,
)
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


def enable_cpp_acceleration(
    patch_node=True,
    *,
    profile="compatible",
    warn_fallback=False,
):
    """
    Enable C++ acceleration for ROS2 Python code.

    The compatible profile keeps stock rclpy nodes, contexts, executors, message
    classes, and entity objects authoritative. Certified operations use C++ over
    their existing native handles; unsupported operations remain stock and are
    visible in :func:`status`.

    Args:
        patch_node (bool): Retained for source compatibility. Node identity is no
                          longer replaced in any profile.
        profile (str): ``compatible``, ``required_cpp``, or ``optimized``.
        warn_fallback (bool): Warn once for each stock fallback reason.

    Returns:
        bool: True if successful

    Example:
        ```python
        import rclcppyy; rclcppyy.enable_cpp_acceleration()
        ```
    """
    # Apply monkey patching
    result = patch_ros2(profile=profile, warn_fallback=warn_fallback)

    # Optionally patch the Node class directly
    if patch_node:
        patch_node_class()

    return result


def native(arguments=None):
    """Create a managed session whose entities are the real C++ objects."""
    normalized_arguments = tuple(arguments or ())
    session = _native_session(arguments=normalized_arguments)
    record_decision(
        "operations",
        "cpp",
        "created managed native rclcpp session",
        policies=("native", "explicit_opt_in"),
        metadata={
            "operation": "native",
            "arguments_count": len(normalized_arguments),
            "capabilities": session.capabilities.to_dict(),
        },
    )
    return session


__all__ = [
    'bringup_rclcpp',
    'shutdown_rclcpp',
    'status',
    'RclcppyyNode',
    'Node',
    'enable_cpp_acceleration',
    'patch_ros2',
    'patch_node_class',
    'AccelerationPolicy',
    'BackendUnavailableError',
    'NativeCapabilities',
    'NativeSession',
    'native',
    'publisher_capabilities',
    'rosbag2_cpp',
    'serialization',
    'rosbag2_py_compat',
    'tf',
]
