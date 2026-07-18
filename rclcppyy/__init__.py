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
import sys
from types import ModuleType

from rclcppyy._status import status
from rclcppyy._status import record_decision
from rclcppyy.policy import AccelerationPolicy, BackendUnavailableError

_MOVED_SUBMODULES = ("serialization", "rosbag2_cpp", "rosbag2_py_compat", "tf")
_LAZY_EXPORTS = {
    "bringup_rclcpp": ("rclcppyy.bringup_rclcpp", "bringup_rclcpp"),
    "shutdown_rclcpp": ("rclcppyy.bringup_rclcpp", "shutdown_rclcpp"),
    "RclcppyyNode": ("rclcppyy.node", "RclcppyyNode"),
    "Node": ("rclcppyy.node", "RclcppyyNode"),
    "patch_ros2": ("rclcppyy.monkey", "patch_ros2"),
    "patch_node_class": ("rclcppyy.monkey", "patch_node_class"),
    "NativeCapabilities": ("rclcpp_kit.native", "NativeCapabilities"),
    "NativeSession": ("rclcpp_kit.native", "NativeSession"),
    "publisher_capabilities": ("rclcpp_kit.native", "publisher_capabilities"),
}


class _RclcppyyModule(ModuleType):
    def __getattribute__(self, name):
        value = super().__getattribute__(name)
        if name == "bringup_rclcpp" and isinstance(value, ModuleType):
            return value.bringup_rclcpp
        return value


# Importing the legacy ``rclcppyy.bringup_rclcpp`` submodule makes importlib bind
# that module on its parent package. Preserve the historical callable package
# export regardless of whether the submodule or package attribute is imported first.
sys.modules[__name__].__class__ = _RclcppyyModule


def __getattr__(name):
    if name in _MOVED_SUBMODULES:
        module = importlib.import_module(f"rclcppyy.{name}")
        globals()[name] = module
        return module
    target = _LAZY_EXPORTS.get(name)
    if target is not None:
        module_name, attribute = target
        value = getattr(importlib.import_module(module_name), attribute)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | set(__all__))


def enable_cpp_acceleration(
    patch_node=True,
    *,
    profile="compatible",
    warn_fallback=False,
):
    """
    Enable C++ acceleration for ROS2 Python code.

    The compatible profile keeps stock rclpy nodes, contexts, executors, message
    classes, entity objects, and ``Publisher.publish`` authoritative. The explicit
    ``publisher_cpp`` profile enables the same-handle C++ publisher route; other
    unsupported operations remain stock and are visible in :func:`status`.

    Args:
        patch_node (bool): Retained for source compatibility. Node identity is no
                          longer replaced in any profile.
        profile (str): ``compatible``, ``publisher_cpp``, ``required_cpp``, or
                       ``optimized``.
        warn_fallback (bool): Warn once for each stock fallback reason.

    Returns:
        bool: True if successful

    Example:
        ```python
        import rclcppyy; rclcppyy.enable_cpp_acceleration()
        ```
    """
    from rclcppyy.monkey import patch_node_class, patch_ros2

    result = patch_ros2(profile=profile, warn_fallback=warn_fallback)

    # Optionally patch the Node class directly
    if patch_node:
        patch_node_class()

    return result


def native(arguments=None):
    """Create a managed session whose entities are the real C++ objects."""
    from rclcpp_kit.native import native as native_session

    normalized_arguments = tuple(arguments or ())
    session = native_session(arguments=normalized_arguments)
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
