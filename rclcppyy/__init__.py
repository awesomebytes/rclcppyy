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


_ACTIVE_PROFILE = None
_ACTIVE_OPTIMIZATIONS = ()
_ACTIVE_INTERFACES = ()
_SUPPORTED_OPTIMIZATIONS = frozenset(("subscription_shared_lease",))

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


def _normalize_optimizations(optimizations):
    if optimizations is None:
        return ()
    values = (optimizations,) if isinstance(optimizations, str) else optimizations
    try:
        selected = tuple(values)
    except TypeError as exc:
        raise TypeError("optimizations must be an iterable of strings") from exc
    if any(not isinstance(value, str) or not value for value in selected):
        raise TypeError("optimizations must contain only non-empty strings")
    normalized = tuple(sorted(set(selected)))
    unknown = sorted(set(normalized) - _SUPPORTED_OPTIMIZATIONS)
    if unknown:
        raise ValueError(
            "unknown C++ acceleration optimization(s): %s" % ", ".join(unknown))
    return normalized


def enable_cpp_acceleration(
    patch_node=True,
    *,
    profile="compatible",
    warn_fallback=False,
    optimizations=(),
    interfaces=(),
):
    """
    Enable C++ acceleration for ROS2 Python code.

    The compatible profile keeps stock rclpy nodes, contexts, executors, message
    classes, entity objects, and ``Publisher.publish`` authoritative. The explicit
    ``publisher_cpp`` profile enables the same-handle C++ publisher route; other
    unsupported operations remain stock and are visible in :func:`status`. The
    Jazzy/Cyclone-only ``message_facade`` profile additionally gives certified
    scalar/string messages C++ ownership through publish and subscription take.

    Args:
        patch_node (bool): Retained for source compatibility. Node identity is no
                          longer replaced in any profile.
        profile (str): ``compatible``, ``publisher_cpp``, ``message_facade``,
                       ``required_cpp``, ``optimized``, or ``direct_cpp``.
        warn_fallback (bool): Warn once for each stock fallback reason.
        optimizations (Iterable[str]): Explicit opt-in C++ optimizations. The
            ``subscription_shared_lease`` option is valid only with ``direct_cpp``.
        interfaces (Iterable[str]): Additional canonical ``package/msg/Message``
            or ``package/srv/Service`` interfaces to expose as generated C++
            classes under ``direct_cpp``.

    Returns:
        bool: True if successful

    Example:
        ```python
        import rclcppyy; rclcppyy.enable_cpp_acceleration()
        ```
    """
    global _ACTIVE_INTERFACES, _ACTIVE_OPTIMIZATIONS, _ACTIVE_PROFILE
    normalized_optimizations = _normalize_optimizations(optimizations)
    from rclcppyy.direct_services import normalize_registered_interfaces

    normalized_interfaces = normalize_registered_interfaces(interfaces)
    if normalized_optimizations and profile != "direct_cpp":
        raise ValueError("C++ acceleration optimizations require profile='direct_cpp'")
    if normalized_interfaces and profile != "direct_cpp":
        raise ValueError("C++ interfaces require profile='direct_cpp'")
    if _ACTIVE_PROFILE is not None:
        if profile != _ACTIVE_PROFILE:
            raise RuntimeError(
                "rclcppyy is already active with profile %r" % _ACTIVE_PROFILE)
        if normalized_optimizations != _ACTIVE_OPTIMIZATIONS:
            raise RuntimeError(
                "rclcppyy is already active with optimizations %r" %
                (_ACTIVE_OPTIMIZATIONS,))
        if normalized_interfaces != _ACTIVE_INTERFACES:
            raise RuntimeError(
                "rclcppyy is already active with interfaces %r" %
                (_ACTIVE_INTERFACES,))
        return True

    if profile == "direct_cpp":
        from rclcppyy.direct_cpp import activate

        result = activate(
            optimizations=normalized_optimizations,
            interfaces=normalized_interfaces,
        )
        _ACTIVE_PROFILE = profile
        _ACTIVE_OPTIMIZATIONS = normalized_optimizations
        _ACTIVE_INTERFACES = normalized_interfaces
        return result

    from rclcppyy.monkey import patch_node_class, patch_ros2

    result = patch_ros2(profile=profile, warn_fallback=warn_fallback)
    _ACTIVE_PROFILE = profile
    _ACTIVE_OPTIMIZATIONS = normalized_optimizations
    _ACTIVE_INTERFACES = normalized_interfaces

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
