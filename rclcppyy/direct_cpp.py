"""Bounded, source-compatible control plane over direct ``rclcpp`` entities."""

from __future__ import annotations

import math
import os
import sys

import cppyy

from rclcppyy._status import record_decision
from rclcppyy.policy import BackendUnavailableError


_ACTIVE = False
_RUNTIME = None
_PATCHES = ()
_MESSAGE_INSTALLATION = None


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


class _DirectContext:
    def __init__(self, runtime):
        self._runtime = runtime

    def ok(self) -> bool:
        return self._runtime.ok()

    def shutdown(self) -> None:
        self._runtime.shutdown()


class _DirectRuntime:
    def __init__(self):
        self.session = None
        self.executor = None
        self.nodes = []
        self.context = _DirectContext(self)
        self._shutting_down = False

    def init(self, arguments) -> None:
        if self.ok():
            raise RuntimeError("direct_cpp context is already initialized")
        from rclcpp_kit.native import NativeSession

        self._shutting_down = False
        self.session = NativeSession(arguments=arguments).open()
        self.executor = self.session.create_executor("single_threaded")

    def ok(self) -> bool:
        if self._shutting_down or self.session is None or self.session.closed:
            return False
        return bool(self.session.context.is_valid())

    def require_session(self):
        if not self.ok():
            raise RuntimeError("rclpy.init() must be called before creating a direct_cpp node")
        return self.session

    def attach(self, facade, node) -> None:
        self.require_session()
        self.executor.add_node(node)
        self.nodes.append(facade)

    def detach(self, facade, node) -> None:
        if facade in self.nodes:
            self.nodes.remove(facade)
        if self.session is not None and not self.session.closed:
            self.session.release_node(node)

    def spin_once(self, node, timeout_sec) -> None:
        if node not in self.nodes or node._direct_cpp_node is None:
            raise ValueError("node is not owned by the active direct_cpp context")
        executor = self.executor
        if executor is None or not self.ok():
            raise RuntimeError("direct_cpp context is not initialized")
        if timeout_sec is None or timeout_sec < 0:
            executor.spin_once()
        else:
            duration = cppyy.gbl.std.chrono.nanoseconds(int(timeout_sec * 1e9))
            executor.spin_once(duration)

    def spin(self, node) -> None:
        if node not in self.nodes or node._direct_cpp_node is None:
            raise ValueError("node is not owned by the active direct_cpp context")
        while self.ok():
            try:
                self.spin_once(node, 0.1)
            except Exception:
                if self._shutting_down or not self.ok():
                    return
                raise

    def shutdown(self) -> None:
        if self.session is None:
            return
        self._shutting_down = True
        if self.executor is not None:
            self.executor.cancel()
        for node in tuple(self.nodes):
            node._mark_runtime_shutdown()
        self.nodes.clear()
        self.session.close("rclcppyy direct_cpp shutdown")
        self.executor = None
        self.session = None


class DirectNode:
    """Small rclpy-style facade whose data-plane entities are real C++ objects."""

    def __init__(
        self,
        node_name: str,
        *,
        context=None,
        cli_args=None,
        namespace=None,
        use_global_arguments=True,
        enable_rosout=True,
        start_parameter_services=True,
        parameter_overrides=None,
        allow_undeclared_parameters=False,
        automatically_declare_parameters_from_overrides=False,
        enable_logger_service=False,
    ):
        unsupported = {
            "context": context is not None,
            "cli_args": cli_args is not None,
            "use_global_arguments": use_global_arguments is not True,
            "enable_rosout": enable_rosout is not True,
            "start_parameter_services": start_parameter_services is not True,
            "parameter_overrides": parameter_overrides is not None,
            "allow_undeclared_parameters": bool(allow_undeclared_parameters),
            "automatically_declare_parameters_from_overrides": bool(
                automatically_declare_parameters_from_overrides),
            "enable_logger_service": bool(enable_logger_service),
        }
        requested = sorted(name for name, value in unsupported.items() if value)
        if requested:
            _unsupported(
                "direct_cpp node does not support constructor option(s): %s" %
                ", ".join(requested)
            )
        session = _runtime().require_session()
        self._direct_cpp_node = session.create_node(
            str(node_name), namespace=str(namespace or ""))
        self._direct_cpp_publishers = []
        self._direct_cpp_subscriptions = []
        self._direct_cpp_timers = []
        _runtime().attach(self, self._direct_cpp_node)
        record_decision(
            "nodes",
            "cpp",
            "direct_cpp owns one NativeSession rclcpp node",
            policies=("direct_cpp", "native_node_authority"),
            metadata={"name": str(node_name), "namespace": str(namespace or "")},
        )

    @property
    def context(self):
        return _runtime().context

    @property
    def publishers(self):
        return list(self._direct_cpp_publishers)

    @property
    def subscriptions(self):
        return [item.entity for item in self._direct_cpp_subscriptions]

    @property
    def timers(self):
        return list(self._direct_cpp_timers)

    def get_name(self):
        return str(self._require_node().get_name())

    def get_namespace(self):
        return str(self._require_node().get_namespace())

    def get_fully_qualified_name(self):
        return str(self._require_node().get_fully_qualified_name())

    def get_logger(self):
        return self._require_node().get_logger()

    def create_publisher(
        self,
        msg_type,
        topic,
        qos_profile,
        *,
        callback_group=None,
        event_callbacks=None,
        qos_overriding_options=None,
        publisher_class=None,
    ):
        requested = {
            "callback_group": callback_group is not None,
            "event_callbacks": event_callbacks is not None,
            "qos_overriding_options": qos_overriding_options is not None,
            "publisher_class": publisher_class is not None,
        }
        self._reject_entity_options("publisher", requested)
        from rclcpp_kit import direct_entities

        direct_entities.resolve_supported_type(msg_type)
        qos = direct_entities.qos_from_depth(
            _runtime().session.rclcpp, qos_profile)
        publisher = direct_entities.create_publisher(
            self._require_node(), msg_type, str(topic), qos)
        self._direct_cpp_publishers.append(publisher)
        self._record_entity("publisher", topic, msg_type)
        return publisher

    def create_subscription(
        self,
        msg_type,
        topic,
        callback,
        qos_profile,
        *,
        callback_group=None,
        event_callbacks=None,
        qos_overriding_options=None,
        raw=False,
        content_filter_options=None,
    ):
        requested = {
            "callback_group": callback_group is not None,
            "event_callbacks": event_callbacks is not None,
            "qos_overriding_options": qos_overriding_options is not None,
            "raw": bool(raw),
            "content_filter_options": content_filter_options is not None,
        }
        self._reject_entity_options("subscription", requested)
        from rclcpp_kit import direct_entities

        direct_entities.resolve_supported_type(msg_type)
        if not callable(callback):
            raise TypeError("subscription callback must be callable")
        qos = direct_entities.qos_from_depth(
            _runtime().session.rclcpp, qos_profile)
        subscription = direct_entities.create_subscription(
            self._require_node(), msg_type, str(topic), callback, qos)
        self._direct_cpp_subscriptions.append(subscription)
        self._record_entity("subscription", topic, msg_type)
        return subscription.entity

    def create_timer(
        self,
        timer_period_sec,
        callback,
        callback_group=None,
        clock=None,
        autostart=True,
        **options,
    ):
        requested = {
            "callback_group": callback_group is not None,
            "clock": clock is not None,
            "autostart": autostart is not True,
            **{str(name): True for name in options},
        }
        self._reject_entity_options("timer", requested)
        if not callable(callback):
            raise TypeError("timer callback must be callable")
        if (
            isinstance(timer_period_sec, bool)
            or not isinstance(timer_period_sec, (int, float))
        ):
            raise TypeError("timer period must be a finite positive number")
        period = float(timer_period_sec)
        if not math.isfinite(period) or period <= 0:
            raise ValueError("timer period must be a finite positive number")
        period_ns = int(period * 1e9)
        if period_ns <= 0:
            raise ValueError("timer period must be at least one nanosecond")

        from rclcpp_kit import direct_entities

        timer = direct_entities.create_wall_timer(
            self._require_node(), period_ns, callback)
        self._direct_cpp_timers.append(timer)
        record_decision(
            "entities",
            "cpp",
            "direct rclcpp wall timer with Python callback",
            policies=("direct_cpp", "native_timer_authority", "no_conversion"),
            metadata={
                "entity_type": "timer",
                "period_ns": period_ns,
                "clock": "steady",
                "callback_handoff": "direct_std_function",
                "creation_route": timer.creation_route,
                "native_type": timer.native_type_name,
            },
        )
        return timer

    def destroy_timer(self, timer):
        for index, candidate in enumerate(self._direct_cpp_timers):
            if timer is candidate:
                candidate.destroy()
                del self._direct_cpp_timers[index]
                return True
        return False

    def destroy_node(self):
        node = self._direct_cpp_node
        if node is None:
            return
        for timer in tuple(self._direct_cpp_timers):
            timer.destroy()
        self._direct_cpp_timers.clear()
        self._direct_cpp_publishers.clear()
        self._direct_cpp_subscriptions.clear()
        _runtime().detach(self, node)
        self._direct_cpp_node = None

    def _mark_runtime_shutdown(self):
        for timer in tuple(self._direct_cpp_timers):
            timer.destroy()
        self._direct_cpp_timers.clear()
        self._direct_cpp_publishers.clear()
        self._direct_cpp_subscriptions.clear()
        self._direct_cpp_node = None

    def _require_node(self):
        if self._direct_cpp_node is None:
            raise RuntimeError("direct_cpp node is destroyed")
        return self._direct_cpp_node

    def _reject_entity_options(self, entity_type, requested):
        selected = sorted(name for name, value in requested.items() if value)
        if selected:
            _unsupported(
                "direct_cpp %s does not support option(s): %s" %
                (entity_type, ", ".join(selected))
            )

    def _record_entity(self, entity_type, topic, msg_type):
        policies = ["direct_cpp", "direct_cpp_message", "no_conversion"]
        metadata = {
            "entity_type": entity_type,
            "topic": str(topic),
            "message_type": str(getattr(msg_type, "__cpp_name__", msg_type)),
        }
        if entity_type == "subscription":
            policies.append("owning_cpp_callback_copy")
            metadata["callback_handoff"] = "one_native_cpp_copy"
        record_decision(
            "entities",
            "cpp",
            "direct typed rclcpp %s with C++ message storage" % entity_type,
            policies=policies,
            metadata=metadata,
        )


def _runtime() -> _DirectRuntime:
    if _RUNTIME is None:
        raise RuntimeError("direct_cpp is not active")
    return _RUNTIME


def _check_runtime() -> None:
    if os.environ.get("ROS_DISTRO") != "jazzy":
        _unsupported("direct_cpp requires ROS_DISTRO=jazzy")
    import rclpy

    implementation = rclpy.get_rmw_implementation_identifier()
    if implementation != "rmw_cyclonedds_cpp":
        _unsupported(
            "direct_cpp requires rmw_cyclonedds_cpp, got %s" % implementation)


def _check_early_activation() -> None:
    stale = sorted(
        name for name in ("rclpy.node", "rclpy.executors") if name in sys.modules)
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing: %s" % ", ".join(stale))
    from rclcppyy.direct_messages import assert_early_imports

    assert_early_imports()


def _direct_init(
    *, args=None, context=None, domain_id=None, signal_handler_options=None
):
    if context is not None or domain_id is not None or signal_handler_options is not None:
        _unsupported("direct_cpp init currently supports only args and the default context")
    _runtime().init(tuple(args or ()))


def _direct_ok(*, context=None):
    if context is not None and context is not _runtime().context:
        _unsupported("direct_cpp does not accept a stock or foreign Context")
    return _runtime().ok()


def _direct_shutdown(*, context=None, uninstall_handlers=None):
    if context is not None and context is not _runtime().context:
        _unsupported("direct_cpp does not accept a stock or foreign Context")
    if uninstall_handlers is not None:
        _unsupported("direct_cpp does not expose rclpy signal-handler ownership")
    _runtime().shutdown()


def _direct_try_shutdown(*, context=None, uninstall_handlers=None):
    if _runtime().ok():
        _direct_shutdown(context=context, uninstall_handlers=uninstall_handlers)


def _direct_spin_once(node, *, executor=None, timeout_sec=None):
    if executor is not None:
        _unsupported("direct_cpp first slice does not accept a public executor")
    _runtime().spin_once(node, timeout_sec)


def _direct_spin(node, executor=None):
    if executor is not None:
        _unsupported("direct_cpp does not accept a public executor")
    _runtime().spin(node)


def activate() -> bool:
    """Install the complete first-slice surface, rolling back on any failure."""
    global _ACTIVE, _MESSAGE_INSTALLATION, _PATCHES, _RUNTIME
    if _ACTIVE:
        return True
    _check_early_activation()
    _check_runtime()

    from rclcppyy import direct_messages

    installation = direct_messages.install()
    patches = []
    try:
        import rclpy
        import rclpy.node as node_module

        runtime = _DirectRuntime()
        replacements = (
            (node_module, "Node", DirectNode),
            (rclpy, "init", _direct_init),
            (rclpy, "ok", _direct_ok),
            (rclpy, "shutdown", _direct_shutdown),
            (rclpy, "try_shutdown", _direct_try_shutdown),
            (rclpy, "spin_once", _direct_spin_once),
            (rclpy, "spin", _direct_spin),
        )
        for module, name, replacement in replacements:
            original = getattr(module, name)
            setattr(module, name, replacement)
            patches.append((module, name, original, replacement))
        _RUNTIME = runtime
    except Exception:
        for module, name, original, replacement in reversed(patches):
            if getattr(module, name, None) is replacement:
                setattr(module, name, original)
        installation.restore()
        raise
    _MESSAGE_INSTALLATION = installation
    _PATCHES = tuple(patches)
    _ACTIVE = True
    record_decision(
        "operations",
        "cpp",
        "installed explicit direct_cpp source-compatible slice",
        policies=("direct_cpp", "jazzy", "cyclonedds", "no_conversion"),
        metadata={
            "operation": "enable_cpp_acceleration",
            "profile": "direct_cpp",
            "message_types": [binding.cpp_type_name for binding in installation.bindings],
        },
    )
    return True


__all__ = ["DirectNode", "activate"]
