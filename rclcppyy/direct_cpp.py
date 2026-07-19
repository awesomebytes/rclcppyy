"""Bounded, source-compatible control plane over direct ``rclcpp`` entities."""

from __future__ import annotations

from enum import Enum
import math
import inspect
import os
import sys
import threading
import time
import weakref

import cppyy

from rclcppyy._status import record_decision
from rclcppyy.policy import BackendUnavailableError


_ACTIVE = False
_RUNTIME = None
_PATCHES = ()
_MESSAGE_INSTALLATION = None
_SERVICE_INSTALLATION = None
_ACTION_INSTALLATION = None
_ACTIVE_OPTIMIZATIONS = ()
_ACTIVE_INTERFACES = ()
_DEFAULT_SERVICE_QOS = object()


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


def _cpp_string(value) -> str:
    if isinstance(value, bytes):
        return value.decode()
    return str(value)


def _graph_names_and_types(values) -> list[tuple[str, list[str]]]:
    return [
        (_cpp_string(row.first), [_cpp_string(value) for value in row.second])
        for row in values
    ]


def _topic_endpoint_info(value):
    from rclpy.duration import Duration
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSLivelinessPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from rclpy.topic_endpoint_info import TopicEndpointInfo
    from rclpy.type_hash import TypeHash

    native_qos = value.qos_profile().get_rmw_qos_profile()

    def policy(enum_type, raw_value):
        try:
            return enum_type(int(raw_value))
        except ValueError:
            return enum_type.UNKNOWN

    def duration(raw_value):
        return Duration(
            seconds=int(raw_value.sec), nanoseconds=int(raw_value.nsec))

    native_hash = value.topic_type_hash()
    qos = QoSProfile(
        history=policy(QoSHistoryPolicy, native_qos.history),
        depth=int(native_qos.depth),
        reliability=policy(QoSReliabilityPolicy, native_qos.reliability),
        durability=policy(QoSDurabilityPolicy, native_qos.durability),
        deadline=duration(native_qos.deadline),
        lifespan=duration(native_qos.lifespan),
        liveliness=policy(QoSLivelinessPolicy, native_qos.liveliness),
        liveliness_lease_duration=duration(
            native_qos.liveliness_lease_duration),
        avoid_ros_namespace_conventions=bool(
            native_qos.avoid_ros_namespace_conventions),
    )
    return TopicEndpointInfo(
        node_name=_cpp_string(value.node_name()),
        node_namespace=_cpp_string(value.node_namespace()),
        topic_type=_cpp_string(value.topic_type()),
        topic_type_hash=TypeHash(
            version=int(native_hash.version),
            value=bytes(native_hash.value),
        ),
        endpoint_type=int(value.endpoint_type()),
        endpoint_gid=[int(item) for item in value.endpoint_gid()],
        qos_profile=qos,
    )


class _DirectContext:
    def __init__(self, runtime):
        self._runtime = runtime

    def ok(self) -> bool:
        return self._runtime.ok()

    def shutdown(self) -> None:
        self._runtime.shutdown()


class _DirectRuntime:
    def __init__(self, optimizations=(), interfaces=()):
        self.session = None
        # Kept for compatibility with existing teardown assertions. Direct nodes
        # are no longer attached to one hidden executor.
        self.executor = None
        self.nodes = []
        self.context = _DirectContext(self)
        self.optimizations = tuple(optimizations)
        self.interfaces = tuple(interfaces)
        self.membership_lock = threading.RLock()
        self._executors = weakref.WeakSet()
        self._global_executor = None
        self._shutting_down = False

    def init(self, arguments) -> None:
        if self.ok():
            raise RuntimeError("direct_cpp context is already initialized")
        from rclcpp_kit.native import NativeSession

        self._shutting_down = False
        self.session = NativeSession(arguments=arguments).open()

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
        with self.membership_lock:
            self.nodes.append(facade)

    def detach(self, facade, node) -> None:
        with self.membership_lock:
            current = facade.executor
            if current is not None:
                current.remove_node(facade)
                facade._set_direct_executor(None)
            if facade in self.nodes:
                self.nodes.remove(facade)
            if self.session is not None and not self.session.closed:
                self.session.release_node(node)

    def register_executor(self, executor) -> None:
        with self.membership_lock:
            self._executors.add(executor)

    def unregister_executor(self, executor) -> None:
        with self.membership_lock:
            self._executors.discard(executor)
            if self._global_executor is executor:
                self._global_executor = None

    def global_executor(self):
        if not self.ok():
            raise RuntimeError("direct_cpp context is not initialized")
        with self.membership_lock:
            if self._global_executor is None:
                from rclcppyy.direct_executors import DirectSingleThreadedExecutor

                self._global_executor = DirectSingleThreadedExecutor(
                    context=self.context)
            return self._global_executor

    def shutdown(self) -> None:
        if self.session is None:
            return
        self._shutting_down = True
        for executor in tuple(self._executors):
            executor._runtime_shutdown()
        for node in tuple(self.nodes):
            node._mark_runtime_shutdown()
        self.nodes.clear()
        self.session.close("rclcppyy direct_cpp shutdown")
        self._executors.clear()
        self._global_executor = None
        self.executor = None
        self.session = None


def _invalid_handle(reason):
    from rclpy.exceptions import InvalidHandle

    raise InvalidHandle(reason)


def _lower_entity_qos(qos_profile):
    """Return native creation QoS and the normalized rclpy metadata value."""
    from rclcpp_kit import direct_entities
    from rclpy.qos import QoSProfile

    rclcpp = _runtime().session.rclcpp
    if isinstance(qos_profile, int):
        native = direct_entities.qos_from_depth(rclcpp, qos_profile)
        return native, QoSProfile(depth=qos_profile)
    native = direct_entities.qos_from_profile(rclcpp, qos_profile)
    return native, qos_profile


class DirectPublisher:
    """rclpy-shaped metadata and lifetime around a typed C++ publisher."""

    def __init__(
        self, msg_type, topic, qos_profile, logger_name, native, callback_group
    ):
        self._native = native
        self._closed = False
        self.msg_type = msg_type
        self.topic = str(native.entity().get_topic_name())
        self.qos_profile = qos_profile
        self.callback_group = callback_group
        self.event_handlers = []
        self._logger_name = str(logger_name)
        # This is a cppyy-bound ManagedPublisher<MessageT>::publish overload.
        # Do not replace it with a Python method: publish is the message hot path.
        self.publish = native.publish

    @property
    def closed(self):
        return self._closed

    @property
    def handle(self):
        _unsupported("direct_cpp publishers do not expose a stock rclpy handle")

    @property
    def native_entity(self):
        return self._require_native().entity()

    @property
    def topic_name(self):
        return str(self.native_entity.get_topic_name())

    @property
    def logger_name(self):
        self._require_native()
        return self._logger_name

    def get_subscription_count(self):
        return int(self.native_entity.get_subscription_count())

    def assert_liveliness(self):
        result = self.native_entity.assert_liveliness()
        if result is False:
            raise RuntimeError("direct_cpp publisher could not assert liveliness")

    def wait_for_all_acked(self, timeout=None):
        if timeout is None:
            nanoseconds = -1
        else:
            from rclpy.duration import Duration

            if not isinstance(timeout, Duration):
                raise TypeError("timeout must be an rclpy.duration.Duration")
            nanoseconds = timeout.nanoseconds
        duration = cppyy.gbl.std.chrono.nanoseconds(int(nanoseconds))
        return bool(self.native_entity.wait_for_all_acked(duration))

    def _require_native(self):
        if self._closed:
            _invalid_handle("direct_cpp publisher is destroyed")
        return self._native

    def _close(self):
        if self._closed:
            return False
        closed = bool(self._native.close())
        self._closed = True
        return closed

    def destroy(self):
        if not self._close():
            _invalid_handle("direct_cpp publisher is already destroyed")


class DirectSubscription:
    """rclpy-shaped control plane over one existing C++ callback route."""

    class CallbackType(Enum):
        MessageOnly = 0
        WithMessageInfo = 1

    def __init__(
        self,
        msg_type,
        topic,
        callback,
        qos_profile,
        logger_name,
        native,
        callback_group,
        with_message_info=False,
    ):
        self._native = native
        self._closed = False
        self.msg_type = msg_type
        self.topic = str(native.entity.get_topic_name())
        self._callback = callback
        self.callback_group = callback_group
        self._executor_event = False
        self.qos_profile = qos_profile
        self.raw = False
        self.event_handlers = []
        self._callback_type = (
            self.CallbackType.WithMessageInfo
            if with_message_info
            else self.CallbackType.MessageOnly
        )
        self._logger_name = str(logger_name)

    @property
    def closed(self):
        return self._closed

    @property
    def handle(self):
        _unsupported("direct_cpp subscriptions do not expose a stock rclpy handle")

    @property
    def native_entity(self):
        return self._require_native().entity

    @property
    def topic_name(self):
        return str(self.native_entity.get_topic_name())

    @property
    def logger_name(self):
        self._require_native()
        return self._logger_name

    @property
    def callback(self):
        return self._callback

    @callback.setter
    def callback(self, value):
        _unsupported("direct_cpp subscriptions do not support callback replacement")

    @property
    def is_cft_enabled(self):
        _unsupported("direct_cpp subscriptions do not support content filtering")

    def set_content_filter(self, filter_expression, expression_parameters):
        _unsupported("direct_cpp subscriptions do not support content filtering")

    def get_content_filter(self):
        _unsupported("direct_cpp subscriptions do not support content filtering")

    def get_publisher_count(self):
        return int(self.native_entity.get_publisher_count())

    def _require_native(self):
        if self._closed:
            _invalid_handle("direct_cpp subscription is destroyed")
        return self._native

    def _close(self):
        if self._closed:
            return False
        closed = bool(self._native.close())
        self._closed = True
        return closed

    def destroy(self):
        if not self._close():
            _invalid_handle("direct_cpp subscription is already destroyed")


class DirectClient:
    """Small rclpy-style facade over one managed typed ``rclcpp`` client."""

    def __init__(
        self,
        node,
        service_type,
        service_name,
        qos_profile,
        native_client,
        callback_group,
    ):
        self._node = node
        self._native = native_client
        self._pending = {}
        self._lock = threading.RLock()
        self._closed = False
        self.context = node.context
        self.srv_type = service_type
        self.srv_name = str(native_client.raw_client.get_service_name())
        self.qos_profile = qos_profile
        self.callback_group = callback_group

    @property
    def service_name(self):
        return self.srv_name

    @property
    def closed(self):
        return self._closed

    @property
    def compile_result(self):
        return dict(self._native.compile_result)

    @property
    def source_id(self):
        return self._native.source_id

    @property
    def handle(self):
        _unsupported("direct_cpp clients do not expose a stock rclpy handle")

    def call(self, request, timeout_sec=None):
        _unsupported("direct_cpp first service slice supports call_async(), not call()")

    def call_async(self, request):
        if self._closed:
            raise RuntimeError("direct_cpp client is destroyed")
        if not isinstance(request, self.srv_type.Request):
            raise TypeError(
                "request must be an actual direct_cpp C++ %s.Request" %
                self.srv_type.__name__)
        token = int(self._native.send_cpp_value(request))
        from rclpy.task import Future

        future = Future()
        future._rclcppyy_direct_runtime = _runtime()
        future._rclcppyy_direct_client = self
        future._rclcppyy_direct_token = token
        with self._lock:
            self._pending[token] = future
        future.add_done_callback(self._retire_finished_future)
        return future

    def _retire_finished_future(self, future):
        token = getattr(future, "_rclcppyy_direct_token", None)
        if token is None:
            return
        with self._lock:
            current = self._pending.get(token)
            if current is not future:
                return
            del self._pending[token]
        if future.cancelled():
            self._native.cancel(token)

    def _poll_ready(self):
        if self._closed:
            return
        with self._lock:
            pending = tuple(self._pending.items())
        for token, future in pending:
            if future.cancelled():
                self._retire_finished_future(future)
                continue
            with self._lock:
                if self._pending.get(token) is not future:
                    continue
            try:
                if not self._native.ready(token):
                    continue
                response = self._native.take(token)
            except Exception as exc:
                if not future.cancelled() and not future.done():
                    future.set_exception(exc)
            else:
                if not future.cancelled() and not future.done():
                    future.set_result(response)

    def get_pending_request(self, sequence_number):
        with self._lock:
            return self._pending[int(sequence_number)]

    def remove_pending_request(self, future):
        with self._lock:
            match = next(
                ((token, item) for token, item in self._pending.items()
                 if item is future),
                None,
            )
            if match is None:
                return
            token, _ = match
            del self._pending[token]
        self._native.cancel(token)

    def service_is_ready(self):
        return False if self._closed else self._native.service_is_ready()

    def wait_for_service(self, timeout_sec=None):
        if self._closed:
            return False
        if timeout_sec is None:
            while self.context.ok() and not self.service_is_ready():
                self._native.wait_for_service(0.25)
            return self.service_is_ready()
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("timeout_sec must be a finite non-negative number or None")
        return self._native.wait_for_service(timeout)

    def configure_introspection(self, *args, **kwargs):
        _unsupported("direct_cpp clients do not support service introspection")

    def stats(self):
        return self._native.stats()

    def destroy(self):
        return self.close()

    def close(self):
        if self._closed:
            return False
        with self._lock:
            futures = tuple(self._pending.values())
        for future in futures:
            future.cancel()
        self._native.close()
        self._closed = True
        return True


class DirectService:
    """Small rclpy-style facade over one typed C++ service callback bridge."""

    def __init__(
        self, service_type, callback, qos_profile, native_service, callback_group
    ):
        self._native = native_service
        self._closed = False
        self.srv_type = service_type
        self.srv_name = str(native_service.raw_service.get_service_name())
        self.callback = callback
        self.callback_group = callback_group
        self.qos_profile = qos_profile

    @property
    def service_name(self):
        return self.srv_name

    @property
    def closed(self):
        return self._closed

    @property
    def compile_result(self):
        return dict(self._native.compile_result)

    @property
    def source_id(self):
        return self._native.source_id

    @property
    def handle(self):
        _unsupported("direct_cpp services do not expose a stock rclpy handle")

    def send_response(self, *args, **kwargs):
        _unsupported("direct_cpp services send only the callback return value")

    def configure_introspection(self, *args, **kwargs):
        _unsupported("direct_cpp services do not support service introspection")

    def stats(self):
        return self._native.stats()

    def destroy(self):
        return self.close()

    def close(self):
        if self._closed:
            return False
        self._native.close()
        self._closed = True
        self.callback = None
        return True


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
        self._direct_cpp_executor_ref = None
        from rclcppyy.direct_callback_groups import DirectCallbackGroup

        native_default_group = (
            self._direct_cpp_node.get_node_base_interface()
            .get_default_callback_group()
        )
        self._default_callback_group = DirectCallbackGroup._default_for(
            self, native_default_group)
        self._direct_cpp_callback_groups = [self._default_callback_group]
        self._direct_cpp_publishers = []
        self._direct_cpp_subscriptions = []
        self._direct_cpp_timers = []
        self._direct_cpp_clients = []
        self._direct_cpp_services = []
        self._direct_cpp_action_clients = []
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
    def executor(self):
        if self._direct_cpp_executor_ref is None:
            return None
        return self._direct_cpp_executor_ref()

    @executor.setter
    def executor(self, new_executor):
        current = self.executor
        if current is new_executor:
            return
        if current is not None:
            current.remove_node(self)
        if new_executor is None:
            self._set_direct_executor(None)
            return
        new_executor.add_node(self)

    def _set_direct_executor(self, executor):
        self._direct_cpp_executor_ref = (
            None if executor is None else weakref.ref(executor))

    def _wake_executor(self):
        executor = self.executor
        if executor is not None:
            executor.wake()

    @property
    def default_callback_group(self):
        return self._default_callback_group

    @property
    def callback_groups(self):
        return tuple(self._direct_cpp_callback_groups)

    def _retain_callback_group(self, callback_group):
        if callback_group not in self._direct_cpp_callback_groups:
            self._direct_cpp_callback_groups.append(callback_group)

    def _resolve_callback_group(self, callback_group):
        from rclcppyy.direct_callback_groups import DirectCallbackGroup

        if callback_group is None:
            return self._default_callback_group, None
        selected = callback_group
        if not isinstance(selected, DirectCallbackGroup):
            raise TypeError("callback_group must be a direct_cpp CallbackGroup")
        return selected, selected._bind(self)

    def _discard_group_entity(self, entity):
        callback_group = getattr(entity, "callback_group", None)
        if callback_group is not None:
            callback_group.discard_entity(entity)

    def _release_callback_groups(self):
        for callback_group in tuple(self._direct_cpp_callback_groups):
            callback_group._unbind(self)
        self._direct_cpp_callback_groups.clear()

    @property
    def publishers(self):
        yield from self._direct_cpp_publishers

    @property
    def subscriptions(self):
        yield from self._direct_cpp_subscriptions

    @property
    def timers(self):
        return list(self._direct_cpp_timers)

    @property
    def clients(self):
        return list(self._direct_cpp_clients)

    @property
    def services(self):
        return list(self._direct_cpp_services)

    @property
    def action_clients(self):
        return list(self._direct_cpp_action_clients)

    def get_name(self):
        return str(self._require_node().get_name())

    def get_namespace(self):
        return str(self._require_node().get_namespace())

    def get_fully_qualified_name(self):
        return str(self._require_node().get_fully_qualified_name())

    def get_logger(self):
        return self._require_node().get_logger()

    def _graph_interface(self):
        return self._require_node().get_node_graph_interface()

    def get_topic_names_and_types(self, no_demangle: bool = False):
        return _graph_names_and_types(
            self._graph_interface().get_topic_names_and_types(no_demangle))

    def get_service_names_and_types(self):
        return _graph_names_and_types(
            self._graph_interface().get_service_names_and_types())

    def get_node_names_and_namespaces(self):
        return [
            (_cpp_string(row.first), _cpp_string(row.second))
            for row in self._graph_interface().get_node_names_and_namespaces()
        ]

    def get_node_names(self):
        return [name for name, _namespace in self.get_node_names_and_namespaces()]

    def get_fully_qualified_node_names(self):
        return [
            namespace + ("" if namespace.endswith("/") else "/") + name
            for name, namespace in self.get_node_names_and_namespaces()
        ]

    def get_node_names_and_namespaces_with_enclaves(self):
        return [
            tuple(_cpp_string(row[index]) for index in range(3))
            for row in self._graph_interface().get_node_names_with_enclaves()
        ]

    def _names_and_types_by_node(
        self,
        method_name: str,
        kind: str,
        node_name: str,
        node_namespace: str,
        no_demangle=None,
    ):
        from rclpy._rclpy_pybind11 import NodeNameNonExistentError
        from rclpy.validate_namespace import validate_namespace
        from rclpy.validate_node_name import validate_node_name

        validate_node_name(node_name)
        validate_namespace(node_namespace)
        identity = (str(node_name), str(node_namespace))
        if identity not in self.get_node_names_and_namespaces():
            raise NodeNameNonExistentError(
                "cannot get %s names and types for nonexistent node: error not set"
                % kind
            )
        method = getattr(self._graph_interface(), method_name)
        arguments = (
            (node_name, node_namespace)
            if no_demangle is None
            else (node_name, node_namespace, no_demangle)
        )
        try:
            return _graph_names_and_types(method(*arguments))
        except Exception:
            if identity not in self.get_node_names_and_namespaces():
                raise NodeNameNonExistentError(
                    "cannot get %s names and types for nonexistent node: error not set"
                    % kind
                ) from None
            raise

    def get_publisher_names_and_types_by_node(
        self, node_name: str, node_namespace: str, no_demangle: bool = False
    ):
        return self._names_and_types_by_node(
            "get_publisher_names_and_types_by_node",
            "publisher",
            node_name,
            node_namespace,
            no_demangle,
        )

    def get_subscriber_names_and_types_by_node(
        self, node_name: str, node_namespace: str, no_demangle: bool = False
    ):
        return self._names_and_types_by_node(
            "get_subscriber_names_and_types_by_node",
            "subscriber",
            node_name,
            node_namespace,
            no_demangle,
        )

    def get_service_names_and_types_by_node(
        self, node_name: str, node_namespace: str
    ):
        return self._names_and_types_by_node(
            "get_service_names_and_types_by_node",
            "service",
            node_name,
            node_namespace,
        )

    def get_client_names_and_types_by_node(
        self, node_name: str, node_namespace: str
    ):
        return self._names_and_types_by_node(
            "get_client_names_and_types_by_node",
            "client",
            node_name,
            node_namespace,
        )

    def _expand_graph_name(self, name: str, *, is_service: bool) -> str:
        from rclpy.expand_topic_name import expand_topic_name
        from rclpy.validate_full_topic_name import validate_full_topic_name

        expanded = expand_topic_name(name, self.get_name(), self.get_namespace())
        validate_full_topic_name(expanded, is_service=is_service)
        return expanded

    def resolve_topic_name(self, topic: str, *, only_expand: bool = False) -> str:
        expanded = self._expand_graph_name(topic, is_service=False)
        if only_expand:
            return expanded
        interface = self._require_node().get_node_topics_interface()
        return _cpp_string(interface.resolve_topic_name(topic, False))

    def resolve_service_name(
        self, service: str, *, only_expand: bool = False
    ) -> str:
        expanded = self._expand_graph_name(service, is_service=True)
        if only_expand:
            return expanded
        interface = self._require_node().get_node_services_interface()
        return _cpp_string(interface.resolve_service_name(service, False))

    def count_publishers(self, topic_name: str) -> int:
        topic = self._expand_graph_name(topic_name, is_service=False)
        return int(self._graph_interface().count_publishers(topic))

    def count_subscribers(self, topic_name: str) -> int:
        topic = self._expand_graph_name(topic_name, is_service=False)
        return int(self._graph_interface().count_subscribers(topic))

    def count_clients(self, service_name: str) -> int:
        service = self._expand_graph_name(service_name, is_service=True)
        return int(self._graph_interface().count_clients(service))

    def count_services(self, service_name: str) -> int:
        service = self._expand_graph_name(service_name, is_service=True)
        return int(self._graph_interface().count_services(service))

    def _get_info_by_topic(self, topic_name: str, no_mangle: bool, method_name: str):
        selected_name = (
            topic_name if no_mangle else self.resolve_topic_name(topic_name)
        )
        method = getattr(self._graph_interface(), method_name)
        return [
            _topic_endpoint_info(value)
            for value in method(selected_name, no_mangle)
        ]

    def get_publishers_info_by_topic(
        self, topic_name: str, no_mangle: bool = False
    ):
        return self._get_info_by_topic(
            topic_name, no_mangle, "get_publishers_info_by_topic")

    def get_subscriptions_info_by_topic(
        self, topic_name: str, no_mangle: bool = False
    ):
        return self._get_info_by_topic(
            topic_name, no_mangle, "get_subscriptions_info_by_topic")

    def wait_for_node(self, fully_qualified_node_name: str, timeout: float) -> bool:
        if not fully_qualified_node_name.startswith("/"):
            fully_qualified_node_name = "/" + fully_qualified_node_name
        started = time.time()
        found = False
        while time.time() - started < timeout and not found:
            found = fully_qualified_node_name in self.get_fully_qualified_node_names()
            time.sleep(0.1)
        return found

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
            "event_callbacks": event_callbacks is not None,
            "qos_overriding_options": qos_overriding_options is not None,
            "publisher_class": publisher_class is not None,
        }
        self._reject_entity_options("publisher", requested)
        from rclcpp_kit import direct_entities

        direct_entities.resolve_supported_type(msg_type)
        qos, normalized_qos = _lower_entity_qos(qos_profile)
        group, native_group = self._resolve_callback_group(callback_group)
        native = direct_entities.create_managed_publisher(
            self._require_node(),
            msg_type,
            str(topic),
            qos,
            callback_group=native_group,
        )
        publisher = DirectPublisher(
            msg_type,
            topic,
            normalized_qos,
            self._logger_name(),
            native,
            group,
        )
        group.add_entity(publisher)
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
        with_message_info = self._validate_subscription_callback(callback)
        qos, normalized_qos = _lower_entity_qos(qos_profile)
        group, native_group = self._resolve_callback_group(callback_group)
        if "subscription_shared_lease" in _runtime().optimizations:
            from rclcpp_kit import direct_subscription_lease

            native = direct_subscription_lease.create_subscription_lease(
                self._require_node(),
                msg_type,
                str(topic),
                callback,
                qos,
                with_message_info=with_message_info,
                callback_group=native_group,
            )
        else:
            native = direct_entities.create_subscription(
                self._require_node(),
                msg_type,
                str(topic),
                callback,
                qos,
                with_message_info=with_message_info,
                callback_group=native_group,
            )
        subscription = DirectSubscription(
            msg_type,
            topic,
            callback,
            normalized_qos,
            self._logger_name(),
            native,
            group,
            with_message_info,
        )
        group.add_entity(subscription)
        self._direct_cpp_subscriptions.append(subscription)
        self._record_entity(
            "subscription",
            topic,
            msg_type,
            native,
            with_message_info=with_message_info,
        )
        return subscription

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

        group, native_group = self._resolve_callback_group(callback_group)
        timer = direct_entities.create_wall_timer(
            self._require_node(),
            period_ns,
            callback,
            callback_group=native_group,
        )
        timer.callback_group = group
        group.add_entity(timer)
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

    def create_client(
        self,
        srv_type,
        srv_name,
        *,
        qos_profile=_DEFAULT_SERVICE_QOS,
        callback_group=None,
    ):
        from rclcppyy import direct_services

        binding = direct_services.resolve_supported_type(
            srv_type, _SERVICE_INSTALLATION)
        qos = self._require_default_service_qos(qos_profile)
        group, native_group = self._resolve_callback_group(callback_group)
        native_client = _runtime().require_session().create_native_client(
            self._require_node(),
            srv_type,
            str(srv_name),
            callback_group=native_group,
        )
        client = DirectClient(
            self, srv_type, str(srv_name), qos, native_client, group)
        group.add_entity(client)
        self._direct_cpp_clients.append(client)
        self._record_service_entity(
            "client", client.srv_name, binding, client)
        return client

    def create_service(
        self,
        srv_type,
        srv_name,
        callback,
        *,
        qos_profile=_DEFAULT_SERVICE_QOS,
        callback_group=None,
    ):
        from rclcppyy import direct_services

        binding = direct_services.resolve_supported_type(
            srv_type, _SERVICE_INSTALLATION)
        qos = self._require_default_service_qos(qos_profile)
        if not callable(callback):
            raise TypeError("service callback must be callable")
        callback_target = getattr(callback, "__call__", callback)
        if inspect.iscoroutinefunction(callback) or inspect.iscoroutinefunction(
            callback_target
        ):
            _unsupported("direct_cpp services require a synchronous callback")
        try:
            inspect.signature(callback).bind(object(), object())
        except (TypeError, ValueError) as exc:
            raise TypeError(
                "direct_cpp service callback must accept request and response") from exc
        group, native_group = self._resolve_callback_group(callback_group)
        native_service = _runtime().require_session().create_python_service(
            self._require_node(),
            srv_type,
            str(srv_name),
            callback,
            callback_group=native_group,
        )
        service = DirectService(
            srv_type, callback, qos, native_service, group)
        group.add_entity(service)
        self._direct_cpp_services.append(service)
        self._record_service_entity(
            "service", service.srv_name, binding, service)
        return service

    def destroy_timer(self, timer):
        for index, candidate in enumerate(self._direct_cpp_timers):
            if timer is candidate:
                self._discard_group_entity(candidate)
                candidate.destroy()
                del self._direct_cpp_timers[index]
                return True
        return False

    def destroy_publisher(self, publisher):
        for index, candidate in enumerate(self._direct_cpp_publishers):
            if publisher is candidate:
                del self._direct_cpp_publishers[index]
                self._discard_group_entity(candidate)
                return candidate._close()
        return False

    def destroy_subscription(self, subscription):
        for index, candidate in enumerate(self._direct_cpp_subscriptions):
            if subscription is candidate:
                del self._direct_cpp_subscriptions[index]
                self._discard_group_entity(candidate)
                return candidate._close()
        return False

    def destroy_client(self, client):
        for index, candidate in enumerate(self._direct_cpp_clients):
            if client is candidate:
                self._discard_group_entity(candidate)
                candidate.close()
                del self._direct_cpp_clients[index]
                return True
        return False

    def destroy_service(self, service):
        for index, candidate in enumerate(self._direct_cpp_services):
            if service is candidate:
                self._discard_group_entity(candidate)
                candidate.close()
                del self._direct_cpp_services[index]
                return True
        return False

    def _discard_direct_action_client(self, action_client):
        try:
            self._direct_cpp_action_clients.remove(action_client)
        except ValueError:
            pass

    def destroy_node(self):
        node = self._direct_cpp_node
        if node is None:
            return
        executor = self.executor
        if executor is not None:
            executor.remove_node(self)
            self._set_direct_executor(None)
        while self._direct_cpp_publishers:
            self.destroy_publisher(self._direct_cpp_publishers[0])
        while self._direct_cpp_subscriptions:
            self.destroy_subscription(self._direct_cpp_subscriptions[0])
        for timer in tuple(self._direct_cpp_timers):
            timer.destroy()
        for client in tuple(self._direct_cpp_clients):
            client.close()
        for service in tuple(self._direct_cpp_services):
            service.close()
        for action_client in tuple(self._direct_cpp_action_clients):
            action_client.close()
        self._direct_cpp_timers.clear()
        self._direct_cpp_clients.clear()
        self._direct_cpp_services.clear()
        self._direct_cpp_action_clients.clear()
        self._release_callback_groups()
        _runtime().detach(self, node)
        self._direct_cpp_node = None

    def _mark_runtime_shutdown(self):
        for publisher in tuple(self._direct_cpp_publishers):
            publisher._close()
        for subscription in tuple(self._direct_cpp_subscriptions):
            subscription._close()
        for timer in tuple(self._direct_cpp_timers):
            timer.destroy()
        for client in tuple(self._direct_cpp_clients):
            client.close()
        for service in tuple(self._direct_cpp_services):
            service.close()
        for action_client in tuple(self._direct_cpp_action_clients):
            action_client.close()
        self._direct_cpp_timers.clear()
        self._direct_cpp_clients.clear()
        self._direct_cpp_services.clear()
        self._direct_cpp_action_clients.clear()
        self._direct_cpp_publishers.clear()
        self._direct_cpp_subscriptions.clear()
        self._release_callback_groups()
        self._set_direct_executor(None)
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

    def _logger_name(self):
        logger = self._require_node().get_logger()
        return str(logger.name)

    def _validate_subscription_callback(self, callback):
        callback_target = getattr(callback, "__call__", callback)
        if inspect.iscoroutinefunction(callback) or inspect.iscoroutinefunction(
            callback_target
        ):
            _unsupported("direct_cpp subscriptions require a synchronous callback")
        signature = inspect.signature(callback)
        try:
            signature.bind(object())
            return False
        except TypeError:
            pass
        try:
            signature.bind(object(), object())
        except TypeError as exc:
            raise RuntimeError(
                "subscription callback must accept one message argument or "
                "message plus MessageInfo"
            ) from exc
        return True

    def _require_default_service_qos(self, qos_profile):
        from rclpy.qos import qos_profile_services_default

        value = (
            qos_profile_services_default
            if qos_profile is _DEFAULT_SERVICE_QOS
            else qos_profile
        )
        if value != qos_profile_services_default:
            _unsupported("direct_cpp services and clients require default service QoS")
        return value

    def _poll_direct_clients(self):
        for client in tuple(self._direct_cpp_clients):
            client._poll_ready()
        for action_client in tuple(self._direct_cpp_action_clients):
            action_client._poll_ready()

    def _record_service_entity(self, entity_type, service_name, binding, entity):
        if entity_type == "client":
            policies = (
                "direct_cpp", "direct_cpp_service", "no_conversion",
                "per_operation_future", "cpp_pending_state",
            )
            handoff = {
                "request_handoff": "one_native_cpp_value_copy",
                "response_handoff": "shared_cpp_response",
                "future_control": "per_operation_rclpy_task_future",
                "python_request_crossings_per_call": 1,
                "python_response_crossings_per_call": 1,
                "cpp_request_copies_per_call": 1,
            }
        else:
            policies = (
                "direct_cpp", "direct_cpp_service", "no_conversion",
                "python_callback", "owning_cpp_callback_values",
            )
            handoff = {
                "callback_handoff": "one_python_callback_crossing",
                "request_handoff": "one_owning_native_cpp_copy",
                "response_handoff": "one_native_cpp_assignment",
                "python_callback_crossings_per_request": 1,
                "cpp_request_copies_per_request": 1,
                "cpp_response_copies_per_request": 1,
            }
        record_decision(
            "entities",
            "cpp",
            "direct typed rclcpp %s with C++ service messages" % entity_type,
            policies=policies,
            metadata={
                "entity_type": entity_type,
                "service_name": str(service_name),
                "service_type": binding.cpp_type_name,
                "service_interface": binding.interface,
                "request_representation": "actual_cpp",
                "response_representation": "actual_cpp",
                "python_message_conversions": 0,
                "source_id": entity.source_id,
                **handoff,
            },
        )

    def _record_entity(
        self,
        entity_type,
        topic,
        msg_type,
        entity=None,
        *,
        with_message_info=False,
    ):
        policies = ["direct_cpp", "direct_cpp_message", "no_conversion"]
        metadata = {
            "entity_type": entity_type,
            "topic": str(topic),
            "message_type": str(getattr(msg_type, "__cpp_name__", msg_type)),
        }
        if entity_type == "subscription":
            metadata["message_info"] = bool(with_message_info)
            if with_message_info:
                policies.append("native_message_info")
            if getattr(entity, "creation_route", "") == (
                "rclcpp_unique_ptr_subscription_lease"
            ):
                policies.extend((
                    "subscription_shared_lease",
                    "actual_cpp_message",
                ))
                metadata.update({
                    "callback_handoff": "shared_cpp_message_lease",
                    "subscription_creation_route": (
                        "rclcpp_unique_ptr_subscription_lease"),
                    "message_representation": "actual_cpp",
                    "python_message_conversions": 0,
                    "serialization_operations": 0,
                    "message_deep_copies_per_callback": 0,
                    "shared_control_blocks_per_callback": 1,
                    "shared_owner_acquisitions_per_callback": 1,
                    "owning_cpp_copy_count_at_creation": (
                        entity.owning_cpp_copy_count),
                    "lease_count_at_creation": entity.lease_count,
                })
            else:
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
        name for name in (
            "rclpy.callback_groups",
            "rclpy.executors",
            "rclpy.node",
            "rclpy.publisher",
            "rclpy.subscription",
        ) if name in sys.modules)
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing: %s" % ", ".join(stale))
    from rclcppyy.direct_actions import assert_early_imports as assert_action_imports
    from rclcppyy.direct_messages import assert_early_imports
    from rclcppyy.direct_services import assert_early_imports as assert_service_imports

    assert_action_imports()
    assert_service_imports()
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


def _direct_get_global_executor():
    return _runtime().global_executor()


def _select_direct_executor(executor):
    from rclcppyy.direct_executors import DirectExecutor

    selected = _direct_get_global_executor() if executor is None else executor
    if not isinstance(selected, DirectExecutor):
        _unsupported("direct_cpp requires a direct_cpp executor")
    if selected.context is not _runtime().context:
        _unsupported("direct_cpp executor and node contexts do not match")
    return selected


def _direct_spin_once(node, *, executor=None, timeout_sec=None):
    using_global_executor = executor is None
    selected = _select_direct_executor(executor)
    node_was_added = False
    try:
        node_was_added = selected.add_node(node)
        selected.spin_once(timeout_sec=timeout_sec)
    finally:
        if node_was_added:
            if using_global_executor:
                selected.park_node(node)
            else:
                selected.remove_node(node)


def _direct_spin(node, executor=None):
    selected = _select_direct_executor(executor)
    try:
        selected.add_node(node)
        while selected.context.ok():
            selected.spin_once()
    finally:
        selected.remove_node(node)


def _direct_spin_until_future_complete(
    node, future, executor=None, timeout_sec=None
):
    runtime = _runtime()
    future_runtime = getattr(future, "_rclcppyy_direct_runtime", runtime)
    if future_runtime is not runtime:
        _unsupported("direct_cpp cannot spin a Future from a foreign context")
    using_global_executor = executor is None
    selected = _select_direct_executor(executor)
    node_was_added = False
    try:
        node_was_added = selected.add_node(node)
        selected.spin_until_future_complete(future, timeout_sec=timeout_sec)
    finally:
        if node_was_added:
            if using_global_executor:
                selected.park_node(node)
            else:
                selected.remove_node(node)


def activate(*, optimizations=(), interfaces=()) -> bool:
    """Install the complete first-slice surface, rolling back on any failure."""
    global _ACTION_INSTALLATION, _ACTIVE, _ACTIVE_INTERFACES
    global _ACTIVE_OPTIMIZATIONS
    global _MESSAGE_INSTALLATION, _PATCHES, _RUNTIME, _SERVICE_INSTALLATION
    normalized_optimizations = tuple(sorted(set(optimizations)))
    from rclcppyy import direct_actions, direct_messages, direct_services

    normalized_interfaces = direct_actions.normalize_registered_interfaces(
        interfaces)
    message_interfaces = tuple(
        value for value in normalized_interfaces if "/msg/" in value)
    service_interfaces = tuple(
        value for value in normalized_interfaces if "/srv/" in value)
    action_interfaces = tuple(
        value for value in normalized_interfaces if "/action/" in value)
    unknown = sorted(
        set(normalized_optimizations) - {"subscription_shared_lease"})
    if unknown:
        _unsupported(
            "direct_cpp does not support optimization(s): %s" %
            ", ".join(unknown))
    if _ACTIVE:
        if normalized_optimizations != _ACTIVE_OPTIMIZATIONS:
            raise RuntimeError(
                "direct_cpp is already active with optimizations %r" %
                (_ACTIVE_OPTIMIZATIONS,))
        if normalized_interfaces != _ACTIVE_INTERFACES:
            raise RuntimeError(
                "direct_cpp is already active with interfaces %r" %
                (_ACTIVE_INTERFACES,))
        return True
    _check_early_activation()
    _check_runtime()

    service_plan = direct_services.prepare(service_interfaces)
    action_plan = direct_actions.prepare(action_interfaces)
    installation = direct_messages.install(
        message_interfaces
        + service_plan.message_dependencies
        + action_plan.message_dependencies)
    service_installation = None
    action_installation = None
    patches = []
    try:
        service_installation = direct_services.install(plan=service_plan)
        action_installation = direct_actions.install(plan=action_plan)
        import rclpy
        import rclpy.action as action_module
        import rclpy.action.client as action_client_module
        import rclpy.action.server as action_server_module
        import rclpy.callback_groups as callback_groups_module
        import rclpy.executors as executors_module
        import rclpy.node as node_module
        import rclpy.publisher as publisher_module
        import rclpy.subscription as subscription_module

        runtime = _DirectRuntime(
            normalized_optimizations, normalized_interfaces)
        from rclcppyy.direct_executors import (
            DirectExecutor,
            DirectMultiThreadedExecutor,
            DirectSingleThreadedExecutor,
        )
        from rclcppyy.direct_callback_groups import (
            DirectCallbackGroup,
            DirectMutuallyExclusiveCallbackGroup,
            DirectReentrantCallbackGroup,
        )

        DirectSubscription.CallbackType = (
            subscription_module.Subscription.CallbackType)
        replacements = (
            (callback_groups_module, "CallbackGroup", DirectCallbackGroup),
            (
                callback_groups_module,
                "MutuallyExclusiveCallbackGroup",
                DirectMutuallyExclusiveCallbackGroup,
            ),
            (
                callback_groups_module,
                "ReentrantCallbackGroup",
                DirectReentrantCallbackGroup,
            ),
            (executors_module, "Executor", DirectExecutor),
            (
                executors_module,
                "SingleThreadedExecutor",
                DirectSingleThreadedExecutor,
            ),
            (
                executors_module,
                "MultiThreadedExecutor",
                DirectMultiThreadedExecutor,
            ),
            (node_module, "Node", DirectNode),
            (publisher_module, "Publisher", DirectPublisher),
            (subscription_module, "Subscription", DirectSubscription),
            (action_module, "ActionClient", direct_actions.DirectActionClient),
            (action_client_module, "ActionClient", direct_actions.DirectActionClient),
            (
                action_client_module,
                "ClientGoalHandle",
                direct_actions.DirectClientGoalHandle,
            ),
            (action_module, "ActionServer", direct_actions.DirectActionServer),
            (action_server_module, "ActionServer", direct_actions.DirectActionServer),
            (rclpy, "init", _direct_init),
            (rclpy, "ok", _direct_ok),
            (rclpy, "shutdown", _direct_shutdown),
            (rclpy, "try_shutdown", _direct_try_shutdown),
            (rclpy, "get_global_executor", _direct_get_global_executor),
            (rclpy, "spin_once", _direct_spin_once),
            (rclpy, "spin", _direct_spin),
            (rclpy, "spin_until_future_complete", _direct_spin_until_future_complete),
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
        if action_installation is not None:
            action_installation.restore()
        if service_installation is not None:
            service_installation.restore()
        installation.restore()
        raise
    _MESSAGE_INSTALLATION = installation
    _SERVICE_INSTALLATION = service_installation
    _ACTION_INSTALLATION = action_installation
    _PATCHES = tuple(patches)
    _ACTIVE_OPTIMIZATIONS = normalized_optimizations
    _ACTIVE_INTERFACES = normalized_interfaces
    _ACTIVE = True
    record_decision(
        "operations",
        "cpp",
        "installed explicit direct_cpp source-compatible slice",
        policies=("direct_cpp", "jazzy", "cyclonedds", "no_conversion"),
        metadata={
            "operation": "enable_cpp_acceleration",
            "profile": "direct_cpp",
            "optimizations": list(normalized_optimizations),
            "requested_message_interfaces": list(message_interfaces),
            "requested_service_interfaces": list(service_interfaces),
            "requested_action_interfaces": list(action_interfaces),
            "message_types": [binding.cpp_type_name for binding in installation.bindings],
            "service_types": [
                binding.cpp_type_name for binding in service_installation.bindings],
            "action_types": [
                binding.cpp_types.cpp_name
                for binding in action_installation.bindings],
        },
    )
    return True


__all__ = [
    "DirectClient",
    "DirectNode",
    "DirectPublisher",
    "DirectService",
    "DirectSubscription",
    "activate",
]
