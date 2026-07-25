"""Bounded, source-compatible control plane over direct ``rclcpp`` entities."""

from __future__ import annotations

from enum import Enum
import functools
import math
import inspect
import os
import sys
import threading
import time
import weakref

import cppyy

from rclcpp_kit import native_parameters as _native_parameters
from rclcppyy._signature_mirror import mirror_class, mirror_function
from rclcppyy._status import record_decision
from rclcppyy._surface import _DirectSurface
from rclcppyy import direct_parameters as _direct_parameters
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
_PARAMETER_CACHE_CAPACITY_ENV = "RCLCPPYY_DIRECT_PARAMETER_CACHE_CAPACITY"
_PARAMETER_CACHE_DEFAULT_CAPACITY = 1024
_PARAMETER_CACHE_MISSING = object()


class _TrackingParameterCache(dict):
    """Test-only hit accounting; production nodes use an exact plain dict."""

    __slots__ = ("hits",)

    def __init__(self, values=()):
        super().__init__(values)
        self.hits = 0

    def __getitem__(self, name):
        value = super().__getitem__(name)
        self.hits += 1
        return value


def _parameter_cache_capacity():
    raw_value = os.environ.get(
        _PARAMETER_CACHE_CAPACITY_ENV,
        str(_PARAMETER_CACHE_DEFAULT_CAPACITY),
    )
    try:
        capacity = int(raw_value)
    except ValueError as exception:
        raise ValueError(
            "%s must be a non-negative integer" %
            _PARAMETER_CACHE_CAPACITY_ENV
        ) from exception
    if capacity < 0:
        raise ValueError(
            "%s must be a non-negative integer" %
            _PARAMETER_CACHE_CAPACITY_ENV)
    return capacity


class _DirectParameterCallbackError(Exception):
    def __init__(self, original):
        super().__init__(str(original))
        self.original = original


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
        try:
            return bool(self.session.context.is_valid())
        except RuntimeError:
            # The native context can become invalid through a path this
            # runtime never tracked (e.g. a native rclcpp shutdown outside
            # NativeSession.close()), in which case NativeSession.context
            # itself raises rather than reporting False. ok() is a boolean
            # predicate and must not raise on an unowned, already-gone
            # context; not ok is exactly the honest answer here.
            return False

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
                facade._native_release(node)

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
        for node in tuple(self.nodes):
            node._require_idle_action_server_callbacks("shut down the context")
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


def _direct_node_options(
    session,
    *,
    cli_args,
    use_global_arguments,
    enable_rosout,
    start_parameter_services,
    parameter_overrides,
    allow_undeclared_parameters,
    automatically_declare_parameters_from_overrides,
    enable_logger_service,
):
    boolean_options = {
        "use_global_arguments": use_global_arguments,
        "enable_rosout": enable_rosout,
        "start_parameter_services": start_parameter_services,
        "allow_undeclared_parameters": allow_undeclared_parameters,
        "automatically_declare_parameters_from_overrides": (
            automatically_declare_parameters_from_overrides),
        "enable_logger_service": enable_logger_service,
    }
    invalid = sorted(
        name for name, value in boolean_options.items()
        if not isinstance(value, bool)
    )
    if invalid:
        raise TypeError(
            "direct_cpp node option(s) must be bool: %s" % ", ".join(invalid))
    if cli_args is not None:
        if not isinstance(cli_args, list) or not all(
                isinstance(value, str) for value in cli_args):
            raise TypeError("direct_cpp cli_args must be a list of strings or None")
        native_arguments = cppyy.gbl.std.vector["std::string"](cli_args)
    else:
        native_arguments = None
    if parameter_overrides is not None and not isinstance(parameter_overrides, list):
        raise TypeError("direct_cpp parameter_overrides must be a list or None")

    from rclcpp_kit import native_parameters
    from rclcppyy import direct_parameters

    native_overrides = tuple(
        direct_parameters.native_parameter(parameter)
        for parameter in (parameter_overrides or ()))
    options = session.rclcpp.NodeOptions()
    if native_arguments is not None:
        options.arguments(native_arguments)
    if native_overrides:
        options.parameter_overrides(
            native_parameters.parameter_vector(native_overrides))
    options.use_global_arguments(use_global_arguments)
    options.enable_rosout(enable_rosout)
    options.start_parameter_services(start_parameter_services)
    options.allow_undeclared_parameters(allow_undeclared_parameters)
    options.automatically_declare_parameters_from_overrides(
        automatically_declare_parameters_from_overrides)
    options.enable_logger_service(enable_logger_service)
    return options, len(native_overrides)


# Canonical event-callback field order -- matches the suite's own
# _PUBLISHER_EVENT_NAMES / _SUBSCRIPTION_EVENT_NAMES (rclcpp_kit.direct_entities),
# which mirrors rclcpp's event_handler.hpp ordering.
_PUBLISHER_EVENT_FIELDS = (
    "deadline", "liveliness", "incompatible_qos", "incompatible_type", "matched",
)
_SUBSCRIPTION_EVENT_FIELDS = (
    "deadline", "liveliness", "incompatible_qos", "message_lost",
    "incompatible_type", "matched",
)


def _extract_event_callbacks(event_callbacks, fields, expected_type):
    """Translate a stock ``PublisherEventCallbacks``/``SubscriptionEventCallbacks``
    instance into ``{event_name: callable}``, keeping only the fields the
    caller actually set -- the same fields stock's own
    ``create_event_handlers`` reads off the container one at a time
    (``rclpy.event_handler``). ``None`` means no event callbacks requested.
    """
    if event_callbacks is None:
        return {}
    if not isinstance(event_callbacks, expected_type):
        raise TypeError(
            "event_callbacks must be a %s or None, got %s" %
            (expected_type.__qualname__, type(event_callbacks).__name__)
        )
    extracted = {}
    for name in fields:
        callback = getattr(event_callbacks, name)
        if callback is not None:
            extracted[name] = callback
    return extracted


class _DirectEventHandler:
    """Introspection-only facade entry for one registered QoS event callback.

    Stock exposes ``Publisher.event_handlers``/``Subscription.event_handlers``
    as a list of executor-visible ``Waitable`` objects (rclpy.event_handler);
    dispatch here is fully native (suite-owned; no Python-side Waitable ever
    exists), so this mirrors only what an unchanged app reads off each entry:
    the original callback, plus the event name for identification.
    """

    __slots__ = ("event_type", "callback")

    def __init__(self, event_type, callback):
        self.event_type = event_type
        self.callback = callback


class ContentFilterUnsupportedError(RuntimeError):
    """A requested content-filtered subscription cannot be honored by the
    active RMW.

    Stock rclpy has no exception for this case: on an RMW that cannot content-
    filter (Cyclone, the production default), stock silently returns an
    *unfiltered* subscription -- the caller's filter expression is dropped
    without any signal. That silent narrowing is exactly the fail-closed
    hazard rclcppyy refuses to reproduce; this is the deliberate, documented
    stricter-than-stock divergence (PLAN-qos-events-product.md S3/S6#3): the
    suite's own ``ContentFilterUnsupported`` (raised only after probing the
    just-created entity's ``is_cft_enabled()``, so no unfiltered subscription
    is ever handed back under the guise of filtering) is translated to this
    public rclcppyy-surface type rather than silently degrading.
    """


def _extract_content_filter(content_filter_options):
    """Translate a stock ``ContentFilterOptions`` NamedTuple into the suite's
    ``(filter_expression, expression_parameters)`` pair. ``None`` means no
    content filter requested.
    """
    if content_filter_options is None:
        return None
    from rclpy.subscription_content_filter_options import ContentFilterOptions

    if not isinstance(content_filter_options, ContentFilterOptions):
        raise TypeError(
            "content_filter_options must be a ContentFilterOptions or None, "
            "got %s" % type(content_filter_options).__name__
        )
    return (
        content_filter_options.filter_expression,
        tuple(content_filter_options.expression_parameters),
    )


# The only policy-kind set the suite ever attaches (QosOverridingOptions::
# with_default_policies() -- rclcpp_kit.direct_entities' own
# _validate_qos_overriding accepts nothing but a plain bool and always maps
# True to exactly this set). Set, not sequence, comparison: with_default_
# policies() and an equivalent explicit QoSOverridingOptions(policy_kinds=...)
# construction must be recognized identically regardless of member order.
_QOS_OVERRIDING_DEFAULT_POLICY_KINDS = frozenset(("history", "depth", "reliability"))


def _extract_qos_overriding(qos_overriding_options):
    """Translate a stock ``QoSOverridingOptions`` into the suite's plain
    ``qos_overriding`` bool. ``None`` means no QoS overriding requested.

    Only ``with_default_policies()`` (or an equivalent explicit
    ``QoSOverridingOptions(policy_kinds=(HISTORY, DEPTH, RELIABILITY))``, no
    ``callback``, no ``entity_id``) is supported this wave -- the suite has
    no attachment point for a custom policy-kind subset, a validation
    callback, or an ``entity_id`` (recorded suite gap,
    PLAN-qos-events-product.md S3). Accepting a narrower/different request
    and silently applying the full default set instead would over-claim, so
    anything outside that exact shape fails closed instead.
    """
    if qos_overriding_options is None:
        return False
    from rclpy.qos_overriding_options import QoSOverridingOptions

    if not isinstance(qos_overriding_options, QoSOverridingOptions):
        raise TypeError(
            "qos_overriding_options must be a QoSOverridingOptions or None, "
            "got %s" % type(qos_overriding_options).__name__
        )
    policy_kinds = frozenset(
        kind.name.lower() for kind in qos_overriding_options.policy_kinds)
    if (
        policy_kinds != _QOS_OVERRIDING_DEFAULT_POLICY_KINDS
        or qos_overriding_options.callback is not None
        or qos_overriding_options.entity_id is not None
    ):
        _unsupported(
            "direct_cpp subscription qos_overriding_options only supports "
            "with_default_policies() (history, depth, reliability; no "
            "callback, no entity_id) this wave; got policy_kinds=%s, "
            "callback=%r, entity_id=%r" % (
                sorted(policy_kinds),
                qos_overriding_options.callback,
                qos_overriding_options.entity_id,
            )
        )
    return True


class DirectPublisher(metaclass=_DirectSurface):
    """rclpy-shaped metadata and lifetime around a typed C++ publisher."""

    _PARITY_HIDDEN = frozenset({"closed", "native_entity"})

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


class DirectSubscription(metaclass=_DirectSurface):
    """rclpy-shaped control plane over one existing C++ callback route."""

    _PARITY_HIDDEN = frozenset({"closed", "native_entity"})

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
        if not isinstance(request, self.srv_type.Request):
            raise TypeError(
                "request must be an actual direct_cpp C++ %s.Request" %
                self.srv_type.__name__)
        event = threading.Event()
        future = self.call_async(request)
        future.add_done_callback(lambda _future: event.set())
        if not future.done() and not event.wait(timeout_sec):
            self.remove_pending_request(future)
        if future.exception() is not None:
            raise future.exception()
        return future.result()

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


# Self-destroy detection (Slice 2.5, docs/plans/PLAN-mte-unlock.md Addendum):
# a per-thread set of nodes the calling thread is currently dispatching a
# callback for. destroy_node()/destroy_*() consult this to tell "an external
# thread is destroying me" (safe to wait for quiescence) from "I am
# destroying myself from inside my own callback" (waiting for my own
# in-flight count would self-deadlock -- must defer instead).
#
# Status (Addendum v3/v3.1): this in-flight-counter/quiescence machinery
# alone cannot make destroy-under-dispatch safe -- it is blind to the
# pre-shim "marshal window" (a worker committed to dispatch, cppyy still
# marshaling, before this counter increments). The suite's native-owned
# callable lifetime (ManagedCallbackEntityImpl + the callable-lifetime
# reaper, cppyy_kit 6d60a85/cc70d1b/9ff96fd) is what actually closes that
# window. This machinery stays load-bearing for ordering/semantics and
# defect-A exception draining, and as defense-in-depth (it also keeps the
# reaper's release queue quieter by avoiding unnecessary racing) -- not
# ripped out, just no longer the sole safety argument.
_dispatch_marker = threading.local()

# Bound on how long destroy_node()/destroy_*() wait for a node's in-flight
# callback count to reach zero before freeing anything. On timeout this
# fails loud rather than freeing a callable a worker might still be
# invoking -- leak-safe beats crash-safe (Addendum risk 2).
_DESTROY_QUIESCENCE_TIMEOUT_SEC = 10.0


def _enter_dispatch(node) -> None:
    nodes = getattr(_dispatch_marker, "nodes", None)
    if nodes is None:
        nodes = set()
        _dispatch_marker.nodes = nodes
    nodes.add(node)


def _exit_dispatch(node) -> None:
    nodes = getattr(_dispatch_marker, "nodes", None)
    if nodes is not None:
        nodes.discard(node)


def _is_dispatching_for(node) -> bool:
    """True if the calling thread is already inside a callback dispatched
    for ``node`` -- a destroy_*() call reaching here is a self-destroy that
    must defer to the owning executor's pump instead of waiting in place."""
    nodes = getattr(_dispatch_marker, "nodes", None)
    return nodes is not None and node in nodes


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
        runtime = _runtime()
        if context is not None and context is not runtime.context:
            _unsupported("direct_cpp node requires its active runtime context")
        session = runtime.require_session()
        options, override_count = _direct_node_options(
            session,
            cli_args=cli_args,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=(
                automatically_declare_parameters_from_overrides),
            enable_logger_service=enable_logger_service,
        )
        native_node = session.create_node(
            str(node_name), namespace=str(namespace or ""), options=options)
        self._init_common(
            native_node, node_name, namespace,
            allow_undeclared_parameters=allow_undeclared_parameters)
        record_decision(
            "nodes",
            "cpp",
            "direct_cpp owns one NativeSession rclcpp node",
            policies=("direct_cpp", "native_node_authority"),
            metadata={
                "name": str(node_name),
                "namespace": str(namespace or ""),
                "context": "direct_runtime",
                "cli_arguments": 0 if cli_args is None else len(cli_args),
                "use_global_arguments": use_global_arguments,
                "enable_rosout": enable_rosout,
                "start_parameter_services": start_parameter_services,
                "parameter_overrides": override_count,
                "allow_undeclared_parameters": allow_undeclared_parameters,
                "automatically_declare_parameters_from_overrides": (
                    automatically_declare_parameters_from_overrides),
                "enable_logger_service": enable_logger_service,
                "parameter_cache_capacity": self._direct_cpp_parameter_cache_capacity,
            },
        )

    def _init_common(
        self, native_node, node_name, namespace, *,
        allow_undeclared_parameters=False,
        enable_parameter_cache=True,
    ):
        """Node-agnostic setup shared by every direct_cpp node kind.

        Takes an already-created native node (a real rclcpp::Node for
        DirectNode, a real rclcpp_lifecycle::LifecycleNode for
        DirectLifecycleNode -- both expose the same node-interface getters
        this method reads) and wires up everything that does not depend on
        which kind of node created it: logger, default callback group,
        entity-registry lists, the callback-containment/in-flight-counter
        machinery, and the parameter-cache bridge. ``node_name``/``namespace``
        are accepted for parity with the per-kind __init__ callers but are
        not otherwise read here -- the native node already baked them in at
        construction.

        ``enable_parameter_cache=False`` reuses the existing capacity-zero
        disabled path below (same as ``RCLCPPYY_DIRECT_PARAMETER_CACHE_CAPACITY
        =0``): parameters on a lifecycle node are not wired yet (a later
        slice), and the eager post-set-parameters-callback bridge this method
        would otherwise install is hard-typed to ``rclcpp::Node`` in the
        suite, which a ``rclcpp_lifecycle::LifecycleNode`` is not.
        """
        parameter_cache_capacity = (
            _parameter_cache_capacity() if enable_parameter_cache else 0)
        self._direct_cpp_node = native_node
        from rclpy.logging import get_logger

        native_logging = self._direct_cpp_node.get_node_logging_interface()
        self._logger = get_logger(_cpp_string(native_logging.get_logger_name()))
        self._direct_cpp_executor_ref = None
        self._allow_undeclared_parameters = allow_undeclared_parameters
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
        self._direct_cpp_action_servers = []
        # Callback-containment sink (defect A -- see PLAN-mte-unlock.md): the
        # shim applied at create_subscription/create_timer/create_service
        # hand-off points records a raise here instead of letting it cross
        # back into C++ from a native worker thread; the owning executor
        # drains and re-raises on its own spin/pump thread.
        self._direct_cpp_exception_sink_lock = threading.Lock()
        self._direct_cpp_exception_sink = []
        # In-flight callback counter (Slice 2.5 -- see PLAN-mte-unlock.md
        # Addendum): bumped by the containment shims around every native-
        # worker-dispatched user callback (subscription/timer/service, and
        # action callbacks via DirectActionServer._invoke_callback).
        # destroy_node()/destroy_*() wait for this to reach zero before
        # freeing an entity's callable, so nothing is freed while a worker
        # might still be invoking it.
        self._direct_cpp_in_flight_cv = threading.Condition()
        self._direct_cpp_in_flight_count = 0
        self._direct_cpp_clock = None
        self._direct_cpp_sleeper = None
        self._pre_set_parameters_callbacks = []
        self._on_set_parameters_callbacks = []
        self._post_set_parameters_callbacks = []
        self._direct_cpp_parameter_callback_bridges = {
            "pre": None,
            "on": None,
            "post": None,
        }
        self._direct_cpp_parameter_cache = {}
        self._direct_cpp_parameter_cache_capacity = parameter_cache_capacity
        self._direct_cpp_parameter_cache_misses = 0
        self._direct_cpp_parameter_cache_updates = 0
        self._direct_cpp_parameter_cache_invalidations = 0
        self._direct_cpp_parameter_cache_capacity_skips = 0
        self._direct_cpp_parameter_cache_max_size = 0
        self._direct_cpp_parameter_cache_pending_invalidations = {}
        self._direct_cpp_parameter_cache_declarations = {}
        self._direct_cpp_parameter_cache_disabled_reason = (
            None if parameter_cache_capacity else "capacity_zero")
        self._direct_cpp_parameter_cache_bridge_required = bool(
            parameter_cache_capacity)
        if self._direct_cpp_parameter_cache_bridge_required:
            self._install_parameter_callback_bridge("post")
        _runtime().attach(self, self._direct_cpp_node)

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

    def _native_executor_add(self, executor_native) -> None:
        """Attach this node's native entity to a raw rclcpp Executor.

        Private seam (PLAN-lifecycle.md 2.3.1-adjacent -- the plan's own
        seam analysis covered entity creation but missed executor
        membership): stock rclcpp::Executor::add_node has exactly two
        overloads, NodeBaseInterface::SharedPtr and
        std::shared_ptr<rclcpp::Node> (executor.hpp:200,208) -- both of
        which DirectNode's plain rclcpp::Node satisfies directly.
        DirectLifecycleNode overrides this because
        rclcpp_lifecycle::LifecycleNode is neither, and must instead go
        through the suite's NativeLifecycleNode.attach_executor().
        """
        executor_native.add_node(self._direct_cpp_node)

    def _native_executor_remove(self, executor_native) -> None:
        executor_native.remove_node(self._direct_cpp_node)

    def _native_release(self, node) -> None:
        """Release this node's native resource from the owning session.

        Private seam, called from the tail of ``_DirectRuntime.detach()``.
        DirectNode's plain rclcpp::Node was tracked by
        ``NativeSession.create_node()`` and is released the same way here;
        DirectLifecycleNode overrides this to close its NativeLifecycleNode
        resource instead, since that entity is retained via
        ``register_resource()`` -- a separate list ``release_node()``
        does not search.
        """
        _runtime().session.release_node(node)

    def _wake_executor(self):
        executor = self.executor
        if executor is not None:
            executor.wake()

    def _record_callback_exception(self, exc) -> None:
        """Record a callback exception contained at a product hand-off
        point (see ``_contain_callback_exceptions``/
        ``_contain_service_callback_exceptions`` below) for the owning
        executor to drain and re-raise on its own spin/pump thread. Never
        raises itself -- that would defeat the containment this exists for.
        """
        with self._direct_cpp_exception_sink_lock:
            self._direct_cpp_exception_sink.append((exc, self))
        self._wake_executor()

    def _drain_callback_exceptions(self):
        """Return and clear this node's captured callback exceptions."""
        with self._direct_cpp_exception_sink_lock:
            drained = self._direct_cpp_exception_sink
            self._direct_cpp_exception_sink = []
        return drained

    def _enter_in_flight(self) -> None:
        with self._direct_cpp_in_flight_cv:
            self._direct_cpp_in_flight_count += 1

    def _exit_in_flight(self) -> None:
        with self._direct_cpp_in_flight_cv:
            self._direct_cpp_in_flight_count -= 1
            if self._direct_cpp_in_flight_count <= 0:
                self._direct_cpp_in_flight_cv.notify_all()

    def _wait_quiescent(self, timeout: float) -> bool:
        """Block until this node has no in-flight callback, or return False
        on timeout. The caller must not free anything on a False return --
        leak-safe beats crash-safe (PLAN-mte-unlock.md Addendum risk 2)."""
        deadline = time.monotonic() + timeout
        with self._direct_cpp_in_flight_cv:
            while self._direct_cpp_in_flight_count > 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return self._direct_cpp_in_flight_count <= 0
                self._direct_cpp_in_flight_cv.wait(timeout=remaining)
            return True

    def _defer_teardown(self, teardown) -> None:
        """Enqueue ``teardown`` on the owning executor's pump instead of
        running it here. Called only for a self-destroy (the current thread
        is already dispatching a callback for this node, per
        ``_is_dispatching_for``): waiting for our own in-flight count to
        reach zero from inside our own dispatch would self-deadlock. The
        pump thread drains this queue once genuinely quiescent -- see
        ``DirectExecutor._drain_deferred_teardown``.
        """
        executor = self.executor
        if executor is None:
            # No owning executor to defer to -- shouldn't normally happen
            # while dispatching, but run synchronously rather than drop the
            # request outright.
            teardown()
            return
        executor._enqueue_deferred_teardown(teardown)

    def _contain_callback_exceptions(self, callback):
        """Wrap a void-style user callback (subscription/timer) so a raise
        is captured into this node's exception sink instead of crossing
        back into C++ as an uncaught exception on a native worker thread
        (defect A -- see docs/plans/PLAN-mte-unlock.md). The owning
        executor drains the sink and re-raises the first captured
        exception on the spin/pump thread, matching stock's
        ``future.result()`` re-raise semantics.

        Also bumps this node's in-flight counter and the calling thread's
        self-destroy marker around the call (Slice 2.5, Addendum Q1) --
        every native-worker-dispatched user callback funnels through here
        or through ``_contain_service_callback_exceptions``.
        """
        @functools.wraps(callback)
        def _contained(*args, **kwargs):
            self._enter_in_flight()
            _enter_dispatch(self)
            try:
                return callback(*args, **kwargs)
            except Exception as exc:
                self._record_callback_exception(exc)
                return None
            finally:
                _exit_dispatch(self)
                self._exit_in_flight()

        return _contained

    def _contain_service_callback_exceptions(self, callback):
        """Like ``_contain_callback_exceptions``, but for a service
        callback's ``(request, response) -> response`` contract: on a
        contained raise, the original (untouched) ``response`` is returned
        so the suite's dispatch bridge still receives a well-typed
        response to commit. Stock rclpy sends no reply at all when a
        service callback raises; this backend cannot suppress the native
        reply without a suite change, so a default-valued response is the
        closest safe containment (a documented differential, not a parity
        claim).

        Also bumps the in-flight counter and self-destroy marker exactly
        as ``_contain_callback_exceptions`` does -- see Slice 2.5.
        """
        @functools.wraps(callback)
        def _contained(request, response):
            self._enter_in_flight()
            _enter_dispatch(self)
            try:
                return callback(request, response)
            except Exception as exc:
                self._record_callback_exception(exc)
                return response
            finally:
                _exit_dispatch(self)
                self._exit_in_flight()

        return _contained

    def _contain_transition_callback(self, callback):
        """Wrap a lifecycle transition callback (``on_configure``, etc).

        Unlike the two shims above, a raise here is SWALLOWED and mapped to
        ``TransitionCallbackReturn.ERROR`` instead of being recorded for the
        executor to re-raise -- matching stock's own
        ``LifecycleNodeMixin.__execute_callback`` (node.py:315-320), which
        never re-raises a raising transition callback. A non-
        ``TransitionCallbackReturn`` result is likewise mapped to ``ERROR``:
        the value crosses back into a native ``static_cast<CallbackReturn>``
        (see ``rclcpp_kit.native_lifecycle``), so anything else would become
        an undefined enum value in C++ rather than a controlled outcome.

        Still bumps the in-flight counter and self-destroy marker exactly
        like the other containment shims (teardown safety, Slice 2.5) --
        the bridge runs on an executor worker thread (service dispatch) or
        the calling thread (``trigger_*``), either way inside the native
        node's own state-machine handler.
        """
        from rclpy.lifecycle.node import TransitionCallbackReturn

        @functools.wraps(callback)
        def _contained(state):
            self._enter_in_flight()
            _enter_dispatch(self)
            try:
                result = callback(state)
                if isinstance(result, TransitionCallbackReturn):
                    return int(result)
                return int(TransitionCallbackReturn.ERROR)
            except Exception:
                return int(TransitionCallbackReturn.ERROR)
            finally:
                _exit_dispatch(self)
                self._exit_in_flight()

        return _contained

    @property
    def default_callback_group(self):
        return self._default_callback_group

    @property
    def _callback_groups(self):
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
    def _action_clients(self):
        return list(self._direct_cpp_action_clients)

    @property
    def _action_servers(self):
        return list(self._direct_cpp_action_servers)

    def get_name(self):
        return str(self._require_node().get_name())

    def get_namespace(self):
        return str(self._require_node().get_namespace())

    def get_fully_qualified_name(self):
        return str(self._require_node().get_fully_qualified_name())

    def get_logger(self):
        return self._logger

    def _clock_sleeper(self):
        sleeper = self._direct_cpp_sleeper
        if sleeper is not None:
            return sleeper
        sleeper = self._native_create_clock_sleeper()
        self._direct_cpp_sleeper = sleeper
        return sleeper

    def _native_create_clock_sleeper(self):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind ``_clock_sleeper``. ``DirectLifecycleNode``
        overrides this -- no lifecycle-typed clock sleeper exists in the
        suite yet, so it fails closed rather than pass the raw lifecycle
        node into a ``rclcpp::Node``-typed native call."""
        return _runtime().session.create_native_clock_sleeper(self._require_node())

    def get_clock(self):
        clock = self._direct_cpp_clock
        if clock is not None:
            return clock
        from rclcppyy.direct_clock import wrap_node_clock

        native_node_clock = self._native_create_clock()
        node_ref = weakref.ref(self)

        def sleeper_provider():
            node = node_ref()
            if node is None:
                raise RuntimeError("direct_cpp node is destroyed")
            return node._clock_sleeper()

        clock = wrap_node_clock(native_node_clock, sleeper_provider=sleeper_provider)
        self._direct_cpp_clock = clock
        return clock

    def _native_create_clock(self):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind ``get_clock``. ``DirectLifecycleNode`` overrides
        this to retain the lifecycle node's own clock instead."""
        return _runtime().session.create_native_node_clock(self._require_node())

    def _parameter_modules(self):
        return _native_parameters, _direct_parameters

    def _store_direct_parameter_cache(self, name, facade):
        if self._direct_cpp_parameter_cache_disabled_reason is not None:
            return
        cache = self._direct_cpp_parameter_cache
        if name not in cache and len(cache) >= self._direct_cpp_parameter_cache_capacity:
            self._direct_cpp_parameter_cache_capacity_skips += 1
            return
        cache[name] = facade
        self._direct_cpp_parameter_cache_updates += 1
        self._direct_cpp_parameter_cache_max_size = max(
            self._direct_cpp_parameter_cache_max_size, len(cache))

    def _invalidate_direct_parameter_cache(self, name):
        removed = self._direct_cpp_parameter_cache.pop(
            name, _PARAMETER_CACHE_MISSING)
        if removed is not _PARAMETER_CACHE_MISSING:
            self._direct_cpp_parameter_cache_invalidations += 1

    def _disable_direct_parameter_cache(self, reason):
        cache = self._direct_cpp_parameter_cache
        self._direct_cpp_parameter_cache_invalidations += len(cache)
        cache.clear()
        self._direct_cpp_parameter_cache_disabled_reason = reason

    def _mark_direct_parameter_cache_invalidation(self, name):
        pending = self._direct_cpp_parameter_cache_pending_invalidations
        pending[name] = pending.get(name, 0) + 1

    def _unmark_direct_parameter_cache_invalidations(self, names):
        pending = self._direct_cpp_parameter_cache_pending_invalidations
        for name in names:
            count = pending.get(name)
            if count is None:
                continue
            remaining = count - 1
            if remaining:
                pending[name] = remaining
            else:
                del pending[name]

    def _update_direct_parameter_cache(self, parameter_list, invalidated_names):
        if self._direct_cpp_parameter_cache_disabled_reason is not None:
            return
        native_parameters, direct_parameters = self._parameter_modules()
        actions = []
        for facade in parameter_list:
            native = direct_parameters.native_parameter(facade)
            name = native.name
            if native.type_code != native_parameters.PARAMETER_NOT_SET:
                actions.append(("store", name, facade))
                continue
            declaration_kind = self._direct_cpp_parameter_cache_declarations.get(
                name)
            if declaration_kind == "value":
                actions.append(("store", name, facade))
                continue
            actions.append(("invalidate", name, None))

        for action, name, facade in actions:
            if action == "store":
                self._store_direct_parameter_cache(name, facade)
            else:
                if name not in self._direct_cpp_parameter_cache_declarations:
                    invalidated_names.append(name)
                    self._mark_direct_parameter_cache_invalidation(name)
                self._invalidate_direct_parameter_cache(name)

    def _set_direct_parameter_cache_hit_tracking(self, enabled):
        if not isinstance(enabled, bool):
            raise TypeError("parameter cache hit tracking must be a bool")
        cache = self._direct_cpp_parameter_cache
        if enabled and type(cache) is dict:
            self._direct_cpp_parameter_cache = _TrackingParameterCache(cache)
        elif not enabled and isinstance(cache, _TrackingParameterCache):
            self._direct_cpp_parameter_cache = dict(cache)

    def _direct_cpp_parameter_cache_stats(self):
        cache = self._direct_cpp_parameter_cache
        tracking = isinstance(cache, _TrackingParameterCache)
        reason = self._direct_cpp_parameter_cache_disabled_reason
        return {
            "enabled": reason is None,
            "disabled_reason": reason,
            "hit_tracking_enabled": tracking,
            "hits": cache.hits if tracking else None,
            "misses": self._direct_cpp_parameter_cache_misses,
            "updates": self._direct_cpp_parameter_cache_updates,
            "invalidations": self._direct_cpp_parameter_cache_invalidations,
            "capacity_skips": self._direct_cpp_parameter_cache_capacity_skips,
            "size": len(cache),
            "max_size": self._direct_cpp_parameter_cache_max_size,
            "capacity": self._direct_cpp_parameter_cache_capacity,
            "pending_invalidations": len(
                self._direct_cpp_parameter_cache_pending_invalidations),
        }

    def _raise_parameter_callback_exception(self):
        for kind in ("pre", "on", "post"):
            bridge = self._direct_cpp_parameter_callback_bridges[kind]
            if bridge is not None:
                exception = bridge.take_exception()
                if exception is not None:
                    raise _DirectParameterCallbackError(exception) from exception

    def _run_parameter_operation(self, operation, *, unwrap_callback=True):
        try:
            result = operation()
        except BaseException:
            try:
                self._raise_parameter_callback_exception()
            except _DirectParameterCallbackError as callback_error:
                if unwrap_callback:
                    raise callback_error.original from callback_error
                raise
            raise
        try:
            self._raise_parameter_callback_exception()
        except _DirectParameterCallbackError as callback_error:
            if unwrap_callback:
                raise callback_error.original from callback_error
            raise
        return result

    def _require_declared_parameter(self, name):
        if not self.has_parameter(name):
            if self._allow_undeclared_parameters:
                return
            from rclpy.exceptions import ParameterNotDeclaredException

            raise ParameterNotDeclaredException(name)

    def _prepare_parameter_declarations(self, namespace, parameters):
        import warnings

        from rclpy.exceptions import ParameterAlreadyDeclaredException
        from rclpy.validate_parameter_name import validate_parameter_name

        native_parameters, direct_parameters = self._parameter_modules()
        parameter_type = direct_parameters.parameter_class().Type
        prepared = []
        for index, parameter_tuple in enumerate(parameters):
            if not isinstance(parameter_tuple, tuple):
                raise TypeError(
                    "Parameter descriptor at index %d is not a tuple" % index)
            if len(parameter_tuple) < 1 or len(parameter_tuple) > 3:
                raise TypeError(
                    "Invalid parameter tuple length at index %d in parameters list: "
                    "%r; expecting length between 1 and 3" %
                    (index, parameter_tuple))
            name = parameter_tuple[0]
            if not isinstance(name, str):
                raise TypeError(
                    "First element %r at index %d in parameters list is not a str." %
                    (name, index))
            if namespace:
                name = "%s.%s" % (namespace, name)
            validate_parameter_name(name)
            second = parameter_tuple[1] if len(parameter_tuple) > 1 else None
            source_descriptor = parameter_tuple[2] if len(parameter_tuple) > 2 else None
            descriptor = direct_parameters.descriptor_to_cpp(
                source_descriptor, name=name)
            if len(parameter_tuple) == 1:
                warnings.warn(
                    "when declaring parameter named '%s', declaring a parameter only "
                    "providing its name is deprecated" % name,
                    stacklevel=3,
                )
                descriptor.dynamic_typing = True
            if isinstance(second, parameter_type):
                if second == parameter_type.NOT_SET:
                    raise ValueError(
                        "Cannot declare parameter {%s} as statically typed of type "
                        "NOT_SET" % name)
                if bool(descriptor.dynamic_typing):
                    raise ValueError(
                        "When declaring parameter {%s} passing a descriptor with "
                        "`dynamic_typing=True` is not allowed when the parameter type "
                        "is provided" % name)
                descriptor.type = int(second.value)
                prepared.append(("type", name, second, descriptor))
                continue
            facade = direct_parameters.parameter_class()(name, value=second)
            if not bool(descriptor.dynamic_typing):
                if facade.type_ == parameter_type.NOT_SET:
                    raise ValueError(
                        "Cannot declare a statically typed parameter with default value "
                        "of type PARAMETER_NOT_SET")
                descriptor.type = int(facade.type_.value)
            prepared.append(("value", name, facade, descriptor))

        duplicates = [
            name for _kind, name, _value, _descriptor in prepared
            if native_parameters.has_parameter(self._require_node(), name)
        ]
        if duplicates:
            raise ParameterAlreadyDeclaredException(duplicates)
        return prepared

    def declare_parameter(
        self, name, value=None, descriptor=None, ignore_override=False
    ):
        if value is None and descriptor is None:
            declaration = (name,)
        elif descriptor is None:
            declaration = (name, value)
        else:
            declaration = (name, value, descriptor)
        return self.declare_parameters(
            "", [declaration], ignore_override=ignore_override)[0]

    def declare_parameters(
        self, namespace, parameters, ignore_override=False
    ):
        if not isinstance(namespace, str):
            raise TypeError("parameter namespace must be a str")
        if not isinstance(parameters, list):
            raise TypeError("parameters must be a list")
        native_parameters, direct_parameters = self._parameter_modules()
        prepared = self._prepare_parameter_declarations(namespace, parameters)
        result = []
        for kind, name, value, descriptor in prepared:
            if kind == "type":
                def operation(n=name, v=value, d=descriptor):
                    return native_parameters.declare_parameter_type(
                        self._require_node(), n, int(v.value), d,
                        ignore_override=ignore_override)
            else:
                def operation(v=value, d=descriptor):
                    return native_parameters.declare_parameter(
                        self._require_node(),
                        direct_parameters.native_parameter(v),
                        d,
                        ignore_override=ignore_override)
            self._direct_cpp_parameter_cache_declarations[name] = kind
            try:
                declared = self._run_parameter_operation(
                    operation, unwrap_callback=False)
            except _DirectParameterCallbackError as callback_error:
                raise callback_error.original from callback_error
            except BaseException as exception:
                from rclpy.exceptions import InvalidParameterValueException

                if self.has_parameter(name):
                    raise
                raise InvalidParameterValueException(
                    name,
                    None if kind == "type" else value.value,
                    str(exception),
                ) from exception
            finally:
                self._direct_cpp_parameter_cache_declarations.pop(name, None)
            facade = direct_parameters.wrap_native(declared)
            try:
                if kind == "type":
                    self._invalidate_direct_parameter_cache(name)
                else:
                    self._store_direct_parameter_cache(name, facade)
            except BaseException:
                self._disable_direct_parameter_cache(
                    "declare_update_failure")
            result.append(facade)
        return result

    def has_parameter(self, name):
        native_parameters, _direct_parameters = self._parameter_modules()
        return native_parameters.has_parameter(self._require_node(), name)

    def get_parameter(self, name):
        try:
            return self._direct_cpp_parameter_cache[name]
        except KeyError:
            self._direct_cpp_parameter_cache_misses += 1
        declaration_kind = self._direct_cpp_parameter_cache_declarations.get(name)
        if declaration_kind == "type":
            from rclpy.exceptions import ParameterUninitializedException

            raise ParameterUninitializedException(name)
        if name in self._direct_cpp_parameter_cache_pending_invalidations:
            if self._allow_undeclared_parameters:
                return _direct_parameters.parameter_class()(name)
            from rclpy.exceptions import ParameterNotDeclaredException

            raise ParameterNotDeclaredException(name)
        native_parameters, direct_parameters = self._parameter_modules()
        status, parameter = self._native_get_parameter_checked(name)
        if status == native_parameters.CHECKED_PARAMETER_MISSING:
            if self._allow_undeclared_parameters:
                return direct_parameters.parameter_class()(name)
            from rclpy.exceptions import ParameterNotDeclaredException

            raise ParameterNotDeclaredException(name)
        if status == native_parameters.CHECKED_PARAMETER_STATIC_UNINITIALIZED:
            from rclpy.exceptions import ParameterUninitializedException

            raise ParameterUninitializedException(name)
        if parameter is None:
            raise RuntimeError("checked native parameter result has no value")
        facade = direct_parameters.wrap_native(parameter)
        try:
            if name not in (
                self._direct_cpp_parameter_cache_pending_invalidations
            ):
                self._store_direct_parameter_cache(name, facade)
        except BaseException:
            self._disable_direct_parameter_cache("get_update_failure")
        return facade

    def _native_get_parameter_checked(self, name):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind ``get_parameter``. ``native_parameters.declare_
        parameter``/``get_parameter``/``set_parameters``/``has_parameter``
        are plain duck-typed passthroughs that already work on any node
        exposing ``rclcpp::Node``'s parameter method names (lifecycle
        included, PLAN-lifecycle.md S5) -- but ``get_parameter_checked`` is a
        dedicated compiled C++ helper hard-typed to
        ``std::shared_ptr<rclcpp::Node>``, so it rejects a lifecycle node's
        raw shared_ptr outright. ``DirectLifecycleNode`` overrides this to
        replicate the same ``(status, parameter)`` contract from the plain
        ``has_parameter``/``get_parameter`` calls instead."""
        native_parameters, _direct_parameters = self._parameter_modules()
        return native_parameters.get_parameter_checked(self._require_node(), name)

    def get_parameters(self, names):
        if not isinstance(names, list):
            raise TypeError("names must be a list")
        return [self.get_parameter(name) for name in names]

    def get_parameter_or(self, name, alternative_value=None):
        if self.has_parameter(name):
            return self.get_parameter(name)
        if alternative_value is None:
            _native, direct_parameters = self._parameter_modules()
            return direct_parameters.parameter_class()(name)
        return alternative_value

    def get_parameter_type(self, name):
        return self.get_parameter_types([name])[0]

    def get_parameter_types(self, names):
        if not isinstance(names, list):
            raise TypeError("names must be a list")
        native_parameters, _direct_parameters = self._parameter_modules()
        result = []
        for name in names:
            if not self.has_parameter(name):
                self._require_declared_parameter(name)
                result.append(native_parameters.PARAMETER_NOT_SET)
                continue
            result.extend(native_parameters.get_parameter_types(
                self._require_node(), (name,)))
        return result

    def set_parameters(self, parameter_list):
        if not isinstance(parameter_list, list):
            raise TypeError("parameter_list must be a list")
        native_parameters, direct_parameters = self._parameter_modules()
        results = []
        for parameter in parameter_list:
            native = direct_parameters.native_parameter(parameter)
            if self._direct_cpp_parameter_callback_bridges["pre"] is None:
                self._require_declared_parameter(parameter.name)
                if native.type_code == native_parameters.PARAMETER_NOT_SET:
                    _unsupported(
                        "direct_cpp does not support implicit undeclare through NOT_SET")
            result = self._run_parameter_operation(
                lambda p=native: native_parameters.set_parameters_atomically(
                    self._require_node(), (p,)))
            results.append(result)
        return results

    def set_parameters_atomically(self, parameter_list):
        if not isinstance(parameter_list, list):
            raise TypeError("parameter_list must be a list")
        native_parameters, direct_parameters = self._parameter_modules()
        native = tuple(
            direct_parameters.native_parameter(parameter)
            for parameter in parameter_list)
        if self._direct_cpp_parameter_callback_bridges["pre"] is None:
            for parameter in parameter_list:
                self._require_declared_parameter(parameter.name)
            if any(
                parameter.type_code == native_parameters.PARAMETER_NOT_SET
                for parameter in native
            ):
                _unsupported(
                    "direct_cpp does not support implicit undeclare through NOT_SET")
        return self._run_parameter_operation(
            lambda: native_parameters.set_parameters_atomically(
                self._require_node(), native))

    def describe_parameter(self, name):
        return self.describe_parameters([name])[0]

    def describe_parameters(self, names):
        if not isinstance(names, list):
            raise TypeError("names must be a list")
        native_parameters, direct_parameters = self._parameter_modules()
        result = []
        for name in names:
            if not self.has_parameter(name):
                self._require_declared_parameter(name)
                result.append(direct_parameters.descriptor_to_cpp(None))
                continue
            result.extend(native_parameters.describe_parameters(
                self._require_node(), (name,)))
        return result

    def list_parameters(self, prefixes, depth):
        if not isinstance(prefixes, list):
            raise TypeError("The prefixes argument must be a list")
        if not all(isinstance(prefix, str) for prefix in prefixes):
            raise TypeError("All prefixes must be instances of type str")
        native_parameters, _direct_parameters = self._parameter_modules()
        return native_parameters.list_parameters(
            self._require_node(), prefixes, depth)

    def _install_parameter_callback_bridge(self, kind):
        if self._direct_cpp_parameter_callback_bridges[kind] is not None:
            return
        native_parameters, direct_parameters = self._parameter_modules()

        def facades(values):
            return [direct_parameters.wrap_native(value) for value in values]

        if kind == "pre":
            def dispatch(values):
                original = facades(values)
                modified = []
                for callback in self._pre_set_parameters_callbacks:
                    modified.extend(callback(original))
                native = tuple(
                    direct_parameters.native_parameter(value)
                    for value in modified)
                for value in modified:
                    self._require_declared_parameter(value.name)
                if any(
                    value.type_code == native_parameters.PARAMETER_NOT_SET
                    for value in native
                ):
                    _unsupported(
                        "direct_cpp does not support implicit undeclare through NOT_SET")
                return native

            bridge = native_parameters.add_pre_set_parameters_callback(
                self._require_node(), dispatch)
        elif kind == "on":
            def dispatch(values):
                parameter_list = facades(values)
                for callback in self._on_set_parameters_callbacks:
                    result = direct_parameters.result_to_cpp(
                        callback(parameter_list))
                    if not bool(result.successful):
                        return result
                return native_parameters.make_set_parameters_result(True)

            bridge = native_parameters.add_on_set_parameters_callback(
                self._require_node(), dispatch)
        else:
            def dispatch(values):
                parameter_list = facades(values)
                invalidated_names = []
                try:
                    self._update_direct_parameter_cache(
                        parameter_list, invalidated_names)
                except BaseException:
                    self._disable_direct_parameter_cache(
                        "post_update_failure")
                try:
                    for callback in self._post_set_parameters_callbacks:
                        callback(parameter_list)
                finally:
                    self._unmark_direct_parameter_cache_invalidations(
                        invalidated_names)

            bridge = native_parameters.add_post_set_parameters_callback(
                self._require_node(), dispatch)
        self._direct_cpp_parameter_callback_bridges[kind] = bridge

    def add_pre_set_parameters_callback(self, callback):
        if not callable(callback):
            raise TypeError("Callback must be callable")
        self._install_parameter_callback_bridge("pre")
        self._pre_set_parameters_callbacks.insert(0, callback)

    def add_on_set_parameters_callback(self, callback):
        if not callable(callback):
            raise TypeError("Callback must be callable")
        self._install_parameter_callback_bridge("on")
        self._on_set_parameters_callbacks.insert(0, callback)

    def add_post_set_parameters_callback(self, callback):
        if not callable(callback):
            raise TypeError("Callback must be callable")
        self._install_parameter_callback_bridge("post")
        self._post_set_parameters_callbacks.insert(0, callback)

    def _remove_parameter_callback(self, kind, callbacks, callback):
        callbacks.remove(callback)
        if not callbacks:
            if (
                kind == "post" and
                self._direct_cpp_parameter_cache_bridge_required
            ):
                return
            bridge = self._direct_cpp_parameter_callback_bridges[kind]
            if bridge is not None:
                bridge.close()
                self._direct_cpp_parameter_callback_bridges[kind] = None

    def remove_pre_set_parameters_callback(self, callback):
        self._remove_parameter_callback(
            "pre", self._pre_set_parameters_callbacks, callback)

    def remove_on_set_parameters_callback(self, callback):
        self._remove_parameter_callback(
            "on", self._on_set_parameters_callbacks, callback)

    def remove_post_set_parameters_callback(self, callback):
        self._remove_parameter_callback(
            "post", self._post_set_parameters_callbacks, callback)

    def _close_parameter_callbacks(self):
        for kind, bridge in self._direct_cpp_parameter_callback_bridges.items():
            if bridge is not None:
                bridge.close()
                self._direct_cpp_parameter_callback_bridges[kind] = None
        self._pre_set_parameters_callbacks.clear()
        self._on_set_parameters_callbacks.clear()
        self._post_set_parameters_callbacks.clear()
        self._disable_direct_parameter_cache("destroyed")

    def undeclare_parameter(self, name):
        native_parameters, _direct_parameters = self._parameter_modules()
        if not native_parameters.has_parameter(self._require_node(), name):
            from rclpy.exceptions import ParameterNotDeclaredException

            raise ParameterNotDeclaredException(name)
        descriptor = native_parameters.describe_parameters(
            self._require_node(), (name,))[0]
        if bool(descriptor.read_only):
            from rclpy.exceptions import ParameterImmutableException

            raise ParameterImmutableException(name)
        if not bool(descriptor.dynamic_typing):
            _unsupported(
                "direct_cpp cannot undeclare statically typed parameters on "
                "ROS 2 Jazzy: public rclcpp rejects this operation although "
                "rclpy permits it")
        native_parameters.undeclare_parameter(self._require_node(), name)
        try:
            self._invalidate_direct_parameter_cache(name)
        except BaseException:
            self._disable_direct_parameter_cache(
                "undeclare_invalidation_failure")

    def set_descriptor(self, name, descriptor, alternative_value=None):
        _native_parameters, direct_parameters = self._parameter_modules()
        direct_parameters.descriptor_to_cpp(descriptor, name=name)
        if alternative_value is not None and not isinstance(
                alternative_value,
                cppyy.gbl.rcl_interfaces.msg.ParameterValue):
            raise TypeError(
                "alternative_value must be an actual direct_cpp C++ "
                "ParameterValue")
        _unsupported(
            "direct_cpp cannot support set_descriptor on ROS 2 Jazzy: public "
            "rclcpp has no descriptor mutation operation, and emulating it "
            "with undeclare/redeclare would change rclpy atomicity, callbacks, "
            "and parameter events")

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
            "qos_overriding_options": qos_overriding_options is not None,
            "publisher_class": publisher_class is not None,
        }
        self._reject_entity_options("publisher", requested)
        from rclcpp_kit import direct_entities
        from rclpy.event_handler import PublisherEventCallbacks

        direct_entities.resolve_supported_type(msg_type)
        raw_events = _extract_event_callbacks(
            event_callbacks, _PUBLISHER_EVENT_FIELDS, PublisherEventCallbacks)
        qos, normalized_qos = _lower_entity_qos(qos_profile)
        group, native_group = self._resolve_callback_group(callback_group)
        # Every event callback dispatches on a native MTE worker exactly like
        # a subscription/timer callback -- route it through the same
        # containment shim (defect A, PLAN-mte-unlock.md) so a raise is
        # captured instead of crossing back into C++ uncaught, and so the
        # in-flight/quiescence counter sees it while dispatching.
        contained_events = {
            name: self._contain_callback_exceptions(event_callback)
            for name, event_callback in raw_events.items()
        }
        try:
            native = self._native_create_publisher(
                msg_type,
                str(topic),
                qos,
                callback_group=native_group,
                event_callbacks=contained_events or None,
            )
        except direct_entities.QoSEventUnsupported as exc:
            from rclpy.event_handler import UnsupportedEventTypeError

            raise UnsupportedEventTypeError(str(exc)) from exc
        publisher = DirectPublisher(
            msg_type,
            topic,
            normalized_qos,
            self._logger_name(),
            native,
            group,
        )
        publisher.event_handlers = [
            _DirectEventHandler(name, raw_events[name])
            for name in _PUBLISHER_EVENT_FIELDS if name in raw_events
        ]
        group.add_entity(publisher)
        self._direct_cpp_publishers.append(publisher)
        self._record_entity("publisher", topic, msg_type)
        return publisher

    def _native_create_publisher(
        self, msg_type, topic, qos, *, callback_group, event_callbacks
    ):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind the public, inherited ``create_publisher``.
        ``DirectLifecycleNode`` overrides this to create the publisher on the
        lifecycle node's own ``create_publisher<>()`` instead (native
        activation gating) -- everything else in ``create_publisher`` above
        (QoS lowering, event-callback containment, entity bookkeeping) is
        node-agnostic and stays here, unmodified and inherited."""
        from rclcpp_kit import direct_entities

        return direct_entities.create_managed_publisher(
            self._require_node(), msg_type, topic, qos,
            callback_group=callback_group, event_callbacks=event_callbacks)

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
            "raw": bool(raw),
        }
        self._reject_entity_options("subscription", requested)
        from rclcpp_kit import direct_entities
        from rclpy.event_handler import SubscriptionEventCallbacks

        direct_entities.resolve_supported_type(msg_type)
        if not callable(callback):
            raise TypeError("subscription callback must be callable")
        raw_events = _extract_event_callbacks(
            event_callbacks, _SUBSCRIPTION_EVENT_FIELDS, SubscriptionEventCallbacks)
        content_filter = _extract_content_filter(content_filter_options)
        qos_overriding = _extract_qos_overriding(qos_overriding_options)
        with_message_info = self._validate_subscription_callback(callback)
        qos, normalized_qos = _lower_entity_qos(qos_profile)
        group, native_group = self._resolve_callback_group(callback_group)
        # Contain the user callback before it becomes a native std::function
        # (defect A -- see docs/plans/PLAN-mte-unlock.md): a raise inside it
        # is captured into this node's exception sink instead of crossing
        # back into C++ off a native MultiThreadedExecutor worker thread.
        # DirectSubscription below keeps the original, unwrapped callback
        # for introspection parity.
        contained_callback = self._contain_callback_exceptions(callback)
        # Event callbacks dispatch on a native MTE worker exactly like the
        # subscription callback above -- same shim, same rationale.
        contained_events = {
            name: self._contain_callback_exceptions(event_callback)
            for name, event_callback in raw_events.items()
        }
        if (
            not contained_events
            and content_filter is None
            and not qos_overriding
            and self._native_subscription_shared_lease_supported()
            and "subscription_shared_lease" in _runtime().optimizations
        ):
            from rclcpp_kit import direct_subscription_lease

            native = direct_subscription_lease.create_subscription_lease(
                self._require_node(),
                msg_type,
                str(topic),
                contained_callback,
                qos,
                with_message_info=with_message_info,
                callback_group=native_group,
            )
        else:
            # The shared-lease optimization has no event-callback/content-
            # filter/qos-overriding parameter; an event-bearing,
            # content-filtered, or qos-overriding subscription always takes
            # this route below, even when the optimization is otherwise
            # active for this node -- likewise a lifecycle node (the lease
            # path is rclcpp::Node-typed, PLAN-lifecycle.md §3.7), which
            # always falls back here regardless of the runtime optimization
            # flag (`_native_subscription_shared_lease_supported` is False).
            # Not a parity loss: the lease is an internal handoff
            # optimization, not observable API surface -- this path is still
            # full parity.
            try:
                native = self._native_create_subscription(
                    msg_type,
                    str(topic),
                    contained_callback,
                    qos,
                    with_message_info=with_message_info,
                    callback_group=native_group,
                    event_callbacks=contained_events or None,
                    content_filter=content_filter,
                    qos_overriding=qos_overriding,
                )
            except direct_entities.QoSEventUnsupported as exc:
                from rclpy.event_handler import UnsupportedEventTypeError

                raise UnsupportedEventTypeError(str(exc)) from exc
            except direct_entities.ContentFilterUnsupported as exc:
                raise ContentFilterUnsupportedError(str(exc)) from exc
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
        subscription.event_handlers = [
            _DirectEventHandler(name, raw_events[name])
            for name in _SUBSCRIPTION_EVENT_FIELDS if name in raw_events
        ]
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

    def _native_subscription_shared_lease_supported(self) -> bool:
        """Private seam: whether the ``subscription_shared_lease`` runtime
        optimization may apply to this node's subscriptions. ``DirectNode``
        supports it; ``DirectLifecycleNode`` overrides this to ``False``
        because ``direct_subscription_lease`` is ``rclcpp::Node``-typed
        (PLAN-lifecycle.md §3.7) -- the optimization is an internal handoff
        detail, not observable API surface, so disabling it here is a
        silent, parity-preserving fallback to ``_native_create_subscription``
        below, not a behavior change."""
        return True

    def _native_create_subscription(
        self, msg_type, topic, callback, qos, *,
        with_message_info, callback_group, event_callbacks,
        content_filter, qos_overriding,
    ):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind the public, inherited ``create_subscription``.
        ``DirectLifecycleNode`` overrides this to create the subscription on
        the lifecycle node's own ``create_subscription<>()`` instead."""
        from rclcpp_kit import direct_entities

        return direct_entities.create_subscription(
            self._require_node(), msg_type, topic, callback, qos,
            with_message_info=with_message_info, callback_group=callback_group,
            event_callbacks=event_callbacks, content_filter=content_filter,
            qos_overriding=qos_overriding)

    def create_timer(
        self,
        timer_period_sec,
        callback,
        callback_group=None,
        clock=None,
        autostart=True,
    ):
        if clock is not None and clock is not self.get_clock():
            _unsupported(
                "create_timer honors only the node's own clock or None; "
                "standalone/foreign clocks are not supported")
        if not isinstance(autostart, bool):
            raise TypeError("timer autostart must be a bool")
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

        group, native_group = self._resolve_callback_group(callback_group)
        # Contain the user callback before it becomes a native std::function
        # -- same rationale as create_subscription above.
        contained_callback = self._contain_callback_exceptions(callback)
        timer = self._native_create_timer(
            period_ns,
            contained_callback,
            callback_group=native_group,
            autostart=autostart,
        )
        timer.callback_group = group
        group.add_entity(timer)
        self._direct_cpp_timers.append(timer)
        record_decision(
            "entities",
            "cpp",
            "direct rclcpp GenericTimer with Python callback on the node's own "
            "ROS clock -- sim-time-aware, matching stock's create_timer(clock="
            "None) default; standalone/foreign clocks are not supported",
            policies=("direct_cpp", "native_timer_authority", "no_conversion"),
            metadata={
                "entity_type": "timer",
                "period_ns": period_ns,
                "autostart": autostart,
                "clock": "ros",
                "ros_clock_support": "managed_clock_timers",
                "callback_handoff": "direct_std_function",
                "creation_route": timer.creation_route,
                "native_type": timer.native_type_name,
            },
        )
        return timer

    def _native_create_timer(self, period_ns, callback, *, callback_group, autostart):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind the public, inherited ``create_timer``.
        ``DirectLifecycleNode`` overrides this to create the timer on the
        lifecycle node's own ``create_wall_timer<>()`` instead."""
        from rclcpp_kit import direct_entities

        return direct_entities.create_clock_timer(
            self._require_node(), period_ns, callback,
            callback_group=callback_group, autostart=autostart)

    def create_rate(self, frequency, clock=None):
        if frequency <= 0:
            raise ValueError("frequency must be > 0")
        if clock is not None and clock is not self.get_clock():
            _unsupported(
                "direct_cpp create_rate honors only the node's own clock or "
                "None; standalone/foreign clocks are not supported")
        from rclcppyy.direct_clock import DirectRate

        period_ns = int(1e9 / frequency)
        return DirectRate(
            self._clock_sleeper(), self.get_clock(), period_ns, _runtime().context)

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
        native_client = self._native_create_client(
            srv_type, str(srv_name), callback_group=native_group)
        client = DirectClient(
            self, srv_type, str(srv_name), qos, native_client, group)
        group.add_entity(client)
        self._direct_cpp_clients.append(client)
        self._record_service_entity(
            "client", client.srv_name, binding, client)
        return client

    def _native_create_client(self, srv_type, srv_name, *, callback_group):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind the public, inherited ``create_client``.
        ``DirectLifecycleNode`` overrides this to create the client on the
        lifecycle node's own ``create_client<>()`` instead."""
        return _runtime().require_session().create_native_client(
            self._require_node(), srv_type, srv_name, callback_group=callback_group)

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
        # Contain the user callback before it becomes a native std::function
        # -- same rationale as create_subscription; the service variant
        # preserves the (request, response) -> response contract on a
        # contained raise (see _contain_service_callback_exceptions).
        contained_callback = self._contain_service_callback_exceptions(callback)
        native_service = self._native_create_service(
            srv_type, str(srv_name), contained_callback,
            callback_group=native_group)
        service = DirectService(
            srv_type, callback, qos, native_service, group)
        group.add_entity(service)
        self._direct_cpp_services.append(service)
        self._record_service_entity(
            "service", service.srv_name, binding, service)
        return service

    def _native_create_service(self, srv_type, srv_name, callback, *, callback_group):
        """Private seam (PLAN-lifecycle.md §2.3.1): the one node-type-specific
        native call behind the public, inherited ``create_service``.
        ``DirectLifecycleNode`` overrides this to create the service on the
        lifecycle node's own ``create_service<>()`` instead."""
        return _runtime().require_session().create_python_service(
            self._require_node(), srv_type, srv_name, callback,
            callback_group=callback_group)

    def _quiesce_or_raise(self, what: str) -> None:
        """Wait for this node to have no in-flight callback before freeing
        ``what``, or fail loud on timeout (Slice 2.5 -- see
        PLAN-mte-unlock.md Addendum: destroying a callable while a worker
        might still be invoking it is the defect-A-adjacent UAF class that
        crashed the self-destroy probe). Never called from the thread that
        is itself dispatching for this node -- callers must defer instead
        (``_is_dispatching_for`` / ``_defer_teardown``)."""
        if not self._wait_quiescent(_DESTROY_QUIESCENCE_TIMEOUT_SEC):
            raise RuntimeError(
                "direct_cpp node still has an in-flight callback after "
                "%.1fs; refusing to free %s (leak-safe beats crash-safe -- "
                "PLAN-mte-unlock.md Addendum risk 2)"
                % (_DESTROY_QUIESCENCE_TIMEOUT_SEC, what)
            )

    def destroy_timer(self, timer):
        if timer not in self._direct_cpp_timers:
            return False
        if _is_dispatching_for(self):
            self._defer_teardown(lambda: self._destroy_timer_now(timer))
            return True
        return self._destroy_timer_now(timer)

    def _destroy_timer_now(self, timer) -> bool:
        for index, candidate in enumerate(self._direct_cpp_timers):
            if timer is candidate:
                self._quiesce_or_raise("a timer")
                self._discard_group_entity(candidate)
                candidate.destroy()
                del self._direct_cpp_timers[index]
                return True
        return False

    def destroy_rate(self, rate):
        from rclcppyy.direct_clock import DirectRate

        if not isinstance(rate, DirectRate):
            raise TypeError("destroy_rate requires a direct_cpp Rate object")
        rate.destroy()
        return True

    def destroy_publisher(self, publisher):
        for index, candidate in enumerate(self._direct_cpp_publishers):
            if publisher is candidate:
                del self._direct_cpp_publishers[index]
                self._discard_group_entity(candidate)
                return candidate._close()
        return False

    def destroy_subscription(self, subscription):
        if subscription not in self._direct_cpp_subscriptions:
            return False
        if _is_dispatching_for(self):
            self._defer_teardown(lambda: self._destroy_subscription_now(subscription))
            return True
        return self._destroy_subscription_now(subscription)

    def _destroy_subscription_now(self, subscription) -> bool:
        for index, candidate in enumerate(self._direct_cpp_subscriptions):
            if subscription is candidate:
                self._quiesce_or_raise("a subscription")
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
        if service not in self._direct_cpp_services:
            return False
        if _is_dispatching_for(self):
            self._defer_teardown(lambda: self._destroy_service_now(service))
            return True
        return self._destroy_service_now(service)

    def _destroy_service_now(self, service) -> bool:
        for index, candidate in enumerate(self._direct_cpp_services):
            if service is candidate:
                self._quiesce_or_raise("a service")
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

    def _discard_direct_action_server(self, action_server):
        try:
            self._direct_cpp_action_servers.remove(action_server)
        except ValueError:
            pass

    def _require_idle_action_server_callbacks(self, operation):
        for action_server in tuple(self._direct_cpp_action_servers):
            with action_server._lock:
                callback_active = bool(action_server._callback_depth)
            if callback_active:
                _unsupported(
                    "direct_cpp cannot %s from an active action-server callback" %
                    operation)

    def destroy_node(self):
        node = self._direct_cpp_node
        if node is None:
            return
        self._require_idle_action_server_callbacks("destroy the node")
        if _is_dispatching_for(self):
            self._defer_teardown(self._destroy_node_now)
            return
        self._destroy_node_now()

    def _destroy_node_now(self) -> None:
        """The actual destroy_node() teardown, run only once this node is
        known (or has been waited) quiescent -- either directly, from an
        external thread, or deferred to the owning executor's pump after a
        self-destroy (Slice 2.5, PLAN-mte-unlock.md Addendum Q2/Q3). Detach
        from the executor natively *before* waiting, so no worker collects
        this node's entities again while we wait for the ones already
        checked out to finish.
        """
        node = self._direct_cpp_node
        if node is None:
            return
        executor = self.executor
        if executor is not None:
            executor.remove_node(self)
            self._set_direct_executor(None)
        self._quiesce_or_raise("a node's entities")
        self._close_parameter_callbacks()
        while self._direct_cpp_publishers:
            self.destroy_publisher(self._direct_cpp_publishers[0])
        while self._direct_cpp_subscriptions:
            self._destroy_subscription_now(self._direct_cpp_subscriptions[0])
        for timer in tuple(self._direct_cpp_timers):
            timer.destroy()
        for client in tuple(self._direct_cpp_clients):
            client.close()
        for service in tuple(self._direct_cpp_services):
            service.close()
        for action_server in tuple(self._direct_cpp_action_servers):
            action_server.close()
        for action_client in tuple(self._direct_cpp_action_clients):
            action_client.close()
        self._direct_cpp_timers.clear()
        self._direct_cpp_clients.clear()
        self._direct_cpp_services.clear()
        self._direct_cpp_action_servers.clear()
        self._direct_cpp_action_clients.clear()
        self._close_direct_clock()
        self._release_callback_groups()
        _runtime().detach(self, node)
        self._direct_cpp_node = None

    def _close_direct_clock(self):
        sleeper = self._direct_cpp_sleeper
        if sleeper is not None:
            sleeper.close()
        self._direct_cpp_sleeper = None
        clock = self._direct_cpp_clock
        if clock is not None:
            clock.close()
        self._direct_cpp_clock = None

    def _mark_runtime_shutdown(self):
        self._close_parameter_callbacks()
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
        for action_server in tuple(self._direct_cpp_action_servers):
            action_server.close()
        for action_client in tuple(self._direct_cpp_action_clients):
            action_client.close()
        self._direct_cpp_timers.clear()
        self._direct_cpp_clients.clear()
        self._direct_cpp_services.clear()
        self._direct_cpp_action_servers.clear()
        self._direct_cpp_action_clients.clear()
        self._direct_cpp_publishers.clear()
        self._direct_cpp_subscriptions.clear()
        self._close_direct_clock()
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
        return self._logger.name

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

    def _poll_direct_entities(self):
        for action_server in tuple(self._direct_cpp_action_servers):
            action_server._poll_ready()
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


def _prepare_check_is_valid_msg_type(original, check_for_type_support):
    """Keep ``check_is_valid_msg_type`` parity for registered direct message aliases.

    3.A (direct_messages._mirror_type_support_metadata) deliberately withholds
    the four conversion capsules stock ``check_is_valid_msg_type`` asserts, so
    unpatched it would raise a misleading "this might be a service or action"
    ``RuntimeError`` for a perfectly valid rebound cppyy message. A registered
    direct message alias is accepted after running ``check_for_type_support``
    (matching stock's own "this also imports stuff we need later" side
    effect); anything else -- including an unregistered type -- delegates to
    stock, which rejects it precisely.
    """
    def check_is_valid_msg_type(msg_type):
        check_for_type_support(msg_type)
        installation = _MESSAGE_INSTALLATION
        if installation is not None and any(
            msg_type is binding.cpp_type for binding in installation.bindings
        ):
            return
        original(msg_type)
    return check_is_valid_msg_type


def activate(*, optimizations=(), interfaces=()) -> bool:
    """Install the complete first-slice surface, rolling back on any failure."""
    global _ACTION_INSTALLATION, _ACTIVE, _ACTIVE_INTERFACES
    global _ACTIVE_OPTIMIZATIONS
    global _MESSAGE_INSTALLATION, _PATCHES, _RUNTIME, _SERVICE_INSTALLATION
    normalized_optimizations = tuple(sorted(set(optimizations)))
    from rclcppyy import (
        direct_actions,
        direct_messages,
        direct_parameters,
        direct_services,
        direct_wait_for_message,
    )

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
        + direct_parameters.CONTROL_MESSAGE_INTERFACES
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
        import rclpy.parameter as parameter_module
        import rclpy.publisher as publisher_module
        import rclpy.subscription as subscription_module
        import rclpy.type_support as type_support_module
        import rclpy.wait_for_message as wait_for_message_module

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
        DirectParameter = direct_parameters.prepare(parameter_module.Parameter)
        direct_wait = direct_wait_for_message.prepare(
            wait_for_message_module.wait_for_message)
        direct_check_is_valid_msg_type = _prepare_check_is_valid_msg_type(
            type_support_module.check_is_valid_msg_type,
            type_support_module.check_for_type_support,
        )
        replacements = (
            (parameter_module, "Parameter", DirectParameter),
            (rclpy, "Parameter", DirectParameter),
            (node_module, "Parameter", DirectParameter),
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
            (
                action_server_module,
                "ServerGoalHandle",
                direct_actions.DirectServerGoalHandle,
            ),
            (rclpy, "init", _direct_init),
            (rclpy, "ok", _direct_ok),
            (rclpy, "shutdown", _direct_shutdown),
            (rclpy, "try_shutdown", _direct_try_shutdown),
            (rclpy, "get_global_executor", _direct_get_global_executor),
            (rclpy, "spin_once", _direct_spin_once),
            (rclpy, "spin", _direct_spin),
            (rclpy, "spin_until_future_complete", _direct_spin_until_future_complete),
            (wait_for_message_module, "wait_for_message", direct_wait),
            (
                type_support_module,
                "check_is_valid_msg_type",
                direct_check_is_valid_msg_type,
            ),
        )
        # The mirror only ever fires for a member verified structurally
        # identical to stock and payload-free (see rclcppyy._signature_mirror);
        # everything else keeps rendering its own, still-divergent signature.
        # DirectSingleThreadedExecutor/DirectMultiThreadedExecutor need no
        # entry here: they define no members of their own, so they already
        # read the base DirectExecutor's mirrored signatures through normal
        # inheritance.
        mirrored_classes = frozenset({
            "CallbackGroup",
            "MutuallyExclusiveCallbackGroup",
            "ReentrantCallbackGroup",
            "Executor",
            "Node",
            "Publisher",
            "Subscription",
        })
        mirrored_functions = frozenset({
            "init",
            "ok",
            "shutdown",
            "spin",
            "spin_once",
            "spin_until_future_complete",
            "get_global_executor",
        })
        for module, name, replacement in replacements:
            original = getattr(module, name)
            if name in mirrored_classes:
                mirror_class(replacement, original)
            elif name in mirrored_functions:
                mirror_function(replacement, original)
            setattr(module, name, replacement)
            patches.append((module, name, original, replacement))

        # Import rclpy.lifecycle only after Node is already rebound to
        # DirectNode above: LifecycleNode(LifecycleNodeMixin, Node) resolves
        # its base class at import time, and importing it any earlier would
        # build LifecycleNode on stock Node even under direct_cpp (R5).
        import rclpy.lifecycle as lifecycle_module

        from rclcppyy.direct_lifecycle import (
            DirectLifecycleNode,
            DirectLifecycleNodeMixin,
            DirectLifecyclePublisher,
        )

        mirror_class(DirectLifecycleNode, lifecycle_module.LifecycleNode)
        mirror_class(DirectLifecycleNodeMixin, lifecycle_module.LifecycleNodeMixin)
        mirror_class(DirectLifecyclePublisher, lifecycle_module.LifecyclePublisher)
        # LifecycleNode/LifecycleNodeMixin/LifecyclePublisher are each bound
        # at three names -- the package-level name, its package-level alias
        # (Node/NodeMixin/Publisher), and the defining submodule's own
        # binding -- all three must move together for every consumer of any
        # of them to see the direct facade (PLAN-lifecycle.md 3.1, 5.1
        # groups B/C/D).
        lifecycle_replacements = (
            (lifecycle_module, "LifecycleNode", DirectLifecycleNode),
            (lifecycle_module, "Node", DirectLifecycleNode),
            (lifecycle_module.node, "LifecycleNode", DirectLifecycleNode),
            (lifecycle_module, "LifecycleNodeMixin", DirectLifecycleNodeMixin),
            (lifecycle_module, "NodeMixin", DirectLifecycleNodeMixin),
            (
                lifecycle_module.node,
                "LifecycleNodeMixin",
                DirectLifecycleNodeMixin,
            ),
            (lifecycle_module, "LifecyclePublisher", DirectLifecyclePublisher),
            (lifecycle_module, "Publisher", DirectLifecyclePublisher),
            (
                lifecycle_module.publisher,
                "LifecyclePublisher",
                DirectLifecyclePublisher,
            ),
        )
        for module, name, replacement in lifecycle_replacements:
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
