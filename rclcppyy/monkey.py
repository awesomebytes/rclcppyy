"""Transparent acceleration patches over authoritative stock rclpy objects."""

from __future__ import annotations

import warnings
from functools import wraps
from inspect import signature

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.task import Future

from rclcppyy._status import record_decision
from rclcppyy.policy import BackendUnavailableError, resolve_policy


_original_create_publisher = Node.create_publisher
_original_create_subscription = Node.create_subscription
_original_create_timer = Node.create_timer
_original_create_service = Node.create_service
_original_create_client = Node.create_client
_original_create_guard_condition = Node.create_guard_condition
_original_set_parameters = Node.set_parameters
_original_set_parameters_atomically = Node.set_parameters_atomically
_original_publish = Publisher.publish
_original_spin = rclpy.spin
_original_spin_once = rclpy.spin_once
_original_single_threaded_spin = SingleThreadedExecutor.spin
_original_multi_threaded_spin = MultiThreadedExecutor.spin
_original_future_set_result = Future.set_result
_original_future_set_exception = Future.set_exception
_original_future_cancel = Future.cancel
_original_lifecycle_node_init = LifecycleNode.__init__
_original_action_client_init = ActionClient.__init__
_original_action_server_init = ActionServer.__init__

_PATCHED = False
_POLICY = resolve_policy()
_WARNED_FALLBACKS = set()
_OPTIMIZED_SPIN_TIMEOUT_SEC = 0.1


def _claim_once(owner, key):
    reported = getattr(owner, "_rclcppyy_reported_operations", None)
    if reported is None:
        reported = set()
        try:
            owner._rclcppyy_reported_operations = reported
        except (AttributeError, TypeError):
            return True
    if key in reported:
        return False
    reported.add(key)
    return True


def _warn_once(reason):
    if not _POLICY.warn_fallback or reason in _WARNED_FALLBACKS:
        return
    _WARNED_FALLBACKS.add(reason)
    warnings.warn("rclcppyy fell back to stock rclpy: %s" % reason, RuntimeWarning)


def _record_node_once(node):
    status_id = getattr(node, "_rclcppyy_status_id", None)
    if status_id is not None:
        return status_id
    status_id = record_decision(
        "nodes",
        "python",
        "stock rclpy Node and Context remain authoritative",
        policies=("stock_node_authority", _POLICY.name),
        metadata={
            "name": node.get_name(),
            "namespace": node.get_namespace(),
            "profile": _POLICY.name,
        },
    )
    node._rclcppyy_status_id = status_id
    return status_id


def _record_python_entity(
    node,
    entity_type,
    reason,
    metadata=None,
    *,
    warn=True,
    policies=None,
):
    if warn:
        _warn_once(reason)
    values = {
        "entity_type": entity_type,
        "node_id": _record_node_once(node),
        "profile": _POLICY.name,
    }
    values.update(metadata or {})
    record_decision(
        "entities",
        "python",
        reason,
        policies=policies or ("stock_fallback", _POLICY.name),
        metadata=values,
    )


def _record_python_operation(
    node,
    operation,
    reason,
    metadata=None,
    *,
    once_key=None,
    policies=None,
):
    if once_key is not None and not _claim_once(node, once_key):
        return
    _warn_once(reason)
    values = {
        "operation": operation,
        "node_id": _record_node_once(node),
        "profile": _POLICY.name,
    }
    values.update(metadata or {})
    record_decision(
        "operations",
        "python",
        reason,
        policies=policies or ("stock_fallback", _POLICY.name),
        metadata=values,
    )


def _record_runtime_operation(
    owner,
    operation,
    reason,
    metadata=None,
    *,
    once_key=None,
    warn=False,
    authority="stock_runtime_authority",
    policies=None,
):
    if once_key is not None and not _claim_once(owner, once_key):
        return
    if warn:
        _warn_once(reason)
    values = {"operation": operation, "profile": _POLICY.name}
    values.update(metadata or {})
    record_decision(
        "operations",
        "python",
        reason,
        policies=policies or (authority, _POLICY.name),
        metadata=values,
    )


def _unavailable(operation, reason):
    record_decision(
        "operations",
        "unsupported",
        reason,
        policies=("required_cpp", "fail_closed"),
        metadata={"operation": operation, "profile": _POLICY.name},
    )
    raise BackendUnavailableError(reason)


def _stock_entity(node, operation, entity_type, reason, create, metadata=None):
    """Create one stock entity or fail before invoking its constructor."""
    initializing = not hasattr(node, "_type_description_service")
    if _POLICY.require_cpp and not initializing:
        _unavailable(operation, reason)
    entity = create()
    values = metadata(entity) if callable(metadata) else (metadata or {})
    if initializing:
        values["requested_operation"] = operation
        _record_python_entity(
            node,
            entity_type,
            "stock Node constructor owns this compatibility entity",
            values,
            warn=False,
            policies=("stock_node_infrastructure", _POLICY.name),
        )
    else:
        _record_python_entity(node, entity_type, reason, values)
    return entity


def _stock_operation(node, operation, reason, invoke, metadata=None):
    """Run one stock operation or fail before it can mutate the node."""
    if _POLICY.require_cpp:
        _unavailable(operation, reason)
    result = invoke()
    values = metadata(result) if callable(metadata) else (metadata or {})
    _record_python_operation(node, operation, reason, values)
    return result


def _load_borrowed_publish():
    try:
        from rclcpp_kit import borrowed_publish
    except (ImportError, AttributeError) as exc:
        return None, "installed rclcpp_kit has no same-handle publisher route: %s" % exc
    return borrowed_publish, None


def _create_publisher_wrapper(
    self,
    msg_type,
    topic,
    qos_profile,
    *,
    callback_group=None,
    event_callbacks=None,
    qos_overriding_options=None,
    publisher_class=Publisher,
):
    initializing = not hasattr(self, "_type_description_service")
    if initializing:
        publisher = _original_create_publisher(
            self,
            msg_type,
            topic,
            qos_profile,
            callback_group=callback_group,
            event_callbacks=event_callbacks,
            qos_overriding_options=qos_overriding_options,
            publisher_class=publisher_class,
        )
        _record_python_entity(
            self,
            "publisher",
            "stock Node constructor owns this compatibility entity",
            metadata={
                "topic": publisher.topic_name,
                "requested_operation": "create_publisher",
            },
            warn=False,
            policies=("stock_node_infrastructure", _POLICY.name),
        )
        return publisher

    if not _POLICY.use_cpp_publisher:
        publisher = _original_create_publisher(
            self,
            msg_type,
            topic,
            qos_profile,
            callback_group=callback_group,
            event_callbacks=event_callbacks,
            qos_overriding_options=qos_overriding_options,
            publisher_class=publisher_class,
        )
        _record_python_entity(
            self,
            "publisher",
            "stock rclpy Publisher.publish remains authoritative by policy",
            metadata={
                "topic": publisher.topic_name,
                "callback_group_requested": callback_group is not None,
                "event_callbacks_requested": event_callbacks is not None,
                "qos_overrides_requested": qos_overriding_options is not None,
                "custom_publisher_class_requested": publisher_class is not Publisher,
            },
            warn=False,
            policies=("stock_publish_authority", _POLICY.name),
        )
        return publisher

    borrowed_publish, unavailable_reason = _load_borrowed_publish()
    route = None
    if borrowed_publish is not None:
        try:
            # Resolve/JIT before creating the endpoint. Required-C++ failure must
            # not leave a partially created stock entity behind.
            route = borrowed_publish.prepare(msg_type)
        except Exception as exc:
            unavailable_reason = "same-handle publisher preparation failed: %s" % exc

    if route is None and _POLICY.require_cpp:
        _unavailable("create_publisher", unavailable_reason)

    publisher = _original_create_publisher(
        self,
        msg_type,
        topic,
        qos_profile,
        callback_group=callback_group,
        event_callbacks=event_callbacks,
        qos_overriding_options=qos_overriding_options,
        publisher_class=publisher_class,
    )
    node_id = _record_node_once(self)
    if route is None:
        _record_python_entity(
            self,
            "publisher",
            unavailable_reason,
            metadata={
                "topic": publisher.topic_name,
                "callback_group_requested": callback_group is not None,
                "event_callbacks_requested": event_callbacks is not None,
                "qos_overrides_requested": qos_overriding_options is not None,
                "custom_publisher_class_requested": publisher_class is not Publisher,
            },
        )
        return publisher

    publisher._rclcppyy_publish_route = route
    publisher._rclcppyy_policy = _POLICY
    publisher._rclcppyy_reported_fallbacks = set()
    publisher._rclcppyy_publish_tainted = False
    publisher._rclcppyy_last_publish_backend = None
    record_decision(
        "entities",
        "cpp",
        "stock publisher endpoint uses a same-handle C++ publish route",
        policies=(
            "stock_entity_contract",
            "borrowed_rcl_handle",
            "python_to_cpp_message_conversion",
            _POLICY.name,
        ),
        metadata={
            "entity_type": "publisher",
            "node_id": node_id,
            "topic": publisher.topic_name,
            "message_type": route.cpp_type_name,
            "profile": _POLICY.name,
            "callback_group_requested": callback_group is not None,
            "event_callbacks_requested": event_callbacks is not None,
            "qos_overrides_requested": qos_overriding_options is not None,
            "custom_publisher_class_requested": publisher_class is not Publisher,
        },
    )
    return publisher


def _record_publish_fallback_once(publisher, reason):
    reported = getattr(publisher, "_rclcppyy_reported_fallbacks", None)
    if reported is not None:
        if reason in reported:
            return
        reported.add(reason)
    _warn_once(reason)
    record_decision(
        "operations",
        "python",
        reason,
        policies=("stock_fallback", _POLICY.name),
        metadata={
            "operation": "publish",
            "topic": getattr(publisher, "topic_name", None),
            "profile": _POLICY.name,
        },
    )


def _record_cpp_publish_once(publisher):
    if not _claim_once(publisher, "publish_cpp"):
        return
    policy = getattr(publisher, "_rclcppyy_policy", _POLICY)
    record_decision(
        "operations",
        "cpp",
        "same-handle C++ publish completed",
        policies=("borrowed_rcl_handle", policy.name),
        metadata={
            "operation": "publish",
            "topic": getattr(publisher, "topic_name", None),
            "profile": policy.name,
        },
    )


def _publish_wrapper(self, message):
    route = getattr(self, "_rclcppyy_publish_route", None)
    if route is None:
        return _original_publish(self, message)
    policy = getattr(self, "_rclcppyy_policy", _POLICY)
    if isinstance(message, (bytes, bytearray, memoryview)):
        reason = "serialized-byte publishing has no certified C++ route"
        if policy.require_cpp:
            _unavailable("publish", reason)
        _record_publish_fallback_once(self, reason)
        result = _original_publish(self, message)
        self._rclcppyy_publish_tainted = True
        self._rclcppyy_last_publish_backend = "python"
        return result
    try:
        result = route.publish(self, message)
    except TypeError as exc:
        reason = "same-handle C++ publish rejected the message: %s" % exc
        if policy.require_cpp:
            _unavailable("publish", reason)
        # Compatible mode preserves the stock exception contract for invalid
        # objects, but also records the complete-operation fallback.
        _record_publish_fallback_once(self, reason)
        result = _original_publish(self, message)
        self._rclcppyy_publish_tainted = True
        self._rclcppyy_last_publish_backend = "python"
        return result
    except Exception as exc:
        reason = "same-handle C++ publish failed: %s" % exc
        if policy.require_cpp:
            _unavailable("publish", reason)
        _record_publish_fallback_once(self, reason)
        result = _original_publish(self, message)
        self._rclcppyy_publish_tainted = True
        self._rclcppyy_last_publish_backend = "python"
        return result
    _record_cpp_publish_once(self)
    if not self._rclcppyy_publish_tainted:
        self._rclcppyy_last_publish_backend = "cpp"
    return result


def _create_subscription_wrapper(self, *args, **kwargs):
    reason = "subscription take/dispatch has no certified same-handle C++ route"
    topic = args[1] if len(args) > 1 else kwargs.get("topic")
    callback_group = kwargs.get("callback_group")
    event_callbacks = kwargs.get("event_callbacks")
    qos_overriding_options = kwargs.get("qos_overriding_options")
    content_filter_options = kwargs.get("content_filter_options")
    return _stock_entity(
        self,
        "create_subscription",
        "subscription",
        reason,
        lambda: _original_create_subscription(self, *args, **kwargs),
        metadata={
            "topic": topic,
            "raw_requested": bool(kwargs.get("raw", False)),
            "callback_group_requested": callback_group is not None,
            "event_callbacks_requested": event_callbacks is not None,
            "qos_overrides_requested": qos_overriding_options is not None,
            "content_filter_requested": content_filter_options is not None,
        },
    )


def _create_timer_wrapper(self, *args, **kwargs):
    reason = "timer/executor integration has no certified same-handle C++ route"
    period = args[0] if args else kwargs.get("timer_period_sec")
    return _stock_entity(
        self,
        "create_timer",
        "timer",
        reason,
        lambda: _original_create_timer(self, *args, **kwargs),
        metadata={"period_sec": period},
    )


def _create_service_wrapper(self, *args, **kwargs):
    reason = "service request/response has no certified same-handle C++ route"
    requested_name = args[1] if len(args) > 1 else kwargs.get("srv_name")
    return _stock_entity(
        self,
        "create_service",
        "service",
        reason,
        lambda: _original_create_service(self, *args, **kwargs),
        metadata=lambda service: {
            "requested_name": requested_name,
            "service_name": service.service_name,
        },
    )


def _create_client_wrapper(self, *args, **kwargs):
    reason = "client request/response has no certified same-handle C++ route"
    requested_name = args[1] if len(args) > 1 else kwargs.get("srv_name")
    return _stock_entity(
        self,
        "create_client",
        "client",
        reason,
        lambda: _original_create_client(self, *args, **kwargs),
        metadata=lambda client: {
            "requested_name": requested_name,
            "service_name": client.service_name,
        },
    )


def _create_guard_condition_wrapper(self, *args, **kwargs):
    reason = "guard conditions have no certified C++ executor route"
    return _stock_entity(
        self,
        "create_guard_condition",
        "guard_condition",
        reason,
        lambda: _original_create_guard_condition(self, *args, **kwargs),
    )


def _set_parameters_wrapper(self, parameter_list):
    reason = "parameter mutation has no certified same-handle C++ route"
    return _stock_operation(
        self,
        "set_parameters",
        reason,
        lambda: _original_set_parameters(self, parameter_list),
        metadata=lambda results: {
            "parameter_names": [parameter.name for parameter in parameter_list],
            "successful": all(result.successful for result in results),
        },
    )


def _set_parameters_atomically_wrapper(self, parameter_list):
    reason = "parameter mutation has no certified same-handle C++ route"
    return _stock_operation(
        self,
        "set_parameters_atomically",
        reason,
        lambda: _original_set_parameters_atomically(self, parameter_list),
        metadata=lambda result: {
            "parameter_names": [parameter.name for parameter in parameter_list],
            "successful": result.successful,
        },
    )


def _record_executor_exception(node, source_operation, exception):
    _record_python_operation(
        node,
        "callback_exception",
        "stock executor propagated an exception without translation",
        metadata={
            "source_operation": source_operation,
            "exception_type": type(exception).__name__,
        },
        once_key=("callback_exception", source_operation, type(exception).__name__),
    )


def _optimized_spin_metadata(outcome, **values):
    metadata = {
        "outcome": outcome,
        "bounded_wait_timeout_sec": _OPTIMIZED_SPIN_TIMEOUT_SEC,
        "mitigation": "signal_guard_lost_wake",
    }
    metadata.update(values)
    return metadata


def _bounded_rclpy_spin(node, executor=None):
    executor = rclpy.get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        while executor.context.ok():
            executor.spin_once(timeout_sec=_OPTIMIZED_SPIN_TIMEOUT_SEC)
    finally:
        executor.remove_node(node)


def _bounded_executor_spin(executor):
    executor._enter_spin()
    try:
        while executor._context.ok() and not executor._is_shutdown:
            executor._spin_once_impl(_OPTIMIZED_SPIN_TIMEOUT_SEC)
    finally:
        executor._exit_spin()


@wraps(_original_spin)
def _spin_wrapper(node, executor=None):
    optimized = _POLICY.allow_contract_changes
    reason = (
        "optimized profile bounds stock executor waits after signal shutdown"
        if optimized else
        "rclpy.spin has no certified C++ executor route"
    )
    if _POLICY.require_cpp:
        _unavailable("spin", reason)
    try:
        result = (
            _bounded_rclpy_spin(node, executor=executor)
            if optimized else
            _original_spin(node, executor=executor)
        )
    except BaseException as exc:
        metadata = {"outcome": "exception", "exception_type": type(exc).__name__}
        if optimized:
            metadata = _optimized_spin_metadata(
                "exception", exception_type=type(exc).__name__)
        _record_python_operation(
            node,
            "spin",
            reason,
            metadata=metadata,
            once_key=("spin", "exception", type(exc).__name__),
            policies=("stock_executor_authority", "optimized_bounded_wait", "optimized")
            if optimized else None,
        )
        _record_executor_exception(node, "spin", exc)
        raise
    metadata = {"outcome": "returned"}
    if optimized:
        metadata = _optimized_spin_metadata("returned")
    _record_python_operation(
        node,
        "spin",
        reason,
        metadata=metadata,
        once_key=("spin", "returned"),
        policies=("stock_executor_authority", "optimized_bounded_wait", "optimized")
        if optimized else None,
    )
    return result


@wraps(_original_spin_once)
def _spin_once_wrapper(node, *, executor=None, timeout_sec=None):
    reason = "rclpy.spin_once has no certified C++ executor route"
    if _POLICY.require_cpp:
        _unavailable("spin_once", reason)
    try:
        result = _original_spin_once(
            node, executor=executor, timeout_sec=timeout_sec)
    except BaseException as exc:
        _record_python_operation(
            node,
            "spin_once",
            reason,
            metadata={"outcome": "exception", "exception_type": type(exc).__name__},
            once_key=("spin_once", "exception", type(exc).__name__),
        )
        _record_executor_exception(node, "spin_once", exc)
        raise
    _record_python_operation(
        node,
        "spin_once",
        reason,
        metadata={"outcome": "returned"},
        once_key=("spin_once", "returned"),
    )
    return result


@wraps(_original_multi_threaded_spin)
def _multi_threaded_spin_wrapper(self):
    reason = "MultiThreadedExecutor.spin has no certified C++ executor route"
    if _POLICY.require_cpp:
        _unavailable("multi_threaded_spin", reason)
    try:
        result = _original_multi_threaded_spin(self)
    except BaseException as exc:
        _record_runtime_operation(
            self,
            "multi_threaded_spin",
            reason,
            metadata={"outcome": "exception", "exception_type": type(exc).__name__},
            once_key=("multi_threaded_spin", "exception", type(exc).__name__),
            warn=True,
            authority="stock_executor_authority",
        )
        raise
    _record_runtime_operation(
        self,
        "multi_threaded_spin",
        reason,
        metadata={"outcome": "returned"},
        once_key=("multi_threaded_spin", "returned"),
        warn=True,
        authority="stock_executor_authority",
    )
    return result


@wraps(_original_single_threaded_spin)
def _optimized_executor_spin_wrapper(self):
    operation = (
        "multi_threaded_spin"
        if isinstance(self, MultiThreadedExecutor) else
        "single_threaded_spin"
    )
    reason = "optimized profile bounds stock executor waits after signal shutdown"
    executor_type = type(self).__name__
    try:
        result = _bounded_executor_spin(self)
    except BaseException as exc:
        _record_runtime_operation(
            self,
            operation,
            reason,
            metadata=_optimized_spin_metadata(
                "exception",
                executor_type=executor_type,
                exception_type=type(exc).__name__,
            ),
            once_key=(operation, "exception", type(exc).__name__),
            policies=("stock_executor_authority", "optimized_bounded_wait", "optimized"),
        )
        raise
    _record_runtime_operation(
        self,
        operation,
        reason,
        metadata=_optimized_spin_metadata("returned", executor_type=executor_type),
        once_key=(operation, "returned"),
        policies=("stock_executor_authority", "optimized_bounded_wait", "optimized"),
    )
    return result


def _record_future_completion(future, outcome, detail=None):
    metadata = {"outcome": outcome}
    if detail is not None:
        metadata["detail_type"] = type(detail).__name__
    _record_runtime_operation(
        future,
        "future",
        "stock rclpy Future remains authoritative",
        metadata=metadata,
        once_key=("future", outcome),
        authority="stock_future_authority",
    )


@wraps(_original_future_set_result)
def _future_set_result_wrapper(self, result):
    value = _original_future_set_result(self, result)
    _record_future_completion(self, "result", result)
    return value


@wraps(_original_future_set_exception)
def _future_set_exception_wrapper(self, exception):
    value = _original_future_set_exception(self, exception)
    _record_future_completion(self, "exception", exception)
    return value


@wraps(_original_future_cancel)
def _future_cancel_wrapper(self):
    was_canceled = self.cancelled()
    result = _original_future_cancel(self)
    if not was_canceled and self.cancelled():
        _record_future_completion(self, "canceled")
    return result


@wraps(_original_lifecycle_node_init)
def _lifecycle_node_init_wrapper(
    self,
    node_name,
    *,
    enable_communication_interface=True,
    **kwargs,
):
    reason = "lifecycle state machines have no certified C++ ownership route"
    if _POLICY.require_cpp:
        _unavailable("create_lifecycle_node", reason)
    _original_lifecycle_node_init(
        self,
        node_name,
        enable_communication_interface=enable_communication_interface,
        **kwargs,
    )
    _record_python_entity(
        self,
        "lifecycle_node",
        reason,
        metadata={
            "communication_interface": enable_communication_interface,
            "node_class": "%s.%s" % (type(self).__module__, type(self).__qualname__),
        },
    )


def _action_type_name(action_type):
    return "%s.%s" % (action_type.__module__, action_type.__qualname__)


@wraps(_original_action_client_init)
def _action_client_init_wrapper(
    self,
    node,
    action_type,
    action_name,
    *args,
    **kwargs,
):
    reason = "action clients have no certified C++ ownership route"
    if _POLICY.require_cpp:
        _unavailable("create_action_client", reason)
    _original_action_client_init(
        self, node, action_type, action_name, *args, **kwargs)
    _record_python_entity(
        node,
        "action_client",
        reason,
        metadata={
            "action_name": action_name,
            "action_type": _action_type_name(action_type),
        },
        policies=("stock_action_authority", _POLICY.name),
    )


@wraps(_original_action_server_init)
def _action_server_init_wrapper(
    self,
    node,
    action_type,
    action_name,
    *args,
    **kwargs,
):
    reason = "action servers have no certified C++ ownership route"
    if _POLICY.require_cpp:
        _unavailable("create_action_server", reason)
    _original_action_server_init(
        self, node, action_type, action_name, *args, **kwargs)
    _record_python_entity(
        node,
        "action_server",
        reason,
        metadata={
            "action_name": action_name,
            "action_type": _action_type_name(action_type),
        },
        policies=("stock_action_authority", _POLICY.name),
    )


_NODE_PATCHES = (
    ("create_publisher", _create_publisher_wrapper, _original_create_publisher),
    ("create_subscription", _create_subscription_wrapper, _original_create_subscription),
    ("create_timer", _create_timer_wrapper, _original_create_timer),
    ("create_service", _create_service_wrapper, _original_create_service),
    ("create_client", _create_client_wrapper, _original_create_client),
    (
        "create_guard_condition",
        _create_guard_condition_wrapper,
        _original_create_guard_condition,
    ),
    ("set_parameters", _set_parameters_wrapper, _original_set_parameters),
    (
        "set_parameters_atomically",
        _set_parameters_atomically_wrapper,
        _original_set_parameters_atomically,
    ),
)
for _name, _wrapper, _original in _NODE_PATCHES:
    # Keep inspect.signature-compatible call surfaces without changing wrapper
    # names used by startup-hook diagnostics.
    _wrapper.__signature__ = signature(_original)


def patch_ros2(profile="compatible", *, warn_fallback=False):
    """Install idempotent method patches while retaining stock object identity."""
    global _PATCHED, _POLICY
    requested = resolve_policy(profile, warn_fallback=warn_fallback)
    if _PATCHED:
        if requested != _POLICY:
            raise RuntimeError(
                "rclcppyy is already active with profile %r" % _POLICY.name)
        return True

    _POLICY = requested
    for name, wrapper, _original in _NODE_PATCHES:
        setattr(Node, name, wrapper)
    Publisher.publish = (
        _publish_wrapper if requested.use_cpp_publisher else _original_publish)
    rclpy.spin = _spin_wrapper
    rclpy.spin_once = _spin_once_wrapper
    if requested.allow_contract_changes:
        SingleThreadedExecutor.spin = _optimized_executor_spin_wrapper
        MultiThreadedExecutor.spin = _optimized_executor_spin_wrapper
    else:
        MultiThreadedExecutor.spin = _multi_threaded_spin_wrapper
    Future.set_result = _future_set_result_wrapper
    Future.set_exception = _future_set_exception_wrapper
    Future.cancel = _future_cancel_wrapper
    LifecycleNode.__init__ = _lifecycle_node_init_wrapper
    ActionClient.__init__ = _action_client_init_wrapper
    ActionServer.__init__ = _action_server_init_wrapper
    _PATCHED = True
    activation_backend = "cpp" if requested.use_cpp_publisher else "python"
    record_decision(
        "operations",
        activation_backend,
        "installed compatibility routing with explicit backend authority",
        policies=("stock_node_authority", requested.name),
        metadata={
            "operation": "enable_cpp_acceleration",
            "profile": requested.name,
            "publisher_backend": (
                "cpp" if requested.use_cpp_publisher else "python"),
        },
    )
    return True


def patch_node_class():
    """Compatibility alias retained; stock Node identity is intentionally unchanged."""
    return True


__all__ = ["patch_ros2", "patch_node_class"]
