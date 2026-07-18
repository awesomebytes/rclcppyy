"""Transparent acceleration patches over authoritative stock rclpy objects."""

from __future__ import annotations

import warnings

from rclpy.node import Node
from rclpy.publisher import Publisher

from rclcppyy._status import record_decision
from rclcppyy.policy import BackendUnavailableError, resolve_policy


_original_create_publisher = Node.create_publisher
_original_create_subscription = Node.create_subscription
_original_create_timer = Node.create_timer
_original_publish = Publisher.publish

_PATCHED = False
_POLICY = resolve_policy()
_WARNED_FALLBACKS = set()


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


def _record_python_entity(node, entity_type, reason, metadata=None):
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
        policies=("stock_fallback", _POLICY.name),
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
            metadata={"topic": publisher.topic_name},
        )
        return publisher

    publisher._rclcppyy_publish_route = route
    publisher._rclcppyy_policy = _POLICY
    publisher._rclcppyy_reported_fallbacks = set()
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
        return _original_publish(self, message)
    try:
        return route.publish(self, message)
    except TypeError:
        # Preserve the stock exception contract for invalid message objects.
        return _original_publish(self, message)
    except Exception as exc:
        reason = "same-handle C++ publish failed: %s" % exc
        if policy.require_cpp:
            _unavailable("publish", reason)
        _record_publish_fallback_once(self, reason)
        return _original_publish(self, message)


def _create_subscription_wrapper(self, *args, **kwargs):
    reason = "subscription take/dispatch has no certified same-handle C++ route"
    if _POLICY.require_cpp:
        _unavailable("create_subscription", reason)
    subscription = _original_create_subscription(self, *args, **kwargs)
    topic = args[1] if len(args) > 1 else kwargs.get("topic")
    _record_python_entity(self, "subscription", reason, metadata={"topic": topic})
    return subscription


def _create_timer_wrapper(self, *args, **kwargs):
    reason = "timer/executor integration has no certified same-handle C++ route"
    if _POLICY.require_cpp:
        _unavailable("create_timer", reason)
    timer = _original_create_timer(self, *args, **kwargs)
    period = args[0] if args else kwargs.get("timer_period_sec")
    _record_python_entity(self, "timer", reason, metadata={"period_sec": period})
    return timer


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
    Node.create_publisher = _create_publisher_wrapper
    Node.create_subscription = _create_subscription_wrapper
    Node.create_timer = _create_timer_wrapper
    Publisher.publish = _publish_wrapper
    _PATCHED = True
    record_decision(
        "operations",
        "cpp",
        "installed stock-authority compatibility routing",
        policies=("stock_node_authority", requested.name),
        metadata={
            "operation": "enable_cpp_acceleration",
            "profile": requested.name,
        },
    )
    return True


def patch_node_class():
    """Compatibility alias retained; stock Node identity is intentionally unchanged."""
    return True


__all__ = ["patch_ros2", "patch_node_class"]
