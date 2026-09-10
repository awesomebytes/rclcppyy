"""Hand-built ``Signature`` objects for the bucket-2 pristine-capture mirrors.

Extends the Wave 7 technique already implemented in
``rclcppyy._payload_signature`` (a class captured *pristine* -- before
``direct_messages.install()``/``activate()``'s rebinding runs -- used to
hand-build an ``inspect.Signature`` with live, non-string annotation
objects, then assigned directly to a member's ``__signature__``) to a second
group of stock rclpy members that suffer the identical taint but were out of
that module's original scope.

Every member fixed here belongs to a *stock* class this package never
reimplements or rebinds: ``rclpy.duration.Duration``, ``rclpy.time.Time``,
``rclpy.qos_overriding_options.QoSOverridingOptions``,
``rclpy.parameter_client.AsyncParameterClient``,
``rclpy.parameter_event_handler.ParameterEventHandler``, and the
``rclpy.experimental.events_executor.EventsExecutor`` function. Their own
annotations still get tainted, because each defining module happens to be
first imported -- by anything, at any point in the process -- only *after*
``direct_messages.install()`` (for the two message-type taints) or the
``Node``/``CallbackGroup``/``Subscription``/``Executor`` rebind in
``direct_cpp.activate()`` (for the four facade-type taints) has already run,
so the annotation expression evaluates against the already-rebound name
instead of stock's own.

Every "class" ledger row among these (``QoSOverridingOptions``,
``AsyncParameterClient``, ``ParameterEventHandler``) is deliberately left
without its own ``__signature__``: ``inspect.signature()`` on a class with no
``__signature__`` of its own falls back to its ``__init__``'s signature
(dropping the leading parameter), so fixing ``__init__`` alone flips the
class-symbol ledger row too. See ``rclcppyy._signature_mirror.mirror_class``'s
docstring for why a class-level ``__signature__`` assignment is avoided on
purpose elsewhere in this package (an MRO-leak risk on subclasses outside a
mirror's own scope) -- the same reasoning applies here, and none of these
three classes need it since they define their own ``__init__`` regardless.
"""

from __future__ import annotations

import inspect
from typing import Callable, Iterable, Optional, Sequence, Union


_EMPTY = inspect.Parameter.empty
_POSITIONAL_OR_KEYWORD = inspect.Parameter.POSITIONAL_OR_KEYWORD
_KEYWORD_ONLY = inspect.Parameter.KEYWORD_ONLY


def _parameter(name, *, kind=_POSITIONAL_OR_KEYWORD, default=_EMPTY, annotation=_EMPTY):
    return inspect.Parameter(name, kind, default=default, annotation=annotation)


def _classmethod_func(cls, name):
    """Return the plain function a class's own ``classmethod`` wraps.

    Matches how ``rclcppyy._signature_mirror._callable_target`` (and, per
    its docstring, the ledger extractor itself) resolves a classmethod's
    signature target: the raw, unbound function, not the bound method
    ``cls.name`` would hand back -- the latter has already had its leading
    ``cls`` parameter stripped by the descriptor protocol, which would
    silently drop it from the rendered signature too.
    """
    return cls.__dict__[name].__func__


def install_signatures(
    *,
    duration_msg_class,
    time_msg_class,
    set_parameters_result,
    parameter_msg_class,
    parameter_class,
    node_class,
    callback_group_class,
    subscription_class,
    executor_class,
):
    """Assign every bucket-2 hand-built ``Signature``.

    Every keyword argument is a *pristine* class -- captured by the caller
    before this package's own rebinding of it runs -- never a string, so
    ``str(inspect.signature(...))`` renders each one's real dotted path
    exactly as stock's own baseline does (``formatannotation`` reads
    ``__module__``/``__qualname__`` straight off the object).

    Idempotent: re-assigning the same rendered ``Signature`` is a no-op in
    effect, so calling this more than once in a process is harmless.
    """
    _install_duration_signatures(duration_msg_class)
    _install_time_signatures(time_msg_class)
    _install_qos_overriding_options_signatures(set_parameters_result)
    _install_async_parameter_client_signatures(
        node_class, callback_group_class, subscription_class,
        parameter_class, parameter_msg_class)
    _install_parameter_event_handler_signatures(node_class, callback_group_class)
    _install_events_executor_signature(executor_class)


def _install_duration_signatures(duration_msg_class):
    from rclpy.duration import Duration

    Duration.to_msg.__signature__ = inspect.Signature(
        [_parameter("self")], return_annotation=duration_msg_class)
    _classmethod_func(Duration, "from_msg").__signature__ = inspect.Signature(
        [
            _parameter("cls"),
            _parameter("msg", annotation=duration_msg_class),
        ],
        return_annotation="Duration",
    )


def _install_time_signatures(time_msg_class):
    from rclpy.clock_type import ClockType
    from rclpy.time import Time

    Time.to_msg.__signature__ = inspect.Signature(
        [_parameter("self")], return_annotation=time_msg_class)
    _classmethod_func(Time, "from_msg").__signature__ = inspect.Signature(
        [
            _parameter("cls"),
            _parameter("msg", annotation=time_msg_class),
            _parameter(
                "clock_type", default=ClockType.ROS_TIME, annotation=ClockType),
        ],
        return_annotation="Time",
    )


def _install_qos_overriding_options_signatures(set_parameters_result):
    from rclpy.qos import QoSPolicyKind, QoSProfile
    from rclpy.qos_overriding_options import QoSOverridingOptions

    callback_annotation = Optional[Callable[[QoSProfile], set_parameters_result]]

    QoSOverridingOptions.__init__.__signature__ = inspect.Signature([
        _parameter("self"),
        _parameter("policy_kinds", annotation=Iterable[QoSPolicyKind]),
        _parameter(
            "callback", kind=_KEYWORD_ONLY, default=None,
            annotation=callback_annotation),
        _parameter(
            "entity_id", kind=_KEYWORD_ONLY, default=None,
            annotation=Optional[str]),
    ])
    QoSOverridingOptions.callback.fget.__signature__ = inspect.Signature(
        [_parameter("self")], return_annotation=callback_annotation)
    _classmethod_func(QoSOverridingOptions, "with_default_policies").__signature__ = (
        inspect.Signature(
            [
                _parameter("cls"),
                _parameter(
                    "callback", kind=_KEYWORD_ONLY, default=None,
                    annotation=callback_annotation),
                _parameter(
                    "entity_id", kind=_KEYWORD_ONLY, default=None,
                    annotation=Optional[str]),
            ],
            return_annotation="QoSOverridingOptions",
        )
    )


def _install_async_parameter_client_signatures(
    node_class, callback_group_class, subscription_class,
    parameter_class, parameter_msg_class,
):
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import (
        QoSProfile, qos_profile_parameter_events, qos_profile_services_default)
    from rclpy.qos_overriding_options import QoSOverridingOptions
    from rclpy.task import Future

    callback_group_annotation = Optional[callback_group_class]
    parameters_annotation = Sequence[Union[parameter_class, parameter_msg_class]]

    AsyncParameterClient.__init__.__signature__ = inspect.Signature(
        [
            _parameter("self"),
            _parameter("node", annotation=node_class),
            _parameter("remote_node_name", annotation=str),
            _parameter(
                "qos_profile", default=qos_profile_services_default,
                annotation=QoSProfile),
            _parameter(
                "callback_group", default=None,
                annotation=callback_group_annotation),
        ],
        return_annotation=None,
    )
    AsyncParameterClient.on_parameter_event.__signature__ = inspect.Signature(
        [
            _parameter("self"),
            _parameter("callback", annotation=Callable),
            _parameter(
                "qos_profile", default=qos_profile_parameter_events,
                annotation=QoSProfile),
            _parameter(
                "callback_group", kind=_KEYWORD_ONLY, default=None,
                annotation=callback_group_annotation),
            _parameter(
                "event_callbacks", kind=_KEYWORD_ONLY, default=None,
                annotation=Optional[SubscriptionEventCallbacks]),
            _parameter(
                "qos_overriding_options", kind=_KEYWORD_ONLY, default=None,
                annotation=Optional[QoSOverridingOptions]),
            _parameter("raw", kind=_KEYWORD_ONLY, default=False, annotation=bool),
        ],
        return_annotation=subscription_class,
    )
    for name in ("set_parameters", "set_parameters_atomically"):
        getattr(AsyncParameterClient, name).__signature__ = inspect.Signature(
            [
                _parameter("self"),
                _parameter("parameters", annotation=parameters_annotation),
                _parameter("callback", default=None, annotation=Optional[Callable]),
            ],
            return_annotation=Future,
        )


def _install_parameter_event_handler_signatures(node_class, callback_group_class):
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.parameter_event_handler import ParameterEventHandler
    from rclpy.qos import QoSProfile, qos_profile_parameter_events
    from rclpy.qos_overriding_options import QoSOverridingOptions

    ParameterEventHandler.__init__.__signature__ = inspect.Signature([
        _parameter("self"),
        _parameter("node", annotation=node_class),
        _parameter(
            "qos_profile", default=qos_profile_parameter_events,
            annotation=QoSProfile),
        _parameter(
            "callback_group", default=None,
            annotation=Optional[callback_group_class]),
        _parameter(
            "event_callbacks", default=None,
            annotation=Optional[SubscriptionEventCallbacks]),
        _parameter(
            "qos_overriding_options", default=None,
            annotation=Optional[QoSOverridingOptions]),
        _parameter("raw", default=False, annotation=bool),
    ])


def _install_events_executor_signature(executor_class):
    from rclpy.context import Context

    try:
        import rclpy.experimental.events_executor as events_executor_module
    except ImportError:
        # The experimental C++ EventsExecutor binding is not built into
        # every distribution; this mirror is a no-op where it is absent
        # (there is nothing to fix, and nothing else in this package
        # depends on it existing).
        return
    events_executor_module.EventsExecutor.__signature__ = inspect.Signature(
        [
            _parameter(
                "context", kind=_KEYWORD_ONLY, default=None,
                annotation=Optional[Context]),
        ],
        return_annotation=executor_class,
    )


__all__ = ["install_signatures"]
