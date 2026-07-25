"""Direct-C++ facade for ``rclpy.lifecycle`` -- a real ``rclcpp_lifecycle::LifecycleNode``.

Imported only from ``rclcppyy.direct_cpp.activate()``, after ``rclpy.node.Node``
has already been rebound to ``DirectNode`` (``LifecycleNode(LifecycleNodeMixin,
Node)`` resolves its base class at import time).

Transition dispatch, the state machine, and the five lifecycle services are
native (``rclcpp_kit.native_lifecycle``, proven end-to-end against a stock
rclpy client and an AOT C++ peer); this module registers contained Python
bridges for the six transition callbacks and forwards ``trigger_*``/state
accessors to the native node. The managed ``LifecyclePublisher`` (P2) wraps
a native ``rclcpp_lifecycle::LifecyclePublisher`` -- gating is native, not
the Python ``SimpleManagedEntity.when_enabled`` decorator. The remaining
data-plane entities (subscription/timer/service/client) on a lifecycle node
are a later slice (P3).
"""

from __future__ import annotations

import functools

from lifecycle_msgs.msg import Transition as _Transition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.lifecycle.managed_entity import ManagedEntity, SimpleManagedEntity
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from rclcppyy._status import record_decision
from rclcppyy.direct_cpp import (
    DirectNode,
    DirectPublisher,
    _direct_node_options,
    _lower_entity_qos,
    _runtime,
    _unsupported,
)


_TRANSITION_KINDS = (
    "configure", "cleanup", "shutdown", "activate", "deactivate", "error")

_NODE_KWARG_DEFAULTS = {
    "context": None,
    "cli_args": None,
    "namespace": None,
    "use_global_arguments": True,
    "enable_rosout": True,
    "start_parameter_services": True,
    "parameter_overrides": None,
    "allow_undeclared_parameters": False,
    "automatically_declare_parameters_from_overrides": False,
    "enable_logger_service": False,
}


def _transition_bridge(contained, state_id, state_label):
    """Adapt the suite's ``(state_id, state_label) -> int`` callback shape to
    the contained callable's ``(LifecycleState) -> int`` shape."""
    return contained(LifecycleState(label=state_label, state_id=state_id))


class DirectLifecycleNodeMixin(ManagedEntity):
    """rclcppyy facade mixin backed by a real ``rclcpp_lifecycle::LifecycleNode``.

    Mirrors ``rclpy.lifecycle.LifecycleNodeMixin``'s public surface. Requires
    ``self._direct_cpp_lifecycle_resource`` (a
    ``rclcpp_kit.native_lifecycle.NativeLifecycleNode``) to already be set --
    ``DirectLifecycleNode.__init__`` does so before calling this.
    """

    def __init__(
        self,
        *,
        enable_communication_interface: bool = True,
        callback_group=None,
    ):
        if callback_group is not None:
            _unsupported(
                "direct_cpp lifecycle nodes do not support a callback_group: "
                "the five native services are not Python Service objects")
        self._managed_entities = set()
        self._direct_cpp_lifecycle_bridges = tuple(
            self._direct_cpp_lifecycle_resource.register_transition_callback(
                kind,
                functools.partial(
                    _transition_bridge,
                    self._contain_transition_callback(
                        functools.partial(self._dispatch_transition, kind)),
                ),
            )
            for kind in _TRANSITION_KINDS
        )

    def _dispatch_transition(self, kind, state):
        """Late-bound bridge target: a user override of ``on_<kind>`` wins."""
        return getattr(self, "on_" + kind)(state)

    def _transition_callback_impl(self, callback_name, state):
        """Mirrors stock's ``LifecycleNodeMixin.__transition_callback_impl``
        (node.py:179-190): dispatch to every managed entity, giving up at the
        first non-SUCCESS result."""
        for entity in self._managed_entities:
            callback = getattr(entity, callback_name)
            result = callback(state)
            if not isinstance(result, TransitionCallbackReturn):
                raise TypeError(
                    f'{callback_name}() return value of class {type(entity)} '
                    'should be `TransitionCallbackReturn`.\n'
                    f'Instance of the class that caused the failure: {entity}')
            if result != TransitionCallbackReturn.SUCCESS:
                return result
        return TransitionCallbackReturn.SUCCESS

    def on_configure(self, state) -> TransitionCallbackReturn:
        return self._transition_callback_impl('on_configure', state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return self._transition_callback_impl('on_cleanup', state)

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return self._transition_callback_impl('on_shutdown', state)

    def on_activate(self, state) -> TransitionCallbackReturn:
        return self._transition_callback_impl('on_activate', state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        return self._transition_callback_impl('on_deactivate', state)

    def on_error(self, state) -> TransitionCallbackReturn:
        return self._transition_callback_impl('on_error', state)

    def _trigger(self, transition_id):
        """Validate ``transition_id`` is registered from the current state,
        then trigger it. rclcpp_lifecycle's own ``trigger_transition``
        silently no-ops (leaving the CallbackReturn out-param untouched) on
        an unregistered id instead of raising -- unlike stock's
        ``_rclpy.LifecycleStateMachine`` path, which raises ``RCLError``
        (verified live). This pre-check restores that observable stock
        behavior with the same exception type."""
        resource = self._direct_cpp_lifecycle_resource
        if not any(
            item[0] == transition_id for item in resource.available_transitions
        ):
            raise _rclpy.RCLError(
                "Failed to trigger lifecycle state machine transition: "
                "Transition is not registered.")
        return TransitionCallbackReturn(
            resource.trigger_transition_by_id(transition_id))

    def trigger_configure(self):
        return self._trigger(_Transition.TRANSITION_CONFIGURE)

    def trigger_cleanup(self):
        return self._trigger(_Transition.TRANSITION_CLEANUP)

    def trigger_activate(self):
        return self._trigger(_Transition.TRANSITION_ACTIVATE)

    def trigger_deactivate(self):
        return self._trigger(_Transition.TRANSITION_DEACTIVATE)

    def trigger_shutdown(self):
        # Mirrors stock's state-dependent id selection (node.py:156-166)
        # exactly, including the raised type/message for an impossible
        # shutdown -- a pure Python pre-check in stock too, not a native
        # error.
        current_label = self._direct_cpp_lifecycle_resource.current_state[1]
        if current_label == 'unconfigured':
            transition_id = _Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
        elif current_label == 'inactive':
            transition_id = _Transition.TRANSITION_INACTIVE_SHUTDOWN
        elif current_label == 'active':
            transition_id = _Transition.TRANSITION_ACTIVE_SHUTDOWN
        else:
            raise _rclpy.RCLError('Shutdown transition not possible')
        return self._trigger(transition_id)

    def add_managed_entity(self, entity: ManagedEntity):
        if not isinstance(entity, ManagedEntity):
            raise TypeError('Expected a rclpy.lifecycle.ManagedEntity instance.')
        self._managed_entities.add(entity)

    def create_lifecycle_publisher(
        self, msg_type, topic, qos_profile, *,
        callback_group=None, event_callbacks=None, qos_overriding_options=None,
        publisher_class=None,
    ):
        if publisher_class is not None:
            # Stock's own message text, kept verbatim for differential
            # fidelity: LifecycleNodeMixin.create_lifecycle_publisher
            # (node.py:276-288) forwards straight to Node.create_publisher,
            # so the observed message says "create_publisher()", not
            # "create_lifecycle_publisher()" -- a quirk of stock, not a typo
            # here.
            raise TypeError(
                "create_publisher() got an unexpected keyword argument "
                "'publisher_class'")
        self._reject_entity_options(
            "lifecycle_publisher",
            {
                "event_callbacks": event_callbacks is not None,
                "qos_overriding_options": qos_overriding_options is not None,
            },
        )
        from rclcpp_kit import native_lifecycle

        qos, normalized_qos = _lower_entity_qos(qos_profile)
        group, native_group = self._resolve_callback_group(callback_group)
        native = native_lifecycle.create_lifecycle_publisher(
            self._direct_cpp_lifecycle_resource.raw_node,
            msg_type, str(topic), qos, callback_group=native_group,
        )
        publisher = DirectLifecyclePublisher(
            msg_type, topic, normalized_qos, self._logger_name(), native, group)
        group.add_entity(publisher)
        self._direct_cpp_publishers.append(publisher)
        self._managed_entities.add(publisher)
        self._record_entity("publisher", topic, msg_type)
        return publisher

    def destroy_lifecycle_publisher(self, publisher):
        self._managed_entities.discard(publisher)
        return self.destroy_publisher(publisher)

    @property
    def _current_state(self) -> LifecycleState:
        state_id, label = self._direct_cpp_lifecycle_resource.current_state
        return LifecycleState(label=label, state_id=state_id)

    @property
    def _available_states(self) -> list[LifecycleState]:
        return [
            LifecycleState(label=label, state_id=state_id)
            for state_id, label
            in self._direct_cpp_lifecycle_resource.available_states
        ]

    @property
    def _available_transitions(self):
        return list(self._direct_cpp_lifecycle_resource.available_transitions)

    @property
    def _transition_graph(self):
        return list(self._direct_cpp_lifecycle_resource.transition_graph)

    def _get_transition_by_label(self, label: str) -> int:
        return self._direct_cpp_lifecycle_resource.get_transition_by_label(label)

    @property
    def _initialized(self) -> bool:
        return self._direct_cpp_lifecycle_resource.initialized


class DirectLifecyclePublisher(SimpleManagedEntity, DirectPublisher):
    """A managed publisher wrapping a real ``rclcpp_lifecycle::LifecyclePublisher``.

    ``publish`` (bound by ``DirectPublisher.__init__`` straight to the
    native method) is gated natively: ``LifecyclePublisher::publish()``
    itself drops the message while the node is unconfigured/inactive
    (``rclcpp_kit.native_lifecycle``), so nothing here re-implements
    ``SimpleManagedEntity.when_enabled``'s Python gate. This is *stricter*
    than stock's Python-decorator gating -- a disclosed divergence, not a
    behavior change: an inactive publish is still suppressed either way.
    ``is_activated``/``on_activate``/``on_deactivate`` are direct native
    passthroughs.
    """

    def __init__(
        self, msg_type, topic, qos_profile, logger_name, native, callback_group
    ):
        SimpleManagedEntity.__init__(self)
        DirectPublisher.__init__(
            self, msg_type, topic, qos_profile, logger_name, native, callback_group)

    @property
    def is_activated(self):
        return bool(self._require_native().is_activated())

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._require_native().on_activate()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._require_native().on_deactivate()
        return TransitionCallbackReturn.SUCCESS


class DirectLifecycleNode(DirectLifecycleNodeMixin, DirectNode):
    """A ROS 2 managed node backed by a real ``rclcpp_lifecycle::LifecycleNode``."""

    def __init__(
        self, node_name, *, enable_communication_interface: bool = True, **kwargs
    ):
        unknown = sorted(set(kwargs) - set(_NODE_KWARG_DEFAULTS))
        if unknown:
            raise TypeError(
                "DirectLifecycleNode() got unexpected keyword argument(s): %s" %
                ", ".join(unknown))
        resolved = dict(_NODE_KWARG_DEFAULTS)
        resolved.update(kwargs)
        context = resolved.pop("context")
        namespace = resolved.pop("namespace")
        allow_undeclared_parameters = resolved["allow_undeclared_parameters"]

        runtime = _runtime()
        if context is not None and context is not runtime.context:
            _unsupported("direct_cpp node requires its active runtime context")
        session = runtime.require_session()
        options, override_count = _direct_node_options(
            session,
            cli_args=resolved["cli_args"],
            use_global_arguments=resolved["use_global_arguments"],
            enable_rosout=resolved["enable_rosout"],
            start_parameter_services=resolved["start_parameter_services"],
            parameter_overrides=resolved["parameter_overrides"],
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=(
                resolved["automatically_declare_parameters_from_overrides"]),
            enable_logger_service=resolved["enable_logger_service"],
        )
        resource = session.create_native_lifecycle_node(
            str(node_name),
            namespace=str(namespace or ""),
            options=options,
            enable_communication_interface=enable_communication_interface,
        )
        self._direct_cpp_lifecycle_resource = resource
        self._init_common(
            resource.raw_node, node_name, namespace,
            allow_undeclared_parameters=allow_undeclared_parameters,
            enable_parameter_cache=False)
        DirectLifecycleNodeMixin.__init__(
            self, enable_communication_interface=enable_communication_interface)
        record_decision(
            "nodes",
            "cpp",
            "direct_cpp owns one NativeSession rclcpp_lifecycle node",
            policies=("direct_cpp", "native_lifecycle_node_authority"),
            metadata={
                "name": str(node_name),
                "namespace": str(namespace or ""),
                "context": "direct_runtime",
                "enable_communication_interface": enable_communication_interface,
                "cli_arguments": (
                    0 if resolved["cli_args"] is None else len(resolved["cli_args"])),
                "parameter_overrides": override_count,
                "allow_undeclared_parameters": allow_undeclared_parameters,
            },
        )

    def _native_executor_add(self, executor_native) -> None:
        self._direct_cpp_lifecycle_resource.attach_executor(executor_native)

    def _native_executor_remove(self, executor_native) -> None:
        self._direct_cpp_lifecycle_resource.detach_executor(executor_native)

    def _native_release(self, node) -> None:
        del node
        self._direct_cpp_lifecycle_resource.close()


__all__ = [
    "DirectLifecycleNode",
    "DirectLifecycleNodeMixin",
    "DirectLifecyclePublisher",
]
