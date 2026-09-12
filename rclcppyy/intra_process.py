"""Opt-in facade over rclcpp's own native intra-process communication.

Design note (read before "reimplementing" anything here): rclcpp already
ships a complete intra-process path --
``rclcpp::experimental::IntraProcessManager`` plus the wiring built into
``rclcpp::Publisher<MessageT>``/``rclcpp::Subscription<MessageT>``
themselves (see rclcpp/publisher.hpp, rclcpp/subscription.hpp,
rclcpp/detail/resolve_use_intra_process.hpp). It already does everything a
from-scratch manager would need to reinvent:

- topic+QoS compatibility matching between local publishers/subscriptions
  (``IntraProcessManager::can_communicate``);
- an owning copy per subscriber, with the *last* recipient getting the
  original allocation moved rather than copied
  (``IntraProcessManager::add_owned_msg_to_buffers``);
- GID-based dedup so a subscriber with both a local and an external route
  to the same publisher never receives one message twice
  (``SubscriptionBase::matches_any_intra_process_publishers``);
- mixed local+external delivery: a publisher with both local and remote
  subscribers still publishes over DDS too
  (``Publisher<T>::publish``'s ``get_subscription_count() >
  get_intra_process_subscription_count() || buffer_`` check);
- TRANSIENT_LOCAL replay for late-joining local subscribers
  (``IntraProcessManager::do_transient_local_publish``).

rclcpp_kit's ``ManagedPublisher``/``ManagedSubscription`` (see
rclcpp_kit.direct_entities) already wrap genuine
``rclcpp::Publisher<MessageT>``/``rclcpp::Subscription<MessageT>``
entities -- not a custom transport -- so all of the above is already
reachable. The only missing piece is turning it on: intra-process is
selected per entity by ``PublisherOptions``/``SubscriptionOptions``
.use_intra_process_comm (default: defer to the owning node), and the node
default is set once, at construction, via
``NodeOptions::use_intra_process_comms(bool)``. It cannot be toggled on an
already-constructed node -- the flag is baked into ``NodeBase`` at
creation.

This module intentionally does NOT add a new keyword argument to
``DirectNode.__init__``/``create_node``: rclcppyy mirrors rclpy's public
surface under a fail-closed signature/superset guard, and a stock rclpy
``Node`` has no ``enable_intra_process`` parameter. Adding one there would
either trip that guard or require registering a new manifest/ledger
surface -- out of scope for this spike. Instead, this is a process-global,
pre-node-creation opt-in switch (default off), consulted by
``direct_cpp._direct_node_options()`` when it builds each node's
``NodeOptions``. Call ``set_default_enabled(True)`` (or set the
``RCLCPPYY_DIRECT_ENABLE_INTRA_PROCESS`` environment variable) before
creating the nodes that should use it.
"""

from __future__ import annotations

import os

_ENV_VAR = "RCLCPPYY_DIRECT_ENABLE_INTRA_PROCESS"
_override: bool | None = None


def _env_default() -> bool:
    raw = os.environ.get(_ENV_VAR, "0")
    return raw.strip().lower() not in ("", "0", "false", "no", "off")


def default_enabled() -> bool:
    """Whether a direct_cpp node created right now would enable intra-process
    communication by default (env var, unless overridden via
    ``set_default_enabled``)."""
    return _env_default() if _override is None else _override


def set_default_enabled(enabled: bool | None) -> None:
    """Process-global opt-in switch (see module docstring).

    ``True``/``False`` pins the default regardless of the environment
    variable; ``None`` reverts to reading the environment variable again.
    Only nodes created *after* this call are affected -- rclcpp bakes
    ``use_intra_process_comms`` into a node's ``NodeOptions`` at
    construction and never re-reads it afterward.
    """
    global _override
    if enabled is not None and not isinstance(enabled, bool):
        raise TypeError("enabled must be a bool or None")
    _override = enabled


def _require_native_node(node):
    native_node = getattr(node, "_direct_cpp_node", None)
    if native_node is None:
        raise TypeError("node is not a direct_cpp DirectNode")
    return native_node


def node_uses_intra_process(node) -> bool:
    """Introspection: does this direct_cpp node's underlying ``rclcpp::Node``
    have intra-process communication enabled?"""
    native_node = _require_native_node(node)
    return bool(native_node.get_node_options().use_intra_process_comms())


def _require_native_entity(entity_owner, attr_chain):
    target = entity_owner
    for attr in attr_chain:
        target = getattr(target, attr)
    return target


def publisher_intra_process_subscription_count(publisher) -> int:
    """How many local (same-process) subscriptions a direct_cpp publisher's
    native ``rclcpp::Publisher`` is currently matched with. Zero whenever
    the owning node was created without intra-process enabled."""
    entity = publisher.native_entity
    return int(entity.get_intra_process_subscription_count())


def publisher_external_subscription_count(publisher) -> int:
    """External (DDS-visible) subscriber count -- total minus intra-process,
    matching how ``rclcpp::Publisher<T>::publish`` itself decides whether to
    also take the network path."""
    entity = publisher.native_entity
    total = int(entity.get_subscription_count())
    local = int(entity.get_intra_process_subscription_count())
    return max(total - local, 0)
