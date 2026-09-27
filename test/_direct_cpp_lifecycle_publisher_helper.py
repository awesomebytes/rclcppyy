#!/usr/bin/env python3
"""Managed LifecyclePublisher + native gating proof (PLAN-lifecycle.md P2).

Gated publish (unconfigured/inactive suppressed, active delivered)
observed on a stock rclpy subscriber; ``is_activated`` correctness at each
stage; a user ``SimpleManagedEntity`` added via ``add_managed_entity``
toggles across the same transition as the native managed publisher;
``publisher_class`` rejection matches stock's exact message (differential).
"""
import os
import threading
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.lifecycle import (  # noqa: E402
    LifecycleNode,
    ManagedEntity,
    SimpleManagedEntity,
    TransitionCallbackReturn,
)
from std_msgs.msg import String  # noqa: E402

from rclcppyy.direct_lifecycle import DirectLifecyclePublisher  # noqa: E402


def _wait_for(predicate, timeout_s=5.0, poll_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return predicate()


def main():
    rclpy.init(args=None)
    pid = os.getpid()

    node = LifecycleNode("direct_lifecycle_pub_%d" % pid)
    topic = "direct_lifecycle_pub_topic_%d" % pid
    pub = node.create_lifecycle_publisher(String, topic, 10)
    assert type(pub) is DirectLifecyclePublisher
    assert isinstance(pub, SimpleManagedEntity)
    assert isinstance(pub, ManagedEntity)
    assert pub in node._managed_entities
    assert pub.is_activated is False
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_CONSTRUCT_OK", flush=True)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_spin = threading.Event()

    def _spin_node():
        while not stop_spin.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_node, name="lifecycle-pub-spin")
    spin_thread.start()

    sub_node = rclpy.create_node("direct_lifecycle_pub_sub_%d" % pid)
    received = []
    sub_node.create_subscription(
        String, topic, lambda msg: received.append(msg.data), 10)
    sub_executor = SingleThreadedExecutor()
    sub_executor.add_node(sub_node)
    stop_sub_spin = threading.Event()

    def _spin_sub():
        while not stop_sub_spin.is_set():
            sub_executor.spin_once(timeout_sec=0.05)

    sub_spin_thread = threading.Thread(target=_spin_sub, name="lifecycle-pub-sub-spin")
    sub_spin_thread.start()

    # Best-effort discovery settle; the _wait_for polling below is the real
    # synchronization, not this sleep.
    time.sleep(0.5)

    # --- unconfigured: native gate closed, suppressed ---
    pub.publish(String(data="while-unconfigured"))
    assert not _wait_for(lambda: len(received) > 0, timeout_s=1.5)
    assert pub.is_activated is False
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_UNCONFIGURED_SUPPRESSED_OK", flush=True)

    # --- inactive: still gated, suppressed ---
    result = node.trigger_configure()
    assert result == TransitionCallbackReturn.SUCCESS
    assert pub.is_activated is False
    pub.publish(String(data="while-inactive"))
    assert not _wait_for(lambda: len(received) > 0, timeout_s=1.5)
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_INACTIVE_SUPPRESSED_OK", flush=True)

    # --- active: the node's own create_publisher<>() auto-registered this
    # publisher as a managed entity in C++, so is_activated flips without
    # any Python-side driving --- delivered to a stock subscriber ---
    result = node.trigger_activate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert pub.is_activated is True
    pub.publish(String(data="while-active"))
    assert _wait_for(lambda: len(received) > 0, timeout_s=5.0)
    assert received[-1] == "while-active"
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_ACTIVE_DELIVERED_OK", flush=True)

    # --- deactivate: native gate closes again ---
    result = node.trigger_deactivate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert pub.is_activated is False
    before = len(received)
    pub.publish(String(data="while-inactive-again"))
    assert not _wait_for(lambda: len(received) > before, timeout_s=1.5)
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_DEACTIVATE_SUPPRESSED_OK", flush=True)

    # --- user SimpleManagedEntity coexistence across the same transition:
    # the native publisher is gated in C++, the user entity in Python via
    # the mixin's default on_activate/on_deactivate -- both must toggle
    # together, driven by the same trigger_* call ---
    class RecordingManagedEntity(SimpleManagedEntity):
        pass

    user_entity = RecordingManagedEntity()
    node.add_managed_entity(user_entity)
    assert user_entity.is_activated is False

    result = node.trigger_activate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert user_entity.is_activated is True
    assert pub.is_activated is True
    before = len(received)
    pub.publish(String(data="coexistence-active"))
    assert _wait_for(lambda: len(received) > before, timeout_s=5.0)
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_COEXISTENCE_ACTIVATE_OK", flush=True)

    result = node.trigger_deactivate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert user_entity.is_activated is False
    assert pub.is_activated is False
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_COEXISTENCE_DEACTIVATE_OK", flush=True)

    # --- publisher_class rejection: exact stock message text (differential).
    # Stock's LifecycleNodeMixin.create_lifecycle_publisher (node.py:276-288)
    # forwards straight to Node.create_publisher, so the observed message
    # says "create_publisher()", not "create_lifecycle_publisher()" -- kept
    # verbatim, not a typo here.
    try:
        node.create_lifecycle_publisher(
            String, topic + "_reject", 10, publisher_class=object)
    except TypeError as exc:
        assert str(exc) == (
            "create_publisher() got an unexpected keyword argument "
            "'publisher_class'"), str(exc)
    else:
        raise AssertionError(
            "expected create_lifecycle_publisher(publisher_class=...) to raise")
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_CLASS_REJECTED_OK", flush=True)

    # --- destroy_lifecycle_publisher: removed from managed entities ---
    assert node.destroy_lifecycle_publisher(pub) is True
    assert pub not in node._managed_entities
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_DESTROY_OK", flush=True)

    stop_sub_spin.set()
    sub_spin_thread.join(timeout=15.0)
    assert not sub_spin_thread.is_alive(), "subscriber spin thread hung after teardown"
    sub_executor.remove_node(sub_node)
    sub_node.destroy_node()

    stop_spin.set()
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "node spin thread hung after teardown"
    executor.remove_node(node)
    node.destroy_node()

    rclpy.shutdown()
    assert not rclpy.ok()
    print("DIRECT_CPP_LIFECYCLE_PUBLISHER_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
