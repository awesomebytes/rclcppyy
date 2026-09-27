#!/usr/bin/env python3
"""End-to-end lifecycle data-plane proof (PLAN-lifecycle.md P3).

A lifecycle node creates a subscription, timer, service, and a declared
parameter through its INHERITED (non-lifecycle-specific)
create_subscription/create_timer/create_service methods -- proving the
private ``_native_create_*`` seams (PLAN-lifecycle.md §2.3.1) route each to
the lifecycle-node-typed suite helper while the public method signatures
stay byte-identical to ``DirectNode``'s. These are plain (unmanaged)
entities and operate immediately, unaffected by the node's lifecycle state.

The inherited ``create_publisher()`` is deferred fail-closed instead
(disclosed divergence): stock's inherited create_publisher is UNGATED, but
``rclcpp_lifecycle::LifecycleNode`` has only one (inherently managed/gated)
``create_publisher<>()``, and this product's transition-callback bridge
(P1) shadows the native default handler that would otherwise auto-activate
it -- reusing it here would silently create a publisher that never
delivers. ``create_lifecycle_publisher()`` (P2) remains the supported,
already-gated path and is exercised here to complete the end-to-end story.

Also proves the fail-closed rejections for actions on a lifecycle node
(deferred, PLAN-lifecycle.md §3.7).
"""
import os
import threading
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402

from rclcppyy.policy import BackendUnavailableError  # noqa: E402


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

    node = LifecycleNode("direct_lifecycle_dataplane_%d" % pid)
    out_topic = "direct_lifecycle_dataplane_out_%d" % pid

    # --- fail-closed: actions are deferred on a lifecycle node ---
    try:
        ActionServer(node, LookupTransform, "dataplane_action_%d" % pid)
    except BackendUnavailableError as exc:
        assert "lifecycle node" in str(exc)
    else:
        raise AssertionError("expected ActionServer on a lifecycle node to fail closed")
    try:
        ActionClient(node, LookupTransform, "dataplane_action_%d" % pid)
    except BackendUnavailableError as exc:
        assert "lifecycle node" in str(exc)
    else:
        raise AssertionError("expected ActionClient on a lifecycle node to fail closed")
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_ACTIONS_REJECTED_OK", flush=True)

    # --- fail-closed: the inherited create_publisher() is deferred too ---
    try:
        node.create_publisher(String, out_topic, 10)
    except BackendUnavailableError as exc:
        assert "create_lifecycle_publisher" in str(exc)
    else:
        raise AssertionError(
            "expected the inherited create_publisher() on a lifecycle node "
            "to fail closed")
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_PUBLISHER_REJECTED_OK", flush=True)

    # --- the supported, already-gated path (P2) completes the story ---
    pub = node.create_lifecycle_publisher(String, out_topic, 10)

    in_topic = "direct_lifecycle_dataplane_in_%d" % pid
    service_name = "direct_lifecycle_dataplane_trigger_%d" % pid

    # --- entity creation via the INHERITED (non-lifecycle-specific) methods ---
    received = []
    node.create_subscription(
        String, in_topic, lambda msg: received.append(str(msg.data)), 10)
    timer_calls = []
    node.create_timer(0.05, lambda: timer_calls.append(1))

    def _serve(request, response):
        response.success = bool(request.data)
        response.message = "lifecycle-dataplane-response"
        return response

    node.create_service(SetBool, service_name, _serve)
    node.declare_parameter("dataplane_param", 42)
    assert node.get_parameter("dataplane_param").value == 42
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_CONSTRUCT_OK", flush=True)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_spin = threading.Event()

    def _spin_node():
        while not stop_spin.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_node, name="lifecycle-dataplane-spin")
    spin_thread.start()

    peer = rclpy.create_node("direct_lifecycle_dataplane_peer_%d" % pid)
    peer_pub = peer.create_publisher(String, in_topic, 10)
    out_received = []
    peer.create_subscription(
        String, out_topic, lambda msg: out_received.append(str(msg.data)), 10)
    peer_client = peer.create_client(SetBool, service_name)
    peer_executor = SingleThreadedExecutor()
    peer_executor.add_node(peer)
    stop_peer_spin = threading.Event()

    def _spin_peer():
        while not stop_peer_spin.is_set():
            peer_executor.spin_once(timeout_sec=0.05)

    peer_spin_thread = threading.Thread(
        target=_spin_peer, name="lifecycle-dataplane-peer-spin")
    peer_spin_thread.start()

    # Best-effort discovery settle; the _wait_for polling below is the real
    # synchronization, not this sleep.
    time.sleep(0.5)

    # --- subscription: a plain rclcpp::Subscription, unmanaged -- works
    # before any transition ---
    peer_pub.publish(String(data="pre-activate"))
    assert _wait_for(lambda: len(received) > 0, timeout_s=5.0)
    assert received[-1] == "pre-activate"
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_SUBSCRIPTION_OK", flush=True)

    # --- timer: a plain rclcpp::TimerBase, unmanaged -- fires before any
    # transition ---
    assert _wait_for(lambda: len(timer_calls) > 0, timeout_s=5.0)
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_TIMER_OK", flush=True)

    # --- service: a plain rclcpp::Service, unmanaged -- responds before any
    # transition ---
    assert peer_client.wait_for_service(timeout_sec=5.0)
    future = peer_client.call_async(SetBool.Request(data=True))
    assert _wait_for(lambda: future.done(), timeout_s=5.0)
    assert future.exception() is None
    response = future.result()
    assert response.success is True
    assert str(response.message) == "lifecycle-dataplane-response"
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_SERVICE_OK", flush=True)

    # --- publisher (create_lifecycle_publisher, P2): natively gated --
    # suppressed until active ---
    pub.publish(String(data="while-unconfigured"))
    assert not _wait_for(lambda: len(out_received) > 0, timeout_s=1.5)

    result = node.trigger_configure()
    assert result == TransitionCallbackReturn.SUCCESS
    pub.publish(String(data="while-inactive"))
    assert not _wait_for(lambda: len(out_received) > 0, timeout_s=1.5)

    result = node.trigger_activate()
    assert result == TransitionCallbackReturn.SUCCESS
    pub.publish(String(data="while-active"))
    assert _wait_for(lambda: len(out_received) > 0, timeout_s=5.0)
    assert out_received[-1] == "while-active"
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_PUBLISHER_GATED_OK", flush=True)

    # --- post-activation: subscription/timer/service continue to operate,
    # unaffected by the transition ---
    before_sub = len(received)
    peer_pub.publish(String(data="post-activate"))
    assert _wait_for(lambda: len(received) > before_sub, timeout_s=5.0)
    assert received[-1] == "post-activate"

    before_timer = len(timer_calls)
    assert _wait_for(lambda: len(timer_calls) > before_timer, timeout_s=5.0)

    future2 = peer_client.call_async(SetBool.Request(data=True))
    assert _wait_for(lambda: future2.done(), timeout_s=5.0)
    assert future2.result().success is True
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_POST_ACTIVATE_OK", flush=True)

    stop_peer_spin.set()
    peer_spin_thread.join(timeout=15.0)
    assert not peer_spin_thread.is_alive(), "peer spin thread hung after teardown"
    peer_executor.remove_node(peer)
    peer.destroy_node()

    stop_spin.set()
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "node spin thread hung after teardown"
    executor.remove_node(node)
    node.destroy_node()

    rclpy.shutdown()
    assert not rclpy.ok()
    print("DIRECT_CPP_LIFECYCLE_DATAPLANE_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
