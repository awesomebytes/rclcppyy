#!/usr/bin/env python3
"""Proves QoS event-callback exception containment under a real, publicly
constructed ``MultiThreadedExecutor`` (PLAN-qos-events-product.md S3/S6#1).

Mirrors ``_direct_cpp_multi_threaded_executor_exception_crash_helper.py``
(the subscription-callback containment proof for defect A,
PLAN-mte-unlock.md) exactly in spirit, but for a QoS event callback instead
of a message callback: event callbacks dispatch on the same native MTE
worker and, before this slice, did NOT flow through
``_contain_callback_exceptions`` -- wired naively they would be the same
terminate-on-raise, counter-blind class the message-callback fix already
closed. This is the load-bearing proof that Slice 1's wiring closes that gap
for event callbacks specifically:

  1. the process exits cleanly (no ``std::terminate``/abort),
  2. the exact exception surfaces on the spin thread (type and message),
  3. a non-raising peer event callback dispatched normally on the same
     executor beforehand -- containing one event callback's exception does
     not retroactively poison an already-completed dispatch.

Uses the ``matched`` publisher event (fires once, immediately, the moment a
compatible subscription is discovered) for both the peer and the raising
callback -- no QoS-deadline/liveliness timing recipe needed, only genuine DDS
discovery. The two are deliberately sequenced (peer proven first, then the
raising one introduced) rather than started concurrently: a contained
exception, once drained, ends that ``spin()`` invocation entirely (matching
stock's own re-raise-and-stop semantics), so if both were racing from the
start the raise could in principle end the spin loop before the peer's own
dispatch ever got a turn.
"""
import os
import threading

os.environ.setdefault("ROS_DOMAIN_ID", "77")

import rclcppyy  # noqa: E402

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.event_handler import PublisherEventCallbacks  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init(args=[])
node = Node("qos_event_mte_raise_%d" % os.getpid())
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)

good_topic = "/qos_events_product/mte_raise/good/p%d" % os.getpid()
bad_topic = "/qos_events_product/mte_raise/bad/p%d" % os.getpid()

peer_matched = threading.Event()


def peer_matched_callback(_info):
    peer_matched.set()


def raising_matched_callback(_info):
    raise RuntimeError("deliberate-qos-event-callback-failure")


good_pub = node.create_publisher(
    UInt64, good_topic, 10,
    event_callbacks=PublisherEventCallbacks(
        matched=peer_matched_callback, use_default_callbacks=False))
node.create_subscription(UInt64, good_topic, lambda _msg: None, 10)

spin_errors = []


def _spin_target():
    try:
        executor.spin()
    except BaseException as exc:  # noqa: BLE001 -- captured for the assertions below
        spin_errors.append(exc)


thread = threading.Thread(target=_spin_target, name="qos-event-mte-raise-spin")
thread.start()

# Proven first, on the spin thread, before the raising one is even created:
# normal event-callback dispatch works under a real MultiThreadedExecutor.
assert peer_matched.wait(timeout=15.0), "peer (non-raising) matched event never fired"
print("QOS_EVENT_MTE_RAISE_PEER_DISPATCHED", flush=True)

# Now introduce the raising side -- its "matched" event fires as soon as the
# subscription below is discovered, on the live spin thread.
bad_pub = node.create_publisher(
    UInt64, bad_topic, 10,
    event_callbacks=PublisherEventCallbacks(
        matched=raising_matched_callback, use_default_callbacks=False))
node.create_subscription(UInt64, bad_topic, lambda _msg: None, 10)
del bad_pub

print("QOS_EVENT_MTE_RAISE_READY", flush=True)

thread.join(timeout=15.0)

# Reaching here (without the process having aborted) is itself proof #1.
# Proofs #2 and #3 are asserted below.
if thread.is_alive():
    print("QOS_EVENT_MTE_RAISE_SPIN_THREAD_STILL_ALIVE", flush=True)
else:
    assert len(spin_errors) == 1, (
        "expected exactly one captured exception on the spin thread, got %r"
        % (spin_errors,)
    )
    captured = spin_errors[0]
    assert type(captured) is RuntimeError, (
        "expected RuntimeError, got %r" % (captured,)
    )
    assert str(captured) == "deliberate-qos-event-callback-failure", (
        "unexpected exception message: %r" % (str(captured),)
    )
    print("QOS_EVENT_MTE_RAISE_CONTAINED_AND_RERAISED", flush=True)

executor.shutdown(timeout_sec=10.0)
node.destroy_node()
rclpy.shutdown()
# Reaching here means the process did NOT abort.
print("QOS_EVENT_MTE_RAISE_DID_NOT_ABORT", flush=True)
