#!/usr/bin/env python3
"""Proves callback-exception containment under real ``MultiThreadedExecutor``
dispatch (defect A fix -- see docs/plans/PLAN-mte-unlock.md).

This file used to document a process abort: a raising Python subscription
callback, invoked from a native ``rclcpp`` MultiThreadedExecutor worker
thread, crossed into C++ via cppyy as an uncaught exception and reached
``std::terminate()`` (SIGABRT) -- ``rclcpp::executors::MultiThreadedExecutor``
has no callback exception boundary around a worker-thread invocation. The
fix wraps the user callback at the product hand-off (``create_subscription``
et al. in ``direct_cpp.py``) so a raise is captured into a per-node sink
instead of ever reaching that boundary; the owning executor drains the sink
and re-raises the exact exception on its own spin/pump thread.

This helper now proves three things in one run: (1) the process exits
cleanly (no abort), (2) the exact exception surfaces on the spin thread
(type and message), and (3) a non-raising peer callback still dispatches
normally in the same run -- containing one callback's exception does not
poison the others or the whole executor.

Still runs only under the test-only construction guard (public construction
fails closed before ever reaching this code path this wave).
"""

import os
import threading
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init(args=[])
node = Node("mte_exception_containment_%d" % os.getpid())
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)

bad_topic = "/direct_cpp/mte_exception_containment/bad/p%d" % os.getpid()
good_topic = "/direct_cpp/mte_exception_containment/good/p%d" % os.getpid()
bad_pub = node.create_publisher(UInt64, bad_topic, 10)
good_pub = node.create_publisher(UInt64, good_topic, 10)

peer_received = threading.Event()


def raising_callback(_message):
    raise RuntimeError("deliberate-mte-callback-failure")


def peer_callback(_message):
    peer_received.set()


node.create_subscription(UInt64, bad_topic, raising_callback, 10)
node.create_subscription(UInt64, good_topic, peer_callback, 10)

deadline = time.monotonic() + 15.0
while (
    (bad_pub.get_subscription_count() < 1 or good_pub.get_subscription_count() < 1)
    and time.monotonic() < deadline
):
    executor.spin_once(timeout_sec=0.05)
assert bad_pub.get_subscription_count() == 1
assert good_pub.get_subscription_count() == 1
# Printed (and flushed) before the raising publish so the wrapper can tell
# "setup never completed" apart from "containment/proof failed".
print("MTE_EXCEPTION_CRASH_READY", flush=True)

spin_errors = []


def _spin_target():
    try:
        executor.spin()
    except BaseException as exc:  # noqa: BLE001 -- captured for the assertions below
        spin_errors.append(exc)


thread = threading.Thread(target=_spin_target, name="mte-exception-containment-spin")
thread.start()

# The peer message goes first and is confirmed delivered *before* the
# raising one is even published -- proves non-raising dispatch continues
# under real MultiThreadedExecutor concurrency without depending on exact
# ordering against the contained exception (that only needs to happen at
# some point in the same run, not concurrently with this).
good_pub.publish(UInt64(data=1))
assert peer_received.wait(timeout=15.0), "peer (non-raising) callback never fired"

bad_pub.publish(UInt64(data=2))
thread.join(timeout=15.0)

# Reaching here (without the process having aborted) is itself proof #1.
# Proofs #2 and #3 are asserted below.
if thread.is_alive():
    print("MTE_EXCEPTION_CRASH_SPIN_THREAD_STILL_ALIVE", flush=True)
else:
    assert len(spin_errors) == 1, (
        "expected exactly one captured exception on the spin thread, got %r"
        % (spin_errors,)
    )
    captured = spin_errors[0]
    assert type(captured) is RuntimeError, (
        "expected RuntimeError, got %r" % (captured,)
    )
    assert str(captured) == "deliberate-mte-callback-failure", (
        "unexpected exception message: %r" % (str(captured),)
    )
    print("MTE_EXCEPTION_CRASH_CONTAINED_AND_RERAISED", flush=True)

executor.shutdown(timeout_sec=10.0)
node.destroy_node()
rclpy.shutdown()
# Reaching here means the process did NOT abort.
print("MTE_EXCEPTION_CRASH_DID_NOT_ABORT", flush=True)
