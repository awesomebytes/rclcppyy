#!/usr/bin/env python3
"""Regression guard for the defect-A containment shim (Slice 2 -- see
docs/plans/PLAN-mte-unlock.md): a raising Python subscription callback
under ``SingleThreadedExecutor`` must still propagate out of
``spin_once()`` with its exact type and message, unchanged from before the
shim existed.

Before the shim, the raise crossed the cppyy boundary directly out of
``self._native.spin_once()`` because single-threaded callbacks run on the
calling Python thread. The shim now catches it earlier (at the
create_subscription hand-off) so it never reaches that boundary at all;
``_spin_once_impl`` drains the node's exception sink and re-raises
immediately after the native step, on the same calling thread, within the
same ``spin_once()`` call -- so the observable stays the same even though
the mechanism changed.
"""

import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init(args=[])
node = Node("single_threaded_exception_propagation_%d" % os.getpid())
executor = SingleThreadedExecutor()
executor.add_node(node)

topic = "/direct_cpp/single_threaded_exception_propagation/p%d" % os.getpid()
pub = node.create_publisher(UInt64, topic, 10)


def raising_callback(_message):
    raise RuntimeError("deliberate-mte-callback-failure")


node.create_subscription(UInt64, topic, raising_callback, 10)

deadline = time.monotonic() + 15.0
while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert pub.get_subscription_count() == 1
print("SINGLE_THREADED_EXCEPTION_PROPAGATION_READY", flush=True)

pub.publish(UInt64(data=1))
# A matched subscription count does not mean the sample has actually
# reached the DDS reader queue yet (the same discovery-to-delivery gap
# every other pubsub helper in this suite polls for); loop spin_once()
# with a short per-call timeout until it raises, rather than relying on
# a single call to straddle the exact moment delivery completes.
raised = None
deadline = time.monotonic() + 15.0
while raised is None and time.monotonic() < deadline:
    try:
        executor.spin_once(timeout_sec=0.1)
    except RuntimeError as exc:
        raised = exc

if raised is None:
    # Reaching here means spin_once() never raised -- a regression,
    # investigate rather than treat as a pass.
    print("SINGLE_THREADED_EXCEPTION_PROPAGATION_DID_NOT_RAISE", flush=True)
else:
    assert str(raised) == "deliberate-mte-callback-failure", (
        "unexpected exception message: %r" % (str(raised),)
    )
    print("SINGLE_THREADED_EXCEPTION_PROPAGATION_OK", flush=True)

executor.shutdown(timeout_sec=10.0)
node.destroy_node()
rclpy.shutdown()
