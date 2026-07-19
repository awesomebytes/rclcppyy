#!/usr/bin/env python3
"""Documents the CURRENT process-abort behavior for a raising Python
callback under real ``MultiThreadedExecutor`` dispatch.

This is why public ``DirectMultiThreadedExecutor`` construction is
fail-closed this wave (see
``rclcppyy.direct_executors._MULTI_THREADED_FAIL_CLOSED_REASON``): a Python
subscription callback that raises, invoked from a native ``rclcpp``
MultiThreadedExecutor worker thread, crosses into C++ via cppyy as an
uncaught exception. ``rclcpp::executors::MultiThreadedExecutor``'s own
dispatch has no callback exception boundary around that worker-thread
invocation (unlike the single, outer ``spin()``-calling thread, which the
suite's ``ExecutorThread`` does wrap in a try/catch -- see
``rclcpp_kit/native.py``'s ``ExecutorThread::close()``/constructor). The
result is an unhandled C++ exception unwinding off the top of that worker
thread's stack: ``std::terminate()``, i.e. the whole process aborts
(SIGABRT), not a catchable Python exception. This is proven, reproducible,
and the reason "surface .exceptions, raise on shutdown" (this lane's
original best-effort design) is not actually reachable for real concurrent
(threads>=1) dispatch -- see docs/plans/PLAN-executor-slice.md.

This helper only runs under the test-only construction guard (public
construction fails closed before ever reaching this code path). It exists
so a FUTURE change to this behavior -- for better (contained) or worse
(silent data loss) -- is caught by its pytest wrapper, not silently missed.
"""

import os
import threading
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_executors import (  # noqa: E402
    _multi_threaded_construction_test_only,
)
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init(args=[])
node = Node("mte_exception_crash_%d" % os.getpid())
with _multi_threaded_construction_test_only():
    executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)

topic = "/direct_cpp/mte_exception_crash/p%d" % os.getpid()
pub = node.create_publisher(UInt64, topic, 10)


def raising_callback(_message):
    raise RuntimeError("deliberate-mte-callback-failure")


node.create_subscription(UInt64, topic, raising_callback, 10)

deadline = time.monotonic() + 15.0
while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert pub.get_subscription_count() == 1
# Printed (and flushed) before the crash-inducing publish so the wrapper can
# tell "setup never completed" apart from "the documented abort happened".
print("MTE_EXCEPTION_CRASH_READY", flush=True)

thread = threading.Thread(target=executor.spin, name="mte-exception-crash-spin")
thread.start()
pub.publish(UInt64(data=1))
thread.join(timeout=15.0)
# Reaching here means the process did NOT abort -- a behavior change from
# what this helper documents, investigate rather than treat as a pass.
print("MTE_EXCEPTION_CRASH_DID_NOT_ABORT", flush=True)
