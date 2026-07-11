#!/usr/bin/env python3
"""Helper process for test_clean_exit.py.

The narrowest exercise of the teardown path: bring rclcpp up, create a node with
a publisher and a subscription, pump a couple of messages through the C++
backend, then return normally from main() -- deliberately NO ``os._exit``. The
point is to let the process go through real Python interpreter finalization so
that rclcppyy's ordered teardown (rclcppyy.shutdown_rclcpp, registered on
cppyy_kit's atexit hook) runs and the rclcpp context / DDS layer comes down
before cppyy's Cling teardown.

Prints ``CLEAN_EXIT_OK`` as its last line; the test asserts that plus a zero
return code and no fault text on stderr. This is the regression tripwire for the
teardown-segfault fix.
"""
import time

from rclcppyy.bringup_rclcpp import bringup_rclcpp

SPIN_DEADLINE_S = 15.0


def main():
    rclcpp = bringup_rclcpp()
    from std_msgs.msg import String

    if not rclcpp.ok():
        rclcpp.init()

    node = rclcpp.Node("rclcppyy_clean_exit_helper")
    received = []
    sub = node.create_subscription(  # noqa: F841 - keep the subscription alive
        String, "rclcppyy_clean_exit", lambda m: received.append(str(m.data)), 10)
    pub = node.create_publisher(String, "rclcppyy_clean_exit", 10)

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + SPIN_DEADLINE_S
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)

    for i in range(3):
        msg = String()
        msg.data = f"clean {i}"
        pub.publish(msg)
    while len(received) < 3 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.01)

    print("CLEAN_EXIT_OK", flush=True)
    # Intentionally no os._exit: fall off the end of main() and let the
    # interpreter finalize normally. If teardown regresses, the return code goes
    # nonzero / a fault lands on stderr and the test catches it.


if __name__ == "__main__":
    main()
