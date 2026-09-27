#!/usr/bin/env python3
"""Destroy-under-transition-dispatch stress (PLAN-lifecycle.md P1, risk 3):
an external (non-dispatch) thread destroys a DirectLifecycleNode while its
own on_configure transition callback is genuinely in flight on a native
MultiThreadedExecutor worker -- dispatched from inside the node's native
/change_state service handler. Expected observable: destroy_node() blocks
until the callback returns (the in-flight counter/_wait_quiescent path),
then completes cleanly -- no crash, no std::terminate. N=50 iterations,
watchdog.
"""
import faulthandler
import os
import sys
import threading
import time

import rclcppyy


WATCHDOG_SECONDS = 180.0
faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)

rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=["lifecycle_msgs/srv/ChangeState"])

import rclpy  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn  # noqa: E402

ITERATIONS = 50
SLOW_SLEEP_S = 0.2


class SlowConfigureNode(LifecycleNode):
    def __init__(self, name):
        super().__init__(name)
        self.entered = threading.Event()

    def on_configure(self, state):
        self.entered.set()
        time.sleep(SLOW_SLEEP_S)
        return TransitionCallbackReturn.SUCCESS


def run_iteration(index, pid):
    node = SlowConfigureNode(
        "lifecycle_destroy_dispatch_%d_%d" % (pid, index))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_errors = []

    def _spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            spin_errors.append(exc)

    spin_thread = threading.Thread(
        target=_spin_target, name="lifecycle-destroy-dispatch-spin")
    spin_thread.start()

    client_node = rclpy.create_node(
        "lifecycle_destroy_dispatch_client_%d_%d" % (pid, index))
    client_executor = SingleThreadedExecutor()
    client_executor.add_node(client_node)
    client = client_node.create_client(
        ChangeState, "/%s/change_state" % node.get_name())
    assert client.wait_for_service(timeout_sec=15.0), "service not available"

    request = ChangeState.Request()
    request.transition.id = Transition.TRANSITION_CONFIGURE
    future = client.call_async(request)

    assert node.entered.wait(timeout=15.0), "on_configure never entered"

    # This (main/test) thread is NOT dispatching any callback for this
    # node -- destroy_node() must take the synchronous quiescence-wait
    # path, blocking here until on_configure returns, never severing the
    # transition-callback bridge while it might still be in flight.
    before = time.monotonic()
    node.destroy_node()
    elapsed = time.monotonic() - before
    assert elapsed >= SLOW_SLEEP_S * 0.5, (
        "destroy_node returned suspiciously fast (%.3fs) -- did it "
        "actually wait for the in-flight transition callback?" % elapsed
    )

    # The service call may or may not resolve successfully depending on
    # exactly when destroy_node() tore down the node's services relative to
    # the in-flight request -- only crash-freedom and the wait-for-
    # quiescence above are asserted; the response value is not.
    client_executor.spin_until_future_complete(future, timeout_sec=15.0)

    assert executor.shutdown(timeout_sec=15.0) is True
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after teardown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)

    client_executor.remove_node(client_node)
    client_node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_iteration(index, pid)
            print("LIFECYCLE_DESTROY_DISPATCH_ITER_%d_OK" % index, flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("LIFECYCLE_DESTROY_DISPATCH_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
