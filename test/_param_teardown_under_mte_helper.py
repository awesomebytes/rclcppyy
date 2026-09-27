#!/usr/bin/env python3
"""Committed proof for Slice 3 (PLAN-mte-unlock.md un-fail-close): the
product-level counterpart of the suite's parameter-callback teardown-under-
worker-dispatch proof (Slice 2.5a4), through the public rclpy-style API
(``DirectNode.add_on_set_parameters_callback``/``remove_on_set_parameters_
callback``) and a real, publicly-constructed ``MultiThreadedExecutor``.

on/pre/post-set-parameters callbacks are worker-dispatched (rclcpp invokes
them synchronously inside ``set_parameters``, from a worker thread or the
node's parameter service on a remote request) and were, before Slice 2.5a4,
covered by neither the product's in-flight counter nor the suite's
callable-lifetime reaper. This self-removes the callback from within its
own dispatch on a worker thread genuinely calling ``set_parameters``, drops
every Python reference, and forces ``gc.collect()``. Must stay crash-free
across every iteration.
"""
import faulthandler
import gc
import os
import sys
import threading

import rclcppyy

WATCHDOG_SECONDS = 250.0
faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

ITERATIONS = 50


def run_iteration(index, pid):
    node = Node("param_teardown_mte_%d_%d" % (pid, index))
    node.declare_parameter("test_param", 0)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    done = threading.Event()
    holder = {}

    def on_set(_parameters):
        node.remove_on_set_parameters_callback(holder["callback"])
        holder["callback"] = None
        gc.collect()
        gc.collect()
        done.set()
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    holder["callback"] = on_set
    node.add_on_set_parameters_callback(on_set)

    worker_errors = []

    def worker():
        try:
            node.set_parameters([Parameter("test_param", value=1)])
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            worker_errors.append(exc)

    worker_thread = threading.Thread(
        target=worker, name="param-teardown-mte-worker")
    worker_thread.start()
    worker_thread.join(timeout=15.0)
    assert not worker_thread.is_alive(), "worker thread hung"
    assert done.wait(timeout=1.0), "on_set callback never completed"
    assert worker_errors == [], (
        "unexpected set_parameters exception(s): %r" % (worker_errors,)
    )

    assert executor.shutdown(timeout_sec=15.0) is True
    node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_iteration(index, pid)
            print("PARAM_TEARDOWN_MTE_ITER_%d_OK" % index, flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("PARAM_TEARDOWN_MTE_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
