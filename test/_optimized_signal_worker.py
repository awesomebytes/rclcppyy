#!/usr/bin/env python3
"""Fresh-process SIGTERM worker for optimized executor integration tests."""

import argparse
import faulthandler
from inspect import signature
import os
import signal

import rclcppyy
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("global", "single", "multi"), required=True)
    args = parser.parse_args(argv)

    signatures = {
        "global": signature(rclpy.spin),
        "single": signature(SingleThreadedExecutor.spin),
        "multi": signature(MultiThreadedExecutor.spin),
    }
    rclcppyy.enable_cpp_acceleration(profile="optimized")
    assert signature(rclpy.spin) == signatures["global"]
    assert signature(SingleThreadedExecutor.spin) == signatures["single"]
    assert signature(MultiThreadedExecutor.spin) == signatures["multi"]

    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=[])
    node = rclpy.create_node("optimized_signal_%s_%d" % (args.mode, os.getpid()))
    executor = None
    if args.mode == "single":
        executor = SingleThreadedExecutor()
    elif args.mode == "multi":
        executor = MultiThreadedExecutor(num_threads=2)
    global_executor = None
    if executor is not None:
        executor.add_node(node)
    else:
        global_executor = rclpy.get_global_executor()

    ready_timer = None

    def mark_spinning():
        ready_timer.cancel()
        print("OPTIMIZED_SIGNAL_READY", flush=True)

    ready_timer = node.create_timer(0.01, mark_spinning)
    try:
        try:
            if executor is None:
                rclpy.spin(node)
            else:
                executor.spin()
        except ExternalShutdownException:
            pass
        except RCLError:
            if rclpy.ok():
                raise

        if executor is None:
            assert node not in global_executor.get_nodes()
            operation = "spin"
        else:
            assert node in executor.get_nodes()
            operation = "%s_threaded_spin" % args.mode

        matching = [
            record for record in rclcppyy.status()["operations"]
            if record["metadata"].get("operation") == operation
            and "optimized_bounded_wait" in record["policies"]
        ]
        assert matching, rclcppyy.status()
        record = matching[-1]
        assert record["metadata"]["bounded_wait_timeout_sec"] == 0.1
        assert record["metadata"]["mitigation"] == "signal_guard_lost_wake"
        print("OPTIMIZED_SIGNAL_STATUS_OK", flush=True)
        print("OPTIMIZED_SIGNAL_OWNERSHIP_OK", flush=True)
        print("OPTIMIZED_SIGNAL_SIGNATURES_OK", flush=True)
    finally:
        if executor is not None:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("OPTIMIZED_SIGNAL_CLEAN", flush=True)


if __name__ == "__main__":
    main()
