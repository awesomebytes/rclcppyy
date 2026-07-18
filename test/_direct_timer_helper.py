#!/usr/bin/env python3
"""Prove direct-node timers need no per-instance C++ compilation."""

import time

import cppyy
import rclpy

import rclcppyy


TIMEOUT_S = 10.0


def main():
    rclpy.init(args=[])
    node = rclcppyy.Node("direct_timer_factory")
    calls = []

    original_cppdef = cppyy.cppdef

    def reject_cppdef(*_args, **_kwargs):
        raise AssertionError("timer creation attempted per-instance C++ compilation")

    cppyy.cppdef = reject_cppdef
    try:
        timer = node.create_timer(0.01, lambda: calls.append("timer"))
    finally:
        cppyy.cppdef = original_cppdef

    assert timer in node._cpp_timers
    assert node._cpp_timers[timer]["callback"] is not None
    assert node._cpp_timers[timer]["cpp_callback"] is not None

    executor = node._rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node._rclcpp_node)
    deadline = time.monotonic() + TIMEOUT_S
    while not calls and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.005)
    assert calls

    timer.cancel()
    executor.remove_node(node._rclcpp_node)
    node.destroy_node()
    rclpy.try_shutdown()

    timer_records = [
        record for record in rclcppyy.status()["entities"]
        if record["metadata"].get("entity_type") == "timer"
    ]
    assert len(timer_records) == 1
    assert "reusable_cppyy_std_function" in timer_records[0]["policies"]
    print("DIRECT_TIMER_FACTORY_OK")


if __name__ == "__main__":
    main()
