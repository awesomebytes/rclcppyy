#!/usr/bin/env python3
"""Committed proof for Slice 2.5c (PLAN-mte-unlock.md): extends the
self-destroy proof to the other native-owned-lifetime entity kinds fixed by
the suite (timers, services) -- a callback destroys its own timer/service
while a peer subscription's callback is genuinely in flight on another
native MultiThreadedExecutor worker. Analogous to the subscription/node
self-destroy scenarios (Scenario A/B); this is the same mechanism, just
exercising ``destroy_timer``/``destroy_service`` instead of
``destroy_subscription``/``destroy_node``.
"""
import faulthandler
import os
import sys
import threading
import time

import rclcppyy

WATCHDOG_SECONDS = 250.0
faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_executors import (  # noqa: E402
    _multi_threaded_construction_test_only,
)
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

ITERATIONS = 50
SLOW_SLEEP_S = 0.3


def drive_until(executor, predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "did not complete before the deadline"


def spin_in_background(executor):
    errors = []

    def _target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            errors.append(exc)

    thread = threading.Thread(target=_target, name="timer-service-self-destroy")
    thread.start()
    return thread, errors


def run_timer_self_destroy(index, pid):
    node = Node("timer_self_destroy_%d_%d" % (pid, index))
    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    slow_topic = "/direct_cpp/timer_self_destroy/p%d/i%d/slow" % (pid, index)
    slow_pub = node.create_publisher(UInt64, slow_topic, 10)

    slow_done = threading.Event()
    destroyer_done = threading.Event()
    timer_holder = {}

    def slow_callback(_message):
        time.sleep(SLOW_SLEEP_S)
        slow_done.set()

    def destroyer_callback():
        node.destroy_timer(timer_holder["timer"])
        destroyer_done.set()

    node.create_subscription(
        UInt64, slow_topic, slow_callback, 10, callback_group=group)
    timer_holder["timer"] = node.create_timer(
        0.05, destroyer_callback, callback_group=group)

    drive_until(executor, lambda: slow_pub.get_subscription_count() == 1)

    spin_thread, spin_errors = spin_in_background(executor)
    slow_pub.publish(UInt64(data=1))

    assert slow_done.wait(timeout=15.0), "slow peer callback never completed"
    assert destroyer_done.wait(timeout=15.0), "destroyer timer never fired"

    assert executor.shutdown(timeout_sec=15.0) is True
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after shutdown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)
    node.destroy_node()


def run_service_self_destroy(index, pid):
    node = Node("service_self_destroy_%d_%d" % (pid, index))
    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    slow_topic = "/direct_cpp/service_self_destroy/p%d/i%d/slow" % (pid, index)
    service_name = "/direct_cpp/service_self_destroy/p%d/i%d/svc" % (pid, index)
    slow_pub = node.create_publisher(UInt64, slow_topic, 10)

    slow_done = threading.Event()
    destroyer_done = threading.Event()
    service_holder = {}

    def slow_callback(_message):
        time.sleep(SLOW_SLEEP_S)
        slow_done.set()

    def destroyer_callback(request, response):
        node.destroy_service(service_holder["service"])
        destroyer_done.set()
        response.success = True
        return response

    node.create_subscription(
        UInt64, slow_topic, slow_callback, 10, callback_group=group)
    service_holder["service"] = node.create_service(
        SetBool, service_name, destroyer_callback, callback_group=group)
    client = node.create_client(SetBool, service_name, callback_group=group)

    drive_until(
        executor,
        lambda: slow_pub.get_subscription_count() == 1 and client.service_is_ready(),
    )

    spin_thread, spin_errors = spin_in_background(executor)
    slow_pub.publish(UInt64(data=1))
    client.call_async(SetBool.Request(data=True))

    assert slow_done.wait(timeout=15.0), "slow peer callback never completed"
    assert destroyer_done.wait(timeout=15.0), "destroyer service call never completed"

    assert executor.shutdown(timeout_sec=15.0) is True
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after shutdown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)
    node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_timer_self_destroy(index, pid)
            print("TIMER_SELF_DESTROY_ITER_%d_OK" % index, flush=True)
        print("TIMER_SELF_DESTROY_ALL_OK", flush=True)

        for index in range(ITERATIONS):
            run_service_self_destroy(index, pid)
            print("SERVICE_SELF_DESTROY_ITER_%d_OK" % index, flush=True)
        print("SERVICE_SELF_DESTROY_ALL_OK", flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("TIMER_SERVICE_SELF_DESTROY_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
