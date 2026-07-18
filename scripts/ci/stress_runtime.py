#!/usr/bin/env python3
"""Scheduled entity-churn, concurrency, and signal-shutdown stress probes."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import resource
import selectors
import signal
import subprocess
import sys
import threading
import time


def _rss_kib() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)


def entity_churn(cycles: int) -> None:
    import rclcppyy
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    rclcppyy.enable_cpp_acceleration()
    start_rss = _rss_kib()
    for cycle in range(cycles):
        context = Context()
        context.init(args=[])
        node = rclpy.create_node(
            "stress_churn_%d_%d" % (os.getpid(), cycle),
            context=context,
            start_parameter_services=False,
        )
        group = node.default_callback_group
        publisher = node.create_publisher(String, "stress_topic", 10)
        subscription = node.create_subscription(
            String, "stress_topic", lambda _message: None, 10,
            callback_group=group)
        timer = node.create_timer(60.0, lambda: None, callback_group=group)
        guard = node.create_guard_condition(lambda: None, callback_group=group)
        service = node.create_service(
            SetBool, "stress_service", lambda request, response: response,
            callback_group=group)
        client = node.create_client(
            SetBool, "stress_service", callback_group=group)
        node.declare_parameter("iteration", cycle)
        result = node.set_parameters([Parameter("iteration", value=cycle + 1)])
        assert result[0].successful
        publisher.publish(String(data="cycle-%d" % cycle))
        assert node.destroy_client(client)
        assert node.destroy_service(service)
        assert node.destroy_guard_condition(guard)
        assert node.destroy_timer(timer)
        assert node.destroy_subscription(subscription)
        assert node.destroy_publisher(publisher)
        node.destroy_node()
        context.shutdown()
    growth = max(0, _rss_kib() - start_rss)
    print("ENTITY_CHURN_OK cycles=%d peak_rss_growth_kib=%d" % (cycles, growth))


def concurrent_publish(threads: int, messages_per_thread: int) -> None:
    import rclcppyy
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    rclcppyy.enable_cpp_acceleration()
    context = Context()
    context.init(args=[])
    node = rclpy.create_node(
        "stress_concurrent_%d" % os.getpid(), context=context,
        start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = set()
    lock = threading.Lock()

    def on_message(message):
        with lock:
            received.add(message.data)

    subscription = node.create_subscription(String, "stress_concurrent", on_message, 100)
    publisher = node.create_publisher(String, "stress_concurrent", 100)
    deadline = time.monotonic() + 10.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert publisher.get_subscription_count() >= 1

    workers = []
    for worker_id in range(threads):
        def publish(worker=worker_id):
            for sequence in range(messages_per_thread):
                publisher.publish(String(data="%d:%d" % (worker, sequence)))
        thread = threading.Thread(target=publish)
        workers.append(thread)
        thread.start()
    while any(thread.is_alive() for thread in workers):
        executor.spin_once(timeout_sec=0.01)
    for thread in workers:
        thread.join()
    expected = threads * messages_per_thread
    deadline = time.monotonic() + 15.0
    while len(received) < expected and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert len(received) == expected, (len(received), expected)

    assert node.destroy_publisher(publisher)
    assert node.destroy_subscription(subscription)
    executor.remove_node(node)
    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    context.shutdown()
    print("CONCURRENT_PUBLISH_OK threads=%d messages=%d" % (threads, expected))


def signal_worker() -> None:
    import rclcppyy
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclcppyy.enable_cpp_acceleration()
    rclpy.init(args=[])
    node = rclpy.create_node("stress_signal_%d" % os.getpid())
    node.create_timer(60.0, lambda: None)
    print("SIGNAL_WORKER_READY", flush=True)
    try:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("SIGNAL_WORKER_CLEAN", flush=True)


def signal_shutdown(timeout: float) -> None:
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), "--signal-worker"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + timeout
        output = bytearray()
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        while process.poll() is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not selector.select(remaining):
                break
            chunk = os.read(process.stdout.fileno(), 4096)
            if chunk:
                output.extend(chunk)
                if b"SIGNAL_WORKER_READY" in output:
                    break
        selector.close()
        assert b"SIGNAL_WORKER_READY" in output, output.decode(errors="replace")
        os.killpg(process.pid, signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=timeout)
        output.extend(stdout)
        rendered = output.decode(errors="replace")
        assert process.returncode == 0, (
            process.returncode, rendered, stderr.decode(errors="replace"))
        assert "SIGNAL_WORKER_CLEAN" in rendered, (
            rendered, stderr.decode(errors="replace"))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    print("SIGNAL_SHUTDOWN_OK")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--messages-per-thread", type=int, default=250)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--signal-worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.signal_worker:
        signal_worker()
        return 0
    if min(args.cycles, args.threads, args.messages_per_thread) <= 0:
        parser.error("cycle, thread, and message counts must be positive")
    entity_churn(args.cycles)
    concurrent_publish(args.threads, args.messages_per_thread)
    signal_shutdown(args.timeout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
