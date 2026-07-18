#!/usr/bin/env python3
"""Scheduled entity-churn, concurrency, and signal-shutdown stress probes."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import faulthandler
import json
import os
from pathlib import Path
import platform
import random
import resource
import selectors
import signal
import subprocess
import sys
import threading
import time
import uuid


SCHEMA = "rclcppyy.runtime-stress/v1"


def _rss_kib() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)


def entity_churn(cycles: int, seed: int) -> dict:
    import rclcppyy
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    rclcppyy.enable_cpp_acceleration()
    start_rss = _rss_kib()
    randomizer = random.Random(seed)
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
        entities = [
            (node.destroy_client, client),
            (node.destroy_service, service),
            (node.destroy_guard_condition, guard),
            (node.destroy_timer, timer),
            (node.destroy_subscription, subscription),
            (node.destroy_publisher, publisher),
        ]
        randomizer.shuffle(entities)
        for destroy, entity in entities:
            assert destroy(entity)
        node.destroy_node()
        context.shutdown()
    growth = max(0, _rss_kib() - start_rss)
    print("ENTITY_CHURN_OK cycles=%d peak_rss_growth_kib=%d" % (cycles, growth))
    return {"cycles": cycles, "peak_rss_growth_kib": growth}


def concurrent_publish(threads: int, messages_per_thread: int) -> dict:
    import rclcppyy
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    rclcppyy.enable_cpp_acceleration()
    expected = threads * messages_per_thread
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

    # The probe asserts lossless delivery of one complete concurrent burst. Size
    # the history for that contract so scheduler timing cannot turn it into an
    # accidental depth-100 drop test before the executor gets CPU.
    subscription = node.create_subscription(
        String, "stress_concurrent", on_message, expected)
    publisher = node.create_publisher(
        String, "stress_concurrent", expected)
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
    return {
        "threads": threads,
        "messages_per_thread": messages_per_thread,
        "qos_depth": expected,
        "messages_expected": expected,
        "messages_received": len(received),
    }


def signal_worker(accelerated: bool = True) -> None:
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.executors import ExternalShutdownException

    if accelerated:
        import rclcppyy
        rclcppyy.enable_cpp_acceleration()
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=[])
    node = rclpy.create_node("stress_signal_%d" % os.getpid())
    ready_timer = None

    def mark_spinning():
        ready_timer.cancel()
        print("SIGNAL_WORKER_READY", flush=True)

    # Announce readiness from inside an executor callback. Printing before
    # rclpy.spin() enters its executor leaves a race where SIGTERM can shut the
    # context down before the global executor has been created.
    ready_timer = node.create_timer(0.01, mark_spinning)
    try:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
        except RCLError:
            if rclpy.ok():
                raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("SIGNAL_WORKER_CLEAN", flush=True)


def signal_shutdown(timeout: float, accelerated: bool = True) -> dict:
    started = time.monotonic()
    worker_flag = "--signal-worker" if accelerated else "--stock-signal-worker"
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), worker_flag],
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
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGUSR1)
            time.sleep(0.25)
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate(timeout=5)
            output.extend(stdout)
            raise AssertionError(
                "signal worker timed out after %.3fs (accelerated=%s):\n%s\n%s" % (
                    timeout,
                    accelerated,
                    output.decode(errors="replace"),
                    stderr.decode(errors="replace"),
                )
            )
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
    return {
        "returncode": process.returncode,
        "accelerated": accelerated,
        "clean_marker": True,
        "duration_s": round(time.monotonic() - started, 6),
    }


def _write_evidence(path: Path, evidence: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.%s.tmp" % (path.name, uuid.uuid4().hex))
    temporary.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def run_stress(args) -> dict:
    import rclcppyy
    import rclpy  # noqa: F401
    from rclpy.utilities import get_rmw_implementation_identifier

    # Exclude one-time interpreter/header setup from the entity-lifetime RSS budget.
    rclcppyy.enable_cpp_acceleration()
    started_at = datetime.now(timezone.utc).isoformat()
    started = time.monotonic()
    start_rss = _rss_kib()
    rounds = []
    round_index = 0
    while (
        round_index < args.repetitions
        or time.monotonic() - started < args.min_duration_seconds
    ):
        round_seed = args.seed + round_index
        round_started = time.monotonic()
        churn = entity_churn(args.cycles, round_seed)
        concurrent = concurrent_publish(args.threads, args.messages_per_thread)
        signals = [signal_shutdown(args.timeout)
                   for _ in range(args.signal_repetitions)]
        rounds.append({
            "index": round_index,
            "seed": round_seed,
            "duration_s": round(time.monotonic() - round_started, 6),
            "entity_churn": churn,
            "concurrent_publish": concurrent,
            "signal_shutdown": signals,
        })
        round_index += 1

    rss_growth = max(0, _rss_kib() - start_rss)
    evidence = {
        "schema": SCHEMA,
        "started_at": started_at,
        "architecture": platform.machine(),
        "python": platform.python_version(),
        "rmw_implementation": get_rmw_implementation_identifier(),
        "parameters": {
            "cycles": args.cycles,
            "threads": args.threads,
            "messages_per_thread": args.messages_per_thread,
            "timeout_s": args.timeout,
            "repetitions": args.repetitions,
            "signal_repetitions": args.signal_repetitions,
            "min_duration_s": args.min_duration_seconds,
            "seed": args.seed,
            "max_rss_growth_kib": args.max_rss_growth_kib,
        },
        "rounds": rounds,
        "summary": {
            "rounds": len(rounds),
            "entity_cycles": sum(
                item["entity_churn"]["cycles"] for item in rounds),
            "messages_expected": sum(
                item["concurrent_publish"]["messages_expected"]
                for item in rounds),
            "messages_received": sum(
                item["concurrent_publish"]["messages_received"]
                for item in rounds),
            "clean_signal_shutdowns": sum(
                len(item["signal_shutdown"]) for item in rounds),
            "duration_s": round(time.monotonic() - started, 6),
            "peak_rss_growth_kib": rss_growth,
        },
    }
    if args.max_rss_growth_kib and rss_growth > args.max_rss_growth_kib:
        raise AssertionError(
            "peak RSS growth %d KiB exceeds budget %d KiB" % (
                rss_growth, args.max_rss_growth_kib))
    return evidence


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--messages-per-thread", type=int, default=250)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--signal-repetitions", type=int, default=1)
    parser.add_argument("--min-duration-seconds", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=20260718)
    parser.add_argument("--max-rss-growth-kib", type=int, default=0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--signal-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--stock-signal-worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.signal_worker:
        signal_worker(accelerated=True)
        return 0
    if args.stock_signal_worker:
        signal_worker(accelerated=False)
        return 0
    if min(
        args.cycles,
        args.threads,
        args.messages_per_thread,
        args.repetitions,
        args.signal_repetitions,
    ) <= 0:
        parser.error("cycle, thread, message, and repetition counts must be positive")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    if min(args.min_duration_seconds, args.max_rss_growth_kib) < 0:
        parser.error("duration and RSS budget must not be negative")
    evidence = run_stress(args)
    if args.output is not None:
        _write_evidence(args.output, evidence)
    print(
        "RUNTIME_STRESS_OK rounds=%d duration_s=%.3f rss_growth_kib=%d" % (
            evidence["summary"]["rounds"],
            evidence["summary"]["duration_s"],
            evidence["summary"]["peak_rss_growth_kib"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
