#!/usr/bin/env python3
"""Scheduled entity-churn, concurrency, and signal-shutdown stress probes."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import faulthandler
import importlib.util
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


SCHEMA = "rclcppyy.runtime-stress/v3"
SIGNAL_SCHEMA = "rclcppyy.signal-stress/v2"
REPO_ROOT = Path(__file__).resolve().parents[2]


def _rss_kib() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)


def _git(repo: Path, *arguments: str) -> str | None:
    process = subprocess.run(
        ["git", "-C", str(repo), *arguments],
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    return process.stdout.strip() if process.returncode == 0 else None


def _source_metadata() -> dict:
    lock_path = REPO_ROOT / "suite-source.lock.json"
    suite_lock = json.loads(lock_path.read_text(encoding="utf-8"))
    spec = importlib.util.find_spec("rclcpp_kit")
    suite_origin = None if spec is None else spec.origin
    suite_checkout = None
    suite_commit = None
    suite_dirty = None
    if suite_origin:
        checkout = _git(Path(suite_origin).resolve().parent, "rev-parse", "--show-toplevel")
        if checkout:
            suite_checkout = str(Path(checkout).resolve())
            suite_commit = _git(Path(checkout), "rev-parse", "HEAD")
            status = _git(Path(checkout), "status", "--porcelain")
            suite_dirty = None if status is None else bool(status)
    product_status = _git(REPO_ROOT, "status", "--porcelain")
    return {
        "product": {
            "repository": str(REPO_ROOT),
            "commit": _git(REPO_ROOT, "rev-parse", "HEAD"),
            "dirty": None if product_status is None else bool(product_status),
        },
        "suite": {
            "locked_commit": suite_lock.get("commit"),
            "repository": suite_checkout,
            "active_commit": suite_commit,
            "dirty": suite_dirty,
            "module_origin": suite_origin,
        },
    }


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


def _failure(round_index: int, probe: str, exception: Exception, **fields) -> dict:
    value = {
        "round": round_index,
        "probe": probe,
        "exception_type": type(exception).__name__,
        "error": str(exception),
    }
    value.update(fields)
    return value


def run_signal_stress(repetitions: int, timeout: float, accelerated: bool) -> dict:
    started_at = datetime.now(timezone.utc).isoformat()
    started = time.monotonic()
    source = _source_metadata()
    results = []
    failures = []
    for attempt in range(repetitions):
        try:
            results.append(signal_shutdown(timeout, accelerated=accelerated))
        except Exception as exception:
            failures.append({
                "attempt": attempt,
                "probe": "signal_shutdown",
                "exception_type": type(exception).__name__,
                "error": str(exception),
            })
            break
    return {
        "schema": SIGNAL_SCHEMA,
        "started_at": started_at,
        "command": list(sys.argv),
        "architecture": platform.machine(),
        "python": platform.python_version(),
        "source": source,
        "backend": "accelerated" if accelerated else "stock",
        "parameters": {
            "requested_repetitions": repetitions,
            "timeout_s": timeout,
            "signal": "SIGTERM",
            "fresh_process_per_attempt": True,
        },
        "results": results,
        "failures": failures,
        "summary": {
            "result": "fail" if failures else "pass",
            "attempts": len(results) + len(failures),
            "clean_shutdowns": len(results),
            "failures": len(failures),
            "duration_s": round(time.monotonic() - started, 6),
        },
        "performance_claims_allowed": False,
    }


def run_stress(args, *, enable_acceleration=None, rmw_identifier=None) -> dict:
    if enable_acceleration is None or rmw_identifier is None:
        import rclcppyy
        from rclpy.utilities import get_rmw_implementation_identifier

        if enable_acceleration is None:
            enable_acceleration = rclcppyy.enable_cpp_acceleration
        if rmw_identifier is None:
            rmw_identifier = get_rmw_implementation_identifier

    # Exclude one-time interpreter/header setup from the entity-lifetime RSS budget.
    enable_acceleration()
    started_at = datetime.now(timezone.utc).isoformat()
    started = time.monotonic()
    source = _source_metadata()
    start_rss = _rss_kib()
    rounds = []
    failures = []
    signal_probe_enabled = args.signal_repetitions > 0
    round_index = 0
    while (
        round_index < args.repetitions
        or time.monotonic() - started < args.min_duration_seconds
    ):
        round_seed = args.seed + round_index
        round_started = time.monotonic()
        round_evidence = {
            "index": round_index,
            "seed": round_seed,
            "entity_churn": None,
            "concurrent_publish": None,
            "signal_shutdown": [],
        }
        fatal_failure = False
        try:
            round_evidence["entity_churn"] = entity_churn(args.cycles, round_seed)
        except Exception as exception:
            failures.append(_failure(round_index, "entity_churn", exception))
            fatal_failure = True
        if not fatal_failure:
            try:
                round_evidence["concurrent_publish"] = concurrent_publish(
                    args.threads, args.messages_per_thread)
            except Exception as exception:
                failures.append(_failure(round_index, "concurrent_publish", exception))
                fatal_failure = True
        if not fatal_failure and signal_probe_enabled:
            for repetition in range(args.signal_repetitions):
                try:
                    round_evidence["signal_shutdown"].append(
                        signal_shutdown(args.timeout))
                except Exception as exception:
                    failures.append(_failure(
                        round_index,
                        "signal_shutdown",
                        exception,
                        repetition=repetition,
                        accelerated=True,
                    ))
                    # Each signal worker is isolated, so churn/concurrency can keep
                    # soaking. One timeout is enough to fail the gate; disabling
                    # later signal attempts avoids spending 30 seconds per repeat.
                    signal_probe_enabled = False
                    round_evidence["signal_probe_disabled_after_failure"] = True
                    break
        elif not signal_probe_enabled and args.signal_repetitions:
            round_evidence["signal_probe_skipped_after_failure"] = True
        round_evidence["duration_s"] = round(
            time.monotonic() - round_started, 6)
        rounds.append(round_evidence)
        round_index += 1
        if fatal_failure:
            break

    rss_growth = max(0, _rss_kib() - start_rss)
    evidence = {
        "schema": SCHEMA,
        "started_at": started_at,
        "command": list(sys.argv),
        "architecture": platform.machine(),
        "python": platform.python_version(),
        "source": source,
        "rmw_implementation": rmw_identifier(),
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
        "failures": failures,
        "summary": {
            "result": "fail" if failures else "pass",
            "rounds": len(rounds),
            "entity_cycles": sum(
                item["entity_churn"]["cycles"]
                for item in rounds if item["entity_churn"] is not None),
            "messages_expected": sum(
                item["concurrent_publish"]["messages_expected"]
                for item in rounds if item["concurrent_publish"] is not None),
            "messages_received": sum(
                item["concurrent_publish"]["messages_received"]
                for item in rounds if item["concurrent_publish"] is not None),
            "clean_signal_shutdowns": sum(
                len(item["signal_shutdown"]) for item in rounds),
            "failures": len(failures),
            "duration_s": round(time.monotonic() - started, 6),
            "peak_rss_growth_kib": rss_growth,
        },
        "performance_claims_allowed": False,
    }
    if args.max_rss_growth_kib and rss_growth > args.max_rss_growth_kib:
        evidence["failures"].append({
            "round": None,
            "probe": "rss_budget",
            "exception_type": "BudgetExceeded",
            "error": "peak RSS growth %d KiB exceeds budget %d KiB" % (
                rss_growth, args.max_rss_growth_kib),
        })
        evidence["summary"]["failures"] = len(evidence["failures"])
        evidence["summary"]["result"] = "fail"
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
    parser.add_argument("--signal-only", action="store_true")
    parser.add_argument(
        "--signal-backend", choices=("accelerated", "stock"),
        default="accelerated")
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
    if args.signal_only:
        if args.signal_repetitions <= 0:
            parser.error("signal-only repetitions must be positive")
        if args.timeout <= 0:
            parser.error("timeout must be positive")
        evidence = run_signal_stress(
            args.signal_repetitions,
            args.timeout,
            accelerated=args.signal_backend == "accelerated",
        )
        if args.output is not None:
            _write_evidence(args.output, evidence)
        print(
            "SIGNAL_STRESS_%s backend=%s attempts=%d clean=%d failures=%d" % (
                "FAILED" if evidence["failures"] else "OK",
                evidence["backend"],
                evidence["summary"]["attempts"],
                evidence["summary"]["clean_shutdowns"],
                evidence["summary"]["failures"],
            )
        )
        return 1 if evidence["failures"] else 0
    if min(
        args.cycles,
        args.threads,
        args.messages_per_thread,
        args.repetitions,
    ) <= 0:
        parser.error("cycle, thread, message, and repetition counts must be positive")
    if args.signal_repetitions < 0:
        parser.error("signal repetitions must not be negative")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    if min(args.min_duration_seconds, args.max_rss_growth_kib) < 0:
        parser.error("duration and RSS budget must not be negative")
    evidence = run_stress(args)
    if args.output is not None:
        _write_evidence(args.output, evidence)
    if evidence["failures"]:
        print(
            "RUNTIME_STRESS_FAILED rounds=%d failures=%d duration_s=%.3f" % (
                evidence["summary"]["rounds"],
                evidence["summary"]["failures"],
                evidence["summary"]["duration_s"],
            ),
            file=sys.stderr,
        )
        return 1
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
