#!/usr/bin/env python3
"""Bounded declarative backend/workload benchmark matrix.

Every case runs a publisher and subscriber in isolated child processes.  The
parent verifies machine-readable backend evidence, explicitly starts/stops the
subscriber measurement window, samples process CPU, and owns child teardown.

``--smoke`` is a fast functional gate.  Its JSON and table output explicitly
forbid performance claims; use normal measurement mode for intentional runs.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import threading
import time
import uuid

import psutil

from _backend_marker import PREFIX as BACKEND_PREFIX, SCHEMA as BACKEND_SCHEMA
from _benchmark_matrix import (
    BACKENDS,
    DEFAULT_BACKENDS,
    DEFAULT_PAYLOAD_BYTES,
    DEFAULT_RATES_HZ,
    DEFAULT_WORKLOADS,
    SMOKE_BACKENDS,
    SMOKE_PAYLOAD_BYTES,
    SMOKE_RATES_HZ,
    SMOKE_WORKLOADS,
    WORKLOADS,
    build_cases,
    parse_int_values,
    parse_keys,
)
from _benchmark_protocol import (
    READY_PREFIX,
    RESULT_PREFIX,
    STARTED_PREFIX,
    control_document,
    validate_event,
    validate_window_result,
)
from _result_schema import build_document, dumps, write


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "bench_worker.py"
DEFAULT_SAMPLE_HZ = 4.0


def log(message):
    """Progress goes to stderr so stdout remains strict JSON when requested."""
    print(message, file=sys.stderr, flush=True)


def validate_backend_marker(marker):
    """Validate one child marker before it can become benchmark evidence."""
    if not isinstance(marker, dict) or marker.get("schema") != BACKEND_SCHEMA:
        raise ValueError("invalid benchmark backend marker schema")
    if marker.get("role") not in ("publisher", "subscriber"):
        raise ValueError("invalid benchmark backend marker role")
    if marker.get("backend") not in ("python", "cpp"):
        raise ValueError("invalid benchmark backend marker backend")
    if not isinstance(marker.get("evidence"), str) or not marker["evidence"]:
        raise ValueError("benchmark backend marker requires evidence")
    if not isinstance(marker.get("metadata"), dict):
        raise ValueError("benchmark backend marker metadata must be an object")


def require_backend_marker(child, role, expected_backend):
    """Return verified backend evidence or reject the benchmark run."""
    markers, errors = child.backend_snapshot()
    if errors:
        raise RuntimeError(f"{role} emitted invalid backend evidence: {errors[-1]}")
    matching = [marker for marker in markers if marker["role"] == role]
    if not matching:
        raise RuntimeError(f"{role} emitted no backend evidence")
    marker = matching[-1]
    if marker["backend"] != expected_backend:
        raise RuntimeError(
            f"{role} backend mismatch: expected {expected_backend}, "
            f"observed {marker['backend']} ({marker['evidence']})")
    return marker


class ChildProcess:
    """One isolated benchmark worker with parsed evidence and control events."""

    def __init__(self, name, argv, echo=False):
        self.name = name
        self.echo = echo
        self.backend_markers = []
        self.backend_marker_errors = []
        self.ready_events = []
        self.started_events = []
        self.window_results = []
        self.protocol_errors = []
        self.tail = deque(maxlen=80)
        self._lock = threading.Lock()
        self.proc = psutil.Popen(
            argv,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _parse_document(self, line, prefix, validator, destination):
        try:
            document = json.loads(line[len(prefix):])
            validator(document)
        except (json.JSONDecodeError, ValueError) as exc:
            with self._lock:
                self.protocol_errors.append(str(exc))
        else:
            with self._lock:
                destination.append(document)

    def _read_loop(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.tail.append(line)
            if self.echo:
                print(f"  [{self.name}] {line}", file=sys.stderr, flush=True)
            if line.startswith(BACKEND_PREFIX):
                try:
                    marker = json.loads(line[len(BACKEND_PREFIX):])
                    validate_backend_marker(marker)
                except (json.JSONDecodeError, ValueError) as exc:
                    with self._lock:
                        self.backend_marker_errors.append(str(exc))
                else:
                    with self._lock:
                        self.backend_markers.append(marker)
            elif line.startswith(READY_PREFIX):
                self._parse_document(
                    line, READY_PREFIX,
                    lambda document: validate_event(document, "ready"), self.ready_events)
            elif line.startswith(STARTED_PREFIX):
                self._parse_document(
                    line, STARTED_PREFIX,
                    lambda document: validate_event(document, "started"), self.started_events)
            elif line.startswith(RESULT_PREFIX):
                self._parse_document(
                    line, RESULT_PREFIX, validate_window_result, self.window_results)

    def backend_snapshot(self):
        with self._lock:
            return list(self.backend_markers), list(self.backend_marker_errors)

    def snapshot(self):
        with self._lock:
            return {
                "ready": list(self.ready_events),
                "started": list(self.started_events),
                "results": list(self.window_results),
                "errors": list(self.protocol_errors),
            }

    def send_control(self, command, run_id):
        document = control_document(command, run_id)
        try:
            self.proc.stdin.write(json.dumps(document, sort_keys=True) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise RuntimeError(f"{self.name} control pipe failed: {exc}") from exc

    def alive(self):
        return self.proc.poll() is None

    def tail_text(self):
        return "\n".join(self.tail)

    def stop(self, grace=3.0):
        if self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            try:
                self.proc.terminate()
            except psutil.Error:
                pass
        try:
            self.proc.wait(timeout=grace)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                try:
                    self.proc.kill()
                except psutil.Error:
                    pass
            try:
                self.proc.wait(timeout=grace)
            except Exception:
                pass


def _wait_for(predicate, children, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        for child in children:
            snapshot = child.snapshot()
            _markers, marker_errors = child.backend_snapshot()
            if snapshot["errors"]:
                raise RuntimeError(f"{child.name} protocol error: {snapshot['errors'][-1]}")
            if marker_errors:
                raise RuntimeError(f"{child.name} backend marker error: {marker_errors[-1]}")
            if not child.alive():
                raise RuntimeError(
                    f"{child.name} exited while waiting for {description} "
                    f"(code {child.proc.returncode})\n--- output tail ---\n{child.tail_text()}")
        time.sleep(0.02)
    tails = "\n".join(
        f"--- {child.name} output tail ---\n{child.tail_text()}" for child in children)
    raise RuntimeError(f"timed out waiting for {description} after {timeout:.1f}s\n{tails}")


def _sample_cpu(process, samples):
    total = 0.0
    try:
        total += process.cpu_percent(None)
        for child in process.children(recursive=True):
            try:
                total += child.cpu_percent(None)
            except psutil.Error:
                pass
    except psutil.Error:
        return
    samples.append(total)


def cpu_summary(samples):
    """Keep raw CPU observations and reproducible summaries."""
    if not samples:
        return {
            "samples": [], "count": 0, "mean": None, "median": None,
            "stdev": None, "min": None, "max": None,
        }
    return {
        "samples": [round(value, 3) for value in samples],
        "count": len(samples),
        "mean": statistics.fmean(samples),
        "median": statistics.median(samples),
        "stdev": statistics.stdev(samples) if len(samples) > 1 else 0.0,
        "min": min(samples),
        "max": max(samples),
    }


def _worker_argv(case, role):
    return [
        sys.executable,
        "-u",
        str(WORKER),
        "--backend", case["worker_backend"],
        "--role", role,
        "--workload", case["workload"],
        "--topic", case["topic"],
        "--node-name", f"bench_{case['case_id']}_{role}",
        "--rate-hz", str(case["target_rate_hz"]),
        "--payload-bytes", str(case["payload_bytes"]),
    ]


def _has_role_marker(child, role):
    markers, errors = child.backend_snapshot()
    return not errors and any(marker["role"] == role for marker in markers)


def run_case(case, duration, warmup_timeout, sample_hz=DEFAULT_SAMPLE_HZ, echo=False):
    """Run one bounded case and return its structured, backend-verified result."""
    subscriber = ChildProcess("subscriber", _worker_argv(case, "subscriber"), echo=echo)
    time.sleep(0.25)
    publisher = ChildProcess("publisher", _worker_argv(case, "publisher"), echo=echo)
    children = (subscriber, publisher)

    try:
        _wait_for(
            lambda: (
                bool(subscriber.snapshot()["ready"]) and
                _has_role_marker(subscriber, "subscriber") and
                _has_role_marker(publisher, "publisher")
            ),
            children,
            warmup_timeout,
            "subscriber readiness and backend evidence",
        )

        expected = case["expected_backends"]
        pub_backend = require_backend_marker(publisher, "publisher", expected["publisher"])
        sub_backend = require_backend_marker(subscriber, "subscriber", expected["subscriber"])

        pub_process = psutil.Process(publisher.proc.pid)
        sub_process = psutil.Process(subscriber.proc.pid)
        pub_process.cpu_percent(None)
        sub_process.cpu_percent(None)
        for process in (pub_process, sub_process):
            for child in process.children(recursive=True):
                child.cpu_percent(None)

        run_id = uuid.uuid4().hex
        subscriber.send_control("start", run_id)
        _wait_for(
            lambda: any(
                event.get("run_id") == run_id for event in subscriber.snapshot()["started"]),
            children,
            min(5.0, warmup_timeout),
            "measurement start acknowledgement",
        )

        pub_cpu = []
        sub_cpu = []
        interval = 1.0 / sample_hz
        deadline = time.monotonic() + duration
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(interval, remaining))
            for child in children:
                if not child.alive():
                    raise RuntimeError(
                        f"{child.name} exited during measurement (code {child.proc.returncode})\n"
                        f"--- output tail ---\n{child.tail_text()}")
            _sample_cpu(pub_process, pub_cpu)
            _sample_cpu(sub_process, sub_cpu)

        subscriber.send_control("stop", run_id)
        _wait_for(
            lambda: any(
                result.get("run_id") == run_id for result in subscriber.snapshot()["results"]),
            children,
            min(5.0, warmup_timeout),
            "measurement result",
        )
        snapshot = subscriber.snapshot()
        if snapshot["errors"]:
            raise RuntimeError(f"subscriber protocol error: {snapshot['errors'][-1]}")
        window = next(result for result in snapshot["results"] if result["run_id"] == run_id)
        if window["messages"]["received"] <= 0:
            raise RuntimeError("measurement window received no messages")

        return {
            "case_id": case["case_id"],
            "backend": case["backend"],
            "backend_label": case["backend_label"],
            "workload": case["workload"],
            "workload_label": case["workload_label"],
            "message_type": case["message_type"],
            "target_rate_hz": case["target_rate_hz"],
            "payload_bytes": case["payload_bytes"],
            "requested_duration_s": duration,
            "backend_verified": True,
            "expected_backends": expected,
            "publisher_backend": pub_backend,
            "subscriber_backend": sub_backend,
            "observed_window": window["window"],
            "messages": window["messages"],
            "latency_us": window["latency_us"],
            "cpu_pct": {
                "publisher": cpu_summary(pub_cpu),
                "subscriber": cpu_summary(sub_cpu),
            },
        }
    finally:
        for child in children:
            child.echo = False
            child.stop()


def _fmt(value, digits=1):
    return "-" if value is None else f"{value:.{digits}f}"


def print_table(results, mode):
    if mode == "smoke":
        print("\nSmoke validation only: metrics below are not performance claims.")
    else:
        print("\nBounded benchmark matrix")
    headings = ("backend", "workload", "Hz", "bytes", "recv", "eff Hz", "pub CPU", "sub CPU", "p99 us")
    widths = (23, 17, 8, 8, 9, 10, 10, 10, 10)
    print("  " + "".join(f"{heading:<{width}}" for heading, width in zip(headings, widths)))
    print("  " + "-" * sum(widths))
    for result in results:
        cells = (
            result["backend"],
            result["workload"],
            result["target_rate_hz"],
            result["payload_bytes"],
            result["messages"]["received"],
            _fmt(result["messages"]["effective_rate_hz"]),
            _fmt(result["cpu_pct"]["publisher"]["mean"]),
            _fmt(result["cpu_pct"]["subscriber"]["mean"]),
            _fmt(result["latency_us"]["p99"]),
        )
        print("  " + "".join(f"{str(cell):<{width}}" for cell, width in zip(cells, widths)))


def _selectors(args, parser):
    is_smoke = args.smoke or args.demo
    mode = "smoke" if is_smoke else "measurement"
    try:
        backends = parse_keys(
            args.backends, BACKENDS,
            SMOKE_BACKENDS if is_smoke else DEFAULT_BACKENDS, "backend")
        workloads = parse_keys(
            args.workloads, WORKLOADS,
            SMOKE_WORKLOADS if is_smoke else DEFAULT_WORKLOADS, "workload")
        rates = parse_int_values(
            args.rate, SMOKE_RATES_HZ if is_smoke else DEFAULT_RATES_HZ,
            "rate", minimum=1)
        payloads = parse_int_values(
            args.payload_bytes,
            SMOKE_PAYLOAD_BYTES if is_smoke else DEFAULT_PAYLOAD_BYTES,
            "payload bytes", minimum=0)
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    duration = args.duration if args.duration is not None else (2.0 if is_smoke else 15.0)
    if duration <= 0:
        parser.error("--duration must be positive")
    return mode, backends, workloads, rates, payloads, duration


def _parser():
    parser = argparse.ArgumentParser(
        description="Run the isolated rclcppyy backend/workload matrix.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--backends", "--variants", dest="backends",
        help=f"comma-separated backends; available: {', '.join(BACKENDS)}")
    parser.add_argument(
        "--workloads", help=f"comma-separated workloads; available: {', '.join(WORKLOADS)}")
    parser.add_argument("--rate", action="append", help="repeatable/comma-separated target Hz")
    parser.add_argument(
        "--payload-bytes", action="append",
        help="repeatable/comma-separated application padding bytes")
    parser.add_argument("--duration", type=float, help="measurement seconds per case")
    parser.add_argument("--warmup-timeout", type=float, default=60.0)
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--smoke", action="store_true", help="fast validation; forbids performance claims")
    parser.add_argument("--json", action="store_true", help="emit only versioned JSON on stdout")
    parser.add_argument("--output", type=Path, help="atomically write the versioned JSON document")
    parser.add_argument("--demo", action="store_true", help="echo child output for the first selected case")
    parser.add_argument("--list-matrix", action="store_true", help="print selected cases as JSON and exit")
    return parser


def main():
    parser = _parser()
    args = parser.parse_args()
    if args.sample_hz <= 0:
        parser.error("--sample-hz must be positive")
    mode, backends, workloads, rates, payloads, duration = _selectors(args, parser)
    cases = build_cases(backends, workloads, rates, payloads)
    if not cases:
        parser.error("selected matrix has no supported cases")

    if args.list_matrix:
        print(json.dumps(cases, indent=2, sort_keys=True))
        return 0

    log(
        f"Running {len(cases)} {mode} case(s): backends={backends}, workloads={workloads}, "
        f"rates={rates}, payload_bytes={payloads}, duration={duration:.2f}s")
    results = []
    failures = []
    for index, case in enumerate(cases, 1):
        log(f"[{index}/{len(cases)}] {case['case_id']}")
        try:
            results.append(run_case(
                case,
                duration,
                warmup_timeout=args.warmup_timeout,
                sample_hz=args.sample_hz,
                echo=args.demo and index == 1,
            ))
        except RuntimeError as exc:
            log(f"  FAILED: {exc}")
            failures.append({
                "case_id": case["case_id"],
                "backend": case["backend"],
                "workload": case["workload"],
                "target_rate_hz": case["target_rate_hz"],
                "payload_bytes": case["payload_bytes"],
                "error": str(exc).splitlines()[0],
            })

    matrix = {
        "backends": backends,
        "workloads": workloads,
        "target_rates_hz": rates,
        "payload_bytes": payloads,
        "duration_s": duration,
        "sample_hz": args.sample_hz,
        "warmup_timeout_s": args.warmup_timeout,
        "case_count": len(cases),
    }
    document = build_document(
        repo_root=REPO_ROOT,
        benchmark_name="pubsub_backend_workload_matrix",
        mode=mode,
        matrix=matrix,
        results=results,
        failures=failures,
    )
    if args.output is not None:
        write(document, args.output)
    if args.json:
        print(dumps(document), end="")
    else:
        print_table(results, mode)
        if failures:
            log(f"{len(failures)} case(s) failed")
        else:
            log("All cases completed with verified backend evidence.")
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("Interrupted; active child groups are being cleaned up.")
        raise SystemExit(130)
