"""Line protocol and exact-window statistics shared by benchmark processes."""

from __future__ import annotations

import json
import math
import statistics
import threading
import time


CONTROL_SCHEMA = "rclcppyy.benchmark-control/v1"
EVENT_SCHEMA = "rclcppyy.benchmark-event/v1"
RESULT_SCHEMA = "rclcppyy.benchmark-window/v1"
WIRE_SCHEMA = "rclcppyy.benchmark-wire-values/v1"

READY_PREFIX = "RCLCPPYY_BENCH_READY "
STARTED_PREFIX = "RCLCPPYY_BENCH_STARTED "
RESULT_PREFIX = "RCLCPPYY_BENCH_RESULT "


def emit(prefix, document):
    print(prefix + json.dumps(document, sort_keys=True, allow_nan=False), flush=True)


def event_document(event, **fields):
    return {"schema": EVENT_SCHEMA, "event": event, **fields}


def control_document(command, run_id):
    return {"schema": CONTROL_SCHEMA, "command": command, "run_id": run_id}


def validate_control(document):
    if not isinstance(document, dict):
        raise ValueError("benchmark control must be an object")
    if document.get("schema") != CONTROL_SCHEMA:
        raise ValueError("invalid benchmark control schema")
    if document.get("command") not in ("start", "stop"):
        raise ValueError("invalid benchmark control command")
    if not isinstance(document.get("run_id"), str) or not document["run_id"]:
        raise ValueError("benchmark control requires run_id")


def validate_event(document, expected_event=None):
    if not isinstance(document, dict):
        raise ValueError("benchmark event must be an object")
    if document.get("schema") != EVENT_SCHEMA:
        raise ValueError("invalid benchmark event schema")
    if document.get("event") not in ("ready", "started"):
        raise ValueError("invalid benchmark event")
    if expected_event is not None and document.get("event") != expected_event:
        raise ValueError(f"expected benchmark event {expected_event}")
    if document["event"] == "ready" and document.get("role") != "subscriber":
        raise ValueError("ready event must identify the subscriber")
    if document["event"] == "started":
        if not isinstance(document.get("run_id"), str) or not document["run_id"]:
            raise ValueError("started event requires run_id")


def validate_window_result(document):
    if not isinstance(document, dict):
        raise ValueError("benchmark window result must be an object")
    if document.get("schema") != RESULT_SCHEMA:
        raise ValueError("invalid benchmark window result schema")
    if not isinstance(document.get("run_id"), str) or not document["run_id"]:
        raise ValueError("benchmark window result requires run_id")
    messages = document.get("messages")
    latency = document.get("latency_us")
    window = document.get("window")
    wire_values = document.get("wire_values")
    if not isinstance(window, dict) or not _is_nonnegative_number(window.get("duration_s")):
        raise ValueError("benchmark window result requires duration")
    if (not isinstance(messages, dict) or
            not _is_nonnegative_int(messages.get("received")) or
            not _is_nonnegative_int(messages.get("dropped")) or
            not _is_nonnegative_number(messages.get("effective_rate_hz"))):
        raise ValueError("benchmark window result requires message counts")
    if not isinstance(latency, dict) or latency.get("count") != messages["received"]:
        raise ValueError("latency count must equal received messages")
    for field in ("mean", "p50", "p95", "p99", "min", "max"):
        value = latency.get(field)
        if messages["received"] == 0:
            if value is not None:
                raise ValueError("empty latency summaries must use null values")
        elif not _is_nonnegative_number(value):
            raise ValueError(f"latency {field} must be a non-negative finite number")
    if not isinstance(wire_values, dict) or wire_values.get("schema") != WIRE_SCHEMA:
        raise ValueError("benchmark window result requires wire-value evidence")
    if not isinstance(wire_values.get("contract_id"), str) or not wire_values["contract_id"]:
        raise ValueError("wire-value evidence requires a contract id")
    if not _is_nonnegative_int(wire_values.get("expected_payload_bytes")):
        raise ValueError("wire-value evidence requires expected payload bytes")
    if wire_values.get("checked_messages") != messages["received"]:
        raise ValueError("wire-value checked messages must equal received messages")
    if not _is_nonnegative_int(wire_values.get("violations")):
        raise ValueError("wire-value violations must be a non-negative integer")
    if not isinstance(wire_values.get("violation_types"), dict):
        raise ValueError("wire-value violation types must be an object")
    if not all(
            isinstance(name, str) and name and _is_nonnegative_int(count)
            for name, count in wire_values["violation_types"].items()):
        raise ValueError("wire-value violation types require non-negative integer counts")
    if sum(wire_values["violation_types"].values()) != wire_values["violations"]:
        raise ValueError("wire-value violation counts do not match")
    if wire_values.get("value_contract_verified") is not (
            wire_values["violations"] == 0 and messages["received"] > 0):
        raise ValueError("wire-value verification flag contradicts observations")


def _is_nonnegative_int(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _is_nonnegative_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(value) and value >= 0)


def nearest_rank(values, percentile):
    """Return the nearest-rank percentile, with its definition kept stable."""
    if not values:
        return None
    ordered = sorted(values)
    rank = max(1, math.ceil((percentile / 100.0) * len(ordered)))
    return ordered[rank - 1]


def latency_summary(values):
    if not values:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "p50": nearest_rank(values, 50),
        "p95": nearest_rank(values, 95),
        "p99": nearest_rank(values, 99),
        "min": min(values),
        "max": max(values),
    }


class MeasurementState:
    """Thread-safe subscriber state controlled by explicit start/stop commands."""

    def __init__(
            self, wire_contract, expected_payload_bytes, clock_ns=time.monotonic_ns):
        if not isinstance(wire_contract, str) or not wire_contract:
            raise ValueError("wire_contract must be a non-empty string")
        if not _is_nonnegative_int(expected_payload_bytes):
            raise ValueError("expected_payload_bytes must be a non-negative integer")
        self._clock_ns = clock_ns
        self._wire_contract = wire_contract
        self._expected_payload_bytes = expected_payload_bytes
        self._lock = threading.Lock()
        self._active = False
        self._run_id = None
        self._started_ns = None
        self._last_sequence = None
        self._received = 0
        self._dropped = 0
        self._latencies = []
        self._wire_checked = 0
        self._wire_violations = {}

    def observe(self, sequence, latency_us):
        with self._lock:
            previous = self._last_sequence
            self._last_sequence = sequence
            if not self._active:
                return
            if previous is not None and sequence <= previous:
                name = "non_monotonic_sequence"
                self._wire_violations[name] = self._wire_violations.get(name, 0) + 1
            if previous is not None and sequence > previous + 1:
                self._dropped += sequence - previous - 1
            self._received += 1
            self._latencies.append(float(latency_us))
            self._wire_checked += 1

    def reject_wire_value(self, error_type):
        """Record a callback whose decoded value violated the workload contract."""
        with self._lock:
            if not self._active:
                return
            name = str(error_type) or "unknown"
            self._wire_violations[name] = self._wire_violations.get(name, 0) + 1

    def start(self, run_id):
        with self._lock:
            if self._active:
                raise RuntimeError("measurement window already active")
            self._active = True
            self._run_id = run_id
            self._started_ns = self._clock_ns()
            self._received = 0
            self._dropped = 0
            self._latencies = []
            self._wire_checked = 0
            self._wire_violations = {}

    def stop(self, run_id):
        with self._lock:
            if not self._active or run_id != self._run_id:
                raise RuntimeError("measurement window is not active for this run_id")
            stopped_ns = self._clock_ns()
            duration_s = max(0.0, (stopped_ns - self._started_ns) / 1e9)
            received = self._received
            dropped = self._dropped
            latencies = list(self._latencies)
            wire_checked = self._wire_checked
            wire_violations = dict(sorted(self._wire_violations.items()))
            self._active = False

        result = {
            "schema": RESULT_SCHEMA,
            "run_id": run_id,
            "window": {"duration_s": duration_s},
            "messages": {
                "received": received,
                "dropped": dropped,
                "effective_rate_hz": received / duration_s if duration_s > 0 else 0.0,
            },
            "latency_us": latency_summary(latencies),
            "wire_values": {
                "schema": WIRE_SCHEMA,
                "contract_id": self._wire_contract,
                "expected_payload_bytes": self._expected_payload_bytes,
                "checked_messages": wire_checked,
                "violations": sum(wire_violations.values()),
                "violation_types": wire_violations,
                "value_contract_verified": not wire_violations and received > 0,
            },
        }
        validate_window_result(result)
        return result
