"""Declarative backends and workloads for the bounded benchmark matrix."""

from __future__ import annotations

import itertools
import re


BACKENDS = {
    "rclpy": {
        "label": "stock rclpy",
        "worker_backend": "stock",
        "expected_backends": {"publisher": "python", "subscriber": "python"},
        "workloads": ("small-string", "nested-header"),
    },
    "rclcppyy": {
        "label": "rclcppyy compatibility",
        "worker_backend": "compatibility",
        "expected_backends": {"publisher": "python", "subscriber": "python"},
        "workloads": ("small-string", "nested-header"),
    },
    "rclcppyy-templated": {
        "label": "direct rclcpp via cppyy",
        "worker_backend": "native",
        "expected_backends": {"publisher": "cpp", "subscriber": "cpp"},
        "workloads": ("small-string", "nested-header"),
    },
    "rclcppyy-direct-copy": {
        "label": "source-compatible direct C++ with callback copy",
        "worker_backend": "direct-copy",
        "expected_backends": {"publisher": "cpp", "subscriber": "cpp"},
        "workloads": ("small-string", "nested-header"),
    },
    "rclcppyy-direct-lease": {
        "label": "source-compatible direct C++ with shared callback lease",
        "worker_backend": "direct-lease",
        "expected_backends": {"publisher": "cpp", "subscriber": "cpp"},
        "workloads": ("small-string", "nested-header"),
    },
}

WORKLOADS = {
    "small-string": {
        "label": "std_msgs/String",
        "message_type": "std_msgs/msg/String",
        "shape": "flat",
        "wire_contract": "std_msgs/String:sequence-timestamp-padding/v1",
    },
    "nested-header": {
        "label": "std_msgs/Header",
        "message_type": "std_msgs/msg/Header",
        "shape": "nested",
        "wire_contract": "std_msgs/Header:stamp-frame-sequence-padding/v1",
    },
}

DEFAULT_BACKENDS = ("rclpy", "rclcppyy")
DEFAULT_WORKLOADS = ("small-string", "nested-header")
DEFAULT_RATES_HZ = (1000, 10000)
DEFAULT_PAYLOAD_BYTES = (0,)

SMOKE_BACKENDS = ("rclcppyy",)
SMOKE_WORKLOADS = ("small-string",)
SMOKE_RATES_HZ = (1000,)
SMOKE_PAYLOAD_BYTES = (0,)


def parse_keys(value, definitions, defaults, option_name):
    """Parse a comma-separated key selector with stable de-duplication."""
    if value is None:
        return list(defaults)
    selected = []
    for part in value.split(","):
        key = part.strip()
        if not key:
            continue
        if key not in definitions:
            raise ValueError(
                f"unknown {option_name} {key!r}; choose from: {', '.join(definitions)}")
        if key not in selected:
            selected.append(key)
    if not selected:
        raise ValueError(f"at least one {option_name} is required")
    return selected


def parse_int_values(values, defaults, option_name, minimum=0):
    """Parse repeatable/comma-separated integer selectors."""
    if not values:
        return list(defaults)
    selected = []
    for chunk in values:
        for part in str(chunk).split(","):
            part = part.strip()
            if not part:
                continue
            value = int(part)
            if value < minimum:
                raise ValueError(f"{option_name} must be >= {minimum}: {value}")
            if value not in selected:
                selected.append(value)
    if not selected:
        raise ValueError(f"at least one {option_name} is required")
    return selected


def _case_id(backend, workload, rate_hz, payload_bytes):
    raw = f"{backend}__{workload}__{rate_hz}hz__{payload_bytes}b"
    return re.sub(r"[^a-zA-Z0-9_]", "_", raw)


def build_cases(backends, workloads, rates_hz, payload_bytes_values, *, run_token=None):
    """Return the supported deterministic matrix cross-product."""
    cases = []
    for backend, workload, rate_hz, payload_bytes in itertools.product(
            backends, workloads, rates_hz, payload_bytes_values):
        backend_spec = BACKENDS[backend]
        if workload not in backend_spec["workloads"]:
            continue
        case_id = _case_id(backend, workload, rate_hz, payload_bytes)
        cases.append({
            "case_id": case_id,
            "backend": backend,
            "backend_label": backend_spec["label"],
            "worker_backend": backend_spec["worker_backend"],
            "expected_backends": dict(backend_spec["expected_backends"]),
            "workload": workload,
            "workload_label": WORKLOADS[workload]["label"],
            "message_type": WORKLOADS[workload]["message_type"],
            "wire_contract": WORKLOADS[workload]["wire_contract"],
            "target_rate_hz": rate_hz,
            "payload_bytes": payload_bytes,
            "topic": "/rclcppyy_bench/%s%s" % (
                (str(run_token) + "/") if run_token else "", case_id),
        })
    return cases
