"""Machine-readable backend evidence emitted by benchmark children."""

from __future__ import annotations

import json


PREFIX = "RCLCPPYY_BENCH_BACKEND "
SCHEMA = "rclcppyy.benchmark-backend/v1"


def _emit(role, backend, evidence, **metadata):
    if role not in ("publisher", "subscriber"):
        raise ValueError("benchmark role must be publisher or subscriber")
    if backend not in ("python", "cpp"):
        raise ValueError("benchmark backend must be python or cpp")
    document = {
        "schema": SCHEMA,
        "role": role,
        "backend": backend,
        "evidence": evidence,
        "metadata": metadata,
    }
    print(PREFIX + json.dumps(document, sort_keys=True), flush=True)


def emit_stock_backend(role, entity):
    """Report a stock Python entity using its concrete runtime type as evidence."""
    entity_type = type(entity)
    _emit(
        role,
        "python",
        "stock_rclpy_entity",
        entity_type=f"{entity_type.__module__}.{entity_type.__qualname__}",
    )


def emit_native_backend(role, entity):
    """Report a directly constructed cppyy entity."""
    _emit(role, "cpp", "direct_cppyy_entity", entity_type=str(type(entity)))


def emit_status_backend(role, entity_type):
    """Report the latest matching decision from ``rclcppyy.status()``.

    Required-C++ benchmarks deliberately fail here when instrumentation is absent;
    a child cannot claim acceleration merely because activation was requested.
    """
    import rclcppyy

    snapshot = rclcppyy.status()
    matching = [
        record
        for record in snapshot["entities"]
        if record["metadata"].get("entity_type") == entity_type
    ]
    if not matching:
        raise RuntimeError(f"no backend decision recorded for {entity_type}")
    decision = matching[-1]
    if role == "publisher" and decision["backend"] == "cpp":
        completed = [
            record
            for record in snapshot["operations"]
            if record["metadata"].get("operation") == "publish"
        ]
        if not completed:
            raise RuntimeError("no completed publish backend decision recorded")
        decision = completed[-1]
    _emit(
        role,
        decision["backend"],
        "rclcppyy_status",
        entity_type=entity_type,
        decision_id=decision["id"],
        reason=decision["reason"],
        policies=decision["policies"],
    )


def emit_direct_status_backend(role, entity_type):
    """Report a direct entity without requiring a wrapped publish operation."""
    import rclcppyy

    matching = [
        record for record in rclcppyy.status()["entities"]
        if record["metadata"].get("entity_type") == entity_type
    ]
    if not matching:
        raise RuntimeError(f"no direct backend decision recorded for {entity_type}")
    decision = matching[-1]
    if decision["backend"] != "cpp" or "no_conversion" not in decision["policies"]:
        raise RuntimeError(f"{entity_type} did not select the strict direct C++ route")
    _emit(
        role,
        "cpp",
        "rclcppyy_direct_status",
        entity_type=entity_type,
        decision_id=decision["id"],
        reason=decision["reason"],
        policies=decision["policies"],
        decision_metadata=decision["metadata"],
    )
