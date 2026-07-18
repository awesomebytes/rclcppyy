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

    matching = [
        record
        for record in rclcppyy.status()["entities"]
        if record["metadata"].get("entity_type") == entity_type
    ]
    if not matching:
        raise RuntimeError(f"no backend decision recorded for {entity_type}")
    decision = matching[-1]
    _emit(
        role,
        decision["backend"],
        "rclcppyy_status",
        entity_type=entity_type,
        decision_id=decision["id"],
        reason=decision["reason"],
        policies=decision["policies"],
    )
