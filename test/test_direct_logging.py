"""Direct logging compatibility and native ownership proofs."""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_LOGGING_PROBE "
ROSOUT_PREFIX = "RCLCPPYY_LOGGING_ROSOUT_PROBE "


def _probe(backend):
    process = run_helper(
        "_direct_logging_probe.py", "--backend", backend, timeout=240)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def _rosout_probe(backend, enabled):
    process = run_helper(
        "_direct_logging_rosout_probe.py",
        "--backend", backend,
        "--enable-rosout", "yes" if enabled else "no",
        timeout=240,
    )
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(ROSOUT_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(ROSOUT_PREFIX):])


def test_direct_node_logger_matches_stock_rcutils_contract_without_boundaries():
    stock = _probe("stock")
    direct = _probe("direct")

    assert stock["contract"] == direct["contract"]
    assert direct["contract"]["is_exact_rcutils_logger"] is True
    assert direct["contract"]["same_logger_identity"] is True
    assert direct["contract"]["retained_after_destroy"]["same_identity"] is True
    assert direct["boundary"] == {
        "conversion_forbidden": True,
        "serialization_forbidden": True,
    }


def test_direct_native_rosout_matches_stock_with_external_stock_subscriber():
    stock = _rosout_probe("stock", True)
    direct = _rosout_probe("direct", True)

    assert stock["result"] == direct["result"]
    assert len(direct["result"]["records"]) == 4
    assert direct["result"]["server_publisher_removed"] is True
    assert direct["boundary"] == {
        "conversion_forbidden": True,
        "serialization_forbidden": True,
    }


def test_direct_disabled_rosout_matches_stock_without_publishing():
    stock = _rosout_probe("stock", False)
    direct = _rosout_probe("direct", False)

    assert stock["result"] == direct["result"]
    assert direct["result"]["records"] == []
    assert direct["result"]["observed_before_teardown"] == []
    assert direct["result"]["server_publisher_removed"] is True
