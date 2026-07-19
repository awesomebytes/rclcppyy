"""Stock-authoritative differential for the direct ``wait_for_message`` slice."""

import json

from _run_helper import format_output, run_helper


REPORT_PREFIX = "DIRECT_WAIT_FOR_MESSAGE_REPORT="


def _run(backend):
    process = run_helper(
        "_direct_wait_for_message_probe.py",
        "--backend", backend,
        timeout=240,
    )
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(REPORT_PREFIX):])


def test_wait_for_message_matches_stock_and_returns_exact_cpp_message():
    stock = _run("stock")
    direct = _run("direct")

    assert direct["behavior"] == stock["behavior"] == {
        "default": "default-qos",
        "default_subscription_cleaned": True,
        "explicit": "explicit-qos",
        "explicit_subscription_cleaned": True,
        "timeout": [False, None],
        "zero_timeout": [False, None],
    }
    assert direct["public_contract"] == stock["public_contract"]
    assert direct["public_contract"]["module"] == "rclpy.wait_for_message"
    assert direct["public_contract"]["name"] == "wait_for_message"
    assert direct["public_contract"]["signature_preserved"] is True
    assert direct["public_contract"]["installed_source_preserved"] is True
    assert direct["exact_cpp_message"] is True
    assert stock["exact_cpp_message"] is False
    assert direct["boundary_calls"] == {
        "cdr": 0,
        "conversion": 0,
        "serialization": 0,
    }
    assert direct["recursive_guard"] == {
        "exception": "BackendUnavailableError",
        "failed_before_subscription": True,
    }
    assert stock["recursive_guard"] is None
    assert direct["retained_after_teardown"] == stock[
        "retained_after_teardown"] == ["default-qos", "explicit-qos"]
    assert direct["runtime_released"] is True
