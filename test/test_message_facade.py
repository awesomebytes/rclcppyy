"""Differential and failure evidence for the opt-in C++ message facade."""

import json
import os

import pytest

from _run_helper import format_output, run_helper


def _observations(process):
    for line in process.stdout.splitlines():
        if line.startswith("MESSAGE_FACADE_OBSERVATIONS="):
            return json.loads(line.split("=", 1)[1])
    raise AssertionError("missing observations%s" % format_output(process))


@pytest.mark.parametrize("executor", ("single", "multi"))
def test_facade_matches_stock_observations_and_proves_cpp_routes(executor):
    stock = run_helper(
        "_message_facade_product_helper.py", "stock", executor, timeout=180)
    facade = run_helper(
        "_message_facade_product_helper.py", "facade", executor, timeout=180)
    assert stock.returncode == 0, format_output(stock)
    assert facade.returncode == 0, format_output(facade)
    assert _observations(facade) == _observations(stock)
    assert "MESSAGE_FACADE_CPP_ROUTES_OK" in facade.stdout
    assert "MESSAGE_FACADE_FALLBACKS_OK" in facade.stdout
    assert "MESSAGE_FACADE_TEARDOWN_OK" in facade.stdout


def test_facade_runtime_gate_fails_before_patching_or_replacing_classes(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    process = run_helper("_message_facade_gate_helper.py")
    assert process.returncode == 0, format_output(process)
    assert "MESSAGE_FACADE_GATE_OK" in process.stdout


def test_pre_activation_message_import_stays_stock_and_visible():
    process = run_helper("_message_facade_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "MESSAGE_FACADE_STALE_IMPORT_OK" in process.stdout


def test_profile_is_jazzy_cyclone_only():
    assert os.environ.get("ROS_DISTRO") == "jazzy"
