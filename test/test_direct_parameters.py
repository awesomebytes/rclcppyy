"""Native parameter facade and node-authority integration proof."""

import json
from pathlib import Path

from _run_helper import format_output, run_helper


REPORT_PREFIX = "DIRECT_PARAMETER_GET_REPORT="
DIRECT_CPP_PATH = Path(__file__).resolve().parents[1] / "rclcppyy" / "direct_cpp.py"


def _report(process):
    assert process.returncode == 0, format_output(process)
    rows = [
        line for line in process.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(rows) == 1, format_output(process)
    return json.loads(rows[0][len(REPORT_PREFIX):])


def test_direct_cpp_parameters_remain_native_without_app_conversion():
    process = run_helper("_direct_cpp_parameters_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_PARAMETERS_OWNING_FACADE_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_NATIVE_NODE_API_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_CHECKED_GET_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_CALLBACKS_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_RETAINED_REINIT_OK" in process.stdout


def test_direct_cpp_checked_get_matches_stock_parameter_behavior():
    stock = _report(run_helper(
        "_direct_parameter_get_probe.py", "--backend", "stock", timeout=180))
    direct = _report(run_helper(
        "_direct_parameter_get_probe.py", "--backend", "direct", timeout=180))

    assert direct["behavior"] == stock["behavior"]
    assert stock["exact_cpp_parameter"] is False
    assert direct["exact_cpp_parameter"] is True
    assert stock["retained_after_teardown"] is True
    assert direct["retained_after_teardown"] is True
    assert direct["forbidden_boundary_calls"] == 0


def test_direct_cpp_get_uses_one_compiled_node_query():
    source = DIRECT_CPP_PATH.read_text(encoding="utf-8")
    get_source = source[
        source.index("    def get_parameter(self, name):"):
        source.index("    def get_parameters(self, names):")
    ]
    assert get_source.count("get_parameter_checked(") == 1
    assert "self.has_parameter(" not in get_source
    assert "get_parameter_types(" not in get_source
    assert "describe_parameters(" not in get_source
