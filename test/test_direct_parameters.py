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


PAYLOAD_TAINTED_METHOD_NAMES = (
    "declare_parameter",
    "declare_parameters",
    "describe_parameter",
    "describe_parameters",
    "list_parameters",
    "add_on_set_parameters_callback",
    "remove_on_set_parameters_callback",
    "set_parameters",
    "set_parameters_atomically",
    "set_descriptor",
    "get_parameters_by_prefix",
)


def _signature_report(backend):
    process = run_helper(
        "_direct_parameter_signature_probe.py", "--backend", backend, timeout=180)
    assert process.returncode == 0, format_output(process)
    rows = [
        line for line in process.stdout.splitlines()
        if line.startswith("DIRECT_PARAMETER_SIGNATURE_REPORT=")
    ]
    assert len(rows) == 1, format_output(process)
    return json.loads(rows[0][len("DIRECT_PARAMETER_SIGNATURE_REPORT="):])


def test_direct_cpp_payload_tainted_parameter_signatures_match_stock():
    """Wave 7 slice 3 (PLAN-wave7.md §4): the 11 parameter/descriptor methods
    whose stock signature references a message type this package rebinds to
    cppyy get a hand-built ``__signature__`` (rclcppyy._payload_signature),
    since ``_signature_mirror`` correctly refuses a payload-tainted stock
    signature. This proves DirectNode's own, hand-built signature matches a
    genuinely pristine, non-activated stock reference exactly, and that the
    taint being sidestepped is real, not hypothetical: an activated
    process's own in-process view of the ORIGINAL stock ``Node`` class
    (before the ``Node`` rebind) does show a ``cppyy.gbl`` marker, for every
    method but ``get_parameters_by_prefix`` (whose own declared hint is
    plain ``typing`` generics, never a message type)."""
    stock = _signature_report("stock")
    direct = _signature_report("direct")

    assert set(stock["signatures"]) == set(PAYLOAD_TAINTED_METHOD_NAMES)
    assert set(direct["signatures"]) == set(PAYLOAD_TAINTED_METHOD_NAMES)

    for name in PAYLOAD_TAINTED_METHOD_NAMES:
        direct_signature = direct["signatures"][name]
        assert "cppyy" not in direct_signature, (name, direct_signature)
        assert direct_signature == stock["signatures"][name], name

    tainted_stock = direct["tainted_stock_signatures"]
    assert set(tainted_stock) == set(PAYLOAD_TAINTED_METHOD_NAMES)
    tainted = [
        name for name in PAYLOAD_TAINTED_METHOD_NAMES
        if "cppyy" in tainted_stock[name]
    ]
    assert set(tainted) == set(PAYLOAD_TAINTED_METHOD_NAMES) - {
        "get_parameters_by_prefix"}


def _by_prefix_report(backend):
    process = run_helper(
        "_direct_parameters_by_prefix_probe.py", "--backend", backend, timeout=180)
    assert process.returncode == 0, format_output(process)
    rows = [
        line for line in process.stdout.splitlines()
        if line.startswith("DIRECT_PARAMETERS_BY_PREFIX_REPORT=")
    ]
    assert len(rows) == 1, format_output(process)
    return json.loads(rows[0][len("DIRECT_PARAMETERS_BY_PREFIX_REPORT="):])


def test_direct_cpp_get_parameters_by_prefix_matches_stock_naive_behavior():
    """Wave 7 slice 3 (PLAN-wave7.md §4.3): get_parameters_by_prefix is new
    on DirectNode this slice. Stock's own prefix rule is documented as naive
    (a trailing separator is always appended before matching, so a prefix
    that already ends in the separator produces a doubled separator) -- this
    proves DirectNode replicates that exactly, including the double-
    separator edge case, not just the common case."""
    stock = _by_prefix_report("stock")
    direct = _by_prefix_report("direct")

    assert direct["all_values_are_parameter_instances"] is True
    assert stock["all_values_are_parameter_instances"] is True

    declared_names = {"foo.ping", "foo..oddname", "bar.baz", "standalone"}
    for prefix in ("foo.", "foo", "nope"):
        assert direct["results"][prefix] == stock["results"][prefix], prefix

    # The "" (match-everything) case also picks up each backend's own
    # default-declared node parameters (use_sim_time, direct_cpp's
    # qos_overrides.* additions, ...), which legitimately differ between
    # backends and are out of this slice's scope -- only the 4 parameters
    # this probe itself declared need to agree.
    stock_all = stock["results"][""]
    direct_all = direct["results"][""]
    for suffix in declared_names:
        assert stock_all[suffix] == direct_all[suffix], suffix
