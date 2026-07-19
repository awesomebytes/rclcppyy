"""Focused native DirectNode constructor option proof."""

from _run_helper import format_output, run_helper


def test_direct_node_options_lower_to_native_without_conversion():
    process = run_helper("_direct_cpp_node_options_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_NODE_OPTIONS_FAIL_BEFORE_MUTATION_OK" in process.stdout
    assert "DIRECT_CPP_NODE_OPTIONS_LOCAL_NATIVE_OK" in process.stdout
    assert "DIRECT_CPP_NODE_OPTIONS_DEFERRED_OVERRIDE_OK" in process.stdout
    assert "DIRECT_CPP_NODE_OPTIONS_GLOBAL_NATIVE_OK" in process.stdout
    assert "DIRECT_CPP_NODE_OPTIONS_NO_CONVERSION_TEARDOWN_OK" in process.stdout
