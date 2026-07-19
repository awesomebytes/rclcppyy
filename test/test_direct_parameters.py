"""Native parameter facade and node-authority integration proof."""

from _run_helper import format_output, run_helper


def test_direct_cpp_parameters_remain_native_without_app_conversion():
    process = run_helper("_direct_cpp_parameters_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_PARAMETERS_OWNING_FACADE_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_NATIVE_NODE_API_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_CALLBACKS_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_PARAMETERS_RETAINED_REINIT_OK" in process.stdout
