"""Dynamic undeclare and descriptor-mutation boundary proof."""

from _run_helper import format_output, run_helper


def test_direct_dynamic_undeclare_preserves_cpp_values_and_rclpy_errors():
    process = run_helper("_direct_parameter_undeclare_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_PARAMETER_DYNAMIC_UNDECLARE_OK" in process.stdout
    assert "DIRECT_PARAMETER_UNDECLARE_ERRORS_OK" in process.stdout
    assert "DIRECT_PARAMETER_DESCRIPTOR_MUTATION_LIMIT_OK" in process.stdout
    assert "DIRECT_PARAMETER_UNDECLARE_RETAINED_EXACT_CPP_OK" in process.stdout
