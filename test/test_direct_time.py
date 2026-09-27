"""Time/Duration builtin_interfaces payload proof for direct_cpp."""

from _run_helper import format_output, run_helper


def test_direct_cpp_time_and_duration_keep_generated_cpp_payloads():
    process = run_helper("_direct_cpp_time_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_TIME_TO_MSG_OK" in process.stdout
    assert "DIRECT_CPP_TIME_FROM_MSG_OK" in process.stdout
    assert "DIRECT_CPP_DURATION_MSG_ROUNDTRIP_OK" in process.stdout
    assert "DIRECT_CPP_TIME_HEADER_FIELD_OK" in process.stdout
    assert "DIRECT_CPP_TIME_NO_CONVERSION_OK" in process.stdout
