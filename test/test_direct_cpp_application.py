"""Unchanged application-shaped scenarios on direct_cpp authority."""

from _run_helper import format_output, run_helper


def test_direct_cpp_unchanged_application_authority_and_conversion_boundary():
    process = run_helper("_direct_cpp_application_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_APPLICATION_AUTHORITY_OK" in process.stdout
    assert "DIRECT_CPP_APPLICATION_NO_CONVERSION_OK" in process.stdout
    assert "DIRECT_CPP_APPLICATION_TEARDOWN_OK" in process.stdout
