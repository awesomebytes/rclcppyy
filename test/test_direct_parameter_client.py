"""Focused unchanged AsyncParameterClient proof over direct C++ values."""

from _run_helper import format_output, run_helper


def test_async_parameter_client_reuses_stock_source_with_cpp_values():
    process = run_helper("_direct_cpp_parameter_client_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert (
        "DIRECT_CPP_PARAMETER_CLIENT_STOCK_SOURCE_CPP_ALIASES_OK"
        in process.stdout
    )
    assert "DIRECT_CPP_PARAMETER_CLIENT_READINESS_OK" in process.stdout
    assert (
        "DIRECT_CPP_PARAMETER_CLIENT_SET_GET_TYPES_DESCRIBE_LIST_OK"
        in process.stdout
    )
    assert "DIRECT_CPP_PARAMETER_CLIENT_ATOMIC_DELETE_EVENTS_OK" in process.stdout
    assert (
        "DIRECT_CPP_PARAMETER_CLIENT_NO_CONVERSION_TEARDOWN_OK"
        in process.stdout
    )


def test_async_parameter_client_stale_import_fails_closed():
    process = run_helper(
        "_direct_cpp_parameter_client_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert (
        "DIRECT_CPP_PARAMETER_CLIENT_STALE_IMPORT_REJECTED_OK"
        in process.stdout
    )
