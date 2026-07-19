"""use_sim_time + /clock activation proof through the native node clock."""

from _run_helper import format_output, run_helper


def test_direct_cpp_sim_time_activation_matches_stock():
    process = run_helper("_direct_cpp_sim_time_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_SIM_TIME_ACTIVATION_OK" in process.stdout
    assert "DIRECT_CPP_SIM_TIME_SOURCE_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_SIM_TIME_DEACTIVATION_OK" in process.stdout
    assert "DIRECT_CPP_SIM_TIME_TEARDOWN_OK" in process.stdout
    assert "DIRECT_CPP_SIM_TIME_STOCK_DIFFERENTIAL_OK" in process.stdout
