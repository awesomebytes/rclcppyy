"""Executor/callback-group public-surface parity proofs (wave-3 Lane 2).

See ``docs/plans/PLAN-executor-slice.md`` for the commit sequence this file
tracks. Commit 1 covers structural surface parity only; later commits extend
this file with task/Future driving and MultiThreadedExecutor concurrency.
"""

from _run_helper import format_output, run_helper


def test_direct_cpp_executor_surface_matches_stock_signatures():
    process = run_helper("_direct_cpp_executor_surface_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_EXECUTOR_SURFACE_CALLBACK_GROUP_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_SURFACE_BASE_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_SURFACE_SINGLE_THREADED_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_SURFACE_MULTI_THREADED_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_SURFACE_CAN_EXECUTE_OK" in process.stdout
    assert (
        "DIRECT_CPP_EXECUTOR_SURFACE_WAIT_FOR_READY_FAIL_CLOSED_OK"
        in process.stdout
    )
    assert (
        "DIRECT_CPP_EXECUTOR_SURFACE_MULTI_THREADED_FAIL_CLOSED_OK"
        in process.stdout
    )
    assert "DIRECT_CPP_EXECUTOR_SURFACE_TEARDOWN_OK" in process.stdout
