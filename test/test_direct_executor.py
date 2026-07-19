"""Executor/callback-group public-surface parity proofs (wave-3 Lane 2).

See ``docs/plans/PLAN-executor-slice.md`` for the commit sequence this file
tracks. Commit 1 covers structural surface parity; Commit 2 adds create_task/
Task/coroutine driving. Later commits extend this file with
MultiThreadedExecutor concurrency and callback-group proofs under live
dispatch.
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


def test_direct_cpp_executor_create_task_drives_tasks_and_futures():
    process = run_helper("_direct_cpp_executor_tasks_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_EXECUTOR_TASKS_PLAIN_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_TASKS_COROUTINE_AWAIT_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_TASKS_RAISE_OK" in process.stdout
    assert "DIRECT_CPP_EXECUTOR_TASKS_CANCEL_OK" in process.stdout
    assert (
        "DIRECT_CPP_EXECUTOR_TASKS_SPIN_UNTIL_FUTURE_COMPLETE_OK"
        in process.stdout
    )
    assert "DIRECT_CPP_EXECUTOR_TASKS_STOCK_DIFFERENTIAL_OK" in process.stdout


def test_direct_cpp_callback_group_subclass_with_overridden_hook_fails_closed():
    process = run_helper(
        "_direct_cpp_callback_group_hook_override_helper.py", timeout=120)
    assert process.returncode == 0, format_output(process)
    assert (
        "DIRECT_CPP_CALLBACK_GROUP_HOOK_OVERRIDE_HARMLESS_OK"
        in process.stdout
    )
    assert (
        "DIRECT_CPP_CALLBACK_GROUP_HOOK_OVERRIDE_REJECTED_OK"
        in process.stdout
    )
    assert (
        "DIRECT_CPP_CALLBACK_GROUP_HOOK_OVERRIDE_TEARDOWN_OK"
        in process.stdout
    )


def test_direct_cpp_multi_threaded_executor_dispatch_correctness_wake_and_teardown():
    process = run_helper(
        "_direct_cpp_multi_threaded_executor_helper.py", timeout=120)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_MTE_EXCLUSIVE_CORRECTNESS_OK" in process.stdout
    assert "DIRECT_CPP_MTE_CROSS_GROUP_CORRECTNESS_OK" in process.stdout
    assert "DIRECT_CPP_MTE_REENTRANT_ADMISSION_OK" in process.stdout
    assert "DIRECT_CPP_MTE_WAKE_OK" in process.stdout
    assert "DIRECT_CPP_MTE_TEARDOWN_OK" in process.stdout
    assert "DIRECT_CPP_MTE_ALL_OK" in process.stdout


def test_direct_cpp_multi_threaded_executor_callback_exception_currently_aborts_process():
    """Documents the CURRENT process-abort behavior for a raising Python
    callback under real ``MultiThreadedExecutor`` dispatch -- the reason
    public construction is fail-closed this wave (see
    ``rclcppyy.direct_executors._MULTI_THREADED_FAIL_CLOSED_REASON`` and the
    queued callback-exception containment task). If this ever starts
    passing with a clean exit, that is a behavior change (for better --
    contained -- or worse -- silent) that must be investigated and this
    test updated, not silently left broken.
    """
    process = run_helper(
        "_direct_cpp_multi_threaded_executor_exception_crash_helper.py",
        timeout=60,
    )
    assert "MTE_EXCEPTION_CRASH_READY" in process.stdout, format_output(process)
    assert "MTE_EXCEPTION_CRASH_DID_NOT_ABORT" not in process.stdout, (
        format_output(process)
    )
    assert process.returncode != 0, format_output(process)
