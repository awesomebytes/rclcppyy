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


def test_direct_cpp_multi_threaded_executor_callback_exception_contained_and_reraised():
    """Proves the defect-A fix (docs/plans/PLAN-mte-unlock.md Slice 2): a
    raising Python callback under real ``MultiThreadedExecutor`` dispatch no
    longer crosses into C++ off a native worker thread and aborts the
    process. It is instead captured into the owning node's exception sink
    at the product hand-off (``create_subscription`` et al. in
    ``direct_cpp.py``) and re-raised, with its exact type and message, on
    the executor's own spin thread -- while a non-raising peer callback
    keeps dispatching in the same run. This test previously documented the
    opposite (a process abort); see its own prior revision if that history
    is needed.
    """
    process = run_helper(
        "_direct_cpp_multi_threaded_executor_exception_crash_helper.py",
        timeout=60,
    )
    assert "MTE_EXCEPTION_CRASH_READY" in process.stdout, format_output(process)
    assert (
        "MTE_EXCEPTION_CRASH_CONTAINED_AND_RERAISED" in process.stdout
    ), format_output(process)
    assert "MTE_EXCEPTION_CRASH_DID_NOT_ABORT" in process.stdout, (
        format_output(process)
    )
    assert process.returncode == 0, format_output(process)


def test_direct_cpp_single_threaded_executor_callback_exception_still_propagates():
    """Regression guard for the defect-A containment shim (Slice 2 test
    plan): a raising subscription callback under ``SingleThreadedExecutor``
    must still propagate out of ``spin_once()`` with its exact type and
    message, unchanged from before the shim existed -- see
    ``_direct_cpp_single_threaded_executor_exception_propagation_helper.py``.
    """
    process = run_helper(
        "_direct_cpp_single_threaded_executor_exception_propagation_helper.py",
        timeout=60,
    )
    assert (
        "SINGLE_THREADED_EXCEPTION_PROPAGATION_READY" in process.stdout
    ), format_output(process)
    assert (
        "SINGLE_THREADED_EXCEPTION_PROPAGATION_OK" in process.stdout
    ), format_output(process)
    assert process.returncode == 0, format_output(process)
