#!/usr/bin/env python3
"""create_task/Task/Future driving proof for the direct single-threaded executor.

Covers Commit 2 of the executor slice (``docs/plans/PLAN-executor-slice.md``):
plain-callable and coroutine tasks, exception propagation with stock-parity
re-raise, cancellation before first drive, and ``spin_until_future_complete``
ending on a task-set future. A separate stock-differential section runs the
identical coroutines under a genuine stock ``SingleThreadedExecutor`` (its own
independent, directly-initialized ``rclpy.context.Context`` -- never touching
the direct_cpp native runtime) and asserts identical result/exception
outcomes.

One scenario is deliberately direct-only, not part of the differential: a
task cancelled before its first drive. Stock rclpy 7.1.11 crashes with a
``KeyError`` in ``Executor._wait_for_ready_callbacks`` in this exact scenario
(its ``_pending_tasks``/``_ready_tasks`` bookkeeping goes stale the moment a
still-queued task is cancelled out from under it -- reproducible with zero
direct_cpp involvement). The direct facade carries no such dict, so it has no
equivalent failure mode; this is a genuine, pre-existing stock limitation,
not a bug this lane owns or an accelerated-parity claim.
"""

import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_cpp import _PATCHES  # noqa: E402


_BY_NAME = {
    name: (original, replacement) for _module, name, original, replacement in _PATCHES
}


def _drive_until(executor, predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "did not complete before the deadline"


rclpy.init(args=[])
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

executor = SingleThreadedExecutor()


# --- plain-callable task completes with its return value ------------------
def add_one(x):
    return x + 1


plain_task = executor.create_task(add_one, 41)
_drive_until(executor, plain_task.done)
assert plain_task.result() == 42
assert plain_task.exception() is None
print("DIRECT_CPP_EXECUTOR_TASKS_PLAIN_OK", flush=True)


# --- coroutine task that awaits a Future the test resolves -----------------
awaited_future = executor.create_future()


async def doubles_the_awaited_value():
    value = await awaited_future
    return value * 2


coroutine_task = executor.create_task(doubles_the_awaited_value)
executor.spin_once(timeout_sec=0.2)
assert not coroutine_task.done(), "coroutine should be suspended on the future"
awaited_future.set_result(10)
_drive_until(executor, coroutine_task.done)
assert coroutine_task.result() == 20
print("DIRECT_CPP_EXECUTOR_TASKS_COROUTINE_AWAIT_OK", flush=True)


# --- a task that raises: future.exception()/result() stock parity --------
def boom():
    raise ValueError("task-boom")


raising_task = executor.create_task(boom)
try:
    executor.spin_once(timeout_sec=1.0)
except ValueError as exc:
    assert str(exc) == "task-boom"
else:
    raise AssertionError("expected spin_once to re-raise the task exception")
assert raising_task.done()
assert not raising_task.cancelled()
assert isinstance(raising_task.exception(), ValueError)
try:
    raising_task.result()
except ValueError:
    pass
else:
    raise AssertionError("expected task.result() to re-raise")
print("DIRECT_CPP_EXECUTOR_TASKS_RAISE_OK", flush=True)


# --- task.cancel() before it runs -> cancelled() (direct-only, see module
# docstring for why this is not part of the stock differential) -----------
cancel_task = executor.create_task(add_one, 1)
cancel_task.cancel()
for _ in range(3):
    executor.spin_once(timeout_sec=0.1)
assert cancel_task.cancelled()
assert not cancel_task.done()
print("DIRECT_CPP_EXECUTOR_TASKS_CANCEL_OK", flush=True)


# --- spin_until_future_complete(future) ends when a task sets it ----------
task_set_future = executor.create_future()


def resolve_it():
    task_set_future.set_result("resolved-by-task")


executor.create_task(resolve_it)
executor.spin_until_future_complete(task_set_future, timeout_sec=10.0)
assert task_set_future.done()
assert task_set_future.result() == "resolved-by-task"
print("DIRECT_CPP_EXECUTOR_TASKS_SPIN_UNTIL_FUTURE_COMPLETE_OK", flush=True)

executor.shutdown(timeout_sec=2.0)


# --- stock differential: identical coroutines, identical outcomes --------
# Stays under the same still-active direct_cpp runtime (a fresh executor
# instance) rather than cycling rclpy.init()/shutdown() a second time; the
# stock side uses its own independently-initialized Context and never
# touches the direct native session at all.
def coroutine_outcomes(make_executor):
    executor = make_executor()

    async def succeed():
        return 42 * 2

    async def fail():
        raise KeyError("differential-boom")

    ok_task = executor.create_task(succeed)
    _drive_until(executor, ok_task.done)

    fail_task = executor.create_task(fail)
    try:
        executor.spin_once(timeout_sec=1.0)
        raised = None
    except KeyError as exc:
        raised = exc
    _drive_until(executor, fail_task.done)

    future = executor.create_future()

    async def waits():
        value = await future
        return value + 1

    wait_task = executor.create_task(waits)
    executor.spin_once(timeout_sec=0.2)
    suspended = not wait_task.done()
    future.set_result(9)
    _drive_until(executor, wait_task.done)

    executor.shutdown(timeout_sec=2.0)
    return {
        "ok_result": ok_task.result(),
        "spin_once_raised": type(raised).__name__ if raised is not None else None,
        "fail_exception_type": type(fail_task.exception()).__name__,
        "fail_exception_args": fail_task.exception().args,
        "wait_suspended_before_resolve": suspended,
        "wait_result": wait_task.result(),
    }


direct_outcomes = coroutine_outcomes(SingleThreadedExecutor)

stock_single_threaded_executor, _direct_single_threaded_executor = _BY_NAME[
    "SingleThreadedExecutor"
]
stock_context = rclpy.context.Context()
stock_context.init(args=[])
stock_outcomes = coroutine_outcomes(
    lambda: stock_single_threaded_executor(context=stock_context))
stock_context.shutdown()

assert direct_outcomes == stock_outcomes, (direct_outcomes, stock_outcomes)
print("DIRECT_CPP_EXECUTOR_TASKS_STOCK_DIFFERENTIAL_OK", flush=True)

rclpy.shutdown()
assert not rclpy.ok()
