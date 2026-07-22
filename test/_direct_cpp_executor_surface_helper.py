#!/usr/bin/env python3
"""Structural-parity proof for the executor/callback-group surface fixes.

This pins Commit 1 of the executor slice: the 18 executor + 1 callback-group
`missing_mismatch` ledger rows a boundary-local edit can clear (contract in
`docs/plans/PLAN-executor-slice.md` Sec. 3), re-derived here ahead of a full
ledger regen. It also proves the two runtime-behavior invariants this commit
holds steady: `can_execute` delegates to the real callback-group contract, and
`MultiThreadedExecutor` (Slice 3, PLAN-mte-unlock.md) publicly constructs and
dispatches -- the real concurrent-dispatch machinery this was proven under a
test-only construction guard for (see
`_direct_cpp_multi_threaded_executor_helper.py`) is reachable through the
public constructor now that the containment (Slice 2) and zero-dispatch
(Slice 1) fixes hold and the destroy-under-dispatch battery (Slice 2.5)
proves the self-destroy/teardown paths.
"""

import inspect
import threading

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_cpp import _PATCHES  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.callback_groups import (  # noqa: E402
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import (  # noqa: E402
    Executor,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node  # noqa: E402


_BY_NAME = {
    name: (original, replacement) for _module, name, original, replacement in _PATCHES
}


def _static(cls, name):
    return inspect.getattr_static(cls, name)


def assert_own_member(cls, name):
    """The member is owned by `cls` itself, not merely inherited."""
    assert name in vars(cls), "%s.%s: not defined on the class itself" % (
        cls.__qualname__, name)


def assert_matches_stock(direct_cls, stock_cls, name):
    direct_signature = inspect.signature(_static(direct_cls, name))
    stock_signature = inspect.signature(_static(stock_cls, name))
    assert direct_signature == stock_signature, (
        "%s.%s: direct=%s stock=%s"
        % (direct_cls.__qualname__, name, direct_signature, stock_signature)
    )


# --- Row 1: ReentrantCallbackGroup.__init__ (callback_groups.py) ----------
stock_reentrant, direct_reentrant = _BY_NAME["ReentrantCallbackGroup"]
assert direct_reentrant is ReentrantCallbackGroup
assert_own_member(direct_reentrant, "__init__")
assert_matches_stock(direct_reentrant, stock_reentrant, "__init__")
ReentrantCallbackGroup()  # constructs with no arguments, matching stock
print("DIRECT_CPP_EXECUTOR_SURFACE_CALLBACK_GROUP_OK", flush=True)


# --- Rows 2-6: Executor base (direct_executors.py) ------------------------
stock_executor, direct_executor = _BY_NAME["Executor"]
assert direct_executor is Executor
for name in ("__init__", "__exit__", "can_execute", "wait_for_ready_callbacks"):
    assert_matches_stock(direct_executor, stock_executor, name)
assert "context" in inspect.signature(direct_executor.__init__).parameters
assert "num_threads" not in inspect.signature(direct_executor.__init__).parameters
print("DIRECT_CPP_EXECUTOR_SURFACE_BASE_OK", flush=True)


# --- Rows 7-12: SingleThreadedExecutor (direct_executors.py) --------------
stock_single, direct_single = _BY_NAME["SingleThreadedExecutor"]
assert direct_single is SingleThreadedExecutor
for name in ("__init__", "__enter__", "__exit__"):
    assert_own_member(direct_single, name)
    assert_matches_stock(direct_single, stock_single, name)
# can_execute/wait_for_ready_callbacks are inherited from the base on both
# sides (stock from Executor, direct from DirectExecutor) -- still equal.
for name in ("can_execute", "wait_for_ready_callbacks"):
    assert_matches_stock(direct_single, stock_single, name)
assert inspect.signature(direct_single) == inspect.signature(stock_single)
print("DIRECT_CPP_EXECUTOR_SURFACE_SINGLE_THREADED_OK", flush=True)


# --- Rows 13-18: MultiThreadedExecutor (direct_executors.py) --------------
stock_multi, direct_multi = _BY_NAME["MultiThreadedExecutor"]
assert direct_multi is MultiThreadedExecutor
for name in ("__init__", "__enter__", "__exit__"):
    assert_own_member(direct_multi, name)
    assert_matches_stock(direct_multi, stock_multi, name)
for name in ("can_execute", "wait_for_ready_callbacks"):
    assert_matches_stock(direct_multi, stock_multi, name)
assert inspect.signature(direct_multi) == inspect.signature(stock_multi)
print("DIRECT_CPP_EXECUTOR_SURFACE_MULTI_THREADED_OK", flush=True)


# --- can_execute truth table: delegates to the real callback-group contract
rclpy.init(args=[])
node = Node("direct_executor_surface_%d" % __import__("os").getpid())
executor = SingleThreadedExecutor(context=node.context)
assert executor.add_node(node)

exclusive = MutuallyExclusiveCallbackGroup()
reentrant = ReentrantCallbackGroup()
timer_a = node.create_timer(1.0, lambda: None, callback_group=exclusive)
timer_b = node.create_timer(1.0, lambda: None, callback_group=exclusive)
timer_c = node.create_timer(1.0, lambda: None, callback_group=reentrant)

assert executor.can_execute(timer_a) is True
assert executor.can_execute(timer_b) is True
assert exclusive.beginning_execution(timer_a) is True
assert executor.can_execute(timer_a) is False  # active_entity is timer_a
assert executor.can_execute(timer_b) is False  # exclusive group is busy
exclusive.ending_execution(timer_a)
assert executor.can_execute(timer_b) is True

assert executor.can_execute(timer_c) is True
assert reentrant.beginning_execution(timer_c) is True
assert executor.can_execute(timer_c) is True  # reentrant never blocks
reentrant.ending_execution(timer_c)

node.destroy_timer(timer_a)
node.destroy_timer(timer_b)
node.destroy_timer(timer_c)
print("DIRECT_CPP_EXECUTOR_SURFACE_CAN_EXECUTE_OK", flush=True)


# --- wait_for_ready_callbacks: fail-closed, no Python-level wait set ------
try:
    executor.wait_for_ready_callbacks()
except BackendUnavailableError:
    pass
else:
    raise AssertionError(
        "direct_cpp executor accepted wait_for_ready_callbacks()")
print("DIRECT_CPP_EXECUTOR_SURFACE_WAIT_FOR_READY_FAIL_CLOSED_OK", flush=True)


# --- MultiThreadedExecutor: public construction constructs and dispatches
# (Slice 3, PLAN-mte-unlock.md): the full concurrent-dispatch battery lives
# in _direct_cpp_multi_threaded_executor_helper.py and the Slice 2.5c
# destroy-under-dispatch tests; this is just the surface-level smoke check
# that the public constructor itself works and a dispatched callback fires.
for kwargs in ({"num_threads": 2}, {}):
    mte_node = Node(
        "direct_executor_surface_mte_%d" % __import__("os").getpid())
    mte = MultiThreadedExecutor(context=node.context, **kwargs)
    mte.add_node(mte_node)
    fired = threading.Event()
    timer = mte_node.create_timer(0.01, fired.set)
    spin_errors = []

    def _spin_target():
        try:
            mte.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            spin_errors.append(exc)

    spin_thread = threading.Thread(target=_spin_target)
    spin_thread.start()
    assert fired.wait(timeout=5.0), "MultiThreadedExecutor never dispatched"
    mte_node.destroy_timer(timer)
    assert mte.shutdown(timeout_sec=5.0) is True
    spin_thread.join(timeout=5.0)
    assert not spin_thread.is_alive()
    assert spin_errors == []
    mte_node.destroy_node()
print("DIRECT_CPP_EXECUTOR_SURFACE_MULTI_THREADED_CONSTRUCTS_AND_DISPATCHES_OK",
      flush=True)


executor.remove_node(node)
node.destroy_node()
assert executor.shutdown(timeout_sec=2.0)
rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_EXECUTOR_SURFACE_TEARDOWN_OK", flush=True)
