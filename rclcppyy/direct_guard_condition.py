"""rclpy-shaped ``GuardCondition`` facade over a native ``rclcpp::GuardCondition``.

``rclcpp_kit.native_waitset`` exposes exactly ``trigger()``/``close()`` on a
guard condition and ``add_guard_condition()``/``wait()`` on a wait set --
``rclcpp::GuardCondition::set_on_trigger_callback`` exists on the underlying
C++ class but the suite primitive does not wrap it, and adding that wrapper
is out of this package's scope (product-only, no suite changes). Dispatch is
therefore built entirely on the product side: each ``DirectGuardCondition``
owns a private background thread blocking on a two-entry ``NativeWaitSet`` --
the guard condition itself, plus a second, internal guard condition used only
to unblock the wait immediately on ``close()`` (the primitive exposes no way
to remove an entity from a wait set or to cancel a blocking ``wait()`` any
other way). A trigger from any thread is therefore observed with no polling
latency, and shutdown does not need to wait out a poll interval either.
"""

from __future__ import annotations

import threading
from typing import Any, Callable

from rclcppyy.policy import BackendUnavailableError


def _unsupported(reason: str) -> None:
    raise BackendUnavailableError(reason)


# Bound on how long DirectGuardCondition.destroy() waits for an in-progress
# dispatch on this guard's own waiter thread to finish before failing loud
# instead of freeing a resource that thread might still be using --
# leak-safe beats crash-safe, mirroring direct_cpp._DESTROY_QUIESCENCE_TIMEOUT_SEC
# (not imported directly: this module is loaded from inside DirectNode method
# bodies, well after direct_cpp itself, but keeps its own small constant
# rather than reaching back into that module for one number).
_WAITER_JOIN_TIMEOUT_SEC = 10.0


class _GuardConditionWaiter:
    """Own the private thread that turns a native trigger into a dispatch.

    Built with two already-created native guard conditions on the same
    session: ``native_guard_condition`` (the one this facade wraps) and
    ``stop_guard`` (private, never exposed, triggered only by ``close()`` to
    unblock a thread parked in ``wait()``).
    """

    def __init__(
        self, native_guard_condition: Any, stop_guard: Any, wait_set: Any,
        dispatch: Callable[[], None],
    ) -> None:
        self._stop_guard = stop_guard
        self._wait_set = wait_set
        self._dispatch = dispatch
        self._stop_requested = False
        self._thread = threading.Thread(
            target=self._run, name="rclcppyy-guard-condition", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_requested:
            try:
                kind = self._wait_set.wait(-1)
            except RuntimeError:
                # The wait set (or the session context backing it) was
                # closed out from under this thread -- nothing left to wait
                # on.
                return
            if self._stop_requested:
                return
            if kind == "ready":
                self._dispatch()

    def close(self) -> None:
        self._stop_requested = True
        if threading.current_thread() is self._thread:
            # A self-destroy: this guard condition's own dispatched callback
            # called destroy() on it, so this call is running on the waiter
            # thread itself, which cannot join itself. The loop above
            # rechecks ``_stop_requested`` the instant this call stack
            # unwinds back to it and exits without blocking on wait() again,
            # so closing the native resources here directly (without
            # joining) is safe -- nothing will touch them from this thread
            # afterward.
            self._stop_guard.close()
            self._wait_set.close()
            return
        self._stop_guard.trigger()
        self._thread.join(_WAITER_JOIN_TIMEOUT_SEC)
        if self._thread.is_alive():
            raise RuntimeError(
                "direct_cpp guard condition dispatch still running after "
                "%.1fs; refusing to free it (leak-safe beats crash-safe)"
                % _WAITER_JOIN_TIMEOUT_SEC
            )
        self._stop_guard.close()
        self._wait_set.close()


class DirectGuardCondition:
    """A stock-``GuardCondition``-shaped facade over one native guard condition.

    Built only by ``DirectNode.create_guard_condition``, which already wraps
    ``callback`` through the owning node's ``_contain_callback_exceptions``
    before handing it here as ``contained_callback`` -- exactly as
    ``create_timer``/``create_subscription`` do for their own callbacks, so
    this facade needs no separate node reference of its own. ``session`` is
    the active ``NativeSession``, used only to create this guard condition's
    private stop-guard and wait set.
    """

    def __init__(
        self, callback: Any, callback_group: Any, native_guard_condition: Any,
        session: Any, contained_callback: Callable[[], None],
    ) -> None:
        self._native = native_guard_condition
        self.callback = callback
        self.callback_group = callback_group
        self._closed = False
        stop_guard = session.create_native_guard_condition()
        wait_set = session.create_native_wait_set()
        wait_set.add_guard_condition(native_guard_condition)
        wait_set.add_guard_condition(stop_guard)
        # A blocking native wait must not monopolize the interpreter --
        # cppyy holds the GIL for the duration of a C++ call unless told
        # otherwise, so an unmarked wait(-1) on the waiter thread below
        # would never let the GIL go and would hang the whole process
        # (verified empirically). rclcpp_kit.native_waitset.NativeWaitSet
        # does not mark its own wait_kind() this way, so this reaches
        # through its ``_implementation`` to the raw cppyy-bound method,
        # exactly like direct_executors.py's
        # ``self._native.spin_once.__release_gil__ = True`` does for the
        # native executor's own blocking wait.
        wait_set._implementation.wait_kind.__release_gil__ = True
        self._waiter = _GuardConditionWaiter(
            native_guard_condition, stop_guard, wait_set, contained_callback)

    def trigger(self) -> None:
        if self._closed:
            raise RuntimeError("direct_cpp guard condition is destroyed")
        self._native.trigger()

    @property
    def handle(self) -> Any:
        _unsupported(
            "direct_cpp guard conditions do not expose a stock rclpy handle")

    @property
    def closed(self) -> bool:
        return self._closed

    def destroy(self) -> bool:
        if self._closed:
            return False
        self._waiter.close()
        self._native.close()
        self._closed = True
        return True


__all__ = ["DirectGuardCondition"]
