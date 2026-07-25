"""Native-node-backed rclpy ``Clock`` facade for the ``direct_cpp`` profile.

``DirectNode.get_clock()`` is the only supported construction path: the
facade wraps the exact ``NativeNodeClock`` the native foundation retains for
a node's ``rclcpp::Clock`` (one ``rclcpp::TimeSource`` owns sim-time
natively; there is no product Python ``TimeSource``). Standalone
``Clock(...)`` construction, the raw ``handle``, jump callbacks, and
``set_ros_time_override`` all fail closed with a typed error -- none of them
is half-built. ``sleep_for``/``sleep_until`` are the one accelerated
exception: a node-bound clock runs them over the node's native
``NativeClockSleeper`` (ROS-time-aware, context-interruptible); a clock built
without a ``sleeper_provider`` (e.g. ``DirectClock._wrap`` used bare in a
fast unit test) still fails the same sleep calls closed.

``DirectNode.create_rate()`` is the only supported construction path for
``DirectRate``, a fixed-rate sleeper built over that same node sleeper. The
``rclpy.timer.Rate`` symbol itself is not patched -- a directly-constructed
stock ``Rate`` keeps running unmodified stock code -- exactly as
``rclpy.clock.Clock``/``ROSClock`` are not patched above.
"""

from __future__ import annotations

from typing import Any

import rclpy.clock as _stock_clock
import rclpy.timer as _stock_timer
from rclpy.clock_type import ClockType

from rclcppyy.policy import BackendUnavailableError


def _unsupported(reason: str) -> None:
    raise BackendUnavailableError(reason)


class DirectClock(_stock_clock.Clock):
    """A stock-``Clock``-shaped facade retaining one node's native clock.

    Never construct this directly; only ``wrap_node_clock`` (used by
    ``DirectNode.get_clock()``) produces instances, since the facade has no
    clock of its own to create -- it always retains an existing node's.
    """

    def __new__(cls, *args: Any, **kwargs: Any) -> "DirectClock":
        _unsupported(
            "direct_cpp clocks come from Node.get_clock(); standalone "
            "Clock(...) construction is not supported")

    @classmethod
    def _wrap(
        cls, native_node_clock: Any, sleeper_provider: Any = None,
    ) -> "DirectClock":
        clock_type = ClockType(int(native_node_clock.clock_type))
        target = DirectROSClock if clock_type is ClockType.ROS_TIME else cls
        self = object.__new__(target)
        self._native_node_clock = native_node_clock
        self._clock_type = clock_type
        self._sleeper_provider = sleeper_provider
        return self

    def _require_native(self) -> Any:
        native = self._native_node_clock
        if native.closed:
            raise RuntimeError("direct_cpp clock is closed")
        return native

    @property
    def clock_type(self) -> ClockType:
        return self._clock_type

    @property
    def handle(self) -> Any:
        _unsupported(
            "direct_cpp clocks do not expose a stock rclpy Clock handle")

    def now(self):
        from rclpy.time import Time

        native = self._require_native()
        return Time(
            nanoseconds=native.now_nanoseconds(), clock_type=self._clock_type)

    def create_jump_callback(self, *args: Any, **kwargs: Any) -> Any:
        _unsupported("direct_cpp clocks do not support jump callbacks")

    def _raw_native_clock(self) -> Any:
        """Private seam: the raw, shared-pointer-backed ``rclcpp::Clock``
        this facade retains, for cross-node handoff -- e.g.
        ``Node.create_timer(clock=other_node.get_clock())`` threading one
        node's clock into another node's timer."""
        return self._require_native().raw_clock

    def _require_sleeper_provider(self) -> None:
        if self._sleeper_provider is None:
            _unsupported(
                "direct_cpp clock sleep requires a node-bound clock "
                "(built by Node.get_clock())")

    def _resolve_sleep_context(self, context: Any) -> Any:
        """Validate ``context`` against the single active session context.

        ``None`` resolves to the active context (the one the node's
        sleeper is bound to); a foreign, non-active context fails closed --
        direct_cpp has exactly one context per process, so there is no
        second context to honor. Raises stock's own
        ``NotInitializedException`` when the active context is not ok,
        matching ``rclpy.clock.Clock.sleep_for/sleep_until``.
        """
        from rclcppyy.direct_cpp import _runtime

        active = _runtime().context
        if context is not None and context is not active:
            _unsupported(
                "direct_cpp clock sleep only supports the active session "
                "context")
        if not active.ok():
            from rclpy.exceptions import NotInitializedException

            raise NotInitializedException()
        return active

    def sleep_until(self, until: Any, context: Any = None) -> bool:
        self._require_sleeper_provider()
        self._resolve_sleep_context(context)
        if until.clock_type != self._clock_type:
            raise ValueError(
                "until's clock type does not match this clock's type")
        return bool(self._sleeper_provider().sleep_until(int(until.nanoseconds)))

    def sleep_for(self, rel_time: Any, context: Any = None) -> bool:
        self._require_sleeper_provider()
        self._resolve_sleep_context(context)
        return bool(self._sleeper_provider().sleep_for(int(rel_time.nanoseconds)))

    def close(self) -> bool:
        return self._native_node_clock.close()

    def __repr__(self) -> str:
        return "DirectClock(clock_type={0})".format(self._clock_type.name)


class DirectROSClock(DirectClock, _stock_clock.ROSClock):
    """The only shape a direct_cpp node clock takes (node clocks are ROS_TIME)."""

    @property
    def ros_time_is_active(self) -> bool:
        return bool(self._require_native().ros_time_is_active)

    def set_ros_time_override(self, *args: Any, **kwargs: Any) -> None:
        _unsupported(
            "direct_cpp clocks do not support set_ros_time_override; ROS "
            "time is driven natively by use_sim_time + /clock")


def wrap_node_clock(
    native_node_clock: Any, sleeper_provider: Any = None,
) -> DirectClock:
    """Build the rclpy-shaped facade that ``DirectNode.get_clock()`` returns.

    ``sleeper_provider``, when given, is a zero-argument callable returning
    the node's ``NativeClockSleeper`` on demand -- passed in rather than a
    node reference so the facade never holds a hard reference to the node.
    """
    return DirectClock._wrap(native_node_clock, sleeper_provider=sleeper_provider)


class DirectRate(_stock_timer.Rate):
    """A stock-``Rate``-shaped fixed-rate sleeper over the node's sleeper.

    Built only by ``DirectNode.create_rate()``. Stock's ``Rate`` wraps a
    ``Timer`` whose callback sets a ``threading.Event``, so ``sleep()``
    blocks forever without a spinning executor; this facade instead sleeps
    directly on the node's ``NativeClockSleeper`` with its own fixed-rate
    bookkeeping (the ``rclcpp::Rate`` algorithm), so it sleeps correctly with
    or without one -- a documented behavioral superset, never a deficit.
    """

    def __init__(self, sleeper: Any, clock: Any, period_ns: int, context: Any) -> None:
        # Deliberately does not call super().__init__: stock's Rate.__init__
        # requires a stock Timer, and this facade owns none.
        self._sleeper = sleeper
        self._clock = clock
        self._period_ns = period_ns
        self._context = context
        self._next_ns = clock.now().nanoseconds + period_ns
        self._destroyed = False

    def sleep(self) -> None:
        from rclpy.exceptions import ROSInterruptException

        if self._destroyed:
            raise RuntimeError("Rate cannot sleep because it has been destroyed")
        if not self._context.ok():
            raise ROSInterruptException()
        self._sleeper.sleep_until(self._next_ns)
        self._next_ns += self._period_ns
        now_ns = self._clock.now().nanoseconds
        if now_ns > self._next_ns + self._period_ns:
            # rclcpp::Rate's catch-up guard: after a long stall, resync to
            # now + one period instead of bursting zero-length sleeps to
            # close the gap against the original schedule.
            self._next_ns = now_ns + self._period_ns
        if not self._context.ok():
            raise ROSInterruptException()

    def destroy(self) -> None:
        self._destroyed = True


__all__ = ["DirectClock", "DirectROSClock", "DirectRate", "wrap_node_clock"]
