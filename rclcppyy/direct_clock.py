"""Native-node-backed rclpy ``Clock`` facade for the ``direct_cpp`` profile.

``DirectNode.get_clock()`` is the only supported construction path: the
facade wraps the exact ``NativeNodeClock`` the native foundation retains for
a node's ``rclcpp::Clock`` (one ``rclcpp::TimeSource`` owns sim-time
natively; there is no product Python ``TimeSource``). Standalone
``Clock(...)`` construction, the raw ``handle``, jump callbacks, ``sleep_for``
/``sleep_until``, and ``set_ros_time_override`` all fail closed with a typed
error -- none of them is half-built.
"""

from __future__ import annotations

from typing import Any

import rclpy.clock as _stock_clock
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
    def _wrap(cls, native_node_clock: Any) -> "DirectClock":
        clock_type = ClockType(int(native_node_clock.clock_type))
        target = DirectROSClock if clock_type is ClockType.ROS_TIME else cls
        self = object.__new__(target)
        self._native_node_clock = native_node_clock
        self._clock_type = clock_type
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

    def sleep_until(self, *args: Any, **kwargs: Any) -> bool:
        _unsupported("direct_cpp clocks do not support sleep_until")

    def sleep_for(self, *args: Any, **kwargs: Any) -> bool:
        _unsupported("direct_cpp clocks do not support sleep_for")

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


def wrap_node_clock(native_node_clock: Any) -> DirectClock:
    """Build the rclpy-shaped facade that ``DirectNode.get_clock()`` returns."""
    return DirectClock._wrap(native_node_clock)


__all__ = ["DirectClock", "DirectROSClock", "wrap_node_clock"]
