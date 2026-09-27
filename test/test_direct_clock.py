"""Native-node-clock proofs for direct_cpp's Node.get_clock()."""

from rclpy.clock_type import ClockType
from rclpy.duration import Duration
from rclpy.time import Time

from rclcppyy.direct_clock import DirectClock, DirectROSClock
from rclcppyy.policy import BackendUnavailableError

from _run_helper import format_output, run_helper


class _FakeNativeNodeClock:
    """Mirrors NativeNodeClock's shape without touching any C++.

    Every accessor raises if called while closed, mirroring the suite's own
    ``test_closed_native_node_clock_fences_access_without_touching_cpp``: if
    DirectClock ever forwarded a call into a closed native clock instead of
    fencing it in Python first, this would trip instead of raising cleanly.
    """

    def __init__(self, clock_type=1):
        self._closed = False
        self._clock_type = clock_type

    @property
    def closed(self):
        return self._closed

    def close(self):
        was_open = not self._closed
        self._closed = True
        return was_open

    @property
    def clock_type(self):
        return self._clock_type

    @property
    def ros_time_is_active(self):
        if self._closed:
            raise AssertionError("touched native clock after close")
        return False

    def now_nanoseconds(self):
        if self._closed:
            raise AssertionError("touched native clock after close")
        return 12_000_000_345


def test_direct_clock_fences_closed_access_without_touching_native():
    fake = _FakeNativeNodeClock()
    clock = DirectClock._wrap(fake)
    assert isinstance(clock, DirectROSClock)

    first = clock.now()
    assert first.nanoseconds == 12_000_000_345
    assert clock.ros_time_is_active is False

    assert clock.close() is True
    assert clock.close() is False

    for operation in (clock.now, lambda: clock.ros_time_is_active):
        try:
            operation()
        except RuntimeError as exc:
            assert "closed" in str(exc)
        else:
            raise AssertionError("closed direct clock did not fence access")


def test_direct_clock_construction_and_fail_closed_surface_are_explicit():
    try:
        DirectClock(clock_type=1)
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("standalone DirectClock construction succeeded")

    fake = _FakeNativeNodeClock()
    clock = DirectClock._wrap(fake)
    for operation in (
        clock.set_ros_time_override,
        lambda: clock.handle,
    ):
        try:
            operation()
        except BackendUnavailableError:
            pass
        else:
            raise AssertionError("unsupported direct clock operation succeeded")


def test_direct_clock_sleep_fences_without_a_sleeper_provider():
    """An unbound clock (no ``sleeper_provider``) still fails sleep closed.

    ``DirectClock._wrap`` used bare -- as every other fast unit test in this
    file does -- never receives a ``sleeper_provider``; only
    ``Node.get_clock()`` does. Sleep must fence on that alone, without ever
    touching ``rclcppyy.direct_cpp``'s runtime (this test does not call
    ``rclpy.init()``), while ``now()``/``clock_type`` keep working exactly as
    the fencing test above already proves.
    """
    fake = _FakeNativeNodeClock()
    clock = DirectClock._wrap(fake)
    for operation in (
        lambda: clock.sleep_for(Duration(seconds=0.1)),
        lambda: clock.sleep_until(Time(nanoseconds=1, clock_type=ClockType.ROS_TIME)),
    ):
        try:
            operation()
        except BackendUnavailableError:
            pass
        else:
            raise AssertionError("unbound direct clock sleep succeeded")
    assert clock.now().nanoseconds == 12_000_000_345
    assert clock.clock_type == ClockType.ROS_TIME


def test_direct_clock_jump_callback_fences_without_a_jump_container_provider():
    """An unbound clock (no ``jump_container_provider``) fails jump-callback
    registration closed, mirroring the sleep-fencing test above -- only
    ``Node.get_clock()`` supplies a provider."""
    from rclpy.clock import JumpThreshold
    from rclpy.duration import Duration

    fake = _FakeNativeNodeClock()
    clock = DirectClock._wrap(fake)
    threshold = JumpThreshold(
        min_forward=Duration(nanoseconds=1), min_backward=None, on_clock_change=True)
    try:
        clock.create_jump_callback(threshold, post_callback=lambda time_jump: None)
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("unbound direct clock jump callback succeeded")


def test_direct_cpp_node_clock_matches_native_node_clock_exactly():
    process = run_helper("_direct_cpp_clock_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CLOCK_IDENTITY_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_STOCK_DIFFERENTIAL_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_NATIVE_IDENTITY_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TEARDOWN_OK" in process.stdout


def test_direct_cpp_clock_sleep_over_native_sleeper():
    process = run_helper("_direct_cpp_clock_sleep_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CLOCK_SLEEP_FOR_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_SLEEP_NATIVE_IDENTITY_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_SLEEP_SIM_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_SLEEP_INTERRUPT_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_SLEEP_NOTINIT_OK" in process.stdout
