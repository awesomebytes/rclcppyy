"""Live and fast proofs for direct_cpp's Node.create_rate()/DirectRate."""

from rclcppyy.direct_clock import DirectRate

from _run_helper import format_output, run_helper


class _FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _FakeClock:
    """Just enough of DirectClock's shape for DirectRate.__init__'s anchor.

    A destroyed rate's sleep() never touches the sleeper/clock/context
    (the destroyed check comes first), so this fake only needs to satisfy
    construction, not sleep() itself -- there is no cppyy/rclpy.init()
    dependency here at all, matching this file's fast in-process unit style.
    """

    def now(self):
        return _FakeTime(0)


def test_direct_rate_destroy_fences_sleep():
    rate = DirectRate(None, _FakeClock(), 100_000_000, None)
    rate.destroy()
    try:
        rate.sleep()
    except RuntimeError as exc:
        assert "destroyed" in str(exc)
    else:
        raise AssertionError("destroyed direct rate did not fence sleep")


def test_direct_cpp_rate_over_native_sleeper():
    process = run_helper("_direct_cpp_rate_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_RATE_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_RATE_ISINSTANCE_OK" in process.stdout
    assert "DIRECT_CPP_RATE_PERIOD_OK" in process.stdout
    assert "DIRECT_CPP_RATE_DESTROY_OK" in process.stdout
    assert "DIRECT_CPP_RATE_SIM_OK" in process.stdout
    assert "DIRECT_CPP_RATE_SHUTDOWN_OK" in process.stdout
