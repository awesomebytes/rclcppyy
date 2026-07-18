"""Optimized-profile coverage for bounded stock executor waits."""

import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

import pytest

from rclcppyy import monkey


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "test" / "_optimized_signal_worker.py"


class _FakeContext:
    def __init__(self, valid=True):
        self.valid = valid

    def ok(self):
        return self.valid


class _FakeExecutor:
    def __init__(self, exception=None):
        self._context = _FakeContext()
        self._is_shutdown = False
        self.exception = exception
        self.entered = 0
        self.exited = 0
        self.timeouts = []

    def _enter_spin(self):
        self.entered += 1

    def _exit_spin(self):
        self.exited += 1

    def _spin_once_impl(self, timeout_sec):
        self.timeouts.append(timeout_sec)
        if self.exception is not None:
            raise self.exception
        self._context.valid = False


class _MarkerError(RuntimeError):
    pass


def test_bounded_executor_spin_preserves_exception_and_spin_state():
    error = _MarkerError("callback failed")
    executor = _FakeExecutor(exception=error)

    with pytest.raises(_MarkerError) as caught:
        monkey._bounded_executor_spin(executor)

    assert caught.value is error
    assert executor.entered == 1
    assert executor.exited == 1
    assert executor.timeouts == [0.1]


def test_bounded_executor_spin_exits_after_context_shutdown():
    executor = _FakeExecutor()

    assert monkey._bounded_executor_spin(executor) is None

    assert executor.entered == 1
    assert executor.exited == 1
    assert executor.timeouts == [0.1]


@pytest.mark.parametrize("profile", ("compatible", "required_cpp"))
def test_nonoptimized_profiles_leave_direct_single_threaded_spin_unchanged(profile):
    code = """
import rclcppyy
from rclpy.executors import SingleThreadedExecutor
stock_spin = SingleThreadedExecutor.spin
rclcppyy.enable_cpp_acceleration(profile=%r)
assert SingleThreadedExecutor.spin is stock_spin
print('NONOPTIMIZED_SINGLE_SPIN_UNCHANGED')
""" % profile
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=20.0,
        check=False,
    )
    diagnostics = result.stdout + "\n" + result.stderr
    assert result.returncode == 0, diagnostics
    assert "NONOPTIMIZED_SINGLE_SPIN_UNCHANGED" in result.stdout, diagnostics


def _run_immediate_sigterm(mode, repetition):
    environment = os.environ.copy()
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    environment["ROS_DOMAIN_ID"] = str(20 + repetition)
    process = subprocess.Popen(
        [sys.executable, str(WORKER), "--mode", mode],
        cwd=ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    output = bytearray()
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    try:
        deadline = time.monotonic() + 20.0
        while process.poll() is None:
            remaining = deadline - time.monotonic()
            assert remaining > 0, output.decode(errors="replace")
            events = selector.select(remaining)
            assert events, output.decode(errors="replace")
            chunk = os.read(process.stdout.fileno(), 4096)
            if chunk:
                output.extend(chunk)
                if b"OPTIMIZED_SIGNAL_READY" in output:
                    break
        assert b"OPTIMIZED_SIGNAL_READY" in output, output.decode(errors="replace")
        os.killpg(process.pid, signal.SIGTERM)
        try:
            stdout, stderr = process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGUSR1)
            try:
                stdout, stderr = process.communicate(timeout=0.25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                stdout, stderr = process.communicate(timeout=2.0)
            pytest.fail(
                "optimized %s worker timed out:\n%s\n%s" % (
                    mode,
                    (output + stdout).decode(errors="replace"),
                    stderr.decode(errors="replace"),
                )
            )
        output.extend(stdout)
        rendered = output.decode(errors="replace")
        diagnostics = rendered + "\n" + stderr.decode(errors="replace")
        assert process.returncode == 0, diagnostics
        assert "OPTIMIZED_SIGNAL_STATUS_OK" in rendered, diagnostics
        assert "OPTIMIZED_SIGNAL_OWNERSHIP_OK" in rendered, diagnostics
        assert "OPTIMIZED_SIGNAL_SIGNATURES_OK" in rendered, diagnostics
        assert "OPTIMIZED_SIGNAL_CLEAN" in rendered, diagnostics
    finally:
        selector.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)


@pytest.mark.parametrize("mode", ("global", "single", "multi"))
def test_immediate_sigterm_exits_cleanly_in_fresh_optimized_processes(mode):
    for repetition in range(5):
        _run_immediate_sigterm(mode, repetition)
