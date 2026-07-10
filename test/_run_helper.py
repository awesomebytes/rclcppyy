"""Shared utility: run a sibling helper script in a fresh interpreter.

The monkeypatch and plain-bringup smoke scenarios each need their own Python
process (see the individual tests for why), so they live in ``_*_helper.py``
scripts run via subprocess. This centralises spawning them and surfacing their
output when an assertion fails.
"""
import os
import subprocess
import sys

# Safety cap only. A warm run finishes in a few seconds; the generous ceiling
# covers a cold cppyy JIT compile of rclcpp on a slow CI runner.
DEFAULT_TIMEOUT_S = 120


def run_helper(helper_filename, timeout=DEFAULT_TIMEOUT_S):
    """Run test/<helper_filename> in a subprocess, returning the CompletedProcess.

    The child inherits the current environment (activated pixi env, RMW choice,
    and any ROS_DOMAIN_ID the caller exported), so tests never hardcode a domain.
    """
    helper = os.path.join(os.path.dirname(__file__), helper_filename)
    return subprocess.run(
        [sys.executable, helper],
        capture_output=True,
        text=True,
        timeout=timeout,
        env=os.environ.copy(),
    )


def format_output(proc):
    """Readable dump of a helper's stdout/stderr for assertion messages."""
    return (
        f"\n--- exit code: {proc.returncode} ---"
        f"\n--- stdout ---\n{proc.stdout}"
        f"\n--- stderr ---\n{proc.stderr}"
    )
