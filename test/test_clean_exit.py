#!/usr/bin/env python3
"""Regression tripwire: a bringup + node + normal-exit process exits cleanly.

rclcppyy processes historically segfaulted (or otherwise exited nonzero) during
Python interpreter finalization -- after all useful work was done -- because the
rclcpp context / DDS layer was torn down at an undefined point relative to
cppyy's Cling teardown. Several scripts papered over it with ``os._exit(0)``,
which hides the return code. That dodge has been removed and replaced with an
ordered teardown (rclcppyy.shutdown_rclcpp on cppyy_kit's atexit hook).

This test is the tripwire that keeps the fix honest: it runs a minimal
bringup+node helper that returns normally (no os._exit) in its own interpreter
and asserts a clean, deterministic exit. If teardown regresses, the return code
goes nonzero or a fault lands on stderr, and this fails.
"""
import unittest

from _run_helper import run_helper, format_output

# Text that appears on stderr for the failure modes this guards against
# (SIGSEGV / SIGABRT via faulthandler or the shell, or a Python traceback from
# teardown). Matched case-insensitively.
_FAULT_MARKERS = ("segmentation", "fatal python error", "core dumped",
                   "traceback (most recent call last)", "aborted")


class TestCleanExit(unittest.TestCase):

    def test_bringup_node_exits_cleanly(self):
        proc = run_helper("_clean_exit_helper.py")
        # The work completed and the process reached its normal end-of-main.
        self.assertIn("CLEAN_EXIT_OK", proc.stdout, format_output(proc))
        # A normal return, not an os._exit dodge, gave a zero return code.
        self.assertEqual(proc.returncode, 0, format_output(proc))
        # Nothing crashed during interpreter finalization.
        lowered = proc.stderr.lower()
        for marker in _FAULT_MARKERS:
            self.assertNotIn(marker, lowered, format_output(proc))


if __name__ == "__main__":
    unittest.main()
