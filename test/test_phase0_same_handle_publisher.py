#!/usr/bin/env python3
"""Isolated test wrapper for the Phase 0 same-handle publisher probe."""
import os
from pathlib import Path
import subprocess
import sys
import unittest


PROBE = Path(__file__).resolve().parents[1] / "scripts" / "probes" / "phase0_same_handle_publisher.py"
TIMEOUT_S = 120
EXPECTED_MARKERS = (
    "PHASE0_AUTHORITY_OK",
    "PHASE0_GRAPH_OK",
    "PHASE0_ROUNDTRIP_OK",
    "PHASE0_TEARDOWN_OK",
)
FAULT_MARKERS = (
    "segmentation",
    "fatal python error",
    "core dumped",
    "traceback (most recent call last)",
    "aborted",
)


def _format_output(proc):
    return (
        f"\n--- exit code: {proc.returncode} ---"
        f"\n--- stdout ---\n{proc.stdout}"
        f"\n--- stderr ---\n{proc.stderr}"
    )


class TestPhase0SameHandlePublisher(unittest.TestCase):

    def test_stock_node_authority_and_clean_exit(self):
        proc = subprocess.run(
            [sys.executable, str(PROBE)],
            capture_output=True,
            text=True,
            timeout=TIMEOUT_S,
            env=os.environ.copy(),
        )
        details = _format_output(proc)

        for marker in EXPECTED_MARKERS:
            self.assertIn(marker, proc.stdout, details)
        self.assertEqual(proc.returncode, 0, details)

        lowered_stderr = proc.stderr.lower()
        for marker in FAULT_MARKERS:
            self.assertNotIn(marker, lowered_stderr, details)


if __name__ == "__main__":
    unittest.main()
