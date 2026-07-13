#!/usr/bin/env python3
"""Headless smoke test for the heavy-topic ``ros2 topic hz`` demo machinery.

Runs the demo orchestrator (scripts/heavy_hz_demo/run_heavy_hz.py) end to end --
heavy publisher + hz subscriber + CPU/throughput measurement -- but on plain rclpy
for both sides (``--variant rclpy --plain-publisher``), a tiny image, and a short
window. Plain rclpy needs no acceleration bridge, so this runs in the default env /
CI. It asserts only that the machinery ran and messages flowed (the runner exits
non-zero if the pair never produced a stat line); there is no performance
assertion, so CI stays stable regardless of the runner's load.

The accelerated variants and the cold-vs-warm startup story need the demo bridge
(rclcpp_kit/cppyy_kit newer than the published 0.1.0), so they are exercised by
``pixi run -e heavydemo demo-heavy-hz`` / ``demo-heavy-startup``, not here.
"""
import os
import subprocess
import sys
import unittest
from pathlib import Path

RUNNER = (Path(__file__).resolve().parent.parent
          / "scripts" / "heavy_hz_demo" / "run_heavy_hz.py")


class TestHeavyHzDemo(unittest.TestCase):

    def test_demo_machinery_runs_plain_rclpy(self):
        env = os.environ.copy()
        env.setdefault("ROS_DOMAIN_ID", "67")
        proc = subprocess.run(
            [sys.executable, str(RUNNER),
             "--variant", "rclpy", "--plain-publisher",
             "--rate", "30", "--width", "64", "--height", "64",
             "--duration", "2", "--warmup-timeout", "60"],
            capture_output=True, text=True, timeout=180, env=env,
        )
        detail = (f"\n--- exit {proc.returncode} ---"
                  f"\n--- stdout ---\n{proc.stdout}"
                  f"\n--- stderr ---\n{proc.stderr}")
        # Exit 0 means the publisher/subscriber pair warmed up and produced stats
        # (the runner exits 1 if no stat line ever arrives).
        self.assertEqual(proc.returncode, 0, detail)
        # The comparison table was printed with the rclpy variant row.
        self.assertIn("eff Hz", proc.stdout, detail)
        self.assertIn("rclpy", proc.stdout, detail)


if __name__ == "__main__":
    unittest.main()
