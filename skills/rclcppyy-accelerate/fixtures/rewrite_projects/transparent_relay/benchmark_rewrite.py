#!/usr/bin/env python3
"""Generate paired stock/compatible smoke evidence for this project."""

from pathlib import Path
import subprocess
import sys


RUNNER = Path(__file__).resolve().parents[1] / "evidence_protocol.py"


if __name__ == "__main__":
    raise SystemExit(subprocess.call(
        [sys.executable, str(RUNNER), "--project", "transparent_relay", *sys.argv[1:]]))
