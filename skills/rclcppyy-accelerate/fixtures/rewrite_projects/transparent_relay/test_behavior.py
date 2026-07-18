"""Project-local parity test; the repository suite runs the same evidence gate."""

from pathlib import Path
import subprocess
import sys


RUNNER = Path(__file__).resolve().parents[1] / "evidence_protocol.py"


def test_transparent_rewrite_preserves_bounded_behavior(tmp_path):
    output = tmp_path / "transparent-relay-evidence.json"
    subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--project",
            "transparent_relay",
            "--output",
            str(output),
        ],
        check=True,
        timeout=180,
    )
    assert output.is_file()
