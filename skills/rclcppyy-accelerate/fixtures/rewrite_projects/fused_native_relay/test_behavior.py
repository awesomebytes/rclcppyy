"""Project-local parity test; the repository suite runs the same evidence gate."""

from pathlib import Path
import subprocess
import sys


RUNNER = Path(__file__).resolve().parents[1] / "evidence_protocol.py"


def test_fused_native_rewrite_preserves_bounded_behavior(tmp_path):
    output = tmp_path / "fused-native-relay-evidence.json"
    subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--project",
            "fused_native_relay",
            "--output",
            str(output),
        ],
        check=True,
        timeout=180,
    )
    assert output.is_file()
