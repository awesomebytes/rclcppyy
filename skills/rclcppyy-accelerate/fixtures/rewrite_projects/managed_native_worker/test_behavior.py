"""Project-local parity test; the repository suite runs the same evidence gate."""

from pathlib import Path
import subprocess
import sys


RUNNER = Path(__file__).resolve().parents[1] / "evidence_protocol.py"


def test_managed_native_rewrite_preserves_bounded_behavior(tmp_path):
    output = tmp_path / "managed-native-worker-evidence.json"
    subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--project",
            "managed_native_worker",
            "--output",
            str(output),
        ],
        check=True,
        timeout=180,
    )
    assert output.is_file()
