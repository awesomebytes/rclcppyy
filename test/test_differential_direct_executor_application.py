"""Stock/direct parity for unchanged executor and callback-group usage."""

import os
import sys
from pathlib import Path

from differential.domains import acquire_domain
from differential.process import run_owned_process


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = REPO_ROOT / "build" / "test-artifacts" / "differential"


def test_direct_cpp_executor_callback_group_application_matches_stock():
    results = {}
    with acquire_domain() as domain:
        for mode in ("stock", "activated"):
            env = domain.environment()
            test_root = str(REPO_ROOT / "test")
            env["PYTHONPATH"] = os.pathsep.join(filter(None, [
                test_root,
                env.get("PYTHONPATH"),
            ]))
            artifact = ARTIFACT_ROOT / (
                "direct-executor-application-" + mode + ".json")
            process = run_owned_process(
                [
                    sys.executable,
                    "-m",
                    "differential._direct_executor_application_probe",
                    "--mode",
                    mode,
                ],
                cwd=REPO_ROOT,
                env=env,
                timeout_s=180,
                artifact_path=artifact,
            )
            details = "artifact=%s\n%s" % (artifact, process.diagnostics())
            assert process.returncode == 0, details
            assert not process.timed_out, details
            assert process.protocol_error is None, details
            assert process.protocol_result["outcome"] == "pass", details
            assert process.protocol_result["backend_verified"], details
            assert all(process.protocol_result["cleanup"].values()), details
            results[mode] = process.protocol_result

    assert results["activated"]["observations"] == results["stock"]["observations"]
