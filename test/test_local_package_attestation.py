import hashlib
from pathlib import Path

import pytest

from scripts.ci import write_local_package_attestation as attestation


PRODUCT_COMMIT = "1" * 40
SUITE_COMMIT = "2" * 40


def _write_artifacts(root: Path) -> dict[str, bytes]:
    contents = {
        "noarch/cppyy-kit-0.2.0-test.conda": b"base",
        "noarch/ros-jazzy-rclcpp-kit-0.2.0-test.conda": b"ros",
        "linux-64/ros-jazzy-rclcppyy-0.3.0-test.conda": b"product",
    }
    for relative, content in contents.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    return contents


def test_attestation_binds_exact_commits_and_artifact_bytes(tmp_path):
    contents = _write_artifacts(tmp_path)

    value = attestation.build_attestation(
        tmp_path,
        product_commit=PRODUCT_COMMIT,
        suite_commit=SUITE_COMMIT,
        architecture="x86_64",
    )

    assert value["schema"] == attestation.SCHEMA
    assert value["source_snapshots"] == {
        "method": "git-archive",
        "rclcppyy_commit": PRODUCT_COMMIT,
        "cppyy_kit_commit": SUITE_COMMIT,
    }
    assert value["signed"] is False
    assert value["performance_claims_allowed"] is False
    artifacts = {item["path"]: item for item in value["artifacts"]}
    assert set(artifacts) == set(contents)
    for path, content in contents.items():
        assert artifacts[path]["size_bytes"] == len(content)
        assert artifacts[path]["sha256"] == hashlib.sha256(content).hexdigest()


def test_attestation_rejects_ambiguous_or_missing_artifacts(tmp_path):
    _write_artifacts(tmp_path)
    duplicate = tmp_path / "noarch" / "cppyy-kit-0.2.0-second.conda"
    duplicate.write_bytes(b"duplicate")

    with pytest.raises(ValueError, match="exactly one artifact"):
        attestation.collect_artifacts(tmp_path)


def test_attestation_rejects_non_commit_identity(tmp_path):
    _write_artifacts(tmp_path)

    with pytest.raises(ValueError, match="full lowercase Git commit"):
        attestation.build_attestation(
            tmp_path,
            product_commit="main",
            suite_commit=SUITE_COMMIT,
            architecture="x86_64",
        )
