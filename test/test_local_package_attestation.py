import json
import hashlib
from pathlib import Path

import pytest

from scripts.ci import write_local_package_attestation as attestation
from scripts.ci import verify_published_support as support


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
        "rclcppyy": {"commit": PRODUCT_COMMIT, "method": "git-archive"},
        "cppyy_kit": {
            "commit": SUITE_COMMIT,
            "method": "detached-git-checkout",
        },
    }
    assert value["cppyy_arm_bridge"] is None
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
        attestation.collect_artifacts(tmp_path, "x86_64")


def test_attestation_rejects_non_commit_identity(tmp_path):
    _write_artifacts(tmp_path)

    with pytest.raises(ValueError, match="full lowercase Git commit"):
        attestation.build_attestation(
            tmp_path,
            product_commit="main",
            suite_commit=SUITE_COMMIT,
            architecture="x86_64",
        )


def test_arm_attestation_requires_native_bridge_proof(tmp_path):
    contents = {
        "noarch/cppyy-kit-0.2.0-test.conda": b"base",
        "noarch/ros-jazzy-rclcpp-kit-0.2.0-test.conda": b"ros",
        "linux-aarch64/ros-jazzy-rclcppyy-0.3.0-test.conda": b"product",
        "linux-aarch64/cppyy-3.5.0-py312-test.conda": b"bridge",
    }
    for relative, content in contents.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    runtime = tmp_path / "cppyy-arm-runtime-proof.log"
    runtime.write_text(
        "CPPYY_ARM_IMPORT_OK 3.5.0\nCPPYY_ARM_CPPDEF_OK 42\n",
        encoding="utf-8",
    )
    bridge_path = tmp_path / "linux-aarch64/cppyy-3.5.0-py312-test.conda"
    proof = {
        "schema": "cppyy-kit.cppyy-package-proof/v1",
        "source_snapshot": {"commit": SUITE_COMMIT, "dirty": False},
        "build_host": {"native_arm64": True},
        "runtime_proof": {
            "filename": runtime.name,
            "native_import_version_and_cppdef": True,
            "sha256": hashlib.sha256(runtime.read_bytes()).hexdigest(),
        },
        "artifact": {
            "filename": bridge_path.name,
            "sha256": hashlib.sha256(bridge_path.read_bytes()).hexdigest(),
        },
        "source": {
            "schema": "cppyy-kit.upstream-package-source/v1",
            "package": {"platform": "linux-aarch64"},
        },
    }
    proof_path = tmp_path / "cppyy-arm-package-proof.json"
    proof_path.write_text(json.dumps(proof), encoding="utf-8")

    value = attestation.build_attestation(
        tmp_path,
        product_commit=PRODUCT_COMMIT,
        suite_commit=SUITE_COMMIT,
        architecture="aarch64",
    )

    assert len(value["artifacts"]) == 4
    assert value["cppyy_arm_bridge"]["artifact_sha256"] == proof["artifact"]["sha256"]
    assert value["cppyy_arm_bridge"]["package_proof"]["sha256"] == hashlib.sha256(
        proof_path.read_bytes()).hexdigest()


def test_release_attestation_binds_retained_published_support_bytes(tmp_path):
    contents = {
        "noarch/cppyy-kit-0.2.0-pyh4616a5c_0.conda": b"published-base",
        "noarch/ros-jazzy-rclcpp-kit-0.2.0-pyh4616a5c_0.conda": b"published-ros",
        "linux-64/ros-jazzy-rclcppyy-0.3.0-test.conda": b"product",
    }
    for relative, content in contents.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    packages = []
    for name in ("cppyy-kit", "ros-jazzy-rclcpp-kit"):
        relative = "noarch/%s-0.2.0-pyh4616a5c_0.conda" % name
        content = contents[relative]
        digest = hashlib.sha256(content).hexdigest()
        packages.append({
            "name": name,
            "version": "0.2.0",
            "build": "pyh4616a5c_0",
            "subdir": "noarch",
            "filename": Path(relative).name,
            "published_artifact": {
                "sha256": digest,
                "size_bytes": len(content),
            },
            "retained_artifact": {
                "path": relative,
                "sha256": digest,
                "size_bytes": len(content),
            },
            "provenance": {
                "verified_attestations": 1,
                "repository": "awesomebytes/cppyy_kit",
                "source_commit": SUITE_COMMIT,
                "source_ref": "refs/tags/v0.2.0",
            },
        })
    published_proof = {
        "schema": support.SCHEMA,
        "architecture": "x86_64",
        "suite": {
            "repository": "awesomebytes/cppyy_kit",
            "commit": SUITE_COMMIT,
            "package_version": "0.2.0",
        },
        "packages": packages,
        "validated": {
            "available_before_product_build": True,
            "exact_package_identities": True,
            "github_provenance": True,
            "isolated_local_channel": True,
            "published_bytes_match_repodata": True,
            "retained_bytes_match_published": True,
            "suite_source_identity": True,
        },
    }

    value = attestation.build_attestation(
        tmp_path,
        product_commit=PRODUCT_COMMIT,
        suite_commit=SUITE_COMMIT,
        architecture="x86_64",
        published_support_proof=published_proof,
    )

    assert value["source_snapshots"]["cppyy_kit"]["method"] == (
        "published-release-provenance")
    assert value["published_dependencies"]["exact_retained_bytes"] is True
    assert {row["sha256"] for row in value["published_dependencies"]["packages"]} == {
        hashlib.sha256(content).hexdigest()
        for path, content in contents.items() if path.startswith("noarch/")
    }
