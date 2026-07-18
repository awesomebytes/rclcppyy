import hashlib
import json

import pytest

from scripts.ci import verify_release_matrix as matrix


PRODUCT_COMMIT = "1" * 40
SUITE_COMMIT = "2" * 40
TAG = "v0.3.0"


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _bundle(root, architecture, platform, *, support_suffix=""):
    bundle = root / ("release-bundle-" + platform)
    artifact = bundle / "product" / platform / (
        "ros-jazzy-rclcppyy-0.3.0-h%s_0.conda" % architecture)
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(("product-" + architecture).encode())
    digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
    evidence = bundle / "evidence"
    evidence.mkdir()
    attestation = {
        "schema": "rclcppyy.local-package-attestation/v2",
        "architecture": architecture,
        "source_snapshots": {
            "rclcppyy": {"commit": PRODUCT_COMMIT},
            "cppyy_kit": {"commit": SUITE_COMMIT},
        },
        "artifacts": [{
            "path": "%s/%s" % (platform, artifact.name),
            "sha256": digest,
            "size_bytes": artifact.stat().st_size,
        }],
    }
    inventory = {
        "schema": "rclcppyy.release-package-inventory/v1",
        "architecture": architecture,
        "product_commit": PRODUCT_COMMIT,
        "suite": {"commit": SUITE_COMMIT, "package_version": "0.2.0"},
        "validated": {
            "artifact_hashes": True,
            "conda_identities": True,
            "exact_support_dependencies": True,
            "source_commits": True,
        },
        "packages": [{
            "name": matrix.PRODUCT_NAME,
            "version": "0.3.0",
            "subdir": platform,
            "artifact": {"sha256": digest},
        }],
    }
    spdx = {
        "spdxVersion": "SPDX-2.3",
        "packages": [{
            "name": matrix.PRODUCT_NAME,
            "checksums": [{"algorithm": "SHA256", "checksumValue": digest}],
        }],
    }
    support_packages = []
    for name in matrix.SUPPORT_NAMES:
        filename = "%s-0.2.0-py_fixture_0.conda" % name
        published_sha = hashlib.sha256((name + support_suffix).encode()).hexdigest()
        support_packages.append({
            "name": name,
            "version": "0.2.0",
            "build": "py_fixture_0",
            "subdir": "noarch",
            "filename": filename,
            "published_artifact": {"sha256": published_sha, "size_bytes": 123},
            "provenance": {
                "verified_attestations": 1,
                "repository": "awesomebytes/cppyy_kit",
                "source_commit": SUITE_COMMIT,
                "source_ref": "refs/tags/v0.2.0",
                "signer_workflow": "awesomebytes/cppyy_kit/.github/workflows/release.yml",
            },
        })
    support = {
        "schema": "rclcppyy.published-support-proof/v1",
        "validated": {
            "available_before_product_publication": True,
            "github_provenance": True,
            "published_bytes_match_repodata": True,
            "same_suite_source_and_build_identity": True,
            "suite_source_identity": True,
        },
        "packages": support_packages,
    }
    _write_json(evidence / "local-package-attestation.json", attestation)
    _write_json(evidence / "release-package-inventory.json", inventory)
    _write_json(evidence / "rclcppyy.spdx.json", spdx)
    _write_json(evidence / "published-support.json", support)
    _write_json(evidence / "provenance.sigstore.json", {"bundle": "provenance"})
    _write_json(evidence / "sbom.sigstore.json", {"bundle": "sbom"})
    markers = [
        "INSTALLED_NATIVE_SERVICE_OK",
        "INSTALLED_RCLCPPYY_SAME_HANDLE_SERIALIZED_PUBLISH_OK",
    ]
    if architecture == "aarch64":
        markers.append("INSTALLED_LOCAL_CPPYY_ARM_BRIDGE_OK")
    (evidence / "package-proof.log").write_text("\n".join(markers), encoding="utf-8")
    return bundle


def _suite_lock():
    return {
        "schema": "rclcppyy.suite-source/v1",
        "repository": "awesomebytes/cppyy_kit",
        "commit": SUITE_COMMIT,
        "package_version": "0.2.0",
    }


def _verifier(calls):
    def verify(artifact, bundle, predicate, source_commit, source_ref):
        calls.append((artifact.name, bundle.name, predicate, source_commit, source_ref))
        return {"verified_attestations": 1, "predicate_type": predicate}
    return verify


def test_matrix_requires_and_reverifies_both_complete_architectures(tmp_path):
    _bundle(tmp_path, "x86_64", "linux-64")
    _bundle(tmp_path, "aarch64", "linux-aarch64")
    calls = []

    proof = matrix.verify_release_matrix(
        tmp_path,
        product_commit=PRODUCT_COMMIT,
        tag=TAG,
        suite_lock=_suite_lock(),
        verifier=_verifier(calls),
    )

    assert proof["ready_to_publish"] is True
    assert proof["publication_scope"] == [matrix.PRODUCT_NAME]
    assert {row["platform"] for row in proof["architectures"]} == {
        "linux-64", "linux-aarch64"}
    assert len(calls) == 4
    assert {call[2] for call in calls} == set(matrix.PREDICATES.values())
    assert all(call[3:] == (PRODUCT_COMMIT, "refs/tags/v0.3.0") for call in calls)


def test_matrix_rejects_missing_architecture_or_installed_proof(tmp_path):
    _bundle(tmp_path, "x86_64", "linux-64")
    with pytest.raises(ValueError, match="exactly two transferred bundles"):
        matrix.verify_release_matrix(
            tmp_path, product_commit=PRODUCT_COMMIT, tag=TAG,
            suite_lock=_suite_lock(), verifier=_verifier([]))

    arm = _bundle(tmp_path, "aarch64", "linux-aarch64")
    proof_log = arm / "evidence" / "package-proof.log"
    proof_log.write_text("INSTALLED_NATIVE_SERVICE_OK\n", encoding="utf-8")
    with pytest.raises(ValueError, match="package proof lacks"):
        matrix.verify_release_matrix(
            tmp_path, product_commit=PRODUCT_COMMIT, tag=TAG,
            suite_lock=_suite_lock(), verifier=_verifier([]))


def test_matrix_rejects_architecture_support_identity_disagreement(tmp_path):
    _bundle(tmp_path, "x86_64", "linux-64")
    _bundle(tmp_path, "aarch64", "linux-aarch64", support_suffix="different")

    with pytest.raises(ValueError, match="disagree on published support identities"):
        matrix.verify_release_matrix(
            tmp_path, product_commit=PRODUCT_COMMIT, tag=TAG,
            suite_lock=_suite_lock(), verifier=_verifier([]))
