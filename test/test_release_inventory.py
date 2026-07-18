import hashlib
import json
from pathlib import Path

import pytest

from scripts.ci import build_release_inventory as inventory
from scripts.ci import verify_published_support as support


PRODUCT_COMMIT = "1" * 40
SUITE_COMMIT = "2" * 40


def _metadata(name, version, build, subdir, depends=()):
    return {
        "index": {
            "name": name,
            "version": version,
            "build": build,
            "build_number": 0,
            "subdir": subdir,
            "depends": list(depends),
            "license": "BSD-3-Clause",
        },
        "about": {
            "home": "https://example.invalid/%s" % name,
            "dev_url": "https://example.invalid/src/%s" % name,
            "summary": "%s fixture" % name,
        },
        "paths": {
            "paths_version": 1,
            "paths": [{
                "_path": "site-packages/%s.py" % name,
                "path_type": "hardlink",
                "sha256": hashlib.sha256(name.encode()).hexdigest(),
                "size_in_bytes": len(name),
            }],
        },
    }


def _fixture(tmp_path, architecture="x86_64"):
    platform = "linux-64" if architecture == "x86_64" else "linux-aarch64"
    definitions = [
        ("cppyy-kit", "0.2.0", "pyh4616a5c_0", "noarch", ()),
        ("ros-jazzy-rclcpp-kit", "0.2.0", "pyh4616a5c_0", "noarch", ()),
        (
            "ros-jazzy-rclcppyy",
            "0.3.0",
            "h_fixture_0",
            platform,
            (
                "cppyy >=3.5,<4",
                "cppyy-kit ==0.2.0",
                "ros-jazzy-rclcpp-kit ==0.2.0",
            ),
        ),
    ]
    if architecture != "x86_64":
        definitions.append(("cppyy", "3.5.0", "py312hf18b547_0", platform, ()))
    metadata = {}
    artifacts = []
    for name, version, build, subdir, depends in definitions:
        relative = "%s/%s-%s-%s.conda" % (subdir, name, version, build)
        path = tmp_path / "output" / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((name + version + build).encode())
        metadata[relative] = _metadata(name, version, build, subdir, depends)
        artifacts.append({
            "path": relative,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "size_bytes": path.stat().st_size,
        })
    expected_published_names = {"cppyy-kit", "ros-jazzy-rclcpp-kit"}
    if architecture != "x86_64":
        expected_published_names.add("cppyy")
    published_packages = []
    for artifact in artifacts:
        relative = artifact["path"]
        package = metadata[relative]["index"]
        if package["name"] not in expected_published_names:
            continue
        published_packages.append({
            "name": package["name"],
            "version": package["version"],
            "build": package["build"],
            "subdir": package["subdir"],
            "filename": Path(relative).name,
            "published_artifact": {
                "sha256": artifact["sha256"],
                "size_bytes": artifact["size_bytes"],
            },
            "retained_artifact": {
                "path": relative,
                "sha256": artifact["sha256"],
                "size_bytes": artifact["size_bytes"],
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
        "architecture": architecture,
        "suite": {
            "repository": "awesomebytes/cppyy_kit",
            "commit": SUITE_COMMIT,
            "package_version": "0.2.0",
        },
        "packages": published_packages,
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
    attestation = {
        "schema": inventory.ATTESTATION_SCHEMA,
        "architecture": architecture,
        "source_snapshots": {
            "rclcppyy": {"commit": PRODUCT_COMMIT},
            "cppyy_kit": {
                "commit": SUITE_COMMIT,
                "method": "published-release-provenance",
            },
        },
        "artifacts": artifacts,
        "published_dependencies": {"exact_retained_bytes": True},
    }
    suite_lock = {
        "schema": inventory.SUITE_LOCK_SCHEMA,
        "repository": "awesomebytes/cppyy_kit",
        "commit": SUITE_COMMIT,
        "package_version": "0.2.0",
    }

    def extract(artifact, destination):
        relative = artifact.relative_to(tmp_path / "output").as_posix()
        info = destination / "info"
        info.mkdir(parents=True)
        values = metadata[relative]
        for key in ("index", "about", "paths"):
            (info / (key + ".json")).write_text(json.dumps(values[key]), encoding="utf-8")

    return attestation, published_proof, suite_lock, extract


def _build(tmp_path, architecture="x86_64"):
    attestation, published_proof, suite_lock, extractor = _fixture(
        tmp_path, architecture)
    value = inventory.build_inventory(
        tmp_path / "output",
        tmp_path / "extract",
        attestation=attestation,
        published_support_proof=published_proof,
        suite_lock=suite_lock,
        architecture=architecture,
        product_version="0.3.0",
        product_commit=PRODUCT_COMMIT,
        extractor=extractor,
    )
    return value, attestation, published_proof, suite_lock


def test_inventory_validates_exact_x86_stack_and_emits_file_level_spdx(tmp_path):
    value, _attestation, _published_proof, _suite_lock = _build(tmp_path)

    assert value["validated"] == {
        "artifact_hashes": True,
        "conda_identities": True,
        "exact_support_dependencies": True,
        "published_dependency_bytes": True,
        "source_commits": True,
    }
    assert [package["name"] for package in value["packages"]] == [
        "cppyy-kit", "ros-jazzy-rclcpp-kit", "ros-jazzy-rclcppyy"]
    spdx = inventory.build_spdx(value)
    assert spdx["spdxVersion"] == "SPDX-2.3"
    assert len(spdx["packages"]) == 3
    assert len(spdx["files"]) == 3
    product = next(package for package in spdx["packages"]
                   if package["name"] == inventory.PRODUCT_NAME)
    assert product["filesAnalyzed"] is True
    assert product["checksums"][0]["algorithm"] == "SHA256"
    assert "subdir=linux-64" in product["externalRefs"][0]["referenceLocator"]
    dependencies = [row for row in spdx["relationships"]
                    if row["relationshipType"] == "DEPENDS_ON"]
    assert len(dependencies) == 2


def test_inventory_includes_native_arm_bridge_as_exact_dependency(tmp_path):
    value, _attestation, _published_proof, _suite_lock = _build(
        tmp_path, "aarch64")

    by_name = {package["name"]: package for package in value["packages"]}
    assert by_name["cppyy"]["version"] == "3.5.0"
    assert by_name["cppyy"]["subdir"] == "linux-aarch64"
    spdx = inventory.build_spdx(value)
    assert len([row for row in spdx["relationships"]
                if row["relationshipType"] == "DEPENDS_ON"]) == 3


def test_inventory_rejects_dependency_or_source_identity_drift(tmp_path):
    attestation, published_proof, suite_lock, extractor = _fixture(tmp_path)
    metadata_extractor = extractor
    suite_lock["commit"] = "3" * 40
    with pytest.raises(ValueError, match="attestation suite commit"):
        inventory.build_inventory(
            tmp_path / "output", tmp_path / "extract-source",
            attestation=attestation, suite_lock=suite_lock,
            published_support_proof=published_proof,
            architecture="x86_64", product_version="0.3.0",
            product_commit=PRODUCT_COMMIT, extractor=metadata_extractor)

    attestation, published_proof, suite_lock, extractor = _fixture(
        tmp_path / "dependency")

    def missing_exact_dependency(artifact, destination):
        extractor(artifact, destination)
        index_path = destination / "info" / "index.json"
        index = json.loads(index_path.read_text())
        if index["name"] == inventory.PRODUCT_NAME:
            index["depends"].remove("cppyy-kit ==0.2.0")
            index["depends"].append("cppyy-kit >=0.1")
            index_path.write_text(json.dumps(index), encoding="utf-8")

    with pytest.raises(ValueError, match="exact dependency cppyy-kit ==0.2.0"):
        inventory.build_inventory(
            tmp_path / "dependency" / "output", tmp_path / "extract-dependency",
            attestation=attestation, suite_lock=suite_lock,
            published_support_proof=published_proof,
            architecture="x86_64", product_version="0.3.0",
            product_commit=PRODUCT_COMMIT, extractor=missing_exact_dependency)


def test_inventory_rejects_artifact_bytes_or_conda_identity_drift(tmp_path):
    attestation, published_proof, suite_lock, extractor = _fixture(tmp_path)
    artifact = tmp_path / "output" / attestation["artifacts"][0]["path"]
    artifact.write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash differs"):
        inventory.build_inventory(
            tmp_path / "output", tmp_path / "extract-hash",
            attestation=attestation, suite_lock=suite_lock,
            published_support_proof=published_proof,
            architecture="x86_64", product_version="0.3.0",
            product_commit=PRODUCT_COMMIT, extractor=extractor)
