#!/usr/bin/env python3
"""Verify a complete dual-architecture release bundle before publication."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Callable


SCHEMA = "rclcppyy.release-matrix-proof/v1"
PRODUCT_NAME = "ros-jazzy-rclcppyy"
SUPPORT_NAMES = ("cppyy-kit", "ros-jazzy-rclcpp-kit")
ARM_SUPPORT_NAME = "cppyy"
ARCHITECTURES = {
    "x86_64": "linux-64",
    "aarch64": "linux-aarch64",
}
PREDICATES = {
    "provenance": "https://slsa.dev/provenance/v1",
    "sbom": "https://spdx.dev/Document/v2.3",
}
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    _require(isinstance(value, dict), "%s must contain a JSON object" % path)
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_attestation(
    artifact: Path,
    bundle: Path,
    predicate_type: str,
    source_commit: str,
    source_ref: str,
) -> dict:
    repository = "awesomebytes/rclcppyy"
    signer = "%s/.github/workflows/release.yml" % repository
    command = [
        "gh", "attestation", "verify", str(artifact),
        "--bundle", str(bundle),
        "--repo", repository,
        "--signer-workflow", signer,
        "--source-digest", source_commit,
        "--source-ref", source_ref,
        "--predicate-type", predicate_type,
        "--deny-self-hosted-runners",
        "--format", "json",
    ]
    process = subprocess.run(command, capture_output=True, text=True, check=True)
    value = json.loads(process.stdout)
    _require(isinstance(value, list) and value,
             "%s has no verified %s attestation" % (artifact, predicate_type))
    return {
        "predicate_type": predicate_type,
        "verified_attestations": len(value),
        "repository": repository,
        "signer_workflow": signer,
        "source_commit": source_commit,
        "source_ref": source_ref,
        "self_hosted_runner_denied": True,
    }


def _only(paths, label: str) -> Path:
    values = sorted(paths)
    _require(len(values) == 1, "expected exactly one %s, found %d" % (label, len(values)))
    return values[0]


def _support_identity(proof: dict, architecture: str) -> list[dict]:
    _require(proof.get("schema") == "rclcppyy.published-support-proof/v2",
             "bundle has unsupported published-support proof")
    _require(proof.get("architecture") == architecture,
             "published-support proof architecture differs")
    validated = proof.get("validated", {})
    for field in (
        "available_before_product_build",
        "exact_package_identities",
        "github_provenance",
        "isolated_local_channel",
        "published_bytes_match_repodata",
        "retained_bytes_match_published",
        "suite_source_identity",
    ):
        _require(validated.get(field) is True, "published-support proof lacks %s" % field)
    expected_names = list(SUPPORT_NAMES)
    if architecture == "aarch64":
        expected_names.append(ARM_SUPPORT_NAME)
    rows = proof.get("packages")
    _require(isinstance(rows, list) and len(rows) == len(expected_names),
             "published-support proof has the wrong package count")
    by_name = {row.get("name"): row for row in rows if isinstance(row, dict)}
    _require(set(by_name) == set(expected_names),
             "published-support proof has the wrong package set")
    identities = []
    for name in expected_names:
        row = by_name[name]
        provenance = row.get("provenance", {})
        _require(provenance.get("verified_attestations", 0) > 0,
                 "%s has no verified suite provenance" % name)
        published = row.get("published_artifact", {})
        _require(re.fullmatch(r"[0-9a-f]{64}", str(published.get("sha256"))) is not None,
                 "%s published digest is invalid" % name)
        retained = row.get("retained_artifact", {})
        _require((retained.get("sha256"), retained.get("size_bytes")) ==
                 (published.get("sha256"), published.get("size_bytes")),
                 "%s retained bytes differ from published bytes" % name)
        _require(retained.get("path") == "%s/%s" % (
            row.get("subdir"), row.get("filename")),
            "%s retained artifact path differs" % name)
        identities.append({
            "name": name,
            "version": row.get("version"),
            "build": row.get("build"),
            "subdir": row.get("subdir"),
            "filename": row.get("filename"),
            "published_artifact": published,
            "retained_artifact": retained,
            "suite_repository": provenance.get("repository"),
            "suite_source_commit": provenance.get("source_commit"),
            "suite_source_ref": provenance.get("source_ref"),
            "suite_signer_workflow": provenance.get("signer_workflow"),
        })
    return identities


def _validate_bundle(
    bundle_dir: Path,
    *,
    product_commit: str,
    source_ref: str,
    suite_lock: dict,
    product_version: str,
    verifier: Callable[[Path, Path, str, str, str], dict],
) -> dict:
    product_artifact = _only(
        bundle_dir.glob("product/*/%s-%s-*.conda" % (PRODUCT_NAME, product_version)),
        "product artifact in %s" % bundle_dir,
    )
    evidence = bundle_dir / "evidence"
    _require(evidence.is_dir(), "%s has no evidence directory" % bundle_dir)
    local_attestation = _load_json(evidence / "local-package-attestation.json")
    inventory = _load_json(evidence / "release-package-inventory.json")
    spdx = _load_json(evidence / "rclcppyy.spdx.json")
    support_proof = _load_json(evidence / "published-support.json")
    package_proof = (evidence / "package-proof.log").read_text(encoding="utf-8")
    provenance_bundle = evidence / "provenance.sigstore.json"
    sbom_bundle = evidence / "sbom.sigstore.json"
    for path in (provenance_bundle, sbom_bundle):
        _require(path.is_file() and path.stat().st_size > 0, "missing attestation bundle %s" % path)
        json.loads(path.read_text(encoding="utf-8"))

    architecture = inventory.get("architecture")
    _require(architecture in ARCHITECTURES, "%s has unsupported architecture" % bundle_dir)
    platform = ARCHITECTURES[architecture]
    _require(product_artifact.parent.name == platform,
             "%s product artifact is in the wrong platform directory" % bundle_dir)
    _require(inventory.get("schema") == "rclcppyy.release-package-inventory/v1",
             "%s has unsupported release inventory" % bundle_dir)
    _require(inventory.get("product_commit") == product_commit,
             "%s inventory has the wrong product commit" % bundle_dir)
    _require(inventory.get("suite", {}).get("commit") == suite_lock["commit"],
             "%s inventory has the wrong suite commit" % bundle_dir)
    _require(inventory.get("suite", {}).get("package_version") == suite_lock["package_version"],
             "%s inventory has the wrong suite version" % bundle_dir)
    for field in (
        "artifact_hashes", "conda_identities", "exact_support_dependencies",
        "published_dependency_bytes", "source_commits",
    ):
        _require(inventory.get("validated", {}).get(field) is True,
                 "%s inventory lacks validation %s" % (bundle_dir, field))

    artifact_sha = _sha256(product_artifact)
    inventory_by_name = {
        row.get("name"): row for row in inventory.get("packages", [])
        if isinstance(row, dict)
    }
    product_rows = [row for row in inventory.get("packages", [])
                    if row.get("name") == PRODUCT_NAME]
    _require(len(product_rows) == 1, "%s inventory has no unique product" % bundle_dir)
    product_row = product_rows[0]
    _require(product_row.get("version") == product_version,
             "%s inventory product version differs" % bundle_dir)
    _require(product_row.get("subdir") == platform,
             "%s inventory product subdir differs" % bundle_dir)
    _require(product_row.get("artifact", {}).get("sha256") == artifact_sha,
             "%s inventory product hash differs" % bundle_dir)

    _require(local_attestation.get("schema") == "rclcppyy.local-package-attestation/v2",
             "%s has unsupported local attestation" % bundle_dir)
    _require(local_attestation.get("architecture") == architecture,
             "%s local attestation architecture differs" % bundle_dir)
    _require(local_attestation.get("published_dependencies", {}).get(
        "exact_retained_bytes") is True,
        "%s local attestation lacks exact published dependency bytes" % bundle_dir)
    snapshots = local_attestation.get("source_snapshots", {})
    _require(snapshots.get("rclcppyy", {}).get("commit") == product_commit,
             "%s local attestation product commit differs" % bundle_dir)
    _require(snapshots.get("cppyy_kit", {}).get("commit") == suite_lock["commit"],
             "%s local attestation suite commit differs" % bundle_dir)
    attested_products = [row for row in local_attestation.get("artifacts", [])
                         if Path(row.get("path", "")).name == product_artifact.name]
    _require(len(attested_products) == 1 and attested_products[0].get("sha256") == artifact_sha,
             "%s product artifact differs from local attestation" % bundle_dir)

    _require(spdx.get("spdxVersion") == "SPDX-2.3",
             "%s has unsupported SPDX version" % bundle_dir)
    spdx_products = [row for row in spdx.get("packages", [])
                     if row.get("name") == PRODUCT_NAME]
    _require(len(spdx_products) == 1, "%s SPDX has no unique product package" % bundle_dir)
    checksums = spdx_products[0].get("checksums", [])
    _require({row.get("checksumValue") for row in checksums} == {artifact_sha},
             "%s SPDX product checksum differs" % bundle_dir)

    for marker in (
        "INSTALLED_PUBLISHED_SUPPORT_BYTES_OK",
        "INSTALLED_NATIVE_SERVICE_OK",
        "INSTALLED_RCLCPPYY_COMPATIBLE_STOCK_PUBLISH_OK",
        "INSTALLED_RCLCPPYY_PUBLISHER_CPP_OK",
    ):
        _require(marker in package_proof, "%s package proof lacks %s" % (bundle_dir, marker))
    if architecture == "aarch64":
        _require("INSTALLED_PUBLISHED_CPPYY_ARM_BRIDGE_OK" in package_proof,
                 "%s package proof lacks published ARM bridge marker" % bundle_dir)

    support_packages = _support_identity(support_proof, architecture)
    _require(support_proof.get("suite", {}).get("commit") == suite_lock["commit"],
             "%s published support suite commit differs" % bundle_dir)
    _require(support_proof.get("suite", {}).get("package_version") ==
             suite_lock["package_version"],
             "%s published support suite version differs" % bundle_dir)
    for support in support_packages:
        _require(support.get("suite_source_commit") == suite_lock["commit"],
                 "%s %s provenance commit differs" % (bundle_dir, support["name"]))
        _require(support.get("suite_source_ref") ==
                 "refs/tags/v%s" % suite_lock["package_version"],
                 "%s %s provenance ref differs" % (bundle_dir, support["name"]))
    attested_by_path = {
        row.get("path"): row for row in local_attestation.get("artifacts", [])
        if isinstance(row, dict)
    }
    for support in support_packages:
        name = support["name"]
        package = inventory_by_name.get(name, {})
        artifact = package.get("artifact", {})
        published = support["published_artifact"]
        retained = support["retained_artifact"]
        _require((artifact.get("path"), artifact.get("sha256"),
                  artifact.get("size_bytes")) == (
            retained.get("path"), published.get("sha256"),
            published.get("size_bytes")),
            "%s %s inventory is not bound to published bytes" % (bundle_dir, name))
        attested = attested_by_path.get(retained.get("path"), {})
        _require((attested.get("sha256"), attested.get("size_bytes")) == (
            published.get("sha256"), published.get("size_bytes")),
            "%s %s attestation is not bound to published bytes" % (bundle_dir, name))

    verified_attestations = {
        name: verifier(
            product_artifact,
            provenance_bundle if name == "provenance" else sbom_bundle,
            predicate,
            product_commit,
            source_ref,
        )
        for name, predicate in PREDICATES.items()
    }
    for name, result in verified_attestations.items():
        _require(result.get("verified_attestations", 0) > 0,
                 "%s %s bundle has no verified attestation" % (bundle_dir, name))
    return {
        "architecture": architecture,
        "platform": platform,
        "product_artifact": {
            "path": product_artifact.as_posix(),
            "sha256": artifact_sha,
            "size_bytes": product_artifact.stat().st_size,
        },
        "support_packages": support_packages,
        "attestations": verified_attestations,
        "evidence": {
            "inventory_sha256": _sha256(evidence / "release-package-inventory.json"),
            "spdx_sha256": _sha256(evidence / "rclcppyy.spdx.json"),
            "package_proof_sha256": _sha256(evidence / "package-proof.log"),
            "published_support_sha256": _sha256(evidence / "published-support.json"),
        },
    }


def verify_release_matrix(
    input_root: Path,
    *,
    product_commit: str,
    tag: str,
    suite_lock: dict,
    verifier: Callable[[Path, Path, str, str, str], dict] = _verify_attestation,
) -> dict:
    _require(COMMIT_RE.fullmatch(product_commit) is not None,
             "product commit must be a full lowercase Git commit")
    _require(suite_lock.get("schema") == "rclcppyy.suite-source/v1",
             "unsupported suite source lock schema")
    _require(COMMIT_RE.fullmatch(str(suite_lock.get("commit"))) is not None,
             "suite source lock has an invalid commit")
    _require(re.fullmatch(r"v[0-9]+\.[0-9]+\.[0-9]+", tag) is not None,
             "release tag must be an exact stable version")
    product_version = tag[1:]
    bundle_dirs = sorted(path for path in input_root.iterdir() if path.is_dir())
    _require(len(bundle_dirs) == 2, "release matrix requires exactly two transferred bundles")
    source_ref = "refs/tags/%s" % tag
    bundles = [
        _validate_bundle(
            path,
            product_commit=product_commit,
            source_ref=source_ref,
            suite_lock=suite_lock,
            product_version=product_version,
            verifier=verifier,
        )
        for path in bundle_dirs
    ]
    by_architecture = {bundle["architecture"]: bundle for bundle in bundles}
    _require(set(by_architecture) == set(ARCHITECTURES),
             "release matrix lacks one native architecture")
    common_support_sets = [
        [row for row in bundle["support_packages"] if row["name"] in SUPPORT_NAMES]
        for bundle in bundles
    ]
    _require(common_support_sets[0] == common_support_sets[1],
             "architecture bundles disagree on published support identities")
    return {
        "schema": SCHEMA,
        "tag": tag,
        "source_ref": source_ref,
        "product_commit": product_commit,
        "suite": {
            "repository": suite_lock.get("repository"),
            "commit": suite_lock["commit"],
            "package_version": suite_lock.get("package_version"),
        },
        "architectures": [by_architecture[name] for name in sorted(by_architecture)],
        "published_support_packages": by_architecture["aarch64"]["support_packages"],
        "ready_to_publish": True,
        "publication_scope": [PRODUCT_NAME],
    }


def _write_atomic(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.%d.tmp" % (path.name, os.getpid()))
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", required=True, type=Path)
    parser.add_argument("--product-commit", required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--suite-lock", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        proof = verify_release_matrix(
            arguments.input_root.resolve(),
            product_commit=arguments.product_commit,
            tag=arguments.tag,
            suite_lock=_load_json(arguments.suite_lock),
        )
        _write_atomic(arguments.output, proof)
    except (
        OSError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
        ValueError,
    ) as error:
        print("release matrix verification failed: %s" % error)
        return 1
    print("DUAL_ARCH_RELEASE_MATRIX_VERIFIED artifacts=%d" % len(proof["architectures"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
