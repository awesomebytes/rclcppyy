#!/usr/bin/env python3
"""Validate release conda artifacts and emit an SPDX 2.3 inventory."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Callable
from urllib.parse import quote

try:
    from .verify_published_support import validate_retained_support
except ImportError:
    from verify_published_support import validate_retained_support


INVENTORY_SCHEMA = "rclcppyy.release-package-inventory/v1"
ATTESTATION_SCHEMA = "rclcppyy.local-package-attestation/v2"
SUITE_LOCK_SCHEMA = "rclcppyy.suite-source/v1"
PRODUCT_NAME = "ros-jazzy-rclcppyy"
SUPPORT_NAMES = ("cppyy-kit", "ros-jazzy-rclcpp-kit")
ARCHITECTURES = {
    "x86_64": ("linux-64", False),
    "aarch64": ("linux-aarch64", True),
    "arm64": ("linux-aarch64", True),
}
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    _require(isinstance(value, dict), "%s must contain a JSON object" % path)
    return value


def _default_extractor(artifact: Path, destination: Path) -> None:
    subprocess.run(
        ["rattler-build", "package", "extract", str(artifact), "-d", str(destination)],
        check=True,
    )


def _expected_packages(architecture: str, product_version: str, suite_version: str) -> dict:
    try:
        product_subdir, needs_arm_bridge = ARCHITECTURES[architecture]
    except KeyError as error:
        raise ValueError("unsupported release architecture: %s" % architecture) from error
    expected = {
        PRODUCT_NAME: {"version": product_version, "subdir": product_subdir},
        "cppyy-kit": {"version": suite_version, "subdir": "noarch"},
        "ros-jazzy-rclcpp-kit": {"version": suite_version, "subdir": "noarch"},
    }
    if needs_arm_bridge:
        expected["cppyy"] = {"version": "3.5.0", "subdir": "linux-aarch64"}
    return expected


def _artifact_record(
    artifact: Path,
    relative_path: str,
    extracted: Path,
    attested: dict,
) -> dict:
    info = extracted / "info"
    index = _load_json(info / "index.json")
    about = _load_json(info / "about.json")
    paths = _load_json(info / "paths.json")
    _require(paths.get("paths_version") == 1, "%s has unsupported paths metadata" % artifact)
    path_rows = paths.get("paths")
    _require(isinstance(path_rows, list), "%s has no conda path inventory" % artifact)
    actual_sha = _sha256(artifact)
    _require(actual_sha == attested.get("sha256"), "%s hash differs from attestation" % artifact)
    _require(artifact.stat().st_size == attested.get("size_bytes"),
             "%s size differs from attestation" % artifact)
    _require(relative_path == attested.get("path"), "%s path differs from attestation" % artifact)

    name = index.get("name")
    version = index.get("version")
    build = index.get("build")
    _require(all(isinstance(value, str) and value for value in (name, version, build)),
             "%s has incomplete package identity" % artifact)
    expected_filename = "%s-%s-%s.conda" % (name, version, build)
    _require(artifact.name == expected_filename,
             "%s does not match extracted package identity %s" % (artifact, expected_filename))

    files = []
    for row in path_rows:
        _require(isinstance(row, dict), "%s contains malformed path metadata" % artifact)
        path = row.get("_path")
        _require(isinstance(path, str) and path, "%s contains an unnamed path" % artifact)
        checksum = row.get("sha256")
        if checksum is not None:
            _require(isinstance(checksum, str) and re.fullmatch(r"[0-9a-f]{64}", checksum),
                     "%s contains an invalid file checksum" % artifact)
        files.append({
            "path": path,
            "path_type": row.get("path_type"),
            "sha256": checksum,
            "size_bytes": row.get("size_in_bytes"),
        })
    depends = index.get("depends", [])
    _require(isinstance(depends, list) and all(isinstance(item, str) for item in depends),
             "%s has malformed dependency metadata" % artifact)
    return {
        "name": name,
        "version": version,
        "build": build,
        "build_number": index.get("build_number"),
        "subdir": index.get("subdir"),
        "artifact": {
            "path": relative_path,
            "sha256": actual_sha,
            "size_bytes": artifact.stat().st_size,
        },
        "license": index.get("license") or about.get("license") or "NOASSERTION",
        "homepage": about.get("home"),
        "repository": about.get("dev_url"),
        "summary": about.get("summary"),
        "depends": sorted(depends),
        "files": sorted(files, key=lambda item: item["path"]),
    }


def build_inventory(
    output_dir: Path,
    extract_root: Path,
    *,
    attestation: dict,
    published_support_proof: dict,
    suite_lock: dict,
    architecture: str,
    product_version: str,
    product_commit: str,
    extractor: Callable[[Path, Path], None] = _default_extractor,
) -> dict:
    _require(attestation.get("schema") == ATTESTATION_SCHEMA,
             "unsupported local package attestation schema")
    _require(suite_lock.get("schema") == SUITE_LOCK_SCHEMA,
             "unsupported suite source lock schema")
    _require(COMMIT_RE.fullmatch(product_commit) is not None,
             "product commit must be a full lowercase Git commit")
    suite_commit = suite_lock.get("commit")
    _require(isinstance(suite_commit, str) and COMMIT_RE.fullmatch(suite_commit) is not None,
             "suite lock commit must be a full lowercase Git commit")
    suite_version = suite_lock.get("package_version")
    _require(isinstance(suite_version, str) and suite_version,
             "suite lock package version is required")
    snapshots = attestation.get("source_snapshots", {})
    _require(snapshots.get("rclcppyy", {}).get("commit") == product_commit,
             "attestation product commit differs from release source")
    _require(snapshots.get("cppyy_kit", {}).get("commit") == suite_commit,
             "attestation suite commit differs from source lock")
    _require(attestation.get("architecture") == architecture,
             "attestation architecture differs from release runner")
    _require(attestation.get("source_snapshots", {}).get(
        "cppyy_kit", {}).get("method") == "published-release-provenance",
        "release attestation is not bound to published support bytes")
    _require(attestation.get("published_dependencies", {}).get(
        "exact_retained_bytes") is True,
        "release attestation lacks exact published dependency bytes")

    attested_artifacts = attestation.get("artifacts")
    _require(isinstance(attested_artifacts, list) and attested_artifacts,
             "attestation has no artifacts")
    packages = []
    extract_root.mkdir(parents=True, exist_ok=True)
    for index, attested in enumerate(attested_artifacts):
        _require(isinstance(attested, dict), "attestation contains malformed artifact")
        relative_path = attested.get("path")
        _require(isinstance(relative_path, str) and relative_path and not relative_path.startswith("/"),
                 "attestation artifact path must be relative")
        artifact = output_dir / relative_path
        _require(artifact.is_file(), "attested artifact is missing: %s" % artifact)
        destination = extract_root / ("package-%02d" % index)
        _require(not destination.exists(), "refusing stale extraction directory: %s" % destination)
        extractor(artifact, destination)
        packages.append(_artifact_record(artifact, relative_path, destination, attested))

    expected = _expected_packages(architecture, product_version, suite_version)
    by_name = {package["name"]: package for package in packages}
    _require(len(by_name) == len(packages), "release stack contains duplicate package names")
    _require(set(by_name) == set(expected),
             "release stack package set differs: expected %s, observed %s" % (
                 sorted(expected), sorted(by_name)))
    for name, identity in expected.items():
        package = by_name[name]
        _require(package["version"] == identity["version"],
                 "%s version differs from release contract" % name)
        _require(package["subdir"] == identity["subdir"],
                 "%s subdir differs from release contract" % name)

    published_rows = validate_retained_support(
        published_support_proof,
        output_dir,
        suite_lock=suite_lock,
        architecture=architecture,
    )
    for name, published in published_rows.items():
        artifact = by_name[name]["artifact"]
        _require((artifact["path"], artifact["sha256"], artifact["size_bytes"]) == (
            published["retained_artifact"]["path"],
            published["published_artifact"]["sha256"],
            published["published_artifact"]["size_bytes"],
        ), "%s inventory bytes differ from published dependency" % name)

    product_dependencies = set(by_name[PRODUCT_NAME]["depends"])
    for support_name in SUPPORT_NAMES:
        requirement = "%s ==%s" % (support_name, suite_version)
        _require(requirement in product_dependencies,
                 "product package lacks exact dependency %s" % requirement)
    _require("cppyy >=3.5,<4" in product_dependencies,
             "product package lacks the reviewed cppyy runtime constraint")

    return {
        "schema": INVENTORY_SCHEMA,
        "architecture": architecture,
        "product_commit": product_commit,
        "suite": {
            "repository": suite_lock.get("repository"),
            "commit": suite_commit,
            "package_version": suite_version,
        },
        "product": {"name": PRODUCT_NAME, "version": product_version},
        "published_dependencies": [
            {
                "name": row["name"],
                "version": row["version"],
                "build": row["build"],
                "subdir": row["subdir"],
                "sha256": row["published_artifact"]["sha256"],
                "size_bytes": row["published_artifact"]["size_bytes"],
            }
            for row in published_support_proof["packages"]
        ],
        "packages": sorted(packages, key=lambda package: package["name"]),
        "validated": {
            "artifact_hashes": True,
            "conda_identities": True,
            "exact_support_dependencies": True,
            "published_dependency_bytes": True,
            "source_commits": True,
        },
        "performance_claims_allowed": False,
    }


def _spdx_id(prefix: str, value: str) -> str:
    digest = hashlib.sha256(value.encode("utf-8")).hexdigest()[:20]
    return "SPDXRef-%s-%s" % (prefix, digest)


def build_spdx(inventory: dict) -> dict:
    _require(inventory.get("schema") == INVENTORY_SCHEMA, "unsupported inventory schema")
    product = inventory["product"]
    namespace_seed = "%s:%s:%s" % (
        inventory["product_commit"], inventory["architecture"], product["version"])
    packages = []
    files = []
    relationships = []
    package_ids = {}
    for package in inventory["packages"]:
        package_id = _spdx_id("Package", package["name"])
        package_ids[package["name"]] = package_id
        file_ids = []
        for row in package["files"]:
            if row["sha256"] is None:
                continue
            file_id = _spdx_id("File", package["name"] + ":" + row["path"])
            file_ids.append(file_id)
            files.append({
                "SPDXID": file_id,
                "fileName": "./%s/%s" % (package["name"], row["path"]),
                "checksums": [{"algorithm": "SHA256", "checksumValue": row["sha256"]}],
                "licenseConcluded": "NOASSERTION",
                "copyrightText": "NOASSERTION",
            })
        purl = "pkg:conda/%s@%s?build=%s&subdir=%s" % tuple(
            quote(str(package[field]), safe="")
            for field in ("name", "version", "build", "subdir"))
        packages.append({
            "SPDXID": package_id,
            "name": package["name"],
            "versionInfo": package["version"],
            "packageFileName": package["artifact"]["path"],
            "downloadLocation": "NOASSERTION",
            "filesAnalyzed": True,
            "checksums": [{
                "algorithm": "SHA256",
                "checksumValue": package["artifact"]["sha256"],
            }],
            "licenseConcluded": "NOASSERTION",
            "licenseDeclared": package["license"],
            "copyrightText": "NOASSERTION",
            "externalRefs": [{
                "referenceCategory": "PACKAGE-MANAGER",
                "referenceType": "purl",
                "referenceLocator": purl,
            }],
            "hasFiles": sorted(file_ids),
        })
        relationships.append({
            "spdxElementId": "SPDXRef-DOCUMENT",
            "relationshipType": "DESCRIBES",
            "relatedSpdxElement": package_id,
        })
    product_id = package_ids[PRODUCT_NAME]
    for support_name in SUPPORT_NAMES:
        relationships.append({
            "spdxElementId": product_id,
            "relationshipType": "DEPENDS_ON",
            "relatedSpdxElement": package_ids[support_name],
        })
    if "cppyy" in package_ids:
        relationships.append({
            "spdxElementId": product_id,
            "relationshipType": "DEPENDS_ON",
            "relatedSpdxElement": package_ids["cppyy"],
        })
    return {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": "%s-%s-%s" % (
            product["name"], product["version"], inventory["architecture"]),
        "documentNamespace": "https://github.com/awesomebytes/rclcppyy/spdx/%s" % hashlib.sha256(
            namespace_seed.encode("utf-8")).hexdigest(),
        "creationInfo": {
            "created": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "creators": ["Tool: rclcppyy-release-inventory/1"],
        },
        "documentDescribes": sorted(package_ids.values()),
        "packages": sorted(packages, key=lambda package: package["name"]),
        "files": sorted(files, key=lambda row: row["fileName"]),
        "relationships": sorted(
            relationships,
            key=lambda row: (
                row["spdxElementId"], row["relationshipType"], row["relatedSpdxElement"]),
        ),
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
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--extract-root", required=True, type=Path)
    parser.add_argument("--local-attestation", required=True, type=Path)
    parser.add_argument("--published-support-proof", required=True, type=Path)
    parser.add_argument("--suite-lock", required=True, type=Path)
    parser.add_argument("--architecture", required=True, choices=sorted(ARCHITECTURES))
    parser.add_argument("--product-version", required=True)
    parser.add_argument("--product-commit", required=True)
    parser.add_argument("--inventory", required=True, type=Path)
    parser.add_argument("--spdx", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        inventory = build_inventory(
            arguments.output_dir.resolve(),
            arguments.extract_root.resolve(),
            attestation=_load_json(arguments.local_attestation),
            published_support_proof=_load_json(arguments.published_support_proof),
            suite_lock=_load_json(arguments.suite_lock),
            architecture=arguments.architecture,
            product_version=arguments.product_version,
            product_commit=arguments.product_commit,
        )
        spdx = build_spdx(inventory)
        _write_atomic(arguments.inventory, inventory)
        _write_atomic(arguments.spdx, spdx)
    except (OSError, json.JSONDecodeError, subprocess.CalledProcessError, ValueError) as error:
        print("release inventory failed: %s" % error)
        return 1
    print("RELEASE_PACKAGE_INVENTORY_OK packages=%d files=%d" % (
        len(inventory["packages"]), len(spdx["files"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
