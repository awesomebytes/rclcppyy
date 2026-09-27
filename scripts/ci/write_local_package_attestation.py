#!/usr/bin/env python3
"""Write source identity and checksums for the local immutable package build."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform

try:
    from .verify_published_support import validate_retained_support
except ImportError:
    from verify_published_support import validate_retained_support


SCHEMA = "rclcppyy.local-package-attestation/v2"
COMMON_ARTIFACT_PATTERNS = (
    "noarch/cppyy-kit-0.2.0-*.conda",
    "noarch/ros-jazzy-rclcpp-kit-0.2.0-*.conda",
)
ARCHITECTURES = {
    "x86_64": ("linux-64", False),
    "aarch64": ("linux-aarch64", True),
    "arm64": ("linux-aarch64", True),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_patterns(architecture: str) -> tuple[str, ...]:
    try:
        subdir, needs_bridge = ARCHITECTURES[architecture]
    except KeyError as error:
        raise ValueError("unsupported package architecture: %s" % architecture) from error
    patterns = COMMON_ARTIFACT_PATTERNS + (
        "%s/ros-jazzy-rclcppyy-0.3.0-*.conda" % subdir,
    )
    if needs_bridge:
        patterns += ("linux-aarch64/cppyy-3.5.0-py312*.conda",)
    return patterns


def collect_artifacts(output_dir: Path, architecture: str) -> list[dict[str, object]]:
    artifacts = []
    for pattern in artifact_patterns(architecture):
        matches = sorted(output_dir.glob(pattern))
        if len(matches) != 1:
            raise ValueError(
                "expected exactly one artifact for %s, found %d"
                % (pattern, len(matches)))
        path = matches[0]
        artifacts.append({
            "path": path.relative_to(output_dir).as_posix(),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        })
    return artifacts


def collect_cppyy_arm_proof(
    output_dir: Path,
    *,
    artifacts: list[dict[str, object]],
    suite_commit: str,
) -> dict[str, object]:
    proof_path = output_dir / "cppyy-arm-package-proof.json"
    runtime_path = output_dir / "cppyy-arm-runtime-proof.log"
    proof = json.loads(proof_path.read_text(encoding="utf-8"))
    if proof.get("schema") != "cppyy-kit.cppyy-package-proof/v1":
        raise ValueError("unsupported cppyy ARM package proof schema")
    snapshot = proof.get("source_snapshot", {})
    if snapshot.get("commit") != suite_commit or snapshot.get("dirty") is not False:
        raise ValueError("cppyy ARM proof is not bound to the clean suite commit")
    host = proof.get("build_host", {})
    runtime = proof.get("runtime_proof", {})
    if host.get("native_arm64") is not True:
        raise ValueError("cppyy ARM proof was not built on native ARM64")
    if runtime.get("native_import_version_and_cppdef") is not True:
        raise ValueError("cppyy ARM proof lacks native import and cppdef evidence")
    if (runtime.get("filename") != runtime_path.name or
            runtime.get("sha256") != _sha256(runtime_path)):
        raise ValueError("cppyy ARM runtime evidence hash mismatch")

    artifact_proof = proof.get("artifact", {})
    artifact_name = artifact_proof.get("filename", "")
    artifact_matches = [
        item for item in artifacts
        if item["path"].endswith("/" + artifact_name)
    ]
    if (not artifact_name.startswith("cppyy-3.5.0-py312") or
            len(artifact_matches) != 1 or
            artifact_matches[0]["sha256"] != artifact_proof.get("sha256")):
        raise ValueError("cppyy ARM proof artifact is absent from the local stack")
    source = proof.get("source", {})
    if source.get("schema") != "cppyy-kit.upstream-package-source/v1":
        raise ValueError("cppyy ARM proof lacks an immutable upstream source lock")
    if source.get("package", {}).get("platform") != "linux-aarch64":
        raise ValueError("cppyy ARM source lock has the wrong package platform")
    return {
        "package_proof": {
            "path": proof_path.relative_to(output_dir).as_posix(),
            "sha256": _sha256(proof_path),
            "size_bytes": proof_path.stat().st_size,
        },
        "runtime_evidence": {
            "path": runtime_path.relative_to(output_dir).as_posix(),
            "sha256": _sha256(runtime_path),
            "size_bytes": runtime_path.stat().st_size,
        },
        "artifact_sha256": artifact_proof["sha256"],
        "source": source,
    }


def build_attestation(
    output_dir: Path,
    *,
    product_commit: str,
    suite_commit: str,
    architecture: str,
    published_support_proof: dict | None = None,
) -> dict[str, object]:
    for name, value in (
        ("product_commit", product_commit),
        ("suite_commit", suite_commit),
    ):
        if len(value) != 40 or any(character not in "0123456789abcdef" for character in value):
            raise ValueError("%s must be a full lowercase Git commit" % name)
    artifacts = collect_artifacts(output_dir, architecture)
    source_method = "detached-git-checkout"
    published_dependencies = None
    arm_bridge = None
    if published_support_proof is not None:
        suite = published_support_proof.get("suite", {})
        retained = validate_retained_support(
            published_support_proof,
            output_dir,
            suite_lock={
                "commit": suite_commit,
                "package_version": suite.get("package_version"),
                "repository": suite.get("repository"),
            },
            architecture=architecture,
        )
        artifact_by_path = {item["path"]: item for item in artifacts}
        for name, row in retained.items():
            path = row["retained_artifact"]["path"]
            artifact = artifact_by_path.get(path)
            if (artifact is None or
                    (artifact["sha256"], artifact["size_bytes"]) !=
                    (row["published_artifact"]["sha256"],
                     row["published_artifact"]["size_bytes"])):
                raise ValueError(
                    "%s retained published bytes are absent from package stack" % name)
        source_method = "published-release-provenance"
        published_dependencies = {
            "schema": published_support_proof["schema"],
            "exact_retained_bytes": True,
            "packages": [
                {
                    "name": row["name"],
                    "version": row["version"],
                    "build": row["build"],
                    "subdir": row["subdir"],
                    "path": row["retained_artifact"]["path"],
                    "sha256": row["published_artifact"]["sha256"],
                    "size_bytes": row["published_artifact"]["size_bytes"],
                }
                for row in published_support_proof["packages"]
            ],
        }
        if ARCHITECTURES.get(architecture, (None, False))[1]:
            row = retained["cppyy"]
            arm_bridge = {
                "artifact_sha256": row["published_artifact"]["sha256"],
                "published": True,
                "provenance": row["provenance"],
            }
    elif ARCHITECTURES.get(architecture, (None, False))[1]:
        arm_bridge = collect_cppyy_arm_proof(
            output_dir, artifacts=artifacts, suite_commit=suite_commit)
    return {
        "schema": SCHEMA,
        "architecture": architecture,
        "source_snapshots": {
            "rclcppyy": {
                "commit": product_commit,
                "method": "git-archive",
            },
            "cppyy_kit": {
                "commit": suite_commit,
                "method": source_method,
            },
        },
        "artifacts": artifacts,
        "published_dependencies": published_dependencies,
        "cppyy_arm_bridge": arm_bridge,
        "signed": False,
        "performance_claims_allowed": False,
    }


def _write_atomic(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp.%d" % os.getpid())
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--product-commit", required=True)
    parser.add_argument("--suite-commit", required=True)
    parser.add_argument("--published-support-proof", type=Path)
    parser.add_argument("--attestation", type=Path, required=True)
    arguments = parser.parse_args()

    value = build_attestation(
        arguments.output_dir.resolve(),
        product_commit=arguments.product_commit,
        suite_commit=arguments.suite_commit,
        architecture=platform.machine(),
        published_support_proof=(
            json.loads(arguments.published_support_proof.read_text(encoding="utf-8"))
            if arguments.published_support_proof is not None else None
        ),
    )
    _write_atomic(arguments.attestation, value)
    print("LOCAL_PACKAGE_ATTESTATION_OK %s" % arguments.attestation)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
