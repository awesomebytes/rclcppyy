#!/usr/bin/env python3
"""Write source identity and checksums for the local immutable package build."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform


SCHEMA = "rclcppyy.local-package-attestation/v1"
ARTIFACT_PATTERNS = (
    "noarch/cppyy-kit-0.2.0-*.conda",
    "noarch/ros-jazzy-rclcpp-kit-0.2.0-*.conda",
    "linux-64/ros-jazzy-rclcppyy-0.3.0-*.conda",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def collect_artifacts(output_dir: Path) -> list[dict[str, object]]:
    artifacts = []
    for pattern in ARTIFACT_PATTERNS:
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


def build_attestation(
    output_dir: Path,
    *,
    product_commit: str,
    suite_commit: str,
    architecture: str,
) -> dict[str, object]:
    for name, value in (
        ("product_commit", product_commit),
        ("suite_commit", suite_commit),
    ):
        if len(value) != 40 or any(character not in "0123456789abcdef" for character in value):
            raise ValueError("%s must be a full lowercase Git commit" % name)
    return {
        "schema": SCHEMA,
        "architecture": architecture,
        "source_snapshots": {
            "method": "git-archive",
            "rclcppyy_commit": product_commit,
            "cppyy_kit_commit": suite_commit,
        },
        "artifacts": collect_artifacts(output_dir),
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
    parser.add_argument("--attestation", type=Path, required=True)
    arguments = parser.parse_args()

    value = build_attestation(
        arguments.output_dir.resolve(),
        product_commit=arguments.product_commit,
        suite_commit=arguments.suite_commit,
        architecture=platform.machine(),
    )
    _write_atomic(arguments.attestation, value)
    print("LOCAL_PACKAGE_ATTESTATION_OK %s" % arguments.attestation)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
