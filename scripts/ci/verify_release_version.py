#!/usr/bin/env python3
"""Reject release tags that disagree with product and dependency metadata."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import tomllib
import xml.etree.ElementTree as ET


VERSION_RE = re.compile(r"[0-9]+\.[0-9]+\.[0-9]+")


def _recipe_version(path: Path) -> str:
    match = re.search(
        r"^  version: \"([^\"]+)\"$",
        path.read_text(encoding="utf-8"),
        flags=re.MULTILINE,
    )
    if match is None:
        raise ValueError("release recipe has no context version")
    return match.group(1)


def metadata_versions(repo_root: Path) -> dict[str, str]:
    with (repo_root / "pixi.toml").open("rb") as stream:
        pixi_version = str(tomllib.load(stream)["workspace"]["version"])
    package_version = ET.parse(repo_root / "package.xml").getroot().findtext("version")
    return {
        "pixi.toml": pixi_version,
        "package.xml": str(package_version),
        "recipe/recipe.yaml": _recipe_version(repo_root / "recipe" / "recipe.yaml"),
    }


def verify(repo_root: Path, tag: str) -> str:
    versions = metadata_versions(repo_root)
    distinct = set(versions.values())
    if len(distinct) != 1:
        raise ValueError("product version metadata disagrees: %s" % versions)
    version = distinct.pop()
    if not VERSION_RE.fullmatch(version):
        raise ValueError("product version is not strict X.Y.Z: %s" % version)
    expected_tag = "v" + version
    if tag != expected_tag:
        raise ValueError(
            "release tag mismatch: expected %s from metadata, observed %s" % (
                expected_tag, tag))

    suite_lock = (repo_root / "suite-source.lock.json").read_text(encoding="utf-8")
    suite_version_match = re.search(r'"package_version": "([^\"]+)"', suite_lock)
    recipe = (repo_root / "recipe" / "recipe.yaml").read_text(encoding="utf-8")
    if suite_version_match is None:
        raise ValueError("suite source lock has no package version")
    suite_version = suite_version_match.group(1)
    required = {
        "ros-jazzy-rclcpp-kit ==%s" % suite_version,
        "cppyy-kit ==%s" % suite_version,
    }
    missing = sorted(value for value in required if value not in recipe)
    if missing:
        raise ValueError("release recipe suite pins disagree with lock: %s" % missing)
    return version


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tag")
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[2]
    version = verify(repo_root, args.tag)
    print("RELEASE_VERSION_OK %s" % version)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
