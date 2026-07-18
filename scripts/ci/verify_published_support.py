#!/usr/bin/env python3
"""Require exact, provenance-verified support packages before product release."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Callable
from urllib.parse import quote
from urllib.request import Request, urlopen


SCHEMA = "rclcppyy.published-support-proof/v2"
SUITE_LOCK_SCHEMA = "rclcppyy.suite-source/v1"
COMMON_PACKAGES = (
    {
        "name": "cppyy-kit",
        "version_source": "suite",
        "build": "pyh4616a5c_0",
        "subdir": "noarch",
    },
    {
        "name": "ros-jazzy-rclcpp-kit",
        "version_source": "suite",
        "build": "pyh4616a5c_0",
        "subdir": "noarch",
    },
)
ARM_PACKAGE = {
    "name": "cppyy",
    "version": "3.5.0",
    "build": "py312hf18b547_0",
    "subdir": "linux-aarch64",
}
ARCHITECTURES = {
    "x86_64": False,
    "aarch64": True,
    "arm64": True,
}
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


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


def _fetch(url: str) -> bytes:
    request = Request(url, headers={"User-Agent": "rclcppyy-release-gate/1"})
    with urlopen(request, timeout=60) as response:
        return response.read()


def _verify_github_attestation(
    artifact: Path,
    repository: str,
    source_commit: str,
    source_ref: str,
) -> dict:
    signer = "%s/.github/workflows/release.yml" % repository
    command = [
        "gh", "attestation", "verify", str(artifact),
        "--repo", repository,
        "--signer-workflow", signer,
        "--source-digest", source_commit,
        "--source-ref", source_ref,
        "--deny-self-hosted-runners",
        "--format", "json",
    ]
    process = subprocess.run(command, capture_output=True, text=True, check=True)
    value = json.loads(process.stdout)
    _require(isinstance(value, list) and value,
             "%s has no verified GitHub provenance" % artifact)
    return {
        "verified_attestations": len(value),
        "repository": repository,
        "signer_workflow": signer,
        "source_commit": source_commit,
        "source_ref": source_ref,
        "self_hosted_runner_denied": True,
    }


def _published_records(repodata: dict, subdir: str) -> dict[str, dict]:
    _require(repodata.get("info", {}).get("subdir") == subdir,
             "support channel repodata has the wrong subdir")
    records = repodata.get("packages.conda")
    _require(isinstance(records, dict), "support channel has no .conda repodata")
    return records


def expected_packages(architecture: str, suite_version: str) -> tuple[dict, ...]:
    try:
        needs_arm_bridge = ARCHITECTURES[architecture]
    except KeyError as error:
        raise ValueError("unsupported package architecture: %s" % architecture) from error
    packages = []
    for package in COMMON_PACKAGES:
        value = dict(package)
        value["version"] = suite_version
        value.pop("version_source")
        packages.append(value)
    if needs_arm_bridge:
        packages.append(dict(ARM_PACKAGE))
    return tuple(packages)


def validate_retained_support(
    proof: dict,
    download_dir: Path,
    *,
    suite_lock: dict,
    architecture: str,
) -> dict[str, dict]:
    _require(proof.get("schema") == SCHEMA, "unsupported published support proof")
    _require(proof.get("architecture") == architecture,
             "published support proof architecture differs")
    _require(proof.get("suite", {}).get("commit") == suite_lock.get("commit"),
             "published support proof suite commit differs")
    _require(proof.get("suite", {}).get("package_version") ==
             suite_lock.get("package_version"),
             "published support proof suite version differs")
    for field in (
        "available_before_product_build",
        "exact_package_identities",
        "github_provenance",
        "isolated_local_channel",
        "published_bytes_match_repodata",
        "retained_bytes_match_published",
        "suite_source_identity",
    ):
        _require(proof.get("validated", {}).get(field) is True,
                 "published support proof lacks %s" % field)
    expected = {
        package["name"]: package
        for package in expected_packages(architecture, suite_lock["package_version"])
    }
    rows = proof.get("packages")
    _require(isinstance(rows, list), "published support proof has no packages")
    by_name = {row.get("name"): row for row in rows if isinstance(row, dict)}
    _require(len(by_name) == len(rows) and set(by_name) == set(expected),
             "published support proof package set differs")
    for name, package in expected.items():
        row = by_name[name]
        for field in ("name", "version", "build", "subdir"):
            _require(row.get(field) == package[field],
                     "%s published identity differs at %s" % (name, field))
        filename = "%s-%s-%s.conda" % (
            package["name"], package["version"], package["build"])
        _require(row.get("filename") == filename,
                 "%s published filename differs" % name)
        published = row.get("published_artifact", {})
        retained = row.get("retained_artifact", {})
        _require(retained.get("path") == "%s/%s" % (package["subdir"], filename),
                 "%s retained path differs" % name)
        _require((retained.get("sha256"), retained.get("size_bytes")) ==
                 (published.get("sha256"), published.get("size_bytes")),
                 "%s retained identity differs from published bytes" % name)
        artifact = download_dir / retained["path"]
        _require(artifact.is_file(), "%s retained artifact is missing" % name)
        _require((_sha256(artifact), artifact.stat().st_size) ==
                 (published.get("sha256"), published.get("size_bytes")),
                 "%s retained artifact bytes differ" % name)
        provenance = row.get("provenance", {})
        _require(provenance.get("verified_attestations", 0) > 0,
                 "%s has no verified provenance" % name)
        _require(provenance.get("source_commit") == suite_lock.get("commit"),
                 "%s provenance suite commit differs" % name)
        _require(provenance.get("source_ref") ==
                 "refs/tags/v%s" % suite_lock.get("package_version"),
                 "%s provenance suite ref differs" % name)
        if suite_lock.get("repository") is not None:
            _require(provenance.get("repository") == suite_lock.get("repository"),
                     "%s provenance repository differs" % name)
    return by_name


def verify_published_support(
    *,
    suite_lock: dict,
    architecture: str,
    channel_url: str,
    download_dir: Path,
    fetcher: Callable[[str], bytes] = _fetch,
    provenance_verifier: Callable[[Path, str, str, str], dict] = _verify_github_attestation,
) -> dict:
    _require(suite_lock.get("schema") == SUITE_LOCK_SCHEMA,
             "unsupported suite source lock schema")
    repository = suite_lock.get("repository")
    suite_commit = suite_lock.get("commit")
    suite_version = suite_lock.get("package_version")
    _require(isinstance(repository, str) and re.fullmatch(r"[^/]+/[^/]+", repository),
             "suite repository must be an owner/name identity")
    _require(isinstance(suite_commit, str) and COMMIT_RE.fullmatch(suite_commit) is not None,
             "suite commit must be a full lowercase Git commit")
    _require(isinstance(suite_version, str) and suite_version,
             "suite package version is required")
    expected = expected_packages(architecture, suite_version)

    channel_url = channel_url.rstrip("/")
    records_by_subdir = {}
    for subdir in sorted({package["subdir"] for package in expected}):
        repodata_url = "%s/%s/repodata.json" % (channel_url, subdir)
        repodata = json.loads(fetcher(repodata_url).decode("utf-8"))
        _require(isinstance(repodata, dict), "support channel repodata must be an object")
        records_by_subdir[subdir] = _published_records(repodata, subdir)
    source_ref = "refs/tags/v%s" % suite_version
    download_dir.mkdir(parents=True, exist_ok=True)
    results = []
    retained_records = {subdir: {} for subdir in records_by_subdir}
    for package in expected:
        name = package["name"]
        version = package["version"]
        build = package["build"]
        subdir = package["subdir"]
        filename = "%s-%s-%s.conda" % (name, version, build)
        record = records_by_subdir[subdir].get(filename)
        _require(isinstance(record, dict),
                 "support channel lacks exact artifact %s" % filename)
        _require(record.get("name") == name, "%s repodata name differs" % filename)
        _require(record.get("version") == version,
                 "%s repodata version differs" % filename)
        _require(record.get("subdir") == subdir, "%s repodata subdir differs" % filename)
        published_sha = record.get("sha256")
        published_size = record.get("size")
        _require(isinstance(published_sha, str) and re.fullmatch(r"[0-9a-f]{64}", published_sha),
                 "%s repodata has no valid SHA-256" % filename)
        _require(isinstance(published_size, int) and published_size > 0,
                 "%s repodata has no valid size" % filename)
        _require(record.get("build") == build,
                 "%s repodata build identity differs" % filename)

        artifact_url = "%s/%s/%s" % (channel_url, subdir, quote(filename))
        artifact_bytes = fetcher(artifact_url)
        _require(_sha256_bytes(artifact_bytes) == published_sha,
                 "%s downloaded bytes differ from repodata" % filename)
        _require(len(artifact_bytes) == published_size,
                 "%s downloaded size differs from repodata" % filename)
        destination = download_dir / subdir / filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(".%s.%d.tmp" % (filename, os.getpid()))
        temporary.write_bytes(artifact_bytes)
        os.replace(temporary, destination)
        provenance = provenance_verifier(
            destination, repository, suite_commit, source_ref)
        _require(isinstance(provenance, dict) and provenance.get("verified_attestations", 0) > 0,
                 "%s provenance verifier returned no evidence" % filename)
        results.append({
            "name": name,
            "version": version,
            "build": build,
            "subdir": subdir,
            "filename": filename,
            "channel_url": artifact_url,
            "published_artifact": {
                "sha256": published_sha,
                "size_bytes": published_size,
            },
            "retained_artifact": {
                "path": destination.relative_to(download_dir).as_posix(),
                "sha256": _sha256(destination),
                "size_bytes": destination.stat().st_size,
            },
            "provenance": provenance,
        })
        retained_records[subdir][filename] = record
    for subdir, records in retained_records.items():
        _write_atomic(download_dir / subdir / "repodata.json", {
            "info": {"subdir": subdir},
            "packages": {},
            "packages.conda": records,
            "removed": [],
            "repodata_version": 1,
        })
    proof = {
        "schema": SCHEMA,
        "architecture": architecture,
        "channel": channel_url,
        "suite": {
            "repository": repository,
            "commit": suite_commit,
            "package_version": suite_version,
            "source_ref": source_ref,
        },
        "packages": results,
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
    validate_retained_support(
        proof, download_dir, suite_lock=suite_lock, architecture=architecture)
    return proof


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
    parser.add_argument("--suite-lock", required=True, type=Path)
    parser.add_argument("--architecture", required=True, choices=sorted(ARCHITECTURES))
    parser.add_argument("--channel-url", default="https://repo.prefix.dev/awesomebytes")
    parser.add_argument("--download-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        proof = verify_published_support(
            suite_lock=_load_json(arguments.suite_lock),
            architecture=arguments.architecture,
            channel_url=arguments.channel_url,
            download_dir=arguments.download_dir,
        )
        _write_atomic(arguments.output, proof)
    except (
        OSError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
        UnicodeDecodeError,
        ValueError,
    ) as error:
        print("published support verification failed: %s" % error)
        return 1
    print("PUBLISHED_SUPPORT_VERIFIED packages=%d" % len(proof["packages"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
