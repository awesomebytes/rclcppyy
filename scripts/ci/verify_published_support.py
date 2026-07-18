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


SCHEMA = "rclcppyy.published-support-proof/v1"
ATTESTATION_SCHEMA = "rclcppyy.local-package-attestation/v2"
SUITE_LOCK_SCHEMA = "rclcppyy.suite-source/v1"
SUPPORT_NAMES = ("cppyy-kit", "ros-jazzy-rclcpp-kit")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


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


def _published_records(repodata: dict) -> dict[str, dict]:
    _require(repodata.get("info", {}).get("subdir") == "noarch",
             "support channel repodata has the wrong subdir")
    records = repodata.get("packages.conda")
    _require(isinstance(records, dict), "support channel has no .conda repodata")
    return records


def _local_support(attestation: dict, suite_version: str) -> dict[str, dict]:
    _require(attestation.get("schema") == ATTESTATION_SCHEMA,
             "unsupported local package attestation schema")
    artifacts = attestation.get("artifacts")
    _require(isinstance(artifacts, list), "local package attestation has no artifacts")
    support = {}
    for item in artifacts:
        _require(isinstance(item, dict), "local package attestation has malformed artifacts")
        path = item.get("path")
        if not isinstance(path, str) or not path.startswith("noarch/"):
            continue
        for name in SUPPORT_NAMES:
            prefix = "%s-%s-" % (name, suite_version)
            if Path(path).name.startswith(prefix):
                _require(name not in support, "local stack has duplicate %s artifacts" % name)
                support[name] = item
    _require(set(support) == set(SUPPORT_NAMES),
             "local stack lacks the exact support package set")
    return support


def verify_published_support(
    *,
    attestation: dict,
    suite_lock: dict,
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
    snapshot = attestation.get("source_snapshots", {}).get("cppyy_kit", {})
    _require(snapshot.get("commit") == suite_commit,
             "local support artifacts are not bound to the locked suite commit")
    support = _local_support(attestation, suite_version)

    channel_url = channel_url.rstrip("/")
    repodata_url = channel_url + "/noarch/repodata.json"
    repodata = json.loads(fetcher(repodata_url).decode("utf-8"))
    _require(isinstance(repodata, dict), "support channel repodata must be an object")
    records = _published_records(repodata)
    source_ref = "refs/tags/v%s" % suite_version
    download_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for name in SUPPORT_NAMES:
        local = support[name]
        filename = Path(local["path"]).name
        record = records.get(filename)
        _require(isinstance(record, dict),
                 "support channel lacks exact artifact %s" % filename)
        _require(record.get("name") == name, "%s repodata name differs" % filename)
        _require(record.get("version") == suite_version,
                 "%s repodata version differs" % filename)
        _require(record.get("subdir") == "noarch", "%s repodata subdir differs" % filename)
        published_sha = record.get("sha256")
        published_size = record.get("size")
        _require(isinstance(published_sha, str) and re.fullmatch(r"[0-9a-f]{64}", published_sha),
                 "%s repodata has no valid SHA-256" % filename)
        _require(isinstance(published_size, int) and published_size > 0,
                 "%s repodata has no valid size" % filename)
        build = record.get("build")
        _require(isinstance(build, str) and filename == "%s-%s-%s.conda" % (
            name, suite_version, build), "%s repodata build identity differs" % filename)

        artifact_url = "%s/noarch/%s" % (channel_url, quote(filename))
        artifact_bytes = fetcher(artifact_url)
        _require(_sha256_bytes(artifact_bytes) == published_sha,
                 "%s downloaded bytes differ from repodata" % filename)
        _require(len(artifact_bytes) == published_size,
                 "%s downloaded size differs from repodata" % filename)
        destination = download_dir / filename
        temporary = destination.with_name(".%s.%d.tmp" % (filename, os.getpid()))
        temporary.write_bytes(artifact_bytes)
        os.replace(temporary, destination)
        provenance = provenance_verifier(
            destination, repository, suite_commit, source_ref)
        _require(isinstance(provenance, dict) and provenance.get("verified_attestations", 0) > 0,
                 "%s provenance verifier returned no evidence" % filename)
        results.append({
            "name": name,
            "version": suite_version,
            "build": build,
            "subdir": "noarch",
            "filename": filename,
            "channel_url": artifact_url,
            "local_proof_artifact": {
                "sha256": local["sha256"],
                "size_bytes": local["size_bytes"],
            },
            "published_artifact": {
                "sha256": published_sha,
                "size_bytes": published_size,
            },
            "same_suite_source_and_build_identity": True,
            "provenance": provenance,
        })
    return {
        "schema": SCHEMA,
        "channel": channel_url,
        "suite": {
            "repository": repository,
            "commit": suite_commit,
            "package_version": suite_version,
            "source_ref": source_ref,
        },
        "packages": results,
        "validated": {
            "available_before_product_publication": True,
            "github_provenance": True,
            "published_bytes_match_repodata": True,
            "same_suite_source_and_build_identity": True,
            "suite_source_identity": True,
        },
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
    parser.add_argument("--local-attestation", required=True, type=Path)
    parser.add_argument("--suite-lock", required=True, type=Path)
    parser.add_argument("--channel-url", default="https://repo.prefix.dev/awesomebytes")
    parser.add_argument("--download-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        proof = verify_published_support(
            attestation=_load_json(arguments.local_attestation),
            suite_lock=_load_json(arguments.suite_lock),
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
