#!/usr/bin/env python3
"""Plan and verify retry-safe product uploads to the release channel."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time
import urllib.error
import urllib.parse
import urllib.request


SCHEMA = "rclcppyy.product-upload-proof/v1"
PRODUCT_NAME = "ros-jazzy-rclcppyy"
EXPECTED_SUBDIRS = {"linux-64", "linux-aarch64"}
TAG_RE = re.compile(r"^v([0-9]+\.[0-9]+\.[0-9]+)$")


class UploadVerificationError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _remote_evidence(url: str, *, opener=urllib.request.urlopen) -> dict | None:
    request = urllib.request.Request(
        url, headers={"User-Agent": "rclcppyy-release-verifier/1"})
    digest = hashlib.sha256()
    size = 0
    try:
        with opener(request, timeout=120) as response:
            resolved_url = response.geturl()
            for chunk in iter(lambda: response.read(1024 * 1024), b""):
                digest.update(chunk)
                size += len(chunk)
    except urllib.error.HTTPError as error:
        if error.code == 404:
            return None
        raise UploadVerificationError(
            "remote artifact query failed with HTTP %d: %s" % (error.code, url)
        ) from error
    except urllib.error.URLError as error:
        raise UploadVerificationError(
            "remote artifact query failed: %s: %s" % (url, error.reason)
        ) from error
    return {
        "resolved_url": resolved_url,
        "sha256": digest.hexdigest(),
        "size_bytes": size,
    }


def verify_product_artifacts(
    artifacts: list[Path],
    *,
    channel_url: str,
    tag: str,
    require_present: bool,
    attempts: int = 1,
    delay_seconds: float = 0.0,
    opener=urllib.request.urlopen,
) -> dict:
    match = TAG_RE.fullmatch(tag)
    if match is None:
        raise UploadVerificationError("release tag must be an exact stable version")
    if attempts < 1:
        raise UploadVerificationError("attempts must be positive")
    version = match.group(1)
    records = []
    observed_subdirs = set()
    for artifact in sorted(path.resolve() for path in artifacts):
        if not artifact.is_file():
            raise UploadVerificationError(
                "local product artifact does not exist: %s" % artifact)
        subdir = artifact.parent.name
        if subdir not in EXPECTED_SUBDIRS or subdir in observed_subdirs:
            raise UploadVerificationError(
                "product matrix must contain one artifact per native subdir")
        prefix = "%s-%s-" % (PRODUCT_NAME, version)
        if not artifact.name.startswith(prefix) or not artifact.name.endswith(".conda"):
            raise UploadVerificationError(
                "unexpected product artifact identity: %s" % artifact.name)
        observed_subdirs.add(subdir)
        filename = urllib.parse.quote(artifact.name, safe="-._~")
        url = "%s/%s/%s" % (channel_url.rstrip("/"), subdir, filename)
        local = {
            "sha256": _sha256(artifact),
            "size_bytes": artifact.stat().st_size,
        }
        remote = None
        for attempt in range(1, attempts + 1):
            remote = _remote_evidence(url, opener=opener)
            if remote is not None or not require_present or attempt == attempts:
                break
            time.sleep(delay_seconds)
        if remote is None:
            if require_present:
                raise UploadVerificationError(
                    "uploaded product artifact is still absent: %s" % url)
            status = "missing"
        elif (remote["sha256"], remote["size_bytes"]) != (
                local["sha256"], local["size_bytes"]):
            raise UploadVerificationError(
                "remote product identity already exists with different bytes: %s/%s; "
                "increment the recipe build number or repair the channel" %
                (subdir, artifact.name))
        else:
            status = "exact"
        records.append({
            "artifact": str(artifact),
            "filename": artifact.name,
            "local": local,
            "remote": remote,
            "remote_url": url,
            "status": status,
            "subdir": subdir,
        })
    if observed_subdirs != EXPECTED_SUBDIRS:
        raise UploadVerificationError(
            "product matrix must contain exactly linux-64 and linux-aarch64")
    return {
        "schema": SCHEMA,
        "channel_url": channel_url.rstrip("/"),
        "tag": tag,
        "version": version,
        "require_present": require_present,
        "artifacts": records,
        "all_exact": all(record["status"] == "exact" for record in records),
    }


def _write_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.%d.tmp" % (path.name, os.getpid()))
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _write_missing(path: Path, document: dict) -> None:
    missing = [
        record["artifact"] + "\n"
        for record in document["artifacts"] if record["status"] == "missing"
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(missing), encoding="utf-8")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", nargs="+", type=Path)
    parser.add_argument("--tag", required=True)
    parser.add_argument(
        "--channel-url", default="https://repo.prefix.dev/awesomebytes")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--missing-output", type=Path)
    parser.add_argument("--require-present", action="store_true")
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--delay-seconds", type=float, default=0.0)
    arguments = parser.parse_args(argv)
    if arguments.require_present and arguments.missing_output is not None:
        parser.error("--missing-output cannot be used with --require-present")
    try:
        document = verify_product_artifacts(
            arguments.artifacts,
            channel_url=arguments.channel_url,
            tag=arguments.tag,
            require_present=arguments.require_present,
            attempts=arguments.attempts,
            delay_seconds=arguments.delay_seconds,
        )
        _write_json(arguments.output, document)
        if arguments.missing_output is not None:
            _write_missing(arguments.missing_output, document)
    except (OSError, UploadVerificationError) as error:
        print("product upload verification failed: %s" % error)
        return 1
    print("PRODUCT_UPLOAD_VERIFIED exact=%d missing=%d" % (
        sum(row["status"] == "exact" for row in document["artifacts"]),
        sum(row["status"] == "missing" for row in document["artifacts"]),
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
