import hashlib
import json
from pathlib import Path

import pytest

from scripts.ci import verify_published_support as support


SUITE_COMMIT = "a" * 40
SUITE_VERSION = "0.2.0"
CHANNEL = "https://packages.example.invalid/channel"


def _fixture():
    local_files = {
        "cppyy-kit-0.2.0-py_fixture_0.conda": b"cppyy-kit-package",
        "ros-jazzy-rclcpp-kit-0.2.0-py_fixture_0.conda": b"rclcpp-kit-package",
    }
    published_files = {
        name: b"published-build:" + content for name, content in local_files.items()
    }
    artifacts = []
    records = {}
    for filename, content in local_files.items():
        name = filename.removesuffix("-0.2.0-py_fixture_0.conda")
        artifacts.append({
            "path": "noarch/" + filename,
            "sha256": hashlib.sha256(content).hexdigest(),
            "size_bytes": len(content),
        })
        published = published_files[filename]
        records[filename] = {
            "name": name,
            "version": SUITE_VERSION,
            "build": "py_fixture_0",
            "subdir": "noarch",
            "sha256": hashlib.sha256(published).hexdigest(),
            "size": len(published),
        }
    attestation = {
        "schema": support.ATTESTATION_SCHEMA,
        "source_snapshots": {"cppyy_kit": {"commit": SUITE_COMMIT}},
        "artifacts": artifacts,
    }
    suite_lock = {
        "schema": support.SUITE_LOCK_SCHEMA,
        "repository": "awesomebytes/cppyy_kit",
        "commit": SUITE_COMMIT,
        "package_version": SUITE_VERSION,
    }
    repodata = {"info": {"subdir": "noarch"}, "packages.conda": records}

    def fetch(url):
        if url.endswith("repodata.json"):
            return json.dumps(repodata).encode()
        return published_files[Path(url).name]

    return attestation, suite_lock, repodata, published_files, fetch


def test_published_support_requires_exact_bytes_and_locked_provenance(tmp_path):
    attestation, suite_lock, _repodata, files, fetch = _fixture()
    calls = []

    def verify(path, repository, source_commit, source_ref):
        calls.append((path.name, repository, source_commit, source_ref))
        return {
            "verified_attestations": 1,
            "repository": repository,
            "source_commit": source_commit,
            "source_ref": source_ref,
        }

    proof = support.verify_published_support(
        attestation=attestation,
        suite_lock=suite_lock,
        channel_url=CHANNEL + "/",
        download_dir=tmp_path,
        fetcher=fetch,
        provenance_verifier=verify,
    )

    assert proof["validated"] == {
        "available_before_product_publication": True,
        "github_provenance": True,
        "published_bytes_match_repodata": True,
        "same_suite_source_and_build_identity": True,
        "suite_source_identity": True,
    }
    assert [item["name"] for item in proof["packages"]] == list(support.SUPPORT_NAMES)
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == files
    assert all(
        item["local_proof_artifact"]["sha256"]
        != item["published_artifact"]["sha256"]
        for item in proof["packages"])
    assert calls == [
        ("cppyy-kit-0.2.0-py_fixture_0.conda", "awesomebytes/cppyy_kit",
         SUITE_COMMIT, "refs/tags/v0.2.0"),
        ("ros-jazzy-rclcpp-kit-0.2.0-py_fixture_0.conda", "awesomebytes/cppyy_kit",
         SUITE_COMMIT, "refs/tags/v0.2.0"),
    ]


def test_published_support_rejects_download_that_differs_from_repodata(tmp_path):
    attestation, suite_lock, repodata, _files, fetch = _fixture()
    filename = "cppyy-kit-0.2.0-py_fixture_0.conda"
    repodata["packages.conda"][filename]["sha256"] = "0" * 64

    with pytest.raises(ValueError, match="downloaded bytes differ from repodata"):
        support.verify_published_support(
            attestation=attestation,
            suite_lock=suite_lock,
            channel_url=CHANNEL,
            download_dir=tmp_path,
            fetcher=fetch,
            provenance_verifier=lambda *_args: {"verified_attestations": 1},
        )


def test_published_support_rejects_unverified_or_wrong_source(tmp_path):
    attestation, suite_lock, _repodata, _files, fetch = _fixture()
    wrong_lock = dict(suite_lock, commit="b" * 40)
    with pytest.raises(ValueError, match="not bound to the locked suite commit"):
        support.verify_published_support(
            attestation=attestation,
            suite_lock=wrong_lock,
            channel_url=CHANNEL,
            download_dir=tmp_path,
            fetcher=fetch,
            provenance_verifier=lambda *_args: {"verified_attestations": 1},
        )

    with pytest.raises(ValueError, match="returned no evidence"):
        support.verify_published_support(
            attestation=attestation,
            suite_lock=suite_lock,
            channel_url=CHANNEL,
            download_dir=tmp_path / "no-provenance",
            fetcher=fetch,
            provenance_verifier=lambda *_args: {"verified_attestations": 0},
        )
