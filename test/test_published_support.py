import hashlib
import json
from pathlib import Path

import pytest

from scripts.ci import verify_published_support as support


SUITE_COMMIT = "a" * 40
SUITE_VERSION = "0.2.0"
CHANNEL = "https://packages.example.invalid/channel"


def _verified_provenance(*_args):
    return {
        "verified_attestations": 1,
        "repository": "awesomebytes/cppyy_kit",
        "source_commit": SUITE_COMMIT,
        "source_ref": "refs/tags/v0.2.0",
    }


def _fixture(architecture="x86_64"):
    packages = support.expected_packages(architecture, SUITE_VERSION)
    files = {}
    repodata = {}
    for package in packages:
        filename = "%s-%s-%s.conda" % (
            package["name"], package["version"], package["build"])
        content = ("published:" + filename).encode()
        files[(package["subdir"], filename)] = content
        records = repodata.setdefault(package["subdir"], {})
        records[filename] = {
            **package,
            "sha256": hashlib.sha256(content).hexdigest(),
            "size": len(content),
        }
    suite_lock = {
        "schema": support.SUITE_LOCK_SCHEMA,
        "repository": "awesomebytes/cppyy_kit",
        "commit": SUITE_COMMIT,
        "package_version": SUITE_VERSION,
    }

    def fetch(url):
        if url.endswith("repodata.json"):
            subdir = Path(url).parent.name
            return json.dumps({
                "info": {"subdir": subdir},
                "packages.conda": repodata[subdir],
            }).encode()
        subdir = Path(url).parent.name
        return files[(subdir, Path(url).name)]

    return suite_lock, repodata, files, fetch


def test_published_support_retains_exact_x86_bytes_and_locked_provenance(tmp_path):
    suite_lock, _repodata, files, fetch = _fixture()
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
        suite_lock=suite_lock,
        architecture="x86_64",
        channel_url=CHANNEL + "/",
        download_dir=tmp_path,
        fetcher=fetch,
        provenance_verifier=verify,
    )

    assert proof["validated"] == {
        "available_before_product_build": True,
        "exact_package_identities": True,
        "github_provenance": True,
        "isolated_local_channel": True,
        "published_bytes_match_repodata": True,
        "retained_bytes_match_published": True,
        "suite_source_identity": True,
    }
    assert [item["name"] for item in proof["packages"]] == [
        "cppyy-kit", "ros-jazzy-rclcpp-kit"]
    retained = {
        (path.parent.name, path.name): path.read_bytes()
        for path in tmp_path.glob("*/*.conda")
    }
    assert retained == files
    for subdir in {item[0] for item in files}:
        repodata = json.loads((tmp_path / subdir / "repodata.json").read_text())
        assert set(repodata["packages.conda"]) == {
            filename for package_subdir, filename in files if package_subdir == subdir}
    assert all(
        item["retained_artifact"]["sha256"]
        == item["published_artifact"]["sha256"]
        for item in proof["packages"])
    assert len(calls) == 2
    assert all(call[1:] == (
        "awesomebytes/cppyy_kit", SUITE_COMMIT, "refs/tags/v0.2.0")
        for call in calls)


def test_published_support_includes_exact_native_arm_bridge(tmp_path):
    suite_lock, _repodata, _files, fetch = _fixture("aarch64")

    proof = support.verify_published_support(
        suite_lock=suite_lock,
        architecture="aarch64",
        channel_url=CHANNEL,
        download_dir=tmp_path,
        fetcher=fetch,
        provenance_verifier=_verified_provenance,
    )

    arm = next(item for item in proof["packages"] if item["name"] == "cppyy")
    assert (arm["version"], arm["build"], arm["subdir"]) == (
        "3.5.0", "py312hf18b547_0", "linux-aarch64")
    assert (tmp_path / arm["retained_artifact"]["path"]).is_file()


def test_published_support_rejects_download_that_differs_from_repodata(tmp_path):
    suite_lock, repodata, _files, fetch = _fixture()
    filename = "cppyy-kit-0.2.0-pyh4616a5c_0.conda"
    repodata["noarch"][filename]["sha256"] = "0" * 64

    with pytest.raises(ValueError, match="downloaded bytes differ from repodata"):
        support.verify_published_support(
            suite_lock=suite_lock,
            architecture="x86_64",
            channel_url=CHANNEL,
            download_dir=tmp_path,
            fetcher=fetch,
            provenance_verifier=_verified_provenance,
        )


def test_published_support_rejects_missing_identity_or_provenance(tmp_path):
    suite_lock, repodata, _files, fetch = _fixture()
    repodata["noarch"].pop("cppyy-kit-0.2.0-pyh4616a5c_0.conda")
    with pytest.raises(ValueError, match="lacks exact artifact"):
        support.verify_published_support(
            suite_lock=suite_lock,
            architecture="x86_64",
            channel_url=CHANNEL,
            download_dir=tmp_path,
            fetcher=fetch,
            provenance_verifier=_verified_provenance,
        )

    suite_lock, _repodata, _files, fetch = _fixture()
    with pytest.raises(ValueError, match="returned no evidence"):
        support.verify_published_support(
            suite_lock=suite_lock,
            architecture="x86_64",
            channel_url=CHANNEL,
            download_dir=tmp_path / "no-provenance",
            fetcher=fetch,
            provenance_verifier=lambda *_args: {"verified_attestations": 0},
        )
