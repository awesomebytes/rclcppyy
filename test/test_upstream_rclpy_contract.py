import copy
import hashlib
import json
from pathlib import Path
import subprocess

import pytest

from scripts.ci import run_upstream_rclpy_contract as contract


def _run(*args, cwd):
    return subprocess.run(
        list(args),
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    ).stdout.strip()


def _fake_checkout(tmp_path):
    source = tmp_path / "upstream"
    test_root = source / "rclpy" / "test"
    test_root.mkdir(parents=True)
    (source / "rclpy" / "package.xml").write_text(
        "<package><version>1.2.3</version></package>\n",
        encoding="utf-8",
    )
    (test_root / "test_alpha.py").write_text(
        "def test_alpha():\n    assert True\n",
        encoding="utf-8",
    )
    (test_root / "test_beta.py").write_text(
        "def test_beta():\n    assert True\n",
        encoding="utf-8",
    )
    _run("git", "init", cwd=source)
    _run("git", "config", "user.name", "Contract Test", cwd=source)
    _run("git", "config", "user.email", "contract@example.invalid", cwd=source)
    repository = "https://example.invalid/rclpy.git"
    _run("git", "remote", "add", "origin", repository, cwd=source)
    _run("git", "add", ".", cwd=source)
    _run("git", "commit", "-m", "fixture", cwd=source)
    revision = _run("git", "rev-parse", "HEAD", cwd=source)
    paths, digest = contract._inventory(test_root, "test_*.py")
    assert paths == ["test_alpha.py", "test_beta.py"]
    manifest = {
        "schema_version": 1,
        "contract_id": "fixture-v1",
        "source": {
            "repository": repository,
            "revision": revision,
            "release": "1.2.3",
            "package_version": "1.2.3",
            "test_root": "rclpy/test",
        },
        "inventory": {
            "algorithm": "sha256-path-content-v1",
            "file_pattern": "test_*.py",
            "file_count": 2,
            "digest": digest,
        },
        "selection": [{
            "path": "test_alpha.py",
            "reason": "Selected to exercise the representative alpha contract.",
        }],
        "reviewed_exclusions": [{
            "id": "deferred-beta",
            "paths": ["test_beta.py"],
            "reason": "Reviewed and explicitly deferred from this bounded contract slice.",
        }],
    }
    return source, manifest


def _write_manifest(tmp_path, manifest):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_repository_manifest_has_exact_review_partition():
    manifest = contract.load_manifest(
        Path("compatibility/upstream-rclpy-contract.json")
    )

    assert manifest["source"]["revision"] == (
        "baf9d72cfa127e391a89b4ab51ba9e55c37041fd"
    )
    assert manifest["source"]["package_version"] == "7.1.11"
    selected = {entry["path"] for entry in manifest["selection"]}
    excluded = {
        path
        for group in manifest["reviewed_exclusions"]
        for path in group["paths"]
    }
    assert len(selected) == 49
    assert len(excluded) == 3
    assert not selected & excluded
    assert len(selected | excluded) == manifest["inventory"]["file_count"]
    assert len(manifest["support_files"]) == 5
    publisher = next(
        entry for entry in manifest["selection"]
        if entry["path"] == "test_publisher.py"
    )
    assert publisher["expected_publish_backend"] == "python"
    stock_failure = next(
        group for group in manifest["reviewed_exclusions"]
        if group["id"] == "fails-with-stock-installed-extension"
    )
    assert stock_failure["paths"] == [
        "test_destruction_order.py",
        "test_type_description_service.py",
    ]


def test_validate_source_accepts_exact_clean_checkout(tmp_path):
    source, manifest_data = _fake_checkout(tmp_path)
    manifest = contract.load_manifest(_write_manifest(tmp_path, manifest_data))

    report = contract.validate_source(manifest, source)

    assert report["revision"] == manifest_data["source"]["revision"]
    assert report["inventory_file_count"] == 2
    assert report["support_file_count"] == 0
    assert report["selected_files"] == 1
    assert report["excluded_files"] == 1


@pytest.mark.parametrize("drift", ["content", "revision", "repository"])
def test_validate_source_fails_closed_on_source_drift(tmp_path, drift):
    source, manifest_data = _fake_checkout(tmp_path)
    manifest = contract.load_manifest(_write_manifest(tmp_path, manifest_data))

    if drift == "content":
        (source / "rclpy" / "test" / "test_alpha.py").write_text(
            "def test_alpha():\n    assert False\n",
            encoding="utf-8",
        )
    elif drift == "revision":
        (source / "README.md").write_text("new commit\n", encoding="utf-8")
        _run("git", "add", "README.md", cwd=source)
        _run("git", "commit", "-m", "drift", cwd=source)
    else:
        _run(
            "git",
            "remote",
            "set-url",
            "origin",
            "https://example.invalid/not-rclpy.git",
            cwd=source,
        )

    with pytest.raises(contract.ContractError):
        contract.validate_source(manifest, source)


def test_validate_source_rejects_incomplete_review_partition(tmp_path):
    source, manifest_data = _fake_checkout(tmp_path)
    invalid = copy.deepcopy(manifest_data)
    invalid["reviewed_exclusions"][0]["paths"] = ["test_gamma.py"]
    manifest = contract.load_manifest(_write_manifest(tmp_path, invalid))

    with pytest.raises(contract.ContractError, match="partition"):
        contract.validate_source(manifest, source)


def test_validate_source_rejects_support_file_drift(tmp_path):
    source, manifest_data = _fake_checkout(tmp_path)
    support = source / "rclpy" / "test" / "helper.py"
    support.write_text("VALUE = 1\n", encoding="utf-8")
    _run("git", "add", ".", cwd=source)
    _run("git", "commit", "-m", "support", cwd=source)
    manifest_data["source"]["revision"] = _run("git", "rev-parse", "HEAD", cwd=source)
    manifest_data["support_files"] = [{
        "path": "helper.py",
        "digest": "0" * hashlib.sha256().digest_size * 2,
    }]
    manifest = contract.load_manifest(_write_manifest(tmp_path, manifest_data))

    with pytest.raises(contract.ContractError, match="support file content drift"):
        contract.validate_source(manifest, source)


def test_load_manifest_rejects_unsafe_support_path(tmp_path):
    _, manifest_data = _fake_checkout(tmp_path)
    manifest_data["support_files"] = [{
        "path": "../helper.py",
        "digest": "0" * 64,
    }]

    with pytest.raises(contract.ContractError, match="safe relative path"):
        contract.load_manifest(_write_manifest(tmp_path, manifest_data))


def test_load_manifest_rejects_short_revision(tmp_path):
    _, manifest_data = _fake_checkout(tmp_path)
    invalid = copy.deepcopy(manifest_data)
    invalid["source"]["revision"] = "main"

    with pytest.raises(contract.ContractError, match="full lowercase Git commit"):
        contract.load_manifest(_write_manifest(tmp_path, invalid))


def test_inventory_digest_binds_paths_and_contents(tmp_path):
    root = tmp_path / "tests"
    root.mkdir()
    original = root / "test_alpha.py"
    original.write_text("def test_alpha(): pass\n", encoding="utf-8")
    _, first = contract._inventory(root, "test_*.py")

    original.rename(root / "test_renamed.py")
    _, renamed = contract._inventory(root, "test_*.py")
    assert renamed != first

    changed = root / "test_renamed.py"
    changed.write_text("def test_alpha(): assert False\n", encoding="utf-8")
    _, modified = contract._inventory(root, "test_*.py")
    assert modified != renamed
    assert len({first, renamed, modified}) == 3
    assert all(len(value) == hashlib.sha256().digest_size * 2 for value in {
        first, renamed, modified
    })


def test_validate_runtime_version_fails_closed(monkeypatch):
    manifest = {"source": {"package_version": "7.1.11"}}
    monkeypatch.setattr(contract.importlib.metadata, "version", lambda _: "7.1.12")

    with pytest.raises(contract.ContractError, match="installed rclpy version drift"):
        contract.validate_runtime_version(manifest)


def test_select_entries_preserves_reviewed_order_and_rejects_unknown():
    manifest = {
        "selection": [
            {"path": "test_alpha.py"},
            {"path": "test_beta.py"},
            {"path": "test_gamma.py"},
        ],
    }

    assert contract._select_entries(
        manifest, ["test_gamma.py", "test_alpha.py"]
    ) == [
        {"path": "test_alpha.py"},
        {"path": "test_gamma.py"},
    ]
    with pytest.raises(contract.ContractError, match="not selected"):
        contract._select_entries(manifest, ["test_unknown.py"])
    with pytest.raises(contract.ContractError, match="unique"):
        contract._select_entries(manifest, ["test_alpha.py", "test_alpha.py"])


def test_junit_counts_rejects_empty_evidence(tmp_path):
    evidence = tmp_path / "results.xml"
    evidence.write_text(
        '<testsuites><testsuite tests="3" failures="0" errors="0" skipped="1"/>'
        '</testsuites>',
        encoding="utf-8",
    )
    assert contract._junit_counts(evidence) == {
        "tests": 3,
        "failures": 0,
        "errors": 0,
        "skipped": 1,
    }

    evidence.write_text(
        '<testsuites><testsuite tests="0" failures="0" errors="0" skipped="0"/>'
        '</testsuites>',
        encoding="utf-8",
    )
    with pytest.raises(contract.ContractError, match="no tests"):
        contract._junit_counts(evidence)


def test_no_skip_evidence_requirement_fails_closed():
    counts = {"tests": 21, "failures": 0, "errors": 0, "skipped": 6}

    contract._validate_junit_counts("test_subscription.py", counts)
    with pytest.raises(contract.ContractError, match="contains skips"):
        contract._validate_junit_counts(
            "test_subscription.py", counts, require_no_skips=True)
