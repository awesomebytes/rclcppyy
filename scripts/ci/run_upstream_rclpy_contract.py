#!/usr/bin/env python3
"""Validate and run the pinned, reviewed upstream rclpy contract slice."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET


_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_MANIFEST = _REPO_ROOT / "compatibility" / "upstream-rclpy-contract.json"
_HASH_ALGORITHM = "sha256-path-content-v1"


class ContractError(RuntimeError):
    """A fail-closed contract validation error."""


def _require(condition, message):
    if not condition:
        raise ContractError(message)


def _git(source, *args):
    command = ["git", "-C", str(source), *args]
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        raise ContractError("git command failed (%s): %s" % (" ".join(args), detail))
    return result.stdout.strip()


def _normalize_repository(value):
    return value.rstrip("/").removesuffix(".git")


def _inventory(test_root, pattern):
    files = sorted(path for path in test_root.rglob(pattern) if path.is_file())
    digest = hashlib.sha256()
    relative_paths = []
    for path in files:
        relative = path.relative_to(test_root).as_posix()
        content_digest = hashlib.sha256(path.read_bytes()).hexdigest()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(content_digest.encode("ascii"))
        digest.update(b"\n")
        relative_paths.append(relative)
    return relative_paths, digest.hexdigest()


def load_manifest(path):
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError("cannot read contract manifest %s: %s" % (path, exc))
    _require(isinstance(manifest, dict), "contract manifest must be a JSON object")
    _require(manifest.get("schema_version") == 1, "unsupported contract schema_version")
    _require(isinstance(manifest.get("contract_id"), str), "contract_id must be a string")

    source = manifest.get("source")
    _require(isinstance(source, dict), "source must be an object")
    for key in ("repository", "revision", "release", "package_version", "test_root"):
        _require(isinstance(source.get(key), str) and source[key], "source.%s is required" % key)
    revision = source["revision"]
    _require(
        len(revision) == 40 and all(character in "0123456789abcdef" for character in revision),
        "source.revision must be a full lowercase Git commit",
    )
    _require(
        not Path(source["test_root"]).is_absolute() and ".." not in Path(source["test_root"]).parts,
        "source.test_root must be a safe relative path",
    )

    inventory = manifest.get("inventory")
    _require(isinstance(inventory, dict), "inventory must be an object")
    _require(inventory.get("algorithm") == _HASH_ALGORITHM, "unsupported inventory algorithm")
    _require(
        isinstance(inventory.get("file_pattern"), str) and inventory["file_pattern"],
        "inventory.file_pattern is required",
    )
    _require(
        isinstance(inventory.get("file_count"), int) and inventory["file_count"] > 0,
        "inventory.file_count must be positive",
    )
    expected_digest = inventory.get("digest", "")
    _require(
        len(expected_digest) == 64
        and all(character in "0123456789abcdef" for character in expected_digest),
        "inventory.digest must be a lowercase SHA-256 digest",
    )

    support_files = manifest.get("support_files", [])
    _require(isinstance(support_files, list), "support_files must be a list")
    support_paths = []
    for entry in support_files:
        _require(isinstance(entry, dict), "support file entries must be objects")
        path = entry.get("path")
        _require(isinstance(path, str) and path, "support file path is required")
        relative = Path(path)
        _require(
            not relative.is_absolute() and ".." not in relative.parts,
            "support file path must be a safe relative path: %s" % path,
        )
        digest = entry.get("digest", "")
        _require(
            len(digest) == 64
            and all(character in "0123456789abcdef" for character in digest),
            "support file %s needs a lowercase SHA-256 digest" % path,
        )
        support_paths.append(relative.as_posix())
    _require(
        len(support_paths) == len(set(support_paths)),
        "support_files contains duplicate paths",
    )

    selection = manifest.get("selection")
    _require(isinstance(selection, list) and selection, "selection must not be empty")
    selected_paths = []
    for entry in selection:
        _require(isinstance(entry, dict), "selection entries must be objects")
        path = entry.get("path")
        _require(isinstance(path, str) and path, "selection path is required")
        _require(Path(path).name == path, "selected tests must be top-level test files: %s" % path)
        _require(
            isinstance(entry.get("reason"), str) and len(entry["reason"].strip()) >= 20,
            "selection %s needs a substantive reason" % path,
        )
        if "requires_cpp_publish" in entry:
            _require(
                isinstance(entry["requires_cpp_publish"], bool),
                "requires_cpp_publish must be boolean for %s" % path,
            )
        selected_paths.append(path)
    _require(
        len(selected_paths) == len(set(selected_paths)),
        "selection contains duplicate paths",
    )

    exclusions = manifest.get("reviewed_exclusions")
    _require(
        isinstance(exclusions, list) and exclusions,
        "reviewed_exclusions must not be empty",
    )
    excluded_paths = []
    exclusion_ids = []
    for group in exclusions:
        _require(isinstance(group, dict), "reviewed exclusion entries must be objects")
        exclusion_id = group.get("id")
        _require(isinstance(exclusion_id, str) and exclusion_id, "exclusion id is required")
        _require(
            isinstance(group.get("reason"), str) and len(group["reason"].strip()) >= 20,
            "exclusion %s needs a substantive reason" % exclusion_id,
        )
        paths = group.get("paths")
        _require(isinstance(paths, list) and paths, "exclusion %s has no paths" % exclusion_id)
        for path in paths:
            _require(
                isinstance(path, str) and Path(path).name == path,
                "excluded tests must be top-level test files: %r" % path,
            )
        exclusion_ids.append(exclusion_id)
        excluded_paths.extend(paths)
    _require(
        len(exclusion_ids) == len(set(exclusion_ids)),
        "reviewed exclusions contain duplicate ids",
    )
    _require(
        len(excluded_paths) == len(set(excluded_paths)),
        "reviewed exclusions contain duplicate paths",
    )
    overlap = sorted(set(selected_paths) & set(excluded_paths))
    _require(not overlap, "tests cannot be selected and excluded: %s" % ", ".join(overlap))
    return manifest


def validate_source(manifest, source):
    source = source.resolve()
    _require(source.is_dir(), "upstream source directory does not exist: %s" % source)

    top_level = Path(_git(source, "rev-parse", "--show-toplevel")).resolve()
    _require(top_level == source, "source must point at the upstream Git checkout root")
    head = _git(source, "rev-parse", "HEAD")
    expected_revision = manifest["source"]["revision"]
    _require(
        head == expected_revision,
        "upstream revision drift: expected %s, got %s" % (expected_revision, head),
    )
    origin = _git(source, "remote", "get-url", "origin")
    expected_repository = manifest["source"]["repository"]
    _require(
        _normalize_repository(origin) == _normalize_repository(expected_repository),
        "upstream repository drift: expected %s, got %s" % (expected_repository, origin),
    )
    dirty = _git(source, "status", "--porcelain", "--untracked-files=all")
    _require(not dirty, "upstream checkout is dirty; refusing mixed contract evidence")

    package_xml = source / "rclpy" / "package.xml"
    _require(package_xml.is_file(), "upstream rclpy/package.xml is missing")
    try:
        package_version = ET.parse(package_xml).getroot().findtext("version")
    except ET.ParseError as exc:
        raise ContractError("cannot parse upstream rclpy/package.xml: %s" % exc)
    expected_version = manifest["source"]["package_version"]
    _require(
        package_version == expected_version,
        "upstream package version drift: expected %s, got %s"
        % (expected_version, package_version),
    )

    test_root = source / manifest["source"]["test_root"]
    _require(test_root.is_dir(), "upstream test root is missing: %s" % test_root)
    actual_paths, actual_digest = _inventory(
        test_root,
        manifest["inventory"]["file_pattern"],
    )
    inventory = manifest["inventory"]
    _require(
        len(actual_paths) == inventory["file_count"],
        "upstream test inventory count drift: expected %d, got %d"
        % (inventory["file_count"], len(actual_paths)),
    )
    _require(
        actual_digest == inventory["digest"],
        "upstream test inventory content drift: expected %s, got %s"
        % (inventory["digest"], actual_digest),
    )
    for entry in manifest.get("support_files", []):
        path = test_root / entry["path"]
        _require(path.is_file(), "upstream support file is missing: %s" % path)
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        _require(
            actual == entry["digest"],
            "upstream support file content drift for %s: expected %s, got %s"
            % (entry["path"], entry["digest"], actual),
        )

    selected = {entry["path"] for entry in manifest["selection"]}
    excluded = {
        path
        for group in manifest["reviewed_exclusions"]
        for path in group["paths"]
    }
    actual = set(actual_paths)
    _require(
        selected | excluded == actual,
        "selection/exclusion review does not exactly partition the upstream inventory",
    )
    return {
        "repository": expected_repository,
        "revision": head,
        "release": manifest["source"]["release"],
        "package_version": package_version,
        "inventory_digest": actual_digest,
        "inventory_file_count": len(actual_paths),
        "support_file_count": len(manifest.get("support_files", [])),
        "selected_files": len(selected),
        "excluded_files": len(excluded),
        "test_root": str(test_root),
    }


def validate_runtime_version(manifest):
    expected = manifest["source"]["package_version"]
    try:
        actual = importlib.metadata.version("rclpy")
    except importlib.metadata.PackageNotFoundError:
        raise ContractError("the selected environment has no installed rclpy distribution")
    _require(
        actual == expected,
        "installed rclpy version drift: expected %s, got %s" % (expected, actual),
    )
    return actual


def _select_entries(manifest, only_paths):
    selection = manifest["selection"]
    if not only_paths:
        return selection
    requested = set(only_paths)
    _require(
        len(requested) == len(only_paths),
        "--only-path entries must be unique",
    )
    available = {entry["path"] for entry in selection}
    unknown = sorted(requested - available)
    _require(
        not unknown,
        "--only-path is not selected by the reviewed manifest: %s"
        % ", ".join(unknown),
    )
    return [entry for entry in selection if entry["path"] in requested]


def _activate_bootstrap():
    os.environ["RCLCPPYY_ENABLE_HOOK"] = "1"
    boot_path = _REPO_ROOT / "rclcppyy" / "_hook_boot.py"
    spec = importlib.util.spec_from_file_location("_rclcppyy_contract_boot", boot_path)
    if spec is None or spec.loader is None:
        raise ContractError("cannot load the rclcppyy startup bootstrap")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.activate()


def _prove_backend(entry):
    monkey = sys.modules.get("rclcppyy.monkey")
    if monkey is None or not getattr(monkey, "_PATCHED", False):
        raise ContractError("rclcppyy acceleration was not active during the upstream test")
    if not entry.get("requires_cpp_publish", False):
        return
    import rclcppyy

    operations = rclcppyy.status()["operations"]
    cpp_publish = [
        record for record in operations
        if record["backend"] == "cpp"
        and record["metadata"].get("operation") == "publish"
    ]
    python_publish = [
        record for record in operations
        if record["backend"] == "python"
        and record["metadata"].get("operation") == "publish"
    ]
    _require(cpp_publish, "selected publisher contract did not complete a C++ publish")
    _require(
        not python_publish,
        "selected publisher contract used a Python publish fallback",
    )


def _run_staged_file(args):
    manifest = load_manifest(args.manifest)
    entries = {
        entry["path"]: entry
        for entry in manifest["selection"]
    }
    entry = entries.get(args.selection_path)
    _require(entry is not None, "internal test path is not selected by the manifest")
    validate_runtime_version(manifest)
    _activate_bootstrap()

    import pytest

    pytest_args = [
        "-q",
        "-ra",
        "--rootdir=%s" % _REPO_ROOT,
        "--junitxml=%s" % args.junit_xml,
        str(args.internal_run_file),
    ]
    result = int(pytest.main(pytest_args))
    if result == 0:
        _prove_backend(entry)
    return result


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp.%d" % os.getpid())
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _junit_counts(path):
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ContractError("cannot read JUnit evidence %s: %s" % (path, exc))
    suites = [root] if root.tag == "testsuite" else root.findall("./testsuite")
    _require(suites, "JUnit evidence has no test suites: %s" % path)
    counts = {
        name: sum(int(suite.attrib.get(name, "0")) for suite in suites)
        for name in ("tests", "failures", "errors", "skipped")
    }
    _require(counts["tests"] > 0, "JUnit evidence has no tests: %s" % path)
    return counts


def _validate_junit_counts(relative, counts, *, require_no_skips=False):
    _require(
        counts["tests"] > counts["skipped"],
        "all selected tests were skipped: %s" % relative,
    )
    _require(
        counts["failures"] == 0 and counts["errors"] == 0,
        "passing child returned failing JUnit evidence: %s" % relative,
    )
    if require_no_skips:
        _require(
            counts["skipped"] == 0,
            "selected evidence contains skips: %s" % relative,
        )


def run_contract(args, manifest, source_report):
    evidence_dir = args.evidence_dir.resolve()
    evidence_dir.mkdir(parents=True, exist_ok=True)
    test_root = Path(source_report["test_root"])
    results = []
    selection = _select_entries(manifest, args.only_path)

    for entry in selection:
        relative = entry["path"]
        started = time.monotonic()
        with tempfile.TemporaryDirectory(prefix="rclcppyy-upstream-contract-") as stage:
            staged_root = Path(stage) / "rclpy_contract_tests"
            staged_root.mkdir()
            (staged_root / "__init__.py").write_text("", encoding="utf-8")
            for support in manifest.get("support_files", []):
                source_path = test_root / support["path"]
                staged_support = staged_root / support["path"]
                staged_support.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source_path, staged_support)
            staged_file = staged_root / relative
            shutil.copyfile(test_root / relative, staged_file)
            junit_xml = evidence_dir / (Path(relative).stem + ".xml")
            junit_xml.unlink(missing_ok=True)
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--manifest",
                str(args.manifest),
                "--internal-run-file",
                str(staged_file),
                "--selection-path",
                relative,
                "--junit-xml",
                str(junit_xml),
            ]
            environment = os.environ.copy()
            environment["RCLCPPYY_ENABLE_HOOK"] = "1"
            if args.rmw_implementation is not None:
                environment["RMW_IMPLEMENTATION"] = args.rmw_implementation
            try:
                completed = subprocess.run(
                    command,
                    cwd=_REPO_ROOT,
                    env=environment,
                    check=False,
                    timeout=args.timeout_seconds,
                )
                return_code = completed.returncode
                outcome = "passed" if return_code == 0 else "failed"
            except subprocess.TimeoutExpired:
                return_code = 124
                outcome = "timed_out"
                print(
                    "upstream contract timed out after %ss: %s"
                    % (args.timeout_seconds, relative),
                    file=sys.stderr,
                )
        counts = None
        try:
            counts = _junit_counts(junit_xml)
            if return_code == 0:
                _validate_junit_counts(
                    relative,
                    counts,
                    require_no_skips=args.require_no_skips,
                )
        except ContractError as exc:
            if return_code == 0:
                return_code = 3
                outcome = "invalid_evidence"
            print("upstream contract evidence rejected: %s" % exc, file=sys.stderr)
        result = {
            "path": relative,
            "outcome": outcome,
            "return_code": return_code,
            "duration_seconds": round(time.monotonic() - started, 3),
            "junit_xml": str(junit_xml),
        }
        if counts is not None:
            result["counts"] = counts
        results.append(result)
        if return_code != 0 and args.fail_fast:
            break

    passed = sum(result["outcome"] == "passed" for result in results)
    totals = {
        name: sum(result.get("counts", {}).get(name, 0) for result in results)
        for name in ("tests", "failures", "errors", "skipped")
    }
    summary = {
        "schema_version": 1,
        "contract_id": manifest["contract_id"],
        "source": source_report,
        "installed_rclpy_version": validate_runtime_version(manifest),
        "acceleration_profile": "compatible",
        "rmw_implementation": (
            args.rmw_implementation or os.environ.get("RMW_IMPLEMENTATION")),
        "execution": {
            "isolation": "one_process_per_selected_file",
            "selected_files": len(selection),
            "selected_paths": [entry["path"] for entry in selection],
            "require_no_skips": args.require_no_skips,
            "executed_files": len(results),
            "passed_files": passed,
            "failed_files": len(results) - passed,
            "totals": totals,
            "results": results,
        },
    }
    _write_json(evidence_dir / "summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if passed == len(selection) else 1


def parse_args(argv=None):
    default_source = Path(os.environ.get(
        "RCLPY_CONTRACT_SRC",
        _REPO_ROOT / "_deps" / "rclpy-contract",
    ))
    parser = argparse.ArgumentParser(
        description="Validate and run the exact reviewed upstream rclpy contract slice."
    )
    parser.add_argument("--source", type=Path, default=default_source)
    parser.add_argument("--manifest", type=Path, default=_DEFAULT_MANIFEST)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--only-path",
        action="append",
        default=[],
        help="Run one reviewed selected test path; repeat to select more than one.",
    )
    parser.add_argument(
        "--rmw-implementation",
        choices=("rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"),
        help="Override RMW_IMPLEMENTATION in each isolated contract process.",
    )
    parser.add_argument(
        "--require-no-skips",
        action="store_true",
        help="Reject selected JUnit evidence containing any skipped tests.",
    )
    parser.add_argument(
        "--evidence-dir",
        type=Path,
        default=_REPO_ROOT / "build" / "test-results" / "upstream-rclpy",
    )
    parser.add_argument("--timeout-seconds", type=int, default=180)
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument("--internal-run-file", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--selection-path", help=argparse.SUPPRESS)
    parser.add_argument("--junit-xml", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.internal_run_file is not None:
            _require(args.selection_path is not None, "internal selection path is required")
            _require(args.junit_xml is not None, "internal JUnit path is required")
            return _run_staged_file(args)

        _require(args.timeout_seconds > 0, "--timeout-seconds must be positive")
        manifest = load_manifest(args.manifest.resolve())
        source_report = validate_source(manifest, args.source)
        runtime_version = validate_runtime_version(manifest)
        selection = _select_entries(manifest, args.only_path)
        if args.validate_only:
            report = {
                "schema_version": 1,
                "contract_id": manifest["contract_id"],
                "source": source_report,
                "installed_rclpy_version": runtime_version,
                "selected": [
                    entry["path"]
                    for entry in selection
                ],
                "reviewed_exclusions": [
                    {
                        "id": group["id"],
                        "file_count": len(group["paths"]),
                        "reason": group["reason"],
                    }
                    for group in manifest["reviewed_exclusions"]
                ],
            }
            print(json.dumps(report, indent=2, sort_keys=True))
            return 0
        return run_contract(args, manifest, source_report)
    except ContractError as exc:
        print("upstream contract rejected: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
