import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "ci" / "verify_release_version.py"
SPEC = importlib.util.spec_from_file_location("verify_release_version", SCRIPT)
verify_release_version = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify_release_version)


def test_release_metadata_and_tag_are_exactly_aligned():
    versions = verify_release_version.metadata_versions(ROOT)

    assert versions == {
        "pixi.toml": "0.3.0",
        "package.xml": "0.3.0",
        "recipe/recipe.yaml": "0.3.0",
    }
    assert verify_release_version.verify(ROOT, "v0.3.0") == "0.3.0"


@pytest.mark.parametrize("tag", ["v0.2.0", "v0.3", "0.3.0", "v0.3.0-rc1", "v9.9.9"])
def test_release_rejects_arbitrary_or_stale_tag(tag):
    with pytest.raises(ValueError, match="release tag mismatch"):
        verify_release_version.verify(ROOT, tag)


def test_release_requires_dual_arch_source_preflight_and_exact_suite_build():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "release.yml").read_text())
    jobs = workflow["jobs"]
    preflight = jobs["preflight"]
    release = jobs["release"]
    assert workflow["permissions"] == {"contents": "read"}
    assert "permissions" not in preflight
    assert release["permissions"] == {
        "contents": "read",
        "id-token": "write",
        "attestations": "write",
    }
    matrix = preflight["strategy"]["matrix"]["include"]
    assert {(item["platform"], item["machine"]) for item in matrix} == {
        ("linux-64", "x86_64"),
        ("linux-aarch64", "aarch64"),
    }
    assert release["needs"] == "preflight"
    assert {
        (item["platform"], item["machine"])
        for item in release["strategy"]["matrix"]["include"]
    } == {
        ("linux-64", "x86_64"),
        ("linux-aarch64", "aarch64"),
    }

    suite_commit = json.loads(
        (ROOT / "suite-source.lock.json").read_text())["commit"]
    suite_refs = []
    for job in (preflight, release):
        for step in job["steps"]:
            settings = step.get("with", {})
            if settings.get("repository") == "awesomebytes/cppyy_kit":
                suite_refs.append(settings.get("ref"))
    assert suite_refs == [suite_commit, suite_commit]

    release_commands = "\n".join(
        step.get("run", "") for step in release["steps"])
    assert "build_local_package_stack.sh" in release_commands
    assert "_deps/cppyy_kit output" in release_commands
    assert "local-package-attestation.json" in release_commands


def test_ci_requires_installed_package_proof_on_both_architectures():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "ci.yml").read_text())
    native = workflow["jobs"]["native-architecture"]
    matrix = native["strategy"]["matrix"]["include"]

    assert {
        (item["platform"], item["machine"], item["package_proof"])
        for item in matrix
    } == {
        ("linux-64", "x86_64", True),
        ("linux-aarch64", "aarch64", True),
    }
    commands = "\n".join(step.get("run", "") for step in native["steps"])
    assert "build_local_package_stack.sh" in commands
    assert "prove_rclcppyy_package.sh" in commands
    assert "local-package-attestation.json" in commands
