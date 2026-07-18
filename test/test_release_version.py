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
    package = jobs["package"]
    publish = jobs["publish"]
    assert workflow["permissions"] == {"contents": "read"}
    assert "permissions" not in preflight
    assert package["permissions"] == {
        "contents": "read",
        "id-token": "write",
        "attestations": "write",
    }
    assert publish["permissions"] == {
        "actions": "read",
        "contents": "read",
        "id-token": "write",
    }
    matrix = preflight["strategy"]["matrix"]["include"]
    assert {(item["platform"], item["machine"]) for item in matrix} == {
        ("linux-64", "x86_64"),
        ("linux-aarch64", "aarch64"),
    }
    assert package["needs"] == "preflight"
    assert {
        (item["platform"], item["machine"])
        for item in package["strategy"]["matrix"]["include"]
    } == {
        ("linux-64", "x86_64"),
        ("linux-aarch64", "aarch64"),
    }
    assert publish["needs"] == "package"
    assert "strategy" not in publish

    suite_commit = json.loads(
        (ROOT / "suite-source.lock.json").read_text())["commit"]
    suite_refs = []
    for job in (preflight, package):
        for step in job["steps"]:
            settings = step.get("with", {})
            if settings.get("repository") == "awesomebytes/cppyy_kit":
                suite_refs.append(settings.get("ref"))
    assert suite_refs == [suite_commit, suite_commit]

    package_commands = "\n".join(
        step.get("run", "") for step in package["steps"])
    assert "build_release_product_stack.sh" in package_commands
    assert "output build/release/published-support.json" in package_commands
    assert "local-package-attestation.json" in package_commands
    assert "verify_published_support.py" in package_commands
    assert "build_release_inventory.py" in package_commands
    assert "--published-support-proof build/release/published-support.json" in (
        package_commands)
    assert "rattler-build upload prefix" not in package_commands

    step_names = [step["name"] for step in package["steps"]]
    assert step_names.index("Retain provenance-verified published dependencies") < (
        step_names.index("Build product against exact published dependency bytes"))
    assert step_names.index("Build product against exact published dependency bytes") < (
        step_names.index("Prove the artifact installs and runs from a channel"))

    publish_commands = "\n".join(
        step.get("run", "") for step in publish["steps"])
    assert "verify_release_matrix.py" in publish_commands
    assert publish_commands.count("verify_product_upload.py") == 2
    assert "--missing-output build/release-publication/missing.txt" in publish_commands
    assert "--require-present --attempts 12 --delay-seconds 5" in publish_commands
    assert "mapfile -t artifacts < build/release-publication/missing.txt" in (
        publish_commands)
    assert publish_commands.count("rattler-build upload prefix") == 1
    assert "ros-jazzy-rclcppyy-*.conda" in publish_commands
    assert "cppyy-kit-*.conda" not in publish_commands
    assert "rclcpp-kit-*.conda" not in publish_commands

    downloads = [step for step in publish["steps"]
                 if step.get("uses", "").startswith("actions/download-artifact@")]
    assert len(downloads) == 1
    assert downloads[0]["with"] == {
        "pattern": "release-bundle-*-${{ github.ref_name }}",
        "path": "build/release-input",
        "merge-multiple": False,
    }


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
