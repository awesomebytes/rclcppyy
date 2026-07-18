import importlib.util
import json
import os
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "ci" / "verify_suite_source.py"
SPEC = importlib.util.spec_from_file_location("verify_suite_source", SCRIPT)
verify_suite_source = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify_suite_source)


def test_committed_suite_lock_is_strict_and_matches_checkout():
    lock = verify_suite_source.load_lock(ROOT / "suite-source.lock.json")
    suite = Path(os.environ.get("RCLCPPYY_SUITE_SRC", ROOT.parent / "cppyy_kit"))

    assert lock["commit"] == verify_suite_source._git(suite, "rev-parse", "HEAD")
    assert verify_suite_source.recipe_versions(suite) == {lock["package_version"]}


@pytest.mark.parametrize(
    "change, message",
    [
        ({"schema": "other/v1"}, "schema"),
        ({"commit": "abc"}, "commit"),
        ({"package_version": "latest"}, "version"),
        ({"repository": ""}, "repository"),
    ],
)
def test_suite_lock_rejects_ambiguous_identity(tmp_path, change, message):
    document = {
        "schema": verify_suite_source.SCHEMA,
        "repository": "owner/repository",
        "commit": "a" * 40,
        "package_version": "1.2.3",
    }
    document.update(change)
    path = tmp_path / "suite-source.lock.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        verify_suite_source.load_lock(path)
