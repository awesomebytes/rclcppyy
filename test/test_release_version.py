import importlib.util
from pathlib import Path

import pytest


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
