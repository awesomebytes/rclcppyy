"""The compatible product path must not initialize the native/Cling stack."""

from pathlib import Path
import subprocess
import sys

import pytest


HELPER = Path(__file__).with_name("_lazy_import_helper.py")


@pytest.mark.parametrize(
    ("mode", "marker"),
    [
        ("compatible", "COMPATIBLE_IMPORT_GRAPH_LIGHT_OK"),
        ("bringup", "BRINGUP_EXPORT_LAZY_OK"),
        ("legacy_node", "LEGACY_NODE_EXPORT_LAZY_OK"),
        ("native_exports", "NATIVE_EXPORTS_LAZY_OK"),
    ],
)
def test_public_exports_load_heavy_dependencies_only_on_demand(mode, marker):
    process = subprocess.run(
        [sys.executable, str(HELPER), mode],
        capture_output=True,
        text=True,
        timeout=120,
    )
    details = (
        "returncode=%d\nstdout:\n%s\nstderr:\n%s"
        % (process.returncode, process.stdout, process.stderr)
    )
    assert process.returncode == 0, details
    assert marker in process.stdout, details


@pytest.mark.parametrize(
    "profile", ("compatible", "publisher_cpp", "required_cpp", "optimized"))
def test_every_profile_preserves_exact_stock_future_methods(profile):
    process = subprocess.run(
        [sys.executable, str(HELPER), "future_identity", profile],
        capture_output=True,
        text=True,
        timeout=120,
    )
    details = (
        "returncode=%d\nstdout:\n%s\nstderr:\n%s"
        % (process.returncode, process.stdout, process.stderr)
    )
    assert process.returncode == 0, details
    assert "FUTURE_METHOD_IDENTITY_OK" in process.stdout, details
