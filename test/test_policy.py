"""Policy semantics and required-C++ integration tests."""

import pytest

from rclcppyy.policy import resolve_policy

from _run_helper import format_output, run_helper


@pytest.mark.parametrize(
    "name,required,contract_changes",
    [
        ("compatible", False, False),
        ("required_cpp", True, False),
        ("optimized", False, True),
    ],
)
def test_profile_semantics(name, required, contract_changes):
    policy = resolve_policy(name, warn_fallback=True)
    assert policy.name == name
    assert policy.require_cpp is required
    assert policy.allow_contract_changes is contract_changes
    assert policy.warn_fallback is True


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match="unknown acceleration profile"):
        resolve_policy("silent_fallback")


def test_required_cpp_fails_before_creating_unsupported_entity():
    proc = run_helper("_required_cpp_helper.py")
    details = format_output(proc)
    assert "REQUIRED_PUBLISHER_OK" in proc.stdout, details
    assert "REQUIRED_FAIL_CLOSED_OK" in proc.stdout, details
    assert "REQUIRED_CONTROL_PLANE_OK" in proc.stdout, details
    assert proc.returncode == 0, details


def test_compatible_control_plane_status_and_warnings_are_bounded():
    proc = run_helper("_control_plane_helper.py")
    details = format_output(proc)
    assert "CONTROL_PLANE_STATUS_OK" in proc.stdout, details
    assert "CONTROL_PLANE_WARN_ONCE_OK" in proc.stdout, details
    assert "CONTROL_PLANE_SIGNATURES_OK" in proc.stdout, details
    assert "CONTROL_PLANE_LIFECYCLE_OK" in proc.stdout, details
    assert proc.returncode == 0, details
