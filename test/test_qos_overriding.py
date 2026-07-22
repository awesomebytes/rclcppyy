"""Slice 3 QoS overriding options: product wiring proof
(PLAN-qos-events-product.md). Live cases use ROS_DOMAIN_ID 77 (this lane's
domain, hardcoded in the helper for isolation regardless of the invoking
environment).

QoS overriding is parameter-layer and RMW-independent, unlike slice 2's
content filter -- the positive path IS reachable under direct_cpp's
Jazzy/Cyclone-only gate, so unlike ``test_content_filter.py`` this file
proves the real filtering-equivalent behavior live: declared node
parameters, and an override actually changing the created entity's
effective QoS.

``create_publisher``'s ``qos_overriding_options`` is untouched by this slice
-- the suite (``rclcpp_kit.direct_entities`` @ 042bb29) has no publisher-side
attachment point for ``QosOverridingOptions`` at all (only
``create_subscription`` supports ``qos_overriding``); this is a documented
suite gap, stricter than stock (stock supports ``with_default_policies()``
on publishers too). See the plan's S3 addendum.
"""
import json

import pytest

from _run_helper import format_output, run_helper
from rclcppyy.direct_cpp import _extract_qos_overriding
from rclcppyy.policy import BackendUnavailableError
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSPolicyKind
from rclpy.qos_overriding_options import QoSOverridingOptions


REPORT_PREFIX = "QOS_OVERRIDING_PRODUCT_REPORT="


def _report():
    process = run_helper("_qos_overriding_scenarios_helper.py", timeout=60)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(REPORT_PREFIX):])


@pytest.fixture(scope="module")
def report():
    return _report()


# --- Cheap no-ROS unit tests for the pure translation/gating function -----

def test_extract_qos_overriding_none_means_no_override():
    assert _extract_qos_overriding(None) is False


def test_extract_qos_overriding_with_default_policies_is_true():
    assert _extract_qos_overriding(
        QoSOverridingOptions.with_default_policies()) is True


def test_extract_qos_overriding_equivalent_explicit_construction_is_true():
    """Order-independent: an explicit policy_kinds set equal to the default
    (in any order) is recognized identically to with_default_policies()."""
    assert _extract_qos_overriding(
        QoSOverridingOptions(
            policy_kinds=(
                QoSPolicyKind.RELIABILITY,
                QoSPolicyKind.HISTORY,
                QoSPolicyKind.DEPTH,
            ))) is True


def test_extract_qos_overriding_rejects_custom_subset():
    with pytest.raises(BackendUnavailableError, match="with_default_policies"):
        _extract_qos_overriding(
            QoSOverridingOptions(policy_kinds=(QoSPolicyKind.HISTORY,)))


def test_extract_qos_overriding_rejects_validation_callback():
    with pytest.raises(BackendUnavailableError, match="with_default_policies"):
        _extract_qos_overriding(
            QoSOverridingOptions.with_default_policies(
                callback=lambda _qos: SetParametersResult(successful=True)))


def test_extract_qos_overriding_rejects_entity_id():
    with pytest.raises(BackendUnavailableError, match="with_default_policies"):
        _extract_qos_overriding(
            QoSOverridingOptions.with_default_policies(entity_id="custom"))


def test_extract_qos_overriding_rejects_wrong_type():
    with pytest.raises(TypeError, match="QoSOverridingOptions"):
        _extract_qos_overriding(True)


# --- Live product-wiring proof, through Node.create_subscription/publisher -

def test_qos_overriding_declares_default_parameters_matching_requested_qos(report):
    assert report["default_declared"] == {
        "history": {"declared": True},
        "depth": {"declared": True},
        "reliability": {"declared": True},
    }
    assert report["default_depth_value"] == 8
    assert report["default_actual_depth"] == 8


def test_qos_overriding_override_actually_takes_effect(report):
    assert report["override_declared_value"] == 3
    assert report["override_actual_depth"] == 3


def test_qos_overriding_custom_subset_fails_closed_live(report):
    assert report["custom_subset_raised"] is True
    assert "with_default_policies" in report["custom_subset_message"]


def test_qos_overriding_validation_callback_fails_closed_live(report):
    assert report["callback_raised"] is True
    assert "with_default_policies" in report["callback_message"]


def test_qos_overriding_entity_id_fails_closed_live(report):
    assert report["entity_id_raised"] is True
    assert "with_default_policies" in report["entity_id_message"]


def test_wrong_qos_overriding_options_type_raises_type_error_live(report):
    assert report["wrong_type_raised"] is True
    assert "QoSOverridingOptions" in report["wrong_type_message"]


def test_qos_overriding_combines_with_event_callbacks(report):
    assert report["combo_events_declared"] == {
        "history": {"declared": True},
        "depth": {"declared": True},
        "reliability": {"declared": True},
    }
    assert report["combo_events_matched_fired"] is True


def test_qos_overriding_combined_with_content_filter_still_fails_closed(report):
    """Proves the qos_overriding translation didn't derail slice 2's
    fail-closed content-filter gate: the combo raises
    ContentFilterUnsupportedError specifically, not some other error, and
    never returns an entity."""
    assert report["combo_filter_raised_content_filter_unsupported"] is True
    assert report["combo_filter_created_without_raising"] is False


def test_qos_overriding_publisher_side_reject_is_unchanged(report):
    """create_publisher's qos_overriding_options reject is untouched by this
    slice (suite gap: no publisher-side attachment point)."""
    assert report["publisher_reject_raised"] is True
    assert "qos_overriding_options" in report["publisher_reject_message"]
