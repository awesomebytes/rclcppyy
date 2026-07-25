"""raw=True subscriptions: product wiring proof.

Live, ROS_DOMAIN_ID 81 (this lane's domain, hardcoded in the helper for
isolation regardless of the invoking environment).

Stock rclpy delivers a ``raw=True`` subscription's callback plain Python
bytes (no deserialization) -- this is the one entity type where the
callback deliberately does *not* receive a typed C++ message, matching
stock rather than diverging from it. The suite's own
``rclcpp_kit.direct_entities.create_raw_subscription`` (a
``rclcpp::GenericSubscription``, verified byte-for-byte identical to a real
stock rclpy raw subscriber on the same wire message) is proven at the suite
layer; this file proves the product-level wiring: the public API accepts
``raw=True``, the interaction matrix with event_callbacks/content_filter/
qos_overriding/message_info behaves as designed, and a failure in one
requested entity never leaves the node unusable for the next one.
"""
import json

from _run_helper import format_output, run_helper


REPORT_PREFIX = "RAW_SUBSCRIPTION_REPORT="


def _report():
    process = run_helper("_raw_subscription_helper.py", timeout=60)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(REPORT_PREFIX):])


def test_raw_subscription_roundtrip_and_interaction_matrix():
    report = _report()

    assert report["rmw"] == "rmw_cyclonedds_cpp"

    # Basic roundtrip: bytes in, bytes out, deserializes to the same payload.
    assert report["subscription_raw_attribute"] is True
    assert report["received_are_bytes"] is True
    assert report["roundtrip_ok"] is True
    assert report["deserializes_locally_serialized_bytes_too"] is True

    # event_callbacks fully supported for raw (GenericSubscription forwards
    # options.event_callbacks to the same SubscriptionBase constructor a
    # typed subscription uses).
    assert report["raw_matched_event_fired"] is True
    assert report["raw_subscription_has_event_handler"] is True

    # content_filter_options: fails closed on Cyclone -- same guarantee as
    # the typed path, never a silently-unfiltered subscription.
    assert report["raw_content_filter_raised"] is True
    assert report["raw_content_filter_mentions_rmw"] is True
    assert report["raw_content_filter_created_without_raising"] is False
    assert report["raw_content_filter_control_ok"] is True

    # qos_overriding_options: create_generic_subscription bypasses the
    # parameter-declaration wrapper entirely, so it is rejected up front
    # instead of silently doing nothing.
    assert report["raw_qos_overriding_raised"] is True
    assert report["raw_qos_overriding_created_without_raising"] is False
    assert report["raw_qos_overriding_control_ok"] is True

    # A message_info-style 2-arg callback is not supported for raw this wave.
    assert report["raw_message_info_raised"] is True
    assert report["raw_message_info_created_without_raising"] is False
    assert report["raw_message_info_control_ok"] is True
