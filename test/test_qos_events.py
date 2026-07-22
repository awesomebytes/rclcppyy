"""Slice 1 QoS event callbacks: publisher/subscription wiring proof
(PLAN-qos-events-product.md). Live, ROS_DOMAIN_ID 77 (this lane's domain,
hardcoded in each helper for isolation regardless of the invoking
environment).

Four helpers, run as fresh subprocesses (each activates rclcppyy's
direct_cpp profile once, which is process-global):

- ``_qos_events_scenarios_helper.py --backend {direct,stock}``: per-event-kind
  live fire (deadline/liveliness/incompatible_qos/matched/message_lost) plus
  the ``incompatible_type`` fail-closed-on-Cyclone divergence and event-
  handler introspection, run identically under both backends for the stock-
  differential proof.
- ``_qos_events_mte_raise_helper.py``: a raising event callback is contained
  under a public MultiThreadedExecutor (process does not abort; the
  exception surfaces on the spin thread) -- the containment half of the
  load-bearing MTE-safety proof.
- ``_qos_events_destroy_under_dispatch_helper.py``: destroying a subscription
  while its own event callback is genuinely in flight, N=50 -- the in-flight-
  counter half of the same proof.
"""
import json

import pytest

from _run_helper import format_output, run_helper


RMW_QOS_POLICY_RELIABILITY = 1 << 4
REPORT_PREFIX = "QOS_EVENTS_PRODUCT_REPORT="


def _run_scenarios(backend):
    process = run_helper(
        "_qos_events_scenarios_helper.py", "--backend", backend, timeout=120)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(REPORT_PREFIX):])


@pytest.fixture(scope="module")
def direct_report():
    return _run_scenarios("direct")


@pytest.fixture(scope="module")
def stock_report():
    return _run_scenarios("stock")


FIRES_EVENTS = (
    "publisher_incompatible_qos", "subscription_incompatible_qos",
    "publisher_matched", "subscription_matched",
    "publisher_deadline_missed", "subscription_deadline_missed",
    "publisher_liveliness_lost", "subscription_liveliness_changed",
)


def _assert_fires_report_shape(report):
    events = report["events"]
    for name in FIRES_EVENTS:
        case = events[name]
        assert case["fired"] is True, name
        assert case["count"] >= 1, name
        assert isinstance(case["last"], dict), name
    message_lost = events["subscription_message_lost"]
    assert message_lost["registered"] is True
    assert isinstance(message_lost["fired"], bool)


def test_qos_events_fire_live_under_direct_backend(direct_report):
    assert direct_report["backend"] == "direct"
    _assert_fires_report_shape(direct_report)
    events = direct_report["events"]
    assert (
        events["publisher_incompatible_qos"]["last"]["last_policy_kind"] ==
        RMW_QOS_POLICY_RELIABILITY)
    assert (
        events["subscription_incompatible_qos"]["last"]["last_policy_kind"] ==
        RMW_QOS_POLICY_RELIABILITY)
    assert events["publisher_matched"]["last"]["current_count"] == 1
    assert events["subscription_matched"]["last"]["current_count"] == 1


def test_qos_events_fire_live_under_stock_backend(stock_report):
    """Stock differential control: the identical scenario, unaccelerated."""
    assert stock_report["backend"] == "stock"
    _assert_fires_report_shape(stock_report)
    events = stock_report["events"]
    assert (
        events["publisher_incompatible_qos"]["last"]["last_policy_kind"] ==
        RMW_QOS_POLICY_RELIABILITY)
    assert events["publisher_matched"]["last"]["current_count"] == 1


@pytest.mark.parametrize("name", FIRES_EVENTS)
def test_direct_matches_stock_observable(direct_report, stock_report, name):
    """Same observable across backends: both fire, with the same key fields.

    Payload *type* is a disclosed differential (C++ POD vs stock's rclpy
    info dataclass, PLAN-qos-events-product.md S3) -- field access, not
    type identity, is what this asserts, matching the plan's parity claim.
    """
    direct_case = direct_report["events"][name]
    stock_case = stock_report["events"][name]
    assert direct_case["fired"] == stock_case["fired"] is True
    assert set(direct_case["last"]) == set(stock_case["last"])


def test_event_handlers_introspection_parity(direct_report, stock_report):
    """Publisher/Subscription.event_handlers[i].callback is the original
    callable under both backends -- stock's own EventHandler.callback holds
    exactly what the caller passed (rclpy.event_handler), so this is a real
    parity check, not a product-only convenience."""
    for report in (direct_report, stock_report):
        introspection = report["events"]["event_handlers_introspection"]
        assert introspection["publisher_count"] == 1, report["backend"]
        assert introspection["publisher_callback_is_original"] is True, report["backend"]
        assert introspection["subscription_count"] == 1, report["backend"]
        assert introspection["subscription_callback_is_original"] is True, report["backend"]


def test_incompatible_type_fails_closed_stricter_than_stock(
        direct_report, stock_report):
    """The one deliberate divergence (plan S3/S6#3): Cyclone silently
    registers a dead handler under stock (rcl returns OK, it never fires);
    the product fails closed with UnsupportedEventTypeError instead of ever
    returning an entity with a requested event silently unhonored."""
    direct_case = direct_report["events"]["incompatible_type"]
    assert direct_case["subscription_raised_unsupported"] is True
    assert direct_case["subscription_created_without_raising"] is False
    assert direct_case["publisher_raised_unsupported"] is True
    assert direct_case["publisher_created_without_raising"] is False
    assert direct_case["control_subscription_ok"] is True

    stock_case = stock_report["events"]["incompatible_type"]
    assert stock_case["subscription_raised_unsupported"] is False
    assert stock_case["subscription_created_without_raising"] is True
    assert stock_case["publisher_raised_unsupported"] is False
    assert stock_case["publisher_created_without_raising"] is True
    assert stock_case["control_subscription_ok"] is True


def test_wrong_event_callbacks_type_raises_type_error(direct_report):
    case = direct_report["events"]["wrong_event_callbacks_type"]
    assert case["raised_type_error"] is True
    assert "PublisherEventCallbacks" in case["message"]


def test_qos_event_callback_raise_is_contained_under_mte():
    """Load-bearing MTE-safety proof, part 1 (plan S3/S6#1): a raising
    event callback under a public MultiThreadedExecutor is contained --
    the process does not std::terminate, the exact exception surfaces on
    the spin thread, and a peer (non-raising) event callback dispatched
    normally beforehand on the same executor."""
    process = run_helper("_qos_events_mte_raise_helper.py", timeout=60)
    assert "QOS_EVENT_MTE_RAISE_PEER_DISPATCHED" in process.stdout, format_output(process)
    assert "QOS_EVENT_MTE_RAISE_READY" in process.stdout, format_output(process)
    assert (
        "QOS_EVENT_MTE_RAISE_CONTAINED_AND_RERAISED" in process.stdout
    ), format_output(process)
    assert (
        "QOS_EVENT_MTE_RAISE_DID_NOT_ABORT" in process.stdout
    ), format_output(process)
    assert process.returncode == 0, format_output(process)


def test_qos_event_destroy_during_dispatch_under_mte():
    """Load-bearing MTE-safety proof, part 2 (plan S3/S6#1): destroying a
    subscription while its own event callback is genuinely in flight on a
    native MultiThreadedExecutor worker, repeated N=50 -- no crash, no
    executor exception, proving the in-flight/quiescence counter now sees
    event-callback dispatch (it did not, before this slice's wiring)."""
    process = run_helper(
        "_qos_events_destroy_under_dispatch_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "QOS_EVENT_DESTROY_DISPATCH_ALL_OK" in process.stdout, format_output(process)
