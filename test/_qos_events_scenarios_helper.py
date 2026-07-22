#!/usr/bin/env python3
"""Live per-event-kind fire proof for Slice 1 QoS event callbacks
(PLAN-qos-events-product.md S3/S4/S5), through the public rclpy-shaped
``Node.create_publisher``/``Node.create_subscription`` API.

Runs identically under stock rclpy (``--backend stock``) and the
direct_cpp-activated product (``--backend direct``) so the two JSON reports
are directly comparable field-for-field -- the stock-differential proof the
test plan requires. The QoS recipes (durations, policies) mirror the suite's
own proven-live ``_native_qos_events_helper.py`` (cppyy_kit), lowered to
``rclpy.qos.QoSProfile`` for the public surface this slice actually wires.

Each scenario's nodes are retired (removed from the executor and destroyed)
before the next one starts: ``deadline`` and ``liveliness`` events repeat
indefinitely once triggered (proven by the suite's own teardown-UAF-guard
proof), so a lingering prior scenario's always-ready native event handles
would otherwise starve a fresh scenario's handles in the same executor's
wait-set -- observed directly while developing this script.

``incompatible_type`` on Cyclone is the one deliberately-divergent axis: stock
silently registers a dead handler (rcl returns OK, it just never fires); the
product fails closed with ``UnsupportedEventTypeError``. Both outcomes are
recorded so the calling test can assert each backend's own expected shape.
"""
import argparse
import json
import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "77")

PREFIX = "QOS_EVENTS_PRODUCT_REPORT="


class Counter:
    """Records fire count and the last-observed scalar fields of one event."""

    def __init__(self, extractor):
        self._extractor = extractor
        self.count = 0
        self.last = None

    def __call__(self, info):
        self.count += 1
        self.last = self._extractor(info)


def _incompatible_qos(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
        "last_policy_kind": int(info.last_policy_kind),
    }


def _matched(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
        "current_count": int(info.current_count),
        "current_count_change": int(info.current_count_change),
    }


def _deadline(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
    }


def _liveliness_lost(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
    }


def _liveliness_changed(info):
    return {
        "alive_count": int(info.alive_count),
        "not_alive_count": int(info.not_alive_count),
        "alive_count_change": int(info.alive_count_change),
        "not_alive_count_change": int(info.not_alive_count_change),
    }


def _message_lost(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    args = parser.parse_args()

    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.duration import Duration
    from rclpy.event_handler import (
        PublisherEventCallbacks,
        SubscriptionEventCallbacks,
        UnsupportedEventTypeError,
    )
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        LivelinessPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import UInt64

    suffix = "%s_%d" % (args.backend, os.getpid())

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    live_nodes = []

    def make_node(name):
        node = Node("%s_%s" % (name, suffix))
        live_nodes.append(node)
        executor.add_node(node)
        return node

    def retire(*nodes):
        # Deadline/liveliness events repeat indefinitely once triggered; a
        # lingering scenario's always-ready native event handles would
        # otherwise starve the next scenario's handles in the same
        # executor's wait-set (see module docstring).
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
            live_nodes.remove(node)

    def wait_for(predicate, description, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(
            "timed out waiting for %s (backend=%s)" % (description, args.backend))

    events = {}

    # --- incompatible_qos: publisher offers BEST_EFFORT, subscriber requests
    # RELIABLE on the same topic -> both sides fire immediately on discovery.
    topic = "/qos_events_product/%s/incompatible_qos" % suffix
    pub_node = make_node("iq_pub")
    sub_node = make_node("iq_sub")
    pub_qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    sub_qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)
    pub_iq = Counter(_incompatible_qos)
    sub_iq = Counter(_incompatible_qos)
    pub_node.create_publisher(
        UInt64, topic, pub_qos,
        event_callbacks=PublisherEventCallbacks(
            incompatible_qos=pub_iq, use_default_callbacks=False))
    sub_node.create_subscription(
        UInt64, topic, lambda _msg: None, sub_qos,
        event_callbacks=SubscriptionEventCallbacks(
            incompatible_qos=sub_iq, use_default_callbacks=False))
    wait_for(lambda: pub_iq.count >= 1, "publisher incompatible_qos")
    wait_for(lambda: sub_iq.count >= 1, "subscription incompatible_qos")
    events["publisher_incompatible_qos"] = {
        "fired": True, "count": pub_iq.count, "last": pub_iq.last}
    events["subscription_incompatible_qos"] = {
        "fired": True, "count": sub_iq.count, "last": sub_iq.last}
    retire(pub_node, sub_node)

    # --- matched: a compatible peer connecting is observed 0->1 on both sides.
    topic = "/qos_events_product/%s/matched" % suffix
    pub_node = make_node("matched_pub")
    sub_node = make_node("matched_sub")
    qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)
    pub_matched = Counter(_matched)
    sub_matched = Counter(_matched)
    matched_publisher = pub_node.create_publisher(
        UInt64, topic, qos,
        event_callbacks=PublisherEventCallbacks(
            matched=pub_matched, use_default_callbacks=False))
    matched_subscription = sub_node.create_subscription(
        UInt64, topic, lambda _msg: None, qos,
        event_callbacks=SubscriptionEventCallbacks(
            matched=sub_matched, use_default_callbacks=False))
    wait_for(
        lambda: pub_matched.count >= 1 and pub_matched.last["current_count"] == 1,
        "publisher matched")
    wait_for(
        lambda: sub_matched.count >= 1 and sub_matched.last["current_count"] == 1,
        "subscription matched")
    events["publisher_matched"] = {
        "fired": True, "count": pub_matched.count, "last": pub_matched.last}
    events["subscription_matched"] = {
        "fired": True, "count": sub_matched.count, "last": sub_matched.last}
    # Introspection parity check (plan S3 "confirm what unchanged apps read"):
    # stock's Publisher/Subscription.event_handlers is a list of EventHandler
    # objects, each exposing the original callback via `.callback` -- with
    # use_default_callbacks=False and only `matched` set, stock registers
    # exactly one handler here too, so this assertion holds identically under
    # both backends.
    events["event_handlers_introspection"] = {
        "publisher_count": len(matched_publisher.event_handlers),
        "publisher_callback_is_original": (
            len(matched_publisher.event_handlers) == 1
            and matched_publisher.event_handlers[0].callback is pub_matched),
        "subscription_count": len(matched_subscription.event_handlers),
        "subscription_callback_is_original": (
            len(matched_subscription.event_handlers) == 1
            and matched_subscription.event_handlers[0].callback is sub_matched),
    }
    retire(pub_node, sub_node)

    # --- deadline_missed: a short deadline, publish once, then stop. Fires
    # repeatedly thereafter -- retired immediately after the assertion so it
    # cannot starve later scenarios' native event handles.
    topic = "/qos_events_product/%s/deadline" % suffix
    pub_node = make_node("deadline_pub")
    sub_node = make_node("deadline_sub")
    qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(nanoseconds=150_000_000))
    pub_deadline = Counter(_deadline)
    sub_deadline = Counter(_deadline)
    publisher = pub_node.create_publisher(
        UInt64, topic, qos,
        event_callbacks=PublisherEventCallbacks(
            deadline=pub_deadline, use_default_callbacks=False))
    sub_node.create_subscription(
        UInt64, topic, lambda _msg: None, qos,
        event_callbacks=SubscriptionEventCallbacks(
            deadline=sub_deadline, use_default_callbacks=False))
    wait_for(lambda: publisher.get_subscription_count() == 1, "deadline discovery")
    publisher.publish(UInt64(data=1))
    wait_for(lambda: pub_deadline.count >= 1, "publisher deadline missed", timeout=5.0)
    wait_for(lambda: sub_deadline.count >= 1, "subscription deadline missed", timeout=5.0)
    events["publisher_deadline_missed"] = {
        "fired": True, "count": pub_deadline.count, "last": pub_deadline.last}
    events["subscription_deadline_missed"] = {
        "fired": True, "count": sub_deadline.count, "last": sub_deadline.last}
    retire(pub_node, sub_node)

    # --- liveliness: MANUAL_BY_TOPIC with a short lease; publish once (which
    # asserts liveliness under MANUAL_BY_TOPIC), then stop. Also repeats
    # indefinitely -- retired right after the assertion, same reason as above.
    topic = "/qos_events_product/%s/liveliness" % suffix
    pub_node = make_node("liveliness_pub")
    sub_node = make_node("liveliness_sub")
    qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(nanoseconds=150_000_000))
    pub_liveliness = Counter(_liveliness_lost)
    sub_liveliness = Counter(_liveliness_changed)
    publisher = pub_node.create_publisher(
        UInt64, topic, qos,
        event_callbacks=PublisherEventCallbacks(
            liveliness=pub_liveliness, use_default_callbacks=False))
    sub_node.create_subscription(
        UInt64, topic, lambda _msg: None, qos,
        event_callbacks=SubscriptionEventCallbacks(
            liveliness=sub_liveliness, use_default_callbacks=False))
    wait_for(lambda: publisher.get_subscription_count() == 1, "liveliness discovery")
    publisher.publish(UInt64(data=1))
    wait_for(
        lambda: pub_liveliness.count >= 1, "publisher liveliness lost", timeout=5.0)
    wait_for(
        lambda: sub_liveliness.count >= 1, "subscription liveliness changed",
        timeout=5.0)
    events["publisher_liveliness_lost"] = {
        "fired": True, "count": pub_liveliness.count, "last": pub_liveliness.last}
    events["subscription_liveliness_changed"] = {
        "fired": True, "count": sub_liveliness.count, "last": sub_liveliness.last}
    retire(pub_node, sub_node)

    # --- message_lost: registration-level proof only, matching the suite's own
    # bar (cppyy_kit's _native_qos_events_helper.py) -- not reliably
    # triggerable on a routine loopback run, so "registered" is the assertion,
    # not "fired".
    topic = "/qos_events_product/%s/message_lost" % suffix
    pub_node = make_node("message_lost_pub")
    sub_node = make_node("message_lost_sub")
    ml_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    sub_message_lost = Counter(_message_lost)
    publisher = pub_node.create_publisher(UInt64, topic, ml_qos)
    sub_node.create_subscription(
        UInt64, topic, lambda _msg: None, ml_qos,
        event_callbacks=SubscriptionEventCallbacks(
            message_lost=sub_message_lost, use_default_callbacks=False))
    wait_for(lambda: publisher.get_subscription_count() == 1, "message_lost discovery")
    for value in range(2000):
        publisher.publish(UInt64(data=value))
    settle_deadline = time.monotonic() + 1.0
    while time.monotonic() < settle_deadline:
        executor.spin_once(timeout_sec=0.05)
    events["subscription_message_lost"] = {
        "registered": True,
        "fired": sub_message_lost.count > 0,
        "count": sub_message_lost.count,
        "last": sub_message_lost.last,
    }
    retire(pub_node, sub_node)

    # --- incompatible_type: the deliberate stricter-than-stock divergence.
    # Stock registers a dead handler on Cyclone (rcl returns OK, it never
    # fires); the product fails closed with UnsupportedEventTypeError.
    it_node = make_node("incompatible_type")
    it_qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)

    sub_raised = None
    sub_created = False
    try:
        it_node.create_subscription(
            UInt64, "/qos_events_product/%s/incompatible_type_sub" % suffix,
            lambda _msg: None, it_qos,
            event_callbacks=SubscriptionEventCallbacks(
                incompatible_type=lambda _info: None, use_default_callbacks=False))
        sub_created = True
    except UnsupportedEventTypeError as exc:
        sub_raised = str(exc)

    pub_raised = None
    pub_created = False
    try:
        it_node.create_publisher(
            UInt64, "/qos_events_product/%s/incompatible_type_pub" % suffix,
            it_qos,
            event_callbacks=PublisherEventCallbacks(
                incompatible_type=lambda _info: None, use_default_callbacks=False))
        pub_created = True
    except UnsupportedEventTypeError as exc:
        pub_raised = str(exc)

    # Control: the identical request minus incompatible_type must succeed,
    # proving any failure above is specific to that one event.
    control_topic = "/qos_events_product/%s/incompatible_type_control" % suffix
    control_subscription = it_node.create_subscription(
        UInt64, control_topic, lambda _msg: None, it_qos,
        event_callbacks=SubscriptionEventCallbacks(
            matched=lambda _info: None, use_default_callbacks=False))
    control_ok = control_subscription is not None
    it_node.destroy_subscription(control_subscription)

    events["incompatible_type"] = {
        "subscription_raised_unsupported": sub_raised is not None,
        "subscription_created_without_raising": sub_created,
        "publisher_raised_unsupported": pub_raised is not None,
        "publisher_created_without_raising": pub_created,
        "control_subscription_ok": bool(control_ok),
    }

    if args.backend == "direct":
        # Product-only input-shape validation: a non-PublisherEventCallbacks/
        # SubscriptionEventCallbacks value raises TypeError before any native
        # entity is constructed (mirrors the existing entity-options reject
        # tests' object()-argument cases in _direct_cpp_helper.py, with a
        # specific message asserted here for a stronger proof).
        wrong_type_topic = "/qos_events_product/%s/wrong_type" % suffix
        wrong_type_raised = None
        try:
            it_node.create_publisher(
                UInt64, wrong_type_topic, it_qos, event_callbacks=object())
        except TypeError as exc:
            wrong_type_raised = str(exc)
        events["wrong_event_callbacks_type"] = {
            "raised_type_error": wrong_type_raised is not None,
            "message": wrong_type_raised,
        }

    retire(it_node)

    rclpy.shutdown()

    report = {
        "schema": "rclcppyy.qos-events-product-proof/v1",
        "backend": args.backend,
        "events": events,
    }
    print(PREFIX + json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
