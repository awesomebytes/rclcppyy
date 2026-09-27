#!/usr/bin/env python3
"""Live proof for raw=True subscriptions (rclpy's raw binary-message mode)
through the public, direct_cpp-accelerated ``Node.create_subscription`` API.

ROS_DOMAIN_ID 81, this lane's domain (hardcoded for isolation regardless of
the invoking environment, matching the other product QoS-surface helpers).
"""
import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "81")

PREFIX = "RAW_SUBSCRIPTION_REPORT="


def main():
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.qos_overriding_options import QoSOverridingOptions
    from rclpy.subscription_content_filter_options import ContentFilterOptions
    from rclpy.utilities import get_rmw_implementation_identifier
    from std_msgs.msg import String

    from rclcppyy.direct_cpp import ContentFilterUnsupportedError
    # ``enable_cpp_acceleration`` redirects ``std_msgs.msg.String`` itself to
    # the cppyy C++ alias -- ``rclpy.serialization`` (de)serializes plain
    # Python message instances only, so decoding raw bytes here goes through
    # the suite's own C++-side (de)serializer instead, which the already-
    # redirected ``String`` class is exactly the right input for.
    from rclcpp_kit.serialization import (
        deserialize_message as cpp_deserialize_message,
        serialize_message as cpp_serialize_message,
        serialized_message_from_bytes,
        serialized_message_to_bytes,
    )

    def decode(raw_bytes):
        return str(cpp_deserialize_message(
            serialized_message_from_bytes(raw_bytes), String).data)

    suffix = "p%d" % os.getpid()
    rmw = str(get_rmw_implementation_identifier())

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()

    def wait_for(predicate, description, timeout=15.0):
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("timed out waiting for %s" % description)

    qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)

    report = {
        "schema": "rclcppyy.raw-subscription-product-proof/v1",
        "rmw": rmw,
    }

    # --- Scenario 1: basic roundtrip -- raw bytes deserialize back exactly. --
    topic = "/raw_subscription_product/%s/roundtrip" % suffix
    pub_node = Node("raw_roundtrip_pub_%s" % suffix)
    sub_node = Node("raw_roundtrip_sub_%s" % suffix)
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    received = []
    subscription = sub_node.create_subscription(
        String, topic, lambda b: received.append(b), qos, raw=True)
    report["subscription_raw_attribute"] = bool(subscription.raw)
    publisher = pub_node.create_publisher(String, topic, qos)

    wait_for(lambda: publisher.get_subscription_count() >= 1, "discovery")

    payloads = ["raw hello %d" % i for i in range(5)]
    for payload in payloads:
        msg = String()
        msg.data = payload
        publisher.publish(msg)

    wait_for(lambda: len(received) >= len(payloads), "all raw messages")

    report["received_are_bytes"] = all(isinstance(b, bytes) for b in received)
    decoded = [decode(b) for b in received]
    report["roundtrip_ok"] = decoded == payloads
    # Cross-check against a locally (suite-side) serialized message: the
    # bytes rclcppyy's raw path hands back decode with the same C++
    # (de)serializer a locally-built message round-trips through, both
    # directions.
    control_msg = String()
    control_msg.data = payloads[0]
    control_bytes = serialized_message_to_bytes(cpp_serialize_message(control_msg))
    report["deserializes_locally_serialized_bytes_too"] = (
        decode(control_bytes) == payloads[0]
    )

    # --- Scenario 2: raw + event_callbacks("matched") fires like the typed path. ---
    matched_topic = "/raw_subscription_product/%s/matched" % suffix
    matched_events = []
    raw_matched_sub = sub_node.create_subscription(
        String, matched_topic, lambda b: None, qos, raw=True,
        event_callbacks=SubscriptionEventCallbacks(
            matched=lambda info: matched_events.append(int(info.current_count)),
            use_default_callbacks=False))
    matched_pub = pub_node.create_publisher(String, matched_topic, qos)  # noqa: F841

    wait_for(lambda: bool(matched_events), "matched event callback")
    report["raw_matched_event_fired"] = bool(matched_events)
    report["raw_subscription_has_event_handler"] = any(
        handler.event_type == "matched"
        for handler in raw_matched_sub.event_handlers)

    # --- Scenario 3: raw + content_filter_options fails closed on Cyclone. ---
    cf_topic = "/raw_subscription_product/%s/content_filter" % suffix
    cf_raised = None
    cf_created = False
    try:
        cf_sub = sub_node.create_subscription(
            String, cf_topic, lambda b: None, qos,
            raw=True,
            content_filter_options=ContentFilterOptions(
                filter_expression="data = %0", expression_parameters=["'x'"]))
        cf_created = cf_sub is not None
    except ContentFilterUnsupportedError as exc:
        cf_raised = str(exc)
    report["raw_content_filter_raised"] = cf_raised is not None
    report["raw_content_filter_mentions_rmw"] = (
        cf_raised is not None and rmw in cf_raised)
    report["raw_content_filter_created_without_raising"] = cf_created
    cf_control = sub_node.create_subscription(
        String, cf_topic, lambda b: None, qos, raw=True)
    report["raw_content_filter_control_ok"] = cf_control is not None

    # --- Scenario 4: raw + qos_overriding_options fails closed (not silently ignored). ---
    qo_topic = "/raw_subscription_product/%s/qos_overriding" % suffix
    qo_raised = None
    qo_created = False
    try:
        qo_sub = sub_node.create_subscription(
            String, qo_topic, lambda b: None, qos,
            raw=True, qos_overriding_options=QoSOverridingOptions.with_default_policies())
        qo_created = qo_sub is not None
    except ValueError as exc:
        qo_raised = str(exc)
    report["raw_qos_overriding_raised"] = qo_raised is not None
    report["raw_qos_overriding_created_without_raising"] = qo_created
    qo_control = sub_node.create_subscription(
        String, qo_topic, lambda b: None, qos, raw=True)
    report["raw_qos_overriding_control_ok"] = qo_control is not None

    # --- Scenario 5: raw + a 2-arg (message_info-style) callback fails closed. ---
    mi_topic = "/raw_subscription_product/%s/message_info" % suffix
    mi_raised = None
    mi_created = False
    try:
        mi_sub = sub_node.create_subscription(
            String, mi_topic, lambda b, info: None, qos, raw=True)
        mi_created = mi_sub is not None
    except ValueError as exc:
        mi_raised = str(exc)
    report["raw_message_info_raised"] = mi_raised is not None
    report["raw_message_info_created_without_raising"] = mi_created
    mi_control = sub_node.create_subscription(
        String, mi_topic, lambda b: None, qos, raw=True)
    report["raw_message_info_control_ok"] = mi_control is not None

    for node in (pub_node, sub_node):
        executor.remove_node(node)
        node.destroy_node()
    rclpy.shutdown()

    print(PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
