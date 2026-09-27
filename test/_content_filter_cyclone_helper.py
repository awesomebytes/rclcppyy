#!/usr/bin/env python3
"""Live Cyclone fail-closed proof for Slice 2 content-filtered subscriptions
(PLAN-qos-events-product.md), through the public rclpy-shaped
``Node.create_subscription`` API.

rmw_cyclonedds_cpp has no listener for content filtering at all (see the
suite's ``ContentFilterUnsupported`` docstring), and stock rclpy silently
returns an *unfiltered* subscription in that case -- the product refuses to
reproduce that silent narrowing and instead raises a clear, product-level
``rclcppyy.direct_cpp.ContentFilterUnsupportedError`` (documented
stricter-than-stock divergence, plan S3/S6#3). This proves: (a) the error is
raised instead of any subscription being returned, and (b) an unfiltered
subscription on the exact same topic still works right afterwards (the node
itself is left in a clean, usable state -- the failure is contained to the
one requested entity).

Keyed on the RUNTIME rmw identifier, not the ``RMW_IMPLEMENTATION`` env var
-- same rule as the Fast DDS companion helper.
"""
import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "77")

PREFIX = "CONTENT_FILTER_CYCLONE_REPORT="
EXPECTED_RMW = "rmw_cyclonedds_cpp"


def main():
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.subscription_content_filter_options import ContentFilterOptions
    from rclpy.utilities import get_rmw_implementation_identifier
    from std_msgs.msg import UInt64

    from rclcppyy.direct_cpp import ContentFilterUnsupportedError

    actual_rmw = str(get_rmw_implementation_identifier())
    assert actual_rmw == EXPECTED_RMW, (
        "expected the runtime rmw to be %s, got %s" % (EXPECTED_RMW, actual_rmw))

    # "p" prefix -- a bare PID would make the topic token start with a digit,
    # which rclcpp's topic-name validation rejects.
    suffix = "p%d" % os.getpid()
    topic = "/content_filter_product/%s/cyclone" % suffix
    qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)

    rclpy.init(args=[])
    node = Node("content_filter_cyclone_%s" % suffix)

    raised_message = None
    subscription_created = False
    try:
        subscription = node.create_subscription(
            UInt64, topic, lambda _msg: None, qos,
            content_filter_options=ContentFilterOptions(
                filter_expression="data = %0", expression_parameters=["42"]))
        subscription_created = subscription is not None
    except ContentFilterUnsupportedError as exc:
        raised_message = str(exc)

    # Control: an unfiltered subscription on the exact same topic still works
    # -- the failure is contained to the one requested (filtered) entity.
    control = node.create_subscription(UInt64, topic, lambda _msg: None, qos)
    control_ok = control is not None
    node.destroy_subscription(control)
    node.destroy_node()
    rclpy.shutdown()

    report = {
        "schema": "rclcppyy.content-filter-product-cyclone-proof/v1",
        "rmw": actual_rmw,
        "raised_content_filter_unsupported": raised_message is not None,
        "raised_message_mentions_rmw": (
            raised_message is not None and EXPECTED_RMW in raised_message),
        "subscription_created_without_raising": subscription_created,
        "control_subscription_ok": control_ok,
    }
    print(PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
