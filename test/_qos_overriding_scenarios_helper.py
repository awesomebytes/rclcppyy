#!/usr/bin/env python3
"""Live per-scenario proof for Slice 3 QoS overriding options
(PLAN-qos-events-product.md), through the public rclpy-shaped
``Node.create_subscription``/``Node.create_publisher`` API.

QoS overriding is parameter-layer, RMW-independent -- unlike slice 2's
content filter, the positive path IS reachable under direct_cpp's
Jazzy/Cyclone-only gate, so this proves the real thing: declared node
parameters, an override actually changing the created entity's effective
QoS (queried natively via ``native_entity.get_actual_qos()``, the same
idiom ``_direct_cpp_qos_helper.py`` already uses), and the fail-closed gate
on anything beyond the exact default policy-kind set.

The suite's own foundation test (cppyy_kit's
``_direct_qos_overriding_helper.py``) already proves the mechanism at its
layer using the identical ``get_actual_qos().get_rmw_qos_profile()`` idiom;
this only proves the product's translation from stock's
``QoSOverridingOptions`` through the public API.

``create_publisher``'s ``qos_overriding_options`` reject is untouched by
this slice (suite gap: no publisher-side attachment point in
``direct_entities`` -- see the plan's S3 addendum) -- scenario I proves
that reject still fires exactly as before.
"""
import json
import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "77")

PREFIX = "QOS_OVERRIDING_PRODUCT_REPORT="
DEFAULT_POLICY_NAMES = ("history", "depth", "reliability")
REQUESTED_DEPTH = 8
OVERRIDE_DEPTH = 3


def main():
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSPolicyKind, QoSProfile, ReliabilityPolicy
    from rclpy.qos_overriding_options import QoSOverridingOptions
    from rclpy.subscription_content_filter_options import ContentFilterOptions
    from rcl_interfaces.msg import SetParametersResult
    from std_msgs.msg import UInt64

    from rclcppyy.direct_cpp import ContentFilterUnsupportedError
    from rclcppyy.policy import BackendUnavailableError

    suffix = "p%d" % os.getpid()
    qos = QoSProfile(
        depth=REQUESTED_DEPTH, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)

    rclpy.init(args=[])
    node = Node("qos_overriding_%s" % suffix)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def declared_values(topic, entity_kind):
        return {
            name: {
                "declared": node.has_parameter(
                    "qos_overrides.%s.%s.%s" % (topic, entity_kind, name)),
            }
            for name in DEFAULT_POLICY_NAMES
        }

    report = {}

    # --- Scenario A: with_default_policies() declares the three parameters,
    # with no override, at the requested default value.
    topic_a = "/qos_overriding_product/%s/default" % suffix
    subscription_a = node.create_subscription(
        UInt64, topic_a, lambda _msg: None, qos,
        qos_overriding_options=QoSOverridingOptions.with_default_policies())
    report["default_declared"] = declared_values(topic_a, "subscription")
    report["default_depth_value"] = int(
        node.get_parameter("qos_overrides.%s.subscription.depth" % topic_a).value)
    report["default_actual_depth"] = int(
        subscription_a.native_entity.get_actual_qos().get_rmw_qos_profile().depth)
    node.destroy_subscription(subscription_a)

    # --- Scenario B: an override parameter set at node-construction time
    # actually changes the created entity's effective QoS.
    topic_b = "/qos_overriding_product/%s/overridden" % suffix
    override_param = "qos_overrides.%s.subscription.depth" % topic_b
    override_node = Node(
        "qos_overriding_override_%s" % suffix,
        parameter_overrides=[Parameter(override_param, value=OVERRIDE_DEPTH)])
    subscription_b = override_node.create_subscription(
        UInt64, topic_b, lambda _msg: None, qos,
        qos_overriding_options=QoSOverridingOptions.with_default_policies())
    report["override_declared_value"] = int(
        override_node.get_parameter(override_param).value)
    report["override_actual_depth"] = int(
        subscription_b.native_entity.get_actual_qos().get_rmw_qos_profile().depth)
    override_node.destroy_subscription(subscription_b)
    override_node.destroy_node()

    # --- Scenario C/D/E: fail-closed on custom subset / callback / entity_id.
    def _raises_backend_unavailable(build):
        try:
            build()
        except BackendUnavailableError as exc:
            return True, str(exc)
        return False, None

    custom_subset_raised, custom_subset_message = _raises_backend_unavailable(
        lambda: node.create_subscription(
            UInt64, "/qos_overriding_product/%s/custom_subset" % suffix,
            lambda _msg: None, qos,
            qos_overriding_options=QoSOverridingOptions(
                policy_kinds=(QoSPolicyKind.HISTORY,))))
    callback_raised, callback_message = _raises_backend_unavailable(
        lambda: node.create_subscription(
            UInt64, "/qos_overriding_product/%s/callback" % suffix,
            lambda _msg: None, qos,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(
                callback=lambda _qos: SetParametersResult(successful=True))))
    entity_id_raised, entity_id_message = _raises_backend_unavailable(
        lambda: node.create_subscription(
            UInt64, "/qos_overriding_product/%s/entity_id" % suffix,
            lambda _msg: None, qos,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(
                entity_id="custom")))
    report["custom_subset_raised"] = custom_subset_raised
    report["custom_subset_message"] = custom_subset_message
    report["callback_raised"] = callback_raised
    report["callback_message"] = callback_message
    report["entity_id_raised"] = entity_id_raised
    report["entity_id_message"] = entity_id_message

    # --- Scenario F: a non-QoSOverridingOptions value raises TypeError
    # before any native entity is constructed.
    wrong_type_raised = None
    try:
        node.create_subscription(
            UInt64, "/qos_overriding_product/%s/wrong_type" % suffix,
            lambda _msg: None, qos, qos_overriding_options=True)
    except TypeError as exc:
        wrong_type_raised = str(exc)
    report["wrong_type_raised"] = wrong_type_raised is not None
    report["wrong_type_message"] = wrong_type_raised

    # --- Scenario G: qos_overriding combined with event_callbacks -- both
    # must translate (plan's explicit combo requirement).
    topic_g = "/qos_overriding_product/%s/combo_events" % suffix
    matched_events = []
    subscription_g = node.create_subscription(
        UInt64, topic_g, lambda _msg: None, qos,
        qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        event_callbacks=SubscriptionEventCallbacks(
            matched=lambda info: matched_events.append(int(info.current_count)),
            use_default_callbacks=False))
    publisher_g = node.create_publisher(UInt64, topic_g, qos)
    deadline = time.monotonic() + 10.0
    while not matched_events and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    report["combo_events_declared"] = declared_values(topic_g, "subscription")
    report["combo_events_matched_fired"] = len(matched_events) >= 1
    node.destroy_publisher(publisher_g)
    node.destroy_subscription(subscription_g)

    # --- Scenario H: qos_overriding combined with content_filter_options --
    # still fails closed with ContentFilterUnsupportedError specifically
    # (proves the qos_overriding translation didn't derail slice 2's gate).
    combo_filter_raised = None
    combo_filter_created = False
    try:
        combo_subscription = node.create_subscription(
            UInt64, "/qos_overriding_product/%s/combo_filter" % suffix,
            lambda _msg: None, qos,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            content_filter_options=ContentFilterOptions(
                filter_expression="data = %0", expression_parameters=["1"]))
        combo_filter_created = combo_subscription is not None
    except ContentFilterUnsupportedError as exc:
        combo_filter_raised = str(exc)
    report["combo_filter_raised_content_filter_unsupported"] = (
        combo_filter_raised is not None)
    report["combo_filter_created_without_raising"] = combo_filter_created

    # --- Scenario I: create_publisher's qos_overriding_options reject is
    # untouched by this slice.
    publisher_reject_raised = None
    try:
        node.create_publisher(
            UInt64, "/qos_overriding_product/%s/publisher_reject" % suffix, qos,
            qos_overriding_options=QoSOverridingOptions.with_default_policies())
    except BackendUnavailableError as exc:
        publisher_reject_raised = str(exc)
    report["publisher_reject_raised"] = publisher_reject_raised is not None
    report["publisher_reject_message"] = publisher_reject_raised

    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()

    print(PREFIX + json.dumps({
        "schema": "rclcppyy.qos-overriding-product-proof/v1",
        **report,
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
