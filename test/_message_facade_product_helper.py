#!/usr/bin/env python3
"""Stock/facade differential workflow using exact stock rclpy objects."""

import copy
import json
import os
import pickle
import sys
import time


MODE, EXECUTOR_KIND = sys.argv[1:3]
assert MODE in ("stock", "facade")
assert EXECUTOR_KIND in ("single", "multi")

if MODE == "facade":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="message_facade")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.event_handler import SubscriptionEventCallbacks  # noqa: E402
from rclpy.executors import (  # noqa: E402
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from rclpy.subscription import Subscription  # noqa: E402
from rclpy.subscription_content_filter_options import (  # noqa: E402
    ContentFilterOptions,
)
from rclpy._rclpy_pybind11 import InvalidHandle  # noqa: E402
from std_msgs.msg import Float64, String, UInt64  # noqa: E402


TIMEOUT_S = 10.0


def _executor(context):
    if EXECUTOR_KIND == "single":
        return SingleThreadedExecutor(context=context)
    return MultiThreadedExecutor(num_threads=2, context=context)


def _spin_until(executor, condition):
    deadline = time.monotonic() + TIMEOUT_S
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        time.sleep(0.001)
    assert condition()


def _destroy(executor, node, context, publishers, subscriptions):
    for publisher in publishers:
        assert node.destroy_publisher(publisher)
    for subscription in subscriptions:
        assert node.destroy_subscription(subscription)
    executor.remove_node(node)
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    context.shutdown()


def _run_cycle(index, exercise_fallbacks):
    context = Context()
    context.init(args=[])
    node = rclpy.create_node(
        "facade_%s_%d_%d" % (EXECUTOR_KIND, os.getpid(), index),
        context=context,
    )
    executor = _executor(context)
    executor.add_node(node)
    received_string = []
    received_uint64 = []
    prefix = "/message_facade/%s/cycle_%d" % (EXECUTOR_KIND, index)

    def string_callback(message, info):
        received_string.append((message, info))

    string_subscription = node.create_subscription(
        String, prefix + "/string", string_callback, 10)
    uint64_subscription = node.create_subscription(
        UInt64,
        prefix + "/uint64",
        lambda message: received_uint64.append(message),
        10,
    )
    string_publisher = node.create_publisher(String, prefix + "/string", 10)
    uint64_publisher = node.create_publisher(UInt64, prefix + "/uint64", 10)
    publishers = [string_publisher, uint64_publisher]
    subscriptions = [string_subscription, uint64_subscription]

    assert type(context) is Context
    assert type(node) is Node
    assert type(string_publisher) is Publisher
    assert type(string_subscription) is Subscription
    assert type(executor) is (
        SingleThreadedExecutor
        if EXECUTOR_KIND == "single" else MultiThreadedExecutor)
    assert node.context is context
    assert node.default_callback_group.has_entity(string_subscription)
    assert string_subscription in node.subscriptions
    assert string_publisher in node.publishers

    _spin_until(
        executor,
        lambda: (string_publisher.get_subscription_count() >= 1 and
                 uint64_publisher.get_subscription_count() >= 1),
    )

    sample = String(data="copy-value")
    shallow = copy.copy(sample)
    deep = copy.deepcopy(sample)
    roundtripped = pickle.loads(pickle.dumps(sample))
    assert shallow == sample and shallow is not sample
    assert deep == sample and deep is not sample
    assert roundtripped == sample and type(roundtripped) is String
    unchecked = String(check_fields=False, ignored="stock-compatible")
    assert unchecked._check_fields is False
    try:
        String(check_fields=True, ignored="rejected")
    except AssertionError:
        pass
    else:
        raise AssertionError("checked message accepted an unknown field")

    if MODE == "facade":
        from rclcpp_kit import borrowed_publish, message_facade

        def reject_conversion(*_args, **_kwargs):
            raise AssertionError("Python-to-C++ whole-message conversion was used")

        borrowed_publish.convert_python_msg_to_cpp = reject_conversion
        for entity in publishers + subscriptions:
            assert entity.msg_type in (String, UInt64)
        for publisher in publishers:
            binding = message_facade.binding_for_type(publisher.msg_type)
            assert publisher._rclcppyy_publish_route.message_type is (
                binding.original_type)
        for subscription in subscriptions:
            binding = message_facade.binding_for_type(subscription.msg_type)
            assert subscription._rclcppyy_take_route.original_type is (
                binding.original_type)

    for value in ("first", "second"):
        string_publisher.publish(String(data=value))
    for value in (7, 2**63 + 9):
        uint64_publisher.publish(UInt64(data=value))
    _spin_until(
        executor,
        lambda: len(received_string) == 2 and len(received_uint64) == 2,
    )

    assert [item[0].data for item in received_string] == ["first", "second"]
    assert [item.data for item in received_uint64] == [7, 2**63 + 9]
    info_keys = sorted(received_string[0][1])
    assert info_keys == [
        "publication_sequence_number",
        "received_timestamp",
        "reception_sequence_number",
        "source_timestamp",
    ]
    first_retained = received_string[0][0]
    received_string[1][0].data = "changed"
    assert first_retained.data == "first"

    if MODE == "facade":
        string_addresses = [
            cppyy.addressof(item[0]._rclcpp_kit_cpp_message)
            for item in received_string
        ]
        assert len(set(string_addresses)) == 2
        assert string_publisher._rclcppyy_publish_route._scratch_stats()[
            "publishes"] == 2
        assert string_subscription._rclcppyy_take_route.stats()["takes"] == 2

    identity = (node.get_name(), node.get_namespace())
    assert node.get_node_names_and_namespaces().count(identity) == 1
    observations = {
        "executor": EXECUTOR_KIND,
        "string_values": ["first", "second"],
        "uint64_values": [7, 2**63 + 9],
        "string_repr": repr(String(data="visible")),
        "uint64_repr": repr(UInt64(data=23)),
        "string_fields": String.get_fields_and_field_types(),
        "uint64_fields": UInt64.get_fields_and_field_types(),
        "message_info_keys": info_keys,
        "public_type_names": [
            String.__module__ + "." + String.__name__,
            UInt64.__module__ + "." + UInt64.__name__,
        ],
        "stock_object_identities": [
            type(node).__name__,
            type(string_publisher).__name__,
            type(string_subscription).__name__,
            type(executor).__name__,
        ],
    }

    if exercise_fallbacks:
        unsupported_received = []
        unsupported_subscription = node.create_subscription(
            Float64,
            prefix + "/unsupported",
            lambda message: unsupported_received.append(message.data),
            10,
        )
        unsupported_publisher = node.create_publisher(
            Float64, prefix + "/unsupported", 10)
        publishers.append(unsupported_publisher)
        subscriptions.append(unsupported_subscription)
        assert not hasattr(unsupported_publisher, "_rclcppyy_publish_route")
        assert not hasattr(unsupported_subscription, "_rclcppyy_take_route")
        _spin_until(
            executor,
            lambda: unsupported_publisher.get_subscription_count() >= 1,
        )
        unsupported_publisher.publish(Float64(data=3.25))
        _spin_until(executor, lambda: unsupported_received == [3.25])

        if MODE == "facade":
            raw_received = []
            raw_subscription = node.create_subscription(
                String,
                prefix + "/raw",
                lambda value: raw_received.append(value),
                10,
                raw=True,
            )
            raw_publisher = node.create_publisher(String, prefix + "/raw", 10)
            subscriptions.append(raw_subscription)
            publishers.append(raw_publisher)
            assert not hasattr(raw_subscription, "_rclcppyy_take_route")
            _spin_until(
                executor, lambda: raw_publisher.get_subscription_count() >= 1)
            raw_publisher.publish(String(data="raw"))
            _spin_until(executor, lambda: len(raw_received) == 1)
            assert isinstance(raw_received[0], bytes)

            event_subscription = node.create_subscription(
                String,
                prefix + "/events",
                lambda _message: None,
                10,
                event_callbacks=SubscriptionEventCallbacks(
                    use_default_callbacks=False),
            )
            subscriptions.append(event_subscription)
            assert not hasattr(event_subscription, "_rclcppyy_take_route")

            filtered_subscription = node.create_subscription(
                UInt64,
                prefix + "/filtered",
                lambda _message: None,
                10,
                content_filter_options=ContentFilterOptions("data = 7", []),
            )
            subscriptions.append(filtered_subscription)
            assert not hasattr(filtered_subscription, "_rclcppyy_take_route")

            class CustomPublisher(Publisher):
                pass

            custom_publisher = node.create_publisher(
                String,
                prefix + "/custom",
                10,
                publisher_class=CustomPublisher,
            )
            publishers.append(custom_publisher)
            assert type(custom_publisher) is CustomPublisher
            assert not hasattr(custom_publisher, "_rclcppyy_publish_route")

            original_route = string_publisher._rclcppyy_publish_route

            class FailingPublisherRoute:
                def publish(self, _publisher, _message):
                    raise RuntimeError("deliberate facade publish failure")

            string_publisher._rclcppyy_publish_route = FailingPublisherRoute()
            string_publisher.publish(String(data="stock-failure-fallback"))
            string_publisher._rclcppyy_publish_route = original_route
            _spin_until(executor, lambda: len(received_string) == 3)
            assert received_string[-1][0].data == "stock-failure-fallback"

            original_take_route = string_subscription._rclcppyy_take_route

            class FailingSubscriptionRoute:
                def take(self, _subscription):
                    raise RuntimeError("deliberate facade take failure")

            string_subscription._rclcppyy_take_route = FailingSubscriptionRoute()
            try:
                executor._take_subscription(string_subscription)
            except RuntimeError as exception:
                assert "deliberate facade take failure" in str(exception)
            else:
                raise AssertionError("failed take was silently retried")
            finally:
                string_subscription._rclcppyy_take_route = original_take_route

            status = rclcppyy.status()
            direct_entities = [
                item for item in status["entities"]
                if item["backend"] == "cpp" and
                item["metadata"].get("message_type") in (
                    "std_msgs::msg::String", "std_msgs::msg::UInt64")
            ]
            assert direct_entities
            assert all(
                "direct_cpp_message" in item["policies"] and
                "python_to_cpp_message_conversion" not in item["policies"]
                for item in direct_entities
            )
            assert any(
                item["backend"] == "unsupported" and
                item["metadata"].get("operation") == "subscription_take"
                for item in status["operations"]
            )
            assert any(
                item["backend"] == "python" and
                item["metadata"].get("operation") == "publish" and
                "deliberate facade publish failure" in item["reason"]
                for item in status["operations"]
            )
            print("MESSAGE_FACADE_CPP_ROUTES_OK", flush=True)
            print("MESSAGE_FACADE_FALLBACKS_OK", flush=True)

    destroyed_publisher = string_publisher
    destroyed_subscription = string_subscription
    _destroy(executor, node, context, publishers, subscriptions)
    try:
        destroyed_publisher.publish(String(data="destroyed"))
    except InvalidHandle:
        pass
    else:
        raise AssertionError("destroyed publisher remained usable")
    assert executor._take_subscription(destroyed_subscription) is None
    return observations


def main():
    first = _run_cycle(0, exercise_fallbacks=True)
    second = _run_cycle(1, exercise_fallbacks=False)
    assert first == second
    assert not rclpy.ok(), "default context must remain untouched"
    print(
        "MESSAGE_FACADE_OBSERVATIONS=" +
        json.dumps(first, sort_keys=True, separators=(",", ":")),
        flush=True,
    )
    print("MESSAGE_FACADE_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
