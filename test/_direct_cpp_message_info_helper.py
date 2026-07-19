#!/usr/bin/env python3
"""Live direct-C++ MessageInfo contract for copy and lease subscriptions."""

import gc
import importlib
import json
import os
import select
import subprocess
import sys
import time
import uuid


PROTOCOL_PREFIX = "@@RCLCPPYY_RELAY_BOUNDARY_V1@@"
INFO_KEYS = {
    "source_timestamp",
    "received_timestamp",
    "publication_sequence_number",
    "reception_sequence_number",
}


def run_stock_publisher(topic, value):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import UInt64

    rclpy.init(args=[])
    node = Node("message_info_stock_publisher_%d" % os.getpid())
    publisher = node.create_publisher(UInt64, topic, 10)
    deadline = time.monotonic() + 15.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert publisher.get_subscription_count() == 1
    publisher.publish(UInt64(data=int(value)))
    time.sleep(0.2)
    node.destroy_publisher(publisher)
    node.destroy_node()
    rclpy.shutdown()
    print("STOCK_MESSAGE_INFO_PUBLISHER_OK", flush=True)


def read_protocol(process, event, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready, _, _ = select.select(
            [process.stdout], [], [], max(0.0, deadline - time.monotonic()))
        if not ready:
            break
        line = process.stdout.readline()
        if not line:
            break
        if not line.startswith(PROTOCOL_PREFIX):
            continue
        document = json.loads(line[len(PROTOCOL_PREFIX):])
        assert document["event"] == event, document
        return document
    stderr = process.stderr.read() if process.poll() is not None else ""
    raise AssertionError("timed out waiting for AOT %s: %s" % (event, stderr))


def write_control(process, command):
    process.stdin.write(command + "\n")
    process.stdin.flush()


def assert_message_info(info):
    assert type(info) is dict
    assert set(info) == INFO_KEYS
    assert type(info["source_timestamp"]) is int
    assert info["source_timestamp"] > 0
    assert type(info["received_timestamp"]) is int
    for name in (
        "publication_sequence_number",
        "reception_sequence_number",
    ):
        value = info[name]
        assert value is None or (type(value) is int and value >= 0)


def run_direct(mode, aot_executable):
    if mode not in ("copy", "lease"):
        raise AssertionError("mode must be copy or lease")
    import rclcppyy

    optimization = ("subscription_shared_lease",) if mode == "lease" else ()
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", optimizations=optimization)

    import cppyy
    import rclpy
    import rclpy.serialization as rclpy_serialization
    from rclcppyy.policy import BackendUnavailableError
    from rclpy.node import Node
    from rclpy.subscription import Subscription
    from std_msgs.msg import UInt64

    def forbidden_boundary(*_args, **_kwargs):
        raise AssertionError("a Python-message conversion or serialization path ran")

    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    bringup.convert_python_msg_to_cpp = forbidden_boundary
    serialization.serialize_message = forbidden_boundary
    serialization.deserialize_message = forbidden_boundary
    rclpy_serialization.serialize_message = forbidden_boundary
    rclpy_serialization.deserialize_message = forbidden_boundary

    rclpy.init(args=[])
    node = Node("direct_message_info_%s_%d" % (mode, os.getpid()))
    prefix = "/direct_cpp/message_info/%s/p%d" % (mode, os.getpid())

    def reject_without_entity(callback, suffix):
        before = len(tuple(node.subscriptions))
        try:
            node.create_subscription(
                UInt64, prefix + "/rejected/" + suffix, callback, 10)
        except (BackendUnavailableError, RuntimeError, TypeError):
            pass
        else:
            raise AssertionError("invalid subscription callback created an entity")
        assert len(tuple(node.subscriptions)) == before

    def no_arguments():
        return None

    def three_arguments(_message, _info, _extra):
        return None

    async def coroutine_message(_message):
        return None

    async def coroutine_info(_message, _info):
        return None

    reject_without_entity(no_arguments, "zero")
    reject_without_entity(three_arguments, "three")
    reject_without_entity(coroutine_message, "coroutine_message")
    reject_without_entity(coroutine_info, "coroutine_info")
    print("DIRECT_CPP_MESSAGE_INFO_FAIL_CLOSED_OK", mode, flush=True)

    one_messages = []
    stock_received = []
    aot_received = []
    failing_messages = []
    one_topic = prefix + "/one"
    stock_topic = prefix + "/stock"
    aot_input = prefix + "/aot_input"
    aot_output = prefix + "/aot_output"
    failing_topic = prefix + "/failing"

    one_publisher = node.create_publisher(UInt64, one_topic, 10)
    one_subscription = node.create_subscription(
        UInt64, one_topic, one_messages.append, 10)
    stock_subscription = node.create_subscription(
        UInt64,
        stock_topic,
        lambda message, info: stock_received.append((message, info)),
        10,
    )
    aot_publisher = node.create_publisher(UInt64, aot_input, 10)
    aot_subscription = node.create_subscription(
        UInt64,
        aot_output,
        lambda message, info: aot_received.append((message, info)),
        10,
    )
    failing_publisher = node.create_publisher(UInt64, failing_topic, 10)

    def failing_callback(message, info):
        assert type(message) is UInt64
        assert_message_info(info)
        failing_messages.append(message)
        raise RuntimeError("direct MessageInfo callback sentinel")

    failing_subscription = node.create_subscription(
        UInt64, failing_topic, failing_callback, 10)

    assert one_subscription._callback_type is Subscription.CallbackType.MessageOnly
    for subscription in (
        stock_subscription, aot_subscription, failing_subscription,
    ):
        assert subscription._callback_type is Subscription.CallbackType.WithMessageInfo

    def spin_until(condition, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert condition()

    spin_until(lambda: one_publisher.get_subscription_count() == 1)
    one_publisher.publish(UInt64(data=17))
    spin_until(lambda: len(one_messages) == 1)
    assert type(one_messages[0]) is UInt64
    assert int(one_messages[0].data) == 17
    print("DIRECT_CPP_MESSAGE_INFO_ONE_ARG_OK", mode, flush=True)

    stock_process = subprocess.Popen(
        [sys.executable, __file__, "stock-publisher", stock_topic, "41"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    spin_until(lambda: len(stock_received) == 1)
    stock_stdout, stock_stderr = stock_process.communicate(timeout=10)
    assert stock_process.returncode == 0, (stock_stdout, stock_stderr)
    assert "STOCK_MESSAGE_INFO_PUBLISHER_OK" in stock_stdout
    stock_message, stock_info = stock_received[0]
    assert type(stock_message) is UInt64
    assert int(stock_message.data) == 41
    assert_message_info(stock_info)
    print("DIRECT_CPP_MESSAGE_INFO_STOCK_INTEROP_OK", mode, flush=True)

    token = "message_info_" + uuid.uuid4().hex
    relay = subprocess.Popen(
        [
            aot_executable,
            "relay",
            aot_input,
            aot_output,
            "message_info_aot_relay_" + mode,
            "0",
            "1",
            token,
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    try:
        ready = read_protocol(relay, "ready")
        assert ready["run_token"] == token
        write_control(relay, "START")
        read_protocol(relay, "armed")
        spin_until(
            lambda: (
                aot_publisher.get_subscription_count() == 1
                and aot_subscription.get_publisher_count() == 1
            ))
        aot_publisher.publish(UInt64(data=55))
        spin_until(lambda: len(aot_received) == 1)
        aot_message, aot_info = aot_received[0]
        assert type(aot_message) is UInt64
        assert int(aot_message.data) == 111
        assert_message_info(aot_info)
        write_control(relay, "REPORT")
        report = read_protocol(relay, "report")
        assert report["received"] == 1
        assert report["published"] == 1
        assert report["correct"] is True
        relay_stdout, relay_stderr = relay.communicate(timeout=10)
        assert relay.returncode == 0, (relay_stdout, relay_stderr)
    finally:
        if relay.poll() is None:
            relay.kill()
            relay.wait(timeout=5)
    print("DIRECT_CPP_MESSAGE_INFO_AOT_INTEROP_OK", mode, flush=True)

    spin_until(lambda: failing_publisher.get_subscription_count() == 1)
    failing_publisher.publish(UInt64(data=73))
    deadline = time.monotonic() + 10.0
    while True:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except RuntimeError as error:
            assert "direct MessageInfo callback sentinel" in str(error)
            break
        assert time.monotonic() < deadline
    assert len(failing_messages) == 1
    assert node.destroy_subscription(failing_subscription)
    assert node.destroy_publisher(failing_publisher)
    print("DIRECT_CPP_MESSAGE_INFO_EXCEPTION_OK", mode, flush=True)

    all_subscriptions = (
        one_subscription, stock_subscription, aot_subscription,
    )
    for subscription in all_subscriptions:
        assert "rclcpp::Subscription" in str(
            getattr(type(subscription.native_entity), "__cpp_name__", ""))
    wrappers = [subscription._native for subscription in all_subscriptions]
    if mode == "copy":
        assert [wrapper.owning_cpp_copy_count for wrapper in wrappers] == [1, 1, 1]
        assert wrappers[0].creation_route in (
            "prebuilt_subscription_trampoline", "rclcpp_template")
        assert all(
            wrapper.creation_route == "rclcpp_template_with_message_info"
            for wrapper in wrappers[1:])
    else:
        for wrapper in wrappers:
            stats = wrapper.stats()
            assert stats.leases == 1
            assert stats.message_deep_copies == 0
            assert stats.shared_control_blocks == 1
            assert stats.shared_owner_acquisitions == 1
            assert stats.python_boundary_crossings == 1
            assert stats.exceptions == 0
            assert cppyy.addressof(
                one_messages[0]
                if wrapper is wrappers[0]
                else stock_message
                if wrapper is wrappers[1]
                else aot_message
            ) == wrapper.last_message_address

    subscription_records = [
        record for record in rclcppyy.status()["entities"]
        if record["metadata"].get("entity_type") == "subscription"
        and str(record["metadata"].get("topic", "")).startswith(prefix)
    ]
    assert len(subscription_records) == 4
    info_records = [
        record for record in subscription_records
        if record["metadata"]["message_info"]
    ]
    assert len(info_records) == 3
    assert all("native_message_info" in record["policies"] for record in info_records)
    assert sum(
        not record["metadata"]["message_info"]
        for record in subscription_records) == 1
    print("DIRECT_CPP_MESSAGE_INFO_CONTRACT_OK", mode, flush=True)

    retained = (
        one_messages[0], stock_message, stock_info, aot_message, aot_info,
    )
    node.destroy_node()
    rclpy.shutdown()
    gc.collect()
    assert all(wrapper.closed for wrapper in wrappers)
    retained[0].data = 19
    retained[1].data = 43
    retained[3].data = 113
    assert [int(retained[index].data) for index in (0, 1, 3)] == [19, 43, 113]
    assert_message_info(retained[2])
    assert_message_info(retained[4])
    assert not rclpy.ok()
    print("DIRECT_CPP_MESSAGE_INFO_RETAINED_TEARDOWN_OK", mode, flush=True)


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "stock-publisher":
        assert len(sys.argv) == 4
        run_stock_publisher(sys.argv[2], sys.argv[3])
        return
    if len(sys.argv) != 3:
        raise SystemExit("usage: helper copy|lease AOT_EXECUTABLE")
    run_direct(sys.argv[1], sys.argv[2])


if __name__ == "__main__":
    main()
