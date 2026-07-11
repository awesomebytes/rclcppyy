#!/usr/bin/env python3
"""Helper process for test_pubsub_roundtrip.py (primitive-array case).

Same plain-bringup, rclpy-style pub/sub path as _pubsub_plain_helper.py, but
with a message carrying a non-empty *primitive* sequence field:
std_msgs/Float64MultiArray's ``data`` (declared type ``sequence<double>``).
This is the regression case for the backlog #12 conversion bug: the shared
converter (``convert_python_msg_to_cpp``) used to resolve any non-empty
sequence field as a message-element sequence, which raised for primitive
elements.

Run as a subprocess (not collected by pytest); prints
``PLAIN_PRIMITIVE_ARRAY_OK`` as its last line on success. See the sibling test
for why this runs in its own interpreter.
"""
import time

from rclcppyy.bringup_rclcpp import bringup_rclcpp

# Deadline for the bounded spin loop, measured after the (separate) rclcpp JIT
# bringup so a slow cold compile never eats into it. Generous for CI.
SPIN_DEADLINE_S = 15.0
N_MESSAGES = 5
TOPIC = "rclcppyy_plain_primitive_array_roundtrip"


def main():
    rclcpp = bringup_rclcpp()

    from std_msgs.msg import Float64MultiArray
    # Must work with the unpatched Python message class (no import-hook redirect).
    assert not hasattr(Float64MultiArray, "__smartptr__"), (
        f"expected plain Python Float64MultiArray, got a C++ type: "
        f"{Float64MultiArray.__module__}")
    print("PLAIN_PRIMITIVE_ARRAY_MSG_OK", flush=True)

    if not rclcpp.ok():
        rclcpp.init()

    node = rclcpp.Node("rclcppyy_plain_primitive_array_helper")
    received = []

    def on_msg(msg):
        # msg is a C++ std_msgs::msg::Float64MultiArray proxy; its data is a
        # std::vector<double> -- pull it back into a plain Python list.
        received.append(list(msg.data))

    sub = node.create_subscription(Float64MultiArray, TOPIC, on_msg, 10)  # noqa: F841 - keep alive
    pub = node.create_publisher(Float64MultiArray, TOPIC, 10)

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + SPIN_DEADLINE_S

    # Wait for discovery before publishing (volatile QoS drops early messages).
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)
    assert pub.get_subscription_count() >= 1, "subscription never matched publisher"

    expected = [[float(i), float(i) + 0.5, float(i) * 2.0] for i in range(N_MESSAGES)]
    for payload in expected:
        msg = Float64MultiArray()
        msg.data = payload
        pub.publish(msg)

    while len(received) < N_MESSAGES and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.01)

    assert received == expected, f"primitive array payload mismatch: {received!r} != {expected!r}"

    print("PLAIN_PRIMITIVE_ARRAY_OK", flush=True)
    # A normal return exits cleanly (see _pubsub_plain_helper.py): rclcppyy's
    # ordered teardown brings the rclcpp context down before interpreter
    # finalization, so the return code is deterministic without os._exit.


if __name__ == "__main__":
    main()
