#!/usr/bin/env python3
"""Live proof that the subscriber dispatch trampoline pre-caches sub-message
proxy references into a received message's ``__dict__`` before the user
callback runs (see ``rclcpp_kit.direct_entities._submessage_precache`` /
the field-access investigation, approach 3: cppyy resolves a composite
field through a ~87ns proxy-creation path every access; seeding
``__dict__`` once drops repeat access to ~44ns because cppyy's
``__getattribute__`` checks ``__dict__`` first).

Runs twice from ``test_subscriber_precache.py`` -- once with the
``subscription_shared_lease`` optimization enabled (the shared-lease
dispatch path in ``rclcpp_kit.direct_subscription_lease``) and once
without it (the plain owning-copy dispatch path in
``rclcpp_kit.direct_entities``) -- both trampolines pre-cache identically.
"""
import gc
import os
import sys
import time

import rclcppyy


MODE = sys.argv[1] if len(sys.argv) > 1 else "lease"
assert MODE in ("lease", "copy")
INTERFACE = "std_msgs/msg/Header"
OPTIMIZATION = "subscription_shared_lease"

assert rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp",
    optimizations=(OPTIMIZATION,) if MODE == "lease" else (),
    interfaces=(INTERFACE,),
)

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcpp_kit.direct_entities import (  # noqa: E402
    _composite_field_names,
    _submessage_precache,
)
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Header, UInt64  # noqa: E402


assert Header is cppyy.gbl.std_msgs.msg.Header
assert UInt64 is cppyy.gbl.std_msgs.msg.UInt64

# Header has exactly one composite (sub-message) field, `stamp`
# (builtin_interfaces/Time); `frame_id` is a std::string (module
# `cppyy.gbl.std`, not `<package>.msg`), so it is correctly excluded.
assert _composite_field_names(Header) == ("stamp",)
assert _composite_field_names(UInt64) == ()
assert _submessage_precache(UInt64) is None
print("DIRECT_CPP_PRECACHE_FIELD_CLASSIFICATION_OK", flush=True)


class PrecachePair(Node):
    def __init__(self):
        super().__init__("direct_precache_%s_%d" % (MODE, os.getpid()))
        prefix = "/direct_cpp/precache/%s/p%d" % (MODE, os.getpid())
        self.headers = []
        self.integers = []
        self.header_publisher = self.create_publisher(Header, prefix + "/header", 10)
        self.uint64_publisher = self.create_publisher(UInt64, prefix + "/uint64", 10)
        self.header_subscription = self.create_subscription(
            Header, prefix + "/header", self.headers.append, 10)
        self.uint64_subscription = self.create_subscription(
            UInt64, prefix + "/uint64", self.integers.append, 10)


def spin_until(node, condition):
    deadline = time.monotonic() + 10.0
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition()


rclpy.init(args=[])
node = PrecachePair()
spin_until(
    node,
    lambda: (
        node.header_publisher.get_subscription_count() >= 1
        and node.uint64_publisher.get_subscription_count() >= 1
    ),
)
message = Header()
message.stamp.sec = 11
message.stamp.nanosec = 22
message.frame_id = "precache-proof"
node.header_publisher.publish(message)
node.uint64_publisher.publish(UInt64(data=7))
spin_until(node, lambda: len(node.headers) == 1 and len(node.integers) == 1)

received = node.headers[0]
received_uint64 = node.integers[0]

# The composite field was already resolved and cached into __dict__ BEFORE
# the callback (`self.headers.append`) ever ran -- this is the actual
# dispatch-trampoline behavior under test, not just the classification
# helper checked above.
assert "stamp" in received.__dict__
cached_stamp = received.__dict__["stamp"]
assert received.stamp is cached_stamp
assert int(received.stamp.sec) == 11
assert int(received.stamp.nanosec) == 22
assert str(received.frame_id) == "precache-proof"

# The cached proxy is a live, non-owning view into the same C++ memory:
# mutating through it is visible on every subsequent access of the same
# field, cached or not.
cached_stamp.sec = 99
assert int(received.stamp.sec) == 99

# A scalar-only message type has no composite fields, so its dispatch
# trampoline skips pre-caching entirely -- __dict__ stays empty, no
# per-message overhead paid for nothing.
assert received_uint64.__dict__ == {}
assert int(received_uint64.data) == 7
print("DIRECT_CPP_PRECACHE_DISPATCH_%s_OK" % MODE.upper(), flush=True)

node.headers.clear()
node.integers.clear()
node.header_subscription = None
node.uint64_subscription = None
node.header_publisher = None
node.uint64_publisher = None
node.destroy_node()
gc.collect()
rclpy.shutdown()
gc.collect()

# The cached proxy remains a valid, independent view after node/context
# teardown -- the same retained-after-shutdown contract as every other
# direct_cpp message (see _direct_cpp_generic_message_helper.py).
assert int(cached_stamp.sec) == 99
cached_stamp.sec = -1
assert int(received.stamp.sec) == -1
print("DIRECT_CPP_PRECACHE_RETAINED_%s_OK" % MODE.upper(), flush=True)
