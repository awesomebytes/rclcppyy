#!/usr/bin/env python3
"""Live endpoint proof for direct-profile entity type identity + isinstance.

Identity-proof plan §2 file 4: builds a node and one publisher/subscription
under the direct profile and proves the public ``isinstance`` contract holds
against the currently-exposed ``rclpy.node.Node`` / ``rclpy.publisher.Publisher``
/ ``rclpy.subscription.Subscription`` aliases, and that payload message
objects carry the generated C++ representation (Finding C, behaviorally).
This is a behavioral proof, not an annotation target: entity/node rows belong
to the node/lifecycle lanes and the allocation plan §1.2 stand-in.
"""

from __future__ import annotations

import argparse
import json
import os
import time


PROBE_PREFIX = "RCLCPPYY_IDENTITY_ENTITIES_PROBE "


def _type_descriptor(value_type):
    return {
        "module": value_type.__module__,
        "qualname": value_type.__qualname__,
        "has_cpp_name": hasattr(value_type, "__cpp_name__"),
    }


def _spin_until(node, condition, timeout_s=10.0):
    import rclpy

    deadline = time.monotonic() + timeout_s
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition(), "condition did not become true before the deadline"


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from rclpy.subscription import Subscription  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init()
node = None
try:
    node = Node("identity_entities_probe_%d" % os.getpid())
    prefix = "/identity_entities_probe/p%d" % os.getpid()
    received = []
    publisher = node.create_publisher(UInt64, prefix, 10)
    subscription = node.create_subscription(UInt64, prefix, received.append, 10)

    isinstance_checks = {
        "node_is_Node": isinstance(node, Node),
        "publisher_is_Publisher": isinstance(publisher, Publisher),
        "subscription_is_Subscription": isinstance(subscription, Subscription),
    }

    types = {
        "node": _type_descriptor(type(node)),
        "publisher": _type_descriptor(type(publisher)),
        "subscription": _type_descriptor(type(subscription)),
    }
    public_aliases = {
        "Node": _type_descriptor(Node),
        "Publisher": _type_descriptor(Publisher),
        "Subscription": _type_descriptor(Subscription),
    }

    outgoing = UInt64(data=97)
    constructed_payload_type = _type_descriptor(type(outgoing))

    _spin_until(node, lambda: publisher.get_subscription_count() == 1)
    publisher.publish(outgoing)
    _spin_until(node, lambda: len(received) == 1)
    received_payload_type = _type_descriptor(type(received[0]))
    received_value = int(received[0].data)

    payload = {
        "backend": args.backend,
        "isinstance": isinstance_checks,
        "types": types,
        "public_aliases": public_aliases,
        "constructed_payload_type": constructed_payload_type,
        "received_payload_type": received_payload_type,
        "received_value": received_value,
    }
    print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
finally:
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
