#!/usr/bin/env python3
"""Live endpoint proof that the direct public surface is not a superset of
stock (Finding D, identity-proof plan §2 file 5).

Scope is exactly node + publisher, per plan -- subscription/action/executor/
callback-group classes carry a separately-tracked backend superset (~200+
names) pending a dedicated hygiene slice, and are deliberately out of scope
here.

Builds a node and publisher under the requested backend and emits their
concrete instance types' public ``dir()`` sets, plus a static
(non-instantiated) class-level ``dir()`` check of ``rclpy.lifecycle.
LifecycleNode``/``LifecyclePublisher`` -- the two classes Finding D
originally identified as carrying the six leaked names. The lifecycle check
is class-level rather than live because constructing a LifecycleNode under
the direct profile currently crashes for an unrelated reason (a
``rclpy.type_support.check_is_valid_srv_type`` / ``ServiceEventInfo`` /
``Time`` type-support defect -- see the identity-proof lane's Phase A
report); this probe works around that by not instantiating it.
"""

from __future__ import annotations

import argparse
import json
import os


PROBE_PREFIX = "RCLCPPYY_IDENTITY_NO_PUBLIC_SUPERSET_PROBE "


def _public_dir(value):
    return sorted(name for name in dir(value) if not name.startswith("_"))


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
import rclpy.lifecycle as lifecycle  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


static_types = {
    "LifecycleNode": _public_dir(lifecycle.LifecycleNode),
    "LifecyclePublisher": _public_dir(lifecycle.LifecyclePublisher),
}

rclpy.init()
node = None
try:
    node = Node("identity_no_superset_probe_%d" % os.getpid())
    prefix = "/identity_no_superset_probe/p%d" % os.getpid()
    publisher = node.create_publisher(UInt64, prefix, 10)

    live_types = {
        "Node": _public_dir(type(node)),
        "Publisher": _public_dir(type(publisher)),
    }

    payload = {
        "backend": args.backend,
        "live_types": live_types,
        "static_types": static_types,
    }
    print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
finally:
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
