#!/usr/bin/env python3
"""Subprocess worker for test_bt_freeze.

Run with CLING_STANDARD_PCH pointed at the frozen bt_kit PCH (the caller sets it
before this process starts, so the interpreter binds the frozen PCH at the first
cppyy import). Times the header include -- which the freeze must collapse from
~0.83 s to a few ms -- and exercises the typed-port glue that depends on the
frozen force-symbol fix, then prints one JSON line the test parses.
"""
import json
import os
import time

import cppyy
from ament_index_python.packages import get_package_prefix

from rclcppyy.kits import bt_kit, freeze

prefix = get_package_prefix("behaviortree_cpp")
cppyy.add_include_path(os.path.join(prefix, "include"))

_t = time.perf_counter()
cppyy.include("behaviortree_cpp/bt_factory.h")
include_ms = (time.perf_counter() - _t) * 1e3

bt = bt_kit.bringup_bt()

# Typed-port roundtrip -- getInput<int>/setOutput<int> + makePorts all JIT-compile
# in their own modules and each ODR-uses BT::UndefinedAnyType, so this fails on the
# frozen path unless the force-symbol glue resolved it.
seen = {}


def compute(node):
    node.set_output("sum", node.get_input("a", int) + 100)
    return bt_kit.SUCCESS


def report(node):
    seen["sum"] = node.get_input("sum", int)
    return bt_kit.SUCCESS


factory = bt.BehaviorTreeFactory()
factory.register_simple_action("Compute", compute, ports={"a": int, "sum": int})
factory.register_simple_action("Report", report, ports={"sum": int})
xml = ('<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>'
       '<Compute a="7" sum="{s}"/><Report sum="{s}"/>'
       '</Sequence></BehaviorTree></root>')
status = factory.create_tree_from_text(xml).tickWhileRunning()

print("JSON:" + json.dumps({
    "include_ms": include_ms,
    "status": int(status),
    "sum": seen.get("sum"),
    "frozen": bool(freeze.active("bt")),
    "kit_frozen": bool(bt_kit.frozen()),
}), flush=True)
