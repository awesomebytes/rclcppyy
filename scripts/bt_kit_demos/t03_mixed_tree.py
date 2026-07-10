#!/usr/bin/env python
"""
Mixed-language behavior tree: one BT.CPP tree whose leaves span the full
spectrum bt_kit enables --
  1. CheckSensors  -- a leaf written in Python (the kit as-is);
  2. ComputePlan   -- a leaf JIT-compiled as a C++ functor (bt_kit KIT.md pattern 4);
  3. PublishStatus -- a leaf that drives EXISTING C++ software: it publishes a
                      ROS 2 std_msgs/String via rclcpp (through rclcppyy) from
                      inside the tick.

This is the repo's thesis in one file: a C++ behavior-tree engine, orchestrated
from Python, calling into a real installed C++ stack (rclcpp) -- no wrapper code
generated, no build step. A subscriber in this same process counts the messages
to prove they actually flowed. Run with ROS_DOMAIN_ID set to stay isolated.

Run: pixi run -e bt demo-bt-t03
"""
import os
import sys
import time

import cppyy
from rclcppyy.bringup_rclcpp import bringup_rclcpp
from rclcppyy.kits import bt_kit

os.environ.setdefault("ROS_DOMAIN_ID", "42")
N_CYCLES = 3
XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Repeat num_cycles="3">
      <Sequence>
        <CheckSensors/>
        <ComputePlan/>
        <PublishStatus/>
      </Sequence>
    </Repeat>
  </BehaviorTree>
</root>
"""


def main():
    rclcpp = bringup_rclcpp()
    bt = bt_kit.bringup_bt()
    from std_msgs.msg import String

    if not rclcpp.ok():
        rclcpp.init()
    node = rclcpp.Node("bt_mixed_tree")
    received = []
    sub = node.create_subscription(  # noqa: F841 - keep the subscription alive
        String, "bt_status", lambda m: received.append(str(m.data)), 10)
    pub = node.create_publisher(String, "bt_status", 10)
    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Wait for the subscription to match before ticking (volatile QoS drops
    # anything published before discovery completes).
    deadline = time.monotonic() + 15.0
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)

    # Leaf 2: a C++ functor, JIT-compiled -- no Python on this tick.
    cppyy.cppdef(r"""
    namespace t03 {
      inline void registerComputePlan(BT::BehaviorTreeFactory& f) {
        f.registerSimpleAction("ComputePlan",
          [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
      }
    }
    """)

    sent = {"n": 0}

    def check_sensors(nd):                       # Leaf 1: Python
        print("  [py]  sensors nominal")
        return bt.NodeStatus.SUCCESS

    def publish_status(nd):                       # Leaf 3: existing C++ (rclcpp)
        sent["n"] += 1
        msg = String()
        msg.data = f"status {sent['n']}"
        pub.publish(msg)
        print(f"  [ros] published '{msg.data}' via rclcpp")
        return bt.NodeStatus.SUCCESS

    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleAction("CheckSensors", check_sensors)
    cppyy.gbl.t03.registerComputePlan(factory)
    factory.registerSimpleAction("PublishStatus", publish_status)

    tree = factory.createTreeFromText(XML)
    tree.tickWhileRunning()                        # runs the sequence N_CYCLES times

    # Drain the messages the ticks published, bounded.
    deadline = time.monotonic() + 10.0
    while len(received) < N_CYCLES and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.01)

    print(f"Subscriber received {len(received)}/{N_CYCLES} ROS messages: {received}")
    ok = received == [f"status {i + 1}" for i in range(N_CYCLES)]
    print("RESULT:", "OK" if ok else "FAIL")
    sys.stdout.flush()
    # rclcpp/cppyy static destructors can segfault at shutdown; work is done, so
    # exit hard for a deterministic return code (matches the repo's test helpers).
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()
