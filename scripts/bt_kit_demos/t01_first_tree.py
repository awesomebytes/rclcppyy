#!/usr/bin/env python
"""
BehaviorTree.CPP official tutorial 1 ("Your first behavior tree"), in Python via
rclcppyy's bt_kit. This mirrors the C++ tutorial line-for-line -- same factory,
same registerSimpleAction / registerSimpleCondition, same createTreeFromText /
tickWhileRunning -- only the leaf callbacks are Python.

Reference: https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree
Run:       pixi run -e bt demo-bt-t01
"""
from rclcppyy.kits import bt_kit

bt = bt_kit.bringup_bt()

# The tutorial XML, verbatim.
XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <CheckBattery   name="check_battery"/>
      <OpenGripper    name="open_gripper"/>
      <ApproachObject name="approach_object"/>
      <CloseGripper   name="close_gripper"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


def check_battery(node):
    print("[ Battery: OK ]")
    return bt.NodeStatus.SUCCESS


def open_gripper(node):
    print("GripperInterface::open")
    return bt.NodeStatus.SUCCESS


def approach_object(node):
    print("ApproachObject: approach_object")
    return bt.NodeStatus.SUCCESS


def close_gripper(node):
    print("GripperInterface::close")
    return bt.NodeStatus.SUCCESS


def main():
    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleCondition("CheckBattery", check_battery)
    factory.registerSimpleAction("OpenGripper", open_gripper)
    factory.registerSimpleAction("ApproachObject", approach_object)
    factory.registerSimpleAction("CloseGripper", close_gripper)

    tree = factory.createTreeFromText(XML)
    tree.tickWhileRunning()


if __name__ == "__main__":
    main()
