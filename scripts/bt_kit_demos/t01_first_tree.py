#!/usr/bin/env python
"""
BehaviorTree.CPP official tutorial 1 ("Your first behavior tree"), written in
short Python via rclcppyy's bt_kit. The four leaf actions run in Python; the C++
engine parses the XML and ticks the tree.

Reference: https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree
Run:       pixi run -e bt demo-bt-t01
"""
from rclcppyy.kits import bt_kit

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


@bt_kit.condition_node("CheckBattery")
def check_battery(bb):
    print("[ Battery: OK ]")
    return bt_kit.SUCCESS


@bt_kit.action_node("OpenGripper")
def open_gripper(bb):
    print("GripperInterface::open")
    return bt_kit.SUCCESS


@bt_kit.action_node("ApproachObject")
def approach_object(bb):
    print("ApproachObject: approach_object")
    return bt_kit.SUCCESS


@bt_kit.action_node("CloseGripper")
def close_gripper(bb):
    print("GripperInterface::close")
    return bt_kit.SUCCESS


def main():
    tree = bt_kit.tree_from_xml(XML)
    bt_kit.tick_while_running(tree)


if __name__ == "__main__":
    main()
