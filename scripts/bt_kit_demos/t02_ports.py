#!/usr/bin/env python
"""
BehaviorTree.CPP official tutorial 2 ("Blackboard and ports"), in Python via
rclcppyy's bt_kit. SaySomething reads an input port; ThinkWhatToSay writes an
output port; the blackboard entry {the_answer} carries the value between them.
Mirrors the C++ tutorial: registerSimpleAction with a ports list, node.getInput
/ node.setOutput inside the callback.

Reference: https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports
Run:       pixi run -e bt demo-bt-t02
"""
from rclcppyy.kits import bt_kit

bt = bt_kit.bringup_bt()

# The tutorial XML, verbatim.
XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <SaySomething     message="hello world"/>
      <ThinkWhatToSay   text="{the_answer}"/>
      <SaySomething     message="{the_answer}"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


def say_something(node):
    print("Robot says:", node.get_input("message"))
    return bt.NodeStatus.SUCCESS


def think_what_to_say(node):
    node.set_output("text", "The answer is 42")
    return bt.NodeStatus.SUCCESS


def main():
    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleAction("SaySomething", say_something, ports=["message"])
    factory.registerSimpleAction("ThinkWhatToSay", think_what_to_say, ports=["text"])

    tree = factory.createTreeFromText(XML)
    tree.tickWhileRunning()


if __name__ == "__main__":
    main()
