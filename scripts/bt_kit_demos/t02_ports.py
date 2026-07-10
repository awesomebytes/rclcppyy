#!/usr/bin/env python
"""
BehaviorTree.CPP official tutorial 2 ("Blackboard and ports"), written in short
Python via rclcppyy's bt_kit. SaySomething reads an input port; ThinkWhatToSay
writes an output port; the blackboard entry {the_answer} passes the value from
one to the other.

Reference: https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports
Run:       pixi run -e bt demo-bt-t02
"""
from rclcppyy.kits import bt_kit

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


@bt_kit.action_node("SaySomething", ports=["message"])
def say_something(bb):
    print("Robot says:", bb.get("message"))
    return bt_kit.SUCCESS


@bt_kit.action_node("ThinkWhatToSay", ports=["text"])
def think_what_to_say(bb):
    bb.set("text", "The answer is 42")
    return bt_kit.SUCCESS


def main():
    tree = bt_kit.tree_from_xml(XML)
    bt_kit.tick_while_running(tree)


if __name__ == "__main__":
    main()
