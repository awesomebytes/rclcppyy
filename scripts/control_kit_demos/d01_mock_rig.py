#!/usr/bin/env python3
"""
control_kit demo 1 (Stages 1-2): a real controller_manager in *this* Python process.

Brings up a ``controller_manager::ControllerManager`` in-process with mock hardware
(``mock_components/GenericSystem``) from a URDF string, then loads a **stock C++
controller** (``forward_command_controller/ForwardCommandController``) the same way a
real robot would -- via pluginlib, by type name -- configures and activates it, and
drives the real read/update/write loop from Python. A command published on the
controller's topic flows through the controller into the mock hardware; we read it back
off the controller's command interface to prove the path.

This is the ros2_control machinery (ResourceManager + ControllerManager + pluginlib +
the RT update loop) running under cppyy with nothing mocked but the hardware -- the
baseline the Python controller (d02) and the benchmark (bench_control_loop) build on.

Run:  pixi run -e control demo-control-rig
"""
import os

os.environ.setdefault("ROS_DOMAIN_ID", "49")

import cppyy  # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import control_kit as ck  # noqa: E402

FWD = "forward_command_controller/ForwardCommandController"


def main():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    ck.bringup_control()
    ck.load_message_support()

    # --- Stage 1: the in-process controller_manager + mock hardware ---
    urdf = ck.mock_system_urdf(["joint1", "joint2"])
    rig = ck.make_controller_manager(urdf, update_rate=100)
    print("Stage 1: ControllerManager up in-process")
    print("  resource manager initialized:", rig.cm.is_resource_manager_initialized())
    print("  update rate: %d Hz" % rig.cm.get_update_rate())

    # --- Stage 2: load + configure + activate a stock C++ controller ---
    # NB: controllers must be loaded BEFORE the update() loop starts -- once update()
    # has run, the CM switches to real-time-safe controller-list swaps that block
    # load_controller until the (now-stopped) loop pumps again. This mirrors how
    # ros2_control_node works: configure the stack, then spin the RT loop.
    rig.load_controller(
        "fwd", FWD,
        parameters={"joints": ["joint1", "joint2"], "interface_name": "position"})
    print("\nStage 2: loaded stock controller '%s'" % FWD)
    print("  configure:", rig.configure("fwd"))
    print("  activate: ", rig.activate(["fwd"]))
    fwd = rig.cm.get_loaded_controllers()[0].c
    print("  lifecycle state id (3 == active):", fwd.get_lifecycle_id())

    # --- command flow: publish a target, spin to deliver, read it back ---
    std = cppyy.gbl.std
    helper = std.make_shared["rclcpp::Node"](std.string("d01_cmd_pub"))
    pub = helper.create_publisher["std_msgs::msg::Float64MultiArray"](
        std.string("/fwd/commands"), 10)
    rig.add_node(helper)

    target = [0.5, -0.3]
    msg = cppyy.gbl.std_msgs.msg.Float64MultiArray()
    for v in target:
        msg.data.push_back(v)

    import time
    for i in range(200):
        if i % 25 == 0:
            pub.publish(msg)
        rig.update()
        rig.spin()          # deliver /fwd/commands to the controller's subscription
        time.sleep(0.001)

    cmd = [ck.read_command(fwd, i) for i in range(ck.n_command_interfaces(fwd))]
    print("\n  commanded %s on /fwd/commands" % target)
    print("  controller command interfaces now: [%.4f, %.4f]" % (cmd[0], cmd[1]))
    flowed = abs(cmd[0] - target[0]) < 1e-6 and abs(cmd[1] - target[1]) < 1e-6
    print("  command flowed topic -> controller -> mock hardware:", flowed)

    print("\nOK" if flowed else "\nUNEXPECTED command values")


if __name__ == "__main__":
    main()
