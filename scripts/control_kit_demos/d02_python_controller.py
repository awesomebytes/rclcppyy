#!/usr/bin/env python3
"""
control_kit demo 2 (Stage 3, THE SHOWCASE): a controller written in *Python*, running
inside the real controller_manager.

``PythonPDController`` is a plain Python class that derives the real
``controller_interface::ControllerInterface`` (cross-language inheritance) and overrides
the same virtuals a C++ controller would -- ``on_init``, ``command_interface_configuration``,
``state_interface_configuration``, ``on_configure`` / ``on_activate`` / ``on_deactivate``,
and ``update``. Its ``update`` runs a per-joint PD law in Python, reading the mock robot's
state interfaces and writing its command interfaces. It is injected into a real
``ControllerManager`` and driven by the real read/update/write loop -- no plugin XML, no
CMake, no ``.so``, no spawner.

The controller tracks a moving cosine reference on two joints; the mock
``GenericSystem`` mirrors each position command back to the position state, so the
tracking error is real closed-loop behaviour through the actual ros2_control interfaces.

Contrast with stock ros2_control (see docs/control_kit/WHY.md): a controller there is a
C++ class + a pluginlib ``plugin_description.xml`` + ``CMakeLists.txt`` + a build + a
spawner. Here it is one Python class and three lines to run it.

Run:  pixi run -e control demo-control-python
"""
import math
import os

os.environ.setdefault("ROS_DOMAIN_ID", "49")

from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import control_kit as ck  # noqa: E402

JOINTS = ["shoulder", "elbow"]


class PythonPDController(ck.ControllerInterface):
    """A PD position controller, in Python, inside the real controller_manager."""

    def __init__(self):
        super().__init__()
        self.kp = 12.0
        self.kd = 0.5
        self.dt = 0.01
        self.t = 0.0
        self.prev_err = [0.0, 0.0]
        self.samples = []          # (t, [ref...], [pos...]) for the summary

    # --- the ControllerInterface virtuals, by their real C++ names ---
    def on_init(self):
        return ck.CallbackReturn.SUCCESS

    def command_interface_configuration(self):
        return ck.interface_config(["%s/position" % j for j in JOINTS])

    def state_interface_configuration(self):
        return ck.interface_config(["%s/position" % j for j in JOINTS])

    def on_configure(self, previous_state):
        return ck.CallbackReturn.SUCCESS

    def on_activate(self, previous_state):
        self.t = 0.0
        return ck.CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state):
        return ck.CallbackReturn.SUCCESS

    def reference(self, t):
        return [0.6 * math.cos(t), 0.4 * math.sin(0.7 * t)]

    def update(self, time, period):
        self.t += self.dt
        ref = self.reference(self.t)
        n = ck.n_command_interfaces(self)
        pos = []
        for i in range(n):
            p = ck.read_state(self, i)
            err = ref[i] - p
            derr = (err - self.prev_err[i]) / self.dt
            self.prev_err[i] = err
            ck.write_command(self, i, p + self.dt * (self.kp * err + self.kd * derr))
            pos.append(p)
        if len(self.samples) < 100000:
            self.samples.append((self.t, ref, pos))
        return ck.return_type.OK


def main():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    ck.bringup_control()

    rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=100)
    print("controller_manager up in-process; mock hardware active:",
          rig.cm.is_resource_manager_initialized())

    controller = PythonPDController()
    rig.add_python_controller(controller, "python_pd")
    print("injected Python controller 'python_pd' (no plugin xml / CMake / .so)")
    print("  configure:", rig.configure("python_pd"))
    print("  activate: ", rig.activate(["python_pd"]))

    seconds = 3.0
    cycles = rig.run(seconds=seconds, rate_hz=100)
    print("\nran %d update cycles (%.0f Hz for %.0fs) -- the real ros2_control loop"
          % (cycles, cycles / seconds, seconds))

    # tracking error over the second half (after the transient)
    tail = controller.samples[len(controller.samples) // 2:]
    max_err = 0.0
    for _, ref, pos in tail:
        for r, p in zip(ref, pos):
            max_err = max(max_err, abs(r - p))
    print("Python PD controller tracked the reference; max |error| (2nd half): %.4f rad"
          % max_err)
    last_t, last_ref, last_pos = controller.samples[-1]
    print("  final  ref = [%.3f, %.3f]" % tuple(last_ref))
    print("  final  pos = [%.3f, %.3f]" % tuple(last_pos))
    print("\nOK" if max_err < 0.1 else "\nHIGH tracking error (tune gains)")


if __name__ == "__main__":
    main()
