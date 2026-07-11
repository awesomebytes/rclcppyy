# WHY control_kit — the "write a controller" ceremony, stock ros2_control vs ours

ros2_control is C++-only by design: there is **no Python controller API**. Writing even a
trivial controller is a multi-file, multi-tool ceremony, and running it means launching a
separate `controller_manager` process and spawning the controller into it. control_kit
collapses that to one Python class and three lines, run in your own process.

## Stock ros2_control: what a new controller costs

To add a controller you write, minimum:

1. **A C++ class** deriving `controller_interface::ControllerInterface`, implementing the
   pure virtuals `on_init`, `command_interface_configuration`, `state_interface_configuration`,
   `update`, plus the lifecycle `on_configure`/`on_activate`/`on_deactivate` — in a
   `.hpp` + `.cpp` pair, with visibility macros.
2. **A `plugin_description.xml`** declaring the class as a `pluginlib` plugin with its
   `base_class_type`.
3. **A `CMakeLists.txt`** — `ament_cmake` project, `pluginlib_export_plugin_description_file`,
   `generate_parameter_library` for the params, link `controller_interface` /
   `hardware_interface` / `rclcpp_lifecycle`, install targets.
4. **A `package.xml`** with the build/exec deps.
5. **A colcon build** of the workspace to produce the `.so` and register the plugin in the
   ament index.
6. **A YAML** with the controller's parameters and type, and a **launch file** starting
   `ros2_control_node` (or `controller_manager`) with the robot description + that YAML.
7. **A spawner** (`ros2 run controller_manager spawner my_controller`) to load, configure
   and activate it into the running manager.

Edit the control law → rebuild the workspace → relaunch → respawn. The iteration loop is
minutes, and every experiment is a C++ compile.

## control_kit: the same controller, in Python

```python
from rclcppyy.bringup_rclcpp import bringup_rclcpp
from rclcppyy.kits import control_kit as ck

bringup_rclcpp().init()
ck.bringup_control()

class MyPD(ck.ControllerInterface):                 # derive the REAL base class
    def __init__(self):
        super().__init__(); self.target = [0.5, -0.3]; self.kp = 0.4
    def on_init(self):                       return ck.CallbackReturn.SUCCESS
    def command_interface_configuration(self):
        return ck.interface_config(["joint1/position", "joint2/position"])
    def state_interface_configuration(self):
        return ck.interface_config(["joint1/position", "joint2/position"])
    def on_configure(self, prev):            return ck.CallbackReturn.SUCCESS
    def on_activate(self, prev):             return ck.CallbackReturn.SUCCESS
    def on_deactivate(self, prev):           return ck.CallbackReturn.SUCCESS
    def update(self, time, period):                  # the framework calls this each cycle
        for i in range(ck.n_command_interfaces(self)):
            cur = ck.read_state(self, i)
            ck.write_command(self, i, cur + self.kp * (self.target[i] - cur))
        return ck.return_type.OK

rig = ck.make_controller_manager(ck.mock_system_urdf(["joint1", "joint2"]))
rig.add_python_controller(MyPD(), "pd")             # inject — no plugin xml, no .so
rig.configure("pd"); rig.activate(["pd"])
rig.run(seconds=2.0, rate_hz=100)                   # the REAL read/update/write loop
```

No `plugin_description.xml`, no `CMakeLists.txt`, no `package.xml`, no colcon build, no
launch file, no spawner, no second process. Edit the control law → rerun the script. And
it is not a mock or reimplementation: `MyPD` derives the *actual*
`controller_interface::ControllerInterface`, and the *actual*
`controller_manager::ControllerManager` calls its `update()` in the real control loop,
against real mock hardware (`mock_components/GenericSystem`) parsed from a URDF string.

## Side-by-side

| | stock ros2_control | control_kit |
|---|---|---|
| controller definition | C++ `.hpp`+`.cpp` deriving `ControllerInterface` | Python class deriving the *same* `ControllerInterface` |
| plugin registration | `plugin_description.xml` + `pluginlib` export | none (injected via `add_controller`) |
| build | `CMakeLists.txt` + `package.xml` + colcon | none (cppyy JITs the glue) |
| run | launch `ros2_control_node` + YAML + spawner | `make_controller_manager()` + 3 calls, in-process |
| iterate | rebuild + relaunch + respawn (minutes) | rerun the script (seconds) |
| hardware | real / a `SystemInterface` plugin | `mock_components/GenericSystem` from a URDF string |
| the loop | CM's RT thread | `rig.run()` in your Python process (the real `read/update/write`) |

## What this is for (and what it isn't)

**For:** prototyping a control law against the real ros2_control machinery without the C++
build-and-launch ceremony — fast iteration, teaching, HIL/sim, and validating a controller
end-to-end before committing it to C++. See [REPORT.md](REPORT.md) §4 for the real-time
verdict: 100 Hz is rock-solid, 1 kHz works on average but Python's GC/GIL pauses cost the
odd deadline, so this is soft-real-time / prototyping-grade, not hard-real-time.

**Not for (yet):** shipping a controller that a *separately launched*, stock (C++)
`controller_manager` loads by name — that still needs a compiled pluginlib `.so`
(REPORT §3, Route B). The intended path is to **prototype in Python here, then lower the
validated `update()` to a native C++ plugin** (the L2 direct-compile recipe) for hard-RT
deployment — the interface contract is identical, so the port is mechanical.
