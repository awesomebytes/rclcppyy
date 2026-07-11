# CONTROL_KIT.md — API cheat sheet

`rclcppyy.kits.control_kit` — write a ros2_control controller in Python and run it inside a
real `controller_manager::ControllerManager` in-process. Requires the pixi `control` env
(`pixi run -e control ...`) and rclcpp initialized. See [REPORT.md](REPORT.md) for the
mechanics/verdict and [WHY.md](WHY.md) for the stock-ros2_control contrast.

## Bring-up

```python
from rclcppyy.bringup_rclcpp import bringup_rclcpp
from rclcppyy.kits import control_kit as ck

bringup_rclcpp().init()                 # rclcpp must be up
ns = ck.bringup_control()               # JIT-include CM/controller headers, load .so, cppdef glue
                                        # returns cppyy.gbl.controller_interface; idempotent
ck.load_message_support()               # OPTIONAL: std_msgs typesupport for a command topic
ck.warmup()                             # OPTIONAL: front-load the CM-construction JIT
```

## The mock hardware rig

```python
urdf = ck.mock_system_urdf(["joint1", "joint2"])   # mock_components/GenericSystem, 2 joints
#   mock_system_urdf(joints, command_interfaces=("position",),
#                    state_interfaces=("position","velocity"),
#                    robot_name="mock_bot", initial_value=0.0) -> URDF str
rig = ck.make_controller_manager(urdf, update_rate=100)   # real ControllerManager -> ControlRig
#   make_controller_manager(urdf, update_rate=100, node_name="controller_manager",
#                           parameters=None) -> ControlRig
```

`rig.cm` is the **real** `controller_manager::ControllerManager` — call any of its methods
directly (`is_resource_manager_initialized()`, `get_update_rate()`, `get_loaded_controllers()`,
`read`/`update`/`write`, ...).

## ControlRig methods

| method | does |
|---|---|
| `rig.load_controller(name, type, parameters=None)` | load a **stock C++** controller by pluginlib type; `parameters` (dict) set on its node. **Before the loop.** |
| `rig.add_python_controller(controller, name, type_label="python_controller")` | inject a **Python** controller instance (a `ControllerInterface` subclass). **Before the loop.** |
| `rig.set_controller_parameters(ctrl, {...})` | set params on a loaded controller's node |
| `rig.configure(name) -> bool` | configure (INACTIVE); True on OK |
| `rig.activate(names, timeout_s=5.0) -> bool` | activate (ACTIVE); off-thread switch + pumped `update()` |
| `rig.deactivate(names, timeout_s=5.0) -> bool` | deactivate (INACTIVE) |
| `rig.update(period=None)` | one `read`→`update`→`write` cycle |
| `rig.spin()` | `executor.spin_some()` — service controller-node topics (not during a switch) |
| `rig.run(seconds, rate_hz=None, spin=False, on_cycle=None) -> int` | hold the loop at `rate_hz` for `seconds` |
| `rig.add_node(node)` | add an rclcpp node (e.g. a command publisher) to the CM executor |

**Ordering rule:** load/configure/add every controller **before** the first
`update()`/`activate()`/`run()`. After the loop has run, `load_*`/`add_*` raise (the CM's
real-time-safe list swap would deadlock a synchronous load) — mirrors `ros2_control_node`.

## Writing a Python controller

Derive `ck.ControllerInterface` and override the real C++ virtuals by name:

```python
class MyController(ck.ControllerInterface):
    def __init__(self):
        super().__init__()                                   # REQUIRED
    def on_init(self):                       return ck.CallbackReturn.SUCCESS
    def command_interface_configuration(self):
        return ck.interface_config(["joint1/position"])      # what to claim
    def state_interface_configuration(self):
        return ck.interface_config(["joint1/position"])
    def on_configure(self, prev_state):      return ck.CallbackReturn.SUCCESS
    def on_activate(self, prev_state):       return ck.CallbackReturn.SUCCESS
    def on_deactivate(self, prev_state):     return ck.CallbackReturn.SUCCESS
    def update(self, time, period):                          # called every cycle
        n = ck.n_command_interfaces(self)
        for i in range(n):
            x = ck.read_state(self, i)                       # read hardware state
            ck.write_command(self, i, ...)                   # write hardware command
        return ck.return_type.OK
```

### Enums and helpers

| name | note |
|---|---|
| `ck.CallbackReturn` | `.SUCCESS` / `.FAILURE` / `.ERROR` — return from `on_init`/`on_*` |
| `ck.return_type` | `.OK` / `.ERROR` — return from `update` |
| `ck.ok(value) -> bool` | True if a returned `return_type` is OK (handles the uint8→1-char-str crossing) |
| `ck.interface_config(names, config_type="individual")` | build an `InterfaceConfiguration` (`"individual"`/`"all"`/`"none"`) |
| `ck.interface_configuration_type` | the `INDIVIDUAL`/`ALL`/`NONE` enum |

### Interface accessors (call inside `update`, after activation)

| function | returns / does |
|---|---|
| `ck.n_state_interfaces(self)` / `ck.n_command_interfaces(self)` | count of claimed interfaces |
| `ck.read_state(self, i)` | value of the i-th claimed **state** interface (float) |
| `ck.read_command(self, i)` | value in the i-th claimed **command** interface |
| `ck.write_command(self, i, value)` | write the i-th claimed **command** interface |

Interface order matches the `*_interface_configuration` names.

## Full example (self-commanding PD)

```python
bringup_rclcpp().init(); ck.bringup_control()

class PD(ck.ControllerInterface):
    def __init__(self): super().__init__(); self.target=[0.5,-0.3]; self.kp=0.4
    def on_init(self): return ck.CallbackReturn.SUCCESS
    def command_interface_configuration(self): return ck.interface_config(["j1/position","j2/position"])
    def state_interface_configuration(self):   return ck.interface_config(["j1/position","j2/position"])
    def on_configure(self,s): return ck.CallbackReturn.SUCCESS
    def on_activate(self,s):  return ck.CallbackReturn.SUCCESS
    def on_deactivate(self,s):return ck.CallbackReturn.SUCCESS
    def update(self,t,p):
        for i in range(ck.n_command_interfaces(self)):
            c = ck.read_state(self,i); ck.write_command(self,i, c+self.kp*(self.target[i]-c))
        return ck.return_type.OK

rig = ck.make_controller_manager(ck.mock_system_urdf(["j1","j2"]))
rig.add_python_controller(PD(), "pd"); rig.configure("pd"); rig.activate(["pd"])
rig.run(seconds=2.0, rate_hz=100)
# teardown is automatic (registered atexit): deactivate + drop the CM before rclcpp shutdown
```

## Tasks

```
pixi run -e control demo-control-rig      # d01: CM + mock HW + stock C++ controller (Stages 1-2)
pixi run -e control demo-control-python   # d02: the Python PD controller showcase (Stage 3)
pixi run -e control bench-control         # bench: Python vs C++ loop numbers (Stage 4)
pixi run -e control test-control          # the test suite (auto-skips without ros2_control)
```

## Gotchas (see REPORT §2)

- **Load before the loop** (above). **`super().__init__()` is required** in your controller.
- Derive `ck.ControllerInterface` **directly** (not a further Python/JIT base) or cppyy's
  override dispatcher fails.
- A returned `return_type` is a 1-char `str` — use `ck.ok(...)`, not `int(...)`.
- Clean exit is automatic via the teardown registry; don't `unload_controller` a Python
  controller yourself.
