"""
control_kit -- write a ros2_control controller in Python and run it *inside the real
controller_manager* via cppyy.

ros2_control has no Python controller story: a controller is a C++ class deriving
``controller_interface::ControllerInterface``, exported through a pluginlib
``plugin_description.xml``, built by CMake/ament into a ``.so``, and spawned into a
``controller_manager`` process. This kit collapses that ceremony: you write a plain
Python class deriving MoveIt-style the *real* ``ControllerInterface`` (cross-language
inheritance, ompl_kit's headline pattern), and it is injected into a **real
``controller_manager::ControllerManager``** running in *your* Python process against
mock hardware from a URDF string -- the same C++ update loop
(``read`` -> ``update`` -> ``write``) that drives a physical robot, driven from Python.

Unlike MoveIt (docs/moveit_kit/REPORT.md §2.1), **ros2_control's headers do NOT hit the
generate_parameter_library Cling wall**: ``controller_manager/controller_manager.hpp``
and ``controller_interface/controller_interface.hpp`` JIT-parse cleanly, so the full CM
is reachable. The frictions this kit hides are different and specific (see REPORT):

  * **Cross-inheritance + injection.** A Python class derives the *compiled*
    ``ControllerInterface`` (deriving a JIT'd intermediate base breaks cppyy's override
    dispatcher -- the ``CallbackReturn`` return type resolves to ``<unknown>``). Its
    instance is injected via ``ControllerManager::add_controller(ControllerSpec)``, with
    the spec assembled in C++ and the controller wrapped in a ``shared_ptr`` with a
    **no-op deleter** (Python owns the object; the CM must not free it).
  * **Protected interfaces.** A controller reads/writes hardware through the *protected*
    ``state_interfaces_`` / ``command_interfaces_`` members, invisible to a Python
    subclass. The kit reaches them through a same-layout accessor
    (``ControllerInterfaceBase`` is the offset-0 base of every controller), exposed as
    ``read_state`` / ``write_command`` free functions taking the controller.
  * **Blocking activation.** ``switch_controller`` blocks until the ``update()`` loop
    applies the switch, so it must run off the loop thread. ``std::async`` does not JIT
    in Cling; a plain-function ``std::thread`` does -- the kit runs the switch there and
    pumps ``update()`` until it completes.
  * **uint8_t enums.** ``return_type`` is ``uint8_t``-backed: a *returned* value crosses
    as a 1-char ``str`` (``'\\x00'`` == OK); ``ok()`` reads it with ``ord``.
  * **Teardown.** The CM owns a pal_statistics async publisher thread; if it outlives the
    rclcpp context the process cores at exit. The rig deactivates controllers and resets
    the CM (before rclcpp shutdown) via ``cppyy_kit.register_teardown``.

Minimal Python controller (mirrors the C++ tutorial's names 1:1)::

    from rclcppyy.kits import control_kit as ck
    ck.bringup_control()

    class MyPD(ck.ControllerInterface):
        def __init__(self):
            super().__init__()
            self.target = [0.5, -0.3]; self.kp = 0.4
        def on_init(self):
            return ck.CallbackReturn.SUCCESS
        def command_interface_configuration(self):
            return ck.interface_config(["joint1/position", "joint2/position"])
        def state_interface_configuration(self):
            return ck.interface_config(["joint1/position", "joint2/position"])
        def on_configure(self, prev):   return ck.CallbackReturn.SUCCESS
        def on_activate(self, prev):     return ck.CallbackReturn.SUCCESS
        def on_deactivate(self, prev):   return ck.CallbackReturn.SUCCESS
        def update(self, time, period):
            for i in range(ck.n_command_interfaces(self)):
                cur = ck.read_state(self, i)
                ck.write_command(self, i, cur + self.kp * (self.target[i] - cur))
            return ck.return_type.OK

    rig = ck.make_controller_manager(ck.mock_system_urdf(["joint1", "joint2"]))
    pd = MyPD(); rig.add_python_controller(pd, "pd")
    rig.configure("pd"); rig.activate(["pd"])
    rig.run(seconds=1.0, rate_hz=100)     # the real read/update/write loop

See docs/control_kit/REPORT.md for the capability matrix, the Route-A vs Route-B
analysis, and the honest real-time verdict; CONTROL_KIT.md for the API cheat sheet.
"""
import os
import time

import cppyy

from rclcppyy.kits import cppyy_kit

_HEADERS = (
    "controller_manager/controller_manager.hpp",
    "controller_interface/controller_interface.hpp",
    "rclcpp/executors.hpp",
)
# cppyy resolves a symbol's owning .so at call time; load the CM stack + pluginlib
# engine. Do NOT load the controller/hardware plugin .so -- pluginlib dlopen's those.
_LIBS = (
    "libclass_loader.so",
    "librealtime_tools.so",
    "libhardware_interface.so",
    "libcontroller_interface.so",
    "libcontroller_manager.so",
)

# Message typesupport for optional topic I/O (a controller command topic). Loaded
# lazily by load_message_support(); not needed for a self-commanding controller.
_MSG_LIBS = ("libstd_msgs__rosidl_typesupport_cpp.so",)

# The C++ glue: an executor factory (make_shared of the executor is flaky from Python --
# cppyy overload-cache sensitivity -- so build it in C++), the threaded switch runner,
# the same-layout interface accessors, and the Python-controller injector.
_GLUE = r"""
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include "rclcpp/executors.hpp"

namespace control_kit_cpp {

std::shared_ptr<rclcpp::Executor> make_single_threaded_executor() {
  return std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

// --- interface access -----------------------------------------------------
// A Python class can derive the *compiled* ControllerInterface and override its
// virtuals, but cannot see the protected state_interfaces_/command_interfaces_.
// ControllerInterfaceBase is the offset-0 base of every controller, so an accessor
// deriving it reads those members at their true offsets; free functions take the
// controller as a base pointer (Python passes `self`, which cppyy upcasts).
struct IfaceAccessor : public controller_interface::ControllerInterfaceBase {
  double rd_state(size_t i) { return state_interfaces_[i].get_value(); }
  double rd_command(size_t i) { return command_interfaces_[i].get_value(); }
  void wr_command(size_t i, double v) { (void)command_interfaces_[i].set_value(v); }
  size_t n_state() const { return state_interfaces_.size(); }
  size_t n_command() const { return command_interfaces_.size(); }
};
double read_state(controller_interface::ControllerInterfaceBase* c, size_t i) {
  return reinterpret_cast<IfaceAccessor*>(c)->rd_state(i);
}
double read_command(controller_interface::ControllerInterfaceBase* c, size_t i) {
  return reinterpret_cast<IfaceAccessor*>(c)->rd_command(i);
}
void write_command(controller_interface::ControllerInterfaceBase* c, size_t i, double v) {
  reinterpret_cast<IfaceAccessor*>(c)->wr_command(i, v);
}
size_t n_state(controller_interface::ControllerInterfaceBase* c) {
  return reinterpret_cast<IfaceAccessor*>(c)->n_state();
}
size_t n_command(controller_interface::ControllerInterfaceBase* c) {
  return reinterpret_cast<IfaceAccessor*>(c)->n_command();
}

// --- Python-controller injection ------------------------------------------
// Wrap the raw base pointer in a shared_ptr with a NO-OP deleter (Python owns the
// object; the CM must not delete it), assemble the ControllerSpec in C++ (assigning
// spec.c from Python fails on a cross-inherited object), and add_controller it.
controller_interface::ControllerInterfaceBaseSharedPtr add_py_controller(
    std::shared_ptr<controller_manager::ControllerManager> cm,
    controller_interface::ControllerInterfaceBase* raw,
    const std::string & name, const std::string & type) {
  auto sp = std::shared_ptr<controller_interface::ControllerInterfaceBase>(
      raw, [](controller_interface::ControllerInterfaceBase*){});
  controller_manager::ControllerSpec spec;
  spec.info.name = name;
  spec.info.type = type;
  spec.c = sp;
  return cm->add_controller(spec);
}

// --- threaded switch ------------------------------------------------------
// switch_controller() blocks until the update() loop applies the switch, so it runs
// on a worker thread (std::async will not JIT in Cling; a plain-function std::thread
// does) while the caller pumps update(). One request in flight at a time.
struct SwitchReq {
  std::shared_ptr<controller_manager::ControllerManager> cm;
  std::vector<std::string> activate, deactivate;
  int strictness = 2;
  double timeout_s = 5.0;
  std::atomic<bool> done{false};
  std::atomic<int> result{-1};
};
static SwitchReq g_switch;
void switch_worker() {
  auto r = g_switch.cm->switch_controller(
      g_switch.activate, g_switch.deactivate, g_switch.strictness, true,
      rclcpp::Duration::from_seconds(g_switch.timeout_s));
  g_switch.result.store(static_cast<int>(r));
  g_switch.done.store(true);
}
void request_switch(std::shared_ptr<controller_manager::ControllerManager> cm,
                    std::vector<std::string> activate,
                    std::vector<std::string> deactivate,
                    int strictness, double timeout_s) {
  g_switch.cm = cm; g_switch.activate = activate; g_switch.deactivate = deactivate;
  g_switch.strictness = strictness; g_switch.timeout_s = timeout_s;
  g_switch.done.store(false); g_switch.result.store(-1);
  std::thread(switch_worker).detach();
}
bool switch_ready() { return g_switch.done.load(); }
int switch_result() { return g_switch.result.load(); }
void release_switch() { g_switch.cm.reset(); }

}  // namespace control_kit_cpp
"""

# strictness for switch_controller (controller_manager_msgs/SwitchController)
BEST_EFFORT = 1
STRICT = 2

_BROUGHT_UP = False
_MSG_SUPPORT = False
_CI = None       # cppyy.gbl.controller_interface
_CK = None       # cppyy.gbl.control_kit_cpp
_LIFECYCLE_STATE = {"unknown": 0, "unconfigured": 1, "inactive": 2, "active": 3}


def _lib_dir():
    return os.path.join(os.environ["CONDA_PREFIX"], "lib")


def bringup_control():
    """
    Bring up ros2_control under cppyy and return the ``controller_interface`` namespace.
    Idempotent.

    JIT-includes ``controller_manager.hpp`` / ``controller_interface.hpp`` (both parse
    cleanly in Cling -- ros2_control does *not* hit the generate_parameter_library wall),
    loads the CM ``.so`` stack + pluginlib engine, and defines the C++ glue (executor
    factory, threaded switch, interface accessors, Python-controller injector).

    Returns ``cppyy.gbl.controller_interface`` -- derive ``.ControllerInterface`` for a
    Python controller. The sibling ``cppyy.gbl.controller_manager`` namespace and this
    module's helpers (``mock_system_urdf``, ``make_controller_manager``, the ``read_state``
    / ``write_command`` accessors) are the rest of the surface.
    """
    global _BROUGHT_UP, _CI, _CK
    if _BROUGHT_UP:
        return _CI
    from rclcppyy.bringup_rclcpp import add_ros2_include_paths
    add_ros2_include_paths()
    cppyy.add_include_path(os.path.join(os.environ["CONDA_PREFIX"], "include", "eigen3"))
    cppyy.add_library_path(_lib_dir())
    for header in _HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_LIBS, [_lib_dir()])
    cppyy.cppdef(_GLUE)
    _CI = cppyy.gbl.controller_interface
    _CK = cppyy.gbl.control_kit_cpp
    _BROUGHT_UP = True
    return _CI


def load_message_support():
    """Load ``std_msgs`` typesupport so cppyy can create a publisher/subscription for
    ``std_msgs::msg::Float64MultiArray`` -- the command topic a stock forward/position
    controller subscribes to. Only needed for topic-driven controllers; a
    self-commanding controller (a PD law in ``update``) needs none of it. Idempotent.
    (sensor_msgs is intentionally NOT pulled: its headers JIT-parse very slowly and no
    kit path needs them.)"""
    global _MSG_SUPPORT
    if _MSG_SUPPORT:
        return
    bringup_control()
    cppyy.include("std_msgs/msg/float64_multi_array.hpp")
    cppyy_kit.load_libraries(_MSG_LIBS, [_lib_dir()])
    _MSG_SUPPORT = True


# --- namespace conveniences (populated on bringup; mirror the real names) ---
def _ns():
    if not _BROUGHT_UP:
        bringup_control()


class _Lazy:
    """Attribute proxy that resolves a dotted cppyy path on first access, bringing the
    kit up if needed -- lets ``control_kit.ControllerInterface`` work at import time."""

    def __init__(self, path):
        self._path = path

    def _resolve(self):
        _ns()
        obj = cppyy.gbl
        for part in self._path.split("."):
            obj = getattr(obj, part)
        return obj

    def __getattr__(self, name):
        return getattr(self._resolve(), name)

    def __call__(self, *a, **k):
        return self._resolve()(*a, **k)

    def __mro_entries__(self, bases):
        # Lets `class MyController(control_kit.ControllerInterface)` resolve to the real
        # cppyy base at class-creation time (brings the kit up if needed), so a Python
        # controller can be written against the module attribute.
        return (self._resolve(),)


ControllerInterface = _Lazy("controller_interface.ControllerInterface")
InterfaceConfiguration = _Lazy("controller_interface.InterfaceConfiguration")
interface_configuration_type = _Lazy("controller_interface.interface_configuration_type")
return_type = _Lazy("controller_interface.return_type")
CallbackReturn = _Lazy(
    "rclcpp_lifecycle.node_interfaces.LifecycleNodeInterface.CallbackReturn")


def ok(return_value):
    """True if a controller/CM ``return_type`` value is OK. A ``uint8_t``-backed enum
    *value* returned from C++ crosses as a 1-char ``str`` (``'\\x00'`` == OK == 0), so
    read it with ``ord`` (the enum *member* ``return_type.OK`` is a proxy that ints
    directly -- this handles both)."""
    if isinstance(return_value, str):
        return ord(return_value) == 0
    return int(return_value) == 0


def interface_config(names, config_type="individual"):
    """Build a ``controller_interface::InterfaceConfiguration`` from a list of interface
    names (e.g. ``["joint1/position", "joint2/position"]``). ``config_type`` is
    ``"individual"`` (default), ``"all"``, or ``"none"``. Use in a controller's
    ``command_interface_configuration`` / ``state_interface_configuration`` override."""
    bringup_control()
    cfg = _CI.InterfaceConfiguration()
    ct = _CI.interface_configuration_type
    cfg.type = {"individual": ct.INDIVIDUAL, "all": ct.ALL, "none": ct.NONE}[config_type]
    if config_type == "individual":
        cfg.names = cppyy.gbl.std.vector["std::string"](
            [cppyy.gbl.std.string(n) for n in names])
    return cfg


# --- interface accessors for use inside a Python controller's update() ------
def n_state_interfaces(controller):
    """Number of state interfaces claimed by ``controller`` (call after activation)."""
    return int(_CK.n_state(controller))


def n_command_interfaces(controller):
    """Number of command interfaces claimed by ``controller`` (call after activation)."""
    return int(_CK.n_command(controller))


def read_state(controller, index):
    """Value of ``controller``'s ``index``-th claimed **state** interface (a double).
    Order matches the ``state_interface_configuration`` names."""
    return float(_CK.read_state(controller, index))


def read_command(controller, index):
    """Value currently held in ``controller``'s ``index``-th **command** interface."""
    return float(_CK.read_command(controller, index))


def write_command(controller, index, value):
    """Write ``value`` to ``controller``'s ``index``-th claimed **command** interface --
    what a controller does each ``update()`` to drive the hardware."""
    _CK.write_command(controller, index, float(value))


def mock_system_urdf(joints, command_interfaces=("position",),
                     state_interfaces=("position", "velocity"),
                     robot_name="mock_bot", initial_value=0.0):
    """
    A minimal URDF string for a headless ``mock_components/GenericSystem`` rig -- the
    standard ros2_control mock hardware. One revolute joint per name in ``joints``, each
    with the given command/state interfaces. GenericSystem mirrors a position command
    back to the position state, so a position controller's effect is observable by
    reading the state interface next cycle.

    This is the ``<ros2_control>`` tag the ResourceManager parses; the link/joint tree
    is minimal boilerplate to keep urdf/srdf parsing happy.
    """
    links = ['  <link name="base_link"/>']
    joint_xml = []
    prev = "base_link"
    for idx, jname in enumerate(joints):
        child = "link_%d" % idx
        joint_xml.append(
            '  <joint name="%s" type="revolute">\n'
            '    <parent link="%s"/><child link="%s"/>\n'
            '    <origin xyz="0 0 %d"/><axis xyz="0 0 1"/>\n'
            '    <limit lower="-3.14" upper="3.14" effort="100" velocity="100"/>\n'
            '  </joint>' % (jname, prev, child, idx))
        links.append('  <link name="%s"/>' % child)
        prev = child
    r2c_joints = []
    for jname in joints:
        cmd = "".join('      <command_interface name="%s"/>\n' % c
                      for c in command_interfaces)
        sta = ""
        for s in state_interfaces:
            if s == "position":
                sta += ('      <state_interface name="position">'
                        '<param name="initial_value">%s</param></state_interface>\n'
                        % initial_value)
            else:
                sta += '      <state_interface name="%s"/>\n' % s
        r2c_joints.append(
            '    <joint name="%s">\n%s%s    </joint>' % (jname, cmd, sta))
    return (
        '<?xml version="1.0"?>\n<robot name="%s">\n%s\n%s\n'
        '  <ros2_control name="MockSystem" type="system">\n'
        '    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>\n'
        '%s\n  </ros2_control>\n</robot>\n'
        % (robot_name, "\n".join(links), "\n".join(joint_xml), "\n".join(r2c_joints)))


class ControlRig:
    """
    A running ``controller_manager::ControllerManager`` in this process, plus the cppyy
    friction helpers around it. ``rig.cm`` is the *real* CM -- call any of its methods
    directly (``load_controller``, ``get_update_rate``, ``read``/``update``/``write``);
    the rig's methods wrap the operations that need special handling under cppyy
    (parameterized construction, off-thread activation, ordered teardown).

    Build one with ``make_controller_manager(urdf)``. The rig registers an ordered
    teardown (deactivate controllers, drop the CM before rclcpp shutdown) so the process
    exits cleanly despite the CM's pal_statistics async publisher thread.
    """

    def __init__(self, cm, executor, update_rate):
        self.cm = cm
        self.executor = executor
        self.update_rate = update_rate
        self.clock = cm.get_trigger_clock()
        self._period = cppyy.gbl.rclcpp.Duration.from_seconds(1.0 / update_rate)
        self._kept = []           # pin Python controllers against C++
        self._active = []         # names activated through the rig
        self._updated = False     # has the RT loop started? (load must precede it)
        self._torn_down = False
        cppyy_kit.register_teardown(self._teardown)

    def _guard_before_loop(self, what):
        # Once update() has run, the CM manages its controller list with real-time-safe
        # swaps that block a synchronous load_controller/add_controller until the loop
        # pumps again -- which deadlocks a load issued from the (stopped) loop thread.
        # ros2_control_node loads the whole stack before spinning the RT loop; so must we.
        if self._updated:
            raise cppyy_kit.CppyyKitError(
                "control_kit: %s must be called BEFORE the update() loop starts (before "
                "activate()/update()/run()). Load and configure every controller first, "
                "then activate and run." % what)

    # -- controller setup --
    def load_controller(self, name, controller_type, parameters=None):
        """Load a stock (C++) controller by pluginlib type (e.g.
        ``"forward_command_controller/ForwardCommandController"``); pluginlib dlopen's
        its ``.so``. ``parameters`` (a dict) are set on the controller's own node --
        Jazzy controller nodes do NOT auto-declare from the CM's overrides, and a
        controller's ParamListener declares its params (empty) in ``on_init``, so these
        are ``set`` after load, before ``configure``. Returns the controller handle.

        Must be called before the update() loop starts (see ``_guard_before_loop``)."""
        self._guard_before_loop("load_controller")
        ctrl = self.cm.load_controller(cppyy.gbl.std.string(name),
                                       cppyy.gbl.std.string(controller_type))
        if parameters:
            self.set_controller_parameters(ctrl, parameters)
        return ctrl

    def add_python_controller(self, controller, name, type_label="python_controller"):
        """Inject a Python controller instance (a subclass of
        ``control_kit.ControllerInterface``) into the CM via ``add_controller`` -- no
        pluginlib, no ``.so``. The instance is pinned alive for the rig's lifetime (C++
        holds it through a no-op-deleter ``shared_ptr``; cppyy would otherwise collect
        it). Returns the controller handle. Must be called before the update() loop
        starts (see ``_guard_before_loop``)."""
        self._guard_before_loop("add_python_controller")
        bringup_control()
        added = _CK.add_py_controller(self.cm, controller, cppyy.gbl.std.string(name),
                                      cppyy.gbl.std.string(type_label))
        self._kept.append(controller)
        return added

    def set_controller_parameters(self, controller, parameters):
        """Set parameters on a loaded controller's own lifecycle node (a dict of
        name -> scalar / homogeneous list). The controller's ParamListener already
        declared them (empty) in ``on_init``, so this ``set``s them before configure."""
        node = controller.get_node()
        Param = cppyy.gbl.rclcpp.Parameter
        for pname, value in parameters.items():
            node.set_parameter(Param(cppyy.gbl.std.string(pname), _parameter_value(value)))

    # -- lifecycle --
    def configure(self, name):
        """Configure a loaded controller (calls its ``on_configure``); returns True on
        OK. INACTIVE afterwards."""
        return ok(self.cm.configure_controller(cppyy.gbl.std.string(name)))

    def activate(self, names, timeout_s=5.0):
        """Activate controllers by name (INACTIVE -> ACTIVE). ``switch_controller``
        blocks until the ``update()`` loop applies the switch, so the kit runs it on a
        worker thread and pumps ``update()`` here until it completes. Returns True on OK.
        """
        return self._switch(names, [], timeout_s)

    def deactivate(self, names, timeout_s=5.0):
        """Deactivate controllers by name (ACTIVE -> INACTIVE). Same off-thread-switch
        mechanic as ``activate``."""
        return self._switch([], names, timeout_s)

    def _switch(self, activate, deactivate, timeout_s):
        bringup_control()
        vec = cppyy.gbl.std.vector["std::string"]
        act = vec([cppyy.gbl.std.string(n) for n in activate])
        deact = vec([cppyy.gbl.std.string(n) for n in deactivate])
        _CK.request_switch(self.cm, act, deact, STRICT, float(timeout_s))
        deadline = time.monotonic() + timeout_s + 1.0
        # Pump only read/update/write here: the worker thread mutates the executor's
        # node set during the switch, so spinning the executor concurrently races it.
        while not _CK.switch_ready() and time.monotonic() < deadline:
            self.update()
            time.sleep(0.001)
        result_ok = _CK.switch_ready() and _CK.switch_result() == 0
        if result_ok:
            for n in activate:
                if n not in self._active:
                    self._active.append(n)
            for n in deactivate:
                if n in self._active:
                    self._active.remove(n)
        return result_ok

    # -- the real-time loop --
    def update(self, period=None):
        """One control cycle: ``cm.read(t, period)`` -> ``cm.update(t, period)`` (calls
        every active controller's ``update``) -> ``cm.write(t, period)``. ``t`` is the
        CM's trigger clock. This is the real ros2_control update loop, one iteration."""
        self._updated = True
        t = self.clock.now()
        p = self._period if period is None else period
        self.cm.read(t, p)
        self.cm.update(t, p)
        self.cm.write(t, p)

    def spin(self):
        """``executor.spin_some()`` -- services the controller nodes' subscriptions /
        services (e.g. a command topic). Call between ``update()``s in the command loop,
        NOT during a switch (which mutates the executor's node set)."""
        self.executor.spin_some()

    def run(self, seconds, rate_hz=None, spin=False, on_cycle=None):
        """Run the update loop for ``seconds`` at ``rate_hz`` (default: the CM update
        rate), sleeping to hold the rate. ``spin=True`` also services controller topics
        each cycle. ``on_cycle(i)`` is called after each cycle. Returns the number of
        cycles run."""
        rate = rate_hz or self.update_rate
        dt = 1.0 / rate
        period = cppyy.gbl.rclcpp.Duration.from_seconds(dt)
        n = int(seconds * rate)
        start = time.monotonic()
        for i in range(n):
            self.update(period)
            if spin:
                self.executor.spin_some()
            if on_cycle is not None:
                on_cycle(i)
            target = start + (i + 1) * dt
            sleep = target - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
        return n

    def add_node(self, node):
        """Add an rclcpp node (e.g. a helper publisher/subscriber node) to the CM's
        executor so ``spin()`` services it."""
        self.executor.add_node(node)

    def _teardown(self):
        if self._torn_down:
            return
        self._torn_down = True
        try:
            if self._active:
                self.deactivate(list(self._active), timeout_s=2.0)
        except Exception:
            pass
        # Drop the CM (its destructor deactivates/cleans up controllers and joins the
        # pal_statistics async thread) while the rclcpp context is still valid. Reset the
        # C++ static ref first, then the Python refs.
        try:
            _CK.release_switch()
        except Exception:
            pass
        self.cm = None
        self.executor = None
        self.clock = None
        self._kept = []


def make_controller_manager(urdf, update_rate=100, node_name="controller_manager",
                            parameters=None):
    """
    Construct a real ``controller_manager::ControllerManager`` in this process from a
    URDF string (mock or real hardware) and return a :class:`ControlRig`.

    Uses the CM's URDF-taking constructor, which builds the ``ResourceManager`` and
    activates the hardware components internally. ``update_rate`` (Hz) is set as the CM's
    ``update_rate`` parameter; extra ``parameters`` (a dict) are added as CM node
    parameter overrides. A ``SingleThreadedExecutor`` is built in C++ (cppyy's
    ``make_shared`` of it is flaky) and owned by the CM. Requires rclcpp initialized
    (``bringup_rclcpp(); rclcpp.init()``).
    """
    bringup_control()
    std = cppyy.gbl.std
    opts = cppyy.gbl.controller_manager.get_cm_node_options()
    overrides = std.vector["rclcpp::Parameter"]()
    Param = cppyy.gbl.rclcpp.Parameter
    overrides.push_back(Param(std.string("update_rate"),
                              cppyy.gbl.rclcpp.ParameterValue(int(update_rate))))
    for pname, value in (parameters or {}).items():
        overrides.push_back(Param(std.string(pname), _parameter_value(value)))
    opts.parameter_overrides(overrides)
    executor = _CK.make_single_threaded_executor()
    with cppyy_kit.first_use("control_kit.make_controller_manager",
                             "control_kit.warmup()"):
        cm = std.make_shared["controller_manager::ControllerManager"](
            executor, std.string(urdf), True, std.string(node_name), std.string(""), opts)
    return ControlRig(cm, executor, update_rate)


def _parameter_value(value):
    """A Python scalar / homogeneous list -> ``rclcpp::ParameterValue`` (bool / int /
    float / str and lists thereof) -- the shapes controller params take."""
    pv = cppyy.gbl.rclcpp.ParameterValue
    std = cppyy.gbl.std
    if isinstance(value, bool):
        return pv(value)
    if isinstance(value, int):
        return pv(int(value))
    if isinstance(value, float):
        return pv(float(value))
    if isinstance(value, str):
        return pv(std.string(value))
    if isinstance(value, (list, tuple)):
        if all(isinstance(v, str) for v in value):
            return pv(std.vector["std::string"]([std.string(v) for v in value]))
        if all(isinstance(v, bool) for v in value):
            return pv(std.vector["bool"](list(value)))
        if all(isinstance(v, int) for v in value):
            return pv(std.vector["int64_t"](list(value)))
        return pv(std.vector["double"]([float(v) for v in value]))
    raise cppyy_kit.CppyyKitError(
        "control_kit: unsupported parameter value %r" % (value,))


def warmup():
    """Front-load control_kit's one-time first-use JIT (the CM constructor call wrapper)
    during init, so the first live ``make_controller_manager`` does not stall. Builds a
    throwaway 1-joint mock CM and tears it down. Requires rclcpp initialized."""
    bringup_control()

    def _thunk():
        rig = make_controller_manager(mock_system_urdf(["warmup_joint"]),
                                      node_name="control_kit_warmup")
        rig._teardown()

    cppyy_kit.warmup(_thunk)
