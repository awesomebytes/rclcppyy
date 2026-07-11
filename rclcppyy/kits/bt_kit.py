"""
bt_kit -- drive BehaviorTree.CPP v4 from Python via cppyy.

BehaviorTree.CPP has no official Python binding. This kit is a thin cppyy glue
layer that **mirrors the C++ API**: you use `BehaviorTreeFactory`,
`registerSimpleAction`, `createTreeFromText`, `tickWhileRunning` -- the same
names and shapes as the official C++ tutorials -- and write the leaf callbacks in
Python. The kit's only job is to remove the cppyy friction (bringing the library
up, wrapping Python callables in `std::function`, keeping them alive, building
port lists, and unwrapping `getInput`/`Expected<T>`), so code you already know
from the C++ docs transfers almost verbatim.

Tutorial 1, in Python::

    from rclcppyy.kits import bt_kit
    bt = bt_kit.bringup_bt()

    def approach_object(node):
        print("ApproachObject: approach_object")
        return bt.NodeStatus.SUCCESS          # or bt_kit.SUCCESS

    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleAction("ApproachObject", approach_object)
    tree = factory.createTreeFromText(xml_text_or_path)
    tree.tickWhileRunning()

Ports (tutorial 2)::

    def say(node):
        print(node.get_input("message"))      # str; C++: node.getInput<std::string>("message")
        return bt_kit.SUCCESS
    factory.registerSimpleAction("SaySomething", say, ports=["message"])

Ports may be string (the default) or typed: pass a dict to `ports`, e.g.
`ports={"count": int, "ratio": float, "items": [float]}`, then read/write with the
matching cast: `node.get_input("count", int)` / `node.set_output("ratio", 0.5)`.
Supported: int, float (double), bool, str, and lists of those (vector ports).

Notes / limits:
    * Ports are bidirectional. Directioned (input-only/output-only) declarations
      and arbitrary struct/JSON port types are not modelled (see REPORT.md gaps).
    * Leaf logic runs in Python (holds the GIL); use it for orchestration, not
      hot inner loops. For fast leaves, register a JIT'd C++ functor instead.
"""
import os
import warnings

import cppyy

from rclcppyy.kits import cppyy_kit, freeze

_MISSING = object()

# NodeStatus enum values. Exposed as plain ints for convenience, so user code can
# `return bt_kit.SUCCESS`. `bt.NodeStatus.SUCCESS` (the real enum, mirroring C++)
# works identically -- the two compare equal.
IDLE = 0
RUNNING = 1
SUCCESS = 2
FAILURE = 3
SKIPPED = 4

_BT = None
_STATUS = {}
_BRINGUP_DONE = False


class BtXmlError(ValueError):
    """Raised for malformed XML or an unregistered node ID, with just the
    BehaviorTree.CPP message (not the cppyy C++ signature wrapper)."""

# C++ glue compiled once at bringup. makePorts keeps the (segfault-prone in
# cppyy) unordered_map<string, PortInfo> construction on the C++ side.
# PyStatefulShim exposes the pure-virtual StatefulActionNode hooks as
# std::function slots -- Python cannot subclass StatefulActionNode directly
# (its tick()/halt() are `final`, which cppyy's override dispatcher cannot
# regenerate), so asynchronous nodes route through this shim.
_CPP_GLUE = r"""
namespace rclcppyy_btkit {

// Add one bidirectional port of the requested type. Dispatching on a string tag
// keeps the (segfault-prone in cppyy) PortsList/map construction on the C++ side
// while still letting Python choose the port type. Types beyond these fall back
// to a std::string port (BT's convertFromString still parses the literal).
inline void addPort(BT::PortsList& p, const std::string& name, const std::string& t) {
  if      (t == "int")                      p.insert(BT::BidirectionalPort<int>(name));
  else if (t == "double")                   p.insert(BT::BidirectionalPort<double>(name));
  else if (t == "bool")                     p.insert(BT::BidirectionalPort<bool>(name));
  else if (t == "std::vector<int>")         p.insert(BT::BidirectionalPort<std::vector<int>>(name));
  else if (t == "std::vector<double>")      p.insert(BT::BidirectionalPort<std::vector<double>>(name));
  else if (t == "std::vector<bool>")        p.insert(BT::BidirectionalPort<std::vector<bool>>(name));
  else if (t == "std::vector<std::string>") p.insert(BT::BidirectionalPort<std::vector<std::string>>(name));
  else                                      p.insert(BT::BidirectionalPort<std::string>(name));
}

// Two parallel std::vector<std::string> (names, types) -- passing a
// vector<pair> or building the map from Python is what segfaults.
inline BT::PortsList makePorts(const std::vector<std::string>& names,
                               const std::vector<std::string>& types) {
  BT::PortsList ports;
  for (size_t i = 0; i < names.size(); ++i) {
    addPort(ports, names[i], types[i]);
  }
  return ports;
}

// Each tree-node instance gets its own Python object: when the builder runs
// (once per node in the XML), it calls back into Python to create a fresh
// instance and returns an integer handle; the shim carries that handle and the
// onStart/onRunning/onHalted hooks dispatch on it. This gives per-instance state
// rather than one shared Python object per registered ID.
class PyStatefulShim : public BT::StatefulActionNode {
public:
  int handle;
  std::function<int(int, BT::TreeNode&)> f_start, f_running;
  std::function<void(int, BT::TreeNode&)> f_halted;
  PyStatefulShim(const std::string& name, const BT::NodeConfig& cfg, int h,
                 std::function<int(int, BT::TreeNode&)> fs,
                 std::function<int(int, BT::TreeNode&)> fr,
                 std::function<void(int, BT::TreeNode&)> fh)
    : BT::StatefulActionNode(name, cfg), handle(h),
      f_start(fs), f_running(fr), f_halted(fh) {}
  BT::NodeStatus onStart() override { return static_cast<BT::NodeStatus>(f_start(handle, *this)); }
  BT::NodeStatus onRunning() override { return static_cast<BT::NodeStatus>(f_running(handle, *this)); }
  void onHalted() override { if (f_halted) f_halted(handle, *this); }
};

inline void registerStateful(BT::BehaviorTreeFactory& factory,
                             const std::string& id,
                             const BT::PortsList& ports,
                             std::function<int(const std::string&)> builder,
                             std::function<int(int, BT::TreeNode&)> fs,
                             std::function<int(int, BT::TreeNode&)> fr,
                             std::function<void(int, BT::TreeNode&)> fh) {
  factory.registerBuilder(BT::CreateManifest<PyStatefulShim>(id, ports),
    [builder, fs, fr, fh](const std::string& name, const BT::NodeConfig& cfg)
        -> std::unique_ptr<BT::TreeNode> {
      int h = builder(name);
      return std::make_unique<PyStatefulShim>(name, cfg, h, fs, fr, fh);
    });
}

}  // namespace rclcppyy_btkit
"""


class _Node:
    """The object handed to a leaf callback (wraps a C++ BT::TreeNode).

    Mirrors the C++ node's port access with the cppyy friction removed:
    `get_input(key)` returns a string (C++: `getInput<std::string>(key)` +
    `Expected<T>` unwrap); `set_output(key, value)` writes an output port. The
    camelCase `getInput`/`setOutput` names work too (no template argument
    needed), as do `node["key"]` / `node["key"] = v`. The raw C++ node is at
    `node.raw` for anything not wrapped here.
    """

    def __init__(self, cpp_node):
        self.raw = cpp_node

    def get_input(self, key, cast=str, default=None):
        """Read an input port. `cast` mirrors the C++ template arg:
        `get_input("count", int)` == `getInput<int>("count")`. Also float/bool/str
        and `[int]`/`[float]`/... for vector ports; returns a Python value."""
        tag, to_python = _resolve_type(cast)
        raw = cppyy_kit.unwrap_expected(self.raw.getInput[tag](key), _MISSING)
        return default if raw is _MISSING else to_python(raw)

    def set_output(self, key, value, cast=None):
        """Write an output port. The C++ type is inferred from `value`
        (int/float/bool/str/list) unless `cast` is given."""
        tag = _resolve_type(cast)[0] if cast is not None else _infer_tag(value)
        if tag.startswith("std::vector<"):
            elem = tag[len("std::vector<"):-1]
            value = cppyy.gbl.std.vector[elem](list(value))
        self.raw.setOutput[tag](key, value)

    # camelCase aliases mirroring the C++ method names.
    getInput = get_input
    setOutput = set_output

    def name(self):
        return str(self.raw.name())

    def __getitem__(self, key):
        return self.get_input(key)

    def __setitem__(self, key, value):
        self.set_output(key, value)


def _coerce_status(result):
    """Map a leaf return value to a BT::NodeStatus. None -> SUCCESS,
    bool -> SUCCESS/FAILURE, int / bt_kit.SUCCESS / bt.NodeStatus.X -> the enum."""
    if result is None:
        result = SUCCESS
    elif isinstance(result, bool):
        result = SUCCESS if result else FAILURE
    return _STATUS[int(result)]


# Python type -> (C++ getInput/port tag, function turning the C++ value into a
# Python value). float maps to double, and a Python list [T] means a vector port.
_SCALAR_TAG = {int: "int", float: "double", bool: "bool", str: "std::string"}


def _resolve_type(spec):
    """Map a port/cast spec to (cpp_tag, to_python). `spec` is a Python type
    (int/float/bool/str), a [T] list for a vector, or a raw C++ tag string."""
    if isinstance(spec, str):
        return spec, (lambda v: v)
    if isinstance(spec, (list, tuple)):
        elem = spec[0] if spec else str
        return "std::vector<%s>" % _SCALAR_TAG[elem], (lambda v: list(v))
    if spec is str:
        return "std::string", (lambda v: str(v))
    return _SCALAR_TAG[spec], (lambda v: v)


def _infer_tag(value):
    """C++ tag to setOutput a Python value with (bool before int -- bool is an int)."""
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "double"
    if isinstance(value, (list, tuple)):
        return _resolve_type([type(value[0]) if value else str])[0]
    return "std::string"


def _make_ports(ports):
    """Build a PortsList from either a list of names (string ports) or a dict
    {name: type} for typed ports. type is a Python type, a [T] list, or a tag."""
    if isinstance(ports, dict):
        names = list(ports.keys())
        types = [_resolve_type(ports[n])[0] for n in names]
    else:
        names = [str(n) for n in (ports or [])]
        types = ["std::string"] * len(names)
    svec = cppyy.gbl.std.vector["std::string"]
    return cppyy.gbl.rclcppyy_btkit.makePorts(svec(names), svec(types))


def _tick_functor(fn, owner):
    def tick(cpp_node):
        return _coerce_status(fn(_Node(cpp_node)))
    # callback() infers nothing here (the C++ ref signature is explicit) but pins
    # tick (and, transitively, fn) on the factory so it can't be collected.
    return cppyy_kit.callback(tick, signature="BT::NodeStatus(BT::TreeNode&)", owner=owner)


def _adapt_factory(BT):
    """Patch BehaviorTreeFactory so the C++-named registration/creation methods
    accept plain Python callables and list-of-string ports (friction removed),
    while keeping the exact C++ method names. Idempotent."""
    Factory = BT.BehaviorTreeFactory
    if getattr(Factory, "_bt_kit_adapted", False):
        return

    Factory._orig_register_simple_action = Factory.registerSimpleAction
    Factory._orig_register_simple_condition = Factory.registerSimpleCondition
    Factory._orig_create_tree_from_text = Factory.createTreeFromText
    Factory._orig_create_tree_from_file = Factory.createTreeFromFile

    def register_simple_action(self, name, fn, ports=None):
        with cppyy_kit.first_use("bt_kit.register_simple_action", "bt_kit.warmup()"):
            self._orig_register_simple_action(name, _tick_functor(fn, self), _make_ports(ports))

    def register_simple_condition(self, name, fn, ports=None):
        with cppyy_kit.first_use("bt_kit.register_simple_condition", "bt_kit.warmup()"):
            self._orig_register_simple_condition(name, _tick_functor(fn, self), _make_ports(ports))

    def register_stateful(self, name, node_class, ports=None):
        """Register an asynchronous (multi-tick) node whose behaviour is a Python
        class exposing onStart/onRunning/onHalted (snake_case accepted too), each
        returning a status. This is the kit's stand-in for the C++
        `registerNodeType<StatefulActionNode>`, which cannot take a Python type.

        A fresh instance is created per tree-node instance, so two nodes with the
        same registered ID in one tree keep independent state. `node_class` should
        be a class (a factory callable also works)."""
        # C++ builds each node and calls back with an int handle; the per-instance
        # Python object lives in a registry, dispatched by handle -- ownership never
        # crosses into Python (see cppyy_kit.HandleRegistry).
        registry = cppyy_kit.HandleRegistry()

        def build(_node_name):
            return registry.add(node_class())

        def phase(camel, snake):
            def call(handle, node):
                inst = registry.get(handle)
                method = getattr(inst, camel, None) or getattr(inst, snake, None)
                return _coerce_status(method(_Node(node)))
            return call

        start, running = phase("onStart", "on_start"), phase("onRunning", "on_running")

        def f_start(handle, node):
            return int(start(handle, node))

        def f_running(handle, node):
            return int(running(handle, node))

        def f_halted(handle, node):
            inst = registry.get(handle)
            method = getattr(inst, "onHalted", None) or getattr(inst, "on_halted", None)
            if method is not None:
                method(_Node(node))

        # callback() pins each wrapper (and its Python fn) on the factory; the
        # closures hold `registry`, so the per-instance objects stay alive too.
        with cppyy_kit.first_use("bt_kit.register_stateful", "bt_kit.warmup()"):
            b = cppyy_kit.callback(build, signature="int(const std::string&)", owner=self)
            fs = cppyy_kit.callback(f_start, signature="int(int, BT::TreeNode&)", owner=self)
            fr = cppyy_kit.callback(f_running, signature="int(int, BT::TreeNode&)", owner=self)
            fh = cppyy_kit.callback(f_halted, signature="void(int, BT::TreeNode&)", owner=self)
            cppyy.gbl.rclcppyy_btkit.registerStateful(self, name, _make_ports(ports), b, fs, fr, fh)

    def create_tree_from_text(self, xml, *args):
        try:
            tree = self._orig_create_tree_from_text(_read_xml(xml), *args)
        except Exception as exc:  # cppyy.gbl.BT.RuntimeError / LogicError
            raise BtXmlError(cppyy_kit.pretty_cpp_error(exc)) from None
        # cppyy will not keep the leaf callbacks alive; carry the factory's pinned
        # objects onto the tree, which owns the nodes that hold them.
        cppyy_kit.keep_alive(tree, *getattr(self, "_cppyy_kit_kept_alive", []))
        return tree

    def create_tree_from_file(self, path, *args):
        try:
            tree = self._orig_create_tree_from_file(str(path), *args)
        except Exception as exc:
            raise BtXmlError(cppyy_kit.pretty_cpp_error(exc)) from None
        cppyy_kit.keep_alive(tree, *getattr(self, "_cppyy_kit_kept_alive", []))
        return tree

    # Keep the exact C++ names, and add snake_case aliases.
    Factory.registerSimpleAction = register_simple_action
    Factory.registerSimpleCondition = register_simple_condition
    Factory.createTreeFromText = create_tree_from_text
    Factory.createTreeFromFile = create_tree_from_file
    Factory.register_simple_action = register_simple_action
    Factory.register_simple_condition = register_simple_condition
    Factory.register_stateful = register_stateful
    Factory.create_tree_from_text = create_tree_from_text
    Factory.create_tree_from_file = create_tree_from_file
    Factory._bt_kit_adapted = True


def bringup_bt():
    """
    Bring up BehaviorTree.CPP under cppyy and return the adapted ``BT`` namespace.
    Idempotent.

    Discovers the behaviortree_cpp install via the ament index, adds its include
    path, JIT-includes bt_factory.h, loads libbehaviortree_cpp.so so calls resolve
    without LD_LIBRARY_PATH, compiles the C++ glue, and patches
    BehaviorTreeFactory (see _adapt_factory).
    """
    global _BT, _BRINGUP_DONE
    if _BRINGUP_DONE:
        return _BT

    if os.environ.get("RCLCPPYY_FROZEN") and not freeze.active("bt"):
        warnings.warn(
            "RCLCPPYY_FROZEN is set but no bt_kit frozen PCH is active, so bringup "
            "will JIT-parse the headers as usual. Launch via "
            "scripts/freeze/run_frozen.py -- CLING_STANDARD_PCH must be selected "
            "before cppyy is imported (see rclcppyy.kits.freeze).", stacklevel=2)

    prefix = cppyy_kit.package_prefix("behaviortree_cpp")
    cppyy.add_include_path(os.path.join(prefix, "include"))
    # On the frozen path this is a PCH lookup (~ms) instead of a ~0.83 s parse; the
    # include stays either way so cppyy registers the classes for autoloading.
    cppyy.include("behaviortree_cpp/bt_factory.h")
    # cppyy resolves symbols at call time by owning-library lookup; load the .so
    # explicitly rather than relying on LD_LIBRARY_PATH (see cppyy_kit).
    cppyy_kit.load_libraries(["libbehaviortree_cpp.so"], [os.path.join(prefix, "lib")])
    # Frozen only: emit the header's internal-linkage statics the AST-only PCH
    # doesn't (else the C++ glue below fails to link). No-op on the JIT path.
    freeze.apply_force_symbols("bt")
    cppyy.cppdef(_CPP_GLUE)

    _BT = cppyy.gbl.BT
    ns = _BT.NodeStatus
    _STATUS.update({
        IDLE: ns.IDLE, RUNNING: ns.RUNNING, SUCCESS: ns.SUCCESS,
        FAILURE: ns.FAILURE, SKIPPED: ns.SKIPPED,
    })
    _adapt_factory(_BT)
    _BRINGUP_DONE = True
    return _BT


def frozen():
    """True if bringup ran (or will run) on the frozen PCH path -- i.e. a bt_kit
    frozen PCH is the interpreter's active std PCH (see rclcppyy.kits.freeze)."""
    return freeze.active("bt")


def warmup():
    """Front-load bt_kit's one-time first-use JIT (~0.7 s) during init, so the
    first *live* tree build/tick doesn't stall.

    The first time each callback signature is crossed, cppyy JIT-compiles a call
    wrapper (the first ``registerSimpleAction`` alone is ~0.4 s; the stateful
    hooks another ~0.3 s). This exercises all of bt_kit's wrapper signatures on a
    throwaway factory + tree, so the wrappers are compiled and cached
    process-globally before your real tree is built. Idempotent in effect (cheap
    after the first call, since the wrappers are then cached). A freeze/PCH does
    not remove this cost -- warmup and freeze compose (freeze cuts the header
    parse, warmup moves the wrapper JIT off the first live call).
    """
    bt = bringup_bt()

    def _exercise():
        factory = bt.BehaviorTreeFactory()
        factory.register_simple_action("__warmup_action", lambda node: SUCCESS)
        factory.register_simple_condition("__warmup_condition", lambda node: SUCCESS)

        class _WarmupStateful:
            def onStart(self, node):
                return SUCCESS

            def onRunning(self, node):
                return SUCCESS

            def onHalted(self, node):
                pass

        factory.register_stateful("__warmup_stateful", _WarmupStateful)
        xml = ('<root BTCPP_format="4"><BehaviorTree ID="__warmup"><Sequence>'
               '<__warmup_action/></Sequence></BehaviorTree></root>')
        factory.create_tree_from_text(xml).tickWhileRunning()

    cppyy_kit.warmup(_exercise)


# --- Observability -------------------------------------------------------
# Loggers/observers attach to a tree at construction (RAII) and must outlive the
# ticking, so each helper pins the object on the tree. The logger headers are
# JIT-included lazily (they pull in zmq/flatbuffers) so plain bringup stays fast.
_LOGGERS_INCLUDED = False


def _ensure_loggers():
    global _LOGGERS_INCLUDED
    if _LOGGERS_INCLUDED:
        return
    bringup_bt()
    for header in ("bt_cout_logger.h", "bt_file_logger_v2.h",
                   "bt_observer.h", "groot2_publisher.h"):
        cppyy.include("behaviortree_cpp/loggers/%s" % header)
    _LOGGERS_INCLUDED = True


def add_cout_logger(tree):
    """Print every node status transition to stdout while `tree` ticks."""
    _ensure_loggers()
    logger = _BT.StdCoutLogger(tree)
    cppyy_kit.keep_alive(tree, logger)
    return logger


def add_file_logger(tree, path):
    """Record transitions to a .btlog file (replayable in Groot2)."""
    _ensure_loggers()
    logger = _BT.FileLogger2(tree, str(path))
    cppyy_kit.keep_alive(tree, logger)
    return logger


def add_groot2_publisher(tree, port=1667):
    """Publish live tree state for the Groot2 monitor (ZMQ on `port`)."""
    _ensure_loggers()
    publisher = _BT.Groot2Publisher(tree, port)
    cppyy_kit.keep_alive(tree, publisher)
    return publisher


class _Observer:
    """Thin wrapper over BT::TreeObserver: per-node tick/success/failure counts."""

    def __init__(self, tree):
        _ensure_loggers()
        self._tree = tree
        self._obs = _BT.TreeObserver(tree)
        cppyy_kit.keep_alive(tree, self._obs)

    def stats(self, node_path):
        s = self._obs.getStatistics(node_path)
        return {"transitions": int(s.transitions_count),
                "success": int(s.success_count),
                "failure": int(s.failure_count)}

    def counts(self):
        """{node_full_path: stats} for every node in the tree."""
        out = {}
        for subtree in self._tree.subtrees:
            for node in subtree.nodes:
                path = str(node.fullPath())
                out[path] = self.stats(path)
        return out


def observe(tree):
    """Attach a TreeObserver to `tree` and return an object exposing per-node
    tick/success/failure counts via .stats(path) and .counts()."""
    return _Observer(tree)


def _read_xml(xml):
    """Accept inline XML text or a path to an XML file; return the XML text."""
    if isinstance(xml, str) and xml.lstrip().startswith("<"):
        return xml
    text = str(xml)
    if os.path.isfile(text):
        with open(text, "r") as handle:
            return handle.read()
    return xml
