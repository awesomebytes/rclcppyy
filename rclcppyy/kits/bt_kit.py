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

Notes / limits (v0):
    * Ports are string-typed and bidirectional (readable via `node.get_input`,
      writable via `node.set_output`). Typed/directioned ports are not modelled.
    * Leaf logic runs in Python (holds the GIL); use it for orchestration, not
      hot inner loops. For fast leaves, register a JIT'd C++ functor instead.
"""
import os

import cppyy
from ament_index_python.packages import get_package_prefix

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

# C++ glue compiled once at bringup. makePorts keeps the (segfault-prone in
# cppyy) unordered_map<string, PortInfo> construction on the C++ side.
# PyStatefulShim exposes the pure-virtual StatefulActionNode hooks as
# std::function slots -- Python cannot subclass StatefulActionNode directly
# (its tick()/halt() are `final`, which cppyy's override dispatcher cannot
# regenerate), so asynchronous nodes route through this shim.
_CPP_GLUE = r"""
namespace rclcppyy_btkit {

inline BT::PortsList makePorts(const std::vector<std::string>& names) {
  BT::PortsList ports;
  for (const auto& n : names) {
    ports.insert(BT::BidirectionalPort<std::string>(n));
  }
  return ports;
}

class PyStatefulShim : public BT::StatefulActionNode {
public:
  std::function<int(BT::TreeNode&)> f_start, f_running;
  std::function<void(BT::TreeNode&)> f_halted;
  PyStatefulShim(const std::string& name, const BT::NodeConfig& cfg)
    : BT::StatefulActionNode(name, cfg) {}
  BT::NodeStatus onStart() override { return static_cast<BT::NodeStatus>(f_start(*this)); }
  BT::NodeStatus onRunning() override { return static_cast<BT::NodeStatus>(f_running(*this)); }
  void onHalted() override { if (f_halted) f_halted(*this); }
};

inline void registerStateful(BT::BehaviorTreeFactory& factory,
                             const std::string& id,
                             const BT::PortsList& ports,
                             std::function<int(BT::TreeNode&)> fs,
                             std::function<int(BT::TreeNode&)> fr,
                             std::function<void(BT::TreeNode&)> fh) {
  factory.registerBuilder(BT::CreateManifest<PyStatefulShim>(id, ports),
    [fs, fr, fh](const std::string& name, const BT::NodeConfig& cfg)
        -> std::unique_ptr<BT::TreeNode> {
      auto node = std::make_unique<PyStatefulShim>(name, cfg);
      node->f_start = fs;
      node->f_running = fr;
      node->f_halted = fh;
      return node;
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

    def get_input(self, key, default=None):
        expected = self.raw.getInput["std::string"](key)
        return str(expected.value()) if expected.has_value() else default

    def set_output(self, key, value):
        self.raw.setOutput["std::string"](key, str(value))

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


def _make_ports(names):
    vec = cppyy.gbl.std.vector["std::string"]([str(n) for n in (names or [])])
    return cppyy.gbl.rclcppyy_btkit.makePorts(vec)


def _pin(obj, *items):
    """Pin Python callables + their std::function wrappers on a cppyy object so
    cppyy does not collect them (else 'callable was deleted' at tick time)."""
    store = getattr(obj, "_bt_kit_pins", None)
    if store is None:
        store = []
        obj._bt_kit_pins = store
    store.extend(items)


def _tick_functor(fn):
    functor_t = cppyy.gbl.std.function["BT::NodeStatus(BT::TreeNode&)"]

    def tick(cpp_node):
        return _coerce_status(fn(_Node(cpp_node)))
    return tick, functor_t(tick)


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
        tick, cpp_fn = _tick_functor(fn)
        _pin(self, tick, cpp_fn)
        self._orig_register_simple_action(name, cpp_fn, _make_ports(ports))

    def register_simple_condition(self, name, fn, ports=None):
        tick, cpp_fn = _tick_functor(fn)
        _pin(self, tick, cpp_fn)
        self._orig_register_simple_condition(name, cpp_fn, _make_ports(ports))

    def register_stateful(self, name, node_class, ports=None):
        """Register an asynchronous (multi-tick) node whose behaviour is a Python
        class exposing onStart/onRunning/onHalted (snake_case accepted too), each
        returning a status. This is the kit's stand-in for the C++
        `registerNodeType<StatefulActionNode>`, which cannot take a Python type."""
        instance = node_class() if isinstance(node_class, type) else node_class

        def hook(camel, snake):
            return getattr(instance, camel, None) or getattr(instance, snake, None)

        start = hook("onStart", "on_start")
        running = hook("onRunning", "on_running")
        halted = hook("onHalted", "on_halted")

        def f_start(node):
            return int(_coerce_status(start(_Node(node))))

        def f_running(node):
            return int(_coerce_status(running(_Node(node))))

        def f_halted(node):
            if halted is not None:
                halted(_Node(node))

        int_fn = cppyy.gbl.std.function["int(BT::TreeNode&)"]
        void_fn = cppyy.gbl.std.function["void(BT::TreeNode&)"]
        fs, fr, fh = int_fn(f_start), int_fn(f_running), void_fn(f_halted)
        _pin(self, instance, f_start, f_running, f_halted, fs, fr, fh)
        cppyy.gbl.rclcppyy_btkit.registerStateful(self, name, _make_ports(ports), fs, fr, fh)

    def create_tree_from_text(self, xml, *args):
        tree = self._orig_create_tree_from_text(_read_xml(xml), *args)
        # cppyy will not keep the leaf callbacks alive; carry the factory's pins
        # onto the tree, which owns the nodes that hold them.
        tree._bt_kit_pins = list(getattr(self, "_bt_kit_pins", []))
        return tree

    def create_tree_from_file(self, path, *args):
        tree = self._orig_create_tree_from_file(str(path), *args)
        tree._bt_kit_pins = list(getattr(self, "_bt_kit_pins", []))
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

    prefix = get_package_prefix("behaviortree_cpp")
    cppyy.add_include_path(os.path.join(prefix, "include"))
    cppyy.add_library_path(os.path.join(prefix, "lib"))
    cppyy.include("behaviortree_cpp/bt_factory.h")
    # cppyy resolves symbols at call time by owning-library lookup; load it
    # explicitly rather than relying on LD_LIBRARY_PATH (see bringup_rclcpp).
    cppyy.load_library("libbehaviortree_cpp.so")
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


def _read_xml(xml):
    """Accept inline XML text or a path to an XML file; return the XML text."""
    if isinstance(xml, str) and xml.lstrip().startswith("<"):
        return xml
    text = str(xml)
    if os.path.isfile(text):
        with open(text, "r") as handle:
            return handle.read()
    return xml
