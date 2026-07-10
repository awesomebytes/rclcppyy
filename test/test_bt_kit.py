#!/usr/bin/env python3
"""Tests for rclcppyy.kits.bt_kit (BehaviorTree.CPP via cppyy).

BehaviorTree.CPP is an optional dependency (the pixi `bt` env), absent from the
default env. The whole module therefore auto-skips when behaviortree_cpp is not
installed, so the default `pixi run test` is unaffected. Run the real thing with
`pixi run -e bt test-bt`.

All tests share one process; bringup_bt() is idempotent and each test builds its
own factory/tree, so they stay independent. Pure BT.CPP only (no rclcpp), so the
process exits cleanly with no teardown wart.
"""
import pytest

try:
    from ament_index_python.packages import get_package_prefix
    get_package_prefix("behaviortree_cpp")
    _HAVE_BT = True
except Exception:
    _HAVE_BT = False

pytestmark = pytest.mark.skipif(not _HAVE_BT,
                                reason="behaviortree_cpp not installed (use the bt env)")

if _HAVE_BT:
    from rclcppyy.kits import bt_kit


@pytest.fixture(scope="module")
def bt():
    return bt_kit.bringup_bt()


def _tree(bt, factory, xml):
    return factory.create_tree_from_text(xml)


def test_basic_builtin_tree(bt):
    xml = """
    <root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>
      <AlwaysSuccess/><AlwaysSuccess/>
    </Sequence></BehaviorTree></root>"""
    status = bt.BehaviorTreeFactory().create_tree_from_text(xml).tickWhileRunning()
    assert status == bt_kit.SUCCESS


def test_python_action_and_condition(bt):
    calls = []
    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleCondition("Check", lambda n: calls.append("c") or bt_kit.SUCCESS)
    factory.registerSimpleAction("Act", lambda n: calls.append("a") or bt_kit.SUCCESS)
    xml = """
    <root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>
      <Check/><Act/>
    </Sequence></BehaviorTree></root>"""
    status = factory.create_tree_from_text(xml).tickWhileRunning()
    assert status == bt_kit.SUCCESS
    assert calls == ["c", "a"]


def test_typed_ports_roundtrip(bt):
    seen = {}

    def compute(node):
        node.set_output("sum", node.get_input("a", int) + node.get_input("b", float))
        node.set_output("ok", node.get_input("a", int) > 0)
        return bt_kit.SUCCESS

    def report(node):
        seen["sum"] = node.get_input("sum", float)
        seen["ok"] = node.get_input("ok", bool)
        seen["items"] = node.get_input("items", [float])
        return bt_kit.SUCCESS

    factory = bt.BehaviorTreeFactory()
    factory.register_simple_action("Compute", compute,
                                   ports={"a": int, "b": float, "sum": float, "ok": bool})
    factory.register_simple_action("Report", report,
                                   ports={"sum": float, "ok": bool, "items": [float]})
    xml = """
    <root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>
      <Compute a="3" b="1.5" sum="{s}" ok="{k}"/>
      <Report sum="{s}" ok="{k}" items="0.5;1.0;2.0"/>
    </Sequence></BehaviorTree></root>"""
    factory.create_tree_from_text(xml).tickWhileRunning()
    assert seen == {"sum": 4.5, "ok": True, "items": [0.5, 1.0, 2.0]}


def test_stateful_multi_instance_independent_state(bt):
    log = []

    class CountTo:
        def onStart(self, node):
            self.target = node.get_input("n", int)
            self.i = 0
            return bt_kit.RUNNING

        def onRunning(self, node):
            self.i += 1
            log.append((self.target, self.i))
            return bt_kit.SUCCESS if self.i >= self.target else bt_kit.RUNNING

        def onHalted(self, node):
            pass

    factory = bt.BehaviorTreeFactory()
    factory.register_stateful("CountTo", CountTo, ports={"n": int})
    xml = """
    <root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>
      <CountTo n="2"/><CountTo n="4"/>
    </Sequence></BehaviorTree></root>"""
    status = factory.create_tree_from_text(xml).tickWhileRunning()
    assert status == bt_kit.SUCCESS
    assert [i for t, i in log if t == 2] == [1, 2]
    assert [i for t, i in log if t == 4] == [1, 2, 3, 4]


def test_xml_error_is_readable(bt):
    factory = bt.BehaviorTreeFactory()
    bad = '<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence><NoSuchNode/></Sequence></BehaviorTree></root>'
    with pytest.raises(bt_kit.BtXmlError) as exc:
        factory.create_tree_from_text(bad)
    message = str(exc.value)
    assert "NoSuchNode" in message
    assert "\n" not in message          # one clean line, not a C++ signature wall
    assert "BehaviorTreeFactory::createTreeFromText" not in message


def test_observer_counts_ticks(bt):
    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleAction("A", lambda n: bt_kit.SUCCESS)
    xml = '<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence><A/></Sequence></BehaviorTree></root>'
    tree = factory.create_tree_from_text(xml)
    observer = bt_kit.observe(tree)
    for _ in range(3):
        tree.tickWhileRunning()
    counts = observer.counts()
    assert counts, "observer returned no node statistics"
    assert sum(s["transitions"] for s in counts.values()) > 0


def test_subtree_composition(bt):
    log = []
    factory = bt.BehaviorTreeFactory()
    factory.register_simple_action("Say", lambda n: log.append(n.get_input("m")) or bt_kit.SUCCESS,
                                   ports=["m"])
    xml = """
    <root BTCPP_format="4" main_tree_to_execute="Main">
      <BehaviorTree ID="Main"><Sequence>
        <Say m="start"/><SubTree ID="Greet"/><Say m="end"/>
      </Sequence></BehaviorTree>
      <BehaviorTree ID="Greet"><Sequence>
        <Say m="hello"/><Say m="world"/>
      </Sequence></BehaviorTree>
    </root>"""
    factory.create_tree_from_text(xml).tickWhileRunning()
    assert log == ["start", "hello", "world", "end"]
