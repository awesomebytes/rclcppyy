#!/usr/bin/env python3
"""Tests for rclcppyy.kits.cppyy_kit (the common cppyy-kit pattern layer).

The pure-Python helpers need no C++ library, but the module gates on
behaviortree_cpp (present only in the pixi `bt` env) so the default
`pixi run test` is unaffected. Run via `pixi run -e bt test-bt`.
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
    from rclcppyy.kits import cppyy_kit


def test_pretty_cpp_error_strips_signature_wall():
    exc = RuntimeError(
        "BT::Tree BT::BehaviorTreeFactory::createTreeFromText(const std::string&) =>\n"
        "    RuntimeError: Error at line 4: -> Node not recognized: Nope")
    msg = cppyy_kit.pretty_cpp_error(exc)
    assert msg == "RuntimeError: Error at line 4: -> Node not recognized: Nope"
    assert "\n" not in msg
    assert "createTreeFromText" not in msg


def test_pretty_cpp_error_passthrough_without_marker():
    assert cppyy_kit.pretty_cpp_error(ValueError("plain message")) == "plain message"


def test_handle_registry_add_get_len():
    reg = cppyy_kit.HandleRegistry()
    a, b = object(), object()
    ha, hb = reg.add(a), reg.add(b)
    assert ha != hb
    assert reg.get(ha) is a and reg.get(hb) is b
    assert len(reg) == 2


def test_keep_alive_pins_objects():
    class Owner:
        pass

    owner = Owner()
    marker = object()
    cppyy_kit.keep_alive(owner, marker)
    assert marker in owner._cppyy_kit_kept_alive


def test_keep_alive_best_effort_on_unattributable_owner():
    # ints can't hold attributes; must not raise.
    cppyy_kit.keep_alive(5, object())


def test_unwrap_expected():
    class Present:
        def has_value(self):
            return True

        def value(self):
            return 42

    class Absent:
        def has_value(self):
            return False

        def value(self):
            raise AssertionError("value() must not be called when empty")

    assert cppyy_kit.unwrap_expected(Present()) == 42
    assert cppyy_kit.unwrap_expected(Absent(), default="x") == "x"


def test_std_function_wraps_python_callable():
    # A Python callable becomes a std::function callable from C++/Python.
    # The callable must stay referenced (std_function does not pin it -- passing a
    # throwaway lambda would be collected and raise "callable was deleted").
    def inc(x):
        return x + 1

    fn = cppyy_kit.std_function("int(int)", inc)
    assert int(fn(41)) == 42


def test_load_libraries_smoke():
    # Loading the BT library must not raise (search path from its prefix).
    import os
    prefix = cppyy_kit.package_prefix("behaviortree_cpp")
    cppyy_kit.load_libraries(["libbehaviortree_cpp.so"], [os.path.join(prefix, "lib")])


def test_probe_cppdef_accepts_valid_and_rejects_invalid():
    ok, _ = cppyy_kit.probe_cppdef("namespace ckp { inline int good() { return 1; } }")
    assert ok is True
    bad_ok, message = cppyy_kit.probe_cppdef("this is definitely not c++ !!!")
    assert bad_ok is False
    assert message  # carries some diagnostic text


@pytest.fixture
def isolated_teardown_registry():
    """Give a test its own teardown registry, then restore the module's so the
    real atexit shutdown (and any callbacks bringup registered) is untouched."""
    saved_list = list(cppyy_kit._TEARDOWN)
    saved_done = cppyy_kit._SHUTDOWN_DONE
    cppyy_kit._TEARDOWN.clear()
    cppyy_kit._SHUTDOWN_DONE = False
    try:
        yield
    finally:
        cppyy_kit._TEARDOWN[:] = saved_list
        cppyy_kit._SHUTDOWN_DONE = saved_done


def test_shutdown_runs_lifo_and_is_idempotent(isolated_teardown_registry):
    order = []

    def first():
        order.append("first")

    def second():
        order.append("second")

    cppyy_kit.register_teardown(first)
    cppyy_kit.register_teardown(second)
    # A duplicate registration of the same callable is ignored.
    cppyy_kit.register_teardown(first)

    cppyy_kit.shutdown()
    # LIFO: last registered runs first; the duplicate did not add a second run.
    assert order == ["second", "first"]

    # A second shutdown() is a no-op (idempotent) -- the atexit backstop and an
    # explicit call cannot double-run teardown.
    cppyy_kit.shutdown()
    assert order == ["second", "first"]


def test_shutdown_swallows_callback_exceptions(isolated_teardown_registry):
    ran = []

    def boom():
        raise RuntimeError("teardown failure must not abort the rest")

    def good():
        ran.append("good")

    # good is registered last, so LIFO runs it first, then boom raises; a boom
    # registered first still must not prevent good from having run.
    cppyy_kit.register_teardown(boom)
    cppyy_kit.register_teardown(good)
    cppyy_kit.shutdown()  # must not raise
    assert ran == ["good"]
