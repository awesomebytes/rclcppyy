#!/usr/bin/env python3
"""Tests for rclcppyy.kits.cppyy_kit (the common cppyy-kit pattern layer).

The pure-Python helpers need no C++ library, but the module gates on
behaviortree_cpp (present only in the pixi `bt` env) so the default
`pixi run test` is unaffected. Run via `pixi run -e bt test-bt`.
"""
import gc
import time

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
    import cppyy
    from rclcppyy.kits import bt_kit, cppyy_kit

    # A tiny C++ holder for the C++<->Python direction tests: a free function
    # Python can call, and a std::function slot C++ can store and invoke.
    cppyy.cppdef(r"""
    namespace cbtest {
      inline int cpp_double(int x) { return x * 2; }
      struct Holder {
        std::function<int(int)> fn;
        void store(std::function<int(int)> f) { fn = f; }
        int invoke(int x) { return fn(x); }
      };
    }
    """)


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


# --- callback() ergonomics -------------------------------------------------

def _annotated(arg_type, ret_type):
    """A one-arg function `fn(x)` with the given parameter/return annotations."""
    def fn(x):
        return x
    fn.__annotations__ = {"x": arg_type, "return": ret_type}
    return fn


def test_callback_infers_scalar_signatures():
    table = [
        (int, int, "int(int)"),
        (float, float, "double(double)"),
        (bool, bool, "bool(bool)"),
        (str, str, "std::string(std::string)"),
        (int, None, "void(int)"),
    ]
    for arg_type, ret_type, expected in table:
        assert cppyy_kit._infer_signature(_annotated(arg_type, ret_type)) == expected


def test_callback_infers_cppyy_type_as_reference():
    tree_node = bt_kit.bringup_bt().TreeNode
    assert cppyy_kit._infer_signature(_annotated(tree_node, int)) == "int(BT::TreeNode&)"


def test_callback_explicit_signature_wins_over_inference():
    def unannotated(x):
        return x + 1

    # inference would fail (no hints), but the explicit signature is used.
    fn = cppyy_kit.callback(unannotated, signature="int(int)")
    assert int(fn(41)) == 42


def test_callback_inference_error_is_readable():
    def unannotated(x):
        return x

    with pytest.raises(cppyy_kit.CppyyKitError) as exc:
        cppyy_kit.callback(unannotated)
    message = str(exc.value)
    assert "x" in message and "signature=" in message


def test_callback_survives_gc_without_owner():
    # The classic "callable was deleted" footgun: with callback() it is gone,
    # because the module registry pins the callable even after local refs drop.
    def make():
        def cb(x: int) -> int:
            return x + 1
        return cppyy_kit.callback(cb)

    fn = make()
    gc.collect()
    assert int(fn(41)) == 42


def test_callback_owner_scoped_pin():
    class Owner:
        pass

    owner = Owner()

    def cb(x: int) -> int:
        return x + 2

    fn = cppyy_kit.callback(cb, owner=owner)
    assert cb in owner._cppyy_kit_kept_alive and fn in owner._cppyy_kit_kept_alive
    assert int(fn(40)) == 42


def test_callback_gc_stress():
    wrappers = []
    for i in range(200):
        def cb(x: int, base=i) -> int:
            return x + base
        wrappers.append(cppyy_kit.callback(cb))
    del cb
    gc.collect()
    assert int(wrappers[0](10)) == 10
    assert int(wrappers[199](10)) == 209


def test_cpp_function_is_callable_from_python():
    # C++ -> Python: a cppdef'd function is already a Python callable, no helper.
    assert int(cppyy.gbl.cbtest.cpp_double(21)) == 42


def test_cpp_function_handed_into_cpp_api():
    # A C++ function passed straight back into a C++ API (std::function slot).
    holder = cppyy.gbl.cbtest.Holder()
    holder.store(cppyy.gbl.cbtest.cpp_double)
    assert int(holder.invoke(21)) == 42


def test_python_to_cpp_roundtrip():
    # Python callback -> stored in C++ -> invoked from C++ -> calls a 2nd Python fn.
    seen = []

    def second(y: int) -> int:
        seen.append(y)
        return y + 100

    def first(x: int) -> int:
        return second(x)

    holder = cppyy.gbl.cbtest.Holder()
    holder.store(cppyy_kit.callback(first))  # inferred int(int), auto-pinned
    assert int(holder.invoke(5)) == 105
    assert seen == [5]


def test_release_callbacks_empties_registry():
    # Kept last: clears the shared module registry other owner-less callbacks use.
    def cb(x: int) -> int:
        return x

    cppyy_kit.callback(cb)
    assert len(cppyy_kit._CALLBACKS) > 0
    cppyy_kit.release_callbacks()
    assert len(cppyy_kit._CALLBACKS) == 0


# --- first-use notice + warmup --------------------------------------------
# Unique labels/hints per test so the module-global seen/shown sets don't couple
# tests together (a label fires at most once per process).

def test_first_use_notice_fires_once_when_slow(capsys):
    label, hint = "test.slow.a", "test.warmup.a()"
    with cppyy_kit.first_use(label, hint, threshold_ms=5):
        time.sleep(0.02)
    err = capsys.readouterr().err
    assert label in err and hint in err and "RCLCPPYY_JIT_NOTICE=0" in err
    # second call for the same label: already seen -> silent.
    with cppyy_kit.first_use(label, hint, threshold_ms=5):
        time.sleep(0.02)
    assert capsys.readouterr().err == ""


def test_first_use_silent_when_fast(capsys):
    with cppyy_kit.first_use("test.fast.b", "test.warmup.b()", threshold_ms=1000):
        pass
    assert capsys.readouterr().err == ""


def test_first_use_disabled_by_env(capsys, monkeypatch):
    monkeypatch.setenv("RCLCPPYY_JIT_NOTICE", "0")
    with cppyy_kit.first_use("test.disabled.c", "test.warmup.c()", threshold_ms=1):
        time.sleep(0.01)
    assert capsys.readouterr().err == ""


def test_suppress_first_use_notice(capsys):
    with cppyy_kit.suppress_first_use_notice():
        with cppyy_kit.first_use("test.suppressed.d", "test.warmup.d()", threshold_ms=1):
            time.sleep(0.01)
    assert capsys.readouterr().err == ""


def test_warmup_runs_thunks_under_suppression(capsys):
    ran = []

    def thunk():
        ran.append(1)
        with cppyy_kit.first_use("test.warmup.e", "test.warmup.e()", threshold_ms=1):
            time.sleep(0.01)

    cppyy_kit.warmup(thunk)
    assert ran == [1]
    assert capsys.readouterr().err == ""  # suppressed while warming
