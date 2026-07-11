"""
cppyy_kit -- common patterns shared by rclcppyy's cppyy "kits" (bt_kit, pcl_kit).

A "kit" wraps a C++ library so it can be driven from short Python, mirroring the
library's own API and hiding only the cppyy friction. That friction is the same
from one library to the next, so it lives here:

  * bringup -- locate the install, add include paths, and load the ``.so`` set so
    symbols resolve at call time (``add_library_path`` alone does NOT resolve
    symbols; cppyy finds a symbol's owning library by soname when you call it);
  * lifetime -- pin Python callables / views alive against C++ (``keep_alive``),
    else a collected callback raises "callable was deleted";
  * crossing functions both ways -- hand a Python callable to C++ in one line,
    signature inferred and lifetime pinned (``callback``; ``std_function`` is the
    low-level escape hatch); a cppdef'd C++ function is already a Python callable
    with no helper; give a C++ object a per-instance Python peer through an integer
    handle when ownership can't cross (``HandleRegistry``);
  * safety -- build STL/containers inside ``cppyy.cppdef`` C++ (constructing them
    from Python can SIGSEGV); probe a risky ``cppdef`` in a subprocess first
    (``probe_cppdef``), because a failed one can crash on transaction revert;
  * ergonomics -- unwrap ``Expected<T>``/optional (``unwrap_expected``), and turn
    cppyy's "<C++ signature> =>" error walls into a readable one-line exception
    (``pretty_cpp_error`` / ``CppyyKitError``);
  * teardown -- release C++ resources that own process-global/static state (an
    rclcpp Context, a DDS participant, a ZMQ-backed logger) in a defined order
    *before* Python finalization, so their destructors never interleave with
    cppyy's own Cling teardown (``register_teardown`` / ``shutdown``).

See docs/kits/COMMON_PATTERNS.md for the full catalog and the evidence behind it.
"""
import atexit
import contextlib
import inspect
import os
import subprocess
import sys
import time
import typing

import cppyy


class CppyyKitError(Exception):
    """Base for kit-raised errors; message is the cleaned C++ what() text."""


def pretty_cpp_error(exc):
    """Strip cppyy's leading ``<C++ signature> =>`` from an exception message,
    leaving the underlying C++ ``what()`` as a single readable line."""
    msg = str(exc)
    if "=>" in msg:
        msg = msg.split("=>", 1)[1]
    return " ".join(msg.split()).strip() or str(exc)


def package_prefix(package):
    """Install prefix of an ament package (e.g. 'rclcpp', 'behaviortree_cpp')."""
    from ament_index_python.packages import get_package_prefix
    return get_package_prefix(package)


def load_libraries(sonames, search_paths=()):
    """Make cppyy able to resolve the given libraries' symbols.

    cppyy finds a symbol's *owning* ``.so`` at call time by scanning its own
    library search path, so every library you call into must be ``load_library``'d
    by soname -- ``add_library_path`` alone is not enough. ``search_paths`` (e.g.
    ``$CONDA_PREFIX/lib``) are added to that search path first.
    """
    for path in search_paths:
        cppyy.add_library_path(path)
    for soname in sonames:
        cppyy.load_library(soname)


def keep_alive(owner, *objects):
    """Pin Python objects to ``owner`` so cppyy does not collect them while C++
    still references them (callbacks, their ``std::function`` wrappers, buffers
    backing a zero-copy view, ...). Stored in a list attribute on ``owner``;
    best-effort if ``owner`` cannot hold attributes."""
    store = getattr(owner, "_cppyy_kit_kept_alive", None)
    if store is None:
        store = []
        try:
            owner._cppyy_kit_kept_alive = store
        except (AttributeError, TypeError):
            return
    store.extend(objects)


def std_function(signature, pyfunc):
    """Low-level: wrap a Python callable as ``std::function<signature>``.

    Prefer ``callback()`` (which infers the signature and pins lifetime for you);
    reach for this only when you want the raw wrapper and will handle
    ``keep_alive`` yourself. The callback runs in whatever C++ thread invokes it
    (cppyy takes the GIL); cppyy does not keep the callable alive on its own.
    """
    return cppyy.gbl.std.function[signature](pyfunc)


# Python annotation -> C++ type tag for callback-signature inference. None and
# NoneType both mean a void return.
_SCALAR_CPP = {int: "int", float: "double", bool: "bool", str: "std::string",
               type(None): "void", None: "void"}

# Process-lifetime pins for callback() wrappers registered without an owner.
_CALLBACKS = []


def _cpp_type(annotation, is_return=False):
    if annotation in _SCALAR_CPP:
        return _SCALAR_CPP[annotation]
    cpp_name = getattr(annotation, "__cpp_name__", None)
    if cpp_name:
        # A C++ object crosses by reference by default; use signature= for
        # by-value / const / pointer forms.
        return cpp_name if is_return else cpp_name + "&"
    raise CppyyKitError(
        "cppyy_kit.callback: cannot infer a C++ type for annotation %r. Annotate "
        "with int, float, bool, str, None (void), or a cppyy C++ class (one "
        "exposing __cpp_name__), or pass signature='ret(args)' explicitly." % (annotation,))


def _infer_signature(fn):
    try:
        hints = typing.get_type_hints(fn)
    except Exception as exc:
        raise CppyyKitError("cppyy_kit.callback: could not read type hints of %r (%s); "
                            "pass signature= explicitly." % (fn, exc))
    # Only the parameters C++ actually passes: positional, without a Python-side
    # default (defaults / *args / **kwargs are Python conveniences, not C++ args).
    params = [p.name for p in inspect.signature(fn).parameters.values()
              if p.default is inspect.Parameter.empty
              and p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    missing = [p for p in params if p not in hints]
    if missing:
        raise CppyyKitError(
            "cppyy_kit.callback: parameter(s) %s of %r are not type-annotated, so the "
            "C++ signature can't be inferred. Annotate them (int/float/bool/str/cppyy "
            "class) or pass signature='ret(args)'." % (missing, getattr(fn, "__name__", fn)))
    args = ", ".join(_cpp_type(hints[p]) for p in params)
    return "%s(%s)" % (_cpp_type(hints.get("return", None), is_return=True), args)


def callback(fn, signature=None, owner=None):
    """Wrap a Python callable as a ``std::function`` to hand to C++ -- one line,
    with the signature inferred and the lifetime handled for you.

    Signature: ``signature`` wins if given; otherwise it is inferred from ``fn``'s
    type hints -- ``int``->int, ``float``->double, ``bool``->bool, ``str``->
    std::string, ``None``->void (return), and any cppyy C++ class (via its
    ``__cpp_name__``) as a reference. Inference fails early with a readable error
    if a parameter is unannotated or unmappable; pass ``signature=`` for
    reference/const/pointer forms or types inference can't name (e.g.
    ``signature="BT::NodeStatus(BT::TreeNode&)"``).

    Lifetime (the "callable was deleted" footgun, gone): the wrapper *and* ``fn``
    are always pinned. With ``owner=`` they are pinned on that object and live as
    long as it does; without ``owner=`` they are pinned in a module-level registry
    for the process lifetime (drop those with ``release_callbacks()``).

    Threading: the callback runs in whatever C++ thread invokes it (cppyy takes
    the GIL); a single-threaded driver -- a tick loop, a spin -- never contends.
    """
    sig = signature if signature is not None else _infer_signature(fn)
    wrapper = cppyy.gbl.std.function[sig](fn)
    if owner is not None:
        keep_alive(owner, fn, wrapper)
    else:
        _CALLBACKS.append((fn, wrapper))
    return wrapper


def release_callbacks():
    """Drop the process-lifetime pins held for owner-less ``callback()`` wrappers.
    Only safe once no C++ code will invoke those callbacks again."""
    _CALLBACKS.clear()


# --- First-use JIT visibility & warmup ------------------------------------
# cppyy JIT-compiles a call wrapper the first time a given C++ signature is
# crossed (e.g. the first registerSimpleAction spends ~0.4 s generating the
# std::function<NodeStatus(TreeNode&)> thunk). It is a one-time, per-signature
# cost that a freeze/PCH does *not* remove (that only skips the header parse), so
# a script's first live call can halt unexpectedly. We make it visible and
# movable: kits wrap their known-expensive entry points in `first_use(...)`,
# which -- only on the first call, only if it was actually slow -- prints a
# one-time, LLM-actionable notice naming the API and the warmup() to call. After
# that first call (or when RCLCPPYY_JIT_NOTICE=0, or while warming up) it is a
# bare passthrough: no timing, no output.
_FIRST_USE_SEEN = set()     # labels already observed (timed) once
_FIRST_USE_SHOWN = set()    # warmup hints already printed (dedup the notice)
_WARMING_UP = False


def _jit_notice_enabled():
    return os.environ.get("RCLCPPYY_JIT_NOTICE", "1") != "0"


@contextlib.contextmanager
def suppress_first_use_notice():
    """Within this block, ``first_use`` marks its labels seen but never prints --
    used by ``warmup()``, which pays the first-use cost on purpose."""
    global _WARMING_UP
    previous, _WARMING_UP = _WARMING_UP, True
    try:
        yield
    finally:
        _WARMING_UP = previous


@contextlib.contextmanager
def first_use(label, warmup_hint, threshold_ms=150):
    """Wrap a kit entry point that may pay first-use JIT. On the first call for
    ``label`` that exceeds ``threshold_ms``, print a one-time notice pointing at
    ``warmup_hint``; thereafter (and when disabled or warming up) do nothing but
    run the block."""
    if label in _FIRST_USE_SEEN or not _jit_notice_enabled():
        yield
        return
    start = time.perf_counter()
    try:
        yield
    finally:
        _FIRST_USE_SEEN.add(label)
        if not _WARMING_UP:
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            if elapsed_ms >= threshold_ms and warmup_hint not in _FIRST_USE_SHOWN:
                _FIRST_USE_SHOWN.add(warmup_hint)
                sys.stderr.write(
                    "[cppyy_kit] %s JIT-compiled a call wrapper on first use (%.0f ms). "
                    "Call %s once during init/startup to move this one-time cost off the "
                    "first live call. Silence: RCLCPPYY_JIT_NOTICE=0.\n"
                    % (label, elapsed_ms, warmup_hint))


def warmup(*thunks):
    """Run each zero-arg ``thunk`` once with the first-use notice suppressed, so a
    kit can front-load its per-signature JIT during init. A kit's ``warmup()``
    passes thunks that exercise its expensive entry points on throwaway objects;
    the JIT'd wrappers are cached process-globally, so later live calls are fast.
    Building block only -- what to exercise is kit-specific (see bt_kit.warmup)."""
    with suppress_first_use_notice():
        for thunk in thunks:
            thunk()


class HandleRegistry:
    """Give each C++ object its own Python peer without crossing ownership.

    Returning a ``std::unique_ptr<T>`` *from* a Python ``std::function`` fails, so
    to let C++ create per-instance Python state you have C++ call a builder that
    returns an integer handle (``add``), then dispatch later callbacks by that
    handle (``get``). Used by bt_kit's per-tree-node stateful builder.
    """

    def __init__(self):
        self._by_handle = {}

    def add(self, obj):
        handle = len(self._by_handle)
        self._by_handle[handle] = obj
        return handle

    def get(self, handle):
        return self._by_handle[handle]

    def __len__(self):
        return len(self._by_handle)


def unwrap_expected(expected, default=None):
    """Value of a BT/PCL-style ``Expected<T>``/optional (``has_value()`` /
    ``value()``), or ``default`` when empty."""
    return expected.value() if expected.has_value() else default


_PROBE_TEMPLATE = """\
import cppyy
{setup}
cppyy.cppdef({code!r})
print("CPPYY_KIT_CPPDEF_OK")
"""


def probe_cppdef(code, include_paths=(), library_paths=(), headers=(), libraries=()):
    """Compile ``code`` with ``cppyy.cppdef`` in a throwaway subprocess; return
    ``(ok, message)``.

    A ``cppdef`` that fails to parse can crash the interpreter during transaction
    revert (with no Python traceback), so risky glue should be probed
    out-of-process before it is run for real in-process. The subprocess first
    replicates the given include paths, headers and libraries.
    """
    setup = []
    for path in include_paths:
        setup.append("cppyy.add_include_path(%r)" % path)
    for path in library_paths:
        setup.append("cppyy.add_library_path(%r)" % path)
    for header in headers:
        setup.append("cppyy.include(%r)" % header)
    for lib in libraries:
        setup.append("cppyy.load_library(%r)" % lib)
    script = _PROBE_TEMPLATE.format(setup="\n".join(setup), code=code)
    proc = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    if proc.returncode == 0 and "CPPYY_KIT_CPPDEF_OK" in proc.stdout:
        return True, "ok"
    return False, (proc.stderr.strip() or proc.stdout.strip()
                   or "cppdef crashed (returncode %d)" % proc.returncode)


# --- Ordered teardown -----------------------------------------------------
# A cppyy process mixes two teardown mechanisms with no ordering contract
# between them: Python's finalization (which clears module globals, dropping the
# last references to cppyy-proxied C++ objects and running their destructors)
# and cppyy's own atexit hook (which tears down Cling / the JIT). A C++ object
# that owns *process-global or static* state -- an rclcpp Context, the DDS
# participant it owns and its background threads, a ZMQ-backed BT logger -- is
# the dangerous case: if its destructor runs after Cling is gone (or a DDS
# thread touches freed state), the process can SIGSEGV with no Python traceback,
# after all useful work is done. The historical rclcppyy "teardown wart" (and
# the ``os._exit`` dodges that papered over it) is exactly this class.
#
# The fix is to release those resources at a *defined* point while both Python
# and cppyy are still healthy: register a teardown callback here, and it runs at
# ``shutdown()``. ``shutdown()`` is also wired to ``atexit`` -- Python runs
# atexit callbacks after ``main`` returns but before it clears module globals or
# runs cppyy's (earlier-registered, so later-running) Cling teardown, which is
# the correct window. Callbacks run LIFO (foundation registered first is torn
# down last) and best-effort; ``shutdown`` is idempotent, so an explicit call
# and the atexit backstop cannot double-run it.
_TEARDOWN = []
_SHUTDOWN_DONE = False


def register_teardown(callback):
    """Register ``callback`` (a zero-arg callable) to run at ``shutdown()`` /
    interpreter exit, releasing a C++ resource before Python finalization. A
    given callable is registered at most once; the callback should itself be
    safe to call a single time (see ``rclcppyy.shutdown_rclcpp`` for the
    guarded-once pattern)."""
    if callback not in _TEARDOWN:
        _TEARDOWN.append(callback)


def shutdown():
    """Run every registered teardown callback once, in reverse registration
    order. Idempotent and best-effort: a raising callback is swallowed (we are
    already tearing the process down and want the remaining callbacks to run).
    Called automatically at ``atexit``; a demo or test may also call it
    explicitly (e.g. before re-init in one process)."""
    global _SHUTDOWN_DONE
    if _SHUTDOWN_DONE:
        return
    _SHUTDOWN_DONE = True
    while _TEARDOWN:
        callback = _TEARDOWN.pop()
        try:
            callback()
        except Exception:
            pass


atexit.register(shutdown)
