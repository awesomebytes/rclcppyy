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
  * crossing functions both ways -- wrap a Python callable as ``std::function``
    (``std_function``); give a C++ object a per-instance Python peer through an
    integer handle when ownership can't cross (``HandleRegistry``);
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
import subprocess
import sys

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
    """Wrap a Python callable as ``std::function<signature>`` to pass into C++.

    The callback runs in whatever C++ thread invokes it (cppyy takes the GIL);
    for a single-threaded driver there is no contention. cppyy does not keep the
    callable alive on its own -- ``keep_alive`` both it and ``pyfunc`` on the
    object that will store it.
    """
    return cppyy.gbl.std.function[signature](pyfunc)


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
