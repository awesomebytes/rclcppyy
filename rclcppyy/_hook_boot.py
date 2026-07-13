"""Standalone bootstrap for rclcppyy's startup hook (RCLCPPYY_ENABLE_HOOK).

A copy of this file is installed into the active environment's site-packages (as
``_rclcppyy_hook.py``) next to a ``.pth`` whose single line calls ``activate()`` at
**every interpreter start, before any user import**. That is what lets a process
which never mentions rclcppyy -- the stock ``ros2`` CLI, for example -- pick up the
C++ backend: with ``RCLCPPYY_ENABLE_HOOK=1`` set, ``activate()`` arranges for
``rclcppyy.enable_cpp_acceleration()`` to run the moment ``rclpy`` is imported, so
``rclpy.create_node`` / ``spin`` / ``spin_once`` are already patched before the tool
builds its node.

The SAME module is imported by ``rclcppyy.hook`` as the single source of the
install-path and activation logic. Two hard constraints follow (mirroring
cppyy_kit's auto-PCH bootstrap):

  * **stdlib only, no package-relative imports** -- the installed copy is a
    top-level module and this runs at interpreter startup for *every* program in
    the env;
  * **never raises** -- a bootstrap that threw would print a traceback on every
    ``python`` invocation. ``activate()`` swallows everything.

The single control is ``RCLCPPYY_ENABLE_HOOK``: exactly ``"1"`` turns the hook on;
unset, ``"0"``, or any other value leaves it off, so ``activate()`` returns
immediately (before importing anything) and the cost for unrelated processes is
negligible.
"""
import os
import sys

ENABLE_ENV = "RCLCPPYY_ENABLE_HOOK"

# Per-process guard. A module global (not an env var) so it never leaks into child
# processes -- each interpreter decides for itself whether to accelerate.
_activated = False


def _enabled():
    """Whether the hook is requested for this process (only ``"1"`` enables)."""
    return os.environ.get(ENABLE_ENV) == "1"


def _accelerate():
    """Apply enable_cpp_acceleration() exactly once. Best-effort; never raises."""
    global _activated
    if _activated:
        return
    _activated = True
    try:
        import rclcppyy
        rclcppyy.enable_cpp_acceleration()
    except Exception as exc:  # pragma: no cover - defensive
        sys.stderr.write("rclcppyy auto-acceleration failed (%s); "
                         "running on stock rclpy.\n" % exc)


def _make_finder():
    """A one-shot meta-path finder that accelerates right after ``rclpy`` loads."""
    import importlib.abc
    import importlib.util

    class _RclpyAccelFinder(importlib.abc.MetaPathFinder):
        armed = True

        def find_spec(self, fullname, path, target=None):
            if fullname != "rclpy" or not _RclpyAccelFinder.armed:
                return None
            # Disarm before resolving the real spec so this finder returns None on
            # the reentrant find_spec below (no recursion), and only ever wraps the
            # first import of the top-level rclpy package.
            _RclpyAccelFinder.armed = False
            real = importlib.util.find_spec("rclpy")
            if real is None or real.loader is None:
                return None
            orig_exec = real.loader.exec_module

            def exec_module(module):
                orig_exec(module)
                _accelerate()

            try:
                real.loader.exec_module = exec_module
            except Exception:
                return None  # loader won't allow the wrap; leave stock rclpy alone
            return real

    return _RclpyAccelFinder()


def activate():
    """Entry point invoked by the startup ``.pth``.

    If auto-acceleration is enabled, either accelerate now (rclpy already imported)
    or install a hook that accelerates the instant rclpy is imported. Never raises.
    """
    try:
        if not _enabled() or _activated:
            return
        if "rclpy" in sys.modules:
            _accelerate()
            return
        sys.meta_path.insert(0, _make_finder())
    except Exception:
        pass
