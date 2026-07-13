"""Install / uninstall the opt-in RCLCPPYY_ENABLE_HOOK startup hook.

``enable_cpp_acceleration()`` normally has to be called from inside a process. To
accelerate a process you cannot edit -- notably the stock ``ros2`` CLI verbs
(``ros2 topic hz`` and friends) -- rclcppyy can install a startup ``.pth`` into the
environment's site-packages. The ``.pth`` runs at every interpreter start and calls
``_hook_boot.activate()``, which does nothing unless ``RCLCPPYY_ENABLE_HOOK=1`` is
set; when it is, it arranges for ``enable_cpp_acceleration()`` to run the moment
``rclpy`` is imported. So the workflow is:

    python -m rclcppyy.hook install             # once per environment
    RCLCPPYY_ENABLE_HOOK=1 ros2 topic hz /some_topic    # accelerated, zero code changes
    ros2 topic hz /some_topic                     # unchanged: env var unset -> stock

    python -m rclcppyy.hook status
    python -m rclcppyy.hook uninstall

The single control is ``RCLCPPYY_ENABLE_HOOK``: exactly ``"1"`` turns the hook on
for a process; unset, ``"0"``, or any other value leaves it off. The ``.pth`` itself
is silent and near-zero-cost when the hook is off, and is fully uninstallable.
"""
import argparse
import os
import sys
import sysconfig

from rclcppyy import _hook_boot as _boot

# The installed boot copy is a top-level module (``_rclcppyy_hook``) so it needs no
# package on sys.path at interpreter start. The .pth's one line imports it and calls
# activate(), fully guarded so a broken/absent copy can never crash or spam a
# traceback into an interpreter start.
_PTH_NAME = "rclcppyy_hook.pth"
_BOOT_INSTALLED_NAME = "_rclcppyy_hook.py"
_PTH_LINE = (
    'import sys; exec('
    '"try:\\n import _rclcppyy_hook as _m; _m.activate()\\n'
    'except Exception: pass")\n'
)


def _site_dir():
    return sysconfig.get_path("purelib")


def _boot_source():
    """Canonical text of the boot module, copied verbatim into site-packages."""
    with open(_boot.__file__) as f:
        return f.read()


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def _atomic_write(path, text):
    tmp = path + ".tmp.%d" % os.getpid()
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def install(site_dir=None):
    """Install (or refresh) the ``.pth`` + boot module in the env's site-packages.

    Idempotent: rewrites only what is stale. Returns the two written paths.
    """
    site = site_dir or _site_dir()
    if not site or not os.path.isdir(site):
        raise RuntimeError("site-packages dir not found: %r" % site)
    boot = os.path.join(site, _BOOT_INSTALLED_NAME)
    pth = os.path.join(site, _PTH_NAME)
    src = _boot_source()
    if _read(boot) != src:
        _atomic_write(boot, src)
    if _read(pth) != _PTH_LINE:
        _atomic_write(pth, _PTH_LINE)
    return {"pth": pth, "boot": boot}


def uninstall(site_dir=None):
    """Remove the ``.pth`` + boot module. Returns the list of removed paths."""
    site = site_dir or _site_dir()
    removed = []
    for name in (_PTH_NAME, _BOOT_INSTALLED_NAME):
        p = os.path.join(site, name)
        if os.path.exists(p):
            try:
                os.unlink(p)
                removed.append(p)
            except OSError:
                pass
    return removed


def is_installed(site_dir=None):
    """Whether both the ``.pth`` and boot module are present and current."""
    site = site_dir or _site_dir()
    pth = os.path.join(site, _PTH_NAME)
    boot = os.path.join(site, _BOOT_INSTALLED_NAME)
    return _read(pth) == _PTH_LINE and _read(boot) == _boot_source()


def status(site_dir=None):
    site = site_dir or _site_dir()
    return {
        "site_dir": site,
        "installed": is_installed(site),
        "RCLCPPYY_ENABLE_HOOK": os.environ.get(_boot.ENABLE_ENV),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python -m rclcppyy.hook",
        description="Install the opt-in RCLCPPYY_ENABLE_HOOK startup hook so stock ros2 CLI "
                    "tools accelerate when RCLCPPYY_ENABLE_HOOK=1 is set.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("install", help="install the .pth + boot module into site-packages")
    sub.add_parser("uninstall", help="remove the .pth + boot module")
    sub.add_parser("status", help="show whether the hook is installed and the env var")
    args = parser.parse_args(argv)

    if args.cmd == "install":
        paths = install()
        print("Installed rclcppyy startup hook:")
        print("  %s" % paths["pth"])
        print("  %s" % paths["boot"])
        print("Enable per process with RCLCPPYY_ENABLE_HOOK=1.")
    elif args.cmd == "uninstall":
        removed = uninstall()
        if removed:
            print("Removed:")
            for p in removed:
                print("  %s" % p)
        else:
            print("Nothing to remove (hook not installed).")
    elif args.cmd == "status":
        st = status()
        print("site-packages       : %s" % st["site_dir"])
        print("installed           : %s" % st["installed"])
        print("RCLCPPYY_ENABLE_HOOK : %s" % (st["RCLCPPYY_ENABLE_HOOK"] or "<unset>"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
