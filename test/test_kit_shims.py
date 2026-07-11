#!/usr/bin/env python3
"""Shim contract for ``rclcppyy.kits.*`` (M3).

The cppyy kits were carved out of rclcppyy into standalone packages (the cppyy_kit
repo). ``rclcppyy.kits.<kit>`` are now deprecation shims that re-export the moved
package and emit a ``DeprecationWarning``. The *full* kit test suites live in the
cppyy_kit repo; this only proves the shim contract in rclcppyy:

* a shim whose target package IS installed here (cppyy_kit, via the cppyy-kit dep)
  re-exports it identically and warns;
* a shim whose target is NOT installed here (the domain kits) raises a clear
  ``ImportError`` naming the package to install, not a bare ModuleNotFoundError.

Each shim module is imported at most once per session (import caching), so the
warning is asserted on that first import.
"""
import importlib

import pytest

# (rclcppyy.kits submodule, top-level import target it re-exports)
INSTALLED = [("cppyy_kit", "cppyy_kit"), ("freeze", "cppyy_kit.freeze")]
DOMAIN = ["bt_kit", "pcl_kit", "ompl_kit", "nav2_kit", "moveit_kit",
          "control_kit", "cv_kit", "dbow_kit"]


def _target_installed(target):
    try:
        importlib.import_module(target)
    except ImportError:
        return False
    return True


@pytest.mark.parametrize("shim,target", INSTALLED)
def test_installed_shim_reexports_and_warns(shim, target):
    if not _target_installed(target):
        pytest.skip(f"{target} not installed in this env")
    with pytest.warns(DeprecationWarning):
        mod = importlib.import_module(f"rclcppyy.kits.{shim}")
    tgt = importlib.import_module(target)
    # A representative public attribute is re-exported as the very same object.
    name = next(a for a in dir(tgt)
                if not a.startswith("_") and hasattr(mod, a))
    assert getattr(mod, name) is getattr(tgt, name)


@pytest.mark.parametrize("shim", DOMAIN)
def test_uninstalled_shim_gives_clear_hint(shim):
    if _target_installed(shim):
        pytest.skip(f"{shim} is installed here; the clear-hint path is not exercised")
    # Importing the shim must raise a clear ImportError that names the missing
    # package (the shim converts the underlying ModuleNotFoundError into a hint).
    with pytest.raises(ImportError) as excinfo:
        importlib.import_module(f"rclcppyy.kits.{shim}")
    assert shim in str(excinfo.value)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
