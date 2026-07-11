#!/usr/bin/env python
"""
Run a Python command with a kit's frozen PCH active (L0->L1 freeze launcher).

``CLING_STANDARD_PCH`` must be set before the interpreter's first ``import cppyy``
(which ``import rclcppyy`` triggers). This wrapper resolves the frozen artifact
*without* importing rclcppyy/cppyy, sets the environment, and then ``exec``'s the
requested command in the SAME process image -- so the very first cppyy import in
the target already sees the frozen PCH.

Usage::

    python scripts/freeze/run_frozen.py <script.py> [args...]
    python scripts/freeze/run_frozen.py -m pytest test/test_bt_kit.py -v

The kit defaults to ``bt`` (override with ``RCLCPPYY_FREEZE_KIT``). If the artifact
is missing it prints how to build it and runs unfrozen (JIT), so the command still
works -- just without the speedup.
"""
import importlib.util
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_spec = importlib.util.spec_from_file_location(
    "_rclcppyy_freeze", os.path.join(_REPO, "rclcppyy", "kits", "freeze.py"))
freeze = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(freeze)


def main(argv):
    if not argv:
        sys.exit("usage: python scripts/freeze/run_frozen.py <script.py|-m module> [args...]")
    kit = os.environ.get("RCLCPPYY_FREEZE_KIT", "bt")
    pch = freeze.artifact_path(kit)
    if os.path.exists(pch):
        os.environ["CLING_STANDARD_PCH"] = pch
        os.environ["RCLCPPYY_FROZEN"] = "1"
        sys.stderr.write("[run_frozen] %s_kit frozen PCH active: %s\n" % (kit, pch))
    else:
        sys.stderr.write(
            "[run_frozen] no frozen PCH for %s_kit at %s\n"
            "             build it with `pixi run -e %s freeze-%s-build`; running JIT.\n"
            % (kit, pch, kit, kit))
    os.execv(sys.executable, [sys.executable] + list(argv))


if __name__ == "__main__":
    main(sys.argv[1:])
