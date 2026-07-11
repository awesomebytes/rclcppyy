#!/usr/bin/env python
"""
Build the frozen Cling PCH for bt_kit (L0->L1 freeze).

Bakes ``behaviortree_cpp/bt_factory.h`` on top of cppyy's standard-header set into
a single precompiled header, so a frozen process loads the AST at interpreter
start instead of JIT-parsing the header (~0.83 s) on first use.

This mirrors cppyy's own ``etc/dictpch/makepch.py`` (``rootcling -generate-pch``
over ``allHeaders.h`` + ``allLinkDefs.h`` with the env's ``allCppflags.txt``) but
inserts the kit header and its include path. The artifact is written, gitignored,
under ``<repo>/build/freeze`` with an env-version tag; rebuild it whenever the
cppyy-cling or behaviortree_cpp version changes (a tag mismatch makes that
obvious). Run inside the bt env::

    pixi run -e bt freeze-bt-build

Artifacts are Cling-version-specific and must never be committed.
"""
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile

import cppyy_backend
from ament_index_python.packages import get_package_prefix

# Load rclcppyy.kits.freeze by file path (avoid importing the rclcppyy package,
# which would pull in cppyy/ROS just to resolve the artifact path).
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_spec = importlib.util.spec_from_file_location(
    "_rclcppyy_freeze", os.path.join(_REPO, "rclcppyy", "kits", "freeze.py"))
freeze = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(freeze)


def _cppflags(cppflags_file, bt_inc):
    """The env's standard cppflags (same filtering makepch.py applies), plus the
    kit's include dir so rootcling can find the extra header."""
    with open(cppflags_file) as f:
        lines = f.readlines()
    for drop in ("-fno-plt\n",):
        if drop in lines:
            lines.remove(drop)
    flags = [ln[:-1].strip() for ln in lines]
    if "-isystem" in flags:                     # bare token, path is separate
        flags.remove("-isystem")
    flags.append("-I" + bt_inc)
    return flags


def build_pch(kit="bt", extra_headers=("behaviortree_cpp/bt_factory.h",),
              package="behaviortree_cpp", out_path=None):
    be = os.path.dirname(cppyy_backend.__file__)
    cfgdir = os.path.join("etc", "dictpch")
    all_headers = os.path.join(cfgdir, "allHeaders.h")
    all_linkdefs = os.path.join(cfgdir, "allLinkDefs.h")
    cppflags_file = os.path.join(be, cfgdir, "allCppflags.txt")
    if not os.path.exists(os.path.join(be, all_headers)):
        sys.exit("ERROR: cppyy_backend dictpch machinery not found at %s" % be)

    pkg_inc = os.path.join(get_package_prefix(package), "include")
    out_path = out_path or freeze.artifact_path(kit)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    macros = [
        "-D__CLING__", "-DROOT_PCH",
        "-I" + os.path.join(be, "include"),
        "-I" + os.path.join(be, "etc"),
        "-I" + os.path.join(be, cfgdir),
        "-I" + os.path.join(be, "etc", "cling"),
        "-I" + pkg_inc,
    ]
    rootcling = os.path.join(be, "bin", "rootcling")
    tmpdir = tempfile.mkdtemp()
    outf = os.path.join(tmpdir, "allDict.cxx")
    cmd = [rootcling, "-rootbuild", "-generate-pch", "-f", outf, "-noDictSelection"]
    cmd += macros
    cmd += ["-cxxflags", " ".join(_cppflags(cppflags_file, pkg_inc))]
    cmd += [all_headers] + list(extra_headers) + [all_linkdefs]

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = os.path.join(be, "lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    if "VERBOSE" in os.environ:
        print("cwd:", be)
        print("cmd:", " ".join(cmd))
    print("Building frozen PCH for %s_kit (headers: %s) ..." % (kit, ", ".join(extra_headers)))
    ret = subprocess.call(cmd, cwd=be, env=env)
    try:
        if ret == 0:
            shutil.move(os.path.join(tmpdir, "allDict_rdict.pch"), out_path)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)
    if ret != 0:
        sys.exit("ERROR: rootcling failed (returncode %d)" % ret)
    size_mb = os.path.getsize(out_path) / 1e6
    print("OK  ->  %s  (%.1f MB, tag %s)" % (out_path, size_mb, freeze.version_tag()))
    print("Activate with: RCLCPPYY_FROZEN=1 python scripts/freeze/run_frozen.py <script.py>")
    return out_path


if __name__ == "__main__":
    build_pch()
