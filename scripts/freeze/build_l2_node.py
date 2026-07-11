#!/usr/bin/env python
"""
Compile the L2 native ApproachObject node into a BehaviorTree.CPP plugin .so.

This is the L2 rung of the lowering cycle: a leaf that was Python (L0) is emitted
as native C++ (l2_approach_object.cpp) and built into a plugin the engine loads
with registerFromPlugin() -- no cppyy/JIT in the hot path. Output goes to the
gitignored build/freeze/ dir. Run inside the bt env::

    pixi run -e bt freeze-l2-build

Uses the conda env's C++ compiler ($CXX) and links against the installed
libbehaviortree_cpp.
"""
import os
import subprocess
import sys

from ament_index_python.packages import get_package_prefix

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SRC = os.path.join(HERE, "l2_approach_object.cpp")
OUT = os.path.join(REPO, "build", "freeze", "libl2_approach_object.so")


def main():
    prefix = get_package_prefix("behaviortree_cpp")
    inc = os.path.join(prefix, "include")
    lib = os.path.join(prefix, "lib")
    cxx = os.environ.get("CXX") or "c++"
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    # BT_PLUGIN_EXPORT makes BT_REGISTER_NODES export BT_RegisterNodesFromPlugin
    # (else the macro falls back to `static` and the engine can't find it).
    cmd = [cxx, "-shared", "-fPIC", "-std=c++17", "-O2", "-DBT_PLUGIN_EXPORT",
           "-I" + inc, SRC, "-o", OUT,
           "-L" + lib, "-lbehaviortree_cpp", "-Wl,-rpath," + lib]
    print("Compiling L2 node:\n  " + " ".join(cmd))
    ret = subprocess.call(cmd)
    if ret != 0:
        sys.exit("ERROR: compile failed (returncode %d)" % ret)
    print("OK  ->  %s  (%.0f KB)" % (OUT, os.path.getsize(OUT) / 1024))
    return OUT


if __name__ == "__main__":
    main()
