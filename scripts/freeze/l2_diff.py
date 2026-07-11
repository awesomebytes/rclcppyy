#!/usr/bin/env python
"""
L0 (Python leaf) vs L2 (native C++ plugin node) differential test for t01's
ApproachObject leaf.

The lowering cycle's contract is "same tests, every rung". Here the SAME tree XML
is ticked with ApproachObject as (L0) a Python callback through bt_kit and (L2) a
native SyncActionNode loaded from a compiled plugin .so via registerFromPlugin --
no cppyy/JIT and no Python in the L2 hot path. We assert identical stdout + status
(correctness), then measure the per-tick boundary cost of each (tick rate).

Build the plugin first (`pixi run -e bt freeze-l2-build`), then::

    pixi run -e bt freeze-l2-diff

Correctness is checked in subprocesses so mixed C++/Python stdout is captured
faithfully; the rate benchmark runs in-process.
"""
import argparse
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SO = os.path.join(REPO, "build", "freeze", "libl2_approach_object.so")

# Single-leaf tree; the L0/L2 variants swap only how "ApproachObject" is provided.
XML_PRINT = ('<root BTCPP_format="4"><BehaviorTree ID="M">'
             '<ApproachObject/></BehaviorTree></root>')
XML_SILENT = ('<root BTCPP_format="4"><BehaviorTree ID="M">'
              '<ApproachObjectSilent/></BehaviorTree></root>')


def _l0_factory(bt):
    def approach_object(node):
        print("ApproachObject: approach_object")
        return bt.NodeStatus.SUCCESS
    f = bt.BehaviorTreeFactory()
    f.registerSimpleAction("ApproachObject", approach_object)
    return f


def _l2_factory(bt):
    f = bt.BehaviorTreeFactory()
    f.registerFromPlugin(SO)          # native plugin; no JIT of the node logic
    return f


def _worker(variant):
    from rclcppyy.kits import bt_kit
    bt = bt_kit.bringup_bt()
    factory = _l0_factory(bt) if variant == "l0" else _l2_factory(bt)
    status = factory.create_tree_from_text(XML_PRINT).tickWhileRunning()
    sys.stdout.flush()
    sys.stderr.write("STATUS:%d\n" % int(status))


def _run_variant(variant):
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", variant],
                       capture_output=True, text=True)
    status = None
    for line in p.stderr.splitlines():
        if line.startswith("STATUS:"):
            status = int(line[len("STATUS:"):])
    if status is None:
        raise RuntimeError("worker %s failed:\n%s\n%s" % (variant, p.stdout, p.stderr))
    # keep only the node's own output line(s) (drop any cppyy/pch chatter)
    out = "\n".join(ln for ln in p.stdout.splitlines() if "ApproachObject" in ln)
    return out, status


def _rate(bt, factory, xml, seconds=2.0):
    tree = factory.create_tree_from_text(xml)
    tree.tickOnce()                                  # warm
    ticks, start, deadline = 0, time.perf_counter(), time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        tree.tickOnce()
        ticks += 1
    elapsed = time.perf_counter() - start
    return ticks / elapsed, 1e6 * elapsed / ticks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worker", nargs=1, metavar="VARIANT")
    args, _ = ap.parse_known_args()
    if args.worker:
        return _worker(args.worker[0])

    if not os.path.exists(SO):
        sys.exit("No L2 plugin at %s -- run `pixi run -e bt freeze-l2-build` first." % SO)

    print("L0 (Python leaf) vs L2 (native C++ plugin) -- ApproachObject\n")

    # --- correctness differential (subprocess-captured stdout) ---
    out0, s0 = _run_variant("l0")
    out2, s2 = _run_variant("l2")
    print("differential test (same XML, same node ID 'ApproachObject'):")
    print("  L0 stdout: %r  status=%d" % (out0, s0))
    print("  L2 stdout: %r  status=%d" % (out2, s2))
    assert out0 == out2 == "ApproachObject: approach_object", "stdout differs!"
    assert s0 == s2 == 2, "status differs!"
    print("  PASS: identical output and status.\n")

    # --- tick-rate differential (silent nodes, in-process) ---
    from rclcppyy.kits import bt_kit
    bt = bt_kit.bringup_bt()

    def silent(node):
        return bt_kit.SUCCESS
    f0 = bt.BehaviorTreeFactory()
    f0.registerSimpleAction("ApproachObjectSilent", silent)
    r0, us0 = _rate(bt, f0, XML_SILENT)
    r2, us2 = _rate(bt, _l2_factory(bt), XML_SILENT)
    print("tick rate (single-leaf tree, 2 s window, SUCCESS/tick, no I/O):")
    print("  L0 Python leaf via bt_kit   {:>13,.0f} ticks/s   {:>8.3f} us/tick".format(r0, us0))
    print("  L2 native C++ plugin node   {:>13,.0f} ticks/s   {:>8.3f} us/tick".format(r2, us2))
    print("\n  L2 is {:.1f}x the tick rate of L0 ({:.3f} -> {:.3f} us/tick).".format(
        r2 / r0, us0, us2))


if __name__ == "__main__":
    main()
