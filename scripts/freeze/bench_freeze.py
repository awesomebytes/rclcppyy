#!/usr/bin/env python
"""
L0 (JIT) vs L1 (frozen PCH) bringup benchmark for bt_kit.

Spawns a fresh subprocess per sample (bringup cost is a once-per-process startup
cost, so it must be measured cold), with and without the frozen PCH active, and
reports the median over N samples. Three things are measured:

  * staged bringup wall time -- the same stages as docs/bt_kit/REPORT.md, so the
    ~0.83 s header parse that the freeze removes is visible on its own line;
  * time-to-first-tree-tick -- build the t01 tree and run one tickWhileRunning();
  * total t01 demo runtime -- wall clock of the whole t01_first_tree.py process
    (Python start -> exit), the number a user actually feels.

Run inside the bt env (build the PCH first with `freeze-bt-build`)::

    pixi run -e bt freeze-bench            # default 5 samples

The machine may be shared; medians over several samples keep it honest.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
T01 = os.path.join(REPO, "scripts", "bt_kit_demos", "t01_first_tree.py")
STAGES = ["import cppyy+init", "add_include_path", "include(bt_factory.h)",
          "load_library", "cppdef(glue)", "first factory+tick"]


# ---- worker: one cold staged measurement, prints JSON ----------------------
def _worker():
    t = {}

    def stage(name, fn):
        s = time.perf_counter()
        r = fn()
        t[name] = time.perf_counter() - s
        return r

    import cppyy
    from rclcppyy.kits import bt_kit, freeze
    from ament_index_python.packages import get_package_prefix
    prefix = get_package_prefix("behaviortree_cpp")

    stage("import cppyy+init", lambda: cppyy.gbl)
    stage("add_include_path", lambda: cppyy.add_include_path(os.path.join(prefix, "include")))
    stage("include(bt_factory.h)", lambda: cppyy.include("behaviortree_cpp/bt_factory.h"))

    def _load():
        cppyy.add_library_path(os.path.join(prefix, "lib"))
        cppyy.load_library("libbehaviortree_cpp.so")
    stage("load_library", _load)

    def _glue():
        freeze.apply_force_symbols("bt")
        cppyy.cppdef(bt_kit._CPP_GLUE)
    stage("cppdef(glue)", _glue)

    BT = cppyy.gbl.BT
    bt_kit._STATUS.update({bt_kit.IDLE: BT.NodeStatus.IDLE, bt_kit.RUNNING: BT.NodeStatus.RUNNING,
                           bt_kit.SUCCESS: BT.NodeStatus.SUCCESS, bt_kit.FAILURE: BT.NodeStatus.FAILURE,
                           bt_kit.SKIPPED: BT.NodeStatus.SKIPPED})
    bt_kit._adapt_factory(BT)
    bt_kit._BT = BT
    bt_kit._BRINGUP_DONE = True

    xml = ('<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence>'
           '<CheckBattery/><OpenGripper/><ApproachObject/><CloseGripper/>'
           '</Sequence></BehaviorTree></root>')

    def _first():
        f = BT.BehaviorTreeFactory()
        for n in ("CheckBattery", "OpenGripper", "ApproachObject", "CloseGripper"):
            f.registerSimpleAction(n, (lambda node: bt_kit.SUCCESS))
        return f.create_tree_from_text(xml).tickWhileRunning()
    st = stage("first factory+tick", _first)

    out = {"stages": t, "bringup_total": sum(t[s] for s in STAGES[:-1]),
           "time_to_first_tick": t["first factory+tick"], "status": int(st),
           "frozen": freeze.active("bt")}
    print("JSON:" + json.dumps(out))


# ---- driver ----------------------------------------------------------------
def _run_worker(frozen, pch):
    env = os.environ.copy()
    env.pop("CLING_STANDARD_PCH", None)
    env.pop("RCLCPPYY_FROZEN", None)
    if frozen:
        env["CLING_STANDARD_PCH"] = pch
        env["RCLCPPYY_FROZEN"] = "1"
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker"],
                       capture_output=True, text=True, env=env)
    for line in p.stdout.splitlines():
        if line.startswith("JSON:"):
            return json.loads(line[5:])
    raise RuntimeError("worker failed:\n" + p.stdout + "\n" + p.stderr)


def _time_t01(frozen, pch, samples):
    env = os.environ.copy()
    env.pop("CLING_STANDARD_PCH", None)
    env.pop("RCLCPPYY_FROZEN", None)
    if frozen:
        env["CLING_STANDARD_PCH"] = pch
        env["RCLCPPYY_FROZEN"] = "1"
    out = []
    for _ in range(samples):
        s = time.perf_counter()
        r = subprocess.run([sys.executable, T01], capture_output=True, text=True, env=env)
        out.append(time.perf_counter() - s)
        if r.returncode != 0:
            raise RuntimeError("t01 failed:\n" + r.stderr)
    return statistics.median(out)


def _median_stages(samples_data):
    med = {}
    for s in STAGES:
        med[s] = statistics.median(d["stages"][s] for d in samples_data)
    return med


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("-n", "--samples", type=int, default=5)
    args = ap.parse_args()
    if args.worker:
        return _worker()

    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "_frz", os.path.join(REPO, "rclcppyy", "kits", "freeze.py"))
    frz = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(frz)
    pch = frz.artifact_path("bt")
    if not os.path.exists(pch):
        sys.exit("No frozen PCH at %s -- run `pixi run -e bt freeze-bt-build` first." % pch)

    n = args.samples
    print("bt_kit freeze benchmark -- %d cold samples each, median reported" % n)
    print("PCH: %s\n" % pch)

    l0 = [_run_worker(False, pch) for _ in range(n)]
    l1 = [_run_worker(True, pch) for _ in range(n)]
    assert all(d["status"] == 2 for d in l0 + l1), "a tree did not reach SUCCESS"
    assert all(d["frozen"] for d in l1) and not any(d["frozen"] for d in l0)

    m0, m1 = _median_stages(l0), _median_stages(l1)
    print("%-24s %12s %12s %10s" % ("stage", "L0 JIT (ms)", "L1 frozen(ms)", "speedup"))
    print("-" * 62)
    for s in STAGES:
        a, b = m0[s] * 1e3, m1[s] * 1e3
        sp = ("%.1fx" % (a / b)) if b > 0.05 else "-"
        print("%-24s %12.1f %12.1f %10s" % (s, a, b, sp))
    print("-" * 62)
    b0 = statistics.median(d["bringup_total"] for d in l0) * 1e3
    b1 = statistics.median(d["bringup_total"] for d in l1) * 1e3
    print("%-24s %12.1f %12.1f %9.1fx" % ("bringup total", b0, b1, b0 / b1))
    f0 = statistics.median(d["time_to_first_tick"] for d in l0) * 1e3
    f1 = statistics.median(d["time_to_first_tick"] for d in l1) * 1e3
    print("%-24s %12.1f %12.1f %10s" % ("time-to-first-tick", f0, f1,
                                        ("%.1fx" % (f0 / f1)) if f1 > 0.05 else "-"))

    print("\nEnd-to-end t01_first_tree.py wall time (process start -> exit):")
    w0 = _time_t01(False, pch, n) * 1e3
    w1 = _time_t01(True, pch, n) * 1e3
    print("  L0 JIT     %.0f ms" % w0)
    print("  L1 frozen  %.0f ms   (%.1fx faster, -%.0f ms)" % (w1, w0 / w1, w0 - w1))


if __name__ == "__main__":
    main()
