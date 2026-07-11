#!/usr/bin/env python
"""
Benchmark the honest cost of a Python state-validity checker in OMPL's hot loop.

Same 2D planning problem as d01 (unit square, circular obstacle, RRTConnect),
three ways to answer "is this state valid?" -- the check a sampling planner calls
thousands of times per solve:

  (a) callback     -- a Python function via ompl_kit.validity_checker
                      (std::function<bool(const State*)> wrapping Python)
  (b) crossinherit -- a Python class deriving ob.StateValidityChecker, isValid
                      overridden in Python (C++ vtable dispatch -> Python)
  (c) cpp          -- a native C++ StateValidityChecker, JIT-compiled once
                      (the "lowered" checker; the call never leaves C++)

For each variant we report the real solve (time + validity-call count on a fixed
seed, so all three explore identical geometry) AND a microbenchmark that isolates
the per-call boundary cost (N direct isValid calls from a C++ driver loop). The
microbenchmark is the honest "Python in a real hot loop" number.

Each variant runs in a fresh subprocess: OMPL's global RNG can only be seeded once
per process, so a shared process could not give all three the same geometry.

Run: pixi run -e ompl bench-ompl
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))

VARIANTS = [
    ("callback", "Python fn via callback()"),
    ("crossinherit", "Python subclass (cross-inherit)"),
    ("cpp", "native C++ (JIT'd, lowered)"),
]


# --------------------------------------------------------------------------
# Worker: run ONE variant in this fresh process and print a JSON result line.
# --------------------------------------------------------------------------
def worker(kind, seed, micro_n):
    import cppyy
    from rclcppyy.kits import ompl_kit, cppyy_kit

    ob, og = ompl_kit.bringup_ompl()

    # Native C++ checker (variant c) + microbench drivers, JIT-compiled once.
    cppyy.cppdef(r"""
    namespace ompl_bench {
    using ompl::base::State;
    using ompl::base::RealVectorStateSpace;
    inline bool circle_valid(const State* s) {
      const auto* rv = s->as<RealVectorStateSpace::StateType>();
      double x = (*rv)[0], y = (*rv)[1];
      return (x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) > 0.25*0.25;
    }
    class CppCircleChecker : public ompl::base::StateValidityChecker {
    public:
      CppCircleChecker(const ompl::base::SpaceInformationPtr& si)
        : ompl::base::StateValidityChecker(si) {}
      bool isValid(const State* s) const override { return circle_valid(s); }
    };
    long drive_fn(const std::function<bool(const State*)>& fn, const State* s, long n) {
      long t = 0; for (long i=0;i<n;++i) t += fn(s) ? 1 : 0; return t;
    }
    long drive_checker(const ompl::base::StateValidityChecker* c, const State* s, long n) {
      long t = 0; for (long i=0;i<n;++i) t += c->isValid(s) ? 1 : 0; return t;
    }
    }  // namespace ompl_bench
    """)
    bench = cppyy.gbl.ompl_bench

    def circle(rv):
        return (rv[0] - 0.5) ** 2 + (rv[1] - 0.5) ** 2 > 0.25 ** 2

    counter = {"n": 0}

    def py_valid(state):
        counter["n"] += 1
        return circle(state)

    class PyChecker(ob.StateValidityChecker):
        def __init__(self, si):
            super().__init__(si)
            self.calls = 0

        def isValid(self, state):
            self.calls += 1
            return circle(state)

    def make_setup():
        space = ob.RealVectorStateSpace(2)
        b = ob.RealVectorBounds(2)
        b.setLow(0.0)
        b.setHigh(1.0)
        space.setBounds(b)
        ss = og.SimpleSetup(ob.StateSpacePtr(space))
        ScopedState = ob.ScopedState[ob.RealVectorStateSpace]
        st = ScopedState(ss.getStateSpace())
        st[0], st[1] = 0.1, 0.1
        gl = ScopedState(ss.getStateSpace())
        gl[0], gl[1] = 0.9, 0.9
        ss.setStartAndGoalStates(st, gl)
        return ss

    ompl_kit.set_seed(seed)
    ss = make_setup()
    si = ss.getSpaceInformation()

    # Hold the checker object; attach to variant.
    holder = {}
    if kind == "callback":
        ss.setStateValidityChecker(ompl_kit.validity_checker(py_valid, owner=ss))
    elif kind == "crossinherit":
        chk = PyChecker(si)
        cppyy_kit.keep_alive(ss, chk)
        ss.setStateValidityChecker(ob.StateValidityCheckerPtr(chk))
        holder["chk"] = chk
    elif kind == "cpp":
        chk = bench.CppCircleChecker(si)
        cppyy_kit.keep_alive(ss, chk)
        ss.setStateValidityChecker(ob.StateValidityCheckerPtr(chk))
    ss.setPlanner(ob.PlannerPtr(og.RRTConnect(si)))

    # --- microbench: warm the dispatch path, then time N direct calls -------
    st = si.allocState()
    rv0 = ompl_kit.as_state(st, ob.RealVectorStateSpace.StateType)
    rv0[0], rv0[1] = 0.1, 0.1
    if kind == "callback":
        fn = ompl_kit.validity_checker(py_valid)
        bench.drive_fn(fn, st, 1000)                       # warm (JIT thunk)
        counter["n"] = 0
        t = time.perf_counter()
        bench.drive_fn(fn, st, micro_n)
        micro_dt = time.perf_counter() - t
    else:
        obj = holder.get("chk") or bench.CppCircleChecker(si)
        bench.drive_checker(obj, st, 1000)                 # warm
        t = time.perf_counter()
        bench.drive_checker(obj, st, micro_n)
        micro_dt = time.perf_counter() - t
    si.freeState(st)

    # --- real solve: fixed seed -> identical geometry across variants -------
    counter["n"] = 0
    if kind == "crossinherit":
        holder["chk"].calls = 0
    t = time.perf_counter()
    solved = bool(ss.solve(5.0))
    solve_dt = time.perf_counter() - t
    if kind == "callback":
        calls = counter["n"]
    elif kind == "crossinherit":
        calls = holder["chk"].calls
    else:
        calls = None  # native: not counted from Python
    length = round(ss.getSolutionPath().length(), 4) if solved else None

    print("RESULT " + json.dumps({
        "kind": kind, "solved": solved, "solve_ms": solve_dt * 1000,
        "calls": calls, "length": length,
        "micro_n": micro_n, "micro_ms": micro_dt * 1000,
        "ns_per_call": micro_dt / micro_n * 1e9,
        "calls_per_s": micro_n / micro_dt,
    }))


# --------------------------------------------------------------------------
# Runner: spawn one worker per variant, collect, print table.
# --------------------------------------------------------------------------
def run(args):
    print(f"OMPL validity-checker bench: same 2D problem, seed={args.seed}, "
          f"microbench N={args.micro_n:,}.", file=sys.stderr, flush=True)
    print("(Shared machine: treat as provisional, directional not exact.)",
          file=sys.stderr, flush=True)
    results = {}
    for kind, _label in VARIANTS:
        argv = [sys.executable, "-u", os.path.abspath(__file__),
                "--worker", kind, "--seed", str(args.seed),
                "--micro-n", str(args.micro_n)]
        print(f"  [{kind}] running ...", file=sys.stderr, flush=True)
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=args.timeout)
        line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT ")), None)
        if line is None:
            print(f"  FAILED ({kind}):\n{proc.stdout[-2000:]}\n{proc.stderr[-2000:]}",
                  file=sys.stderr, flush=True)
            continue
        results[kind] = json.loads(line[len("RESULT "):])
    print_table(results, args)
    return 0 if len(results) == len(VARIANTS) else 1


def print_table(results, args):
    cols = ["validity variant", "solve ms", "valid calls", "path len",
            "ns/call*", "calls/s*"]
    widths = [34, 10, 12, 10, 11, 16]
    total = sum(widths)
    print()
    print("  OMPL state-validity checker: Python in the hot loop vs native C++")
    print("  (same 2D plan around a circular obstacle, RRTConnect, fixed seed)")
    print("  " + "=" * total)
    print("  " + "".join(f"{c:<{w}}" for c, w in zip(cols, widths)))
    print("  " + "-" * total)
    for kind, label in VARIANTS:
        r = results.get(kind)
        if not r:
            print(f"  {label:<34}{'FAILED':<10}")
            continue
        calls = "n/a" if r["calls"] is None else f"{r['calls']:,}"
        cells = [label, f"{r['solve_ms']:.2f}", calls, f"{r['length']}",
                 f"{r['ns_per_call']:.1f}", f"{r['calls_per_s']:,.0f}"]
        print("  " + "".join(f"{str(c):<{w}}" for c, w in zip(cells, widths)))
    print("  " + "-" * total)
    print("  * ns/call and calls/s are the microbenchmark (N direct isValid calls")
    print("    from a C++ driver loop) -- the pure Python<->C++ boundary cost.")
    print("    solve ms includes all planner overhead, not just validity.")
    a = results.get("callback")
    c = results.get("cpp")
    if a and c and c["ns_per_call"]:
        print(f"\n  Python validity is ~{a['ns_per_call'] / c['ns_per_call']:.0f}x slower "
              f"per call than native C++ ({a['ns_per_call']:.0f} ns vs "
              f"{c['ns_per_call']:.1f} ns).")
        print("  Lowering story: prototype the checker in Python (a/b), then lower the")
        print("  hot checker to a JIT'd C++ StateValidityChecker (c) for production.")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--worker", choices=[k for k, _ in VARIANTS], default=None)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--micro-n", type=int, default=2_000_000)
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    if args.worker:
        worker(args.worker, args.seed, args.micro_n)
        return 0
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
