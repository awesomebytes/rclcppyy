#!/usr/bin/env python3
"""
control_kit bench (Stage 4): the honest real-time verdict.

Runs the in-process controller_manager update loop at a target rate for a fixed
duration and measures the achieved rate, per-cycle jitter percentiles, and missed
(late) cycles -- for a **Python** controller (cross-inherited PD) vs a **stock C++**
controller (forward_command_controller) under the *same* rig and mock hardware. Also
isolates the pure ``cm.update()`` cost (the controller-dispatch hot path: a Python
crossing per cycle vs staying in C++).

The numbers are directional on a shared machine; the point is the *shape*: where a
Python controller sits relative to a C++ one, and at what rate the GIL / interpreter
jitter starts to dominate. See docs/control_kit/REPORT.md §4 for the reading.

Run:  pixi run -e control bench-control
"""
import gc
import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "49")

from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import control_kit as ck  # noqa: E402

JOINTS = ["joint1", "joint2"]
FWD = "forward_command_controller/ForwardCommandController"


class PyPD(ck.ControllerInterface):
    def __init__(self):
        super().__init__()
        self.kp = 5.0
        self.target = [0.3, -0.2]

    def on_init(self):
        return ck.CallbackReturn.SUCCESS

    def command_interface_configuration(self):
        return ck.interface_config(["%s/position" % j for j in JOINTS])

    def state_interface_configuration(self):
        return ck.interface_config(["%s/position" % j for j in JOINTS])

    def on_configure(self, s):
        return ck.CallbackReturn.SUCCESS

    def on_activate(self, s):
        return ck.CallbackReturn.SUCCESS

    def on_deactivate(self, s):
        return ck.CallbackReturn.SUCCESS

    def update(self, time_, period):
        for i in range(ck.n_command_interfaces(self)):
            p = ck.read_state(self, i)
            ck.write_command(self, i, p + self.kp * (self.target[i] - p) * 0.01)
        return ck.return_type.OK


def pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    k = min(len(sorted_vals) - 1, int(round(p / 100.0 * (len(sorted_vals) - 1))))
    return sorted_vals[k]


def measure_update_cost(rig, n=20000):
    """Pure cm.update() cost: no read/write, no sleep -- isolates controller dispatch."""
    import cppyy
    period = cppyy.gbl.rclcpp.Duration.from_seconds(0.001)
    clock = rig.clock
    for _ in range(500):                      # warm the call wrapper
        rig.cm.update(clock.now(), period)
    t0 = time.perf_counter()
    for _ in range(n):
        rig.cm.update(clock.now(), period)
    dt = time.perf_counter() - t0
    return dt / n * 1e6                        # microseconds per update


def run_rate(rig, rate_hz, seconds):
    """Hold ``rate_hz`` for ``seconds`` with sleep scheduling; return interval stats."""
    import cppyy
    dt = 1.0 / rate_hz
    period = cppyy.gbl.rclcpp.Duration.from_seconds(dt)
    clock = rig.clock
    n = int(rate_hz * seconds)
    stamps = []
    start = time.perf_counter()
    for i in range(n):
        rig.cm.read(clock.now(), period)
        rig.cm.update(clock.now(), period)
        rig.cm.write(clock.now(), period)
        stamps.append(time.perf_counter())
        target = start + (i + 1) * dt
        sleep = target - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
    intervals = [(stamps[i] - stamps[i - 1]) * 1e3 for i in range(1, len(stamps))]  # ms
    intervals.sort()
    late = sum(1 for v in intervals if v > 1.5 * dt * 1e3)
    achieved = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    return {
        "rate": rate_hz, "seconds": seconds, "cycles": len(stamps),
        "achieved": achieved, "late": late,
        "p50": pct(intervals, 50), "p95": pct(intervals, 95),
        "p99": pct(intervals, 99), "max": intervals[-1] if intervals else 0.0,
        "target_ms": dt * 1e3,
    }


def bench_controller(label, make_and_activate, rates, seconds):
    rig = make_and_activate()
    upd_us = measure_update_cost(rig)
    print("\n=== %s ===" % label)
    print("  pure cm.update() cost: %.2f us/cycle" % upd_us)
    rows = []
    for r in rates:
        rows.append(run_rate(rig, r, seconds))
    for row in rows:
        print("  %4d Hz | achieved %7.1f Hz | late %4d/%d | "
              "interval ms p50=%.3f p95=%.3f p99=%.3f max=%.3f (target %.3f)"
              % (row["rate"], row["achieved"], row["late"], row["cycles"],
                 row["p50"], row["p95"], row["p99"], row["max"], row["target_ms"]))
    rig._teardown()
    return upd_us, rows


def main():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    ck.bringup_control()
    ck.warmup()

    seconds = float(os.environ.get("BENCH_SECONDS", "8"))
    rates = [100, 1000]

    def cpp_rig():
        rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=1000)
        rig.load_controller("fwd", FWD,
                            parameters={"joints": JOINTS, "interface_name": "position"})
        rig.configure("fwd")
        rig.activate(["fwd"])
        return rig

    def py_rig():
        rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=1000)
        pd = PyPD()
        rig.add_python_controller(pd, "pd")
        rig.configure("pd")
        rig.activate(["pd"])
        return rig

    cpp_us, _ = bench_controller("C++ baseline (forward_command_controller)",
                                 cpp_rig, rates, seconds)
    py_us, _ = bench_controller("Python controller (cross-inherited PD)",
                                py_rig, rates, seconds)

    print("\n--- summary ---")
    print("  update() dispatch: C++ %.2f us  vs  Python %.2f us  (%.1fx)"
          % (cpp_us, py_us, py_us / cpp_us if cpp_us else 0.0))

    # GIL / GC note: time a forced full GC (the kind of pause that hits a Python RT loop)
    gc.collect()
    t0 = time.perf_counter()
    gc.collect()
    print("  forced gc.collect() pause: %.3f ms (a Python RT loop stalls this long "
          "when GC runs mid-cycle)" % ((time.perf_counter() - t0) * 1e3))
    print("\nOK")


if __name__ == "__main__":
    main()
