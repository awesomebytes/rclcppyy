#!/usr/bin/env python3
"""TF benchmark: stock rclpy Python listener vs rclcppyy C++ listener.

Answers "is TF via rclcppyy more efficient than the stock Python path?" with numbers.
For each scenario it spawns the synthetic TF-storm publisher and then, one at a time,
each listener variant (as its own child process, so their CPU never overlaps). Each
listener reports its own ingest CPU% (process-wide, over a fixed window with the main
thread idle) and its lookup latency / throughput. An extra idle row (no storm) isolates
pure lookup-call overhead.

    pixi run bench-tf                 # default scenarios
    pixi run bench-tf --json          # machine-readable

Shared-machine note: CPU% and latency are directional, not absolute -- a parallel job
inflates both. Run it on a quiet machine for publishable figures.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
STORM = str(HERE / "tf_storm_publisher.py")
LISTENER = str(HERE / "tf_listener_bench.py")

# (label, frames, rate) -> aggregate transforms/s = frames * rate
SCENARIOS = [
    ("1k tf/s", 20, 50.0),
    ("5k tf/s", 50, 100.0),
    ("10k tf/s", 100, 100.0),
]


def run_listener(mode, frames, rate, ingest_sec, lookups, idle=False, timeout=60):
    cmd = [sys.executable, LISTENER, "--mode", mode, "--frames", str(frames),
           "--rate", str(rate), "--ingest-sec", str(ingest_sec),
           "--lookups", str(lookups)]
    if idle:
        cmd.append("--idle")
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    for line in proc.stdout.splitlines():
        if line.startswith("RESULT "):
            return json.loads(line[len("RESULT "):])
    return {"mode": mode, "error": "no RESULT (rc=%d)" % proc.returncode,
            "stderr_tail": "\n".join(proc.stderr.splitlines()[-3:])}


def start_storm(frames, rate):
    proc = subprocess.Popen(
        [sys.executable, STORM, "--frames", str(frames), "--rate", str(rate)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    return proc


def stop(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=8)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def fmt(v):
    if isinstance(v, float):
        return "%.1f" % v
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ingest-sec", type=float, default=3.0, dest="ingest_sec")
    ap.add_argument("--lookups", type=int, default=20000)
    ap.add_argument("--storm-warmup", type=float, default=3.0, dest="storm_warmup")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rows = []  # (label, transforms_per_sec, py_result, cpp_result)

    # Idle lookup-only row (no storm): isolate lookup-call overhead.
    print("[bench] idle lookup-only (no storm) ...", flush=True)
    py_idle = run_listener("py", 20, 50.0, args.ingest_sec, args.lookups, idle=True)
    cpp_idle = run_listener("cpp", 20, 50.0, args.ingest_sec, args.lookups, idle=True)
    rows.append(("idle (no storm)", 0, py_idle, cpp_idle))

    # Storm scenarios.
    for label, frames, rate in SCENARIOS:
        print("[bench] scenario %s (%d frames @ %.0f Hz = %.0f tf/s) ..."
              % (label, frames, rate, frames * rate), flush=True)
        storm = start_storm(frames, rate)
        try:
            time.sleep(args.storm_warmup)
            py = run_listener("py", frames, rate, args.ingest_sec, args.lookups)
            cpp = run_listener("cpp", frames, rate, args.ingest_sec, args.lookups)
        finally:
            stop(storm)
        rows.append((label, int(frames * rate), py, cpp))

    if args.json:
        print(json.dumps([
            {"scenario": r[0], "transforms_per_sec": r[1], "py": r[2], "cpp": r[3]}
            for r in rows], indent=2))
        return

    # --- table ---
    def cell(res, key):
        return fmt(res.get(key, "ERR")) if "error" not in res else "ERR"

    print("\n" + "=" * 92)
    print("TF ingest + lookup: stock rclpy Python listener vs rclcppyy C++ listener")
    print("=" * 92)
    header = ("%-18s | %-21s | %-21s | %-21s"
              % ("scenario", "ingest CPU%  py / cpp",
                 "lookup us med  py / cpp", "lookups/s  py / cpp"))
    print(header)
    print("-" * 92)
    for label, tps, py, cpp in rows:
        ing = "%s / %s" % (cell(py, "ingest_cpu_pct"), cell(cpp, "ingest_cpu_pct"))
        med = "%s / %s" % (cell(py, "lookup_us_median"), cell(cpp, "lookup_us_median"))
        lps = "%s / %s" % (cell(py, "lookups_per_sec"), cell(cpp, "lookups_per_sec"))
        print("%-18s | %-21s | %-21s | %-21s" % (label, ing, med, lps))
    print("-" * 92)
    print("ingest CPU%%: process-wide CPU to keep the buffer fed over %.0fs, main thread"
          " idle." % args.ingest_sec)
    print("Storm rows measure lookups UNDER ingest load; the idle row is lookups only.")
    print("Shared machine -> directional. x_check (chain endpoint) should equal 0.1*frames.")


if __name__ == "__main__":
    main()
