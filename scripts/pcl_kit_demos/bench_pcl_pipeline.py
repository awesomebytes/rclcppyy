#!/usr/bin/env python
"""
Benchmark the pcl_kit ROS pipeline (d02) against the plain rclpy + NumPy baseline
(d03): same 100k-point cloud, same 0.05 m VoxelGrid, same 10 Hz publish rate.

Each demo is a self-contained publisher + pipeline process (see d02 / d03). This
runner spawns each for a short window, samples the process CPU% with psutil (same
methodology as scripts/benchmarks/run_benchmarks.py -- sum the process's own CPU
plus any children, primed then averaged, only while it is in steady state), and
reads the demo's SUMMARY line for per-frame latency. It also counts the
user-facing lines of code of each demo, so the "ease + performance" trade lands in
one table.

CPU is measured at a fixed, realistic 10 Hz so the numbers show headroom: how much
CPU each pipeline burns to keep up with the same sensor rate. "max sustained
msgs/s" is derived from average per-frame latency (1000 / avg_lat_ms), i.e. the
rate each single-threaded pipeline could sustain flat-out.

Run: pixi run -e pcl bench-pcl
"""
import argparse
import ast
import os
import re
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil

HERE = Path(__file__).resolve().parent
os.environ.setdefault("ROS_DOMAIN_ID", "43")

VARIANTS = [
    ("pcl_kit (C++ end-to-end)", "d02_ros_pipeline.py"),
    ("rclpy + NumPy baseline", "d03_baseline_rclpy.py"),
]

STAT_RE = re.compile(r"^STAT ")
SUMMARY_RE = re.compile(
    r"thru_msgs_s=([\d.]+)\s+avg_lat_ms=([\d.]+)\s+p99_lat_ms=([\d.]+)")


def code_loc(path):
    """User-facing lines of code: non-blank, non-comment, non-docstring lines."""
    src = path.read_text()
    lines = src.splitlines()
    tree = ast.parse(src)
    doc_lines = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if ast.get_docstring(node, clean=False) and node.body:
                first = node.body[0]
                doc_lines.update(range(first.lineno, (first.end_lineno or first.lineno) + 1))
    count = 0
    for i, line in enumerate(src.splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or i in doc_lines:
            continue
        count += 1
    return count, len(lines)


class Child:
    """A spawned demo process; streams stdout, flags first STAT (steady state),
    and captures the SUMMARY line."""

    def __init__(self, argv):
        self.proc = psutil.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True)
        self.first_stat = threading.Event()
        self.summary = None
        self.tail = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.tail.append(line)
            if len(self.tail) > 40:
                self.tail.pop(0)
            if STAT_RE.match(line):
                self.first_stat.set()
            m = SUMMARY_RE.search(line)
            if m:
                self.summary = (float(m.group(1)), float(m.group(2)), float(m.group(3)))

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if self.proc.poll() is None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=3)
            except Exception:
                self.proc.kill()


def sample_cpu(child, sample_hz=2.0):
    """Average the process's CPU% (self + children) from first STAT until exit."""
    ps = child.proc
    ps.cpu_percent(None)  # prime
    for c in ps.children(recursive=True):
        try:
            c.cpu_percent(None)
        except psutil.Error:
            pass
    samples = []
    interval = 1.0 / sample_hz
    while child.alive():
        time.sleep(interval)
        total = 0.0
        try:
            total += ps.cpu_percent(None)
            for c in ps.children(recursive=True):
                try:
                    total += c.cpu_percent(None)
                except psutil.Error:
                    pass
        except psutil.Error:
            break
        samples.append(total)
    return statistics.mean(samples) if samples else float("nan")


def run_variant(label, script, rate, duration, warmup_timeout):
    argv = [sys.executable, "-u", str(HERE / script),
            "--rate", str(rate), "--duration", str(duration)]
    print(f"[{label}] starting {script} (bringup + {duration:.0f}s window) ...",
          file=sys.stderr, flush=True)
    child = Child(argv)
    if not child.first_stat.wait(timeout=warmup_timeout):
        child.stop()
        raise RuntimeError(f"{script}: no STAT within {warmup_timeout:.0f}s\n" + "\n".join(child.tail))
    cpu = sample_cpu(child)
    child.proc.wait(timeout=10)
    if child.summary is None:
        child.stop()
        raise RuntimeError(f"{script}: no SUMMARY line\n" + "\n".join(child.tail))
    thru, avg_lat, p99_lat = child.summary
    loc, total = code_loc(HERE / script)
    return {
        "label": label, "cpu": cpu, "thru": thru, "avg_lat": avg_lat,
        "p99_lat": p99_lat, "loc": loc, "total_lines": total,
        "max_sustained": 1000.0 / avg_lat if avg_lat else float("nan"),
    }


def print_table(results, rate):
    cols = ["variant", "avg lat ms", "p99 lat ms", f"CPU% @{rate:g}Hz",
            "max msgs/s*", "user LOC"]
    widths = [28, 12, 12, 13, 13, 9]
    total = sum(widths)
    print()
    print(f"  pcl_kit pipeline vs rclpy baseline  (100k pts, 0.05 m VoxelGrid, {rate:g} Hz)")
    print("  " + "=" * total)
    print("  " + "".join(f"{c:<{w}}" for c, w in zip(cols, widths)))
    print("  " + "-" * total)
    for r in results:
        cells = [r["label"], f"{r['avg_lat']:.2f}", f"{r['p99_lat']:.2f}",
                 f"{r['cpu']:.1f}", f"{r['max_sustained']:.0f}", str(r["loc"])]
        print("  " + "".join(f"{str(c):<{w}}" for c, w in zip(cells, widths)))
    print("  " + "-" * total)
    print("  * max msgs/s is derived from avg per-frame latency (1000/avg_lat_ms).")
    if len(results) == 2:
        a, b = results
        if a["avg_lat"] and b["avg_lat"]:
            print(f"\n  {a['label']} is {b['avg_lat'] / a['avg_lat']:.1f}x lower latency and "
                  f"{b['cpu'] / a['cpu']:.1f}x less CPU than {b['label']},")
            print(f"  in {a['loc']} vs {b['loc']} user lines of code.")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate (Hz)")
    ap.add_argument("--duration", type=float, default=5.0, help="measurement window per variant (s)")
    ap.add_argument("--warmup-timeout", type=float, default=60.0)
    args = ap.parse_args()

    print(f"Benchmarking at {args.rate:g} Hz, {args.duration:.0f}s window per variant. "
          f"(Shared machine: treat as provisional.)", file=sys.stderr, flush=True)
    results = []
    for label, script in VARIANTS:
        try:
            results.append(run_variant(label, script, args.rate, args.duration, args.warmup_timeout))
        except RuntimeError as exc:
            print(f"  FAILED: {exc}", file=sys.stderr, flush=True)
    if results:
        print_table(results, args.rate)
    sys.exit(0 if len(results) == len(VARIANTS) else 1)


if __name__ == "__main__":
    main()
