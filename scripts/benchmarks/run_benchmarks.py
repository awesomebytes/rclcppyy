#!/usr/bin/env python3
"""Benchmark runner: rclpy vs rclcppyy CPU comparison in one command.

Spawns a publisher+subscriber pair (as separate child processes) for each
variant and rate, warms up until the subscriber is receiving, then samples
per-process CPU with psutil while parsing the subscriber's stat lines for
throughput / drops / latency. Prints a comparison table per rate.

This replaces the old "four shells plus top" workflow. It owns the lifecycle
of every child it spawns (terminate -> grace -> kill) and never treats a
teardown signal as a benchmark failure: the underlying bench scripts have a
pre-existing crash on SIGINT teardown after the last message, which is why we
kill them with SIGTERM once measurement is done rather than asking them to
shut down cleanly.

Usage:
    python run_benchmarks.py                       # rclpy vs rclcppyy at 1k+10k Hz
    python run_benchmarks.py --rate 5000 --duration 10
    python run_benchmarks.py --variants rclpy,rclcppyy,rclcppyy-templated
    python run_benchmarks.py --json                # machine-readable (for CI)
    python run_benchmarks.py --demo                # live pub/sub demo, no table
"""

import argparse
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import psutil

HERE = Path(__file__).resolve().parent

# Each variant is a publisher/subscriber script pair. The subscribers print a
# periodic stat line every 1000 received messages; the publishers take a target
# rate (Hz) as their first CLI argument. The pairs use distinct topic names, so
# even variants running back to back never cross-talk.
VARIANTS = {
    "rclpy": {
        "label": "rclpy",
        "pub": "bench_pub_rclpy.py",
        "sub": "bench_sub_rclpy.py",
    },
    "rclcppyy": {
        "label": "rclcppyy (monkeypatched)",
        "pub": "bench_pub_rclcppyy_monkeypatch.py",
        "sub": "bench_sub_rclcppyy_monkeypatched.py",
    },
    "rclcppyy-templated": {
        "label": "rclcppyy (pure cppyy)",
        "pub": "bench_pub_rclcppyy.py",
        "sub": "bench_sub_rclcppyy.py",
    },
}
DEFAULT_VARIANTS = ["rclpy", "rclcppyy"]
DEFAULT_RATES = [1000, 10000]

# Subscribers print, e.g.:
#   (rclpy) Messages: 1000, Rate: 01000.4 msgs/sec, Latency (us) - Avg: 210.3, P99: 512.0, Dropped: 0
# The unicode micro-sign in "(us)" is avoided in the pattern so encoding quirks
# never break parsing. Each such line == 1000 more messages received.
STAT_RE = re.compile(
    r"Rate:\s*([\d.]+)\s*msgs/sec.*?Avg:\s*([\d.]+),\s*P99:\s*([\d.]+),\s*Dropped:\s*(\d+)"
)
MESSAGES_PER_STAT_LINE = 1000


def log(msg):
    """Progress goes to stderr so stdout stays clean for --json."""
    print(msg, file=sys.stderr, flush=True)


class ChildProcess:
    """A spawned bench script whose stdout+stderr is streamed on a thread.

    Sub processes: every stat line is parsed and timestamped so the runner can
    detect warmup, window received/dropped counts, and latency. A bounded tail
    of raw output is kept for error reporting.
    """

    def __init__(self, name, argv, parse_stats=False, echo=False):
        self.name = name
        self.parse_stats = parse_stats
        self.echo = echo
        self.samples = []  # list of (monotonic_time, rate, avg_lat, p99_lat, dropped_cumulative)
        self.tail = deque(maxlen=60)
        self._lock = threading.Lock()
        # start_new_session so the child (and anything it spawns) is its own
        # process group; teardown signals the whole group -> no orphans.
        # -u forces unbuffered stdout so stat lines arrive promptly through the
        # pipe instead of being block-buffered until the child exits.
        self.proc = psutil.Popen(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.tail.append(line)
            if self.echo:
                print(f"  [{self.name}] {line}", file=sys.stderr, flush=True)
            if self.parse_stats:
                m = STAT_RE.search(line)
                if m:
                    with self._lock:
                        self.samples.append((
                            time.monotonic(),
                            float(m.group(1)),
                            float(m.group(2)),
                            float(m.group(3)),
                            int(m.group(4)),
                        ))

    def stat_count(self):
        with self._lock:
            return len(self.samples)

    def samples_snapshot(self):
        with self._lock:
            return list(self.samples)

    def alive(self):
        return self.proc.poll() is None

    def tail_text(self):
        return "\n".join(self.tail)

    def stop(self, grace=3.0):
        """Terminate the whole process group, escalating to KILL after grace."""
        if self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            try:
                self.proc.terminate()
            except psutil.Error:
                pass
        try:
            self.proc.wait(timeout=grace)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                try:
                    self.proc.kill()
                except psutil.Error:
                    pass
            try:
                self.proc.wait(timeout=grace)
            except Exception:
                pass


def _avg_cpu(proc, samples):
    """Sum the process's own CPU% (all threads) since the last cpu_percent call.

    We spawn `python <script>` directly (no ros2run wrapper), so the spawned PID
    is the real worker and psutil.Process.cpu_percent covers all its threads.
    Any children are summed too, defensively, in case a variant ever forks.
    """
    total = 0.0
    try:
        total += proc.cpu_percent(None)
        for child in proc.children(recursive=True):
            try:
                total += child.cpu_percent(None)
            except psutil.Error:
                pass
    except psutil.Error:
        return None
    samples.append(total)
    return total


def run_pair(variant_key, rate, duration, warmup_timeout, sample_hz=2.0, echo=False):
    """Run one publisher+subscriber pair; measure CPU and parse sub stats.

    Returns a result dict on success, or raises RuntimeError with a message
    (and the child's output tail) on a real failure.
    """
    spec = VARIANTS[variant_key]
    pub_script = str(HERE / spec["pub"])
    sub_script = str(HERE / spec["sub"])
    py = [sys.executable, "-u"]

    log(f"  starting subscriber ({spec['sub']}) ...")
    sub = ChildProcess("sub", py + [sub_script], parse_stats=True, echo=echo)
    # Small stagger so the subscriber node exists before the publisher floods;
    # cyclonedds LOCALHOST tolerates the reverse too, and the sub baselines its
    # dropped-count on the first message it actually receives.
    time.sleep(0.5)
    log(f"  starting publisher ({spec['pub']} {rate}) ...")
    pub = ChildProcess("pub", py + [pub_script, str(rate)], echo=echo)

    children = [sub, pub]
    try:
        # --- warmup: wait until the subscriber reports its first stat line ---
        # (rclcppyy variants JIT-compile rclcpp on bringup, ~2 s, excluded here.)
        deadline = time.monotonic() + warmup_timeout
        while time.monotonic() < deadline:
            if sub.stat_count() >= 1:
                break
            if not pub.alive():
                raise RuntimeError(
                    f"publisher exited during warmup (code {pub.proc.returncode})\n"
                    f"--- pub output tail ---\n{pub.tail_text()}"
                )
            if not sub.alive():
                raise RuntimeError(
                    f"subscriber exited during warmup (code {sub.proc.returncode})\n"
                    f"--- sub output tail ---\n{sub.tail_text()}"
                )
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"no messages received within {warmup_timeout:.0f}s warmup\n"
                f"--- sub output tail ---\n{sub.tail_text()}"
            )

        log(f"  warmed up; measuring for {duration:.0f}s ...")

        # --- measurement window ---
        pub_ps = psutil.Process(pub.proc.pid)
        sub_ps = psutil.Process(sub.proc.pid)
        pub_ps.cpu_percent(None)  # prime: first call always returns 0.0
        sub_ps.cpu_percent(None)
        for c in pub_ps.children(recursive=True):
            c.cpu_percent(None)
        for c in sub_ps.children(recursive=True):
            c.cpu_percent(None)

        pub_cpu, sub_cpu = [], []
        dropped_at_start = sub.samples_snapshot()[-1][4]
        t_start = time.monotonic()
        interval = 1.0 / sample_hz
        while time.monotonic() - t_start < duration:
            time.sleep(interval)
            if not pub.alive():
                raise RuntimeError(
                    f"publisher crashed during measurement (code {pub.proc.returncode})\n"
                    f"--- pub output tail ---\n{pub.tail_text()}"
                )
            if not sub.alive():
                raise RuntimeError(
                    f"subscriber crashed during measurement (code {sub.proc.returncode})\n"
                    f"--- sub output tail ---\n{sub.tail_text()}"
                )
            _avg_cpu(pub_ps, pub_cpu)
            _avg_cpu(sub_ps, sub_cpu)
        t_end = time.monotonic()

        # --- derive stats from stat lines that landed inside the window ---
        window = [s for s in sub.samples_snapshot() if t_start <= s[0] <= t_end]
        elapsed = t_end - t_start
        received = len(window) * MESSAGES_PER_STAT_LINE
        dropped = (window[-1][4] - dropped_at_start) if window else 0
        avg_latency = statistics.mean(s[2] for s in window) if window else float("nan")
        p99_latency = max((s[3] for s in window), default=float("nan"))
        eff_rate = received / elapsed if elapsed > 0 else 0.0

        # Stop echoing before teardown: the bench scripts print a (harmless,
        # pre-existing) double-shutdown traceback when signalled, because
        # rclcpp installs its own SIGINT/SIGTERM handler. Measurement is done,
        # so that noise is not interesting.
        for c in children:
            c.echo = False

        return {
            "variant": variant_key,
            "label": spec["label"],
            "target_rate_hz": rate,
            "duration_s": round(elapsed, 2),
            "pub_cpu_pct": round(statistics.mean(pub_cpu), 1) if pub_cpu else float("nan"),
            "sub_cpu_pct": round(statistics.mean(sub_cpu), 1) if sub_cpu else float("nan"),
            "msgs_received": received,
            "effective_rate_hz": round(eff_rate, 1),
            "dropped": dropped,
            "avg_latency_us": round(avg_latency, 1),
            "p99_latency_us": round(p99_latency, 1),
        }
    finally:
        for c in children:
            c.stop()


def fmt(x, nan="-"):
    if isinstance(x, float) and x != x:  # NaN
        return nan
    return x


def print_table(rate, results):
    cols = ["variant", "pub CPU%", "sub CPU%", "msgs recv", "eff Hz", "dropped", "avg lat us"]
    widths = [26, 10, 10, 11, 10, 9, 11]
    total = sum(widths)
    print()
    print(f"  Benchmark @ {rate} Hz target")
    print("  " + "=" * total)
    print("  " + "".join(f"{c:<{w}}" for c, w in zip(cols, widths)))
    print("  " + "-" * total)
    for r in results:
        cells = [
            r["label"],
            fmt(r["pub_cpu_pct"]),
            fmt(r["sub_cpu_pct"]),
            fmt(r["msgs_received"]),
            fmt(r["effective_rate_hz"]),
            fmt(r["dropped"]),
            fmt(r["avg_latency_us"]),
        ]
        print("  " + "".join(f"{str(c):<{w}}" for c, w in zip(cells, widths)))
    sys.stdout.flush()


def parse_rates(rate_args):
    if not rate_args:
        return list(DEFAULT_RATES)
    rates = []
    for chunk in rate_args:
        for part in str(chunk).split(","):
            part = part.strip()
            if part:
                rates.append(int(part))
    return rates


def parse_variants(variants_arg):
    if not variants_arg:
        return list(DEFAULT_VARIANTS)
    out = []
    for part in variants_arg.split(","):
        part = part.strip()
        if not part:
            continue
        if part not in VARIANTS:
            raise SystemExit(
                f"unknown variant '{part}'. choose from: {', '.join(VARIANTS)}"
            )
        out.append(part)
    return out


def run_demo(args):
    """Light one-command pub/sub demo: run one variant live, no comparison."""
    variant = args.demo_variant
    rate = parse_rates(args.rate)[0] if args.rate else 1000
    duration = args.duration if args.duration is not None else 10.0
    log(f"Demo: {VARIANTS[variant]['label']} publisher + subscriber at {rate} Hz "
        f"for ~{duration:.0f}s (Ctrl-C to stop early).")
    log("Live subscriber stats follow (rate / latency / dropped):")
    try:
        result = run_pair(
            variant, rate, duration,
            warmup_timeout=args.warmup_timeout, echo=True,
        )
    except RuntimeError as exc:
        log(f"DEMO FAILED: {exc}")
        return 1
    log("")
    log(f"Demo summary: received {result['msgs_received']} msgs "
        f"(~{result['effective_rate_hz']:.0f} Hz), {result['dropped']} dropped, "
        f"pub {result['pub_cpu_pct']}% CPU, sub {result['sub_cpu_pct']}% CPU, "
        f"avg latency {result['avg_latency_us']} us.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="rclpy vs rclcppyy pub/sub CPU benchmark (one command).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--rate", action="append",
        help="target publish rate in Hz; repeatable or comma-separated "
             "(default: 1000,10000)",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="measurement window per run in seconds (default: 15; demo: 10)",
    )
    parser.add_argument(
        "--variants", default=None,
        help=f"comma-separated variants to run (default: {','.join(DEFAULT_VARIANTS)}). "
             f"available: {', '.join(VARIANTS)}",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="emit machine-readable JSON on stdout instead of a table",
    )
    parser.add_argument(
        "--warmup-timeout", type=float, default=60.0,
        help="max seconds to wait for the subscriber to start receiving",
    )
    parser.add_argument(
        "--demo", action="store_true",
        help="light mode: run one variant's pub/sub live, no comparison table",
    )
    parser.add_argument(
        "--demo-variant", default="rclcppyy", choices=list(VARIANTS),
        help="variant used by --demo",
    )
    args = parser.parse_args()

    if args.demo:
        sys.exit(run_demo(args))

    duration = args.duration if args.duration is not None else 15.0
    rates = parse_rates(args.rate)
    variants = parse_variants(args.variants)

    log(f"Running variants {variants} at rates {rates}, {duration:.0f}s each.")
    all_results = {}
    failures = []
    for rate in rates:
        results = []
        for v in variants:
            log(f"[{VARIANTS[v]['label']} @ {rate} Hz]")
            try:
                results.append(run_pair(
                    v, rate, duration, warmup_timeout=args.warmup_timeout,
                ))
            except RuntimeError as exc:
                log(f"  FAILED: {exc}")
                failures.append((v, rate, str(exc).splitlines()[0]))
        all_results[rate] = results
        if not args.json:
            print_table(rate, results)

    if args.json:
        print(json.dumps(
            {str(k): v for k, v in all_results.items()}, indent=2
        ))
    else:
        print()
        if failures:
            log(f"{len(failures)} run(s) failed:")
            for v, rate, msg in failures:
                log(f"  - {v} @ {rate} Hz: {msg}")
        else:
            log("All runs completed.")

    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log("\nInterrupted; children are cleaned up on the way out.")
        sys.exit(130)
