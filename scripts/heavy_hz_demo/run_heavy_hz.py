#!/usr/bin/env python3
"""Heavy-topic ``ros2 topic hz`` showcase: rclpy vs rclcppyy on the subscriber side.

Spawns a heavy-image publisher and an hz-measuring subscriber as separate child
processes, warms up until messages flow, then samples each process's CPU while
parsing the subscriber's per-second stat lines for achieved hz / received-vs-
expected / drops / latency. The publisher runs C++-accelerated in every variant so
it is never the bottleneck; only the *subscriber* differs between runs -- plain
rclpy vs the one-line ``rclcppyy.enable_cpp_acceleration()``.

    python run_heavy_hz.py --compare               # rclpy vs rclcppyy table
    python run_heavy_hz.py --variant rclcppyy      # one variant
    python run_heavy_hz.py --startup-story         # cold vs warm rclcppyy bringup
    python run_heavy_hz.py --rate 300 --width 1280 --height 960 --duration 12

Run it through the demo env so the bridge + isolated PCH cache are in place:

    pixi run -e heavydemo demo-heavy-hz

It owns the lifecycle of every child (SIGTERM the process group -> grace -> KILL)
and never treats teardown noise as a failure: the underlying spin scripts print a
harmless double-shutdown traceback on signal, so measurement stops by killing them.
"""
import argparse
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

import psutil

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import _hz_common as hz  # noqa: E402

PUB_SCRIPT = str(HERE / "heavy_publisher.py")
SUB_SCRIPT = str(HERE / "hz_subscriber.py")
PROBE_SCRIPT = str(HERE / "startup_probe.py")

HEADERS_LOADED_RE = re.compile(r"rclcpp C\+\+ headers loaded \(([\d.]+)s\)")
READY_RE = re.compile(r"READY elapsed_s=([\d.]+)")
PCH_LOADED_RE = re.compile(r"Cling PCH loaded from")
PCH_BUILDING_RE = re.compile(r"building Cling PCH cache")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


class ChildProcess:
    """A spawned demo script whose stdout+stderr is streamed on a thread.

    For the subscriber, every stat line is parsed and timestamped; a bounded tail
    of raw output is kept for error reporting and for the startup-story scan.
    """

    def __init__(self, name, argv, env, parse_stats=False, echo=False):
        self.name = name
        self.parse_stats = parse_stats
        self.echo = echo
        self.samples = []  # (t_mono, hz, recv, expected, dropped, avg_lat_ms, payload_mb)
        self.tail = deque(maxlen=200)
        # Startup markers, parsed in the reader thread as they arrive (no post-hoc
        # tail scraping, so they survive deque eviction and the exit-time PCH line).
        self.headers_s = None
        self.ready_s = None
        self.pch_loaded = False
        self.pch_building = False
        self._lock = threading.Lock()
        self.proc = psutil.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            bufsize=1, start_new_session=True, env=env,
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            with self._lock:
                self.tail.append(line)
            if self.echo:
                print(f"  [{self.name}] {line}", file=sys.stderr, flush=True)
            self._parse_startup(line)
            if self.parse_stats:
                m = hz.STAT_RE.search(line)
                if m:
                    with self._lock:
                        self.samples.append((
                            time.monotonic(), float(m.group(1)), int(m.group(2)),
                            int(m.group(3)), int(m.group(4)), float(m.group(5)),
                            float(m.group(7)),
                        ))

    def _parse_startup(self, line):
        with self._lock:
            if self.headers_s is None:
                m = HEADERS_LOADED_RE.search(line)
                if m:
                    self.headers_s = float(m.group(1))
            if self.ready_s is None:
                m = READY_RE.search(line)
                if m:
                    self.ready_s = float(m.group(1))
            if PCH_LOADED_RE.search(line):
                self.pch_loaded = True
            if PCH_BUILDING_RE.search(line):
                self.pch_building = True

    def stat_count(self):
        with self._lock:
            return len(self.samples)

    def samples_snapshot(self):
        with self._lock:
            return list(self.samples)

    def alive(self):
        return self.proc.poll() is None

    def tail_text(self):
        with self._lock:
            return "\n".join(self.tail)

    def stop(self, grace=3.0):
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
                pass
            try:
                self.proc.wait(timeout=grace)
            except Exception:
                pass
        # Drain any final lines (e.g. the exit-time PCH-build notice) before callers
        # read the startup markers.
        self._reader.join(timeout=1.0)


def _child_env(accel, topic, rate, width, height, depth):
    env = dict(os.environ)
    env["HEAVY_HZ_ACCEL"] = "1" if accel else "0"
    env["HEAVY_HZ_TOPIC"] = topic
    env["HEAVY_HZ_RATE"] = str(rate)
    env["HEAVY_HZ_WIDTH"] = str(width)
    env["HEAVY_HZ_HEIGHT"] = str(height)
    env["HEAVY_HZ_DEPTH"] = str(depth)
    return env


def _sample_cpu(proc, acc):
    try:
        total = proc.cpu_percent(None)
        for c in proc.children(recursive=True):
            try:
                total += c.cpu_percent(None)
            except psutil.Error:
                pass
    except psutil.Error:
        return
    acc.append(total)


def run_pair(sub_accel, pub_accel, rate, width, height, depth, duration,
             warmup_timeout, echo=False):
    """One publisher+subscriber pair; measure sub/pub CPU + parse sub stats."""
    py = [sys.executable, "-u"]
    # Unique topic per run so back-to-back variants never cross-talk.
    topic = "heavy_image_%d_%s" % (os.getpid(), "acc" if sub_accel else "rcl")
    sub_env = _child_env(sub_accel, topic, rate, width, height, depth)
    pub_env = _child_env(pub_accel, topic, rate, width, height, depth)

    log("  starting subscriber (%s) ..." % hz.variant_label(sub_accel))
    sub = ChildProcess("sub", py + [SUB_SCRIPT, str(rate)], sub_env,
                       parse_stats=True, echo=echo)
    time.sleep(0.5)
    log("  starting publisher (%s, %.0f Hz) ..." % (hz.variant_label(pub_accel), rate))
    pub = ChildProcess("pub", py + [PUB_SCRIPT, str(rate), str(width), str(height)],
                       pub_env, echo=echo)
    children = [sub, pub]
    try:
        deadline = time.monotonic() + warmup_timeout
        while time.monotonic() < deadline:
            if sub.stat_count() >= 1:
                break
            if not pub.alive():
                raise RuntimeError("publisher exited during warmup (code %s)\n%s"
                                   % (pub.proc.returncode, pub.tail_text()))
            if not sub.alive():
                raise RuntimeError("subscriber exited during warmup (code %s)\n%s"
                                   % (sub.proc.returncode, sub.tail_text()))
            time.sleep(0.1)
        else:
            raise RuntimeError("no stat line within %.0fs warmup\n%s"
                               % (warmup_timeout, sub.tail_text()))

        log("  warmed up; measuring for %.0fs ..." % duration)
        pub_ps = psutil.Process(pub.proc.pid)
        sub_ps = psutil.Process(sub.proc.pid)
        for p in (pub_ps, sub_ps):
            p.cpu_percent(None)
            for c in p.children(recursive=True):
                c.cpu_percent(None)

        pub_cpu, sub_cpu = [], []
        t_start = time.monotonic()
        while time.monotonic() - t_start < duration:
            time.sleep(0.5)
            if not sub.alive():
                raise RuntimeError("subscriber crashed during measurement (code %s)\n%s"
                                   % (sub.proc.returncode, sub.tail_text()))
            if not pub.alive():
                raise RuntimeError("publisher crashed during measurement (code %s)\n%s"
                                   % (pub.proc.returncode, pub.tail_text()))
            _sample_cpu(pub_ps, pub_cpu)
            _sample_cpu(sub_ps, sub_cpu)
        t_end = time.monotonic()

        window = [s for s in sub.samples_snapshot() if t_start <= s[0] <= t_end]
        elapsed = t_end - t_start
        recv = sum(s[2] for s in window)
        dropped = sum(s[4] for s in window)
        eff_hz = recv / elapsed if elapsed > 0 else 0.0
        expected = rate * elapsed
        avg_lat = (statistics.mean(s[5] for s in window) if window else float("nan"))
        payload = window[0][6] if window else hz.payload_mb(width, height)

        for c in children:
            c.echo = False
        return {
            "sub_variant": hz.variant_label(sub_accel),
            "payload_mb": round(payload, 2),
            "target_hz": rate,
            "sub_cpu_pct": round(statistics.mean(sub_cpu), 1) if sub_cpu else float("nan"),
            "pub_cpu_pct": round(statistics.mean(pub_cpu), 1) if pub_cpu else float("nan"),
            "eff_hz": round(eff_hz, 1),
            "recv": recv,
            "expected": round(expected),
            "dropped": dropped,
            "avg_lat_ms": round(avg_lat, 1),
            "duration_s": round(elapsed, 1),
        }
    finally:
        for c in children:
            c.stop()


def fmt(x, nan="-"):
    if isinstance(x, float) and x != x:
        return nan
    return x


def print_table(results, rate, width, height):
    cols = ["sub variant", "payload", "target Hz", "sub CPU%", "pub CPU%",
            "eff Hz", "recv/exp", "dropped", "lat ms"]
    widths = [13, 9, 10, 10, 10, 9, 14, 9, 8]
    total = sum(widths)
    print()
    print("  Heavy-topic hz @ %.0f Hz, %dx%d bgr8 (%.2f MB/msg)"
          % (rate, width, height, hz.payload_mb(width, height)))
    print("  " + "=" * total)
    print("  " + "".join(f"{c:<{w}}" for c, w in zip(cols, widths)))
    print("  " + "-" * total)
    for r in results:
        cells = [
            r["sub_variant"], "%.2f MB" % r["payload_mb"], "%.0f" % r["target_hz"],
            fmt(r["sub_cpu_pct"]), fmt(r["pub_cpu_pct"]), fmt(r["eff_hz"]),
            "%s/%s" % (r["recv"], r["expected"]), fmt(r["dropped"]),
            fmt(r["avg_lat_ms"]),
        ]
        print("  " + "".join(f"{str(c):<{w}}" for c, w in zip(cells, widths)))
    sys.stdout.flush()


def _wait_for_pch_build(cache_dir, timeout=90.0):
    """Poll the given cache until the background PCH build finishes."""
    d = Path(cache_dir) / "cppyy_kit" / "pch"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pchs = list(d.glob("*.pch")) if d.is_dir() else []
        locks = list(d.glob("*.lock")) if d.is_dir() else []
        if pchs and not locks:
            return True
        time.sleep(0.5)
    return False


def _probe_once(width, height, depth, warmup_timeout, echo, cache_dir):
    """One run of the single-process bringup probe; returns its startup markers.

    The probe's ``XDG_CACHE_HOME`` is pinned to ``cache_dir`` so the caller controls
    cold (a fresh empty dir) vs warm (the same dir once the PCH is built). We never
    delete a cache the process might be using -- a stale ``CLING_STANDARD_PCH``
    pointing at a removed file crashes Cling.
    """
    topic = "heavy_startup_%d" % os.getpid()
    env = _child_env(True, topic, hz.DEFAULT_RATE, width, height, depth)
    env["XDG_CACHE_HOME"] = str(cache_dir)
    probe = ChildProcess("probe", [sys.executable, "-u", PROBE_SCRIPT], env, echo=echo)
    try:
        deadline = time.monotonic() + warmup_timeout
        while time.monotonic() < deadline:
            if probe.ready_s is not None or not probe.alive():
                break
            time.sleep(0.1)
    finally:
        probe.stop()
    # stop() joins the reader thread, so the cold run's exit-time "building Cling
    # PCH cache" line has been parsed by now.
    return {
        "headers_s": probe.headers_s,
        "ready_s": probe.ready_s,
        "pch_loaded": probe.pch_loaded,
        "pch_building": probe.pch_building,
    }


def _one_bringup(width, height, depth, warmup_timeout, echo, cache_dir, attempts=3):
    """Measure the accelerated subscriber's bringup (headers parse time, time-to-
    ready, PCH flags) in a single process. The first-ever Cling JIT can be slow
    under heavy machine load, so retry a few times before giving up."""
    result = None
    for i in range(attempts):
        result = _probe_once(width, height, depth, warmup_timeout, echo, cache_dir)
        if result["ready_s"] is not None:
            return result
        if i + 1 < attempts:
            log("  probe did not report ready (attempt %d/%d); retrying ..."
                % (i + 1, attempts))
    return result


def startup_story(args):
    """Cold (fresh PCH cache) vs warm rclcppyy bringup: parse cost + time-to-ready."""
    # Force cold with a brand-new empty cache dir rather than deleting the shared
    # one: an empty XDG_CACHE_HOME means no manifest, so the startup .pth sets no
    # CLING_STANDARD_PCH and the run is genuinely cold. The warm run reuses the same
    # dir once its PCH is built.
    story_cache = tempfile.mkdtemp(prefix="heavy_startup_cache_")
    log("Startup story: fresh isolated PCH cache = %s" % story_cache)
    # The cold JIT of rclcpp can take much longer than a warm start, especially on
    # a busy machine, so allow it a generous ceiling.
    probe_timeout = max(args.warmup_timeout, 120.0)
    try:
        log("[COLD] first run (headers JIT-parse; PCH build kicks off at exit) ...")
        cold = _one_bringup(args.width, args.height, hz.DEFAULT_DEPTH,
                            probe_timeout, args.echo, story_cache)
        log("[COLD] rclcpp headers loaded in %ss; time-to-ready %ss; %s"
            % (cold["headers_s"], cold["ready_s"],
               "PCH build scheduled" if cold["pch_building"] else "no PCH build seen"))

        log("Waiting for the background PCH build to finish ...")
        if not _wait_for_pch_build(story_cache):
            log("WARNING: PCH build did not complete in time; warm run may still be cold.")
        else:
            log("PCH ready.")

        log("[WARM] second run (PCH loaded before any import) ...")
        warm = _one_bringup(args.width, args.height, hz.DEFAULT_DEPTH,
                            probe_timeout, args.echo, story_cache)
        log("[WARM] rclcpp headers loaded in %ss; time-to-ready %ss; %s"
            % (warm["headers_s"], warm["ready_s"],
               "PCH loaded from cache" if warm["pch_loaded"] else "PCH NOT loaded"))
    finally:
        shutil.rmtree(story_cache, ignore_errors=True)

    print()
    print("  rclcppyy startup: cold (fresh cache) vs warm (PCH cached)")
    print("  ==========================================================================")
    print("  %-34s %-16s %-16s" % ("", "cold", "warm"))
    print("  %-34s %-16s %-16s" % (
        "rclcpp headers parse (s)", fmt(cold["headers_s"]), fmt(warm["headers_s"])))
    print("  %-34s %-16s %-16s" % (
        "time to ready subscriber (s)", fmt(cold["ready_s"]), fmt(warm["ready_s"])))
    sys.stdout.flush()
    return 0


def main():
    p = argparse.ArgumentParser(
        description="Heavy-topic ros2 topic hz demo: rclpy vs rclcppyy subscriber.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--variant", choices=["rclpy", "rclcppyy"],
                   help="run one subscriber variant (publisher stays accelerated)")
    p.add_argument("--compare", action="store_true",
                   help="run rclpy then rclcppyy and print a comparison table")
    p.add_argument("--startup-story", action="store_true",
                   help="cold vs warm rclcppyy bringup (parse cost + first-msg time)")
    p.add_argument("--rate", type=float, default=hz.DEFAULT_RATE, help="target Hz")
    p.add_argument("--width", type=int, default=hz.DEFAULT_WIDTH)
    p.add_argument("--height", type=int, default=hz.DEFAULT_HEIGHT)
    p.add_argument("--depth", type=int, default=hz.DEFAULT_DEPTH, help="QoS KEEP_LAST depth")
    p.add_argument("--duration", type=float, default=12.0, help="measurement window (s)")
    p.add_argument("--warmup-timeout", type=float, default=60.0)
    p.add_argument("--plain-publisher", action="store_true",
                   help="run the publisher on plain rclpy too (headless test / no bridge)")
    p.add_argument("--echo", action="store_true", help="stream child output live")
    args = p.parse_args()

    if args.startup_story:
        sys.exit(startup_story(args))

    pub_accel = not args.plain_publisher
    if args.compare:
        variants = [False, True]
    elif args.variant:
        variants = [args.variant == "rclcppyy"]
    else:
        variants = [False, True]  # default: compare

    log("Config: %.0f Hz, %dx%d bgr8 (%.2f MB/msg), depth %d, %.0fs window; "
        "publisher=%s." % (args.rate, args.width, args.height,
                           hz.payload_mb(args.width, args.height), args.depth,
                           args.duration, hz.variant_label(pub_accel)))
    results = []
    failures = []
    for sub_accel in variants:
        log("[subscriber = %s]" % hz.variant_label(sub_accel))
        try:
            results.append(run_pair(
                sub_accel, pub_accel, args.rate, args.width, args.height,
                args.depth, args.duration, args.warmup_timeout, echo=args.echo))
        except RuntimeError as exc:
            log("  FAILED: %s" % str(exc).splitlines()[0])
            failures.append((hz.variant_label(sub_accel), str(exc).splitlines()[0]))
    if results:
        print_table(results, args.rate, args.width, args.height)
    print()
    if failures:
        for v, msg in failures:
            log("  - %s failed: %s" % (v, msg))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log("\nInterrupted; children are cleaned up on the way out.")
        sys.exit(130)
