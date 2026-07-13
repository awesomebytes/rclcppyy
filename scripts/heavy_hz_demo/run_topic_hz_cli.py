#!/usr/bin/env python3
"""Accelerate the REAL ``ros2 topic hz`` with RCLCPPYY_ENABLE_HOOK, and measure it.

The subscriber under test is the STOCK ``ros2 topic hz`` binary -- not a demo
script. This orchestrator only drives the harness around it: it publishes a heavy
topic (heavy_publisher.py), runs ``ros2 topic hz <topic> --window N`` two ways
(plain, and with ``RCLCPPYY_ENABLE_HOOK=1`` so the installed .pth accelerates it with zero
code changes), parses the tool's own ``average rate:`` output, and samples the ros2
process's CPU.

    # one-time per env: install the opt-in startup hook (this script does it too)
    python -m rclcppyy.hook install

    python run_topic_hz_cli.py --sweep            # rate x {plain, auto} table
    python run_topic_hz_cli.py --startup-story     # time-to-first-hz-line cold vs warm
    python run_topic_hz_cli.py --sweep --rates 50,100,200 --width 256 --height 256

Run through the demo env so the bridge + isolated PCH cache are in place:

    pixi run -e heavydemo python scripts/heavy_hz_demo/run_topic_hz_cli.py --sweep
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

# `ros2 topic hz` prints, e.g.: "average rate: 100.093"
RATE_RE = re.compile(r"average rate:\s*([\d.]+)")
PUB_RATE_RE = re.compile(r"PUB achieved_hz=([\d.]+)")


_OUT = None  # optional results file (direct, flushed writes -- observable despite pixi buffering)


def log(msg):
    print(msg, file=sys.stderr, flush=True)
    _emit(msg)


def _emit(line):
    if _OUT:
        try:
            with open(_OUT, "a") as f:
                f.write(line + "\n")
                f.flush()
                os.fsync(f.fileno())
        except OSError:
            pass


class Child:
    """A spawned process whose stdout+stderr is streamed on a thread.

    Its whole process group is its own session, so stop() can SIGKILL the group --
    important because rclcpp installs a SIGTERM handler whose teardown can hang.
    """

    def __init__(self, name, argv, env, echo=False):
        self.name = name
        self.echo = echo
        self.lines = deque(maxlen=400)
        self.rates = []          # parsed `average rate:` values (subscriber)
        self.pub_rates = []      # parsed PUB achieved_hz values (publisher)
        self.first_line_t = None  # monotonic time of the first parsed rate line
        self._lock = threading.Lock()
        self.proc = psutil.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, bufsize=1, start_new_session=True, env=env)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            with self._lock:
                self.lines.append(line)
            if self.echo:
                print("  [%s] %s" % (self.name, line), file=sys.stderr, flush=True)
            m = RATE_RE.search(line)
            if m:
                with self._lock:
                    if self.first_line_t is None:
                        self.first_line_t = time.monotonic()
                    self.rates.append(float(m.group(1)))
            m = PUB_RATE_RE.search(line)
            if m:
                with self._lock:
                    self.pub_rates.append(float(m.group(1)))

    def snapshot(self, attr):
        with self._lock:
            return list(getattr(self, attr))

    def alive(self):
        return self.proc.poll() is None

    def tail(self):
        with self._lock:
            return "\n".join(self.lines)

    def stop(self, grace=2.0):
        if self.proc.poll() is not None:
            return
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(os.getpgid(self.proc.pid), sig)
            except (ProcessLookupError, PermissionError):
                break
            try:
                self.proc.wait(timeout=grace)
                break
            except Exception:
                continue
        self._reader.join(timeout=1.0)


def _pub_env(rate, width, height, reliability):
    env = dict(os.environ)
    env["HEAVY_HZ_ACCEL"] = "1"        # C++ publisher: never the bottleneck
    env["HEAVY_HZ_TOPIC"] = "heavy_image"
    env["HEAVY_HZ_RELIABILITY"] = reliability
    return env


def _hz_env(auto, cache_dir=None):
    env = dict(os.environ)
    if auto:
        env["RCLCPPYY_ENABLE_HOOK"] = "1"
    else:
        env.pop("RCLCPPYY_ENABLE_HOOK", None)
    if cache_dir is not None:
        # Point the auto-PCH cache at cache_dir AND drop the PCH the parent process
        # already activated -- otherwise the child inherits CLING_STANDARD_PCH from
        # our warm env and every run is warm regardless of cache_dir.
        env["XDG_CACHE_HOME"] = str(cache_dir)
        env.pop("CLING_STANDARD_PCH", None)
        env.pop("_CPPYY_KIT_AUTOPCH_ACTIVE", None)
    return env


def _sample_cpu(ps, acc):
    try:
        total = ps.cpu_percent(None)
        for c in ps.children(recursive=True):
            try:
                total += c.cpu_percent(None)
            except psutil.Error:
                pass
    except psutil.Error:
        return
    acc.append(total)


def _start_publisher(rate, width, height, reliability, warmup_timeout, echo):
    pub = Child("pub", [sys.executable, "-u", PUB_SCRIPT, str(rate), str(width), str(height)],
                _pub_env(rate, width, height, reliability), echo=echo)
    deadline = time.monotonic() + warmup_timeout
    while time.monotonic() < deadline:
        if pub.snapshot("pub_rates") or not pub.alive():
            break
        time.sleep(0.1)
    return pub


def _run_hz(topic, window, auto, duration, warmup_timeout, cache_dir, echo):
    """Run the stock `ros2 topic hz` once; return reported hz, CPU%, time-to-first-line."""
    argv = ["ros2", "topic", "hz", topic, "--window", str(window)]
    hzc = Child("hz-%s" % ("auto" if auto else "plain"), argv,
                _hz_env(auto, cache_dir), echo=echo)
    try:
        launch = time.monotonic()
        deadline = launch + warmup_timeout
        while time.monotonic() < deadline:
            if hzc.first_line_t is not None or not hzc.alive():
                break
            time.sleep(0.1)
        if hzc.first_line_t is None:
            raise RuntimeError("no 'average rate' line within %.0fs\n%s"
                               % (warmup_timeout, hzc.tail()))
        ttfl = hzc.first_line_t - launch
        # measure CPU over the window
        ps = psutil.Process(hzc.proc.pid)
        ps.cpu_percent(None)
        for c in ps.children(recursive=True):
            c.cpu_percent(None)
        cpu = []
        n0 = len(hzc.snapshot("rates"))
        t_start = time.monotonic()
        while time.monotonic() - t_start < duration:
            time.sleep(0.5)
            if not hzc.alive():
                break
            _sample_cpu(ps, cpu)
        window_rates = hzc.snapshot("rates")[n0:]
        return {
            "reported_hz": round(statistics.median(window_rates), 1) if window_rates else float("nan"),
            "cpu": round(statistics.mean(cpu), 1) if cpu else float("nan"),
            "ttfl": round(ttfl, 2),
        }
    finally:
        hzc.stop()


def sweep(args):
    topic = "/heavy_image"
    rates = [int(r) for r in args.rates.split(",") if r.strip()]
    rows = []
    log("Sweep: %s, %dx%d bgr8 (%.2f MB), publisher QoS=%s, window %d, %.0fs each."
        % (topic, args.width, args.height, hz.payload_mb(args.width, args.height),
           args.reliability, args.window, args.duration))
    for rate in rates:
        log("[%d Hz] starting publisher ..." % rate)
        pub = _start_publisher(rate, args.width, args.height, args.reliability,
                               args.warmup_timeout, args.echo)
        try:
            time.sleep(1.0)
            result = {"rate": rate}
            for auto in (False, True):
                label = "auto" if auto else "plain"
                log("[%d Hz] ros2 topic hz (%s) ..." % (rate, label))
                try:
                    r = _run_hz(topic, args.window, auto, args.duration,
                                args.warmup_timeout, None, args.echo)
                except RuntimeError as exc:
                    log("  FAILED: %s" % str(exc).splitlines()[0])
                    r = {"reported_hz": float("nan"), "cpu": float("nan"), "ttfl": float("nan")}
                result[label] = r
            result["pub_achieved"] = (round(statistics.median(pub.snapshot("pub_rates")), 1)
                                      if pub.snapshot("pub_rates") else float("nan"))
            rows.append(result)
            p, a = result["plain"], result["auto"]
            _emit("ROW rate=%d pub=%s | rclpy hz=%s cpu=%s | rclcppyy hz=%s cpu=%s"
                  % (rate, result["pub_achieved"], p["reported_hz"], p["cpu"],
                     a["reported_hz"], a["cpu"]))
        finally:
            pub.stop()
    _print_sweep(rows, args)


def _f(x):
    return "-" if isinstance(x, float) and x != x else x


def _print_sweep(rows, args):
    print()
    print("  ros2 topic hz on %dx%d bgr8 (%.2f MB), publisher QoS=%s"
          % (args.width, args.height, hz.payload_mb(args.width, args.height), args.reliability))
    cols = ["target Hz", "pub Hz", "plain hz", "plain CPU%", "auto hz", "auto CPU%", "CPU x"]
    w = [10, 8, 9, 11, 9, 10, 7]
    print("  " + "=" * sum(w))
    print("  " + "".join("%-*s" % (wi, c) for c, wi in zip(cols, w)))
    print("  " + "-" * sum(w))
    for r in rows:
        p, a = r["plain"], r["auto"]
        speed = ("%.1f" % (p["cpu"] / a["cpu"]) if isinstance(p["cpu"], float) and isinstance(a["cpu"], float)
                 and a["cpu"] == a["cpu"] and p["cpu"] == p["cpu"] and a["cpu"] > 0 else "-")
        cells = [r["rate"], _f(r["pub_achieved"]), _f(p["reported_hz"]), _f(p["cpu"]),
                 _f(a["reported_hz"]), _f(a["cpu"]), speed]
        print("  " + "".join("%-*s" % (wi, str(c)) for c, wi in zip(cells, w)))
    sys.stdout.flush()


def startup_story(args):
    topic = "/heavy_image"
    log("Startup story on the real `ros2 topic hz`: time-to-first-hz-line, cold vs warm.")
    log("Starting publisher ...")
    pub = _start_publisher(args.rate, args.width, args.height, args.reliability,
                           args.warmup_timeout, args.echo)
    story_cache = tempfile.mkdtemp(prefix="ros2cli_startup_")
    try:
        time.sleep(1.0)
        log("[COLD] fresh PCH cache -> `RCLCPPYY_ENABLE_HOOK=1 ros2 topic hz` ...")
        cold = _run_hz(topic, args.window, True, 2.0, max(args.warmup_timeout, 120.0),
                       story_cache, args.echo)
        log("[COLD] time to first hz line: %ss" % cold["ttfl"])
        # The hz process is SIGKILLed (rclcpp's SIGTERM teardown can hang), so its
        # at-exit PCH build never runs. Build the cache with a clean-exiting helper
        # so the warm run actually finds a PCH.
        log("Building the PCH cache with a clean-exiting helper ...")
        _warm_pch_cache(story_cache, args.echo)
        _wait_pch(story_cache)
        # The hz path also JIT-instantiates create_subscription<Image> (~2.8 s); build
        # its cached trampoline .so so the warm run loads it too.
        log("Building the subscription-trampoline cache ...")
        _prebuild_subscription(story_cache, "sensor_msgs::msg::Image",
                               "sensor_msgs/msg/image.hpp", args.echo)
        log("[WARM] PCH cached -> `RCLCPPYY_ENABLE_HOOK=1 ros2 topic hz` ...")
        warm = _run_hz(topic, args.window, True, 2.0, max(args.warmup_timeout, 120.0),
                       story_cache, args.echo)
        log("[WARM] time to first hz line: %ss" % warm["ttfl"])
        log("[PLAIN] stock `ros2 topic hz` (no acceleration) for reference ...")
        plain = _run_hz(topic, args.window, False, 2.0, args.warmup_timeout, None, args.echo)
        log("[PLAIN] time to first hz line: %ss" % plain["ttfl"])
        print()
        print("  `ros2 topic hz` time-to-first-hz-line")
        print("  " + "=" * 52)
        print("  %-34s %s" % ("stock (no acceleration)", "%.2f s" % plain["ttfl"]))
        print("  %-34s %s" % ("RCLCPPYY_ENABLE_HOOK=1, cold cache", "%.2f s" % cold["ttfl"]))
        print("  %-34s %s" % ("RCLCPPYY_ENABLE_HOOK=1, warm cache", "%.2f s" % warm["ttfl"]))
        sys.stdout.flush()
    finally:
        pub.stop()
        shutil.rmtree(story_cache, ignore_errors=True)


def _prebuild_subscription(cache_dir, cpp_type, header, echo):
    """Synchronously build the subscription-trampoline .so into cache_dir so the warm
    run loads it instead of JIT-instantiating create_subscription<MsgT>."""
    env = dict(os.environ)
    env["XDG_CACHE_HOME"] = str(cache_dir)
    env.pop("CLING_STANDARD_PCH", None)
    env.pop("_CPPYY_KIT_AUTOPCH_ACTIVE", None)
    p = Child("subprebuild", [sys.executable, "-u", "-m", "rclcpp_kit._sub_prebuild",
                              cpp_type, header], env, echo=echo)
    deadline = time.monotonic() + 180.0
    while time.monotonic() < deadline and p.alive():
        time.sleep(0.5)
    p.stop()


def _warm_pch_cache(cache_dir, echo):
    """Bring up acceleration in a throwaway process that exits cleanly, so the
    auto-PCH build (scheduled at interpreter exit) actually runs and populates
    cache_dir. `ros2 topic hz` can't do this itself because we SIGKILL it."""
    env = _hz_env(True, cache_dir)
    code = "import rclpy; rclpy.init(); rclpy.shutdown()"
    warmer = Child("warm", [sys.executable, "-u", "-c", code], env, echo=echo)
    # Clean exit builds the PCH; give it room, then let stop() reap if needed.
    deadline = time.monotonic() + 180.0
    while time.monotonic() < deadline and warmer.alive():
        time.sleep(0.5)
    warmer.stop()


def _wait_pch(cache_dir, timeout=90.0):
    d = Path(cache_dir) / "cppyy_kit" / "pch"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if d.is_dir() and list(d.glob("*.pch")) and not list(d.glob("*.lock")):
            return True
        time.sleep(0.5)
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sweep", action="store_true", help="rate x {plain, auto} CPU/hz table")
    p.add_argument("--startup-story", action="store_true",
                   help="time-to-first-hz-line, cold vs warm, on the real tool")
    p.add_argument("--rates", default="50,100,200,500,700")
    p.add_argument("--rate", type=float, default=100.0, help="rate for --startup-story")
    p.add_argument("--width", type=int, default=256)
    p.add_argument("--height", type=int, default=256)
    p.add_argument("--window", type=int, default=100, help="ros2 topic hz --window")
    p.add_argument("--reliability", choices=["reliable", "best_effort"], default="reliable",
                   help="publisher QoS (the hz tool's reader is always BEST_EFFORT)")
    p.add_argument("--duration", type=float, default=10.0, help="measurement window (s)")
    p.add_argument("--warmup-timeout", type=float, default=60.0)
    p.add_argument("--out", default=None,
                   help="also write progress/results here (direct flushed writes)")
    p.add_argument("--echo", action="store_true")
    args = p.parse_args()

    global _OUT
    _OUT = args.out
    if _OUT:
        open(_OUT, "w").close()  # truncate

    # Ensure the opt-in hook is installed so RCLCPPYY_ENABLE_HOOK=1 works for the child
    # ros2 process. Idempotent; gated on the env var, so a no-op when unset.
    try:
        from rclcppyy import hook
        hook.install()
    except Exception as exc:
        log("warning: could not install auto-accel hook (%s)" % exc)

    if args.startup_story:
        startup_story(args)
    else:
        sweep(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log("\nInterrupted; children cleaned up.")
        sys.exit(130)
