#!/usr/bin/env python3
"""Measured TF listener -- the per-variant worker behind bench_tf.py.

Runs ONE listener variant against the running storm and prints a single
``RESULT {json}`` line with:
  * ingest_cpu_pct  -- process-wide CPU% (all threads, via time.process_time) spent
    just keeping the buffer fed over a fixed window while the main thread sleeps;
  * lookup_us_median / lookup_us_p99 / lookups_per_sec -- from a tight lookup loop.

Two variants:
  --mode py   stock rclpy path: tf2_ros.Buffer + tf2_ros.TransformListener (Python
              callbacks feed each TransformStamped across into the tf2_py C-extension).
  --mode cpp  rclcppyy.tf: tf2::BufferCore + C++ tf2_ros::TransformListener ingesting
              on its own C++ thread (no per-message Python).

With --idle there is no storm: a static chain is seeded straight into the buffer and
only the lookup loop is measured, isolating pure lookup-call overhead.
"""
import argparse
import json
import time


def _target(frames):
    return "link_%d" % (frames - 1)


def _percentiles(samples_us):
    s = sorted(samples_us)
    n = len(s)
    median = s[n // 2]
    p99 = s[min(n - 1, int(round(0.99 * (n - 1))))]
    return median, p99


def run_cpp(args):
    import cppyy
    import rclcppyy
    from rclcppyy import tf

    rclcpp = rclcppyy.bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    listener = tf.TransformListener()
    target = _target(args.frames)

    if args.idle:
        gm = cppyy.gbl.geometry_msgs.msg
        for i in range(args.frames):
            ts = gm.TransformStamped()
            ts.header.frame_id = "world" if i == 0 else "link_%d" % (i - 1)
            ts.child_frame_id = "link_%d" % i
            ts.transform.translation.x = 0.1
            ts.transform.rotation.w = 1.0
            listener.set_transform(ts, "bench", is_static=True)
    else:
        if not listener.can_transform("world", target, timeout=args.wait):
            print("RESULT " + json.dumps({"mode": "cpp", "error": "no transforms"}))
            return

    def do_lookup():
        return listener.lookup_transform("world", target)

    return _measure("cpp", args, do_lookup, len(listener.get_frame_names()),
                    do_lookup().transform.translation.x)


def run_py(args):
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from geometry_msgs.msg import TransformStamped
    import tf2_ros

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("tf_bench_py")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node, spin_thread=True)  # noqa: F841
    target = _target(args.frames)

    if args.idle:
        for i in range(args.frames):
            ts = TransformStamped()
            ts.header.frame_id = "world" if i == 0 else "link_%d" % (i - 1)
            ts.child_frame_id = "link_%d" % i
            ts.transform.translation.x = 0.1
            ts.transform.rotation.w = 1.0
            buffer.set_transform_static(ts, "bench")
    else:
        deadline = time.time() + args.wait
        while time.time() < deadline and not buffer.can_transform(
                "world", target, Time()):
            time.sleep(0.02)
        if not buffer.can_transform("world", target, Time()):
            print("RESULT " + json.dumps({"mode": "py", "error": "no transforms"}))
            return

    zero = Time()
    zero_to = Duration()

    def do_lookup():
        return buffer.lookup_transform("world", target, zero, zero_to)

    result = _measure("py", args, do_lookup,
                      len(buffer.all_frames_as_string().splitlines()),
                      do_lookup().transform.translation.x)
    # Stop the listener's own spin thread/executor BEFORE shutting rclpy down, else
    # its background spin raises ExternalShutdownException on the way out.
    if getattr(listener, "executor", None) is not None:
        listener.executor.shutdown()
        if getattr(listener, "dedicated_listener_thread", None) is not None:
            listener.dedicated_listener_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return result


def _measure(mode, args, do_lookup, n_frames, x_check):
    # --- ingest CPU window: do nothing but let the listener ingest ---
    if not args.idle:
        w0 = time.perf_counter()
        c0 = time.process_time()
        time.sleep(args.ingest_sec)
        cpu = time.process_time() - c0
        wall = time.perf_counter() - w0
        ingest_cpu_pct = 100.0 * cpu / wall
    else:
        ingest_cpu_pct = 0.0

    # --- lookup loop (under load, unless --idle) ---
    samples_us = []
    loop0 = time.perf_counter()
    for _ in range(args.lookups):
        t0 = time.perf_counter()
        do_lookup()
        samples_us.append((time.perf_counter() - t0) * 1e6)
    loop_wall = time.perf_counter() - loop0
    median, p99 = _percentiles(samples_us)
    lookups_per_sec = args.lookups / loop_wall

    result = {
        "mode": mode,
        "idle": args.idle,
        "frames": args.frames,
        "rate": args.rate,
        "transforms_per_sec": args.frames * args.rate,
        "n_frames_seen": n_frames,
        "x_check": round(float(x_check), 4),
        "ingest_cpu_pct": round(ingest_cpu_pct, 1),
        "lookup_us_median": round(median, 2),
        "lookup_us_p99": round(p99, 2),
        "lookups_per_sec": round(lookups_per_sec, 0),
    }
    print("RESULT " + json.dumps(result), flush=True)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["py", "cpp"], required=True)
    ap.add_argument("--frames", type=int, default=50)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--idle", action="store_true")
    ap.add_argument("--ingest-sec", type=float, default=4.0, dest="ingest_sec")
    ap.add_argument("--lookups", type=int, default=20000)
    ap.add_argument("--wait", type=float, default=8.0)
    args = ap.parse_args()
    if args.mode == "cpp":
        run_cpp(args)
    else:
        run_py(args)


if __name__ == "__main__":
    main()
