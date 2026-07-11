# tf spike — is TF via rclcppyy more efficient than the stock Python path?

**Date:** 2026-07-11 · **Env:** pixi `default` (robostack-jazzy `ros-base` + conda-forge),
`cppyy 3.5.0`, Python 3.12.13, `tf2`/`tf2_ros` 0.36.x, cyclonedds, linux-64.
`ROS_DOMAIN_ID=51`. Shared machine during measurement (a parallel vision job on
domain 52) — figures are directional, not absolute.

**Sam's question (verbatim):** *"for rclcppyy maybe there's a more efficient/better
way of using TF. I am not fully sure how TF in python works, but I assume that it
will be way less efficient than using it via rclcppyy."*

**Verdict: Sam is RIGHT, and it's bigger than just ingest. GO.** Running tf2's **C++**
`tf2_ros::TransformListener` + `tf2::BufferCore` on an rclcppyy node ingests `/tf`
entirely in C++ on its own thread — **~7–14× less CPU** than the stock rclpy Python
listener under a TF storm (the win grows with tf traffic) — **and** each
`lookup_transform` is **~5× cheaper** (1.4 µs vs 7.5 µs) because the Python `Buffer`
pays two C-extension round-trips plus a Python message build per call. Delivered as
`rclcppyy/tf.py` (rclcppyy-proper, since tf2 is core ROS 2 in the default env), a
`TransformListener` helper whose lookups return the real
`geometry_msgs::msg::TransformStamped`. Tests run in the default suite (8 tests, 4.2 s,
including a real network-ingest test); demos + bench are pixi tasks with clean exit 0.

---

## Job 1 — how TF-in-Python actually works (from the installed sources)

The stock **Python** listener/buffer path, cited to the site-packages sources:

**`tf2_ros/transform_listener.py`** — `TransformListener.__init__` creates two **rclpy**
subscriptions, on `/tf` (QoS depth 100, volatile) and `/tf_static` (depth 100,
transient-local), whose callbacks are **Python methods** `self.callback` /
`self.static_callback` (lines 85–88); with `spin_thread=True` it spins its own
`SingleThreadedExecutor` in a Python `threading.Thread` (lines 90–99). The callback
(lines 114–118) is the hot path:

```python
def callback(self, data: TFMessage) -> None:
    who = 'default_authority'
    for transform in data.transforms:          # Python for-loop
        self.buffer.set_transform(transform, who)   # one Python->C crossing each
```

By the time this runs, rclpy has already **deserialized the whole `TFMessage` into
Python objects** — a Python list of Python `TransformStamped`, each a tree of Python
`Header`/`Transform`/`Vector3`/`Quaternion`. Then a **Python loop** hands each
transform, **one at a time**, into the buffer.

**`tf2_ros/buffer.py`** — `Buffer` subclasses `tf2_py.BufferCore` (a C extension,
`tf2_py/_tf2_py.so`, wrapping the C++ `tf2::BufferCore`) and `BufferInterface`
(line 56). `set_transform` (lines 111–117) calls `super().set_transform(...)` —
crossing each Python `TransformStamped` into the C extension, which converts it to a
C++ `geometry_msgs::msg::TransformStamped` and inserts it — and then runs
`_call_new_data_callbacks()` (a Python `RLock` + list iterate) **on every insert**.
`lookup_transform` (lines 133–150) calls `can_transform` (which, with the default
zero timeout, immediately calls the C-extension `can_transform_core`) and then
`lookup_transform_core` (C extension); the TF math is C++, but every call marshals
the string/`Time`/`Duration` args in and **builds a fresh Python `TransformStamped`
out**.

**So per `/tf` message with N transforms the Python path pays, all on a Python thread
holding the GIL:** (1) full rclpy deserialization into Python message objects,
(2) a Python for-loop, (3) N Python→C `set_transform` calls each re-converting a
Python message to C++, (4) N `_call_new_data_callbacks` iterations. The actual cache
and interpolation are C++ (in `tf2_py`), but *feeding* the cache is entirely Python.

**The C++ path (`tf2_ros/transform_listener.hpp`)** subscribes with a **C++** callback
`subscription_callback(TFMessage::ConstSharedPtr, is_static)`; rclcpp delivers a **C++**
`TFMessage`, the callback iterates and calls `buffer_.setTransform(...)` in C++. With
`spin_thread=true` it builds its **own `SingleThreadedExecutor` on a dedicated
`std::thread`** and sets `buffer_.setUsingDedicatedThread(true)`. Ingest is therefore
wholly in C++, off the GIL; Python only crosses the boundary when it calls
`lookup_transform`. That is exactly the asymmetry the benchmark measures.

---

## Job 2 — TF via rclcppyy (the probe)

**Coupling triage first (per nav2 REPORT §2 — check the ctor signatures before
investing).** `tf2_ros::TransformListener` has three ctors, all *plain*: `(tf2::BufferCore&,
bool spin_thread=true)` (makes its own node), `(BufferCore&, NodeT&& node, ...)`
(templated on the node), and a node-interfaces form. It takes a **`tf2::BufferCore&`** —
not a `LifecycleNode`, not a pluginlib base — so it is drivable. `tf2::BufferCore`
itself is a plain class (the cache + math, no node).

Proven, in order (scratch probes; evidence in the commit's scratch history):

| # | Capability | Result | Evidence |
|---|---|:--:|---|
| A | **Bringup + JIT**: include `tf2/buffer_core.hpp` + `tf2_ros/transform_listener.h` + TFMessage, load `libtf2`/`libtf2_ros` | **WORKS** | headers JIT-parse in **~0.05 s + ~0.10 s** on top of the rclcpp include (bringup_rclcpp ~1.8 s) |
| B | **Numeric correctness** (plain `BufferCore`, no net): set a known 2-hop tree, `lookupTransform` | **WORKS** | `world<-sensor` returns exactly `(1, 2, 0)`; `canTransform` True/False correct; `allFramesAsString` reads the tree |
| C | **Network ingest by the C++ listener** (no Python per-message crossing) | **WORKS** | publish `/tf` via rclcppyy → C++ `TransformListener` (own thread) ingests → Python `lookup` returns the composed transform; asserted numerically |
| D | **canTransform / timeouts** | **WORKS** | a C++ `can_wait` poll helper (steady-clock deadline, the listener's dedicated thread keeps ingesting) returns on availability or times out; missing-frame lookups raise a clean `TransformException` |
| E | **Clean teardown** | **WORKS** | releasing the listener (dtor cancels its executor + joins its thread) before `rclcpp::shutdown()` via `register_teardown` → exit 0 |

**Two friction points hit and hidden (both known cppyy patterns):**

1. **`tf2_ros::Buffer` mis-resolves under cppyy and crashes.** Its `lookupTransform`/
   `canTransform` are heavily overloaded (a `tf2::TimePoint` form via `using`, plus
   `rclcpp::Time`+`Duration` timeout forms). A 3-arg `lookupTransform(target, source,
   TimePointZero)` from Python resolved into the **timeout-path `canTransform`**, which
   calls `rclcpp::Clock::now()` and **bus-errors** (confirmed by the fault backtrace:
   `Clock::now()` ← `tf2_ros::Buffer::canTransform(...)`). Also its ctor is a template
   with a universal-reference default (`NodeT&& node = NodeT()`) which cppyy rejects
   ("class has no public constructors"). **Fix:** use the plain `tf2::BufferCore` (the
   listener accepts `BufferCore&` directly) whose single `TimePoint` overloads resolve
   cleanly, and route lookups through **unambiguous `cppdef` free functions**.
2. **Build the listener in a `cppdef` factory.** `make_shared<TransformListener>(buf,
   node, spin)` compiles in C++ but doesn't resolve when driven from Python — the
   recurring "construct the object in C++" pattern (control_kit `make_shared`, nav2
   glue). A one-line factory in the glue does it.

The kit is small: `rclcppyy/tf.py` is **~120 lines of Python + ~35 lines of embedded
C++ glue** (a factory, a `TimePoint`-from-nanos converter, and `can`/`lookup`/
`can_wait` accessors).

---

## Job 3 — the benchmark (the answer to Sam)

Same synthetic TF storm (a chain `world -> link_0 -> ... -> link_{N-1}` published as one
`TFMessage` at rate R, so aggregate load = N·R transforms/s), same machine, one variant
process at a time (`scripts/tf_demos/bench_tf.py`; run it with `pixi run bench-tf`):

* **(a) py** = stock rclpy path: `tf2_ros.Buffer` + `tf2_ros.TransformListener`.
* **(b) cpp** = rclcppyy: `tf2::BufferCore` + C++ `tf2_ros::TransformListener`.
* **(c) idle** = no storm, lookup-only (isolates lookup-call overhead).

**ingest CPU%** = process-wide CPU (all threads, `time.process_time`) to keep the buffer
fed over a 3 s window with the main thread idle. **lookup** rows in the storm scenarios
are measured **under ingest load**.

| scenario        | ingest CPU%  py / cpp | lookup µs med  py / cpp | lookups/s  py / cpp |
|-----------------|:---------------------:|:-----------------------:|:-------------------:|
| idle (no storm) | 0.0 / 0.0             | **7.5 / 1.4**  (5.4×)   | 131 791 / 563 265   |
| 1 k tf/s        | **4.0 / 0.6**  (6.7×) | 7.0 / 1.4               | 131 663 / 531 871   |
| 5 k tf/s        | **12.1 / 1.1** (11×)  | 9.4 / 2.5               | 93 330 / 326 391    |
| 10 k tf/s       | **19.3 / 1.4** (14×)  | 13.5 / 4.5  (3×)        | 59 194 / 192 204    |

(p99 lookup tracks the median: e.g. idle py 9.5 µs / cpp 1.6 µs.)

**Interpretation — honest about where the win is big and where it's marginal:**

- **Ingest is the headline, and the win grows with load.** At 1 k tf/s the Python
  listener already burns ~7× the CPU of the C++ one; by 10 k tf/s it is **~14×**
  (19 % of a core vs ~1.4 %). This is exactly the mechanism in Job 1: the Python path
  deserializes every message into Python and crosses each transform individually under
  the GIL, while the C++ path decodes and inserts wholly in C++.
- **Lookups are cheaper too — even at idle (~5×).** The stock Python `lookup_transform`
  pays two C-extension calls (`can_transform_core` + `lookup_transform_core`) and
  builds a Python `TransformStamped` per call; the rclcppyy path is one cppyy call
  returning a proxy. This part of the win is *independent of tf traffic*.
- **Under load the Python lookup degrades further (7 → 13.5 µs) — a GIL effect.** The
  Python listener's ingest thread holds the GIL, so a Python lookup contends with it;
  the C++ listener never holds the GIL, so rclcppyy lookups stay fast while ingest runs.
- **Where it's marginal:** the *math* is identical (both ultimately call the same
  `tf2::BufferCore`), so for a robot with a *quiet* tf tree and occasional lookups the
  absolute CPU difference is small (sub-1 % either way). The C++ path wins decisively
  when the tf tree is busy (many frames / high rate) or lookups are frequent — i.e.
  precisely the cases where TF cost actually shows up in a profile.

---

## Job 4 — deliverable shape

**`rclcppyy/tf.py`, surfaced as `rclcppyy.tf` — rclcppyy-proper, not a "kit".** Every
existing kit (bt/pcl/ompl/nav2/moveit/control/cv/dbow) wraps a **third-party or opt-in**
library living in its own pixi feature-env. tf2 is **core ROS 2**, shipped in the
default `ros-base` env exactly like rclcpp — so it belongs alongside `bringup_rclcpp` /
`serialization` / `rosbag2_cpp`, not behind an opt-in env. Surface:

```python
import rclcppyy
from rclcppyy import tf

rclcppyy.bringup_rclcpp()
listener = tf.TransformListener()                 # own node + own C++ spin thread
# or tf.TransformListener(node=my_rclcppyy_node)  # attach to an existing node
ts = listener.lookup_transform("world", "sensor", timeout=1.0)
x = ts.transform.translation.x                     # the real geometry_msgs message
ok = listener.can_transform("world", "sensor")
```

- `tf.bringup_tf()` — idempotent headers/libs/glue bringup, returns `(tf2, glue)`.
- `tf.TransformListener(node=None, *, spin_thread=True, cache_time_sec=None)` —
  `lookup_transform`, `can_transform` (both with an optional `timeout=`),
  `set_transform` (seed the buffer directly), `get_frame_names`, `all_frames_as_string`/
  `_yaml`, `close`. `time=` accepts `None` (latest) / seconds / an rclpy·rclcpp `Time`.
- `tf.time_from_sec` / `tf.duration_from_sec`; `tf.TransformException`.
- Mirror-don't-sugar: `lookup_transform` returns the real
  `geometry_msgs::msg::TransformStamped` (the same cppyy proxy the rest of rclcppyy
  uses); the raw `tf2` namespace is available for advanced use.

Demos (`scripts/tf_demos/`, pixi tasks): `demo-tf-lookup` (minimal lookup example),
`demo-tf-storm` (the storm publisher), `bench-tf` (the table above). Tests
(`test/test_tf.py`, 8 tests, in the default suite): numeric composition, can/timeout,
chain, time helpers, and a real network-ingest test.

*Not done (candidates):* `node.tf_listener()` sugar on `RclcppyyNode` (kept out to
respect shared-file ownership — additive, easy follow-up); a `TransformBroadcaster`
helper (the publish side); `lookup_transform_full` (fixed-frame/advanced API);
`transform()` of a stamped datatype (needs `tf2_geometry_msgs` converters).

---

## Generic-lesson candidates for COMMON_PATTERNS (for the lead — not edited here)

1. **Overloaded C++ methods can mis-resolve under cppyy to a *compilable but wrong*
   overload that crashes at runtime (extends §9/§17).** `tf2_ros::Buffer`'s
   `lookupTransform(target, source, TimePoint)` resolved into the `rclcpp::Time`+timeout
   `canTransform`, which called `rclcpp::Clock::now()` and bus-errored. Lesson: when a
   class has a thicket of overloads (a `using`-imported base form + timeout/clock forms),
   prefer the base class with the single unambiguous signature (here `tf2::BufferCore`),
   or wrap the exact call in a `cppdef` free function. A wrong-overload crash has no
   Python traceback — probe it.
2. **A template ctor with a universal-reference default (`NodeT&& node = NodeT()`)
   doesn't resolve from Python** ("class has no public constructors") — third instance of
   "build the object in a small C++ factory" (§6 make_shared, control_kit, nav2). Add it
   to the make_shared bullet.
3. **A library that already spins its own C++ thread is the *ideal* cppyy target
   (sharpens §13).** `tf2_ros::TransformListener(spin_thread=true)` ingests `/tf` on its
   own `std::thread`, entirely off the GIL; Python only crosses on `lookup`. Measured
   ~7–14× less ingest CPU than the equivalent Python listener whose callback runs under
   the GIL — and Python-side lookups don't contend with a Python ingest thread. "Let C++
   own the loop/thread; cross into Python only on demand" is a first-class efficiency
   pattern, not just a deadlock-avoidance one.
4. **Teardown: a C++ object owning an executor + `std::thread` must be released before
   `rclcpp::shutdown()` (third instance of §14/§19).** `register_teardown` a callback
   that drops the listener (its dtor cancels the executor + joins the thread); it runs
   LIFO-before `shutdown_rclcpp`, the correct order. Exit 0 confirmed.
5. **`std::string` inside a returned `std::vector<std::string>` can surface as Python
   `bytes`, not `str` (minor, neighbour of §11).** `getAllFrameNames()` came back as a
   list of `bytes`; decode at the kit boundary.

---

## Recommendation — GO

Sam's assumption is confirmed and then some: the stock rclpy TransformListener feeds
its buffer entirely in Python (deserialize → per-transform Python→C crossing → GIL),
and rclcppyy's C++ listener does it in C++ on its own thread for **~7–14× less ingest
CPU** under load, with **~5× cheaper lookups** as a bonus. It is delivered as
`rclcppyy.tf` — a thin, mirror-don't-sugar helper in rclcppyy-proper — with demos, a
reproducible benchmark, and a fast default-suite test that includes the real
network-ingest path. The friction was two familiar cppyy walls (overload mis-resolution,
ctor-in-C++), both hidden behind ~35 lines of glue.
