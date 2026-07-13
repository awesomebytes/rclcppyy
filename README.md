# rclcppyy

[![CI](https://github.com/awesomebytes/rclcppyy/actions/workflows/ci.yml/badge.svg)](https://github.com/awesomebytes/rclcppyy/actions/workflows/ci.yml)

**Prototype in Python, run at C++ speed.** `rclcppyy` is a drop-in accelerator for
`rclpy`: add one line at the top of an existing ROS 2 Python node and its
publishers, subscriptions, timers, and messages run on the `rclcpp` C++ backend
instead — no rewrite, no bindings to maintain, no Python⇄C++ message copies on the
hot path. It is powered by [**cppyy**](https://cppyy.readthedocs.io), which calls
C++ from Python directly via reflection and just-in-time compilation, and built on
the [**cppyy_kit suite**](https://github.com/awesomebytes/cppyy_kit) (docs:
[awesomebytes.github.io/cppyy_kit](https://awesomebytes.github.io/cppyy_kit/)),
which packages the same "mix Python and C++ with ease" machinery for ROS 2 and a
family of C++ robotics libraries.

![](media/rclcppyy_presentation_logo.jpg)

## What it looks like

Take any ordinary `rclpy` node and add a single line at the top:

```python
import rclcppyy; rclcppyy.enable_cpp_acceleration()

# Everything below is unchanged rclpy — but now runs on the rclcpp C++ backend.
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('talker')          # actually an rclcpp-backed node
pub = node.create_publisher(String, 'chatter', 10)
node.create_timer(0.5, lambda: pub.publish(String(data='hello')))
rclpy.spin(node)
```

`enable_cpp_acceleration()` monkeypatches `rclpy` so `create_node`, `spin`,
publishers, subscriptions, and wall timers are served by `rclcpp`, and message
classes (e.g. `std_msgs.msg.String`) resolve to their C++ types — the payload lives
as a C++ object end to end, so there is no per-message Python conversion.

## What you get

Publishing and subscribing on the C++ backend uses **a fraction of the CPU** of
plain `rclpy` at the same message rate, because the hot path (executor, DDS calls,
message handling) runs in C++ instead of the Python interpreter. In the run below —
a small `std_msgs/String` at 1 kHz on the reference machine — the monkeypatched
node used **roughly 4–6× less CPU** than `rclpy` (publisher *and* subscriber), at
the same throughput and about half the latency. Measure it on your own machine in
one command:

```bash
pixi run bench     # rclpy vs rclcppyy CPU comparison table (1 kHz + 10 kHz)
```

Example output (absolute numbers vary by machine — reproduce it yourself with the
command above):

```
  Benchmark @ 1000 Hz target
  =======================================================================================
  variant                   pub CPU%  sub CPU%  msgs recv  eff Hz    dropped  avg lat us
  ---------------------------------------------------------------------------------------
  rclpy                     17.9      19.2      6000       968.9     0        174.8
  rclcppyy (monkeypatched)  4.4       3.4       6000       969.4     0        89.0
```

For the full, consolidated and freshly-measured benchmark set — across the whole
suite, including the freeze/AOT optimization ladder — see the
**[cppyy_kit benchmarks page](https://awesomebytes.github.io/cppyy_kit/docs/benchmarks/)**.

## Install (pixi / conda — no build needed)

`rclcppyy` 0.2.0 is published as `ros-jazzy-rclcppyy` on the prefix.dev
[`awesomebytes` channel](https://prefix.dev/channels/awesomebytes). To *use* it (no clone, no `colcon build`), add the channel
and the package to your own pixi workspace:

```toml
# pixi.toml
[workspace]
channels = ["https://repo.prefix.dev/awesomebytes", "robostack-jazzy", "conda-forge"]
platforms = ["linux-64"]

[dependencies]
ros-jazzy-rclcppyy = "*"
```

Or in one line:

```bash
pixi add -c https://repo.prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge ros-jazzy-rclcppyy
```

Then `import rclcppyy; rclcppyy.enable_cpp_acceleration()` works out of the box — no
`LD_LIBRARY_PATH` or activation setup required. Message packages you publish or
subscribe (e.g. `ros-jazzy-std-msgs`) are separate dependencies, as in any ROS 2
project. Installing rclcppyy pulls its runtime deps `ros-jazzy-rclcpp-kit` and
`cppyy-kit` (the suite) transitively.

## What accelerates, and what stays rclpy

`rclcppyy` accelerates the pub/sub hot path and keeps everything else on stock
`rclpy`, so unpatched code keeps working. Be aware of the boundaries:

**Runs on the C++ backend after `enable_cpp_acceleration()`:**

- Nodes created via `rclpy.create_node(...)` (returned as C++-backed nodes)
- Publishers, subscriptions, and wall timers on those nodes
- `rclpy.spin(node)` (delegates to `rclcpp::spin`)
- Message types — resolved to their `rclcpp` C++ equivalents on import, kept as C++
  objects with no Python⇄C++ conversion on publish/receive

**Stays on stock rclpy (not accelerated):**

- Services and actions
- Parameters, parameter services, and parameter events
- Custom executors, callback groups, and multi-threaded spinning — acceleration
  targets the default single-threaded spin
- Publisher/subscription event callbacks and QoS overriding options

**Known walls:**

- `enable_cpp_acceleration()` monkeypatches `rclpy` **process-globally and
  irreversibly** — call it once, at the very top, before creating nodes.
- The **first** `rclcpp` bringup JIT-compiles headers. The suite ships a zero-config
  Cling PCH cache (`cppyy_kit` auto-PCH) that makes subsequent process starts far
  cheaper (a warm `rclcpp` bringup measured ~1.73 s → ~0.064 s); see the
  [Freeze & Cache](https://awesomebytes.github.io/cppyy_kit/docs/FREEZE/) docs and the
  [benchmarks page](https://awesomebytes.github.io/cppyy_kit/docs/benchmarks/).
- Acceleration is opt-in per process; without the one-line call, your code is
  ordinary `rclpy`.

## Powered by the cppyy_kit suite

As of **0.2.0**, `rclcppyy` is the ROS 2 drop-in *product* in a larger family. The
reusable machinery it pioneered was extracted into the
**[cppyy_kit suite](https://github.com/awesomebytes/cppyy_kit)** — a set of "kits",
where a *kit* is a thin layer that mirrors a C++ library's own API and hides only
the `cppyy` friction (bringup, object lifetime, crossing callbacks, teardown).
`rclcppyy` now **depends on the suite at runtime** and re-exports the moved pieces
through **deprecation shims**, so existing imports (`from rclcppyy.bringup_rclcpp
import ...`, `rclcppyy.tf`, `from rclcppyy.kits import bt_kit`, …) keep working with a
`DeprecationWarning` pointing at the new home. **New code should import the suite
packages directly.**

| Kit | Import / conda package | What it gives you | Docs |
|---|---|---|---|
| `cppyy_kit` | `cppyy_kit` / `cppyy-kit` | ROS-free base: load / keep-alive / callback / teardown primitives, the `freeze` PCH tooling, compile cache | [page](https://awesomebytes.github.io/cppyy_kit/kits/cppyy_kit/) |
| `rclcpp_kit` | `rclcpp_kit` / `ros-jazzy-rclcpp-kit` | ROS 2 core: rclcpp bringup, C++ message resolution/conversion, serialization, rosbag2, **tf** | [page](https://awesomebytes.github.io/cppyy_kit/rclcpp_kit/) |
| `bt_kit` | `bt_kit` / `ros-jazzy-bt-kit` | [BehaviorTree.CPP](https://www.behaviortree.dev/) v4 from Python | [page](https://awesomebytes.github.io/cppyy_kit/bt_kit/WHY/) |
| `pcl_kit` | `pcl_kit` / `ros-jazzy-pcl-kit` | [PCL](https://pointclouds.org/) — clouds stay in C++ end to end | [page](https://awesomebytes.github.io/cppyy_kit/pcl_kit/WHY/) |
| `ompl_kit` | `ompl_kit` / `ros-jazzy-ompl-kit` | [OMPL](https://ompl.kavrakilab.org/) motion planning | [page](https://awesomebytes.github.io/cppyy_kit/ompl_kit/WHY/) |
| `nav2_kit` | `nav2_kit` / `ros-jazzy-nav2-kit` | [Nav2](https://docs.nav2.org/) cores (Costmap2D + NavFn), no lifecycle servers | [page](https://awesomebytes.github.io/cppyy_kit/nav2_kit/WHY/) |
| `moveit_kit` | `moveit_kit` / `ros-jazzy-moveit-kit` | the full [MoveIt 2](https://moveit.ai/) C++ API from Python | [page](https://awesomebytes.github.io/cppyy_kit/moveit_kit/WHY/) |
| `control_kit` | `control_kit` / `ros-jazzy-control-kit` | a Python [ros2_control](https://control.ros.org/) controller in the real controller_manager | [page](https://awesomebytes.github.io/cppyy_kit/control_kit/WHY/) |
| `cv_kit` | `cv_kit` / `ros-jazzy-cv-kit` | OpenCV C++ with a zero-copy `sensor_msgs/Image` → `cv::Mat` bridge | [page](https://awesomebytes.github.io/cppyy_kit/cv_kit/WHY/) |
| `dbow_kit` | `dbow_kit` / `ros-jazzy-dbow-kit` | [DBoW2](https://github.com/dorian3d/DBoW2) place recognition / loop closure | [page](https://awesomebytes.github.io/cppyy_kit/dbow_kit/WHY/) |

The [cppyy_kit docs site](https://awesomebytes.github.io/cppyy_kit/) also carries the
common-pattern playbook, the L0→L1→L2 freeze/AOT ladder, a vision loop-closure
tutorial, per-kit benchmarks, and a `cppyy-accelerate` skill for driving a coding
agent to speed up existing Python.

## Development

The repo is a self-contained [pixi](https://pixi.sh) workspace — the manifest
(`pixi.toml`) and lockfile (`pixi.lock`) live here, so `pixi install` reproduces the
exact environment (ROS 2 Jazzy from robostack, `cppyy` from conda-forge, the suite
from the `awesomebytes` channel, compilers, colcon). No manual steps.

```bash
# If you haven't installed pixi:
curl -fsSL https://pixi.sh/install.sh | sh && source ~/.bashrc

git clone https://github.com/awesomebytes/rclcppyy
cd rclcppyy
pixi install        # downloads the environment (a few GB the first time)
pixi run build      # colcon build --packages-select rclcppyy
```

Inside `pixi shell` the workspace overlay is sourced automatically and the
recommended middleware defaults are already set
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) —
the ROS default fastrtps has intermittent latency issues and drops big messages, so
cyclonedds on LOCALHOST is the default here.

Tasks:

| Task | What it does |
|---|---|
| `pixi run build` | `colcon build --packages-select rclcppyy` into `install/` |
| `pixi run test` | `pytest test/` (bringup, monkeypatch, pub/sub roundtrip, serialization parity, tf, clean-exit, kit shims) |
| `pixi run lint` | `flake8 rclcppyy test` |
| `pixi run clean` | remove `build/ install/ log/` |
| `pixi run bench` | rclpy-vs-rclcppyy CPU comparison table (1 kHz + 10 kHz) |
| `pixi run demo-tutorial` | the rclpy pub/sub tutorial, on the rclcppyy C++ backend |
| `pixi run demo-pubsub` | a live pub/sub pair (rclcppyy backend), stats streamed |

`pixi run bench` spawns each publisher/subscriber pair as separate child processes,
warms up (the rclcppyy variants JIT-compile `rclcpp` on first bringup, excluded from
the measurement), samples each process's CPU with `psutil` while parsing the
subscriber's throughput/latency, kills the children itself, and prints a table per
rate. Flags pass straight through the pixi task:

```bash
pixi run bench --rate 5000 --duration 10                       # custom rate / window
pixi run bench --variants rclpy,rclcppyy,rclcppyy-templated    # add the pure-cppyy pair
pixi run bench --json                                          # machine-readable output
```

<details>
<summary>Advanced: run the individual bench scripts by hand</summary>

Enter the environment with `pixi shell` (middleware defaults and the workspace
overlay are applied automatically), then, one per shell:

```bash
# rclpy baseline
ros2 run rclcppyy bench_pub_rclpy.py 10000
ros2 run rclcppyy bench_sub_rclpy.py

# rclcppyy (monkeypatched rclpy → C++ backend)
ros2 run rclcppyy bench_pub_rclcppyy_monkeypatch.py 10000
ros2 run rclcppyy bench_sub_rclcppyy_monkeypatched.py

# Monitor with:  top -c -p $(pgrep -d, -f bench_)
```

Without entering a shell, any command can be run through pixi directly, e.g.
`pixi run ros2 run rclcppyy bench_sub_rclcppyy_monkeypatched.py`.
</details>

### Accelerate the *real* `ros2 topic hz` with `RCLCPPYY_ENABLE_HOOK`

`enable_cpp_acceleration()` normally has to be called from inside your process. To
accelerate a process you cannot edit — notably the stock **`ros2` CLI** — rclcppyy
ships an **opt-in startup hook**. Install it once, then set `RCLCPPYY_ENABLE_HOOK=1` and
the stock `ros2 topic hz` (and other rclpy-based verbs) runs on the C++ backend with
**zero code changes**:

```bash
python -m rclcppyy.hook install          # once per environment (uninstall/status too)

RCLCPPYY_ENABLE_HOOK=1 ros2 topic hz /some_topic  # runs on the rclcpp backend
ros2 topic hz /some_topic                   # env var unset -> ordinary rclpy, untouched
```

`RCLCPPYY_ENABLE_HOOK` is the single control: `RCLCPPYY_ENABLE_HOOK=1` turns the
hook on for a process; unset, `0`, or any other value leaves it off. `install`
writes a `.pth` into the environment's site-packages that runs at every interpreter
start; when the hook is off it is a near-zero-cost no-op, and `python -m
rclcppyy.hook uninstall` removes it.

At interpreter start the `.pth` registers a post-import hook on `rclpy`; the first
`import rclpy` triggers `enable_cpp_acceleration()`, after rclpy is importable but
before the tool builds its node. `ros2 topic hz` drives its loop with
`rclpy.spin_once` (in ros2cli's `DirectNode` discovery loop and in the hz loop), so
`enable_cpp_acceleration()` also patches `rclpy.spin_once` to run an rclcpp executor;
otherwise the CLI would stall at node bring-up.

**Measured** (stock `ros2 topic hz` vs the same binary under
`RCLCPPYY_ENABLE_HOOK=1`, against a C++ publisher of a `sensor_msgs/Image`,
BEST_EFFORT, `--window 100`, with `net.core.rmem_max` raised (see below);
`pixi run -e heavydemo demo-topic-hz-cli`; numbers vary by machine and run):

| payload | target Hz | rclpy Hz | rclcppyy Hz | rclpy CPU % | rclcppyy CPU % |
|---|---|---|---|---|---|
| 3.0 MB  | 100  | 99.0   | 97.0   | 48.7  | 18.8 |
| 3.0 MB  | 200  | 195.1  | 197.0  | 92.2  | 34.5 |
| 3.0 MB  | 300  | 291.4  | 296.7  | 102.7 | 42.6 |
| 0.05 MB | 1000 | 998.2  | 999.7  | 19.3  | 7.3  |
| 0.05 MB | 3000 | 2967.6 | 2985.0 | 46.6  | 17.2 |

Both backends deliver essentially the publisher's rate wherever the publisher can
sustain it; the difference is CPU — the accelerated backend uses about a third to a
half as much. `ros2 topic hz` (without `--filter`) subscribes with `raw=True`, so
stock rclpy does not deserialise the payload; the saving is the C++ executor and
message handling replacing the Python executor, not avoiding a per-message copy.
Delivered rates stay close because the raw stock reader is cheap enough to keep up
where the publisher can feed it — but note where the CPU lands: on the 3 MB stream
stock rclpy crosses one full core near 200–250 Hz (92–107 %), while the accelerated
backend stays around a third of a core, so on a busier machine or at a higher rate
the stock reader saturates first.

**Startup: a one-time cost.** Time to first hz line
(`pixi run -e heavydemo demo-topic-hz-startup`):

| `ros2 topic hz` first hz line | time |
|---|---|
| stock (no acceleration) | ~1.7 s |
| `RCLCPPYY_ENABLE_HOOK=1`, cold cache | ~8.3 s |
| `RCLCPPYY_ENABLE_HOOK=1`, warm cache | ~4.1 s |

Two caches cut the warm start: the suite's Cling PCH removes the `rclcpp` header
parse, and a per-message-type subscription cache removes the ~2.8 s one-time JIT of
`create_subscription<MsgT>` (both built on the first run and loaded on later ones,
cold → warm above). The warm figure is still above stock because the rest of the
rclcpp bring-up — cppyy symbol discovery and the rclpy-compatibility adapters — is
not yet cached. Acceleration therefore remains a one-time startup cost that pays off
over a long-running `hz` session rather than a single short call. (Needs the newer
suite; see the dev-bridge note below.)

### Large messages under BEST_EFFORT QoS

`ros2 topic hz` subscribes with `qos_profile_sensor_data` (BEST_EFFORT). A 3 MB
image fragments into roughly two thousand UDP datagrams, and if the OS socket
receive buffer is smaller than one message a single dropped fragment loses the whole
message. On a stock Linux install `net.core.rmem_max` defaults to about 200 KB, so a
BEST_EFFORT reader receives little or nothing on a 3 MB topic — for stock rclpy and
for the accelerated backend alike, since both use the same reader QoS. Raising the
kernel limit resolves it (this needs root, and applies to any DDS user, not only
rclcppyy):

```bash
sudo sysctl -w net.core.rmem_max=2147483647
```

With that in place, CycloneDDS's default socket buffer is large enough that a 3 MB
BEST_EFFORT topic is received normally; no `CYCLONEDDS_URI` tuning is required. The
3 MB figures above were measured with this setting.

The startup hook is covered by `test/test_hook.py` (install / uninstall / status;
`RCLCPPYY_ENABLE_HOOK=1` accelerates a fresh `import rclpy`; unset and `=0` leave
stock rclpy untouched). For the acceleration on a node that *reads* message fields —
where stock rclpy pays a per-message Python deserialisation that `ros2 topic hz`
(raw subscription) does not — `scripts/heavy_hz_demo/run_heavy_hz.py` is a controlled
publisher/subscriber harness (`pixi run -e heavydemo demo-heavy-hz`).

> **Dev bridge.** The startup measurement and the fastest bring-up use the
> zero-config auto-PCH, which postdates the published suite 0.1.0. Until the next
> suite release the `heavydemo` pixi env bridges the newer `rclcpp_kit`/`cppyy_kit`
> from a sibling `cppyy_kit` source checkout (default `../cppyy_kit`, override
> `CPPYY_KIT_SRC`) and isolates its PCH cache under `.heavy_demo_cache/`. It shares
> the default solve (no extra conda deps) and leaves the default env's
> published-channel dependency untouched (`workspace_activation.sh`, gated on the
> `heavydemo` env).

### Extra demos (optional env)

Heavier example scripts (OpenCV, PCL, GStreamer, typer, hypothesis) live in an
optional `demos` environment so the default `pixi install` stays lean. Enable it
once, then run any demo task with `pixi run -e demos <task>`:

```bash
pixi install -e demos       # adds opencv, pcl, pcl_conversions, eigen,
                            # gstreamer, pygobject, typer, hypothesis
```

| Task | What it does |
|---|---|
| `pixi run -e demos demo-images` | Publishes a real JPEG as a `sensor_msgs/Image` at 4 Hz through the C++ backend (builds the message straight into a C++ `std::vector`, no copy). Pair with `pixi run -e demos python scripts/big_messages_demos/images_sub.py` to receive it. |
| `pixi run -e demos demo-hypothesis` | Drives a JIT-compiled C++ function with [Hypothesis](https://hypothesis.readthedocs.io) property-based testing. It intentionally finds the failing input (`"DDS"`) and exits non-zero — that's the demo. |
| `pixi run -e demos demo-pointcloud-voxelgrid` | JIT-compiles a PCL `VoxelGrid` filter (via `pcl_conversions`) and runs it as a ROS 2 node, waiting for `PointCloud2` on `/lexus3/os_center/points` (needs an external bag to downsample anything). |

Additional example scripts under `roscon_uk_2025/` need external data not in this
repo (a live `PointCloud2` source, the [cloudini](https://github.com/facontidavide/cloudini)
compressor workspace, or a V4L2 camera) and therefore have no `pixi run` task — see
the scripts' own comments for setup.

## Releasing

`rclcppyy` depends on the published cppyy_kit suite, which creates a publish order
(suite first, then the product). See [`RELEASING.md`](RELEASING.md) for the release
choreography and the `rclcppyy.*` / `rclcppyy.kits.*` deprecation timeline.

## License

BSD 3-Clause — see [`LICENSE`](LICENSE).
