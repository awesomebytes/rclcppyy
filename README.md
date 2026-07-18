# rclcppyy

[![CI](https://github.com/awesomebytes/rclcppyy/actions/workflows/ci.yml/badge.svg)](https://github.com/awesomebytes/rclcppyy/actions/workflows/ci.yml)

**Keep the `rclpy` contract; opt into proven C++ paths.** `rclcppyy` is a
compatibility-first accelerator for existing ROS 2 Python software. Its default
profile keeps the exact stock node, context, executor, message classes, and entity
objects, and routes an operation through C++ only when that route has explicit
contract and backend evidence. It is powered by
[**cppyy**](https://cppyy.readthedocs.io), which calls C++ from Python directly via
reflection and just-in-time compilation, and built on
the [**cppyy_kit suite**](https://github.com/awesomebytes/cppyy_kit) (docs:
[awesomebytes.github.io/cppyy_kit](https://awesomebytes.github.io/cppyy_kit/)),
which packages the same "mix Python and C++ with ease" machinery for ROS 2 and a
family of C++ robotics libraries.

![](media/rclcppyy_presentation_logo.jpg)

## What it looks like

Take any ordinary `rclpy` node and add a single line at the top:

```python
import rclcppyy; rclcppyy.enable_cpp_acceleration()

# Everything below remains ordinary rclpy code and ordinary rclpy objects.
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('talker')
pub = node.create_publisher(String, 'chatter', 10)
node.create_timer(0.5, lambda: pub.publish(String(data='hello')))
rclpy.spin(node)
```

`enable_cpp_acceleration()` patches methods on the original `rclpy` classes. The
publisher above remains a stock `rclpy.Publisher` registered with the stock node;
its current C++ route serializes the message with `rclcpp::Serialization<T>` and
publishes the serialized bytes through that same native publisher handle. The
subscription, timer, executor, context, and message class remain stock Python.

## What you get

- Existing node, graph, context, remapping, parameter, executor, and teardown
  behavior stays authoritative in `rclpy`.
- `rclcppyy.status()` reports the backend selected for each routed entity and
  operation.
- `profile="required_cpp"` fails before creating an entity when no certified C++
  route exists, so tests and benchmarks cannot pass through silent fallback.
- The separate native lane exposes `rclcpp` and other C++ libraries directly when
  compatibility is not the primary constraint.
- The [backend selection and promotion guide](docs/backend-selection.md) defines
  when to use compatible, strict, managed native, and fused C++ execution lanes.

The native lane adds lifecycle management but does not replace the C++ API:

```python
from std_msgs.msg import String
import rclcppyy

with rclcppyy.native(["my_program"]) as ros:
    options = ros.rclcpp.NodeOptions()
    node = ros.create_node("native_node", options=options, use_intra_process=True)
    publisher = node.create_publisher(String, "chatter", 10)
    executor = ros.create_executor("multi_threaded", threads=2)
    executor.add_node(node)
    relay = ros.create_fused_pipeline(
        node, String, String, "input", "output",
        'output.data = input.data + ":native";',
        delivery="latest",
    )
```

Nodes, options, publishers, callback groups, and executors in this block are the
real cppyy-backed C++ objects. The session owns a custom `rclcpp::Context`, orders
shutdown, and exposes `ros.rclcpp` as the unrestricted escape hatch. Loaned-message
availability is queried per publisher with `rclcppyy.publisher_capabilities()`.

Measure routes on the target workload rather than assuming that crossing into C++
is automatically faster:

```bash
pixi run bench
```

The benchmark runner requires machine-readable publisher and subscriber backend
evidence before it records a result. A current one-second development smoke for a
tiny String message showed the safe same-handle serialized route was slower than
stock; that route is correctness-certified but is not advertised as a performance
win. Native-message and fused C++ paths must clear workload-specific performance
gates before they are advertised.

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

The default compatible profile keeps the stock contract and records every current
boundary:

**Current C++ route after `enable_cpp_acceleration()`:**

- A stock publisher can serialize through C++ and publish through its existing
  `rcl_publisher_t`; no companion node or publisher is created.

**Stays on stock `rclpy`:**

- Nodes, contexts, graph identity, message classes, subscriptions, timers, spin,
  services, actions, parameters, lifecycle nodes, callback groups, and executors.
- Publisher creation and destruction, QoS, event callbacks, callback groups, and
  override options. Only the prepared publish operation is routed.

**Known walls:**

- Activation patches methods **process-globally and irreversibly**. Call it once
  before creating nodes. Existing Node aliases and subclasses retain their class
  identity because the original class is patched rather than replaced.
- `profile="required_cpp"` currently supports the publisher route and rejects
  subscriptions and timers. `profile="optimized"` reserves explicit
  contract-changing choices; it does not silently enable them.
- `rclcppyy.Node` remains the legacy companion-node prototype. It is not the
  transparent compatibility architecture and should not be used for new
  compatibility work.
- The **first** `rclcpp` bringup JIT-compiles headers. The suite ships a zero-config
  Cling PCH cache (`cppyy_kit` auto-PCH) that makes subsequent process starts far
  cheaper (a warm `rclcpp` bringup measured ~1.73 s → ~0.064 s); see the
  [Freeze & Cache](https://awesomebytes.github.io/cppyy_kit/docs/FREEZE/) docs and the
  [benchmarks page](https://awesomebytes.github.io/cppyy_kit/docs/benchmarks/).
- Acceleration is opt-in per process; without the one-line call, your code is
  ordinary `rclpy`.

### Inspect backend decisions

`rclcppyy.status()` returns a JSON-serializable process snapshot of the backend
selected for accelerated nodes, entities, and operations:

```python
report = rclcppyy.status()
for entity in report["entities"]:
    print(entity["backend"], entity["reason"], entity["policies"])
```

Each decision reports `cpp`, `python`, or `unsupported`, along with active
policies and value-only metadata such as the entity type, topic, and message
type. History is bounded to the latest 256 records per category; aggregate
counts and dropped-record counts remain available for long-running processes.

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

### Activate unedited software with `RCLCPPYY_ENABLE_HOOK`

`enable_cpp_acceleration()` normally has to be called from inside your process. To
activate a process you cannot edit, including stock **`ros2` CLI** commands, rclcppyy
ships an **opt-in startup hook**. Install it once, then set
`RCLCPPYY_ENABLE_HOOK=1` to enable the current certified routes with **zero code
changes**:

```bash
python -m rclcppyy.hook install          # once per environment (uninstall/status too)

RCLCPPYY_ENABLE_HOOK=1 ros2 topic hz /some_topic  # activated; inspect status for routes
ros2 topic hz /some_topic                   # env var unset -> ordinary rclpy, untouched
```

`RCLCPPYY_ENABLE_HOOK` is the single control: `RCLCPPYY_ENABLE_HOOK=1` turns the
hook on for a process; unset, `0`, or any other value leaves it off. `install`
writes a `.pth` into the environment's site-packages that runs at every interpreter
start; when the hook is off it is a near-zero-cost no-op, and `python -m
rclcppyy.hook uninstall` removes it.

At interpreter start the `.pth` registers a post-import hook on `rclpy`; the first
`import rclpy` triggers `enable_cpp_acceleration()`, after rclpy is importable but
before the tool builds its node. The current compatibility architecture leaves
`rclpy.spin_once`, subscriptions, and executors stock. The startup hook therefore
provides zero-edit activation and backend reporting, but it must not be described as
a C++ subscription/executor route until that route has current contract evidence.

Older measurements for this hook used a companion-node C++ subscription/executor
prototype. They do not describe the current stock-authority architecture and have
therefore been removed. Current startup-hook claims are limited to activation,
object/behavior compatibility, backend reporting, and whichever individual routes
are certified in the compatibility manifest.

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
