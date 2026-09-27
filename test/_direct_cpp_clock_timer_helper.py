#!/usr/bin/env python3
"""Live sim-time, wall-time, type, and destroy proof for the ROS-clock
create_timer default (rclcpp::create_timer over the node's own clock).

Mirrors _direct_cpp_sim_time_helper.py's /clock driver: unlike the clock-sleep
and rate proofs, nothing here blocks on a single uninterrupted native call, so
an ordinary Python executor spin_once loop drives /clock with no GIL wrinkle.
"""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("rosgraph_msgs/msg/Clock",))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered clock timer inspection")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


rclpy.init(args=[])
executor = SingleThreadedExecutor()
sim_node = Node(
    "direct_clock_timer_sim_%d" % os.getpid(),
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
pub_node = Node("direct_clock_timer_pub_%d" % os.getpid())
executor.add_node(sim_node)
executor.add_node(pub_node)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
clock_publisher = pub_node.create_publisher(Clock, "/clock", qos)
message = Clock()

SIM_PERIOD_S = 0.5
SIM_PERIOD_NS = int(SIM_PERIOD_S * 1e9)

# 0. Type/route: the default create_timer() is a GenericTimer on the node's
# own clock now, not a WallTimer.
frozen_ticks = []
frozen_timer = sim_node.create_timer(SIM_PERIOD_S, lambda: frozen_ticks.append(1))
assert "GenericTimer" in frozen_timer.native_type_name
assert frozen_timer.creation_route == "rclcpp_clock_timer"
print("DIRECT_CPP_CLOCK_TIMER_TYPE_OK")

# 1. Negative control: /clock is never published, so sim time stays frozen at
# its zero starting value (matching the sim-time lane's own finding) and the
# 0.5s-sim-period timer must never fire even though real wall time passes
# well beyond 0.5s.
frozen_deadline = time.monotonic() + 1.5
while time.monotonic() < frozen_deadline:
    executor.spin_once(timeout_sec=0.05)
assert frozen_ticks == [], "clock timer fired under frozen sim time"
sim_node.destroy_timer(frozen_timer)
print("DIRECT_CPP_CLOCK_TIMER_SIM_FROZEN_OK")

# 2. Positive control: advance /clock in explicit steps; the timer must fire
# only once sim time crosses the period, not before.
tick_sim_ns = []
tick_timer = sim_node.create_timer(
    SIM_PERIOD_S, lambda: tick_sim_ns.append(sim_node.get_clock().now().nanoseconds))
step_ns = 100_000_000
current_ns = 0
tick_deadline = time.monotonic() + 10.0
while not tick_sim_ns and time.monotonic() < tick_deadline:
    current_ns += step_ns
    message.clock.sec = current_ns // 1_000_000_000
    message.clock.nanosec = current_ns % 1_000_000_000
    clock_publisher.publish(message)
    executor.spin_once(timeout_sec=0.05)
assert tick_sim_ns, "clock timer never fired as sim time advanced"
assert tick_sim_ns[0] >= SIM_PERIOD_NS, (
    "clock timer fired before sim time reached its period: %d" % tick_sim_ns[0])
sim_node.destroy_timer(tick_timer)
print("DIRECT_CPP_CLOCK_TIMER_SIM_TICK_OK sim_ns_at_first_fire=%d" % tick_sim_ns[0])

# 3. A second, non-sim node: the ROS clock follows system time when
# use_sim_time is off, so a short-period timer fires within a bounded wall
# interval exactly like a plain executor-driven periodic timer always has.
wall_node = Node("direct_clock_timer_wall_%d" % os.getpid())
executor.add_node(wall_node)
wall_ticks = []
wall_timer = wall_node.create_timer(0.05, lambda: wall_ticks.append(1))
wall_deadline = time.monotonic() + 5.0
while not wall_ticks and time.monotonic() < wall_deadline:
    executor.spin_once(timeout_sec=0.05)
assert wall_ticks, "wall-clock (use_sim_time=False) timer never fired"
wall_node.destroy_timer(wall_timer)
print("DIRECT_CPP_CLOCK_TIMER_WALL_OK")

# 4. Destroy: the facade holds the sole strong native reference, so
# destroying it stops delivery even though the executor keeps spinning.
destroy_ticks = []
destroy_timer = wall_node.create_timer(0.02, lambda: destroy_ticks.append(1))
fire_deadline = time.monotonic() + 5.0
while not destroy_ticks and time.monotonic() < fire_deadline:
    executor.spin_once(timeout_sec=0.05)
assert destroy_ticks, "clock timer never fired before the destroy proof"
assert wall_node.destroy_timer(destroy_timer)
count_at_destroy = len(destroy_ticks)
for _ in range(10):
    executor.spin_once(timeout_sec=0.02)
assert len(destroy_ticks) == count_at_destroy, "destroyed clock timer kept firing"
print("DIRECT_CPP_CLOCK_TIMER_DESTROY_OK")

print("DIRECT_CPP_CLOCK_TIMER_NO_CONVERSION_OK")

pub_node.destroy_publisher(clock_publisher)
sim_node.destroy_node()
pub_node.destroy_node()
wall_node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_CLOCK_TIMER_TEARDOWN_OK")
