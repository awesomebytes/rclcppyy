#!/usr/bin/env python3
"""Live wall-time, sim-time, interrupt, and identity proof for clock sleep.

``rclcpp::Clock::sleep_for``/``sleep_until`` are single, uninterrupted native
calls with no intermediate return to the Python interpreter loop (unlike
``spin()``, which repeats bounded native waits and so yields the GIL between
iterations): a direct call from Python holds the GIL for the entire blocked
duration. Driving sim time or a context shutdown *while* such a call is
blocked therefore cannot go through any other Python-level call on another
thread -- that call would simply queue for the GIL behind the blocked one.
This helper drives both from small, self-contained native (non-Python)
threads compiled here, mirroring how the suite's own
``_native_clock_sleep_helper.py`` proves the same primitive.
"""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("rosgraph_msgs/msg/Clock",))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.clock import Clock  # noqa: E402
from rclpy.clock_type import ClockType  # noqa: E402
from rclpy.context import Context as _StockContext  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.exceptions import NotInitializedException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.time import Time  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered clock sleep")


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


from rclcpp_kit.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcpp_kit.direct_message_types import load_message_type  # noqa: E402
from rclcpp_kit import direct_entities  # noqa: E402
from rclcppyy import direct_cpp  # noqa: E402


bringup_rclcpp()
clock_message_type = load_message_type("rosgraph_msgs", "Clock")
cppyy.cppdef(
    r"""
    #include <chrono>
    #include <cstdint>
    #include <memory>
    #include <thread>
    #include <rclcpp/rclcpp.hpp>
    #include "%s"

    namespace direct_cpp_clock_sleep_probe {

    // Both helpers below run their blocking/delayed work on a detached,
    // purely-native std::thread: no Python or cppyy call happens on that
    // thread, so it never touches the GIL and can make the process's other
    // (GIL-holding, Python-blocked) thread progress regardless.

    void shutdown_after(std::shared_ptr<rclcpp::Context> context, int64_t delay_ms)
    {
      std::thread([context, delay_ms]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        context->shutdown("direct_cpp_clock_sleep_probe interrupt test");
      }).detach();
    }

    void publish_clock_after(
        std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> publisher,
        int32_t sec,
        uint32_t nanosec,
        int64_t delay_ms)
    {
      std::thread([publisher, sec, nanosec, delay_ms]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        rosgraph_msgs::msg::Clock message;
        message.clock.sec = sec;
        message.clock.nanosec = nanosec;
        publisher->publish(message);
      }).detach();
    }

    }  // namespace direct_cpp_clock_sleep_probe
    """ % clock_message_type.header
)
probe = cppyy.gbl.direct_cpp_clock_sleep_probe


rclpy.init(args=[])
runtime = direct_cpp._runtime()

# 1. Wall-time sleep_for, plus an in-process stock differential: rclpy.clock.Clock
# is never patched by direct_cpp (finding #8), so a bare stock Clock genuinely
# runs unmodified rclpy code right here, in the same process.
node = Node("direct_clock_sleep_%d" % os.getpid())
clk = node.get_clock()
start = time.monotonic()
result = clk.sleep_for(Duration(seconds=0.2))
elapsed = time.monotonic() - start
assert result is True
assert 0.15 <= elapsed <= 2.0, "sleep_for(0.2s) took %.3fs" % elapsed

# direct_cpp never touches rclpy.context.Context (only Node.context/rclpy.init
# route through the direct runtime's own _DirectContext), so a stock Context,
# explicitly constructed and initialized here, is a genuine independent stock
# context -- Clock.sleep_for's own default (get_default_context()) is never
# populated under direct_cpp, hence the explicit context= below.
stock_context = _StockContext()
stock_context.init(args=[])
stock_clock = Clock(clock_type=ClockType.STEADY_TIME)
stock_start = time.monotonic()
stock_result = stock_clock.sleep_for(Duration(seconds=0.2), context=stock_context)
stock_elapsed = time.monotonic() - stock_start
assert stock_result is True
assert 0.15 <= stock_elapsed <= 2.0, "stock sleep_for(0.2s) took %.3fs" % stock_elapsed
stock_context.shutdown()
print("DIRECT_CPP_CLOCK_SLEEP_FOR_OK")

# 2. Native identity: the sleeper retains the exact same rclcpp::Clock the node
# clock wraps.
raw_clock = node._direct_cpp_node.get_clock()
sleeper = node._direct_cpp_sleeper
assert sleeper is not None
assert sleeper.clock_address == cppyy.addressof(raw_clock)
del raw_clock
print("DIRECT_CPP_CLOCK_SLEEP_NATIVE_IDENTITY_OK")
node.destroy_node()

# 3. Sim-time sleep_until: driven entirely natively (a session executor thread
# spinning the sim node's native subscription, and a one-shot delayed native
# publish), so the GIL-holding Python-level sleep_until call on the main
# thread never needs to interleave with anything else.
sim_node = Node(
    "direct_clock_sleep_sim_%d" % os.getpid(),
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
pub_node = Node("direct_clock_sleep_clock_pub_%d" % os.getpid())

native_executor = runtime.session.create_executor("single_threaded")
native_executor.add_node(sim_node._direct_cpp_node)
executor_thread = runtime.session.start_executor(native_executor)

qos = runtime.session.rclcpp.QoS(1)
qos.best_effort()
publisher = direct_entities.create_managed_publisher(
    pub_node._direct_cpp_node, clock_message_type.cpp_type, "/clock", qos)

deadline = time.monotonic() + 10.0
while publisher.entity().get_subscription_count() < 1 and time.monotonic() < deadline:
    time.sleep(0.01)
assert publisher.entity().get_subscription_count() >= 1

sim_clock = sim_node.get_clock()
assert sim_clock.ros_time_is_active

TARGET_SEC = 5
target_time = Time(
    nanoseconds=TARGET_SEC * 1_000_000_000, clock_type=ClockType.ROS_TIME)
probe.publish_clock_after(publisher.entity(), TARGET_SEC, 0, 300)

start = time.monotonic()
sim_result = sim_clock.sleep_until(target_time)
sim_elapsed = time.monotonic() - start
assert sim_result is True
# Genuinely waited for the delayed native publish rather than resolving on
# stale/steady time: well under the 300 ms delay would mean it did not.
assert sim_elapsed >= 0.2, "sim sleep_until resolved too early: %.3fs" % sim_elapsed
assert sim_elapsed < 10.0, "sim sleep_until took too long: %.3fs" % sim_elapsed
assert sim_clock.now().nanoseconds >= target_time.nanoseconds
print("DIRECT_CPP_CLOCK_SLEEP_SIM_OK")

executor_thread.close()
sim_node.destroy_node()
pub_node.destroy_node()

# 4. Context-shutdown interrupt. Runs last: it tears down the shared context,
# so nothing further can be created afterward.
interrupt_node = Node("direct_clock_sleep_interrupt_%d" % os.getpid())
interrupt_clock = interrupt_node.get_clock()
LONG_SLEEP_S = 10.0  # never meant to complete normally
probe.shutdown_after(runtime.session.context, 200)

start = time.monotonic()
interrupt_result = interrupt_clock.sleep_for(Duration(seconds=LONG_SLEEP_S))
interrupt_elapsed = time.monotonic() - start
assert interrupt_result is False
assert interrupt_elapsed < 3.0, (
    "context shutdown did not wake the sleep early: %.3fs" % interrupt_elapsed)
print("DIRECT_CPP_CLOCK_SLEEP_INTERRUPT_OK")

# 5. A fresh sleep_for on the now-shut-down context fails closed up front,
# matching stock's own NotInitializedException.
try:
    interrupt_clock.sleep_for(Duration(seconds=0.01))
except NotInitializedException:
    pass
else:
    raise AssertionError("sleep_for succeeded after context shutdown")
print("DIRECT_CPP_CLOCK_SLEEP_NOTINIT_OK")
