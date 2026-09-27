#!/usr/bin/env python3
"""Live period, sim-time, interrupt, and destroy proof for DirectRate.

Mirrors ``_direct_cpp_clock_sleep_helper.py``'s native-thread driving
technique: ``DirectRate.sleep()`` ultimately calls the same
``NativeClockSleeper.sleep_until`` primitive, a single uninterrupted native
call that holds the GIL for its whole blocked duration, so sim time and the
context-shutdown interrupt are driven from small, self-contained native
(non-Python) threads compiled here rather than a second Python thread.
"""

import importlib
import os
import subprocess
import sys
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("rosgraph_msgs/msg/Clock",))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
import rclpy.timer  # noqa: E402
from rclpy.exceptions import ROSInterruptException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered rate inspection")


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

    namespace direct_cpp_rate_probe {

    void shutdown_after(std::shared_ptr<rclcpp::Context> context, int64_t delay_ms)
    {
      std::thread([context, delay_ms]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        context->shutdown("direct_cpp_rate_probe interrupt test");
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

    }  // namespace direct_cpp_rate_probe
    """ % clock_message_type.header
)
probe = cppyy.gbl.direct_cpp_rate_probe


rclpy.init(args=[])
runtime = direct_cpp._runtime()

# 0. Argument validation (mirrors _direct_cpp_timer_helper.py's own
# expect_rejected style for create_timer's analogous checks).
node = Node("direct_rate_%d" % os.getpid())


def expect_rejected(operation, *exceptions):
    try:
        operation()
    except exceptions:
        pass
    else:
        raise AssertionError("unsupported direct rate request succeeded")


expect_rejected(lambda: node.create_rate(0), ValueError)
expect_rejected(lambda: node.create_rate(-1.0), ValueError)
expect_rejected(lambda: node.create_rate(10.0, clock=object()), BackendUnavailableError)
print("DIRECT_CPP_RATE_FAIL_CLOSED_OK")

# 1. isinstance + period + stock differential.
rate = node.create_rate(10.0)
assert isinstance(rate, rclpy.timer.Rate)
print("DIRECT_CPP_RATE_ISINSTANCE_OK")

ITERATIONS = 5
start = time.monotonic()
timestamps = []
for _ in range(ITERATIONS):
    rate.sleep()
    timestamps.append(time.monotonic())
elapsed = time.monotonic() - start
assert 0.4 <= elapsed <= 2.0, "5 sleeps at 10 Hz took %.3fs" % elapsed
gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
assert all(0.05 <= gap <= 0.3 for gap in gaps), "drifting rate gaps: %r" % gaps

peer = subprocess.run(
    [sys.executable, os.path.join(os.path.dirname(__file__), "_stock_rate_peer.py")],
    capture_output=True, text=True, timeout=30, env=os.environ.copy(),
)
assert peer.returncode == 0, (
    "stock rate peer failed:\nstdout=%s\nstderr=%s" % (peer.stdout, peer.stderr))
marker = "STOCK_RATE_OBSERVATIONS "
line = next(
    (entry for entry in peer.stdout.splitlines() if entry.startswith(marker)), None)
assert line is not None, "stock rate peer produced no observations"
import json  # noqa: E402

stock_elapsed = json.loads(line[len(marker):])["elapsed_s"]
assert 0.4 <= stock_elapsed <= 2.0, "stock 5 sleeps at 10 Hz took %.3fs" % stock_elapsed
print("DIRECT_CPP_RATE_PERIOD_OK")

# 2. Destroy: sleep() on a destroyed rate raises RuntimeError.
assert node.destroy_rate(rate) is True
try:
    rate.sleep()
except RuntimeError as exc:
    assert "destroyed" in str(exc)
else:
    raise AssertionError("destroyed direct rate slept")
print("DIRECT_CPP_RATE_DESTROY_OK")
node.destroy_node()

# 3. Sim-time pacing: driven entirely natively (a session executor thread and
# a one-shot delayed native publish), exactly like the clock-sleep helper.
sim_node = Node(
    "direct_rate_sim_%d" % os.getpid(),
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
pub_node = Node("direct_rate_clock_pub_%d" % os.getpid())

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

# One-second period anchored at sim time 0; the deadline (1e9 ns) is only
# crossed once the delayed native publish jumps /clock to 2s.
sim_rate = sim_node.create_rate(1.0)
probe.publish_clock_after(publisher.entity(), 2, 0, 300)
start = time.monotonic()
sim_rate.sleep()
sim_elapsed = time.monotonic() - start
assert sim_elapsed >= 0.2, "sim rate resolved too early: %.3fs" % sim_elapsed
assert sim_elapsed < 10.0, "sim rate took too long: %.3fs" % sim_elapsed
print("DIRECT_CPP_RATE_SIM_OK")

sim_node.destroy_rate(sim_rate)
executor_thread.close()
sim_node.destroy_node()
pub_node.destroy_node()

# 4. Context-shutdown interrupt. Runs last: it tears down the shared context.
interrupt_node = Node("direct_rate_interrupt_%d" % os.getpid())
interrupt_rate = interrupt_node.create_rate(0.1)  # 10 s period: never fires naturally
probe.shutdown_after(runtime.session.context, 200)
start = time.monotonic()
try:
    interrupt_rate.sleep()
except ROSInterruptException:
    pass
else:
    raise AssertionError("rate sleep succeeded after context shutdown")
interrupt_elapsed = time.monotonic() - start
assert interrupt_elapsed < 3.0, (
    "context shutdown did not wake the rate sleep early: %.3fs" % interrupt_elapsed)
print("DIRECT_CPP_RATE_SHUTDOWN_OK")
