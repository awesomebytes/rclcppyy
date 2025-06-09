#!/usr/bin/env python3

import sys
import signal
import numpy as np
from rclcppyy import bringup_rclcpp
import cppyy

bringup_rclcpp()

cppyy.include("std_msgs/msg/string.hpp")
cppyy.include("chrono")
cppyy.include("functional")

class PythonPerfSubscriber:
    def __init__(self):
        # Initialize ROS2 if not already initialized
        if not cppyy.gbl.rclcpp.ok():
            cppyy.gbl.rclcpp.init(len(sys.argv), sys.argv)
            
        # Create a ROS2 node
        self.node = cppyy.gbl.rclcpp.Node("perf_subscriber_pythonic")
        
        # Create a subscription - minimal C++ wrapper for callback
        cppyy.cppdef("""
            #include <Python.h>
            #include <functional>
            
            static std::function<void(const std_msgs::msg::String::SharedPtr)> 
            create_subscription_callback(PyObject* self) {
                return [self](const std_msgs::msg::String::SharedPtr msg) {
                    if (self && PyObject_HasAttrString(self, "subscription_callback")) {
                        PyObject* py_data = PyUnicode_FromString(msg->data.c_str());
                        PyObject_CallMethod(self, "subscription_callback", "O", py_data);
                        Py_XDECREF(py_data);
                    }
                };
            }
        """)
        
        self.warmup()
        
        # Initialize tracking variables
        self.count = 0
        self.start_time = cppyy.gbl.std.chrono.steady_clock.now()
        self.latencies = []
        self.last_seq = -1
        self.dropped_msgs = 0

        # Create the subscription
        callback = cppyy.gbl.create_subscription_callback(self)
        self.subscription = self.node.create_subscription[cppyy.gbl.std_msgs.msg.String](
            "perf_topic_pythonic", 500, callback)

    def warmup(self):
        """
        These two operations imply a JIT compilation of the code.
        This is a warmup to avoid the first call being slower and dropping some messages.
        """
        _ = cppyy.gbl.std.chrono.duration_cast[cppyy.gbl.std.chrono.nanoseconds](
            cppyy.gbl.std.chrono.high_resolution_clock.now().time_since_epoch()).count()
        _ = cppyy.gbl.std.chrono.steady_clock.now()
        
        
    def subscription_callback(self, msg_data):
        seq, pub_timestamp = msg_data.split(':')
        seq = int(seq)
        pub_timestamp = int(pub_timestamp)
        
        # Calculate latency
        recv_timestamp = cppyy.gbl.std.chrono.duration_cast[cppyy.gbl.std.chrono.nanoseconds](
            cppyy.gbl.std.chrono.high_resolution_clock.now().time_since_epoch()).count()
        latency_us = (recv_timestamp - pub_timestamp) / 1000.0  # Convert to microseconds
        self.latencies.append(latency_us)
        
        # Check for dropped messages
        if self.last_seq >= 0 and seq != self.last_seq + 1:
            self.dropped_msgs += seq - self.last_seq - 1
        self.last_seq = seq
        
        self.count += 1
        
        # Print stats every second (assuming 1000Hz rate)
        if self.count % 1000 == 0:
            elapsed = cppyy.gbl.std.chrono.duration[cppyy.gbl.double](
                cppyy.gbl.std.chrono.steady_clock.now() - self.start_time).count()
            rate = self.count / elapsed
            
            # Calculate latency statistics from recent messages
            recent_latencies = self.latencies[-1000:]
            avg_latency = np.mean(recent_latencies)
            p99_latency = np.percentile(recent_latencies, 99)
            
            print(f"(rclcppyy) Messages: {self.count}, Rate: {rate:.2f} msgs/sec, "
                  f"Latency (μs) - Avg: {avg_latency:.1f}, P99: {p99_latency:.1f}, "
                  f"Dropped: {self.dropped_msgs}")
            
            # Reset counters for next period
            self.start_time = cppyy.gbl.std.chrono.steady_clock.now()
            self.count = 0

    def spin(self):
        try:
            # Spin the node
            cppyy.gbl.rclcpp.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            # Print final statistics
            if self.latencies:
                print("\nFinal Statistics:")
                print(f"Total messages received: {len(self.latencies)}")
                print(f"Total messages dropped: {self.dropped_msgs}")
                print(f"Overall latency - Avg: {np.mean(self.latencies):.1f} μs, "
                      f"P99: {np.percentile(self.latencies, 99):.1f} μs")
            
            # Shutdown ROS2
            cppyy.gbl.rclcpp.shutdown()

def main():
    print("Starting pythonic cppyy subscriber benchmark")
    
    # Set up signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("Shutting down...")
        cppyy.gbl.rclcpp.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and run the subscriber
    subscriber = PythonPerfSubscriber()
    subscriber.spin()

if __name__ == "__main__":
    main() 