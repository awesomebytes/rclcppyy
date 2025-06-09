#!/usr/bin/env python3

import sys
import signal
from rclcppyy import bringup_rclcpp
import cppyy

bringup_rclcpp()

cppyy.include("std_msgs/msg/string.hpp")
cppyy.include("chrono")
cppyy.include("functional")

class PythonPerfPublisher:
    def __init__(self, rate_hz):
        # Initialize ROS2 if not already initialized
        if not cppyy.gbl.rclcpp.ok():
            cppyy.gbl.rclcpp.init(len(sys.argv), sys.argv)
            
        # Create a ROS2 node
        self.node = cppyy.gbl.rclcpp.Node("perf_publisher_pythonic")
        
        # Create a publisher
        self.publisher = self.node.create_publisher[cppyy.gbl.std_msgs.msg.String](
            "perf_topic_pythonic", 10)
            
        # Set up timer
        self.rate_hz = rate_hz
        self.period_ns = int(1e9 / rate_hz)  # Convert Hz to nanoseconds
        
        # Define the callback wrapper with proper Python.h include
        cppyy.cppdef("""
            #include <Python.h>
            #include <functional>
            
            static std::function<void()> create_timer_callback(PyObject* self) {
                return [self]() {
                    if (self && PyObject_HasAttrString(self, "timer_callback")) {
                        PyObject_CallMethod(self, "timer_callback", nullptr);
                    }
                };
            }
        """)
        
        # Create the timer with the wrapped callback
        callback = cppyy.gbl.create_timer_callback(self)
        self.timer = self.node.create_wall_timer(
            cppyy.gbl.std.chrono.nanoseconds(self.period_ns),
            callback)
        
        # Initialize counter and start time
        self.total_count = 0
        self.count = 0
        self.start_time = cppyy.gbl.std.chrono.steady_clock.now()
        
    def timer_callback(self):
        # Create and publish message
        message = cppyy.gbl.std_msgs.msg.String()
        now_ns = cppyy.gbl.std.chrono.duration_cast[cppyy.gbl.std.chrono.nanoseconds](
            cppyy.gbl.std.chrono.high_resolution_clock.now().time_since_epoch()).count()
        message.data = f"{self.total_count}:{now_ns}"
        self.publisher.publish(message)
        
        self.total_count += 1
        self.count += 1
        
        # Print stats every 1000 messages
        if self.count % self.rate_hz == 0:
            self.print_stats()
    
    def print_stats(self):
        # Calculate elapsed time and message rate
        now = cppyy.gbl.std.chrono.steady_clock.now()
        elapsed = cppyy.gbl.std.chrono.duration[cppyy.gbl.double](now - self.start_time).count()
        rate = self.count / elapsed
        print(f"(rclcppyy) Messages: {self.count}, Rate: {rate:.2f} msgs/sec")
        # Reset start time and counter, so we check on every period
        self.start_time = cppyy.gbl.std.chrono.steady_clock.now()
        self.count = 0
    
    def get_elapsed_seconds(self):
        now = cppyy.gbl.std.chrono.steady_clock.now()
        return cppyy.gbl.std.chrono.duration[cppyy.gbl.double](now - self.start_time).count()
    
    def get_count(self):
        return self.count
    
    def spin(self):
        try:
            # Spin the node
            cppyy.gbl.rclcpp.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            # Print final stats
            final_count = self.get_count()
            elapsed = self.get_elapsed_seconds()
            rate = final_count / elapsed
            print(f"Final stats - Messages: {final_count}, Avg Rate: {rate:.2f} msgs/sec")
            
            # Shutdown ROS2
            cppyy.gbl.rclcpp.shutdown()

def main():
    rate_hz = 1000
    if len(sys.argv) > 1:
        rate_hz = int(sys.argv[1])
    
    print(f"Starting pythonic cppyy publisher benchmark at target rate: {rate_hz} Hz")
    
    # Set up signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("Shutting down...")
        cppyy.gbl.rclcpp.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and run the publisher
    publisher = PythonPerfPublisher(rate_hz)
    publisher.spin()

if __name__ == "__main__":
    main() 