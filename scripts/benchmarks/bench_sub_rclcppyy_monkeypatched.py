#!/usr/bin/env python3

import sys
import signal
import numpy as np
import rclpy
from std_msgs.msg import String
import time

# Enable C++ acceleration - this should make subscriptions use C++ under the hood
import rclcppyy
rclcppyy.enable_cpp_acceleration()

class PythonPerfSubscriber:
    def __init__(self):
        # Initialize ROS2 using standard rclpy API
        if not rclpy.ok():
            rclpy.init()
        
        # Create a ROS2 node using standard rclpy API - but it will actually be RclcppyyNode
        self.node = rclpy.create_node("perf_subscriber_monkeypatched")
        
        self.warmup()
        
        # Initialize tracking variables
        self.count = 0
        self.start_time = time.time_ns()
        self.latencies = []
        self.last_seq = -1
        self.dropped_msgs = 0

        # Create the subscription using standard rclpy API - but it will use C++ under the hood
        self.subscription = self.node.create_subscription(
            String,
            "perf_topic_pythonic", 
            self.subscription_callback,
            500  # QoS depth
        )
        
        print(f"Created subscription with type: {type(self.subscription)}")
        print(f"Node type: {type(self.node)}")

    def warmup(self):
        """
        Basic warmup to prepare timing operations.
        """
        # Simple warmup operations
        _ = time.time_ns()
        _ = time.time()
        
    def subscription_callback(self, msg):
        """
        Standard callback that receives the ROS message.
        With monkeypatching, this should receive a C++ message object.
        """
        try:
            # Extract data from the message
            # The message should be a C++ message object due to monkeypatching
            msg_data = msg.data
            
            seq, pub_timestamp = msg_data.split(':')
            seq = int(seq)
            pub_timestamp = int(pub_timestamp)
            
            # Calculate latency using Python time functions
            recv_timestamp = time.time_ns()
            latency_us = (recv_timestamp - pub_timestamp) / 1000.0  # Convert to microseconds
            self.latencies.append(latency_us)
            
            # Check for dropped messages
            if self.last_seq >= 0 and seq != self.last_seq + 1:
                self.dropped_msgs += seq - self.last_seq - 1
            self.last_seq = seq
            
            self.count += 1
            
            # Print stats every second (assuming 1000Hz rate)
            if self.count % 1000 == 0:
                elapsed = (time.time_ns() - self.start_time) / 1e9  # Convert to seconds
                rate = self.count / elapsed
                
                # Calculate latency statistics from recent messages
                recent_latencies = self.latencies[-1000:]
                avg_latency = np.mean(recent_latencies)
                p99_latency = np.percentile(recent_latencies, 99)
                
                print(f"(rclcppyy-monkeypatched) Messages: {self.count}, Rate: {rate:.2f} msgs/sec, "
                      f"Latency (μs) - Avg: {avg_latency:.1f}, P99: {p99_latency:.1f}, "
                      f"Dropped: {self.dropped_msgs}")
                
                # Reset counters for next period
                self.start_time = time.time_ns()
                self.count = 0
                
        except Exception as e:
            print(f"Error in callback: {e}")
            print(f"Message type: {type(msg)}")
            print(f"Message dir: {dir(msg)}")

    def spin(self):
        try:
            # Spin using standard rclpy API - but it will use C++ under the hood
            rclpy.spin(self.node)
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
            rclpy.shutdown()

def main():
    print("Starting monkeypatched rclcppyy subscriber benchmark")
    
    # Set up signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("Shutting down...")
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and run the subscriber
    subscriber = PythonPerfSubscriber()
    subscriber.spin()

if __name__ == "__main__":
    main()