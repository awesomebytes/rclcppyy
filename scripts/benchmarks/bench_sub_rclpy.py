#!/usr/bin/env python3

# Enable using C++ with just this one liner
# import rclcppyy; rclcppyy.enable_cpp_acceleration()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import sys

class PerfSubscriber(Node):
    def __init__(self):
        super().__init__('perf_subscriber_py')
        self.subscription = self.create_subscription(
            String,
            'perf_topic_py',
            self.listener_callback,
            500)
        
        self.count = 0
        self.start_time = time.time()
        self.latencies = []
        self.last_seq = -1
        self.dropped_msgs = 0
        
    def listener_callback(self, msg):
        seq, pub_timestamp = msg.data.split(':')
        seq = int(seq)
        pub_timestamp = int(pub_timestamp)
        
        # Calculate latency
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
            elapsed = time.time() - self.start_time
            rate = self.count / elapsed
            
            # Calculate latency statistics from recent messages
            recent_latencies = self.latencies[-1000:]
            avg_latency = np.mean(recent_latencies)
            p99_latency = np.percentile(recent_latencies, 99)
            
            print(f"(rclpy) Messages: {self.count}, Rate: {rate:07.1f} msgs/sec, "
                  f"Latency (μs) - Avg: {avg_latency:.1f}, P99: {p99_latency:.1f}, "
                  f"Dropped: {self.dropped_msgs}")
            
            # Reset counters for next period
            self.start_time = time.time()
            self.count = 0

def main(args=None):
    print("Starting Python subscriber benchmark")
    rclpy.init(args=args)
    subscriber = PerfSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        if subscriber.latencies:
            print("\nFinal Statistics:")
            print(f"Total messages received: {len(subscriber.latencies)}")
            print(f"Total messages dropped: {subscriber.dropped_msgs}")
            print(f"Overall latency - Avg: {np.mean(subscriber.latencies):.1f} μs, "
                  f"P99: {np.percentile(subscriber.latencies, 99):.1f} μs")
        
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 