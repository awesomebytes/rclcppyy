#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

class PerfPublisher(Node):
    def __init__(self, rate_hz=1000):
        super().__init__('perf_publisher_py')
        self.publisher = self.create_publisher(String, 'perf_topic_py', 10)
        self.rate_hz = rate_hz
        self.timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.total_count = 0
        self.count = 0
        self.start_time = time.time()
        
    def timer_callback(self):
        msg = String()
        timestamp = time.time_ns()
        msg.data = f'{self.total_count}:{timestamp}'
        self.publisher.publish(msg)
        self.count += 1
        self.total_count += 1
        
        # Print stats every second
        if self.count % self.rate_hz == 0:
            elapsed = time.time() - self.start_time
            rate = self.count / elapsed
            print(f"(rclpy) Messages: {self.count}, Rate: {rate:.2f} msgs/sec")
            # Reset start time and counter, so we check on every period
            self.start_time = time.time()
            self.count = 0

def main(args=None):
    rate_hz = 1000
    if len(sys.argv) > 1:
        rate_hz = int(sys.argv[1])
    
    print(f"Starting Python publisher benchmark at target rate: {rate_hz} Hz")
    rclpy.init(args=args)
    publisher = PerfPublisher(rate_hz)
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 