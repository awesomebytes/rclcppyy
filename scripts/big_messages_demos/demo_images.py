#! /usr/bin/env python3

import rclcppyy; rclcppyy.enable_cpp_acceleration()
import cppyy
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import Image
import numpy as np
import time

def get_path_of_this_file():
    import os
    return os.path.dirname(os.path.abspath(__file__))

def get_image_from_disk(path):
    import cv2
    
    img = cv2.imread(path)
    if img is None:
        raise ValueError(f"Failed to load image from {path}")
    
    ros_img = Image()
    ros_img.width = img.shape[1]
    ros_img.height = img.shape[0]
    ros_img.encoding = "bgr8"
    ros_img.is_bigendian = False
    ros_img.step = img.shape[1] * 3
    
    # Get the data as a simple Python list of integers
    flat_img = np.ascontiguousarray(img, dtype=np.uint8).flatten()
    byte_list = [int(b) for b in flat_img]
    
    # Create an empty vector
    vector_type = cppyy.gbl.std.vector["unsigned char"]()
    
    # Add elements using the std::vector::insert method
    # This avoids having to iterate through each element
    vector_type.insert(vector_type.end(), byte_list)
    
    ros_img.data = vector_type
    print(ros_img)
    print(ros_img.data[:20])
    print(f"Size of image: {len(ros_img.data)}, in MB: {len(ros_img.data) / 1024 / 1024}")
    return ros_img

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.publisher = self.create_publisher(Image, "image", 10)
        # Convert the image to a ROS2 Image message
        self.image_msg = get_image_from_disk(os.path.join(get_path_of_this_file(), "pr2_robot_magic_lab.jpg"))
        self.hz = 4
        self.timer = self.create_timer(1.0 / self.hz, self.timer_callback)

    def timer_callback(self):
        start_time = time.time()
        # Rough edge, TODO: make dealing with stamps more Pythonic
        self.image_msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        self.image_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        self.publisher.publish(self.image_msg)
        end_time = time.time()
        # print(f"Time taken to publish image: {(end_time - start_time) * 1000} ms")

if __name__ == "__main__":
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()