import cv2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image


CV_LIBRARY = cv2


class CameraRelay(Node):
    def __init__(self):
        super().__init__("camera_relay")
        self.group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Image, "output", 10)
        self.subscription = self.create_subscription(
            Image, "input", self.on_image, 10, callback_group=self.group)

    def on_image(self, message):
        total = 0
        for value in message.data:
            total += value
        self.publisher.publish(message)
        return total


def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(CameraRelay())
    executor.spin()
