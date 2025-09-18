#! /usr/bin/env python3
import rclcppyy; rclcppyy.enable_cpp_acceleration()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data

class ImageSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_passthrough")
        self.publisher = self.create_publisher(PointCloud2, "/pointcloud_passthrough", qos_profile_sensor_data)
        # Subscribe to the same topic as the demo publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            "/lexus3/os_center/points",
            self._on_pointcloud,
            10,
        )
        print("subscription created")

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        print(f"callback, received pointcloud.")
        self.publisher.publish(msg)

def main() -> None:
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


