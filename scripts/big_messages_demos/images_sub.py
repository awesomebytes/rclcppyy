#! /usr/bin/env python3
import rclcppyy; rclcppyy.enable_cpp_acceleration()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("image_subscriber")
        self.message_count = 0
        # Subscribe to the same topic as the demo publisher
        self.subscription = self.create_subscription(
            Image,
            "image",
            self._on_image,
            10,
        )
        print("subscription created")

    def _on_image(self, msg: Image) -> None:
        print(f"callback, received image: {msg.width}x{msg.height}, encoding: {msg.encoding}")
        print(msg)
        print(dir(msg))
        self.message_count += 1
        if self.message_count % 100 == 0:
            width = getattr(msg, "width", 0)
            height = getattr(msg, "height", 0)
            data_size = len(getattr(msg, "data", []))
            stamp = getattr(msg, "header", None)
            if stamp is not None:
                stamp_str = f"{stamp.stamp.sec}.{stamp.stamp.nanosec:09d}"
            else:
                stamp_str = "n/a"
            print(
                f"Received {self.message_count} images | stamp={stamp_str} | "
                f"resolution={width}x{height} | bytes={data_size}"
            )


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


