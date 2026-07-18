"""Ordinary ROS 2 Python relay used unchanged by stock and activated runs."""

from rclpy.node import Node
from std_msgs.msg import String


class TextRelay(Node):
    def __init__(self, *, context):
        super().__init__("transparent_text_relay", context=context)
        self.callback_count = 0
        self.publisher = self.create_publisher(String, "/rewrite_fixture/t0/output", 10)
        self.subscription = self.create_subscription(
            String,
            "/rewrite_fixture/t0/input",
            self.on_message,
            10,
        )

    def on_message(self, message):
        self.callback_count += 1
        self.publisher.publish(String(data=str(message.data) + ":relayed"))
