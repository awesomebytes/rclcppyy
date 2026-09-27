"""Python transform relay used as the behavior oracle for native fusion."""

from rclpy.node import Node
from std_msgs.msg import String


class CommandAnnotator(Node):
    def __init__(self, *, context):
        super().__init__("command_annotator", context=context)
        self.callback_count = 0
        self.publisher = self.create_publisher(
            String, "/rewrite_fixture/t3/output", 10)
        self.subscription = self.create_subscription(
            String,
            "/rewrite_fixture/t3/input",
            self.on_message,
            10,
        )

    def on_message(self, message):
        self.callback_count += 1
        self.publisher.publish(String(data=str(message.data) + ":annotated"))
