"""A callback-group worker representative of an explicit Python ROS node."""

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class TelemetryNormalizer(Node):
    def __init__(self, *, context):
        super().__init__("telemetry_normalizer", context=context)
        self.callback_count = 0
        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(
            String, "/rewrite_fixture/t2/output", 10)
        self.subscription = self.create_subscription(
            String,
            "/rewrite_fixture/t2/input",
            self.on_message,
            10,
            callback_group=self.callback_group,
        )

    def on_message(self, message):
        self.callback_count += 1
        normalized = str(message.data).strip().lower() + ":normalized"
        self.publisher.publish(String(data=normalized))


def make_executor(context):
    return MultiThreadedExecutor(num_threads=2, context=context)
