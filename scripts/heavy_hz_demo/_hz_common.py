"""Shared plumbing for the heavy-topic ``ros2 topic hz`` demo.

The publisher and subscriber scripts each begin with the one line that is the
whole point of the demo::

    import rclcppyy; rclcppyy.enable_cpp_acceleration()

gated on the ``HEAVY_HZ_ACCEL`` env var so the *same* script runs two ways. This
module holds the parts that are identical between the two runs -- message
construction, the QoS profile, and stat-line formatting -- so that env var is the
only thing that differs. Import it *after* the acceleration toggle, so that
``from sensor_msgs.msg import Image`` below resolves to the C++ message type when
acceleration is on and to the plain rclpy type when it is off.
"""
import os
import re

ACCEL_ENV = "HEAVY_HZ_ACCEL"

# Defaults chosen empirically on the reference machine so plain rclpy visibly
# falls behind while the accelerated subscriber keeps up (see the README table).
# All are overridable via env so the orchestrator can sweep them.
DEFAULT_WIDTH = int(os.environ.get("HEAVY_HZ_WIDTH", "1024"))
DEFAULT_HEIGHT = int(os.environ.get("HEAVY_HZ_HEIGHT", "1024"))  # 1024x1024 bgr8 = 3.0 MB
# 500 Hz is above the knee: plain rclpy tops out ~210 Hz for 3 MB images (CPU-bound,
# pegging ~1.5 cores) while the accelerated subscriber keeps climbing. See README.
DEFAULT_RATE = float(os.environ.get("HEAVY_HZ_RATE", "500"))
DEFAULT_DEPTH = int(os.environ.get("HEAVY_HZ_DEPTH", "10"))
DEFAULT_TOPIC = os.environ.get("HEAVY_HZ_TOPIC", "heavy_image")

# key=value stat line the subscriber prints once per second; the orchestrator and
# any human reader parse the same line.
STAT_RE = re.compile(
    r"STAT hz=([\d.]+) recv=(\d+) expected=(\d+) dropped=(\d+) "
    r"avg_lat_ms=([\d.nae]+) window_s=([\d.]+) payload_mb=([\d.]+)"
)


def accel_enabled():
    """Whether HEAVY_HZ_ACCEL requested the C++ backend for this process."""
    return os.environ.get(ACCEL_ENV) == "1"


def payload_bytes(width, height):
    """Byte count of a bgr8 image of the given resolution (3 bytes/pixel)."""
    return width * height * 3


def payload_mb(width, height):
    return payload_bytes(width, height) / (1024.0 * 1024.0)


def make_payload(width, height, accel):
    """The image's ``data`` field, sized ``width*height*3``.

    Accelerated: a C++ ``std::vector<unsigned char>`` built by size (one C++
    allocation, ~10 ms for 3 MB) so the payload lives as a C++ object with no
    Python copy on the wire. Plain rclpy: an ``array('B', ...)`` of the same size,
    the rclpy-native form of a ``uint8[]`` field. Content is zero-filled; CDR does
    not compress, so the byte count on the wire is what matters, not the values.
    """
    n = payload_bytes(width, height)
    if accel:
        import cppyy
        return cppyy.gbl.std.vector["unsigned char"](n)
    import array
    return array.array("B", bytes(n))


def build_image(width, height, accel):
    """A fully-formed ``sensor_msgs/Image`` with an ``width*height*3`` payload."""
    from sensor_msgs.msg import Image
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = width * 3
    msg.data = make_payload(width, height, accel)
    return msg


def sensor_qos(depth):
    """RELIABLE / KEEP_LAST QoS for the heavy image stream.

    A plain rclpy ``QoSProfile`` works in both backends: native rclpy consumes it
    directly, and the rclcppyy node converts the identical object to an
    ``rclcpp::QoS`` (reliability/history/depth preserved), so the QoS is truly
    identical across the two runs.

    RELIABLE (not BEST_EFFORT) is deliberate: a 3 MB image fragments into ~2000
    UDP datagrams, and under BEST_EFFORT a single lost fragment drops the whole
    message -- so even a fast subscriber loses a large, noisy fraction that has
    nothing to do with its processing cost. RELIABLE retransmits lost fragments,
    so the only thing that limits delivered throughput is how fast the subscriber
    drains its queue. A subscriber that cannot keep up applies backpressure
    (KEEP_LAST depth bounds the writer history); the C++ publisher is otherwise
    never the limiter, so the subscriber is unambiguously the component under test.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    best_effort = os.environ.get("HEAVY_HZ_RELIABILITY", "reliable").lower() == "best_effort"
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort
                     else ReliabilityPolicy.RELIABLE),
    )


def variant_label(accel):
    return "rclcppyy" if accel else "rclpy"
