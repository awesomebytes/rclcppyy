#!/usr/bin/env python3
"""Tests for rclcppyy.tf (tf2 C++ transform stack via cppyy).

tf2 is core ROS 2 (present in the default env), so these run in the default
`pixi run test`. They share one process: bringup is idempotent and each test uses
distinct frame names, so a single module-scoped TransformListener keeps them
independent while staying fast. One test exercises the real network ingest path
(publish on /tf via rclcppyy -> C++ listener -> lookup) to prove transforms are
ingested with no per-message Python callback.
"""
import os
import time

import cppyy
import pytest

# tf2 ships in ros-base; skip defensively only if the C++ headers are truly absent.
_HAVE_TF2 = os.path.isdir(
    os.path.join(os.environ.get("CONDA_PREFIX", ""), "include", "tf2", "tf2"))
pytestmark = pytest.mark.skipif(not _HAVE_TF2, reason="tf2 C++ headers not installed")

if _HAVE_TF2:
    import rclcppyy
    from rclcppyy import tf


@pytest.fixture(scope="module")
def listener():
    rclcpp = rclcppyy.bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    return tf.TransformListener()


def _ts(parent, child, x, y, z):
    t = cppyy.gbl.geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.w = 1.0
    return t


def test_bringup_tf_returns_namespaces():
    tf2, glue = tf.bringup_tf()
    assert hasattr(tf2, "BufferCore")
    assert hasattr(glue, "lookup")


def test_lookup_composed_transform(listener):
    # world -> base (+1x) -> sensor (+2y); world<-sensor should compose to (1, 2, 0).
    listener.set_transform(_ts("world", "base", 1, 0, 0), is_static=True)
    listener.set_transform(_ts("base", "sensor", 0, 2, 0), is_static=True)
    out = listener.lookup_transform("world", "sensor")
    tr = out.transform.translation
    assert abs(tr.x - 1.0) < 1e-9
    assert abs(tr.y - 2.0) < 1e-9
    assert abs(tr.z - 0.0) < 1e-9


def test_can_transform_true_and_false(listener):
    listener.set_transform(_ts("ct_a", "ct_b", 1, 0, 0), is_static=True)
    assert listener.can_transform("ct_a", "ct_b") is True
    assert listener.can_transform("ct_a", "ct_ghost") is False


def test_lookup_missing_raises(listener):
    with pytest.raises(tf.TransformException):
        listener.lookup_transform("no_such_a", "no_such_b")


def test_lookup_timeout_raises_fast(listener):
    t0 = time.perf_counter()
    with pytest.raises(tf.TransformException):
        listener.lookup_transform("to_a", "to_b", timeout=0.2)
    # The C++ poll wait should return close to the requested timeout, not hang.
    assert time.perf_counter() - t0 < 2.0


def test_chain_endpoint(listener):
    for i in range(6):
        parent = "chain_root" if i == 0 else "chain_%d" % (i - 1)
        listener.set_transform(_ts(parent, "chain_%d" % i, 0.1, 0, 0), is_static=True)
    out = listener.lookup_transform("chain_root", "chain_5")
    assert abs(out.transform.translation.x - 0.6) < 1e-6


def test_time_helpers():
    tp = tf.time_from_sec(1.5)
    d = tf.duration_from_sec(0.25)
    assert tp is not None and d is not None


def test_network_ingest_no_python_callback():
    """Publish /tf via rclcppyy; the C++ listener ingests it on its own thread (no
    Python callback), and Python looks the transform up."""
    rclcpp = rclcppyy.bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    net_listener = tf.TransformListener()
    node = rclcpp.Node("tf_test_net_pub")
    pub = node.create_publisher[cppyy.gbl.tf2_msgs.msg.TFMessage]("/tf", 100)
    msg = cppyy.gbl.tf2_msgs.msg.TFMessage()
    msg.transforms.push_back(_ts("net_world", "net_base", 3, 0, 0))
    msg.transforms.push_back(_ts("net_base", "net_tip", 0, 4, 0))

    deadline = time.time() + 5.0
    got = False
    while time.time() < deadline:
        pub.publish(msg)
        if net_listener.can_transform("net_world", "net_tip", timeout=0.1):
            got = True
            break
    assert got, "C++ listener never ingested the published /tf"
    out = net_listener.lookup_transform("net_world", "net_tip")
    assert abs(out.transform.translation.x - 3.0) < 1e-9
    assert abs(out.transform.translation.y - 4.0) < 1e-9
    net_listener.close()


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
