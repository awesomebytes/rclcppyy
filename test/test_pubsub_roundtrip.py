#!/usr/bin/env python3
"""Smoke test: rclpy-style pub/sub roundtrip through the C++ backend.

Covers the plain-bringup path (no ``enable_cpp_acceleration()``): a bare
``rclcpp.Node`` driven with the rclpy calling convention -- publish a handful
of Python ``std_msgs.msg.String`` messages and read them back intact from a
subscription callback fed by the C++ (cppyy) subscription.

The scenario runs in a helper subprocess (test/_pubsub_plain_helper.py). That
isolates the rclcpp bring-up and its noisy C++ shutdown from the pytest
interpreter, and keeps this file collectable alongside the monkeypatch test
without either poisoning the other. We assert on the markers the helper prints
just before it exits, which is the deterministic signal that the roundtrip
logic succeeded.
"""
import unittest

from _run_helper import run_helper, format_output


class TestPubSubRoundtrip(unittest.TestCase):

    def test_rclpy_style_roundtrip_through_cpp(self):
        proc = run_helper("_pubsub_plain_helper.py")
        out = proc.stdout
        # Sanity: this path exercises the unpatched Python message class.
        self.assertIn("PLAIN_MSG_OK", out, format_output(proc))
        # The five published payloads arrived intact and in order.
        self.assertIn("PLAIN_PUBSUB_OK", out, format_output(proc))
        self.assertEqual(proc.returncode, 0, format_output(proc))

    def test_nested_message_roundtrip_through_cpp(self):
        # A nested message (std_msgs/Header -> builtin_interfaces/Time stamp +
        # frame_id) must convert recursively on the plain-bringup publish path;
        # a shallow field copy would fail to assign the nested Time.
        proc = run_helper("_pubsub_nested_helper.py")
        out = proc.stdout
        self.assertIn("PLAIN_NESTED_MSG_OK", out, format_output(proc))
        self.assertIn("PLAIN_NESTED_OK", out, format_output(proc))
        self.assertEqual(proc.returncode, 0, format_output(proc))


if __name__ == "__main__":
    unittest.main()
