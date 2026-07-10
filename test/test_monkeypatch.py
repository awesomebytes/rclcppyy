#!/usr/bin/env python3
"""Smoke test: the enable_cpp_acceleration() monkeypatch path.

Asserts that after ``rclcppyy.enable_cpp_acceleration()``:
  * ``rclpy.create_node`` returns an ``RclcppyyNode``,
  * an imported message class (``std_msgs.msg.String``) is redirected to the
    C++ (cppyy) type by the install-time import hook, and
  * a pub/sub roundtrip works through the patched rclpy API.

``enable_cpp_acceleration()`` monkeypatches rclpy process-globally and
irreversibly, so running it in the pytest interpreter would poison every other
test (notably the plain-bringup and serialization ones that rely on the
unpatched Python classes). It therefore runs in a throwaway subprocess
(test/_monkeypatch_helper.py); we assert on the checkpoint markers it prints.
"""
import unittest

from _run_helper import run_helper, format_output


class TestMonkeypatch(unittest.TestCase):

    def test_enable_cpp_acceleration_path(self):
        proc = run_helper("_monkeypatch_helper.py")
        out = proc.stdout
        self.assertIn("MSG_REDIRECT_OK", out, format_output(proc))
        self.assertIn("CREATE_NODE_OK", out, format_output(proc))
        self.assertIn("ROUNDTRIP_OK", out, format_output(proc))
        self.assertIn("MONKEYPATCH_ALL_OK", out, format_output(proc))
        self.assertEqual(proc.returncode, 0, format_output(proc))


if __name__ == "__main__":
    unittest.main()
