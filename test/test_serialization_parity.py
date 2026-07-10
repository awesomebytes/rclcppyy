#!/usr/bin/env python3
"""Smoke test: serialization parity between rclcppyy and rclpy.

For identical message content, ``rclcppyy.serialization`` (C++ rclcpp CDR via
cppyy) and ``rclpy.serialization`` must produce byte-for-byte identical wire
bytes, and each must be able to deserialize the other's bytes back to the same
content.

Runs in-process: it only serializes/deserializes message values (no nodes, no
spinning) and, crucially, must NOT touch ``enable_cpp_acceleration()`` -- it
relies on the unpatched Python message classes for the rclpy side, which is
exactly what the pytest interpreter has since the monkeypatch scenario is
isolated in its own subprocess.
"""
import unittest

import rclpy.serialization as rclpy_ser
from rclcppyy import serialization as cpp_ser

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


def _string_case():
    payload = "parity round-trip: 123 !@# unicode-é"

    def populate(msg):
        msg.data = payload

    def read(msg):
        return str(msg.data)  # normalise C++ std::string -> Python str

    return String, populate, read, payload


def _vector3_case():
    values = (1.5, -2.25, 3.75)

    def populate(msg):
        msg.x, msg.y, msg.z = values

    def read(msg):
        return (msg.x, msg.y, msg.z)

    return Vector3, populate, read, values


CASES = [_string_case(), _vector3_case()]


class TestSerializationParity(unittest.TestCase):

    def test_byte_for_byte_and_cross_deserialize(self):
        for py_cls, populate, read, expected in CASES:
            with self.subTest(msg_type=py_cls.__name__):
                # rclpy side: serialize the Python message.
                py_msg = py_cls()
                populate(py_msg)
                rclpy_bytes = rclpy_ser.serialize_message(py_msg)

                # rclcppyy side: build the equivalent C++ message, populate the
                # same content, serialize, and pull out the raw bytes.
                _, cpp_cls = cpp_ser.cpp_message_type_from_python(py_cls)
                cpp_msg = cpp_cls()
                populate(cpp_msg)
                serialized = cpp_ser.serialize_message(cpp_msg)
                cpp_bytes = cpp_ser.serialized_message_to_bytes(serialized)

                # Byte-for-byte equal serialized output.
                self.assertEqual(rclpy_bytes, cpp_bytes)

                # rclpy's bytes deserialize through rclcppyy to the same content.
                from_rclpy = cpp_ser.deserialize_message(
                    cpp_ser.serialized_message_from_bytes(rclpy_bytes), py_cls)
                self.assertEqual(read(from_rclpy), expected)

                # rclcppyy's bytes deserialize through rclpy to the same content.
                from_cpp = rclpy_ser.deserialize_message(cpp_bytes, py_cls)
                self.assertEqual(read(from_cpp), expected)


if __name__ == "__main__":
    unittest.main()
