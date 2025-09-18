"""
Expose rclcpp Serialization for use from Python.

This gives access to functions for serializing/deserializing C++ messages
and helpers to construct C++ message classes with cppyy.
"""

from typing import Tuple, Any
import os
import cppyy
from ament_index_python.packages import get_package_prefix

from rclcppyy.bringup_rclcpp import bringup_rclcpp, _resolve_message_type


def _ensure_headers_for_msg(package: str, python_msg_module: str) -> None:
    # python_msg_module looks like 'std_msgs.msg._string'
    # header should be '<package>/msg/<name>.hpp' where name is module third part without leading '_'
    parts = python_msg_module.split('.')
    if len(parts) >= 3:
        header_name = parts[2][1:] + '.hpp'
        cppyy.add_include_path(os.path.join(get_package_prefix(package), 'include', package))
        cppyy.include(f'{package}/msg/{header_name}')


def cpp_message_type_from_python(py_msg_cls: Any) -> Tuple[str, Any]:
    """
    Given a Python message class (e.g. sensor_msgs.msg.Image), return
    (cpp_type_string, cppyy_class) for the corresponding C++ message.
    """
    bringup_rclcpp()
    return _resolve_message_type(py_msg_cls)


def get_serialization_for(py_msg_or_cpp_cls: Any) -> Any:
    """
    Return rclcpp::Serialization<T> bound for the provided message type.
    Accepts either a Python msg class or a cppyy C++ msg class.
    """
    bringup_rclcpp()
    cpp_type_str, cpp_cls = _resolve_message_type(py_msg_or_cpp_cls)
    SerializationT = cppyy.gbl.rclcpp.Serialization[cpp_cls]
    return SerializationT()


def serialize_message(msg_cpp: Any) -> Any:
    """
    Serialize a C++ message (cppyy instance) into rclcpp::SerializedMessage.
    """
    bringup_rclcpp()
    # Deduce T from the instance type by asking its class
    cpp_cls = type(msg_cpp)
    SerializationT = cppyy.gbl.rclcpp.Serialization[cpp_cls]
    ser = SerializationT()
    out = cppyy.gbl.rclcpp.SerializedMessage()
    ser.serialize_message(msg_cpp, out)
    return out


def deserialize_message(serialized: Any, py_msg_or_cpp_cls: Any) -> Any:
    """
    Deserialize rclcpp::SerializedMessage into a cppyy C++ message instance.
    Accepts a Python msg class or cppyy class to identify T.
    """
    bringup_rclcpp()
    cpp_type_str, cpp_cls = _resolve_message_type(py_msg_or_cpp_cls)
    SerializationT = cppyy.gbl.rclcpp.Serialization[cpp_cls]
    ser = SerializationT()
    out = cpp_cls()
    ser.deserialize_message(serialized, out)
    return out


_HELPERS_DEFINED = False


def _ensure_serialization_helpers() -> None:
    global _HELPERS_DEFINED
    if _HELPERS_DEFINED:
        return
    bringup_rclcpp()
    # Helper C++ functions to convert between bytes and rclcpp::SerializedMessage
    cppyy.include('rclcpp/serialized_message.hpp')
    cppyy.include('rosbag2_storage/serialized_bag_message.hpp')
    cppyy.include('rosbag2_storage/ros_helper.hpp')
    cppyy.cppdef(r"""
#include <Python.h>
#include <string>
#include <cstring>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/ros_helper.hpp>

rclcpp::SerializedMessage rclcppyy_make_serialized_message_from_string(const std::string& s) {
  rclcpp::SerializedMessage out;
  out.reserve(s.size());
  auto& r = out.get_rcl_serialized_message();
  std::memcpy(r.buffer, s.data(), s.size());
  r.buffer_length = s.size();
  return out;
}

PyObject* rclcppyy_serialized_message_to_pybytes(const rclcpp::SerializedMessage& sm) {
  const auto& r = sm.get_rcl_serialized_message();
  return PyBytes_FromStringAndSize((const char*)r.buffer, (Py_ssize_t)r.buffer_length);
}

rclcpp::SerializedMessage rclcppyy_make_serialized_message_from_buffer(const void* buf, size_t len) {
  rclcpp::SerializedMessage out;
  out.reserve(len);
  auto& r = out.get_rcl_serialized_message();
  std::memcpy(r.buffer, buf, len);
  r.buffer_length = len;
  return out;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> rclcppyy_make_sbm_from_serialized_message(
    const rclcpp::SerializedMessage& sm,
    const std::string& topic,
    long long recv_ts,
    long long send_ts) {
  auto out = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  const auto& r = sm.get_rcl_serialized_message();
  out->serialized_data = rosbag2_storage::make_serialized_message((const char*)r.buffer, r.buffer_length);
  out->topic_name = topic;
  out->recv_timestamp = (int64_t)recv_ts;
  out->send_timestamp = (int64_t)send_ts;
  return out;
}
""")
    _HELPERS_DEFINED = True


def serialized_message_from_bytes(data: bytes) -> Any:
    """
    Build a C++ rclcpp::SerializedMessage from Python bytes.
    """
    _ensure_serialization_helpers()
    return cppyy.gbl.rclcppyy_make_serialized_message_from_string(data)


def serialized_message_to_bytes(serialized_msg: Any) -> bytes:
    """
    Convert a C++ rclcpp::SerializedMessage to Python bytes.
    """
    _ensure_serialization_helpers()
    return cppyy.gbl.rclcppyy_serialized_message_to_pybytes(serialized_msg)


