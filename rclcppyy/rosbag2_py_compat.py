"""
Drop-in replacement for rosbag2_py backed by rosbag2_cpp via cppyy.

Goals:
- Mirror the common rosbag2_py classes and methods (SequentialReader/Writer,
  StorageOptions, ConverterOptions, TopicMetadata) so existing user code works.
- Keep per-message overhead low by avoiding extra Python wrappers, binding
  C++ methods to locals in hot paths.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Tuple

import cppyy

from .rosbag2_cpp import bringup_rosbag2_cpp


_HELPERS_DEFINED = False


def _ensure_helpers() -> None:
    global _HELPERS_DEFINED
    if _HELPERS_DEFINED:
        return

    bringup_rosbag2_cpp()

    # Include helpers for constructing SerializedBagMessage from bytes and
    # writing via Writer without Python assembling shared_ptr.
    cppyy.include('rosbag2_storage/ros_helper.hpp')
    cppyy.cppdef(r"""
#include <Python.h>
#include <memory>
#include <string>
#include <tuple>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialized_message.hpp>

extern "C" void rclcppyy_writer_write_bytes(
    rosbag2_cpp::Writer* writer,
    const std::string& topic_name,
    const char* data,
    size_t len,
    long long recv_ts,
    long long send_ts) {
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->topic_name = topic_name;
  bag_message->serialized_data = rosbag2_storage::make_serialized_message(data, len);
  bag_message->recv_timestamp = (int64_t)recv_ts;
  bag_message->send_timestamp = (int64_t)send_ts;
  writer->write(bag_message);
}

extern "C" void rclcppyy_writer_write_serialized_message(
    rosbag2_cpp::Writer* writer,
    const std::string& topic_name,
    const rclcpp::SerializedMessage& sm,
    long long recv_ts,
    long long send_ts) {
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  const auto& r = sm.get_rcl_serialized_message();
  bag_message->topic_name = topic_name;
  bag_message->serialized_data = rosbag2_storage::make_serialized_message((const char*)r.buffer, r.buffer_length);
  bag_message->recv_timestamp = (int64_t)recv_ts;
  bag_message->send_timestamp = (int64_t)send_ts;
  writer->write(bag_message);
}

extern "C" PyObject* rclcppyy_reader_read_next_tuple(rosbag2_cpp::Reader* reader) {
  auto sbm = reader->read_next();
  rcutils_uint8_array_t& arr = *sbm->serialized_data.get();
  PyObject* pybytes = PyBytes_FromStringAndSize((const char*)arr.buffer, (Py_ssize_t)arr.buffer_length);
  if (!pybytes) return nullptr;
  PyObject* pytopic = PyUnicode_FromString(sbm->topic_name.c_str());
  if (!pytopic) { Py_DECREF(pybytes); return nullptr; }
  PyObject* pytimestamp = PyLong_FromLongLong((long long)sbm->recv_timestamp);
  if (!pytimestamp) { Py_DECREF(pybytes); Py_DECREF(pytopic); return nullptr; }
  PyObject* tup = PyTuple_New(3);
  if (!tup) { Py_DECREF(pybytes); Py_DECREF(pytopic); Py_DECREF(pytimestamp); return nullptr; }
  PyTuple_SET_ITEM(tup, 0, pytopic);
  PyTuple_SET_ITEM(tup, 1, pybytes);
  PyTuple_SET_ITEM(tup, 2, pytimestamp);
  return tup;
}

// Return (topic, rclcpp::SerializedMessage, timestamp) by value
static inline rclcpp::SerializedMessage rclcppyy_make_sm_from_rcutils(const rcutils_uint8_array_t& arr) {
  rclcpp::SerializedMessage out;
  out.reserve(arr.buffer_length);
  auto& r = out.get_rcl_serialized_message();
  std::memcpy(r.buffer, arr.buffer, arr.buffer_length);
  r.buffer_length = arr.buffer_length;
  return out;
}

std::tuple<std::string, rclcpp::SerializedMessage, long long>
rclcppyy_reader_read_next_sm_tuple(rosbag2_cpp::Reader* reader) {
  auto sbm = reader->read_next();
  rcutils_uint8_array_t& arr = *sbm->serialized_data.get();
  auto sm = rclcppyy_make_sm_from_rcutils(arr);
  return std::make_tuple(sbm->topic_name, sm, (long long)sbm->recv_timestamp);
}
""")

    _HELPERS_DEFINED = True


# Thin Python mirrors of rosbag2_py option/metadata classes
@dataclass
class StorageOptions:
    uri: str
    storage_id: str = 'mcap'
    max_bagfile_size: int = 0
    max_bagfile_duration: int = 0
    max_cache_size: int = 0
    storage_preset_profile: str = ''
    storage_config_uri: str = ''
    snapshot_mode: bool = False
    start_time_ns: int = 0
    end_time_ns: int = 0
    custom_data: Optional[dict] = None


@dataclass
class ConverterOptions:
    input_serialization_format: str = 'cdr'
    output_serialization_format: str = 'cdr'


@dataclass
class TopicMetadata:
    id: int
    name: str
    type: str
    serialization_format: str
    offered_qos_profiles: Optional[List[Any]] = None
    type_description_hash: str = ''


def _to_cxx_storage_options(py_opt: StorageOptions) -> Any:
    cxx = cppyy.gbl.rosbag2_storage.StorageOptions()
    cxx.uri = py_opt.uri
    cxx.storage_id = py_opt.storage_id
    cxx.max_bagfile_size = py_opt.max_bagfile_size
    cxx.max_bagfile_duration = py_opt.max_bagfile_duration
    cxx.max_cache_size = py_opt.max_cache_size
    cxx.storage_preset_profile = py_opt.storage_preset_profile
    cxx.storage_config_uri = py_opt.storage_config_uri
    cxx.snapshot_mode = py_opt.snapshot_mode
    cxx.start_time_ns = py_opt.start_time_ns
    cxx.end_time_ns = py_opt.end_time_ns
    if py_opt.custom_data:
        for k, v in py_opt.custom_data.items():
            cxx.custom_data[k] = v
    return cxx


def _to_cxx_converter_options(py_opt: ConverterOptions) -> Any:
    cxx = cppyy.gbl.rosbag2_cpp.ConverterOptions()
    cxx.input_serialization_format = py_opt.input_serialization_format
    cxx.output_serialization_format = py_opt.output_serialization_format
    return cxx


def _to_cxx_topic_metadata(py_md: TopicMetadata) -> Any:
    md = cppyy.gbl.rosbag2_storage.TopicMetadata()
    md.name = py_md.name
    md.type = py_md.type
    md.serialization_format = py_md.serialization_format
    try:
        if py_md.offered_qos_profiles is not None:
            md.offered_qos_profiles = py_md.offered_qos_profiles
    except Exception:
        pass
    try:
        if py_md.type_description_hash:
            md.type_description_hash = py_md.type_description_hash
    except Exception:
        pass
    return md


class SequentialReader:
    def __init__(self) -> None:
        bringup_rosbag2_cpp()
        self._cxx = cppyy.gbl.rclcppyy_make_reader_default()

    # Compatibility API
    def open(self, storage_options: StorageOptions, converter_options: ConverterOptions) -> None:
        cxo = _to_cxx_storage_options(storage_options)
        cco = _to_cxx_converter_options(converter_options)
        self._cxx.open(cxo, cco)

    def open_uri(self, uri: str) -> None:
        self._cxx.open(uri)

    def close(self) -> None:
        self._cxx.close()

    def has_next(self) -> bool:
        return bool(self._cxx.has_next())

    # def read_next(self) -> Tuple[str, bytes, int]:
    #     # Fast path: build the Python tuple in C++ to avoid pointer fiddling
    #     return cppyy.gbl.rclcppyy_reader_read_next_tuple(self._cxx)

    # Fast-path access to SerializedBagMessage, useful for calling deserialize immediately from Python
    def read_next_serialized(self) -> Any:
        return self._cxx.read_next()

    # Zero-copy view variant: returns (topic, memoryview, ts)
    def read_next_view(self) -> Tuple[str, memoryview, int]:
        return cppyy.gbl.rclcppyy_reader_read_next_tuple_view(self._cxx)

    # SerializedMessage variant: returns (topic, rclcpp::SerializedMessage, ts)
    def read_next(self) -> Tuple[str, Any, int]:
        return cppyy.gbl.rclcppyy_reader_read_next_sm_tuple(self._cxx)


    def get_all_topics_and_types(self) -> Iterable[Any]:
        try:
            return self._cxx.get_all_topics_and_types()
        except Exception:
            return self._cxx.get_topics_and_types()

    # Optional pass-throughs for completeness
    def seek(self, timestamp: int) -> None:
        self._cxx.seek(int(timestamp))

    def set_filter(self, storage_filter: Any) -> None:
        self._cxx.set_filter(storage_filter)

    def reset_filter(self) -> None:
        self._cxx.reset_filter()

    def set_read_order(self, read_order: Any) -> bool:
        return bool(self._cxx.set_read_order(read_order))


class SequentialWriter:
    def __init__(self) -> None:
        bringup_rosbag2_cpp()
        _ensure_helpers()
        self._cxx = cppyy.gbl.rclcppyy_make_writer_default()

    def open(self, storage_options: StorageOptions, converter_options: ConverterOptions) -> None:
        cxo = _to_cxx_storage_options(storage_options)
        cco = _to_cxx_converter_options(converter_options)
        self._cxx.open(cxo, cco)

    def close(self) -> None:
        self._cxx.close()

    def create_topic(self, topic_metadata: Any) -> None:
        if hasattr(topic_metadata, 'name') and not hasattr(topic_metadata, 'serialization_format'):
            # Not expected, but try to be forgiving
            md = _to_cxx_topic_metadata(TopicMetadata(
                id=0,
                name=topic_metadata.name,
                type=getattr(topic_metadata, 'type', ''),
                serialization_format=getattr(topic_metadata, 'serialization_format', 'cdr'),
            ))
        elif isinstance(topic_metadata, TopicMetadata):
            md = _to_cxx_topic_metadata(topic_metadata)
        else:
            md = topic_metadata
        self._cxx.create_topic(md)

    # Match rosbag2_py signature(s)
    def write(self, topic_name: str, message: Any, recv_timestamp: int, send_timestamp: Optional[int] = None) -> None:
        if send_timestamp is None:
            send_timestamp = recv_timestamp
        # Accept either bytes or rclcpp::SerializedMessage
        # Try C++ SerializedMessage first (duck-typing by attribute)
        try:
            # cppyy proxies expose get_rcl_serialized_message
            _ = message.get_rcl_serialized_message  # attribute exists
            cppyy.gbl.rclcppyy_writer_write_serialized_message(self._cxx, topic_name, message, int(recv_timestamp), int(send_timestamp))
            return
        except Exception:
            pass
        # Fallback: assume bytes-like
        cppyy.gbl.rclcppyy_writer_write_bytes(self._cxx, topic_name, message, len(message), int(recv_timestamp), int(send_timestamp))

    # Convenience fast-path for serialized bag messages
    def write_serialized(self, sbm: Any) -> None:
        self._cxx.write(sbm)


