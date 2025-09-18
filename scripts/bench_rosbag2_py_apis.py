#!/usr/bin/env python3

import argparse
import os
import gc
import shutil
import time
from dataclasses import dataclass
from typing import List

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message as rclpy_serialize, deserialize_message as rclpy_deserialize
from rclcppyy.serialization import deserialize_message as cxx_deserialize, serialize_message as cxx_serialize, serialized_message_from_bytes, serialized_message_to_bytes, cpp_message_type_from_python

from rosbag2_py import SequentialReader as PyReader, SequentialWriter as PyWriter, StorageOptions as PyStorageOptions, ConverterOptions as PyConverterOptions, TopicMetadata as PyTopicMetadata

from rclcppyy.rosbag2_py_compat import SequentialReader as CxxReaderCompat, SequentialWriter as CxxWriterCompat, StorageOptions as CxxStorageOptions, ConverterOptions as CxxConverterOptions, TopicMetadata as CxxTopicMetadata


@dataclass
class Args:
    input_uri: str
    storage_id: str
    serialization: str
    runs: int
    add_serialisation_roundtrip: bool
    disable_gc: bool


def run_py_api(args: Args, out_dir: str):
    reader = PyReader()
    reader.open(PyStorageOptions(uri=args.input_uri, storage_id=args.storage_id),
                PyConverterOptions(input_serialization_format=args.serialization, output_serialization_format=args.serialization))
    writer = PyWriter()
    writer.open(PyStorageOptions(uri=out_dir, storage_id=args.storage_id),
                PyConverterOptions(input_serialization_format=args.serialization, output_serialization_format=args.serialization))

    try:
        topics = reader.get_all_topics_and_types()
    except AttributeError:
        topics = reader.get_topics_and_types()

    topic_type = {t.name: t.type for t in topics}

    for t in topics:
        writer.create_topic(PyTopicMetadata(
            id=0,
            name=t.name,
            type=t.type,
            serialization_format=args.serialization
        ))

    t0 = time.time()
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if args.add_serialisation_roundtrip:
            msg_py_cls = get_message(topic_type.get(topic, ''))
            if msg_py_cls is not None:
                msg_py = rclpy_deserialize(data, msg_py_cls)
                # We could do something here with the message
                data = rclpy_serialize(msg_py)
        writer.write(topic, data, ts)
        count += 1
    t1 = time.time()
    return t1 - t0, count


def run_cxx_api(args: Args, out_dir: str):
    reader = CxxReaderCompat()
    reader.open(CxxStorageOptions(uri=args.input_uri, storage_id=args.storage_id),
                CxxConverterOptions(input_serialization_format=args.serialization, output_serialization_format=args.serialization))
    writer = CxxWriterCompat()
    writer.open(CxxStorageOptions(uri=out_dir, storage_id=args.storage_id),
                CxxConverterOptions(input_serialization_format=args.serialization, output_serialization_format=args.serialization))

    try:
        topics = reader.get_all_topics_and_types()
    except Exception:
        topics = reader.get_topics_and_types()

    topic_type = {t.name: t.type for t in topics}

    for t in topics:
        writer.create_topic(CxxTopicMetadata(
            id=0,
            name=t.name,
            type=t.type,
            serialization_format=args.serialization
        ))

    t0 = time.time()
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if args.add_serialisation_roundtrip:
            msg_py_cls = get_message(topic_type.get(topic, ''))
            if msg_py_cls is not None:
                msg_cpp = cxx_deserialize(data, msg_py_cls)
                # We could do something here with the message
                data = cxx_serialize(msg_cpp)
        writer.write(topic, data, ts)
        count += 1
    t1 = time.time()
    return t1 - t0, count


def main():
    p = argparse.ArgumentParser(description='Benchmark rosbag2_py vs rclcppyy.rosbag2_py_compat APIs.')
    p.add_argument('input_uri')
    p.add_argument('--storage-id', default='mcap')
    p.add_argument('--serialization', default='cdr')
    p.add_argument('--runs', type=int, default=int(os.environ.get('RUN_BENCHMARK_TIMES', '50')))
    p.add_argument('--add-serialisation-roundtrip', action='store_true', help='Deserialize bytes to message and re-serialize before writing')
    p.add_argument('--disable-gc', action='store_true', help='Disable Python GC inside each run loop')
    a = p.parse_args()

    args = Args(a.input_uri, a.storage_id, a.serialization, max(1, a.runs), bool(a.add_serialisation_roundtrip), bool(a.disable_gc))

    base_tmp_dir = os.path.join('/tmp', 'some_temporary_folder_pyapi')
    shutil.rmtree(base_tmp_dir, ignore_errors=True)
    os.makedirs(base_tmp_dir, exist_ok=True)

    py_times: List[float] = []
    cxx_times: List[float] = []

    try:
        for i in range(args.runs):
            out_dir = os.path.join(base_tmp_dir, f'py_run_{i}')
            shutil.rmtree(out_dir, ignore_errors=True)
            gc_was_enabled = gc.isenabled()
            if args.disable_gc and gc_was_enabled:
                gc.disable()
            try:
                t, n = run_py_api(args, out_dir)
            finally:
                if args.disable_gc and gc_was_enabled:
                    gc.enable()
                    gc.collect()
            py_times.append(t)
            print(f"rosbag2_py run {i+1}/{args.runs}: {n} messages in {t:.3f}s -> {n/max(t,1e-9):.1f} msgs/s")

        for i in range(args.runs):
            out_dir = os.path.join(base_tmp_dir, f'cxx_run_{i}')
            shutil.rmtree(out_dir, ignore_errors=True)
            gc_was_enabled = gc.isenabled()
            if args.disable_gc and gc_was_enabled:
                gc.disable()
            try:
                t, n = run_cxx_api(args, out_dir)
            finally:
                if args.disable_gc and gc_was_enabled:
                    gc.enable()
                    gc.collect()
            cxx_times.append(t)
            print(f"py_compat   run {i+1}/{args.runs}: {n} messages in {t:.3f}s -> {n/max(t,1e-9):.1f} msgs/s")

        if py_times:
            print(f"rosbag2_py summary over {len(py_times)} runs: min={min(py_times):.3f}s max={max(py_times):.3f}s avg={sum(py_times)/len(py_times):.3f}s")
        if cxx_times:
            print(f"py_compat   summary over {len(cxx_times)} runs: min={min(cxx_times):.3f}s max={max(cxx_times):.3f}s avg={sum(cxx_times)/len(cxx_times):.3f}s")
    finally:
        shutil.rmtree(base_tmp_dir, ignore_errors=True)


if __name__ == '__main__':
    main()
