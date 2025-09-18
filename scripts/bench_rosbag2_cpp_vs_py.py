#!/usr/bin/env python3

import argparse
import os
import gc
import shutil
import time
from dataclasses import dataclass
from typing import List, Tuple

from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

from rclcppyy import rosbag2_cpp as rosbag2_cxx


@dataclass
class Args:
    input_uri: str
    output_uri_py: str
    output_uri_cxx: str
    storage_id: str
    serialization: str


def warmup_cppyy():
    # Bring up C++ symbols and instantiate common types to avoid JIT during timing
    import cppyy
    rosbag2_cxx.bringup_rosbag2_cpp()
    _ = rosbag2_cxx.types()
    # Instantiate key classes (no opening) using factories to avoid unique_ptr copy issues
    _ = cppyy.gbl.rclcppyy_make_reader_default()
    _ = cppyy.gbl.rclcppyy_make_writer_default()
    _ = cppyy.gbl.rosbag2_storage.SerializedBagMessage()


def run_python_copy(args: Args):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.input_uri, storage_id=args.storage_id),
                ConverterOptions(input_serialization_format=args.serialization, output_serialization_format=args.serialization))
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.output_uri_py, storage_id=args.storage_id),
                ConverterOptions(input_serialization_format=args.serialization, output_serialization_format=args.serialization))

    try:
        topics = reader.get_all_topics_and_types()
    except AttributeError:
        topics = reader.get_topics_and_types()

    for t in topics:
        writer.create_topic(TopicMetadata(
            id=0,
            name=t.name,
            type=t.type,
            serialization_format=args.serialization
        ))

    has_next = reader.has_next
    read_next = reader.read_next
    write = writer.write

    gc_was_enabled = gc.isenabled()
    if gc_was_enabled:
        gc.disable()
    try:
        t0 = time.time()
        count = 0
        while has_next():
            topic, data, ts = read_next()
            write(topic, data, ts)
            count += 1
        t1 = time.time()
    finally:
        if gc_was_enabled:
            gc.enable()
            gc.collect()
    return t1 - t0, count


def run_cxx_copy(args: Args):
    rosbag2_cxx.bringup_rosbag2_cpp()
    reader = rosbag2_cxx.open_reader(args.input_uri, storage_id=args.storage_id, serialization=args.serialization)
    writer = rosbag2_cxx.open_writer(args.output_uri_cxx, storage_id=args.storage_id, serialization=args.serialization)

    # Create all topics on writer using direct C++ calls (avoid Python wrappers)
    try:
        topics = reader.get_all_topics_and_types()
    except Exception:
        topics = reader.get_topics_and_types()
    create_topic = writer.create_topic
    for md in topics:
        create_topic(md)

    has_next = reader.has_next
    read_next = reader.read_next
    write = writer.write

    gc_was_enabled = gc.isenabled()
    if gc_was_enabled:
        gc.disable()
    try:
        t0 = time.time()
        count = 0
        while has_next():
            sbm = read_next()
            write(sbm)
            count += 1
        t1 = time.time()
    finally:
        if gc_was_enabled:
            gc.enable()
            gc.collect()
    return t1 - t0, count


def main():
    p = argparse.ArgumentParser(description='Benchmark pure read+write throughput: rosbag2_py vs rosbag2_cpp (cppyy).')
    p.add_argument('input_uri')
    p.add_argument('--out-py', default='out_py_copy')
    p.add_argument('--out-cxx', default='out_cxx_copy')
    p.add_argument('--storage-id', default='mcap')
    p.add_argument('--serialization', default='cdr')
    a = p.parse_args()
    args = Args(a.input_uri, a.out_py, a.out_cxx, a.storage_id, a.serialization)

    # Benchmark configuration
    runs_env = os.environ.get('RUN_BENCHMARK_TIMES', '100')
    try:
        RUNS = max(1, int(runs_env))
    except Exception:
        RUNS = 100

    # Use a dedicated temp base directory for all outputs
    base_tmp_dir = os.path.join('/tmp', 'some_temporary_folder')
    # Start fresh
    shutil.rmtree(base_tmp_dir, ignore_errors=True)
    os.makedirs(base_tmp_dir, exist_ok=True)

    # Warm up cppyy/rosbag2_cpp to avoid JIT in measured section
    warmup_cppyy()

    py_times: List[float] = []
    cxx_times: List[float] = []
    py_counts: List[int] = []
    cxx_counts: List[int] = []

    try:
        # Run Python implementation RUNS times
        for i in range(RUNS):
            out_py_i = os.path.join(base_tmp_dir, f'py_run_{i}')
            # Ensure bag directory does not exist; writer expects to create it
            shutil.rmtree(out_py_i, ignore_errors=True)
            run_args_py = Args(
                input_uri=args.input_uri,
                output_uri_py=out_py_i,
                output_uri_cxx='',
                storage_id=args.storage_id,
                serialization=args.serialization,
            )
            t_py, n_py = run_python_copy(run_args_py)
            py_times.append(t_py)
            py_counts.append(n_py)
            print(f"Python copy run {i+1}/{RUNS}: {n_py} messages in {t_py:.3f}s  -> {n_py/max(t_py,1e-9):.1f} msgs/s")

        # Run C++ implementation RUNS times
        for i in range(RUNS):
            out_cxx_i = os.path.join(base_tmp_dir, f'cxx_run_{i}')
            # Ensure bag directory does not exist; writer expects to create it
            shutil.rmtree(out_cxx_i, ignore_errors=True)
            run_args_cxx = Args(
                input_uri=args.input_uri,
                output_uri_py='',
                output_uri_cxx=out_cxx_i,
                storage_id=args.storage_id,
                serialization=args.serialization,
            )
            t_cxx, n_cxx = run_cxx_copy(run_args_cxx)
            cxx_times.append(t_cxx)
            cxx_counts.append(n_cxx)
            print(f"C++ copy   run {i+1}/{RUNS}: {n_cxx} messages in {t_cxx:.3f}s -> {n_cxx/max(t_cxx,1e-9):.1f} msgs/s")

        # Summaries
        if py_times:
            py_min = min(py_times)
            py_max = max(py_times)
            py_avg = sum(py_times) / len(py_times)
            print(f"Python summary over {len(py_times)} runs: min={py_min:.3f}s max={py_max:.3f}s avg={py_avg:.3f}s")
        if cxx_times:
            cxx_min = min(cxx_times)
            cxx_max = max(cxx_times)
            cxx_avg = sum(cxx_times) / len(cxx_times)
            print(f"C++ summary over {len(cxx_times)} runs: min={cxx_min:.3f}s max={cxx_max:.3f}s avg={cxx_avg:.3f}s")
    finally:
        # Cleanup all generated outputs
        shutil.rmtree(base_tmp_dir, ignore_errors=True)


if __name__ == '__main__':
    main()


