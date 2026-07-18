#!/usr/bin/env python3
"""Isolated cppyy variants for the Python-boundary characterization."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time


HERE = Path(__file__).resolve().parent
KERNEL = HERE / "boundary_kernel.hpp"
MASK64 = (1 << 64) - 1


def python_transform(state, index):
    state ^= (
        index + 0x9E3779B97F4A7C15 + (state << 6) + (state >> 2)
    ) & MASK64
    return (state * 0xBF58476D1CE4E5B9 + 0x94D049BB133111EB) & MASK64


class CountingTransform:
    def __init__(self):
        self.count = 0

    def __call__(self, state, index):
        self.count += 1
        return python_transform(int(state), int(index))


def _cppyy_kernel():
    import cppyy

    cppyy.include(str(KERNEL))
    return cppyy, cppyy.gbl.rclcppyy_boundary_benchmark


def _run_boundary(kernel, cppyy, iterations, seed):
    callback_type = cppyy.gbl.std.function[
        "std::uint64_t(std::uint64_t,std::uint64_t)"]
    warmup_callback = CountingTransform()
    warmup = callback_type(warmup_callback)
    kernel.run_python_boundary(min(iterations, 1000), seed, warmup)

    callback = CountingTransform()
    wrapped = callback_type(callback)
    cpu_start = time.process_time_ns()
    wall_start = time.perf_counter_ns()
    checksum = kernel.run_python_boundary(iterations, seed, wrapped)
    elapsed_ns = time.perf_counter_ns() - wall_start
    cpu_time_ns = time.process_time_ns() - cpu_start
    return int(checksum), callback.count, elapsed_ns, cpu_time_ns


def _run_fused(kernel, _cppyy, iterations, seed):
    kernel.run_fused(min(iterations, 10000), seed)
    cpu_start = time.process_time_ns()
    wall_start = time.perf_counter_ns()
    checksum = kernel.run_fused(iterations, seed)
    elapsed_ns = time.perf_counter_ns() - wall_start
    cpu_time_ns = time.process_time_ns() - cpu_start
    return int(checksum), 0, elapsed_ns, cpu_time_ns


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--variant", choices=("cppyy-python-boundary", "cppyy-fused"), required=True)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run-token", required=True)
    args = parser.parse_args()
    if args.iterations <= 0:
        parser.error("--iterations must be positive")
    if args.seed < 0 or args.seed > MASK64:
        parser.error("--seed must fit uint64")

    setup_start = time.perf_counter_ns()
    cppyy, kernel = _cppyy_kernel()
    setup_ns = time.perf_counter_ns() - setup_start
    if args.variant == "cppyy-python-boundary":
        result = _run_boundary(kernel, cppyy, args.iterations, args.seed)
        model = "cppyy-cpp-to-python-to-cpp-callback"
        evidence = "compiled C++ loop returned through observed Python callbacks"
    else:
        result = _run_fused(kernel, cppyy, args.iterations, args.seed)
        model = "cppyy-jit-fused-cpp"
        evidence = "compiled C++ loop completed without a Python callback"
    checksum, callback_count, elapsed_ns, cpu_time_ns = result

    document = {
        "schema": "rclcppyy.boundary-sample/v1",
        "run_token": args.run_token,
        "variant": args.variant,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "iterations": args.iterations,
        "checksum": checksum,
        "elapsed_ns": elapsed_ns,
        "cpu_time_ns": cpu_time_ns,
        "setup_ns": setup_ns,
        "python_callback_count": callback_count,
        "backend": {
            "schema": "rclcppyy.boundary-backend/v1",
            "backend": "cppyy",
            "execution_model": model,
            "evidence": evidence,
        },
    }
    print(json.dumps(document, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
