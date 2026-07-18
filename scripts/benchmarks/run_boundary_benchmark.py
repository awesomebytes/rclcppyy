#!/usr/bin/env python3
"""Characterize the native Python callback boundary against fused and AOT C++."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from _boundary_protocol import (
    VARIANTS,
    build_document,
    dumps,
    validate_sample,
    write,
)
from _domain_lease import acquire_domain
from boundary_worker import python_transform


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
PYTHON_WORKER = HERE / "boundary_worker.py"
AOT_SOURCE = HERE / "aot_boundary_worker.cpp"
KERNEL = HERE / "boundary_kernel.hpp"
DEFAULT_VARIANTS = tuple(VARIANTS)
DEFAULT_SEED = 1469598103934665603
MAX_ITERATIONS = 10_000_000
MAX_REPETITIONS = 30


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _parse_variants(value):
    if value is None:
        return list(DEFAULT_VARIANTS)
    selected = []
    for item in value.split(","):
        variant = item.strip()
        if not variant:
            continue
        if variant not in VARIANTS:
            raise ValueError(
                f"unknown variant {variant!r}; choose from: {', '.join(VARIANTS)}")
        if variant not in selected:
            selected.append(variant)
    if not selected:
        raise ValueError("at least one boundary benchmark variant is required")
    return selected


def _compiler_version(compiler):
    process = subprocess.run(
        [compiler, "--version"], capture_output=True, text=True, timeout=10)
    if process.returncode != 0:
        raise RuntimeError(f"compiler version probe failed: {process.stderr.strip()}")
    return process.stdout.splitlines()[0]


def _compile_aot(build_directory, compiler_name):
    compiler = shutil.which(compiler_name)
    if compiler is None:
        raise RuntimeError(f"AOT compiler not found: {compiler_name}")
    executable = build_directory / "aot_boundary_worker"
    flags = ["-std=c++17", "-O3", "-DNDEBUG"]
    command = [
        compiler, *flags, "-I", str(HERE), str(AOT_SOURCE), "-o", str(executable)]
    started = time.perf_counter_ns()
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=120)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError("AOT compilation timed out after 120 seconds") from exc
    except BaseException:
        _stop_process(process)
        raise
    build_elapsed_ns = time.perf_counter_ns() - started
    if process.returncode != 0:
        raise RuntimeError(
            "AOT compilation failed:\n" + (stderr or stdout).strip())
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError("AOT compiler did not produce an executable")
    if executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("AOT worker is not an ELF executable")
    return executable, {
        "compiler": compiler,
        "compiler_version": _compiler_version(compiler),
        "flags": flags,
        "source_sha256": _sha256(AOT_SOURCE),
        "kernel_sha256": _sha256(KERNEL),
        "executable_sha256": _sha256(executable),
        "executable_format": "ELF",
        "build_elapsed_ns": build_elapsed_ns,
        "build_directory_persisted": False,
    }


def _expected_checksum(iterations, seed):
    state = seed
    for index in range(iterations):
        state = python_transform(state, index)
    return state


def _stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def _run_process(argv, env, timeout):
    process = subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
        start_new_session=True,
    )
    started = time.perf_counter_ns()
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError(f"sample timed out after {timeout:.1f}s") from exc
    except BaseException:
        _stop_process(process)
        raise
    total_wall_ns = time.perf_counter_ns() - started
    if process.returncode != 0:
        detail = (stderr or stdout).strip()
        raise RuntimeError(
            f"sample exited with code {process.returncode}: {detail[-1000:]}")
    lines = [line for line in stdout.splitlines() if line.strip()]
    if len(lines) != 1:
        raise RuntimeError("sample must emit exactly one JSON document on stdout")
    try:
        sample = json.loads(lines[0])
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"sample emitted invalid JSON: {exc}") from exc
    return process.pid, sample, total_wall_ns, stderr.strip()


def _sample_argv(variant, executable, iterations, seed, run_token):
    if variant == "aot-cpp":
        return [str(executable), str(iterations), str(seed), run_token]
    return [
        sys.executable,
        "-u",
        str(PYTHON_WORKER),
        "--variant", variant,
        "--iterations", str(iterations),
        "--seed", str(seed),
        "--run-token", run_token,
    ]


def _run_sample(
        *, variant, repetition, executable, aot_build, iterations, seed,
        expected_checksum, timeout, env):
    run_token = "run_" + uuid.uuid4().hex
    argv = _sample_argv(variant, executable, iterations, seed, run_token)
    process_id, sample, total_wall_ns, stderr = _run_process(argv, env, timeout)
    validate_sample(
        sample,
        variant=variant,
        run_token=run_token,
        iterations=iterations,
        checksum=expected_checksum,
        process_id=process_id,
    )
    source_digest = (
        aot_build["source_sha256"] if variant == "aot-cpp"
        else _sha256(PYTHON_WORKER)
    )
    sample.update({
        "case_id": f"{variant}__rep_{repetition}",
        "repetition": repetition,
        "backend_verified": True,
        "ns_per_iteration": sample["elapsed_ns"] / iterations,
        "process_total_wall_ns": total_wall_ns,
        "worker_source_sha256": source_digest,
        "kernel_sha256": aot_build["kernel_sha256"],
        "stderr": stderr or None,
    })
    if variant == "aot-cpp":
        sample["backend"]["executable_sha256"] = aot_build["executable_sha256"]
    return sample


def _print_table(results):
    print("\nCharacterization only: raw metrics are not performance claims.")
    print("  %-24s %4s %14s %14s %12s" % (
        "variant", "rep", "ns/iteration", "cpu ns/iter", "callbacks"))
    print("  " + "-" * 72)
    for row in results:
        print("  %-24s %4d %14.3f %14.3f %12d" % (
            row["variant"], row["repetition"], row["ns_per_iteration"],
            row["cpu_time_ns"] / row["iterations"],
            row["python_callback_count"],
        ))


def _parser():
    parser = argparse.ArgumentParser(
        description="Characterize a native C++/Python callback boundary against AOT C++.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--variants", help="comma-separated boundary variants")
    parser.add_argument("--iterations", type=int)
    parser.add_argument("--repetitions", type=int)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser


def main():
    parser = _parser()
    args = parser.parse_args()
    try:
        variants = _parse_variants(args.variants)
    except ValueError as exc:
        parser.error(str(exc))
    iterations = args.iterations if args.iterations is not None else (
        2000 if args.smoke else 50000)
    repetitions = args.repetitions if args.repetitions is not None else (
        1 if args.smoke else 5)
    if not 1 <= iterations <= MAX_ITERATIONS:
        parser.error(f"--iterations must be between 1 and {MAX_ITERATIONS}")
    if not 1 <= repetitions <= MAX_REPETITIONS:
        parser.error(f"--repetitions must be between 1 and {MAX_REPETITIONS}")
    if not 0 <= args.seed <= (1 << 64) - 1:
        parser.error("--seed must fit uint64")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    mode = "smoke" if args.smoke else "measurement"
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    with tempfile.TemporaryDirectory(prefix="rclcppyy-aot-boundary-") as temporary:
        build_directory = Path(temporary)
        build_directory.chmod(0o700)
        executable, aot_build = _compile_aot(build_directory, args.compiler)
        expected_checksum = _expected_checksum(iterations, args.seed)
        results = []
        failures = []
        with acquire_domain() as lease:
            os.environ["ROS_DOMAIN_ID"] = str(lease.domain_id)
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            try:
                execution_order = []
                for repetition in range(1, repetitions + 1):
                    offset = (repetition - 1) % len(variants)
                    ordered_variants = variants[offset:] + variants[:offset]
                    for variant in ordered_variants:
                        execution_order.append(f"{variant}__rep_{repetition}")
                        try:
                            results.append(_run_sample(
                                variant=variant,
                                repetition=repetition,
                                executable=executable,
                                aot_build=aot_build,
                                iterations=iterations,
                                seed=args.seed,
                                expected_checksum=expected_checksum,
                                timeout=args.timeout,
                                env=env,
                            ))
                        except (OSError, RuntimeError, ValueError) as exc:
                            failures.append({
                                "case_id": f"{variant}__rep_{repetition}",
                                "variant": variant,
                                "repetition": repetition,
                                "error": str(exc),
                            })
                isolation = {
                    "fresh_process_per_sample": True,
                    "new_process_group_per_sample": True,
                    "unique_run_token_per_sample": True,
                    "private_aot_build_directory": True,
                    "ros_domain_id": lease.domain_id,
                    "sample_process_ids": [row["pid"] for row in results],
                    "sample_run_tokens": [row["run_token"] for row in results],
                }
                parameters = {
                    "variants": variants,
                    "iterations": iterations,
                    "repetitions": repetitions,
                    "seed": args.seed,
                    "expected_checksum": expected_checksum,
                    "timeout_s": args.timeout,
                    "execution_order": execution_order,
                }
                document = build_document(
                    repo_root=REPO_ROOT,
                    mode=mode,
                    parameters=parameters,
                    isolation=isolation,
                    aot_build=aot_build,
                    results=results,
                    failures=failures,
                )
                if args.output is not None:
                    write(document, args.output)
                if args.json:
                    print(dumps(document), end="")
                else:
                    _print_table(results)
                    if failures:
                        print(f"{len(failures)} sample(s) failed", file=sys.stderr)
                return 1 if failures else 0
            finally:
                if previous_domain is None:
                    os.environ.pop("ROS_DOMAIN_ID", None)
                else:
                    os.environ["ROS_DOMAIN_ID"] = previous_domain


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
