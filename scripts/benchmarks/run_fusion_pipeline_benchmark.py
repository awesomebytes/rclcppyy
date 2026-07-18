#!/usr/bin/env python3
"""Run the controlled Jazzy/CycloneDDS C++ topology-fusion benchmark."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

import psutil

from _domain_lease import acquire_domain
from _fusion_pipeline_protocol import (
    DRIVER_SCHEMA,
    QOS,
    SAMPLE_SCHEMA,
    VARIANTS,
    build_document,
    dumps,
    latency_summary,
    validate_prewarm,
    validate_sample,
    write,
)


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "fusion_pipeline_worker.py"
PROTOCOL = HERE / "_fusion_pipeline_protocol.py"
JSON_SCHEMA = HERE / "fusion_pipeline.schema.json"
KERNEL = HERE / "fusion_pipeline_kernel.hpp"
AOT_DIR = HERE / "fusion_pipeline_aot"
AOT_SOURCE = AOT_DIR / "fusion_pipeline_aot.cpp"
AOT_CMAKE = AOT_DIR / "CMakeLists.txt"
PREFIX = "@@RCLCPPYY_FUSION_PIPELINE_V1@@"
DEFAULT_VARIANTS = tuple(VARIANTS)
MAX_MESSAGES = 100_000
MAX_REPETITIONS = 30


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def _command(command: list[str], *, env: dict, timeout: float, label: str) -> str:
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, env=env, start_new_session=True)
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop(process)
        raise RuntimeError("%s timed out" % label) from exc
    if process.returncode != 0:
        raise RuntimeError(
            "%s failed with code %d: %s" % (
                label, process.returncode, (stderr or stdout).strip()[-3000:]))
    return stdout


def _compiler(cache: Path) -> str:
    for line in cache.read_text(encoding="utf-8").splitlines():
        if line.startswith("CMAKE_CXX_COMPILER:FILEPATH="):
            return line.split("=", 1)[1]
    raise RuntimeError("CMake did not identify the C++ compiler")


def _compile_aot(directory: Path, env: dict, timeout: float) -> tuple[Path, dict]:
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required")
    configure = [
        cmake, "-S", str(AOT_DIR), "-B", str(directory), "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
    ]
    if env.get("CONDA_PREFIX"):
        configure.append("-DCMAKE_PREFIX_PATH=" + env["CONDA_PREFIX"])
    started = time.perf_counter_ns()
    _command(configure, env=env, timeout=timeout, label="fusion AOT configure")
    _command(
        [cmake, "--build", str(directory), "--target", "fusion_pipeline_aot"],
        env=env, timeout=timeout, label="fusion AOT build")
    elapsed = time.perf_counter_ns() - started
    executable = directory / "fusion_pipeline_aot"
    commands_path = directory / "compile_commands.json"
    commands = json.loads(commands_path.read_text(encoding="utf-8"))
    matches = [
        row for row in commands if Path(row["file"]).resolve() == AOT_SOURCE.resolve()]
    if len(matches) != 1:
        raise RuntimeError("fusion AOT compile command is missing or ambiguous")
    compile_command = matches[0].get("command") or " ".join(matches[0]["arguments"])
    if "-O3" not in compile_command or "-DNDEBUG" not in compile_command:
        raise RuntimeError("fusion AOT compile command is not Release optimized")
    if not executable.is_file() or executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("fusion AOT build produced no ELF executable")
    compiler = _compiler(directory / "CMakeCache.txt")
    compiler_version = _command(
        [compiler, "--version"], env=env, timeout=10,
        label="compiler version").splitlines()[0]
    return executable, {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": compiler,
        "compiler_version": compiler_version,
        "compile_command": compile_command,
        "source_sha256": _sha256(AOT_SOURCE),
        "kernel_sha256": _sha256(KERNEL),
        "cmake_sha256": _sha256(AOT_CMAKE),
        "compile_commands_sha256": _sha256(commands_path),
        "executable_sha256": _sha256(executable),
        "executable_format": "ELF",
        "build_elapsed_ns": elapsed,
    }


def _one_record(stdout: str, label: str) -> tuple[dict, list[str]]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    protocol = [line for line in lines if line.startswith(PREFIX)]
    if len(protocol) != 1:
        raise RuntimeError("%s must emit exactly one protocol record" % label)
    try:
        value = json.loads(protocol[0][len(PREFIX):])
    except json.JSONDecodeError as exc:
        raise RuntimeError("%s emitted invalid protocol JSON" % label) from exc
    return value, [line for line in lines if not line.startswith(PREFIX)]


def _prewarm(cache_root: Path, env: dict, timeout: float, requested_rmw: str) -> dict:
    if any(cache_root.iterdir()):
        raise RuntimeError("fusion cache must start empty")
    command = [sys.executable, "-u", str(WORKER), "--prewarm"]
    cold, cold_output = _one_record(
        _command(command, env=env, timeout=timeout, label="cold fusion prewarm"),
        "cold fusion prewarm")
    warm, warm_output = _one_record(
        _command(command, env=env, timeout=timeout, label="warm fusion prewarm"),
        "warm fusion prewarm")
    cold["stdout_diagnostics"] = cold_output
    warm["stdout_diagnostics"] = warm_output
    validate_prewarm(cold, cached=False, requested_rmw=requested_rmw)
    validate_prewarm(warm, cached=True, requested_rmw=requested_rmw)
    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": cold, "warm": warm},
    }


def _spawn(argv: list[str], env: dict) -> subprocess.Popen:
    return subprocess.Popen(
        argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1, env=env, start_new_session=True)


def _read(process: subprocess.Popen, timeout: float, label: str) -> tuple[dict, list[str]]:
    deadline = time.monotonic() + timeout
    diagnostics = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("%s timed out" % label)
        ready, _, _ = select.select([process.stdout], [], [], remaining)
        if not ready:
            raise RuntimeError("%s timed out" % label)
        line = process.stdout.readline()
        if not line:
            stderr = process.stderr.read()
            raise RuntimeError(
                "%s exited with code %s: %s" % (
                    label, process.poll(), stderr.strip()[-3000:]))
        line = line.rstrip("\n")
        if not line.startswith(PREFIX):
            if line:
                diagnostics.append(line)
            continue
        try:
            value = json.loads(line[len(PREFIX):])
        except json.JSONDecodeError as exc:
            raise RuntimeError("%s emitted invalid JSON" % label) from exc
        return value, diagnostics


def _write_control(process: subprocess.Popen, value: str, label: str) -> None:
    try:
        process.stdin.write(value + "\n")
        process.stdin.flush()
    except (BrokenPipeError, OSError) as exc:
        raise RuntimeError("%s control pipe failed" % label) from exc


def _finish(process: subprocess.Popen, timeout: float, label: str) -> dict:
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop(process)
        raise RuntimeError("%s did not terminate" % label) from exc
    if process.returncode != 0:
        raise RuntimeError(
            "%s exited with code %d: %s" % (
                label, process.returncode, stderr.strip()[-3000:]))
    trailing = [line for line in stdout.splitlines() if line.strip()]
    if any(line.startswith(PREFIX) for line in trailing):
        raise RuntimeError("%s emitted an unexpected trailing record" % label)
    return {"stdout": trailing, "stderr": stderr.strip() or None}


def _relay_argv(
        variant: str, executable: Path, topics: list[str], node: str,
        warmup: int, messages: int, token: str) -> list[str]:
    if variant == "cppyy-fused":
        return [
            sys.executable, "-u", str(WORKER),
            "--input-topic", topics[0], "--output-topic", topics[-1],
            "--node-name", node, "--warmup-messages", str(warmup),
            "--messages", str(messages), "--run-token", token,
        ]
    return [
        str(executable), "relay", variant, *topics, node,
        str(warmup), str(messages), token,
    ]


def _driver_argv(
        executable: Path, variant: str, topics: list[str], driver: str,
        relay: str, warmup: int, messages: int, token: str) -> list[str]:
    return [
        str(executable), "driver", variant, *topics, driver, relay,
        str(warmup), str(messages), token,
    ]


def _run_sample(
        *, variant: str, repetition: int, executable: Path, build: dict,
        cache: dict, warmup: int, messages: int, domain_id: int,
        requested_rmw: str, env: dict, timeout: float) -> dict:
    token = "run_" + uuid.uuid4().hex
    base = "/rclcppyy_fusion/%s" % token
    topics = [base + suffix for suffix in ("/input", "/mid1", "/mid2", "/mid3", "/output")]
    suffix = token[4:16]
    relay_node = "fusion_relay_" + suffix
    driver_node = "fusion_driver_" + suffix
    relay = None
    driver = None
    try:
        relay = _spawn(
            _relay_argv(
                variant, executable, topics, relay_node, warmup, messages, token), env)
        ready, ready_diagnostics = _read(relay, timeout, "%s relay ready" % variant)
        if ready.get("pid") != relay.pid or ready.get("process_group_id") != relay.pid:
            raise RuntimeError("relay did not use its fresh process group")
        driver = _spawn(
            _driver_argv(
                executable, variant, topics, driver_node, relay_node,
                warmup, messages, token), env)
        warmed, warmed_diagnostics = _read(driver, timeout, "fusion driver warmup")
        if warmed.get("schema") != DRIVER_SCHEMA or warmed.get("event") != "warmed":
            raise RuntimeError("driver emitted invalid warmed evidence")
        if warmed.get("pid") != driver.pid or warmed.get("process_group_id") != driver.pid:
            raise RuntimeError("driver did not use its fresh process group")
        if psutil.Process(relay.pid).children(recursive=True) or psutil.Process(
                driver.pid).children(recursive=True):
            raise RuntimeError("sample retained setup children before measurement")
        _write_control(relay, "ARM", "%s relay" % variant)
        armed, armed_diagnostics = _read(relay, timeout, "%s relay armed" % variant)
        _write_control(driver, "START", "fusion driver")
        measured, measured_diagnostics = _read(driver, timeout, "fusion driver measured")
        _write_control(relay, "REPORT", "%s relay" % variant)
        report, report_diagnostics = _read(relay, timeout, "%s relay report" % variant)
        relay_finish = _finish(relay, timeout, "%s relay" % variant)
        _write_control(driver, "TEARDOWN", "fusion driver")
        teardown, teardown_diagnostics = _read(driver, timeout, "fusion driver teardown")
        driver_finish = _finish(driver, timeout, "fusion driver")
        relay_cpu = report.get("cpu_time_ns")
        timing = {
            "relay_cpu_time_ns": relay_cpu,
            "relay_cpu_ns_per_message": relay_cpu / messages,
            "driver_cpu_time_ns": measured.get("cpu_time_ns"),
            "driver_cpu_ns_per_message": measured.get("cpu_time_ns") / messages,
            "elapsed_ns": measured.get("elapsed_ns"),
            "throughput_messages_per_second": messages * 1e9 / measured["elapsed_ns"],
            "latency_ns": latency_summary(measured["latency_ns"]),
        }
        sample = {
            "schema": SAMPLE_SCHEMA,
            "case_id": "%s__rep_%d" % (variant, repetition),
            "variant": variant,
            "repetition": repetition,
            "run_token": token,
            "ros_domain_id": domain_id,
            "requested_rmw": requested_rmw,
            "qos": dict(QOS),
            "relay_pid": relay.pid,
            "driver_pid": driver.pid,
            "topology": {
                "process_count": 2,
                "fresh_process_groups": True,
                "driver_node": driver_node,
                "relay_node": relay_node,
                "all_topics": topics,
                "contract_change": {
                    "external_input_output_unchanged": True,
                    "intermediate_topics_observable": variant == "aot-staged",
                    "removed_intermediate_topics": 0 if variant == "aot-staged" else 3,
                },
            },
            "relay_ready": ready,
            "driver_warmed": warmed,
            "relay_armed": armed,
            "driver_measured": measured,
            "relay_report": report,
            "driver_teardown": teardown,
            "timing": timing,
            "backend_verified": True,
            "correctness_verified": True,
            "teardown_verified": True,
            "diagnostics": {
                "relay_stdout": (
                    ready_diagnostics + armed_diagnostics
                    + report_diagnostics + relay_finish["stdout"]),
                "driver_stdout": (
                    warmed_diagnostics + measured_diagnostics
                    + teardown_diagnostics + driver_finish["stdout"]),
                "relay_stderr": relay_finish["stderr"],
                "driver_stderr": driver_finish["stderr"],
            },
        }
        validate_sample(sample, {
            "requested_rmw": requested_rmw,
            "warmup_messages": warmup,
            "messages": messages,
        }, build, cache)
        return sample
    finally:
        _stop(driver)
        _stop(relay)


def _parse_variants(value: str | None) -> list[str]:
    if value is None:
        return list(DEFAULT_VARIANTS)
    selected = []
    for item in value.split(","):
        name = item.strip()
        if not name:
            continue
        if name not in VARIANTS:
            raise ValueError("unknown fusion variant %r" % name)
        if name not in selected:
            selected.append(name)
    if not selected:
        raise ValueError("at least one fusion variant is required")
    return selected


def _print(results: list[dict]) -> None:
    print("\nRaw characterization only; topology-fusion claims are disabled.")
    print("  %-16s %3s %14s %12s %12s %12s" % (
        "variant", "rep", "relay CPU ns", "p50 ns", "p99 ns", "msg/s"))
    for row in results:
        timing = row["timing"]
        print("  %-16s %3d %14.1f %12d %12d %12.1f" % (
            row["variant"], row["repetition"], timing["relay_cpu_ns_per_message"],
            timing["latency_ns"]["p50"], timing["latency_ns"]["p99"],
            timing["throughput_messages_per_second"]))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--variants", help="comma-separated fusion variants")
    parser.add_argument("--messages", type=int)
    parser.add_argument("--warmup-messages", type=int)
    parser.add_argument("--repetitions", type=int)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        variants = _parse_variants(args.variants)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    messages = args.messages if args.messages is not None else (8 if args.smoke else 2000)
    warmup = args.warmup_messages if args.warmup_messages is not None else (
        2 if args.smoke else 100)
    repetitions = args.repetitions if args.repetitions is not None else (
        1 if args.smoke else 5)
    if not 1 <= messages <= MAX_MESSAGES or not 1 <= warmup <= MAX_MESSAGES:
        raise SystemExit("message counts must be between 1 and %d" % MAX_MESSAGES)
    if not 1 <= repetitions <= MAX_REPETITIONS:
        raise SystemExit("repetitions must be between 1 and %d" % MAX_REPETITIONS)
    if args.timeout <= 0:
        raise SystemExit("timeout must be positive")
    requested_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if requested_rmw != "rmw_cyclonedds_cpp":
        raise SystemExit("this first-target benchmark requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise SystemExit("this first-target benchmark requires ROS_DISTRO=jazzy")
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["CPPYY_KIT_NO_AUTOPCH"] = "1"
    env.pop("CPPYY_KIT_NO_CACHE", None)
    results = []
    failures = []
    execution_order = []
    with tempfile.TemporaryDirectory(prefix="rclcppyy-fusion-pipeline-") as temporary:
        root = Path(temporary)
        build_dir = root / "aot-build"
        cache_root = root / "cache"
        build_dir.mkdir(mode=0o700)
        cache_root.mkdir(mode=0o700)
        env["XDG_CACHE_HOME"] = str(cache_root)
        executable, build = _compile_aot(build_dir, env, args.timeout)
        cache = _prewarm(cache_root, env, args.timeout, requested_rmw)
        with acquire_domain() as lease:
            env["ROS_DOMAIN_ID"] = str(lease.domain_id)
            for repetition in range(1, repetitions + 1):
                offset = (repetition - 1) % len(variants)
                for variant in variants[offset:] + variants[:offset]:
                    case_id = "%s__rep_%d" % (variant, repetition)
                    execution_order.append(case_id)
                    try:
                        results.append(_run_sample(
                            variant=variant, repetition=repetition,
                            executable=executable, build=build, cache=cache,
                            warmup=warmup, messages=messages,
                            domain_id=lease.domain_id, requested_rmw=requested_rmw,
                            env=env, timeout=args.timeout))
                    except (OSError, psutil.Error, RuntimeError, ValueError) as exc:
                        failures.append({
                            "case_id": case_id,
                            "variant": variant,
                            "repetition": repetition,
                            "error": str(exc),
                        })
            parameters = {
                "variants": variants,
                "messages": messages,
                "warmup_messages": warmup,
                "repetitions": repetitions,
                "requested_rmw": requested_rmw,
                "ros_distro": "jazzy",
                "qos": dict(QOS),
                "execution_order": execution_order,
            }
            isolation = {
                "fresh_process_pair_per_sample": True,
                "two_process_groups_per_sample": True,
                "unique_topics_per_sample": True,
                "one_leased_domain_per_run": True,
                "rotating_variant_order": True,
                "exact_warmup_measurement_barriers": True,
                "ros_domain_id": lease.domain_id,
            }
            source_files = {
                "runner": _sha256(Path(__file__)),
                "protocol": _sha256(PROTOCOL),
                "json_schema": _sha256(JSON_SCHEMA),
                "worker": _sha256(WORKER),
                "kernel": _sha256(KERNEL),
                "aot_source": _sha256(AOT_SOURCE),
                "aot_cmake": _sha256(AOT_CMAKE),
            }
            document = build_document(
                repo_root=REPO_ROOT,
                mode="smoke" if args.smoke else "measurement",
                parameters=parameters,
                isolation=isolation,
                aot_build=build,
                cache=cache,
                source_files=source_files,
                results=results,
                failures=failures,
                command=[sys.executable, str(Path(__file__)), *sys.argv[1:]],
            )
            if args.output is not None:
                write(document, args.output)
            if args.json:
                print(dumps(document), end="")
            else:
                _print(results)
                if failures:
                    print("%d sample(s) failed" % len(failures), file=sys.stderr)
            return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
