#!/usr/bin/env python3
"""Run the controlled five-variant ROS relay-boundary benchmark."""

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
from _relay_boundary_protocol import (
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
WORKER = HERE / "relay_boundary_worker.py"
KERNEL = HERE / "relay_boundary_kernel.hpp"
AOT_DIRECTORY = HERE / "relay_boundary_aot"
AOT_SOURCE = AOT_DIRECTORY / "relay_boundary_aot.cpp"
AOT_CMAKE = AOT_DIRECTORY / "CMakeLists.txt"
PROTOCOL = HERE / "_relay_boundary_protocol.py"
DEFAULT_VARIANTS = tuple(VARIANTS)
MAX_MESSAGES = 100_000
MAX_REPETITIONS = 30
PROTOCOL_PREFIX = "@@RCLCPPYY_RELAY_BOUNDARY_V1@@"


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def _run_command(command: list[str], *, env: dict, timeout: float, label: str) -> str:
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError("%s timed out after %.1fs" % (label, timeout)) from exc
    if process.returncode != 0:
        raise RuntimeError(
            "%s failed with code %d: %s" % (
                label, process.returncode, (stderr or stdout).strip()[-2000:]))
    return stdout


def _compiler_from_cache(cache: Path) -> str:
    for line in cache.read_text(encoding="utf-8").splitlines():
        if line.startswith("CMAKE_CXX_COMPILER:FILEPATH="):
            return line.split("=", 1)[1]
    raise RuntimeError("CMake cache did not identify the C++ compiler")


def _compiler_version(compiler: str, env: dict) -> str:
    output = _run_command(
        [compiler, "--version"], env=env, timeout=10, label="compiler version probe")
    return output.splitlines()[0]


def _compile_aot(build_directory: Path, env: dict, timeout: float) -> tuple[Path, dict]:
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required for the relay AOT reference")
    configure = [
        cmake,
        "-S", str(AOT_DIRECTORY),
        "-B", str(build_directory),
        "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
    ]
    if env.get("CONDA_PREFIX"):
        configure.append("-DCMAKE_PREFIX_PATH=" + env["CONDA_PREFIX"])
    started = time.perf_counter_ns()
    _run_command(configure, env=env, timeout=timeout, label="relay AOT configure")
    _run_command(
        [cmake, "--build", str(build_directory), "--target", "relay_boundary_aot"],
        env=env,
        timeout=timeout,
        label="relay AOT build",
    )
    build_elapsed_ns = time.perf_counter_ns() - started
    executable = build_directory / "relay_boundary_aot"
    compile_commands = build_directory / "compile_commands.json"
    cmake_cache = build_directory / "CMakeCache.txt"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError("relay AOT build produced no executable")
    if executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("relay AOT executable is not ELF")
    commands = json.loads(compile_commands.read_text(encoding="utf-8"))
    matches = [
        row for row in commands
        if Path(row["file"]).resolve() == AOT_SOURCE.resolve()
    ]
    if len(matches) != 1:
        raise RuntimeError("relay AOT compile command is missing or ambiguous")
    compile_command = matches[0].get("command") or " ".join(matches[0]["arguments"])
    if "-O3" not in compile_command or "-DNDEBUG" not in compile_command:
        raise RuntimeError("relay AOT compile command does not prove Release optimization")
    compiler = _compiler_from_cache(cmake_cache)
    return executable, {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": compiler,
        "compiler_version": _compiler_version(compiler, env),
        "compile_command": compile_command,
        "source_sha256": _sha256(AOT_SOURCE),
        "kernel_sha256": _sha256(KERNEL),
        "cmake_sha256": _sha256(AOT_CMAKE),
        "compile_commands_sha256": _sha256(compile_commands),
        "executable_sha256": _sha256(executable),
        "executable_format": "ELF",
        "build_elapsed_ns": build_elapsed_ns,
    }


def _one_protocol_line(stdout: str, label: str) -> tuple[dict, list[str]]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    protocol_lines = [line for line in lines if line.startswith(PROTOCOL_PREFIX)]
    if len(protocol_lines) != 1:
        raise RuntimeError("%s must emit exactly one sentinel protocol record" % label)
    try:
        document = json.loads(protocol_lines[0][len(PROTOCOL_PREFIX):])
    except json.JSONDecodeError as exc:
        raise RuntimeError("%s emitted invalid sentinel JSON" % label) from exc
    if not isinstance(document, dict):
        raise RuntimeError("%s JSON must be an object" % label)
    diagnostics = [line for line in lines if not line.startswith(PROTOCOL_PREFIX)]
    return document, diagnostics


def _prewarm(cache_root: Path, env: dict, timeout: float) -> dict:
    if cache_root.exists() and any(cache_root.iterdir()):
        raise RuntimeError("relay cache root must start empty")
    command = [sys.executable, "-u", str(WORKER), "--prewarm"]
    cold, cold_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="cold relay prewarm"),
        "cold relay prewarm",
    )
    warm, warm_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="warm relay prewarm"),
        "warm relay prewarm",
    )
    validate_prewarm(cold, expect_hits=False)
    validate_prewarm(warm, expect_hits=True)
    cold["stdout_diagnostics"] = cold_diagnostics
    warm["stdout_diagnostics"] = warm_diagnostics
    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": cold, "warm": warm},
    }


def _read_document(
        process: subprocess.Popen, timeout: float, label: str) -> tuple[dict, list[str]]:
    deadline = time.monotonic() + timeout
    diagnostics = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("%s timed out after %.1fs" % (label, timeout))
        ready, _, _ = select.select([process.stdout], [], [], remaining)
        if not ready:
            raise RuntimeError("%s timed out after %.1fs" % (label, timeout))
        line = process.stdout.readline()
        if not line:
            stderr = process.stderr.read()
            raise RuntimeError(
                "%s exited with code %s before its sentinel record: %s" % (
                    label, process.poll(), stderr.strip()[-2000:]))
        line = line.rstrip("\n")
        if not line.startswith(PROTOCOL_PREFIX):
            if line:
                diagnostics.append(line)
            continue
        try:
            value = json.loads(line[len(PROTOCOL_PREFIX):])
        except json.JSONDecodeError as exc:
            raise RuntimeError("%s emitted invalid sentinel JSON" % label) from exc
        if not isinstance(value, dict):
            raise RuntimeError("%s sentinel JSON must be an object" % label)
        return value, diagnostics


def _finish(process: subprocess.Popen, timeout: float, label: str) -> dict:
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError("%s did not terminate" % label) from exc
    stdout_lines = [line for line in stdout.splitlines() if line.strip()]
    if any(line.startswith(PROTOCOL_PREFIX) for line in stdout_lines):
        raise RuntimeError("%s emitted an unexpected trailing sentinel record" % label)
    if process.returncode != 0:
        raise RuntimeError(
            "%s exited with code %d: %s" % (
                label, process.returncode, stderr.strip()[-2000:]))
    return {
        "stdout": stdout_lines,
        "stderr": stderr.strip() or None,
    }


def _write_control(process: subprocess.Popen, command: str, label: str) -> None:
    try:
        process.stdin.write(command + "\n")
        process.stdin.flush()
    except (BrokenPipeError, OSError) as exc:
        raise RuntimeError("%s control pipe failed" % label) from exc


def _relay_argv(
        variant: str, executable: Path, input_topic: str, output_topic: str,
        relay_node: str, warmup: int, messages: int, token: str) -> list[str]:
    if variant == "aot-staged":
        return [
            str(executable), "relay", input_topic, output_topic, relay_node,
            str(warmup), str(messages), token,
        ]
    return [
        sys.executable, "-u", str(WORKER),
        "--variant", variant,
        "--input-topic", input_topic,
        "--output-topic", output_topic,
        "--node-name", relay_node,
        "--warmup-messages", str(warmup),
        "--messages", str(messages),
        "--run-token", token,
    ]


def _driver_argv(
        executable: Path, input_topic: str, output_topic: str,
        driver_node: str, relay_node: str, warmup: int, messages: int,
        token: str) -> list[str]:
    return [
        str(executable), "driver", input_topic, output_topic,
        driver_node, relay_node, str(warmup), str(messages), token,
    ]


def _spawn(argv: list[str], env: dict) -> subprocess.Popen:
    return subprocess.Popen(
        argv,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        env=env,
        start_new_session=True,
    )


def _run_sample(
        *, variant: str, repetition: int, executable: Path, build: dict,
        cache: dict, warmup: int, messages: int, domain_id: int,
        requested_rmw: str, env: dict, timeout: float) -> dict:
    token = "run_" + uuid.uuid4().hex
    suffix = token[4:16]
    input_topic = "/rclcppyy_relay_boundary/%s/input" % token
    output_topic = "/rclcppyy_relay_boundary/%s/output" % token
    relay_node = "relay_boundary_relay_" + suffix
    driver_node = "relay_boundary_driver_" + suffix
    relay = None
    driver = None
    try:
        relay = _spawn(_relay_argv(
            variant, executable, input_topic, output_topic,
            relay_node, warmup, messages, token), env)
        ready, relay_ready_diagnostics = _read_document(
            relay, timeout, "%s relay readiness" % variant)
        if ready.get("pid") != relay.pid or ready.get("process_group_id") != relay.pid:
            raise RuntimeError("relay did not run in its fresh process group")

        driver = _spawn(_driver_argv(
            executable, input_topic, output_topic,
            driver_node, relay_node, warmup, messages, token), env)
        warmed, driver_warm_diagnostics = _read_document(
            driver, timeout, "AOT driver warmup")
        if warmed.get("schema") != DRIVER_SCHEMA or warmed.get("event") != "warmed":
            raise RuntimeError("AOT driver emitted invalid warmup evidence")
        if warmed.get("pid") != driver.pid or warmed.get("process_group_id") != driver.pid:
            raise RuntimeError("AOT driver did not run in its fresh process group")
        if warmed.get("run_token") != token or warmed.get("warmup_messages") != warmup:
            raise RuntimeError("AOT driver warmup identity is invalid")
        if warmed.get("loaded_rmw") != requested_rmw:
            raise RuntimeError("AOT driver warmup used the wrong RMW")

        relay_process = psutil.Process(relay.pid)
        driver_process = psutil.Process(driver.pid)
        if relay_process.children(recursive=True) or driver_process.children(recursive=True):
            raise RuntimeError("benchmark processes retained setup children before measurement")
        _write_control(relay, "START", "%s relay" % variant)
        _write_control(driver, "START", "AOT driver")
        driver_result, driver_result_diagnostics = _read_document(
            driver, timeout, "AOT driver measured result")
        _write_control(relay, "REPORT", "%s relay" % variant)
        report, relay_report_diagnostics = _read_document(
            relay, timeout, "%s relay report" % variant)
        _write_control(driver, "TEARDOWN", "AOT driver")
        driver_teardown, driver_teardown_diagnostics = _read_document(
            driver, timeout, "AOT driver teardown")
        driver_finish = _finish(driver, timeout, "AOT driver")
        relay_finish = _finish(relay, timeout, "%s relay" % variant)
        relay_cpu_ns = report.get("cpu_time_ns")
        timing = {
            "elapsed_ns": driver_result.get("elapsed_ns"),
            "throughput_messages_per_second": (
                messages * 1e9 / driver_result["elapsed_ns"]
                if driver_result.get("elapsed_ns", 0) > 0 else 0.0
            ),
            "relay_cpu_time_ns": relay_cpu_ns,
            "relay_cpu_ns_per_message": (
                relay_cpu_ns / messages if isinstance(relay_cpu_ns, int) else None),
            "driver_cpu_time_ns": driver_result.get("cpu_time_ns"),
            "driver_cpu_ns_per_message": driver_result.get("cpu_time_ns", 0) / messages,
            "latency_ns": latency_summary(driver_result.get("latency_ns", [])),
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
                "input_topic": input_topic,
                "output_topic": output_topic,
            },
            "relay_ready": ready,
            "relay_report": report,
            "driver_result": driver_result,
            "driver_teardown": driver_teardown,
            "timing": timing,
            "backend_verified": True,
            "correctness_verified": True,
            "teardown_verified": True,
            "diagnostics": {
                "relay_stdout": (
                    relay_ready_diagnostics + relay_report_diagnostics +
                    relay_finish["stdout"]),
                "driver_stdout": (
                    driver_warm_diagnostics + driver_result_diagnostics +
                    driver_teardown_diagnostics + driver_finish["stdout"]),
                "relay_stderr": relay_finish["stderr"],
                "driver_stderr": driver_finish["stderr"],
            },
        }
        validate_sample(sample, {
            "requested_rmw": requested_rmw,
            "qos": QOS,
            "warmup_messages": warmup,
            "messages": messages,
        }, build, cache)
        return sample
    finally:
        _stop_process(driver)
        _stop_process(relay)


def _parse_variants(value: str | None) -> list[str]:
    if value is None:
        return list(DEFAULT_VARIANTS)
    selected = []
    for part in value.split(","):
        variant = part.strip()
        if not variant:
            continue
        if variant not in VARIANTS:
            raise ValueError(
                "unknown variant %r; choose from: %s" % (variant, ", ".join(VARIANTS)))
        if variant not in selected:
            selected.append(variant)
    if not selected:
        raise ValueError("at least one relay-boundary variant is required")
    return selected


def _print_table(results: list[dict]) -> None:
    print("\nCharacterization only: raw metrics are not performance claims.")
    print("  %-28s %3s %12s %12s %12s %12s" % (
        "variant", "rep", "relay cpu", "p50 ns", "p99 ns", "msg/s"))
    print("  " + "-" * 86)
    for row in results:
        timing = row["timing"]
        print("  %-28s %3d %12.1f %12d %12d %12.1f" % (
            row["variant"], row["repetition"],
            timing["relay_cpu_ns_per_message"], timing["latency_ns"]["p50"],
            timing["latency_ns"]["p99"], timing["throughput_messages_per_second"],
        ))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--variants", help="comma-separated relay variants")
    parser.add_argument("--messages", type=int)
    parser.add_argument("--warmup-messages", type=int)
    parser.add_argument("--repetitions", type=int)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    parser = _parser()
    args = parser.parse_args()
    try:
        variants = _parse_variants(args.variants)
    except ValueError as exc:
        parser.error(str(exc))
    messages = args.messages if args.messages is not None else (8 if args.smoke else 2000)
    warmup = args.warmup_messages if args.warmup_messages is not None else (
        2 if args.smoke else 100)
    repetitions = args.repetitions if args.repetitions is not None else (
        1 if args.smoke else 5)
    if not 1 <= messages <= MAX_MESSAGES:
        parser.error("--messages must be between 1 and %d" % MAX_MESSAGES)
    if not 0 <= warmup <= MAX_MESSAGES:
        parser.error("--warmup-messages must be between 0 and %d" % MAX_MESSAGES)
    if not 1 <= repetitions <= MAX_REPETITIONS:
        parser.error("--repetitions must be between 1 and %d" % MAX_REPETITIONS)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    requested_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if not requested_rmw:
        parser.error("RMW_IMPLEMENTATION must identify the controlled backend")

    mode = "smoke" if args.smoke else "measurement"
    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"
    environment["CPPYY_KIT_NO_AUTOPCH"] = "1"
    environment.pop("CPPYY_KIT_NO_CACHE", None)
    results = []
    failures = []
    execution_order = []
    with tempfile.TemporaryDirectory(prefix="rclcppyy-relay-boundary-") as temporary:
        root = Path(temporary)
        build_directory = root / "aot-build"
        cache_root = root / "cache"
        build_directory.mkdir(mode=0o700)
        cache_root.mkdir(mode=0o700)
        environment["XDG_CACHE_HOME"] = str(cache_root)
        executable, aot_build = _compile_aot(build_directory, environment, args.timeout)
        cache = _prewarm(cache_root, environment, args.timeout)
        with acquire_domain() as lease:
            environment["ROS_DOMAIN_ID"] = str(lease.domain_id)
            for repetition in range(1, repetitions + 1):
                offset = (repetition - 1) % len(variants)
                ordered = variants[offset:] + variants[:offset]
                for variant in ordered:
                    execution_order.append("%s__rep_%d" % (variant, repetition))
                    try:
                        results.append(_run_sample(
                            variant=variant,
                            repetition=repetition,
                            executable=executable,
                            build=aot_build,
                            cache=cache,
                            warmup=warmup,
                            messages=messages,
                            domain_id=lease.domain_id,
                            requested_rmw=requested_rmw,
                            env=environment,
                            timeout=args.timeout,
                        ))
                    except (OSError, psutil.Error, RuntimeError, ValueError) as exc:
                        failures.append({
                            "case_id": "%s__rep_%d" % (variant, repetition),
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
                "qos": dict(QOS),
                "execution_order": execution_order,
            }
            isolation = {
                "fresh_process_pair_per_sample": True,
                "two_process_groups_per_sample": True,
                "unique_topics_per_sample": True,
                "one_leased_domain_per_run": True,
                "rotating_variant_order": True,
                "ros_domain_id": lease.domain_id,
            }
            source_files = {
                "runner": _sha256(Path(__file__)),
                "protocol": _sha256(PROTOCOL),
                "worker": _sha256(WORKER),
                "kernel": _sha256(KERNEL),
                "aot_source": _sha256(AOT_SOURCE),
                "aot_cmake": _sha256(AOT_CMAKE),
            }
            document = build_document(
                repo_root=REPO_ROOT,
                mode=mode,
                parameters=parameters,
                isolation=isolation,
                aot_build=aot_build,
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
                _print_table(results)
                if failures:
                    print("%d sample(s) failed" % len(failures), file=sys.stderr)
            return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
