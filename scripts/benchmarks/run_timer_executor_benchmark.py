#!/usr/bin/env python3
"""Run the CPU-first Jazzy/Cyclone timer-executor benchmark."""

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
from _timer_executor_protocol import (
    EVENT_SCHEMA,
    MEASURED_FIRINGS,
    PERIOD_NS,
    REPETITIONS,
    RMW,
    ROS_DISTRO,
    SAMPLE_SCHEMA,
    VARIANTS,
    WARMUP_FIRINGS,
    build_document,
    dumps,
    rotating_order,
    validate_prewarm,
    validate_sample,
    write,
)


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "timer_executor_worker.py"
PROTOCOL = HERE / "_timer_executor_protocol.py"
AOT_DIRECTORY = HERE / "timer_executor_aot"
AOT_SOURCE = AOT_DIRECTORY / "timer_executor_aot.cpp"
AOT_CMAKE = AOT_DIRECTORY / "CMakeLists.txt"
PROTOCOL_PREFIX = "@@RCLCPPYY_TIMER_EXECUTOR_V1@@"


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
    raise RuntimeError("timer CMake cache did not identify the compiler")


def _compile_aot(build_directory: Path, env: dict, timeout: float) -> tuple[Path, dict]:
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required for the timer AOT lane")
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
    _run_command(configure, env=env, timeout=timeout, label="timer AOT configure")
    _run_command(
        [cmake, "--build", str(build_directory), "--target", "timer_executor_aot"],
        env=env,
        timeout=timeout,
        label="timer AOT build",
    )
    elapsed = time.perf_counter_ns() - started
    executable = build_directory / "timer_executor_aot"
    compile_commands = build_directory / "compile_commands.json"
    cmake_cache = build_directory / "CMakeCache.txt"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError("timer AOT build produced no executable")
    if executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("timer AOT executable is not ELF")
    commands = json.loads(compile_commands.read_text(encoding="utf-8"))
    matches = [
        row for row in commands
        if Path(row["file"]).resolve() == AOT_SOURCE.resolve()
    ]
    if len(matches) != 1:
        raise RuntimeError("timer AOT compile command is missing or ambiguous")
    command = matches[0].get("command") or " ".join(matches[0]["arguments"])
    if "-O3" not in command or "-DNDEBUG" not in command:
        raise RuntimeError("timer AOT compile command lacks -O3/-DNDEBUG")
    compiler = _compiler_from_cache(cmake_cache)
    compiler_version = _run_command(
        [compiler, "--version"], env=env, timeout=10, label="timer compiler probe"
    ).splitlines()[0]
    return executable, {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": compiler,
        "compiler_version": compiler_version,
        "compile_command": command,
        "source_sha256": _sha256(AOT_SOURCE),
        "cmake_sha256": _sha256(AOT_CMAKE),
        "compile_commands_sha256": _sha256(compile_commands),
        "executable_sha256": _sha256(executable),
        "executable_format": "ELF",
        "build_elapsed_ns": elapsed,
    }


def _one_protocol_line(stdout: str, label: str) -> tuple[dict, list[str]]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    protocol = [line for line in lines if line.startswith(PROTOCOL_PREFIX)]
    if len(protocol) != 1:
        raise RuntimeError("%s must emit exactly one timer protocol record" % label)
    try:
        document = json.loads(protocol[0][len(PROTOCOL_PREFIX):])
    except json.JSONDecodeError as exc:
        raise RuntimeError("%s emitted invalid timer JSON" % label) from exc
    diagnostics = [line for line in lines if not line.startswith(PROTOCOL_PREFIX)]
    return document, diagnostics


def _prewarm(cache_root: Path, env: dict, timeout: float) -> dict:
    if any(cache_root.iterdir()):
        raise RuntimeError("timer cache root must start empty")
    command = [sys.executable, "-u", str(WORKER), "--prewarm"]
    cold, cold_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="cold timer prewarm"),
        "cold timer prewarm",
    )
    warm, warm_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="warm timer prewarm"),
        "warm timer prewarm",
    )
    validate_prewarm(cold, expect_hit=False)
    validate_prewarm(warm, expect_hit=True)
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


class GraphObserver:
    def __init__(self, name: str):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        self.context = Context()
        self.context.init(args=[])
        self.node = Node(name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def wait_for(self, name: str, *, present: bool, timeout: float) -> dict:
        started = time.monotonic_ns()
        deadline = time.monotonic() + timeout
        observations = 0
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            names = set(self.node.get_node_names())
            observations += 1
            if (name in names) is present:
                return {
                    "observed": True,
                    "observations": observations,
                    "elapsed_ns": time.monotonic_ns() - started,
                }
        raise RuntimeError(
            "timer graph did not show node %r as %s" % (
                name, "present" if present else "absent"))

    def close(self) -> None:
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.context.shutdown()


def _read_document(process: subprocess.Popen, timeout: float, label: str) -> tuple[dict, list[str]]:
    deadline = time.monotonic() + timeout
    diagnostics = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("%s timed out after %.1fs" % (label, timeout))
        readable, _, _ = select.select([process.stdout], [], [], remaining)
        if not readable:
            raise RuntimeError("%s timed out after %.1fs" % (label, timeout))
        line = process.stdout.readline()
        if not line:
            stderr = process.stderr.read()
            raise RuntimeError(
                "%s exited with code %s before its record: %s" % (
                    label, process.poll(), stderr.strip()[-2000:]))
        line = line.rstrip("\n")
        if not line.startswith(PROTOCOL_PREFIX):
            if line:
                diagnostics.append(line)
            continue
        try:
            value = json.loads(line[len(PROTOCOL_PREFIX):])
        except json.JSONDecodeError as exc:
            raise RuntimeError("%s emitted invalid timer JSON" % label) from exc
        if not isinstance(value, dict):
            raise RuntimeError("%s timer record must be an object" % label)
        return value, diagnostics


def _finish(process: subprocess.Popen, timeout: float, label: str) -> dict:
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError("%s did not terminate" % label) from exc
    lines = [line for line in stdout.splitlines() if line.strip()]
    if any(line.startswith(PROTOCOL_PREFIX) for line in lines):
        raise RuntimeError("%s emitted an unexpected trailing timer record" % label)
    if process.returncode != 0:
        raise RuntimeError(
            "%s exited with code %d: %s" % (
                label, process.returncode, stderr.strip()[-2000:]))
    return {"stdout": lines, "stderr": stderr.strip() or None}


def _write_start(process: subprocess.Popen, label: str) -> None:
    try:
        process.stdin.write("START\n")
        process.stdin.flush()
    except (BrokenPipeError, OSError) as exc:
        raise RuntimeError("%s control pipe failed" % label) from exc


def _worker_argv(variant: str, executable: Path, node_name: str, token: str) -> list[str]:
    if variant == "aot-staged":
        return [
            str(executable), node_name, token,
            str(WARMUP_FIRINGS), str(MEASURED_FIRINGS),
        ]
    return [
        sys.executable, "-u", str(WORKER),
        "--variant", variant,
        "--node-name", node_name,
        "--run-token", token,
        "--warmup-firings", str(WARMUP_FIRINGS),
        "--measured-firings", str(MEASURED_FIRINGS),
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
    *,
    variant: str,
    repetition: int,
    executable: Path,
    build: dict,
    cache: dict,
    observer: GraphObserver,
    domain_id: int,
    env: dict,
    timeout: float,
) -> dict:
    token = "timer_" + uuid.uuid4().hex
    node_name = "timer_executor_%s" % token[6:18]
    process = None
    try:
        process = _spawn(_worker_argv(variant, executable, node_name, token), env)
        ready, ready_diagnostics = _read_document(
            process, timeout, "%s timer readiness" % variant)
        if ready.get("event") != "ready" or ready.get("schema") != EVENT_SCHEMA:
            raise RuntimeError("timer worker emitted invalid READY record")
        if ready.get("pid") != process.pid or ready.get("process_group_id") != process.pid:
            raise RuntimeError("timer worker did not run in a fresh process group")
        worker = psutil.Process(process.pid)
        if worker.children(recursive=True):
            raise RuntimeError("timer worker retained setup children before measurement")
        present = observer.wait_for(node_name, present=True, timeout=timeout)
        _write_start(process, "%s timer" % variant)
        armed, armed_diagnostics = _read_document(
            process, timeout, "%s timer armed" % variant)
        if armed.get("event") != "armed" or armed.get("cpu_clock") != (
                "CLOCK_PROCESS_CPUTIME_ID"):
            raise RuntimeError("timer worker emitted invalid ARMED record")
        report, report_diagnostics = _read_document(
            process, timeout, "%s timer report" % variant)
        if report.get("event") != "report":
            raise RuntimeError("timer worker emitted invalid report")
        finish = _finish(process, timeout, "%s timer" % variant)
        absent = observer.wait_for(node_name, present=False, timeout=timeout)
        timing = {
            "worker_cpu_ns_per_firing": report["cpu_time_ns"] / MEASURED_FIRINGS,
            "effective_frequency_hz": (
                MEASURED_FIRINGS * 1e9 / report["wall_duration_ns"]),
            "scheduled_deadline_error": report["scheduled_deadline_error"],
            "first_rearm_error_ns": report["first_rearm_error_ns"],
            "consecutive_interval_error": report["consecutive_interval_error"],
            "missed_periods": report["missed_periods"],
            "max_phase_slip_periods": report["max_phase_slip_periods"],
        }
        sample = {
            "schema": SAMPLE_SCHEMA,
            "case_id": "%s__rep_%d" % (variant, repetition),
            "variant": variant,
            "repetition": repetition,
            "run_token": token,
            "node_name": node_name,
            "worker_pid": process.pid,
            "worker_process_group_id": process.pid,
            "ros_domain_id": domain_id,
            "requested_rmw": RMW,
            "ros_distro": ROS_DISTRO,
            "graph": {
                "present_after_ready": True,
                "absent_after_exit": True,
                "presence_observation": present,
                "absence_observation": absent,
            },
            "worker_ready": ready,
            "worker_armed": armed,
            "worker_report": report,
            "timing": timing,
            "correctness_verified": True,
            "teardown_verified": True,
            "diagnostics": {
                "stdout": (
                    ready_diagnostics + armed_diagnostics +
                    report_diagnostics + finish["stdout"]),
                "stderr": finish["stderr"],
            },
        }
        validate_sample(sample, cache, build)
        return sample
    finally:
        _stop_process(process)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    if os.environ.get("ROS_DISTRO") != ROS_DISTRO:
        raise SystemExit("timer benchmark requires ROS_DISTRO=jazzy")
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = RMW
    env["CPPYY_KIT_NO_AUTOPCH"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    env.pop("CPPYY_KIT_NO_CACHE", None)
    os.environ["RMW_IMPLEMENTATION"] = RMW
    results = []
    failures = []
    execution_order = []
    with tempfile.TemporaryDirectory(prefix="rclcppyy-timer-executor-") as temporary:
        root = Path(temporary)
        build_directory = root / "aot-build"
        cache_root = root / "cache"
        build_directory.mkdir(mode=0o700)
        cache_root.mkdir(mode=0o700)
        env["XDG_CACHE_HOME"] = str(cache_root)
        executable, build = _compile_aot(build_directory, env, args.timeout)
        with acquire_domain() as lease:
            env["ROS_DOMAIN_ID"] = str(lease.domain_id)
            os.environ["ROS_DOMAIN_ID"] = str(lease.domain_id)
            cache = _prewarm(cache_root, env, args.timeout)
            observer = GraphObserver("timer_executor_observer_%s" % uuid.uuid4().hex[:12])
            try:
                variants = list(VARIANTS)
                for repetition in range(1, REPETITIONS + 1):
                    for variant in rotating_order(variants, repetition):
                        case_id = "%s__rep_%d" % (variant, repetition)
                        execution_order.append(case_id)
                        try:
                            results.append(_run_sample(
                                variant=variant,
                                repetition=repetition,
                                executable=executable,
                                build=build,
                                cache=cache,
                                observer=observer,
                                domain_id=lease.domain_id,
                                env=env,
                                timeout=args.timeout,
                            ))
                        except (OSError, psutil.Error, RuntimeError, ValueError) as exc:
                            failures.append({
                                "case_id": case_id,
                                "variant": variant,
                                "repetition": repetition,
                                "error": str(exc),
                            })
            finally:
                observer.close()
            parameters = {
                "variants": list(VARIANTS),
                "period_ns": PERIOD_NS,
                "warmup_firings": WARMUP_FIRINGS,
                "measured_firings": MEASURED_FIRINGS,
                "repetitions": REPETITIONS,
                "requested_rmw": RMW,
                "ros_distro": ROS_DISTRO,
                "execution_order": execution_order,
            }
            isolation = {
                "fresh_worker_process_per_sample": True,
                "fresh_process_group_per_sample": True,
                "unique_node_per_sample": True,
                "one_leased_domain_per_run": True,
                "rotating_variant_order": True,
                "ros_domain_id": lease.domain_id,
            }
            source_files = {
                "runner": _sha256(Path(__file__)),
                "protocol": _sha256(PROTOCOL),
                "worker": _sha256(WORKER),
                "aot_source": _sha256(AOT_SOURCE),
                "aot_cmake": _sha256(AOT_CMAKE),
            }
            document = build_document(
                repo_root=REPO_ROOT,
                mode="measurement",
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
                print(
                    "Timer benchmark emitted %d raw samples; claims and interpretation disabled."
                    % len(results))
                if failures:
                    print("%d sample(s) failed" % len(failures), file=sys.stderr)
            return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
