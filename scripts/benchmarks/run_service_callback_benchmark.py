#!/usr/bin/env python3
"""Run the controlled Jazzy/Cyclone SetBool callback benchmark."""

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
from _service_callback_protocol import (
    CLIENT_SCHEMA,
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
WORKER = HERE / "service_callback_worker.py"
AOT_DIRECTORY = HERE / "service_callback_aot"
AOT_SOURCE = AOT_DIRECTORY / "service_callback_aot.cpp"
AOT_CMAKE = AOT_DIRECTORY / "CMakeLists.txt"
PROTOCOL = HERE / "_service_callback_protocol.py"
DEFAULT_VARIANTS = tuple(VARIANTS)
MAX_MESSAGES = 100_000
MAX_REPETITIONS = 30
PROTOCOL_PREFIX = "@@RCLCPPYY_SERVICE_CALLBACK_V1@@"
REQUIRED_RMW = "rmw_cyclonedds_cpp"


class ProtocolTimeout(RuntimeError):
    def __init__(self, message: str, *, stderr: str | None = None):
        super().__init__(message)
        self.stderr = stderr


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
        raise RuntimeError("cmake is required for the service AOT reference")
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
    _run_command(configure, env=env, timeout=timeout, label="service AOT configure")
    _run_command(
        [cmake, "--build", str(build_directory), "--target", "service_callback_aot"],
        env=env,
        timeout=timeout,
        label="service AOT build",
    )
    build_elapsed_ns = time.perf_counter_ns() - started
    executable = build_directory / "service_callback_aot"
    compile_commands = build_directory / "compile_commands.json"
    cmake_cache = build_directory / "CMakeCache.txt"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError("service AOT build produced no executable")
    if executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("service AOT executable is not ELF")
    commands = json.loads(compile_commands.read_text(encoding="utf-8"))
    matches = [
        row for row in commands
        if Path(row["file"]).resolve() == AOT_SOURCE.resolve()
    ]
    if len(matches) != 1:
        raise RuntimeError("service AOT compile command is missing or ambiguous")
    compile_command = matches[0].get("command") or " ".join(matches[0]["arguments"])
    if "-O3" not in compile_command or "-DNDEBUG" not in compile_command:
        raise RuntimeError("service AOT compile command does not prove Release optimization")
    compiler = _compiler_from_cache(cmake_cache)
    return executable, {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": compiler,
        "compiler_version": _compiler_version(compiler, env),
        "compile_command": compile_command,
        "source_sha256": _sha256(AOT_SOURCE),
        "cmake_sha256": _sha256(AOT_CMAKE),
        "compile_commands_sha256": _sha256(compile_commands),
        "executable_sha256": _sha256(executable),
        "executable_format": "ELF",
        "build_elapsed_ns": build_elapsed_ns,
    }


def _one_protocol_line(stdout: str, label: str) -> tuple[dict, list[str]]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    records = [line for line in lines if line.startswith(PROTOCOL_PREFIX)]
    if len(records) != 1:
        raise RuntimeError("%s must emit exactly one sentinel protocol record" % label)
    try:
        document = json.loads(records[0][len(PROTOCOL_PREFIX):])
    except json.JSONDecodeError as exc:
        raise RuntimeError("%s emitted invalid sentinel JSON" % label) from exc
    if not isinstance(document, dict):
        raise RuntimeError("%s JSON must be an object" % label)
    return document, [line for line in lines if not line.startswith(PROTOCOL_PREFIX)]


def _prewarm(cache_root: Path, env: dict, timeout: float) -> dict:
    if cache_root.exists() and any(cache_root.iterdir()):
        raise RuntimeError("service cache root must start empty")
    command = [sys.executable, "-u", str(WORKER), "--prewarm"]
    cold, cold_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="cold service prewarm"),
        "cold service prewarm",
    )
    warm, warm_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="warm service prewarm"),
        "warm service prewarm",
    )
    cold["stdout_diagnostics"] = cold_diagnostics
    warm["stdout_diagnostics"] = warm_diagnostics
    validate_prewarm(cold, expect_hits=False)
    validate_prewarm(warm, expect_hits=True)
    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": cold, "warm": warm},
    }


def _drain_stderr_after_stack_request(process: subprocess.Popen) -> str | None:
    if process.poll() is None:
        try:
            os.kill(process.pid, signal.SIGUSR1)
        except ProcessLookupError:
            pass
    time.sleep(0.5)
    descriptor = process.stderr.fileno()
    chunks = []
    os.set_blocking(descriptor, False)
    try:
        while True:
            try:
                chunk = os.read(descriptor, 65536)
            except BlockingIOError:
                break
            if not chunk:
                break
            chunks.append(chunk)
    finally:
        os.set_blocking(descriptor, True)
    value = b"".join(chunks).decode("utf-8", errors="replace").strip()
    return value or None


def _protocol_timeout(
        process: subprocess.Popen, timeout: float, label: str,
        *, request_stack_dump: bool) -> ProtocolTimeout:
    stderr = (
        _drain_stderr_after_stack_request(process)
        if request_stack_dump else None
    )
    return ProtocolTimeout(
        "%s timed out after %.1fs" % (label, timeout), stderr=stderr)


def _read_document(
        process: subprocess.Popen, timeout: float, label: str,
        *, request_stack_dump: bool = False) -> tuple[dict, list[str]]:
    deadline = time.monotonic() + timeout
    diagnostics = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise _protocol_timeout(
                process, timeout, label, request_stack_dump=request_stack_dump)
        ready, _, _ = select.select([process.stdout], [], [], remaining)
        if not ready:
            raise _protocol_timeout(
                process, timeout, label, request_stack_dump=request_stack_dump)
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
    return {"stdout": stdout_lines, "stderr": stderr.strip() or None}


def _write_control(process: subprocess.Popen, command: str, label: str) -> None:
    try:
        process.stdin.write(command + "\n")
        process.stdin.flush()
    except (BrokenPipeError, OSError) as exc:
        raise RuntimeError("%s control pipe failed" % label) from exc


def _validate_armed_before_measurement(
        armed: dict, *, variant: str, token: str, server_pid: int,
        warmup: int) -> None:
    expected = {
        "schema": "rclcppyy.service-callback-server-event/v1",
        "event": "armed",
        "variant": variant,
        "run_token": token,
        "pid": server_pid,
        "process_group_id": server_pid,
        "warmup_requests": warmup,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
    }
    if armed != expected:
        raise RuntimeError("service server emitted invalid armed evidence")


def _server_argv(
        variant: str, executable: Path, service_name: str, server_node: str,
        warmup: int, messages: int, token: str) -> list[str]:
    if variant == "aot-staged":
        return [
            str(executable), "server", service_name, server_node,
            str(warmup), str(messages), token,
        ]
    return [
        sys.executable, "-u", str(WORKER),
        "--variant", variant,
        "--service-name", service_name,
        "--node-name", server_node,
        "--warmup-requests", str(warmup),
        "--messages", str(messages),
        "--run-token", token,
    ]


def _client_argv(
        executable: Path, service_name: str, client_node: str,
        server_node: str, warmup: int, messages: int, token: str) -> list[str]:
    return [
        str(executable), "client", service_name, client_node, server_node,
        str(warmup), str(messages), token,
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
    service_name = "/rclcppyy_service_callback/" + token
    server_node = "service_callback_server_" + suffix
    client_node = "service_callback_client_" + suffix
    server = None
    client = None
    try:
        server = _spawn(_server_argv(
            variant, executable, service_name, server_node,
            warmup, messages, token), env)
        ready, server_ready_diagnostics = _read_document(
            server, timeout, "%s server readiness" % variant)
        if ready.get("pid") != server.pid or ready.get("process_group_id") != server.pid:
            raise RuntimeError("service server did not run in its fresh process group")

        client = _spawn(_client_argv(
            executable, service_name, client_node, server_node,
            warmup, messages, token), env)
        warmed, client_warm_diagnostics = _read_document(
            client, timeout, "AOT service client warmup")
        if warmed.get("schema") != CLIENT_SCHEMA or warmed.get("event") != "warmed":
            raise RuntimeError("AOT service client emitted invalid warmup evidence")
        expected_warm = {
            "run_token": token,
            "pid": client.pid,
            "process_group_id": client.pid,
            "warmup_requests": warmup,
            "loaded_rmw": requested_rmw,
        }
        if any(warmed.get(name) != value for name, value in expected_warm.items()):
            raise RuntimeError("AOT service client warmup identity is invalid")

        server_process = psutil.Process(server.pid)
        client_process = psutil.Process(client.pid)
        if server_process.children(recursive=True) or client_process.children(recursive=True):
            raise RuntimeError("benchmark processes retained setup children before measurement")
        _write_control(server, "START", "%s service server" % variant)
        armed, server_armed_diagnostics = _read_document(
            server, timeout, "%s service server armed" % variant)
        _validate_armed_before_measurement(
            armed, variant=variant, token=token, server_pid=server.pid,
            warmup=warmup)
        _write_control(client, "START", "AOT service client")
        client_result, client_result_diagnostics = _read_document(
            client, timeout, "AOT service client measured result")
        _write_control(server, "REPORT", "%s service server" % variant)
        report, server_report_diagnostics = _read_document(
            server, timeout, "%s service server report" % variant,
            request_stack_dump=variant != "aot-staged")
        server_finish = _finish(server, timeout, "%s service server" % variant)
        _write_control(client, "TEARDOWN", "AOT service client")
        client_teardown, client_teardown_diagnostics = _read_document(
            client, timeout, "AOT service client teardown")
        client_finish = _finish(client, timeout, "AOT service client")

        server_cpu_ns = report.get("cpu_time_ns")
        client_cpu_ns = client_result.get("cpu_time_ns")
        elapsed_ns = client_result.get("elapsed_ns")
        timing = {
            "server_cpu_time_ns": server_cpu_ns,
            "server_cpu_ns_per_request": server_cpu_ns / messages,
            "client_cpu_time_ns": client_cpu_ns,
            "client_cpu_ns_per_request": client_cpu_ns / messages,
            "elapsed_ns": elapsed_ns,
            "requests_per_second": messages * 1e9 / elapsed_ns,
            "rtt_ns": latency_summary(client_result.get("latency_ns", [])),
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
            "server_pid": server.pid,
            "client_pid": client.pid,
            "topology": {
                "process_count": 2,
                "fresh_process_groups": True,
                "server_node": server_node,
                "client_node": client_node,
                "service_name": service_name,
            },
            "server_ready": ready,
            "server_armed": armed,
            "server_report": report,
            "client_result": client_result,
            "client_teardown": client_teardown,
            "timing": timing,
            "backend_verified": True,
            "correctness_verified": True,
            "teardown_verified": True,
            "diagnostics": {
                "server_stdout": (
                    server_ready_diagnostics + server_armed_diagnostics +
                    server_report_diagnostics + server_finish["stdout"]),
                "client_stdout": (
                    client_warm_diagnostics + client_result_diagnostics +
                    client_teardown_diagnostics + client_finish["stdout"]),
                "server_stderr": server_finish["stderr"],
                "client_stderr": client_finish["stderr"],
            },
        }
        validate_sample(sample, {
            "requested_rmw": requested_rmw,
            "qos": QOS,
            "warmup_requests": warmup,
            "messages": messages,
        }, build, cache)
        return sample
    except Exception as exc:
        diagnostics = None
        if server is not None and variant != "aot-staged":
            diagnostics = _drain_stderr_after_stack_request(server)
        if diagnostics:
            raise RuntimeError(
                "%s; service server diagnostics: %s" % (
                    exc, diagnostics[-16000:])) from exc
        raise
    finally:
        _stop_process(client)
        _stop_process(server)


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
                "unknown variant %r; choose from: %s" % (
                    variant, ", ".join(VARIANTS)))
        if variant not in selected:
            selected.append(variant)
    if not selected:
        raise ValueError("at least one service callback variant is required")
    return selected


def _print_table(results: list[dict]) -> None:
    print("\nCharacterization only: raw metrics are not performance claims.")
    print("  %-28s %3s %12s %12s %12s %12s" % (
        "variant", "rep", "server cpu", "p50 ns", "p99 ns", "req/s"))
    print("  " + "-" * 86)
    for row in results:
        timing = row["timing"]
        print("  %-28s %3d %12.1f %12d %12d %12.1f" % (
            row["variant"], row["repetition"],
            timing["server_cpu_ns_per_request"], timing["rtt_ns"]["p50"],
            timing["rtt_ns"]["p99"], timing["requests_per_second"],
        ))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--variants", help="comma-separated service variants")
    parser.add_argument("--messages", type=int)
    parser.add_argument("--warmup-requests", type=int)
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
    warmup = args.warmup_requests if args.warmup_requests is not None else (
        2 if args.smoke else 100)
    repetitions = args.repetitions if args.repetitions is not None else (
        1 if args.smoke else 5)
    if not 1 <= messages <= MAX_MESSAGES:
        parser.error("--messages must be between 1 and %d" % MAX_MESSAGES)
    if not 1 <= warmup <= MAX_MESSAGES:
        parser.error("--warmup-requests must be between 1 and %d" % MAX_MESSAGES)
    if not 1 <= repetitions <= MAX_REPETITIONS:
        parser.error("--repetitions must be between 1 and %d" % MAX_REPETITIONS)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    requested_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if requested_rmw != REQUIRED_RMW:
        parser.error("RMW_IMPLEMENTATION must be %s" % REQUIRED_RMW)

    mode = "smoke" if args.smoke else "measurement"
    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"
    environment["CPPYY_KIT_NO_AUTOPCH"] = "1"
    environment.pop("CPPYY_KIT_NO_CACHE", None)
    results = []
    failures = []
    execution_order = []
    with tempfile.TemporaryDirectory(prefix="rclcppyy-service-callback-") as temporary:
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
                        failure = {
                            "case_id": "%s__rep_%d" % (variant, repetition),
                            "variant": variant,
                            "repetition": repetition,
                            "error": str(exc),
                        }
                        if isinstance(exc, ProtocolTimeout) and exc.stderr:
                            failure["diagnostics"] = {"stderr": exc.stderr}
                        failures.append(failure)
            parameters = {
                "variants": variants,
                "messages": messages,
                "warmup_requests": warmup,
                "repetitions": repetitions,
                "requested_rmw": requested_rmw,
                "qos": dict(QOS),
                "execution_order": execution_order,
            }
            isolation = {
                "fresh_process_pair_per_sample": True,
                "two_process_groups_per_sample": True,
                "unique_service_per_sample": True,
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
