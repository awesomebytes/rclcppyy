#!/usr/bin/env python3
"""Run the controlled Jazzy/Cyclone service client benchmark."""

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
from _service_client_protocol import (
    CLIENT_SCHEMA,
    DEFAULT_SERVICE_TYPE,
    QOS,
    SAMPLE_SCHEMA,
    SERVICE_TYPES,
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
WORKER = HERE / "service_client_worker.py"
AOT_DIRECTORY = HERE / "service_client_aot"
AOT_SOURCE = AOT_DIRECTORY / "service_client_aot.cpp"
AOT_CMAKE = AOT_DIRECTORY / "CMakeLists.txt"
PROTOCOL = HERE / "_service_client_protocol.py"
DEFAULT_VARIANTS = tuple(VARIANTS)
MAX_MESSAGES = 100_000
MAX_REPETITIONS = 30
PROTOCOL_PREFIX = "@@RCLCPPYY_SERVICE_CLIENT_V1@@"
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


def _compile_aot(
        build_directory: Path, env: dict, timeout: float,
        service_type: str = DEFAULT_SERVICE_TYPE) -> tuple[Path, dict]:
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required for the service client AOT reference")
    configure = [
        cmake,
        "-S", str(AOT_DIRECTORY),
        "-B", str(build_directory),
        "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DRCLCPPYY_SERVICE_INTERFACE=" + service_type,
    ]
    if env.get("CONDA_PREFIX"):
        configure.append("-DCMAKE_PREFIX_PATH=" + env["CONDA_PREFIX"])
    started = time.perf_counter_ns()
    _run_command(configure, env=env, timeout=timeout, label="service client AOT configure")
    _run_command(
        [cmake, "--build", str(build_directory), "--target", "service_client_aot"],
        env=env,
        timeout=timeout,
        label="service client AOT build",
    )
    build_elapsed_ns = time.perf_counter_ns() - started
    executable = build_directory / "service_client_aot"
    compile_commands = build_directory / "compile_commands.json"
    cmake_cache = build_directory / "CMakeCache.txt"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError("service client AOT build produced no executable")
    if executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("service client AOT executable is not ELF")
    commands = json.loads(compile_commands.read_text(encoding="utf-8"))
    matches = [
        row for row in commands
        if Path(row["file"]).resolve() == AOT_SOURCE.resolve()
    ]
    if len(matches) != 1:
        raise RuntimeError("service client AOT compile command is missing or ambiguous")
    compile_command = matches[0].get("command") or " ".join(matches[0]["arguments"])
    if "-O3" not in compile_command or "-DNDEBUG" not in compile_command:
        raise RuntimeError("service client AOT build does not prove Release optimization")
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


def _prewarm(
        cache_root: Path, env: dict, timeout: float,
        service_type: str = DEFAULT_SERVICE_TYPE) -> dict:
    if cache_root.exists() and any(cache_root.iterdir()):
        raise RuntimeError("service client cache root must start empty")
    command = [
        sys.executable, "-u", str(WORKER), "--prewarm",
        "--service-interface", service_type,
    ]
    cold, cold_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="cold client prewarm"),
        "cold client prewarm",
    )
    warm, warm_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="warm client prewarm"),
        "warm client prewarm",
    )
    cold["stdout_diagnostics"] = cold_diagnostics
    warm["stdout_diagnostics"] = warm_diagnostics
    validate_prewarm(cold, expect_hits=False, service_type=service_type)
    validate_prewarm(warm, expect_hits=True, service_type=service_type)
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
        if request_stack_dump else None)
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
        armed: dict, *, token: str, server_pid: int, warmup: int,
        service_type: str = DEFAULT_SERVICE_TYPE) -> None:
    expected = {
        "schema": "rclcppyy.service-client-server-event/v1",
        "event": "armed",
        "run_token": token,
        "pid": server_pid,
        "process_group_id": server_pid,
        "warmup_requests": warmup,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "service_type": service_type,
    }
    actual = dict(armed)
    actual.setdefault("service_type", DEFAULT_SERVICE_TYPE)
    if actual != expected:
        raise RuntimeError("common server emitted invalid armed evidence")


def _server_argv(
        executable: Path, service_name: str, server_node: str,
        warmup: int, messages: int, token: str,
        service_type: str = DEFAULT_SERVICE_TYPE) -> list[str]:
    return [
        str(executable), "server", service_name, server_node,
        str(warmup), str(messages), token, service_type,
    ]


def _client_argv(
        variant: str, executable: Path, service_name: str, client_node: str,
        server_node: str, warmup: int, messages: int, token: str,
        service_type: str = DEFAULT_SERVICE_TYPE) -> list[str]:
    if variant == "aot-staged":
        return [
            str(executable), "client", service_name, client_node, server_node,
            str(warmup), str(messages), token, service_type,
        ]
    return [
        sys.executable, "-u", str(WORKER),
        "--variant", variant,
        "--service-interface", service_type,
        "--service-name", service_name,
        "--node-name", client_node,
        "--server-node", server_node,
        "--warmup-requests", str(warmup),
        "--messages", str(messages),
        "--run-token", token,
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
        requested_rmw: str, env: dict, timeout: float,
        service_type: str = DEFAULT_SERVICE_TYPE) -> dict:
    token = "run_" + uuid.uuid4().hex
    suffix = token[4:16]
    service_name = "/rclcppyy_service_client/" + token
    server_node = "service_client_server_" + suffix
    client_node = "service_client_under_test_" + suffix
    server = None
    client = None
    try:
        server = _spawn(_server_argv(
            executable, service_name, server_node, warmup, messages, token,
            service_type), env)
        server_ready, server_ready_diagnostics = _read_document(
            server, timeout, "common AOT server readiness")
        if server_ready.get("pid") != server.pid or server_ready.get(
                "process_group_id") != server.pid:
            raise RuntimeError("common AOT server did not use its fresh process group")

        client = _spawn(_client_argv(
            variant, executable, service_name, client_node, server_node,
            warmup, messages, token, service_type), env)
        warmed, client_warm_diagnostics = _read_document(
            client, timeout, "%s client warmup" % variant)
        if warmed.get("schema") != CLIENT_SCHEMA or warmed.get("event") != "warmed":
            raise RuntimeError("client emitted invalid warmup evidence")
        if warmed.get("pid") != client.pid or warmed.get(
                "process_group_id") != client.pid:
            raise RuntimeError("client did not use its fresh process group")
        if warmed.get("run_token") != token or warmed.get(
                "warmup_requests") != warmup or warmed.get(
                    "loaded_rmw") != requested_rmw:
            raise RuntimeError("client warmup identity is invalid")

        server_process = psutil.Process(server.pid)
        client_process = psutil.Process(client.pid)
        if server_process.children(recursive=True) or client_process.children(recursive=True):
            raise RuntimeError("benchmark processes retained setup children before measurement")
        _write_control(server, "START", "common AOT server")
        armed, server_armed_diagnostics = _read_document(
            server, timeout, "common AOT server armed")
        _validate_armed_before_measurement(
            armed, token=token, server_pid=server.pid, warmup=warmup,
            service_type=service_type)
        _write_control(client, "START", "%s client" % variant)
        client_report, client_report_diagnostics = _read_document(
            client, timeout, "%s client measured result" % variant,
            request_stack_dump=variant != "aot-staged")
        _write_control(server, "REPORT", "common AOT server")
        server_report, server_report_diagnostics = _read_document(
            server, timeout, "common AOT server report")
        server_finish = _finish(server, timeout, "common AOT server")
        _write_control(client, "TEARDOWN", "%s client" % variant)
        client_teardown, client_teardown_diagnostics = _read_document(
            client, timeout, "%s client teardown" % variant,
            request_stack_dump=variant != "aot-staged")
        client_finish = _finish(client, timeout, "%s client" % variant)

        client_cpu_ns = client_report.get("cpu_time_ns")
        server_cpu_ns = server_report.get("cpu_time_ns")
        elapsed_ns = client_report.get("elapsed_ns")
        timing = {
            "client_cpu_time_ns": client_cpu_ns,
            "client_cpu_ns_per_response": client_cpu_ns / messages,
            "server_cpu_time_ns": server_cpu_ns,
            "server_cpu_ns_per_request_drift": server_cpu_ns / messages,
            "elapsed_ns": elapsed_ns,
            "requests_per_second": messages * 1e9 / elapsed_ns,
            "rtt_ns": latency_summary(client_report.get("latency_ns", [])),
        }
        sample = {
            "schema": SAMPLE_SCHEMA,
            "case_id": "%s__rep_%d" % (variant, repetition),
            "variant": variant,
            "repetition": repetition,
            "run_token": token,
            "ros_domain_id": domain_id,
            "requested_rmw": requested_rmw,
            "service_type": service_type,
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
            "server_ready": server_ready,
            "client_warmed": warmed,
            "server_armed": armed,
            "client_report": client_report,
            "server_report": server_report,
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
                    client_warm_diagnostics + client_report_diagnostics +
                    client_teardown_diagnostics + client_finish["stdout"]),
                "server_stderr": server_finish["stderr"],
                "client_stderr": client_finish["stderr"],
            },
        }
        validate_sample(sample, {
            "requested_rmw": requested_rmw,
            "service_type": service_type,
            "qos": QOS,
            "warmup_requests": warmup,
            "messages": messages,
        }, build, cache)
        return sample
    except Exception as exc:
        diagnostics = None
        if client is not None and variant != "aot-staged" and client.stderr is not None and (
                not client.stderr.closed):
            diagnostics = _drain_stderr_after_stack_request(client)
        if diagnostics:
            raise RuntimeError(
                "%s; client diagnostics: %s" % (
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
        raise ValueError("at least one service client variant is required")
    return selected


def _print_table(results: list[dict]) -> None:
    print("\nCharacterization only: raw metrics are not performance claims.")
    print("  %-30s %3s %12s %12s %12s %12s" % (
        "variant", "rep", "client cpu", "p50 ns", "p99 ns", "req/s"))
    print("  " + "-" * 88)
    for row in results:
        timing = row["timing"]
        print("  %-30s %3d %12.1f %12d %12d %12.1f" % (
            row["variant"], row["repetition"],
            timing["client_cpu_ns_per_response"], timing["rtt_ns"]["p50"],
            timing["rtt_ns"]["p99"], timing["requests_per_second"],
        ))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--variants", help="comma-separated client variants")
    parser.add_argument(
        "--service-interface", choices=SERVICE_TYPES,
        default=DEFAULT_SERVICE_TYPE)
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
    with tempfile.TemporaryDirectory(prefix="rclcppyy-service-client-") as temporary:
        root = Path(temporary)
        build_directory = root / "aot-build"
        cache_root = root / "cache"
        build_directory.mkdir(mode=0o700)
        cache_root.mkdir(mode=0o700)
        environment["XDG_CACHE_HOME"] = str(cache_root)
        executable, aot_build = _compile_aot(
            build_directory, environment, args.timeout, args.service_interface)
        cache = _prewarm(
            cache_root, environment, args.timeout, args.service_interface)
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
                            service_type=args.service_interface,
                        ))
                    except (OSError, psutil.Error, RuntimeError, ValueError) as exc:
                        failure = {
                            "case_id": "%s__rep_%d" % (variant, repetition),
                            "variant": variant,
                            "repetition": repetition,
                            "service_type": args.service_interface,
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
                "service_type": args.service_interface,
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
