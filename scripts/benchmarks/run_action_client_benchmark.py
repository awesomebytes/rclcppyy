#!/usr/bin/env python3
"""Run the CPU-first Jazzy/Cyclone action-client benchmark."""

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

from _action_client_protocol import (
    ACTION_TYPE,
    CLIENT_SCHEMA,
    FEEDBACK_PER_GOAL,
    MEASURED_GOALS,
    QOS,
    REPETITIONS,
    RMW,
    ROS_DISTRO,
    SAMPLE_SCHEMA,
    SERVER_SCHEMA,
    VARIANTS,
    WARMUP_GOALS,
    build_document,
    dumps,
    endpoint_names,
    rotating_order,
    validate_prewarm,
    validate_sample,
    write,
)
from _domain_lease import acquire_domain


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "action_client_worker.py"
PROTOCOL = HERE / "_action_client_protocol.py"
AOT_DIRECTORY = HERE / "action_client_aot"
SERVER_SOURCE = AOT_DIRECTORY / "action_benchmark_server.cpp"
CLIENT_SOURCE = AOT_DIRECTORY / "action_benchmark_client.cpp"
AOT_CMAKE = AOT_DIRECTORY / "CMakeLists.txt"
PROTOCOL_PREFIX = "@@RCLCPPYY_ACTION_CLIENT_V1@@"

ENDPOINT_TYPES = {
    "send_goal": "tf2_msgs/action/LookupTransform_SendGoal",
    "get_result": "tf2_msgs/action/LookupTransform_GetResult",
    "cancel_goal": "action_msgs/srv/CancelGoal",
    "feedback": "tf2_msgs/action/LookupTransform_FeedbackMessage",
    "status": "action_msgs/msg/GoalStatusArray",
}


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


def _run_command(command: list[str], *, env: dict, timeout: float, label: str) -> str:
    process = _spawn(command, env)
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError("%s timed out after %.1fs" % (label, timeout)) from exc
    if process.returncode != 0:
        raise RuntimeError(
            "%s failed with code %d: %s" % (
                label, process.returncode, (stderr or stdout).strip()[-4000:]))
    return stdout


def _compiler_from_cache(cache: Path) -> str:
    for line in cache.read_text(encoding="utf-8").splitlines():
        if line.startswith("CMAKE_CXX_COMPILER:FILEPATH="):
            return line.split("=", 1)[1]
    raise RuntimeError("action CMake cache did not identify the compiler")


def _compile_aot(build_directory: Path, env: dict, timeout: float) -> tuple[dict, dict]:
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required for the action AOT peers")
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
    _run_command(configure, env=env, timeout=timeout, label="action AOT configure")
    _run_command(
        [cmake, "--build", str(build_directory)],
        env=env,
        timeout=timeout,
        label="action AOT build",
    )
    elapsed = time.perf_counter_ns() - started
    paths = {
        "server": build_directory / "action_benchmark_server",
        "client": build_directory / "action_benchmark_client",
    }
    for name, path in paths.items():
        if not path.is_file() or not os.access(path, os.X_OK) or path.read_bytes()[:4] != b"\x7fELF":
            raise RuntimeError("action AOT %s executable is invalid" % name)
    compile_path = build_directory / "compile_commands.json"
    commands = json.loads(compile_path.read_text(encoding="utf-8"))
    compile_commands = {}
    for role, source in (("server", SERVER_SOURCE), ("client", CLIENT_SOURCE)):
        matches = [row for row in commands if Path(row["file"]).resolve() == source.resolve()]
        if len(matches) != 1:
            raise RuntimeError("action AOT %s compile command is ambiguous" % role)
        command = matches[0].get("command") or " ".join(matches[0]["arguments"])
        if "-O3" not in command or "-DNDEBUG" not in command:
            raise RuntimeError("action AOT %s lacks -O3/-DNDEBUG" % role)
        compile_commands[role] = command
    compiler = _compiler_from_cache(build_directory / "CMakeCache.txt")
    compiler_version = _run_command(
        [compiler, "--version"], env=env, timeout=10, label="action compiler probe"
    ).splitlines()[0]
    evidence = {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": compiler,
        "compiler_version": compiler_version,
        "compile_commands": compile_commands,
        "server_source_sha256": _sha256(SERVER_SOURCE),
        "client_source_sha256": _sha256(CLIENT_SOURCE),
        "cmake_sha256": _sha256(AOT_CMAKE),
        "executables": {
            role: {"format": "ELF", "sha256": _sha256(path)}
            for role, path in paths.items()
        },
        "build_elapsed_ns": elapsed,
    }
    return paths, evidence


def _one_protocol_line(stdout: str, label: str) -> tuple[dict, list[str]]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    protocol = [line for line in lines if line.startswith(PROTOCOL_PREFIX)]
    if len(protocol) != 1:
        raise RuntimeError("%s must emit exactly one action protocol record" % label)
    try:
        document = json.loads(protocol[0][len(PROTOCOL_PREFIX):])
    except json.JSONDecodeError as exc:
        raise RuntimeError("%s emitted invalid action JSON" % label) from exc
    diagnostics = [line for line in lines if not line.startswith(PROTOCOL_PREFIX)]
    return document, diagnostics


def _prewarm(cache_root: Path, env: dict, timeout: float) -> dict:
    if any(cache_root.iterdir()):
        raise RuntimeError("action cache root must start empty")
    command = [sys.executable, "-u", str(WORKER), "--prewarm"]
    cold, cold_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="cold action prewarm"),
        "cold action prewarm",
    )
    warm, warm_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="warm action prewarm"),
        "warm action prewarm",
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

    def _snapshot(self, action_name: str) -> dict:
        names = set(self.node.get_node_names())
        services = {
            name: sorted(types)
            for name, types in self.node.get_service_names_and_types()
            if name.startswith(action_name + "/")
        }
        topics = {
            name: sorted(types)
            for name, types in self.node.get_topic_names_and_types()
            if name.startswith(action_name + "/")
        }
        return {"nodes": names, "services": services, "topics": topics}

    @staticmethod
    def _exact_endpoints(snapshot: dict, action_name: str) -> bool:
        endpoints = endpoint_names(action_name)
        expected_services = {
            endpoints[name]: [ENDPOINT_TYPES[name]]
            for name in ("send_goal", "get_result", "cancel_goal")
        }
        expected_topics = {
            endpoints[name]: [ENDPOINT_TYPES[name]]
            for name in ("feedback", "status")
        }
        return snapshot["services"] == expected_services and snapshot["topics"] == expected_topics

    def wait_ready(
        self, server_node: str, client_node: str, action_name: str, timeout: float
    ) -> dict:
        started = time.monotonic_ns()
        deadline = time.monotonic() + timeout
        observations = 0
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            snapshot = self._snapshot(action_name)
            observations += 1
            if (
                server_node in snapshot["nodes"]
                and client_node in snapshot["nodes"]
                and self._exact_endpoints(snapshot, action_name)
            ):
                return {
                    "observed": True,
                    "observations": observations,
                    "elapsed_ns": time.monotonic_ns() - started,
                }
        raise RuntimeError("action graph did not expose the exact ready topology")

    def wait_client_exit(
        self, server_node: str, client_node: str, action_name: str, timeout: float
    ) -> dict:
        started = time.monotonic_ns()
        deadline = time.monotonic() + timeout
        observations = 0
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            snapshot = self._snapshot(action_name)
            observations += 1
            if (
                client_node not in snapshot["nodes"]
                and server_node in snapshot["nodes"]
                and self._exact_endpoints(snapshot, action_name)
            ):
                return {
                    "observed": True,
                    "observations": observations,
                    "elapsed_ns": time.monotonic_ns() - started,
                }
        raise RuntimeError("action client graph entity did not disappear cleanly")

    def wait_final_exit(
        self, server_node: str, client_node: str, action_name: str, timeout: float
    ) -> dict:
        started = time.monotonic_ns()
        deadline = time.monotonic() + timeout
        observations = 0
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            snapshot = self._snapshot(action_name)
            observations += 1
            if (
                server_node not in snapshot["nodes"]
                and client_node not in snapshot["nodes"]
                and not snapshot["services"]
                and not snapshot["topics"]
            ):
                return {
                    "observed": True,
                    "observations": observations,
                    "elapsed_ns": time.monotonic_ns() - started,
                }
        raise RuntimeError("action server or endpoints did not disappear cleanly")

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
                    label, process.poll(), stderr.strip()[-4000:]))
        line = line.rstrip("\n")
        if not line.startswith(PROTOCOL_PREFIX):
            if line:
                diagnostics.append(line)
            continue
        try:
            value = json.loads(line[len(PROTOCOL_PREFIX):])
        except json.JSONDecodeError as exc:
            raise RuntimeError("%s emitted invalid action JSON" % label) from exc
        if not isinstance(value, dict):
            raise RuntimeError("%s action record must be an object" % label)
        return value, diagnostics


def _finish(process: subprocess.Popen, timeout: float, label: str) -> dict:
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process)
        raise RuntimeError("%s did not terminate" % label) from exc
    lines = [line for line in stdout.splitlines() if line.strip()]
    if any(line.startswith(PROTOCOL_PREFIX) for line in lines):
        raise RuntimeError("%s emitted an unexpected trailing action record" % label)
    if process.returncode != 0:
        raise RuntimeError(
            "%s exited with code %d: %s" % (
                label, process.returncode, stderr.strip()[-4000:]))
    return {"stdout": lines, "stderr": stderr.strip() or None}


def _write_control(process: subprocess.Popen, command: str, label: str) -> None:
    try:
        process.stdin.write(command + "\n")
        process.stdin.flush()
    except (BrokenPipeError, OSError) as exc:
        raise RuntimeError("%s control pipe failed" % label) from exc


def _client_argv(
    variant: str, executables: dict, node_name: str, action_name: str, token: str
) -> list[str]:
    values = [
        node_name, action_name, token, variant,
        str(WARMUP_GOALS), str(MEASURED_GOALS),
    ]
    if variant == "aot-staged":
        return [str(executables["client"]), *values]
    return [
        sys.executable, "-u", str(WORKER),
        "--variant", variant,
        "--node-name", node_name,
        "--action-name", action_name,
        "--run-token", token,
        "--warmup-goals", str(WARMUP_GOALS),
        "--measured-goals", str(MEASURED_GOALS),
    ]


def _run_sample(
    *, variant: str, repetition: int, executables: dict, build: dict, cache: dict,
    observer: GraphObserver, domain_id: int, env: dict, timeout: float,
) -> dict:
    token = "action_" + uuid.uuid4().hex
    suffix = token[7:19]
    action_name = "/rclcppyy/action_benchmark/run_%s" % token[7:]
    server_node = "action_server_%s" % suffix
    client_node = "action_client_%s" % suffix
    server = None
    client = None
    diagnostics = {"server": [], "client": []}
    try:
        server = _spawn([
            str(executables["server"]), server_node, action_name, token, variant,
            str(WARMUP_GOALS), str(MEASURED_GOALS),
        ], env)
        server_ready, values = _read_document(server, timeout, "action server READY")
        diagnostics["server"].extend(values)
        if server_ready.get("schema") != SERVER_SCHEMA or server_ready.get("event") != "ready":
            raise RuntimeError("action server emitted invalid READY evidence")
        client = _spawn(
            _client_argv(variant, executables, client_node, action_name, token), env)
        client_ready, values = _read_document(client, timeout, "action client READY")
        diagnostics["client"].extend(values)
        if client_ready.get("schema") != CLIENT_SCHEMA or client_ready.get("event") != "ready":
            raise RuntimeError("action client emitted invalid READY evidence")
        for role, process, ready in (
            ("server", server, server_ready), ("client", client, client_ready)):
            if ready.get("pid") != process.pid or ready.get("process_group_id") != process.pid:
                raise RuntimeError("action %s did not use a fresh process group" % role)
            if psutil.Process(process.pid).children(recursive=True):
                raise RuntimeError("action %s retained setup children" % role)
        ready_graph = observer.wait_ready(server_node, client_node, action_name, timeout)
        _write_control(client, "START", "action client")
        client_armed, values = _read_document(client, timeout, "action client ARMED")
        diagnostics["client"].extend(values)
        if client_armed.get("event") != "armed" or client_armed.get("cpu_clock") != (
                "CLOCK_PROCESS_CPUTIME_ID"):
            raise RuntimeError("action client emitted invalid ARMED evidence")
        client_report, values = _read_document(client, timeout, "action client report")
        diagnostics["client"].extend(values)
        if client_report.get("event") != "report":
            raise RuntimeError("action client emitted invalid report")
        client_finish = _finish(client, timeout, "action client")
        diagnostics["client"].extend(client_finish["stdout"])
        client_exit_graph = observer.wait_client_exit(
            server_node, client_node, action_name, timeout)
        _write_control(server, "STOP", "action server")
        server_report, values = _read_document(server, timeout, "action server report")
        diagnostics["server"].extend(values)
        if server_report.get("event") != "report":
            raise RuntimeError("action server emitted invalid report")
        server_finish = _finish(server, timeout, "action server")
        diagnostics["server"].extend(server_finish["stdout"])
        final_graph = observer.wait_final_exit(server_node, client_node, action_name, timeout)
        timing = {
            "client_cpu_ns_per_completed_goal": (
                client_report["cpu_time_ns"] / MEASURED_GOALS),
            "completed_goals_per_second": (
                MEASURED_GOALS * 1e9 / client_report["wall_duration_ns"]),
            "latency_ns": client_report["latency_ns"],
        }
        sample = {
            "schema": SAMPLE_SCHEMA,
            "case_id": "%s__rep_%d" % (variant, repetition),
            "variant": variant,
            "repetition": repetition,
            "run_token": token,
            "action_name": action_name,
            "server_node": server_node,
            "client_node": client_node,
            "server_pid": server.pid,
            "client_pid": client.pid,
            "ros_domain_id": domain_id,
            "requested_rmw": RMW,
            "ros_distro": ROS_DISTRO,
            "topology": {
                "process_count": 2,
                "fresh_process_groups": True,
                "one_active_goal": True,
                "common_aot_server": True,
                "action_type": ACTION_TYPE,
                "qos": QOS,
                "endpoints": endpoint_names(action_name),
            },
            "graph": {
                "server_present_after_ready": True,
                "client_present_after_ready": True,
                "exact_endpoints_present": True,
                "client_absent_after_exit": True,
                "server_absent_after_exit": True,
                "endpoints_absent_after_exit": True,
                "ready_observation": ready_graph,
                "client_exit_observation": client_exit_graph,
                "final_observation": final_graph,
            },
            "server_ready": server_ready,
            "client_ready": client_ready,
            "client_armed": client_armed,
            "client_report": client_report,
            "server_report": server_report,
            "timing": timing,
            "server_cpu_diagnostic": {
                "role": "drift_only",
                "cpu_time_ns": server_report["cpu_time_ns"],
                "cpu_ns_per_measured_goal": (
                    server_report["cpu_time_ns"] / MEASURED_GOALS),
            },
            "correctness_verified": True,
            "teardown_verified": True,
            "diagnostics": {
                "server_stdout": diagnostics["server"],
                "server_stderr": server_finish["stderr"],
                "client_stdout": diagnostics["client"],
                "client_stderr": client_finish["stderr"],
            },
        }
        validate_sample(sample, cache, build)
        return sample
    finally:
        _stop_process(client)
        _stop_process(server)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    if os.environ.get("ROS_DISTRO") != ROS_DISTRO:
        raise SystemExit("action benchmark requires ROS_DISTRO=jazzy")
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = RMW
    env["CPPYY_KIT_NO_AUTOPCH"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    env.pop("CPPYY_KIT_NO_CACHE", None)
    os.environ["RMW_IMPLEMENTATION"] = RMW
    results = []
    failures = []
    execution_order = []
    with tempfile.TemporaryDirectory(prefix="rclcppyy-action-client-") as temporary:
        root = Path(temporary)
        build_directory = root / "aot-build"
        cache_root = root / "cache"
        build_directory.mkdir(mode=0o700)
        cache_root.mkdir(mode=0o700)
        env["XDG_CACHE_HOME"] = str(cache_root)
        executables, build = _compile_aot(build_directory, env, args.timeout)
        with acquire_domain() as lease:
            env["ROS_DOMAIN_ID"] = str(lease.domain_id)
            os.environ["ROS_DOMAIN_ID"] = str(lease.domain_id)
            cache = _prewarm(cache_root, env, args.timeout)
            observer = GraphObserver("action_benchmark_observer_%s" % uuid.uuid4().hex[:12])
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
                                executables=executables,
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
                "warmup_goals": WARMUP_GOALS,
                "measured_goals": MEASURED_GOALS,
                "feedback_per_goal": FEEDBACK_PER_GOAL,
                "repetitions": REPETITIONS,
                "requested_rmw": RMW,
                "ros_distro": ROS_DISTRO,
                "action_type": ACTION_TYPE,
                "qos": QOS,
                "execution_order": execution_order,
            }
            isolation = {
                "fresh_process_pair_per_sample": True,
                "fresh_process_groups_per_sample": True,
                "unique_action_and_nodes_per_sample": True,
                "one_leased_domain_per_run": True,
                "rotating_variant_order": True,
                "ros_domain_id": lease.domain_id,
            }
            source_files = {
                "runner": _sha256(Path(__file__)),
                "protocol": _sha256(PROTOCOL),
                "worker": _sha256(WORKER),
                "server_source": _sha256(SERVER_SOURCE),
                "client_source": _sha256(CLIENT_SOURCE),
                "aot_cmake": _sha256(AOT_CMAKE),
            }
            document = build_document(
                repo_root=REPO_ROOT,
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
                    "Action benchmark emitted %d raw samples; claims and interpretation disabled."
                    % len(results))
                if failures:
                    print("%d sample(s) failed" % len(failures), file=sys.stderr)
            return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
