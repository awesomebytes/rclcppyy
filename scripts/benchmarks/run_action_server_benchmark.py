#!/usr/bin/env python3
"""Run the CPU-first Jazzy/Cyclone action-server benchmark."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import psutil
import select
import sys
import tempfile
import uuid

from _action_server_protocol import (
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
from run_action_client_benchmark import (
    AOT_CMAKE,
    CLIENT_SOURCE,
    SERVER_SOURCE,
    GraphObserver,
    _compile_aot,
    _finish,
    _one_protocol_line,
    _read_document,
    _run_command,
    _sha256,
    _spawn,
    _stop_process,
    _write_control,
)


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "action_server_worker.py"
PROTOCOL = HERE / "_action_server_protocol.py"


def _prewarm(cache_root: Path, env: dict, timeout: float) -> dict:
    if any(cache_root.iterdir()):
        raise RuntimeError("action-server cache root must start empty")
    command = [sys.executable, "-u", str(WORKER), "--prewarm"]
    cold, cold_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="cold server prewarm"),
        "cold server prewarm",
    )
    warm, warm_diagnostics = _one_protocol_line(
        _run_command(command, env=env, timeout=timeout, label="warm server prewarm"),
        "warm server prewarm",
    )
    cold["stdout_diagnostics"] = cold_diagnostics
    warm["stdout_diagnostics"] = warm_diagnostics
    validate_prewarm(cold, expect_hit=False)
    validate_prewarm(warm, expect_hit=True)
    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": cold, "warm": warm},
    }


def _server_argv(
    variant: str,
    executables: dict,
    node_name: str,
    action_name: str,
    token: str,
    warmup_goals: int,
    measured_goals: int,
    shared_values: bool = False,
) -> list[str]:
    if variant == "aot-staged":
        return [
            str(executables["server"]),
            node_name,
            action_name,
            token,
            variant,
            str(warmup_goals),
            str(measured_goals),
            "server_under_test",
        ]
    command = [
        sys.executable,
        "-u",
        str(WORKER),
        "--variant",
        variant,
        "--node-name",
        node_name,
        "--action-name",
        action_name,
        "--run-token",
        token,
        "--warmup-goals",
        str(warmup_goals),
        "--measured-goals",
        str(measured_goals),
    ]
    if shared_values:
        command.append("--shared-values")
    return command


def _client_argv(
    executable: Path,
    node_name: str,
    action_name: str,
    token: str,
    variant: str,
    warmup_goals: int,
    measured_goals: int,
) -> list[str]:
    return [
        str(executable),
        node_name,
        action_name,
        token,
        variant,
        str(warmup_goals),
        str(measured_goals),
    ]


def _run_sample(
    *,
    variant: str,
    repetition: int,
    executables: dict,
    cache: dict,
    observer: GraphObserver,
    domain_id: int,
    env: dict,
    timeout: float,
    warmup_goals: int = WARMUP_GOALS,
    measured_goals: int = MEASURED_GOALS,
    validate: bool = True,
    shared_values: bool = False,
) -> dict:
    token = "action_server_" + uuid.uuid4().hex
    suffix = token[14:26]
    action_name = "/rclcppyy/action_server_benchmark/run_%s" % token[14:]
    server_node = "action_server_%s" % suffix
    client_node = "action_client_%s" % suffix
    server = None
    client = None
    server_finish = {"stdout": [], "stderr": None}
    client_finish = {"stdout": [], "stderr": None}
    diagnostics = {"server": [], "client": []}
    try:
        server = _spawn(
            _server_argv(
                variant,
                executables,
                server_node,
                action_name,
                token,
                warmup_goals,
                measured_goals,
                shared_values,
            ),
            env,
        )
        server_ready, values = _read_document(
            server, timeout, "action-server READY")
        diagnostics["server"].extend(values)
        if server_ready.get("schema") != SERVER_SCHEMA or server_ready.get(
                "event") != "ready":
            raise RuntimeError("action-server emitted invalid READY evidence")
        client = _spawn(
            _client_argv(
                executables["client"],
                client_node,
                action_name,
                token,
                variant,
                warmup_goals,
                measured_goals,
            ),
            env,
        )
        client_ready, values = _read_document(
            client, timeout, "common AOT action client READY")
        diagnostics["client"].extend(values)
        if client_ready.get("schema") != CLIENT_SCHEMA or client_ready.get(
                "event") != "ready":
            raise RuntimeError("common AOT client emitted invalid READY evidence")
        for role, process in (("server", server), ("client", client)):
            if process.pid != os.getpgid(process.pid):
                raise RuntimeError("action-server %s lacks a fresh process group" % role)
            if psutil.Process(process.pid).children(recursive=True):
                raise RuntimeError("action-server %s retained setup children" % role)
        ready_graph = observer.wait_ready(
            server_node, client_node, action_name, timeout)
        _write_control(client, "ARM", "common AOT action client")
        client_armed, values = _read_document(
            client, timeout, "common AOT action client ARMED")
        diagnostics["client"].extend(values)
        if client_armed.get("event") != "armed":
            raise RuntimeError("common AOT client emitted invalid ARMED evidence")
        _write_control(client, "MEASURE", "common AOT action client")
        client_report, values = _read_document(
            client, timeout, "common AOT action client REPORT")
        diagnostics["client"].extend(values)
        if client_report.get("event") != "report":
            raise RuntimeError("common AOT client emitted invalid REPORT evidence")
        client_finish = _finish(client, timeout, "common AOT action client")
        diagnostics["client"].extend(client_finish["stdout"])
        client_exit = observer.wait_client_exit(
            server_node, client_node, action_name, timeout)
        _write_control(server, "STOP", "action-server")
        server_report, values = _read_document(
            server, timeout, "action-server REPORT")
        diagnostics["server"].extend(values)
        if server_report.get("event") != "report":
            raise RuntimeError("action-server emitted invalid REPORT evidence")
        server_finish = _finish(server, timeout, "action-server")
        diagnostics["server"].extend(server_finish["stdout"])
        final_exit = observer.wait_final_exit(
            server_node, client_node, action_name, timeout)
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
                "common_aot_client": True,
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
                "client_exit_observation": client_exit,
                "final_observation": final_exit,
            },
            "server_ready": server_ready,
            "client_ready": client_ready,
            "client_armed": client_armed,
            "client_report": client_report,
            "server_report": server_report,
            "timing": {
                "primary_metric": "server_cpu_ns_per_completed_goal",
                "server_cpu_ns_per_completed_goal": (
                    server_report["cpu_time_ns"] / measured_goals),
                "completed_goals_per_second": (
                    measured_goals * 1e9 / client_report["wall_duration_ns"]),
                "latency_ns": client_report["latency_ns"],
                "client_cpu_diagnostic_ns_per_goal": (
                    client_report["cpu_time_ns"] / measured_goals),
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
        if validate:
            validate_sample(sample, cache)
        return sample
    except Exception as exc:
        peer_diagnostics = []
        for role, process in (("server", server), ("client", client)):
            if process is None:
                continue
            status = process.poll()
            stream_values = []
            for stream_name, stream in (
                    ("stdout", process.stdout), ("stderr", process.stderr)):
                readable, _, _ = select.select([stream], [], [], 0)
                if readable:
                    value = os.read(stream.fileno(), 4000).decode(
                        errors="replace").strip()
                    if value:
                        stream_values.append("%s=%s" % (stream_name, value))
            if status is not None and not stream_values:
                stream_values.append("streams-empty")
            if stream_values:
                peer_diagnostics.append(
                    "%s %s: %s" % (
                        role,
                        "exited %s" % status if status is not None else "live",
                        "; ".join(stream_values),
                    ))
        detail = "; ".join(peer_diagnostics) or "both peers still running"
        raise RuntimeError(
            "%s lane failed: %s; peer diagnostics: %s" % (
                variant, exc, detail)) from exc
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
        raise SystemExit("action-server benchmark requires ROS_DISTRO=jazzy")
    env = os.environ.copy()
    env.update({
        "RMW_IMPLEMENTATION": RMW,
        "CPPYY_KIT_NO_AUTOPCH": "1",
        "PYTHONUNBUFFERED": "1",
    })
    env.pop("CPPYY_KIT_NO_CACHE", None)
    os.environ["RMW_IMPLEMENTATION"] = RMW
    results = []
    failures = []
    execution_order = []
    with tempfile.TemporaryDirectory(prefix="rclcppyy-action-server-") as temporary:
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
            observer = GraphObserver(
                "action_server_benchmark_observer_%s" % uuid.uuid4().hex[:12])
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
                "primary_metric": "server_cpu_ns_per_completed_goal",
                "common_driver": "conventional-release-aot-rclcpp-action-client",
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
                "aot_server": _sha256(SERVER_SOURCE),
                "aot_client": _sha256(CLIENT_SOURCE),
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
                    "Action-server benchmark emitted %d raw samples; claims disabled."
                    % len(results))
                if failures:
                    print("%d sample(s) failed" % len(failures), file=sys.stderr)
            return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
