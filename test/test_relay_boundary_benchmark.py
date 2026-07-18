"""Contract and live smoke coverage for the controlled relay benchmark."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_relay_boundary_protocol")
worker = _load("relay_boundary_worker")
runner = _load("run_relay_boundary_benchmark")
DIGEST = "a" * 64
RMW = "rmw_cyclonedds_cpp"
MESSAGES = 4
WARMUP = 2


def _artifact(*, cached):
    return {
        "cached": cached,
        "reason": "hit" if cached else "miss-built",
        "path": "/fixture/cache/artifact.so",
        "sha256": DIGEST,
        "size_bytes": 4096,
    }


def _cache():
    def phase(cached):
        return {
            "schema": protocol.PREWARM_SCHEMA,
            "pid": 42 if not cached else 43,
            "loaded_rmw": RMW,
            "fused_source_id": "1" * 16,
            "artifacts": {
                "subscription": _artifact(cached=cached),
                "fused_pipeline": _artifact(cached=cached),
            },
            "stdout_diagnostics": ["fixture initialization"],
        }

    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": phase(False), "warm": phase(True)},
    }


def _build():
    return {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": "/fixture/c++",
        "compiler_version": "fixture c++ 1.0",
        "compile_command": "c++ -O3 -DNDEBUG relay_boundary_aot.cpp",
        "source_sha256": DIGEST,
        "kernel_sha256": DIGEST,
        "cmake_sha256": DIGEST,
        "compile_commands_sha256": DIGEST,
        "executable_sha256": DIGEST,
        "executable_format": "ELF",
        "build_elapsed_ns": 100,
    }


def _rss_guard():
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": 10_000,
        "final_peak_bytes": 12_000,
        "growth_bytes": 2_000,
        "limit_bytes": protocol.RSS_GUARD_LIMIT_BYTES,
        "within_limit": True,
    }


def _backend_marker(role, backend, evidence, metadata=None):
    return {
        "schema": "rclcppyy.benchmark-backend/v1",
        "role": role,
        "backend": backend,
        "evidence": evidence,
        "metadata": metadata if metadata is not None else {"fixture": True},
    }


def _sample(variant, index=0):
    token = "run_" + format(index + 1, "032x")
    relay_pid = 1000 + index * 2
    driver_pid = relay_pid + 1
    relay_node = "relay_fixture_%d" % index
    driver_node = "driver_fixture_%d" % index
    input_topic = "/fixture/%s/input" % token
    output_topic = "/fixture/%s/output" % token
    spec = protocol.VARIANTS[variant]
    ready = {
        "schema": protocol.RELAY_SCHEMA,
        "event": "ready",
        "variant": variant,
        "run_token": token,
        "pid": relay_pid,
        "process_group_id": relay_pid,
        "node_name": relay_node,
        "loaded_rmw": RMW,
        "execution_model": spec["execution_model"],
    }
    if variant == "stock-rclpy":
        ready["cache"] = {"state": "not_applicable", "kind": "stock-rclpy"}
    elif variant == "compatible-rclcppyy":
        ready["cache"] = {
            "state": "not_applicable",
            "kind": "compatible-stock-publish-authority",
        }
    elif variant == "publisher-cpp-rclcppyy":
        ready["cache"] = {
            "state": "process-warm",
            "kind": "publisher-cpp-borrowed-publish-route",
            "prepared_before_measurement": True,
        }
    elif variant == "direct-cpp-rclcppyy":
        ready["cache"] = {
            "state": "prebuilt",
            "kind": "direct-cpp-subscription-trampoline",
            "path": "/fixture/cache/artifact.so",
            "sha256": DIGEST,
            "size_bytes": 4096,
            "hit": True,
        }
    elif variant == "aot-staged":
        ready["cache"] = {"state": "prebuilt", "kind": "aot-binary"}
    else:
        name = "subscription" if variant == "native-python-callback" else "fused_pipeline"
        ready["cache"] = {
            "state": "prebuilt",
            "kind": spec["cache_kind"],
            "path": "/fixture/cache/artifact.so",
            "sha256": DIGEST,
            "size_bytes": 4096,
            "hit": True,
        }
        if variant == "native-fused":
            ready["cache"]["source_id"] = "1" * 16
        assert name in _cache()["phases"]["warm"]["artifacts"]
    if variant == "direct-cpp-rclcppyy":
        ready["entity_types"] = {
            "node": "rclcpp::Node",
            "publisher": "rclcpp::Publisher<std_msgs::msg::UInt64>",
            "subscription": "rclcpp::Subscription<std_msgs::msg::UInt64>",
            "executor": "rclcpp::executors::SingleThreadedExecutor",
        }
        ready["direct_cpp_proof"] = {
            "actual_cpp_message_class": True,
            "message_cpp_name": "std_msgs::msg::UInt64_<std::allocator<void>>",
            "single_native_node_authority": True,
            "session_node_count": 1,
            "callback_handoff": "one_native_cpp_copy",
            "subscription_creation_route": "prebuilt_subscription_trampoline",
            "python_message_conversion_guarded": True,
            "serialization_guarded": True,
        }
        ready["backend_markers"] = {
            "publisher": _backend_marker(
                "publisher", "cpp", "rclcppyy_status_entity",
                metadata={
                    "policies": ["direct_cpp", "no_conversion"],
                },
            ),
            "subscriber": _backend_marker(
                "subscriber", "cpp", "rclcppyy_status_entity",
                metadata={
                    "policies": [
                        "direct_cpp", "no_conversion",
                        "owning_cpp_callback_copy",
                    ],
                    "callback_handoff": "one_native_cpp_copy",
                },
            ),
        }
    elif variant in (
            "stock-rclpy", "compatible-rclcppyy", "publisher-cpp-rclcppyy"):
        ready["entity_types"] = {
            "node": "rclpy.node.Node",
            "publisher": "rclpy.publisher.Publisher",
            "subscription": "rclpy.subscription.Subscription",
            "executor": "rclpy.executors.SingleThreadedExecutor",
        }
        backend = "cpp" if variant == "publisher-cpp-rclcppyy" else "python"
        evidence = (
            "stock_rclpy_entity" if variant == "stock-rclpy"
            else "rclcppyy_status_entity"
        )
        ready["backend_markers"] = {
            "publisher": _backend_marker("publisher", backend, evidence),
            "subscriber": _backend_marker(
                "subscriber", "python", evidence),
        }
    elif variant != "aot-staged":
        ready["entity_types"] = {
            "node": "rclcpp::Node",
            "publisher": "rclcpp::Publisher",
            "subscription": "rclcpp::Subscription",
            "executor": "rclcpp::executors::SingleThreadedExecutor",
        }

    total = WARMUP + MESSAGES
    report = {
        "schema": protocol.RELAY_SCHEMA,
        "event": "report",
        "variant": variant,
        "run_token": token,
        "received": total,
        "processed": total,
        "published": total,
        "checksum": protocol.expected_input_checksum(total),
        "last": total,
        "python_callback_count": total if spec["python_crossings"] else 0,
        "python_boundary_crossings": total if spec["python_crossings"] else 0,
        "cpu_time_ns": 600,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(),
        "dropped": 0,
        "exceptions": 0,
        "correct": True,
        "teardown_clean": True,
    }
    if variant in ("stock-rclpy", "compatible-rclcppyy"):
        report.update({
            "publish_operation_marker": None,
            "fallback_publish_operations": 0,
            "last_publish_backend": "python",
            "publish_route_tainted": False,
        })
    elif variant == "publisher-cpp-rclcppyy":
        report.update({
            "publish_operation_marker": _backend_marker(
                "publisher", "cpp", "rclcppyy_status_operation"),
            "fallback_publish_operations": 0,
            "last_publish_backend": "cpp",
            "publish_route_tainted": False,
            "status_operation_counts": {
                "cpp": 2, "python": 0, "unsupported": 0},
            "status_dropped_operation_records": 0,
        })
    elif variant == "direct-cpp-rclcppyy":
        report.update({
            "publish_operation_marker": None,
            "fallback_publish_operations": 0,
            "last_publish_backend": "cpp",
            "publish_route_tainted": False,
            "owning_cpp_callback_copies": total,
            "cpp_callback_messages": total,
            "non_cpp_callback_messages": 0,
            "python_message_conversions": 0,
            "serialization_operations": 0,
            "boundary_guard_calls": {
                "python_message_conversion": 0,
                "serialization": 0,
            },
        })
    elif variant == "native-fused":
        report.update({
            "checksum": None,
            "last": None,
            "compile_cache_hits": 1,
            "compile_cache_misses": 0,
        })

    def endpoint(role, node_name, topic, remote=False):
        return {
            "role": role,
            "node_name": node_name,
            "observed_node_name": "_NODE_NAME_UNKNOWN_" if remote else node_name,
            "ownership_evidence": (
                "exact-process-pair-unique-topic" if remote
                else "middleware-graph-owner"
            ),
            "node_namespace": "/",
            "observed_node_namespace": (
                "_NODE_NAMESPACE_UNKNOWN_" if remote else "/"
            ),
            "topic": topic,
            **protocol.QOS,
        }

    latencies = [100, 200, 300, 400]
    driver = {
        "schema": protocol.DRIVER_SCHEMA,
        "event": "measured",
        "run_token": token,
        "pid": driver_pid,
        "process_group_id": driver_pid,
        "loaded_rmw": RMW,
        "execution_model": "identical-release-aot-closed-loop-driver",
        "messages": MESSAGES,
        "checksum": protocol.expected_output_checksum(WARMUP, MESSAGES),
        "last": protocol.transformed(WARMUP + MESSAGES),
        "elapsed_ns": 1000,
        "cpu_time_ns": 500,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(),
        "topology_verified": True,
        "qos_verified": True,
        "endpoints": [
            endpoint("driver_publisher", driver_node, input_topic),
            endpoint("relay_subscription", relay_node, input_topic, True),
            endpoint("relay_publisher", relay_node, output_topic, True),
            endpoint("driver_subscription", driver_node, output_topic),
        ],
        "latency_ns": latencies,
    }
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        "case_id": "%s__rep_1" % variant,
        "variant": variant,
        "repetition": 1,
        "run_token": token,
        "ros_domain_id": 77,
        "requested_rmw": RMW,
        "qos": dict(protocol.QOS),
        "relay_pid": relay_pid,
        "driver_pid": driver_pid,
        "topology": {
            "process_count": 2,
            "fresh_process_groups": True,
            "driver_node": driver_node,
            "relay_node": relay_node,
            "input_topic": input_topic,
            "output_topic": output_topic,
        },
        "relay_ready": ready,
        "relay_armed": {
            "schema": protocol.RELAY_SCHEMA,
            "event": "armed",
            "variant": variant,
            "run_token": token,
            "pid": relay_pid,
            "process_group_id": relay_pid,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        },
        "relay_report": report,
        "driver_result": driver,
        "driver_teardown": {
            "schema": protocol.DRIVER_SCHEMA,
            "event": "teardown",
            "run_token": token,
            "pid": driver_pid,
            "process_group_id": driver_pid,
            "teardown_clean": True,
        },
        "timing": {
            "elapsed_ns": 1000,
            "throughput_messages_per_second": 4_000_000.0,
            "relay_cpu_time_ns": 600,
            "relay_cpu_ns_per_message": 150.0,
            "driver_cpu_time_ns": 500,
            "driver_cpu_ns_per_message": 125.0,
            "latency_ns": protocol.latency_summary(latencies),
        },
        "backend_verified": True,
        "correctness_verified": True,
        "teardown_verified": True,
        "diagnostics": {},
    }


def _parameters(rmw=RMW):
    return {
        "variants": list(protocol.VARIANTS),
        "messages": MESSAGES,
        "warmup_messages": WARMUP,
        "repetitions": 1,
        "requested_rmw": rmw,
        "qos": dict(protocol.QOS),
        "execution_order": ["%s__rep_1" % name for name in protocol.VARIANTS],
    }


def test_transform_percentiles_and_shared_compatibility_relay():
    assert protocol.transformed(5) == 11
    assert protocol.nearest_rank([4, 1, 3, 2], 50) == 2
    assert protocol.latency_summary([4, 1, 3, 2]) == {
        "p50": 2, "p95": 4, "p99": 4, "max": 4}
    assert worker._run_python_relay.__code__.co_varnames[:2] == ("args", "profile")
    worker_source = (
        BENCH_DIR / "relay_boundary_worker.py").read_text(encoding="utf-8")
    relay_source = worker_source[
        worker_source.index("def _run_python_relay"):
        worker_source.index("def _run_python_callback")
    ]
    assert relay_source.index("publish-marker-capture-before") < relay_source.index(
        "cpu_start = time.process_time_ns()")
    assert "finally:\n        if use_direct_cpp:" in relay_source
    assert "teardown_clean = _cleanup_direct_cpp_relay" in relay_source
    assert "teardown_clean = _cleanup_python_relay" in relay_source
    assert 'profile="direct_cpp"' in worker_source
    assert "owning_cpp_callback_copies" in relay_source
    aot_source = (
        BENCH_DIR / "relay_boundary_aot" / "relay_boundary_aot.cpp"
    ).read_text(encoding="utf-8")
    assert "while (endpoints.empty())" in aot_source
    assert "catch (const GraphIncomplete &)" in aot_source
    assert "input_publishers.size() > 1" in aot_source


def test_armed_handshake_and_timeout_stack_dump_are_fail_closed():
    token = "run_" + "1" * 32
    armed = {
        "schema": protocol.RELAY_SCHEMA,
        "event": "armed",
        "variant": "compatible-rclcppyy",
        "run_token": token,
        "pid": 123,
        "process_group_id": 123,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
    }
    runner._validate_armed_before_measurement(
        armed, variant="compatible-rclcppyy", token=token, relay_pid=123)
    invalid = dict(armed, process_group_id=124)
    with pytest.raises(RuntimeError, match="invalid armed evidence"):
        runner._validate_armed_before_measurement(
            invalid, variant="compatible-rclcppyy", token=token, relay_pid=123)

    code = """
import faulthandler
import signal
import sys
import time
faulthandler.enable(file=sys.stderr, all_threads=True)
faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True)
print('fixture-phase-before-block', file=sys.stderr, flush=True)
while True:
    time.sleep(1)
"""
    process = subprocess.Popen(
        [sys.executable, "-c", code],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        with pytest.raises(runner.ProtocolTimeout) as captured:
            runner._read_document(
                process, 0.5, "fixture worker", request_stack_dump=True)
        assert captured.value.stderr is not None
        assert "fixture-phase-before-block" in captured.value.stderr
        assert "Current thread" in captured.value.stderr
    finally:
        runner._stop_process(process)


def test_all_seven_fixture_routes_satisfy_strict_sample_contract():
    parameters = _parameters()
    for index, variant in enumerate(protocol.VARIANTS):
        protocol.validate_sample(
            _sample(variant, index), parameters, _build(), _cache())


@pytest.mark.parametrize(
    ("variant", "mutation", "error"),
    [
        ("publisher-cpp-rclcppyy", lambda row: row["relay_report"].update(
            fallback_publish_operations=1), "fell back"),
        ("compatible-rclcppyy", lambda row: row["driver_result"]["endpoints"][0].update(
            depth=2), "QoS"),
        ("native-python-callback", lambda row: row["relay_ready"]["cache"].update(
            sha256="b" * 64), "differs from the warm manifest"),
        ("compatible-rclcppyy", lambda row: row.update(
            driver_pid=row["relay_pid"]), "distinct process"),
        ("compatible-rclcppyy", lambda row: row["relay_report"].update(
            python_callback_count=0), "Python-boundary count"),
        ("compatible-rclcppyy", lambda row: row["relay_report"]["rss_guard"].update(
            within_limit=False), "RSS"),
        ("compatible-rclcppyy", lambda row: row["relay_report"].update(
            cpu_clock="wall"), "CPU timing"),
        ("publisher-cpp-rclcppyy", lambda row: row["relay_report"].update(
            publish_route_tainted=True), "tainted"),
        ("direct-cpp-rclcppyy", lambda row: row["relay_report"].update(
            owning_cpp_callback_copies=0), r"callback C\+\+ copy"),
        ("direct-cpp-rclcppyy", lambda row: row["relay_report"].update(
            python_message_conversions=1), "conversion or serialization"),
        ("direct-cpp-rclcppyy", lambda row: row["relay_ready"][
            "direct_cpp_proof"].update(actual_cpp_message_class=False),
         "representation or authority"),
    ],
)
def test_routes_reject_tainted_evidence(variant, mutation, error):
    sample = _sample(variant, 1)
    mutation(sample)
    with pytest.raises(ValueError, match=error):
        protocol.validate_sample(sample, _parameters(), _build(), _cache())


def test_unknown_remote_owner_is_rejected_for_non_cyclone_rmw():
    sample = _sample("aot-staged", 4)
    alternate = "rmw_fastrtps_cpp"
    sample["requested_rmw"] = alternate
    sample["relay_ready"]["loaded_rmw"] = alternate
    sample["driver_result"]["loaded_rmw"] = alternate
    with pytest.raises(ValueError, match="observed endpoint owner"):
        protocol.validate_sample(
            sample, _parameters(alternate), _build(), _cache())


def test_document_is_descriptive_and_portable_schema_forbids_claims(monkeypatch):
    monkeypatch.setattr(protocol, "environment_metadata", lambda _root: {
        "source": {"commit": "1" * 40, "dirty": True},
        "source_dependencies": {
            "rclcpp_kit": {"commit": "2" * 40, "dirty": False},
        },
    })
    results = [_sample(variant, index) for index, variant in enumerate(protocol.VARIANTS)]
    document = protocol.build_document(
        repo_root=REPO_ROOT,
        mode="measurement",
        parameters=_parameters(),
        isolation={
            "fresh_process_pair_per_sample": True,
            "two_process_groups_per_sample": True,
            "unique_topics_per_sample": True,
            "one_leased_domain_per_run": True,
            "rotating_variant_order": True,
            "ros_domain_id": 77,
        },
        aot_build=_build(),
        cache=_cache(),
        source_files={"runner": DIGEST},
        results=results,
        failures=[],
        command=["relay-boundary-bench"],
    )
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    assert "rss" not in json.dumps(document["comparison"]).lower()
    assert json.loads(protocol.dumps(document)) == document

    schema = json.loads((
        REPO_ROOT / "schemas" / "relay-boundary-benchmark-v1.schema.json"
    ).read_text(encoding="utf-8"))
    assert schema["properties"]["benchmark"]["properties"][
        "performance_claims_allowed"] == {"const": False}
    assert schema["properties"]["comparison"]["properties"][
        "interpretation_allowed"] == {"const": False}
    assert len(schema["properties"]["results"]["items"]["allOf"]) == 7
    assert "relay_armed" in schema["properties"]["results"]["items"]["required"]


def test_all_variants_run_with_one_aot_driver_and_exact_parity(tmp_path):
    output = tmp_path / "relay-boundary-smoke.json"
    environment = os.environ.copy()
    environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    process = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_relay_boundary_benchmark.py"),
            "--smoke",
            "--messages", "4",
            "--warmup-messages", "1",
            "--repetitions", "1",
            "--output", str(output),
            "--json",
        ],
        cwd=REPO_ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert process.returncode == 0, (
        f"stdout:\n{process.stdout}\nstderr:\n{process.stderr}")
    document = json.loads(process.stdout)
    assert document == json.loads(output.read_text(encoding="utf-8"))
    assert document["failures"] == []
    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["parameters"][
        "requested_rmw"] == "rmw_cyclonedds_cpp"
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    assert {row["variant"] for row in document["results"]} == set(
        protocol.VARIANTS)
    assert len({
        pid for row in document["results"]
        for pid in (row["relay_pid"], row["driver_pid"])
    }) == 14
    for row in document["results"]:
        report = row["relay_report"]
        assert report["received"] == report["processed"] == report["published"] == 5
        assert report["rss_guard"]["within_limit"] is True
        assert row["driver_result"]["messages"] == 4
        assert len(row["driver_result"]["latency_ns"]) == 4
    callbacks = {
        row["variant"]: row["relay_report"]["python_callback_count"]
        for row in document["results"]
    }
    assert callbacks == {
        "stock-rclpy": 5,
        "compatible-rclcppyy": 5,
        "publisher-cpp-rclcppyy": 5,
        "direct-cpp-rclcppyy": 5,
        "native-python-callback": 5,
        "native-fused": 0,
        "aot-staged": 0,
    }
    direct = next(
        row for row in document["results"]
        if row["variant"] == "direct-cpp-rclcppyy"
    )
    assert direct["relay_ready"]["direct_cpp_proof"] == {
        "actual_cpp_message_class": True,
        "message_cpp_name": "std_msgs::msg::UInt64_<std::allocator<void>>",
        "single_native_node_authority": True,
        "session_node_count": 1,
        "callback_handoff": "one_native_cpp_copy",
        "subscription_creation_route": "prebuilt_subscription_trampoline",
        "python_message_conversion_guarded": True,
        "serialization_guarded": True,
    }
    assert direct["relay_report"]["owning_cpp_callback_copies"] == 5
    assert direct["relay_report"]["cpp_callback_messages"] == 5
    assert direct["relay_report"]["boundary_guard_calls"] == {
        "python_message_conversion": 0,
        "serialization": 0,
    }
