"""Contract and live smoke coverage for the service client benchmark."""

from __future__ import annotations

import importlib.util
import inspect
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


protocol = _load("_service_client_protocol")
worker = _load("service_client_worker")
runner = _load("run_service_client_benchmark")
DIGEST = "a" * 64
RMW = "rmw_cyclonedds_cpp"
MESSAGES = 4
WARMUP = 2
SET_BOOL = "std_srvs/srv/SetBool"
TRIGGER = "std_srvs/srv/Trigger"


def _artifact(*, cached):
    return {
        "cached": cached,
        "reason": "hit" if cached else "miss-built",
        "path": "/fixture/cache/artifact.so",
        "sha256": DIGEST,
        "size_bytes": 4096,
    }


def _cache(service_type=SET_BOOL):
    def phase(cached):
        return {
            "schema": protocol.PREWARM_SCHEMA,
            "pid": 42 if not cached else 43,
            "loaded_rmw": RMW,
            "service_type": service_type,
            "native_client_source_id": "1" * 16,
            "artifacts": {
                "native_client": _artifact(cached=cached),
                "cpp_state_machine": _artifact(cached=cached),
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
        "compile_command": "c++ -O3 -DNDEBUG service_client_aot.cpp",
        "source_sha256": DIGEST,
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
        "limit_bytes": protocol.RSS_LIMIT_BYTES,
        "within_limit": True,
    }


def _backend_marker(evidence, service_type=SET_BOOL):
    return {
        "schema": "rclcppyy.benchmark-backend/v1",
        "role": "client",
        "backend": "python",
        "evidence": evidence,
        "metadata": {"fixture": True, "service_type": service_type},
    }


def _sample(variant, index=0, service_type=SET_BOOL):
    token = "run_" + format(index + 1, "032x")
    server_pid = 3000 + index * 2
    client_pid = server_pid + 1
    server_node = "service_server_%d" % index
    client_node = "service_client_%d" % index
    service_name = "/fixture/" + token
    spec = protocol.VARIANTS[variant]
    service_spec = protocol.SERVICE_SPECS[service_type]
    entity_type = "rclcpp::Client<%s>" % service_spec["cpp_type"]
    server_ready = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "ready",
        "run_token": token,
        "pid": server_pid,
        "process_group_id": server_pid,
        "node_name": server_node,
        "loaded_rmw": RMW,
        "execution_model": service_spec["server_model"],
        "service_name": service_name,
        "service_type": service_type,
    }
    warmed = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "warmed",
        "variant": variant,
        "run_token": token,
        "pid": client_pid,
        "process_group_id": client_pid,
        "node_name": client_node,
        "loaded_rmw": RMW,
        "execution_model": spec["model"],
        "client_authority": spec["authority"],
        "warmup_requests": WARMUP,
        "topology_verified": True,
        "endpoint_count": 1,
        "server_node": server_node,
        "service_name": service_name,
        "service_type": service_type,
        "qos_verified": True,
    }
    if variant == "aot-staged":
        warmed["cache"] = {"state": "prebuilt", "kind": "aot-binary"}
        warmed["entity_type"] = entity_type
    elif variant in ("stock-rclpy", "compatible-rclcppyy"):
        warmed["cache"] = {"state": "not_applicable", "kind": spec["cache"]}
        warmed["entity_type"] = "rclpy.client.Client"
        warmed["backend_marker"] = _backend_marker(
            "stock_rclpy_entity" if variant == "stock-rclpy"
            else "rclcppyy_status_entity", service_type)
    else:
        artifact_name = (
            "native_client"
            if variant in ("direct-cpp-rclcppyy", "native-python-orchestrated")
            else "cpp_state_machine")
        warmed["cache"] = {
            **_cache(service_type)["phases"]["warm"]["artifacts"][artifact_name],
            "state": "prebuilt",
            "kind": spec["cache"],
        }
        warmed["entity_type"] = entity_type
        if variant == "direct-cpp-rclcppyy":
            warmed["cache"]["source_id"] = "1" * 16
            warmed["direct_cpp_proof"] = {
                "profile": "direct_cpp",
                "node_authority": "cpp",
                "client_authority": "cpp",
                "client_entity_type": entity_type,
                "runtime_facade_node_count": 1,
                "native_session_node_count": 1,
                "native_node_identity_verified": True,
                "request_representation": "actual_cpp",
                "response_representation": "actual_cpp",
                "future_type": "rclpy.task.Future",
                "future_control": "per_operation_rclpy_task_future",
                "request_handoff": "one_native_cpp_value_copy",
                "response_handoff": "shared_cpp_response",
                "python_request_crossings_per_call": 1,
                "python_response_crossings_per_call": 1,
                "python_message_conversions_per_call": 0,
                "cpp_request_copies_per_call": 1,
                "python_conversion_guard_installed": True,
                "serialization_guards_installed": True,
                "status_decision": {
                    "id": "entity-00000002",
                    "backend": "cpp",
                    "reason": "direct typed rclcpp client with C++ service messages",
                    "policies": [
                        "direct_cpp", "direct_cpp_service", "no_conversion",
                        "per_operation_future", "cpp_pending_state",
                    ],
                    "metadata": {
                        "entity_type": "client",
                        "service_name": service_name,
                        "service_type": service_spec["cpp_type"],
                        "service_interface": service_type,
                        "request_representation": "actual_cpp",
                        "response_representation": "actual_cpp",
                        "python_message_conversions": 0,
                        "source_id": "1" * 16,
                        "request_handoff": "one_native_cpp_value_copy",
                        "response_handoff": "shared_cpp_response",
                        "future_control": "per_operation_rclpy_task_future",
                        "python_request_crossings_per_call": 1,
                        "python_response_crossings_per_call": 1,
                        "cpp_request_copies_per_call": 1,
                    },
                },
            }

    total = WARMUP + MESSAGES
    latency = [100, 200, 300, 400]
    client_report = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "measured",
        "variant": variant,
        "run_token": token,
        "pid": client_pid,
        "process_group_id": client_pid,
        "service_type": service_type,
        "messages": MESSAGES,
        "total_requests": total,
        "true_measured": protocol.true_requests(WARMUP + 1, MESSAGES, service_type),
        "response_checksum": protocol.response_checksum(
            WARMUP + 1, MESSAGES, service_type),
        "python_orchestration_requests_measured": spec["orchestration"] * MESSAGES,
        "python_request_crossings_measured": spec["request_crossing"] * MESSAGES,
        "python_response_crossings_measured": spec["response_crossing"] * MESSAGES,
        "python_message_conversions_measured": spec["message_conversions"] * MESSAGES,
        "exceptions": 0,
        "pending_requests": 0,
        "elapsed_ns": 2_000,
        "cpu_time_ns": 600,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(),
        "latency_ns": latency,
    }
    if variant == "direct-cpp-rclcppyy":
        client_report["cpp_request_copies_measured"] = MESSAGES
    server_report = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "report",
        "run_token": token,
        "service_type": service_type,
        "warmup_requests": WARMUP,
        "total_requests": total,
        "measured_requests": MESSAGES,
        "true_total": protocol.true_requests(1, total, service_type),
        "true_measured": protocol.true_requests(WARMUP + 1, MESSAGES, service_type),
        "response_checksum": protocol.response_checksum(
            WARMUP + 1, MESSAGES, service_type),
        "exceptions": 0,
        "pending_requests": 0,
        "cpu_time_ns": 800,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(),
        "correct": True,
        "teardown_clean": True,
    }
    sample = {
        "schema": protocol.SAMPLE_SCHEMA,
        "case_id": "%s__rep_1" % variant,
        "variant": variant,
        "repetition": 1,
        "run_token": token,
        "ros_domain_id": 77,
        "requested_rmw": RMW,
        "service_type": service_type,
        "qos": dict(protocol.QOS),
        "server_pid": server_pid,
        "client_pid": client_pid,
        "topology": {
            "process_count": 2,
            "fresh_process_groups": True,
            "server_node": server_node,
            "client_node": client_node,
            "service_name": service_name,
        },
        "server_ready": server_ready,
        "client_warmed": warmed,
        "server_armed": {
            "schema": protocol.SERVER_SCHEMA,
            "event": "armed",
            "run_token": token,
            "pid": server_pid,
            "process_group_id": server_pid,
            "warmup_requests": WARMUP,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "service_type": service_type,
        },
        "client_report": client_report,
        "server_report": server_report,
        "client_teardown": {
            "schema": protocol.CLIENT_SCHEMA,
            "event": "teardown",
            "variant": variant,
            "run_token": token,
            "pid": client_pid,
            "process_group_id": client_pid,
            "service_type": service_type,
            "endpoint_disappeared": True,
            "teardown_clean": True,
        },
        "timing": {
            "client_cpu_time_ns": 600,
            "client_cpu_ns_per_response": 600 / MESSAGES,
            "server_cpu_time_ns": 800,
            "server_cpu_ns_per_request_drift": 800 / MESSAGES,
            "elapsed_ns": 2_000,
            "requests_per_second": MESSAGES * 1e9 / 2_000,
            "rtt_ns": protocol.latency_summary(latency),
        },
        "backend_verified": True,
        "correctness_verified": True,
        "teardown_verified": True,
        "diagnostics": {},
    }
    if variant == "direct-cpp-rclcppyy":
        sample["client_teardown"]["direct_cpp_teardown"] = {
            "endpoint_disappeared": True,
            "client_closed": True,
            "node_destroyed": True,
            "context_shutdown": True,
            "native_session_closed": True,
            "native_session_released": True,
            "native_executor_released": True,
            "runtime_nodes_released": True,
        }
    return sample


def _parameters(service_type=SET_BOOL):
    return {
        "variants": list(protocol.VARIANTS),
        "messages": MESSAGES,
        "warmup_requests": WARMUP,
        "repetitions": 1,
        "requested_rmw": RMW,
        "service_type": service_type,
        "qos": dict(protocol.QOS),
        "execution_order": ["fixture"],
    }


@pytest.mark.parametrize("service_type", protocol.SERVICE_TYPES)
@pytest.mark.parametrize("variant", protocol.VARIANTS)
def test_each_client_variant_satisfies_the_strict_contract(variant, service_type):
    protocol.validate_sample(
        _sample(variant, service_type=service_type),
        _parameters(service_type), _build(), _cache(service_type))


@pytest.mark.parametrize(
    ("variant", "mutation", "error"),
    [
        ("compatible-rclcppyy", lambda row: row["client_warmed"].update(
            client_authority="cpp"), "identity/topology"),
        ("compatible-rclcppyy", lambda row: row["client_warmed"][
            "backend_marker"].update(backend="cpp"), "authority marker"),
        ("direct-cpp-rclcppyy", lambda row: row["client_warmed"][
            "direct_cpp_proof"].update(response_representation="python"),
         "representation or authority"),
        ("direct-cpp-rclcppyy", lambda row: row["client_warmed"][
            "direct_cpp_proof"]["status_decision"].update(backend="python"),
         "status authority"),
        ("direct-cpp-rclcppyy", lambda row: row["client_warmed"][
            "direct_cpp_proof"].update(native_session_node_count=2),
         "representation or authority"),
        ("direct-cpp-rclcppyy", lambda row: row["client_warmed"][
            "direct_cpp_proof"].update(python_conversion_guard_installed=False),
         "representation or authority"),
        ("direct-cpp-rclcppyy", lambda row: row["client_warmed"][
            "direct_cpp_proof"].update(future_control="shared_future"),
         "representation or authority"),
        ("direct-cpp-rclcppyy", lambda row: row["client_warmed"]["cache"].update(
            source_id="2" * 16), "source identity"),
        ("direct-cpp-rclcppyy", lambda row: row["client_report"].update(
            cpp_request_copies_measured=0), "crossing"),
        ("direct-cpp-rclcppyy", lambda row: row["client_teardown"][
            "direct_cpp_teardown"].update(native_session_closed=False),
         "endpoint disappearance"),
        ("native-python-orchestrated", lambda row: row["client_report"].update(
            python_message_conversions_measured=1), "crossing"),
        ("native-cpp-state-machine", lambda row: row["client_report"].update(
            python_request_crossings_measured=1), "crossing"),
        ("native-python-orchestrated", lambda row: row["client_warmed"]["cache"].update(
            sha256="b" * 64), "differs from warm manifest"),
        ("aot-staged", lambda row: row.update(qos={**protocol.QOS, "depth": 1}),
         "Cyclone service QoS"),
        ("stock-rclpy", lambda row: row["client_warmed"].update(
            server_node="wrong_owner"), "identity/topology"),
        ("stock-rclpy", lambda row: row["client_warmed"].update(
            endpoint_count=2), "identity/topology"),
        ("aot-staged", lambda row: row["client_report"].update(
            response_checksum=1), "count/parity/crossing"),
        ("aot-staged", lambda row: row["client_report"].update(
            pending_requests=1), "count/parity/crossing"),
        ("aot-staged", lambda row: row.update(client_pid=row["server_pid"]),
         "distinct process"),
        ("aot-staged", lambda row: row["server_armed"].update(
            warmup_requests=99), "armed"),
        ("aot-staged", lambda row: row["client_teardown"].update(
            endpoint_disappeared=False), "endpoint disappearance"),
        ("stock-rclpy", lambda row: row["client_report"]["rss_guard"].update(
            within_limit=False), "RSS"),
    ],
)
def test_tainted_client_evidence_is_rejected(variant, mutation, error):
    sample = _sample(variant, 1)
    mutation(sample)
    with pytest.raises(ValueError, match=error):
        protocol.validate_sample(sample, _parameters(), _build(), _cache())


def test_cold_and_warm_client_artifacts_must_match():
    cache = _cache()
    cache["phases"]["warm"]["artifacts"]["native_client"]["sha256"] = "b" * 64
    with pytest.raises(ValueError, match="identity changed"):
        protocol.validate_cache(cache, RMW)


def test_compatible_clients_share_one_activation_only_implementation():
    source = inspect.getsource(worker._run_python_client)
    dispatcher = inspect.getsource(worker._run_client)
    assert "if activate:" in source
    assert "active.enable_cpp_acceleration()" in source
    assert source.count("def call(value: bool)") == 1
    assert "_run_python_client(args, False)" in dispatcher
    assert "_run_python_client(args, True)" in dispatcher


def test_native_orchestration_uses_cpp_requests_without_conversion_bridge():
    source = inspect.getsource(worker._run_native_orchestrated)
    assert "request = client.make_request()" in source
    assert "client.send(request)" in source
    assert "SetBool.Request" not in source
    assert "convert_python_msg_to_cpp" not in source
    assert set(protocol.VARIANTS) == {
        "stock-rclpy",
        "compatible-rclcppyy",
        "direct-cpp-rclcppyy",
        "native-python-orchestrated",
        "native-cpp-state-machine",
        "aot-staged",
    }


def test_direct_cpp_lane_uses_the_production_client_and_future_path():
    source = inspect.getsource(worker._run_direct_cpp_client)
    measured_call = source.split("    def call(value: bool)", 1)[1].split(
        "    def verify_call(value: bool)", 1)[0]
    assert 'profile="direct_cpp", interfaces=(args.service_interface,)' in source
    assert "cppyy.gbl.std_srvs.srv" in source
    assert "service.Request is not cpp_service.Request" in source
    assert "service.Response is not cpp_service.Response" in source
    assert "type(request) is not service.Request" in source
    assert "client.call_async(request)" in source
    assert "type(future) is not Future" in source
    assert "rclpy.spin_until_future_complete(node, future" in source
    assert "type(response) is not service.Response" in source
    assert "type(" not in measured_call
    assert "final.cpp_request_copies - baseline.cpp_request_copies" in source
    assert "convert_python_msg_to_cpp" not in source
    guards = inspect.getsource(worker._install_direct_cpp_boundary_guards)
    assert "bringup.convert_python_msg_to_cpp = forbidden_boundary" in guards
    assert "native_client.convert_python_msg_to_cpp = forbidden_boundary" in guards
    assert "serialization.serialize_message = forbidden_boundary" in guards
    proof = inspect.getsource(worker._direct_cpp_client_proof)
    assert "type(client._native.raw_client)" in proof
    assert "runtime.nodes != [node]" in proof
    assert "runtime.session.nodes != (node._direct_cpp_node,)" in proof


def test_trigger_workload_uses_empty_requests_and_exact_contract():
    class Service:
        class Request:
            pass

    request = worker._make_request(TRIGGER, Service, True)
    assert type(request) is Service.Request
    response = type("Response", (), {"success": True, "message": "triggered"})()
    assert worker._validate_response(TRIGGER, True, response) == 109
    assert protocol.true_requests(3, 4, TRIGGER) == 4
    assert protocol.response_checksum(3, 4, TRIGGER) == 436


def test_every_generated_trigger_marker_records_the_exact_service_type():
    sample = _sample("direct-cpp-rclcppyy", service_type=TRIGGER)
    assert sample["service_type"] == TRIGGER
    for name in (
            "server_ready", "server_armed", "client_warmed", "client_report",
            "server_report", "client_teardown"):
        assert sample[name]["service_type"] == TRIGGER
    assert sample["client_warmed"]["direct_cpp_proof"][
        "client_entity_type"] == "rclcpp::Client<std_srvs::srv::Trigger>"
    assert sample["client_warmed"]["direct_cpp_proof"]["status_decision"][
        "metadata"]["service_type"] == "std_srvs::srv::Trigger"


def test_cpp_state_machine_has_no_python_per_request_callback():
    source = inspect.getsource(worker._state_compile)
    assert "run_measured(uint64_t)" in source
    assert "std::function" not in source
    assert "Python" not in source


def test_runner_rejects_any_server_armed_drift():
    sample = _sample("stock-rclpy")
    armed = sample["server_armed"]
    runner._validate_armed_before_measurement(
        armed,
        token=sample["run_token"],
        server_pid=sample["server_pid"],
        warmup=WARMUP,
    )
    armed["cpu_clock"] = "wall"
    with pytest.raises(RuntimeError, match="armed evidence"):
        runner._validate_armed_before_measurement(
            armed,
            token=sample["run_token"],
            server_pid=sample["server_pid"],
            warmup=WARMUP,
        )


@pytest.mark.parametrize("service_type", protocol.SERVICE_TYPES)
def test_document_and_schema_forbid_claims_and_server_ranking(
        monkeypatch, service_type):
    monkeypatch.setattr(protocol, "environment_metadata", lambda _root: {
        "source": {"commit": "1" * 40, "dirty": True},
        "source_dependencies": {
            "rclcpp_kit": {"commit": "2" * 40, "dirty": False},
        },
    })
    results = [
        _sample(variant, index, service_type)
        for index, variant in enumerate(protocol.VARIANTS)
    ]
    document = protocol.build_document(
        repo_root=REPO_ROOT,
        mode="measurement",
        parameters=_parameters(service_type),
        isolation={
            "fresh_process_pair_per_sample": True,
            "two_process_groups_per_sample": True,
            "unique_service_per_sample": True,
            "one_leased_domain_per_run": True,
            "rotating_variant_order": True,
            "ros_domain_id": 77,
        },
        aot_build=_build(),
        cache=_cache(service_type),
        source_files={"runner": DIGEST},
        results=results,
        failures=[],
        command=["service-client-bench"],
    )
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    comparison = json.dumps(document["comparison"]).lower()
    assert "server_cpu" not in comparison
    assert all(
        "rss" not in metric
        for metrics in document["comparison"]["raw_medians"].values()
        for metric in metrics)
    assert json.loads(protocol.dumps(document)) == document

    schema = json.loads((
        REPO_ROOT / "schemas" / "service-client-benchmark-v1.schema.json"
    ).read_text(encoding="utf-8"))
    jsonschema = pytest.importorskip("jsonschema")
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(document, schema)
    missing_direct_proof = json.loads(json.dumps(document))
    direct = next(
        row for row in missing_direct_proof["results"]
        if row["variant"] == "direct-cpp-rclcppyy")
    del direct["client_warmed"]["direct_cpp_proof"]
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(missing_direct_proof, schema)
    assert schema["properties"]["benchmark"]["properties"][
        "performance_claims_allowed"] == {"const": False}
    assert schema["properties"]["comparison"]["properties"][
        "interpretation_allowed"] == {"const": False}
    assert schema["properties"]["benchmark"]["properties"]["parameters"][
        "properties"]["requested_rmw"] == {"const": RMW}
    assert len(schema["properties"]["results"]["items"]["allOf"]) == 6


@pytest.mark.parametrize("service_type", protocol.SERVICE_TYPES)
def test_all_client_variants_run_against_common_aot_server(tmp_path, service_type):
    interface_name = service_type.rsplit("/", 1)[1].lower()
    output = tmp_path / ("service-client-%s-smoke.json" % interface_name)
    environment = os.environ.copy()
    environment["RMW_IMPLEMENTATION"] = RMW
    process = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_service_client_benchmark.py"),
            "--smoke",
            "--service-interface", service_type,
            "--messages", "4",
            "--warmup-requests", "2",
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
    assert document["benchmark"]["parameters"]["requested_rmw"] == RMW
    assert document["benchmark"]["parameters"]["service_type"] == service_type
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    assert {row["variant"] for row in document["results"]} == set(
        protocol.VARIANTS)
    assert len({
        pid for row in document["results"]
        for pid in (row["server_pid"], row["client_pid"])
    }) == 12
    for row in document["results"]:
        assert row["service_type"] == service_type
        for marker in (
                "server_ready", "server_armed", "client_warmed", "client_report",
                "server_report", "client_teardown"):
            assert row[marker]["service_type"] == service_type
        assert row["client_report"]["total_requests"] == 6
        assert row["server_report"]["total_requests"] == 6
        expected_checksum = 436 if service_type == TRIGGER else 250
        assert row["client_report"]["response_checksum"] == expected_checksum
        assert row["server_report"]["response_checksum"] == expected_checksum
        assert row["client_report"]["pending_requests"] == 0
        assert len(row["client_report"]["latency_ns"]) == 4
        assert row["client_teardown"]["endpoint_disappeared"] is True
    crossings = {
        row["variant"]: (
            row["client_report"]["python_request_crossings_measured"],
            row["client_report"]["python_response_crossings_measured"],
            row["client_report"]["python_message_conversions_measured"],
        )
        for row in document["results"]
    }
    assert crossings == {
        "stock-rclpy": (4, 4, 8),
        "compatible-rclcppyy": (4, 4, 8),
        "direct-cpp-rclcppyy": (4, 4, 0),
        "native-python-orchestrated": (4, 4, 0),
        "native-cpp-state-machine": (0, 0, 0),
        "aot-staged": (0, 0, 0),
    }
    direct = next(
        row for row in document["results"]
        if row["variant"] == "direct-cpp-rclcppyy")
    assert direct["client_report"]["cpp_request_copies_measured"] == 4
