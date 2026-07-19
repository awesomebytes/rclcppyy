"""Correctness and failure-boundary proofs for the direct-C++ first slice."""

import os
from pathlib import Path
import subprocess
import sys

from _run_helper import format_output, run_helper


HERE = Path(__file__).resolve().parent


def run_custom_interface_helper(
    setup, helper, *arguments, timeout=360, cache_home=None
):
    environment = os.environ.copy()
    if cache_home is not None:
        environment["XDG_CACHE_HOME"] = str(cache_home)
    return subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"; shift; exec "$@"',
            "direct-action-helper",
            str(setup),
            sys.executable,
            str(HERE / helper),
            *map(str, arguments),
        ],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
        env=environment,
    )


def test_direct_cpp_source_compatible_pubsub_and_lifetime():
    process = run_helper("_direct_cpp_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CONSTRUCTORS_OK" in process.stdout
    assert "DIRECT_CPP_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_FACADES_OK" in process.stdout
    assert "DIRECT_CPP_MESSAGES_OK" in process.stdout
    assert "DIRECT_CPP_NAMESPACED_REMAP_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_OK" in process.stdout
    assert "DIRECT_CPP_TEARDOWN_OK" in process.stdout


def test_direct_cpp_public_single_threaded_executor_owns_exact_cpp_nodes():
    process = run_helper("_direct_cpp_executor_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_OWNERSHIP_OK" in process.stdout
    assert "DIRECT_CPP_GLOBAL_SPIN_ONCE_PARKING_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_CPP_DATA_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_TRANSFER_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_CONTEXT_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_SHUTDOWN_WAKE_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_NODE_DESTROY_OK" in process.stdout
    assert "DIRECT_CPP_PUBLIC_EXECUTOR_TEARDOWN_OK" in process.stdout


def test_direct_cpp_callback_groups_bind_native_entities():
    process = run_helper("_direct_cpp_callback_group_helper.py", timeout=360)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CALLBACK_GROUP_NATIVE_OWNERSHIP_OK" in process.stdout
    assert "DIRECT_CPP_CALLBACK_GROUP_CROSS_NODE_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_CALLBACK_GROUP_EXECUTION_OK" in process.stdout
    assert "DIRECT_CPP_CALLBACK_GROUP_TEARDOWN_OK" in process.stdout


def test_direct_cpp_graph_queries_use_native_rclcpp_authority():
    process = run_helper("_direct_cpp_graph_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_GRAPH_CPP_AUTHORITY_OK" in process.stdout
    assert "DIRECT_CPP_GRAPH_RCLPY_SHAPE_OK" in process.stdout
    assert "DIRECT_CPP_GRAPH_INVALID_NAME_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_GRAPH_TEARDOWN_OK" in process.stdout


def test_direct_cpp_qos_profiles_reach_native_endpoints_without_message_conversion():
    process = run_helper("_direct_cpp_qos_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_QOS_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_QOS_EXPLICIT_ENDPOINT_OK" in process.stdout
    assert "DIRECT_CPP_QOS_SENSOR_ENDPOINT_OK" in process.stdout
    assert "DIRECT_CPP_QOS_TEARDOWN_OK" in process.stdout


def test_direct_cpp_opt_in_subscription_shared_lease():
    process = run_helper(
        "_direct_cpp_subscription_lease_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_SUBSCRIPTION_LEASE_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_SUBSCRIPTION_LEASE_IDEMPOTENCE_OK" in process.stdout
    assert "DIRECT_CPP_SUBSCRIPTION_LEASE_MESSAGES_OK" in process.stdout
    assert "DIRECT_CPP_SUBSCRIPTION_LEASE_RETAINED_OK" in process.stdout
    assert "DIRECT_CPP_SUBSCRIPTION_LEASE_REINIT_OK" in process.stdout


def test_direct_cpp_message_info_copy_and_lease_with_stock_and_aot(tmp_path):
    source = HERE.parent / "scripts" / "benchmarks" / "relay_boundary_aot"
    build_directory = tmp_path / "message-info-aot"
    configure = [
        "cmake",
        "-S", str(source),
        "-B", str(build_directory),
        "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    if os.environ.get("CONDA_PREFIX"):
        configure.append("-DCMAKE_PREFIX_PATH=" + os.environ["CONDA_PREFIX"])
    configured = subprocess.run(
        configure,
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=os.environ.copy(),
    )
    assert configured.returncode == 0, format_output(configured)
    built = subprocess.run(
        [
            "cmake", "--build", str(build_directory),
            "--target", "relay_boundary_aot",
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=os.environ.copy(),
    )
    assert built.returncode == 0, format_output(built)
    executable = build_directory / "relay_boundary_aot"
    assert executable.is_file() and os.access(executable, os.X_OK)

    for mode in ("copy", "lease"):
        process = run_helper(
            "_direct_cpp_message_info_helper.py",
            mode,
            executable,
            timeout=360,
        )
        assert process.returncode == 0, format_output(process)
        assert "DIRECT_CPP_MESSAGE_INFO_FAIL_CLOSED_OK" in process.stdout
        assert "DIRECT_CPP_MESSAGE_INFO_ONE_ARG_OK" in process.stdout
        assert "DIRECT_CPP_MESSAGE_INFO_STOCK_INTEROP_OK" in process.stdout
        assert "DIRECT_CPP_MESSAGE_INFO_AOT_INTEROP_OK" in process.stdout
        assert "DIRECT_CPP_MESSAGE_INFO_EXCEPTION_OK" in process.stdout
        assert "DIRECT_CPP_MESSAGE_INFO_CONTRACT_OK" in process.stdout
        assert "DIRECT_CPP_MESSAGE_INFO_RETAINED_TEARDOWN_OK" in process.stdout


def test_direct_cpp_registered_nested_message_uses_exact_cpp_lease():
    process = run_helper(
        "_direct_cpp_generic_message_helper.py", timeout=360)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_GENERIC_INTERFACE_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_CONSTRUCTORS_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_NESTED_LEASE_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_EVIDENCE_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_RETAINED_TEARDOWN_OK" in process.stdout


def test_direct_cpp_uninstalled_interface_fails_closed():
    process = run_helper(
        "_direct_cpp_missing_interface_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_MISSING_INTERFACE_FAIL_CLOSED_OK" in process.stdout


def test_direct_cpp_uninstalled_service_fails_before_alias_mutation():
    process = run_helper(
        "_direct_cpp_missing_service_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_MISSING_SERVICE_TRANSACTIONAL_OK" in process.stdout


def test_direct_cpp_uninstalled_action_fails_before_alias_mutation():
    process = run_helper(
        "_direct_cpp_missing_action_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_MISSING_ACTION_TRANSACTIONAL_OK" in process.stdout


def test_direct_cpp_native_timer_and_bounded_spin():
    process = run_helper("_direct_cpp_timer_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_TIMER_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_TIMER_CONTROL_OK" in process.stdout
    assert "DIRECT_CPP_TIMER_EXCEPTION_OK" in process.stdout
    assert "DIRECT_CPP_SPIN_INTERRUPT_OK" in process.stdout


def test_direct_cpp_setbool_service_client_and_future_control():
    process = run_helper("_direct_cpp_service_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_SERVICE_CONSTRUCTORS_OK" in process.stdout
    assert "DIRECT_CPP_SERVICE_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_SERVICE_CALL_OK" in process.stdout
    assert "DIRECT_CPP_SERVICE_EXCEPTION_OK" in process.stdout
    assert "DIRECT_CPP_SERVICE_FUTURE_CONTROL_OK" in process.stdout
    assert "DIRECT_CPP_SERVICE_EVIDENCE_OK" in process.stdout
    assert "DIRECT_CPP_SERVICE_REINIT_TEARDOWN_OK" in process.stdout


def test_direct_cpp_registered_trigger_service_and_stock_interop():
    process = run_helper(
        "_direct_cpp_trigger_service_helper.py", timeout=360)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_TRIGGER_CONSTRUCTORS_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_CALL_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_EXCEPTION_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_STOCK_INTEROP_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_EVIDENCE_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_RETAINED_REINIT_TEARDOWN_OK" in process.stdout


def test_direct_cpp_registered_trigger_interoperates_with_aot_rclcpp(tmp_path):
    build = subprocess.run(
        [
            "bash",
            str(HERE / "build_direct_trigger_aot_peer.sh"),
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=os.environ.copy(),
    )
    assert build.returncode == 0, format_output(build)
    process = run_helper(
        "_direct_cpp_trigger_aot_helper.py",
        str(tmp_path / "direct_trigger_aot_peer"),
        timeout=360,
    )
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_TRIGGER_AOT_BIDIRECTIONAL_OK" in process.stdout
    assert "DIRECT_CPP_TRIGGER_AOT_RETAINED_TEARDOWN_OK" in process.stdout


def test_direct_cpp_lookup_transform_action_client_and_cpp_envelopes():
    process = run_helper("_direct_cpp_action_helper.py", timeout=360)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_ACTION_CONSTRUCTORS_OK" in process.stdout
    assert "DIRECT_CPP_ACTION_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_ACTION_SUCCESS_OK" in process.stdout
    assert "DIRECT_CPP_ACTION_REJECTION_OK" in process.stdout
    assert "DIRECT_CPP_ACTION_CANCEL_OK" in process.stdout
    assert "DIRECT_CPP_ACTION_EVIDENCE_OK" in process.stdout
    assert "DIRECT_CPP_ACTION_REINIT_TEARDOWN_OK" in process.stdout


def test_direct_cpp_registered_custom_action_uses_aot_cpp_server(tmp_path):
    fixture = HERE / "fixtures" / "custom_interfaces"
    work = tmp_path / "custom-action"
    build = subprocess.run(
        [
            "colcon",
            "--log-base", str(work / "log"),
            "build",
            "--base-paths", str(fixture),
            "--build-base", str(work / "build"),
            "--install-base", str(work / "install"),
            "--packages-select",
            "rclcppyy_test_interfaces",
            "rclcppyy_test_peer",
            "--cmake-args", "-DCMAKE_BUILD_TYPE=Release",
        ],
        capture_output=True,
        text=True,
        timeout=240,
        check=False,
        env=os.environ.copy(),
    )
    assert build.returncode == 0, format_output(build)
    setup = work / "install" / "setup.bash"
    peer = (
        work / "install" / "rclcppyy_test_peer" / "lib"
        / "rclcppyy_test_peer" / "interop_peer"
    )
    assert setup.is_file()
    assert peer.is_file() and os.access(peer, os.X_OK)

    stale = run_custom_interface_helper(
        setup, "_direct_cpp_custom_action_stale_import_helper.py")
    assert stale.returncode == 0, format_output(stale)
    assert "DIRECT_CPP_CUSTOM_ACTION_STALE_IMPORT_OK" in stale.stdout

    unregistered = run_custom_interface_helper(
        setup, "_direct_cpp_unregistered_action_helper.py")
    assert unregistered.returncode == 0, format_output(unregistered)
    assert "DIRECT_CPP_UNREGISTERED_ACTION_FAIL_CLOSED_OK" in \
        unregistered.stdout

    process = run_custom_interface_helper(
        setup,
        "_direct_cpp_generic_action_helper.py",
        peer,
        cache_home=work / "xdg-cache",
    )
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_GENERIC_ACTION_ALIASES_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_ACTION_SERVER_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_ACTION_RUNPATH_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_ACTION_AOT_INTEROP_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_ACTION_EVIDENCE_OK" in process.stdout
    assert "DIRECT_CPP_GENERIC_ACTION_RETAINED_TEARDOWN_OK" in process.stdout


def test_direct_cpp_rejects_supported_message_imported_before_activation():
    process = run_helper("_direct_cpp_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_STALE_IMPORT_OK" in process.stdout


def test_direct_cpp_rejects_setbool_imported_before_activation():
    process = run_helper("_direct_cpp_service_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_SERVICE_STALE_IMPORT_OK" in process.stdout


def test_direct_cpp_rejects_trigger_imported_before_activation():
    process = run_helper(
        "_direct_cpp_trigger_service_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_TRIGGER_STALE_IMPORT_OK" in process.stdout


def test_direct_cpp_rejects_lookup_transform_imported_before_activation():
    process = run_helper("_direct_cpp_action_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_ACTION_STALE_IMPORT_OK" in process.stdout


def test_direct_cpp_profile_is_jazzy_cyclone_only():
    assert os.environ.get("ROS_DISTRO") == "jazzy"
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp"
