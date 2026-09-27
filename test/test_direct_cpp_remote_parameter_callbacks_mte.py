"""Remote parameter callback dispatch through the public direct MTE."""

from _run_helper import format_output, run_helper


def test_remote_parameter_callbacks_dispatch_and_self_remove_under_direct_mte():
    process = run_helper(
        "_direct_cpp_remote_parameter_callbacks_mte_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_REMOTE_PARAMETER_CALLBACKS_MTE_OK" in process.stdout
