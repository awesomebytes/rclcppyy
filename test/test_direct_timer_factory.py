from _run_helper import format_output, run_helper


def test_direct_timer_uses_reusable_callback_bridge():
    proc = run_helper("_direct_timer_helper.py", timeout=120)
    assert proc.returncode == 0, format_output(proc)
    assert "DIRECT_TIMER_FACTORY_OK" in proc.stdout
