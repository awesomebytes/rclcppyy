"""Committed proof battery for Slice 2.5c (PLAN-mte-unlock.md): safe
node/entity destruction under concurrent MultiThreadedExecutor dispatch.

Runs through the public MultiThreadedExecutor constructor (Slice 3,
PLAN-mte-unlock.md un-fail-close). See each helper's own docstring for the
exact scenario and why it matters; this file only wires them to pytest and
states the expected observable each one must clear.
"""
from _run_helper import format_output, run_helper


def test_self_destroy_subscription_and_node_scenarios():
    """Scenario A (subscription self-destroy) and Scenario B (node
    self-destroy with a genuinely in-flight peer -- the crasher: pre-Slice-
    2.5 this crashed by iteration 7-10 of 20). N >= 50 each; the promoted,
    committed form of the scratchpad reference probe used as the stop-gate
    throughout this engagement."""
    process = run_helper("_self_destroy_scenario_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "SELF_DESTROY_SUBSCRIPTION_ALL_OK" in process.stdout, format_output(process)
    assert "SELF_DESTROY_NODE_ALL_OK" in process.stdout, format_output(process)
    assert "SELF_DESTROY_SCENARIO_ALL_OK" in process.stdout, format_output(process)


def test_shutdown_under_spin_the_v1_mandatory_shape():
    """A peer is genuinely mid-trampoline when executor.shutdown() fires
    from another thread. Expected: shutdown() returns True, the spin
    thread joins, spin_errors == []. N >= 50."""
    process = run_helper("_shutdown_under_spin_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "SHUTDOWN_UNDER_SPIN_ALL_OK" in process.stdout, format_output(process)


def test_timer_and_service_self_destroy_under_mte():
    """Extends the self-destroy proof to timers and services -- the other
    native-owned-lifetime entity kinds the suite fixed. N >= 50 each."""
    process = run_helper("_timer_service_self_destroy_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "TIMER_SELF_DESTROY_ALL_OK" in process.stdout, format_output(process)
    assert "SERVICE_SELF_DESTROY_ALL_OK" in process.stdout, format_output(process)
    assert (
        "TIMER_SERVICE_SELF_DESTROY_ALL_OK" in process.stdout
    ), format_output(process)


def test_cross_thread_destroy_takes_the_synchronous_quiescence_path():
    """An external (non-dispatch) thread destroys a subscription while its
    own callback is genuinely in flight -- destroy_subscription must take
    the synchronous quiescence-wait path (blocking until the callback
    returns) rather than deferring, since the calling thread is not itself
    dispatching for this node."""
    process = run_helper("_cross_thread_destroy_helper.py", timeout=150)
    assert process.returncode == 0, format_output(process)
    assert "CROSS_THREAD_DESTROY_ALL_OK" in process.stdout, format_output(process)


def test_action_under_live_mte_confirm():
    """Action servers need no native-owned-lifetime fix -- proven safe by
    dispatch model (creator-thread-only decision dispatch, depth-tracked
    deferred close). This is the product-level confirm: destroying the
    node while an action goal's execute callback is genuinely in flight,
    under a live MultiThreadedExecutor, must not crash. One iteration
    suffices (a confirm/guard, not a timing-dependent proof)."""
    process = run_helper("_action_under_mte_confirm_helper.py", timeout=90)
    assert process.returncode == 0, format_output(process)
    assert "ACTION_UNDER_MTE_CONFIRM_OK" in process.stdout, format_output(process)


def test_destroy_does_not_sever_the_callable_white_box():
    """Regression guard: destroy_subscription must route through the
    suite's C++-owned ManagedSubscription rather than nulling the callable
    directly -- the eager-severing pattern this whole effort fixed. If a
    future edit ever drops the managed= wiring, this fails loud instead of
    silently reintroducing the UAF class."""
    process = run_helper("_destroy_no_sever_regression_helper.py", timeout=60)
    assert process.returncode == 0, format_output(process)
    assert "DESTROY_NO_SEVER_REGRESSION_OK" in process.stdout, format_output(process)


def test_gc_after_gated_destroy_stays_clean():
    """Product-level counterpart of the suite's gc_after_close/
    gc_after_quiescent_close proofs: every product destroy path quiesce-
    gates before closing (the racy variant is unreachable through the
    gated API -- confirmed by recon), so this closes cross-thread while a
    callback is in flight (genuinely blocking on quiescence), then drops
    every reference and forces gc.collect(). N >= 50, must stay clean."""
    process = run_helper("_gc_after_destroy_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "GC_AFTER_DESTROY_ALL_OK" in process.stdout, format_output(process)


def test_parameter_callback_teardown_under_live_mte():
    """Slice 3 addition (PLAN-mte-unlock.md): the product-level counterpart
    of the suite's parameter-callback teardown-under-worker-dispatch proof
    (Slice 2.5a4), through the public rclpy-style API
    (DirectNode.add_on_set_parameters_callback) and a real, publicly-
    constructed MultiThreadedExecutor. Self-removes the callback from
    within its own dispatch on a worker thread genuinely calling
    set_parameters, drops every reference, forces gc.collect(). N=50."""
    process = run_helper("_param_teardown_under_mte_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "PARAM_TEARDOWN_MTE_ALL_OK" in process.stdout, format_output(process)


def test_marshal_window_stress_end_to_end_through_the_product():
    """The discriminating proof, end to end through the full product
    stack: a worker mid-marshal (committed to dispatch, not yet in the
    containment shim) is invisible to the product's own in-flight counter
    by construction -- destroy_subscription's _quiesce_or_raise reads
    in_flight == 0 and proceeds immediately, providing zero protection.
    Confirms the suite's reaper (Slice 2.5a2/2.5a3), not the product's
    gating, is what makes this safe. N >= 50."""
    process = run_helper(
        "_marshal_window_stress_product_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "MARSHAL_WINDOW_PRODUCT_ALL_OK" in process.stdout, format_output(process)
