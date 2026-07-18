"""Fail-closed checks for paired fixed-work stress memory evidence."""

from copy import deepcopy

import pytest

from scripts.ci.compare_stress_memory import compare


def _document(profile, slope=1000.0):
    return {
        "schema": "rclcppyy.runtime-stress/v4",
        "architecture": "x86_64",
        "python": "3.12.13",
        "rmw_implementation": "rmw_fastrtps_cpp",
        "profile": profile,
        "source": {
            "product": {"commit": "a" * 40, "dirty": False},
            "suite": {
                "active_commit": "b" * 40,
                "locked_commit": "b" * 40,
                "dirty": False,
            },
        },
        "parameters": {
            "cycles": 100,
            "threads": 4,
            "messages_per_thread": 250,
            "timeout_s": 30.0,
            "repetitions": 20,
            "signal_repetitions": 0,
            "min_duration_s": 0.0,
            "round_period_s": 0.0,
            "signal_settle_s": 0.05,
            "seed": 20260718,
            "max_rss_growth_kib": 0,
        },
        "memory": {
            "warmup_rounds_excluded": 1,
            "post_warmup_entity_cycles": 1900,
            "post_warmup_anonymous_rss_kib_per_1000_cycles": slope,
        },
        "summary": {"result": "pass", "rounds": 20, "entity_cycles": 2000},
        "failures": [],
        "performance_claims_allowed": False,
    }


def _compare(reference, candidate):
    return compare(
        reference,
        candidate,
        max_relative_slope=1.5,
        max_absolute_delta_kib_per_1000_cycles=2048.0,
        minimum_cycles=1000,
    )


def _change_cycles(document):
    document["parameters"]["cycles"] = 99
    document["summary"]["entity_cycles"] = 1980
    document["memory"]["post_warmup_entity_cycles"] = 1881


def test_equal_work_memory_gate_passes_bounded_candidate():
    result = _compare(_document("stock", 1400.0), _document("compatible", 1900.0))

    assert result["result"] == "pass"
    assert result["failures"] == []
    assert result["policy"] == {
        "purpose": "bounded-growth-guard",
        "optimization_objective": False,
        "bounded_memory_tradeoff_allowed": True,
    }
    assert result["comparison"]["relative_slope"] == pytest.approx(1.357142857)
    assert result["post_warmup_entity_cycles"] == 1900
    assert result["performance_claims_allowed"] is False


def test_memory_gate_rejects_relative_and_absolute_regression():
    result = _compare(_document("stock", 1000.0), _document("compatible", 4100.0))

    assert result["result"] == "fail"
    assert len(result["failures"]) == 2


@pytest.mark.parametrize(
    "mutation,match",
    [
        (lambda value: value.update(schema="old/v1"), "does not use"),
        (lambda value: value["source"]["product"].update(dirty=True), "clean"),
        (lambda value: value["source"]["suite"].update(active_commit="c" * 40),
         "product lock"),
        (_change_cycles, "parameter cycles"),
        (lambda value: value["summary"].update(result="fail"), "must pass"),
        (lambda value: value["summary"].update(rounds=19), "every fixed-work round"),
        (lambda value: value["memory"].update(post_warmup_entity_cycles=1800),
         "incomplete entity cycles"),
        (lambda value: value["memory"].update(
            post_warmup_anonymous_rss_kib_per_1000_cycles=None), "slope is missing"),
    ],
)
def test_memory_gate_rejects_unmatched_or_untrusted_evidence(mutation, match):
    reference = _document("stock")
    candidate = deepcopy(_document("compatible"))
    mutation(candidate)

    with pytest.raises(ValueError, match=match):
        _compare(reference, candidate)
