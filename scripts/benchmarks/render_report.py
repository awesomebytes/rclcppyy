#!/usr/bin/env python3
"""Render validated benchmark-v2 JSON as a reviewable Markdown report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from _result_schema import validate_document


def _cell(value) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        value = "%.3f" % value
    return str(value).replace("|", "\\|").replace("\n", " ")


def _nested(mapping, *keys):
    value = mapping
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _backend_route(row) -> str:
    publisher = _nested(row, "publisher_backend", "backend")
    subscriber = _nested(row, "subscriber_backend", "backend")
    if publisher is None or subscriber is None:
        expected = row.get("expected_backends", {})
        publisher = publisher or expected.get("publisher")
        subscriber = subscriber or expected.get("subscriber")
    return "%s -> %s" % (_cell(publisher), _cell(subscriber))


def render(document: dict) -> str:
    """Return deterministic Markdown for one validated result document."""
    validate_document(document)
    benchmark = document["benchmark"]
    environment = document["environment"]
    host = environment.get("host", {})
    ros = environment.get("ros", {})
    source = environment.get("source", {})
    claims_allowed = benchmark["performance_claims_allowed"]

    lines = [
        "# Benchmark report",
        "",
        "> %s" % (
            "Measurement artifact. Performance claims require repeated controlled "
            "runs and explicit review."
            if claims_allowed else
            "Smoke artifact. Performance claims are forbidden; these timings only "
            "validate execution and evidence collection."
        ),
        "",
        "## Run",
        "",
        "| Field | Value |",
        "| --- | --- |",
        "| Schema | `%s` |" % _cell(document["schema"]),
        "| Benchmark | `%s` |" % _cell(benchmark["name"]),
        "| Mode | `%s` |" % _cell(benchmark["mode"]),
        "| Generated | `%s` |" % _cell(document["generated_at"]),
        "| Commit | `%s` |" % _cell(source.get("commit")),
        "| Dirty source | `%s` |" % _cell(source.get("dirty")),
        "| Architecture | `%s` |" % _cell(host.get("architecture")),
        "| CPU | %s |" % _cell(host.get("cpu_model")),
        "| ROS distribution | `%s` |" % _cell(ros.get("distribution")),
        "| RMW | `%s` |" % _cell(ros.get("rmw_implementation")),
        "",
        "## Results",
        "",
    ]
    if document["results"]:
        lines.extend([
            "| Backend | Workload | Target Hz | Payload B | Received | Dropped | "
            "Effective Hz | Pub CPU mean | Sub CPU mean | Latency mean us | "
            "p95 us | p99 us | Route |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
            "---: | ---: | ---: | --- |",
        ])
        for row in document["results"]:
            values = (
                row.get("backend"),
                row.get("workload"),
                row.get("target_rate_hz"),
                row.get("payload_bytes"),
                _nested(row, "messages", "received"),
                _nested(row, "messages", "dropped"),
                _nested(row, "messages", "effective_rate_hz"),
                _nested(row, "cpu_pct", "publisher", "mean"),
                _nested(row, "cpu_pct", "subscriber", "mean"),
                _nested(row, "latency_us", "mean"),
                _nested(row, "latency_us", "p95"),
                _nested(row, "latency_us", "p99"),
                _backend_route(row),
            )
            lines.append("| %s |" % " | ".join(_cell(value) for value in values))
    else:
        lines.append("No successful result rows were recorded.")

    lines.extend(["", "## Failures", ""])
    if document["failures"]:
        for failure in document["failures"]:
            case_id = failure.get("case_id", "unknown")
            error = failure.get("error", failure)
            lines.append("- `%s`: %s" % (_cell(case_id), _cell(error)))
    else:
        lines.append("None.")

    command = " ".join(_cell(value) for value in document.get("command", []))
    lines.extend([
        "",
        "## Reproduction",
        "",
        "```text",
        command,
        "```",
        "",
        "Statistics: %s" % _cell(benchmark.get("statistics", {})),
        "",
    ])
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    document = json.loads(args.input.read_text(encoding="utf-8"))
    report = render(document)
    if args.output is None:
        print(report, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_name(args.output.name + ".tmp")
        temporary.write_text(report, encoding="utf-8")
        temporary.replace(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
