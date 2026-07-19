"""Developer and dedicated-CI wiring for the two newest benchmark harnesses."""

from pathlib import Path
import tomllib


ROOT = Path(__file__).resolve().parents[1]


def test_parameter_and_action_server_benchmarks_are_developer_tasks_and_linted():
    configuration = tomllib.loads(
        (ROOT / "pixi.toml").read_text(encoding="utf-8"))
    tasks = configuration["tasks"]
    assert tasks["action-server-bench"] == (
        "python scripts/benchmarks/run_action_server_benchmark.py")
    assert tasks["local-parameter-bench"] == (
        "python scripts/benchmarks/run_local_parameter_benchmark.py")
    assert tasks["local-parameter-smoke"].endswith(
        "run_local_parameter_benchmark.py --smoke")
    for path in (
        "scripts/benchmarks/_action_server_protocol.py",
        "scripts/benchmarks/action_server_worker.py",
        "scripts/benchmarks/run_action_server_benchmark.py",
        "scripts/benchmarks/_local_parameter_benchmark_protocol.py",
        "scripts/benchmarks/local_parameter_benchmark_worker.py",
        "scripts/benchmarks/run_local_parameter_benchmark.py",
    ):
        assert path in tasks["lint"].split()


def test_full_parameter_and_action_server_runs_are_dedicated_only():
    scheduled = (ROOT / ".github" / "workflows" / "scheduled.yml").read_text(
        encoding="utf-8")
    pull_request = (ROOT / ".github" / "workflows" / "ci.yml").read_text(
        encoding="utf-8")
    assert "pixi run action-server-bench" in scheduled
    assert "pixi run local-parameter-bench" in scheduled
    assert "pixi run action-server-bench" not in pull_request
    assert "pixi run local-parameter-bench" not in pull_request
