"""Focused exact-C++ local parameter mirror proof."""

from pathlib import Path

from _run_helper import format_output, run_helper


DIRECT_CPP = Path(__file__).resolve().parents[1] / "rclcppyy" / "direct_cpp.py"


def test_direct_parameter_cache_capacity_coherence_and_failure_isolation():
    process = run_helper("_direct_parameter_cache_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    for marker in (
        "DIRECT_PARAMETER_CACHE_CAPACITY_OK",
        "DIRECT_PARAMETER_CACHE_TYPE_REPLACEMENT_OK",
        "DIRECT_PARAMETER_CACHE_MUTATION_SNAPSHOTS_OK",
        "DIRECT_PARAMETER_CACHE_OVERRIDE_OK",
        "DIRECT_PARAMETER_CACHE_FAILURE_ISOLATED_OK",
        "DIRECT_PARAMETER_CACHE_DISABLED_OK",
        "DIRECT_PARAMETER_CACHE_INVALID_CONFIG_OK",
        "DIRECT_PARAMETER_CACHE_RESTART_OK",
    ):
        assert marker in process.stdout


def test_default_parameter_cache_hit_has_no_stats_lock_or_cppyy_work():
    source = DIRECT_CPP.read_text(encoding="utf-8")
    get_source = source[
        source.index("    def get_parameter(self, name):"):
        source.index("    def get_parameters(self, names):")
    ]
    hit_source = get_source[:get_source.index("        except KeyError:")]
    assert "return self._direct_cpp_parameter_cache[name]" in hit_source
    assert "stats" not in hit_source
    assert "lock" not in hit_source
    assert "native_parameters" not in hit_source
    assert "cppyy" not in hit_source
