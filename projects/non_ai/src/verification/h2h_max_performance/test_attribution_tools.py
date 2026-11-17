import importlib.util
from pathlib import Path


ROOT = Path(__file__).parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_distribution_percentiles_are_deterministic():
    analyzer = load("analyze_host_attribution")
    result = analyzer.distribution([5, 1, 4, 2, 3])
    assert result == {"mean": 3, "median": 3, "p95": 4, "p99": 4, "max": 5}


def test_cadence_uses_receipt_intervals():
    analyzer = load("analyze_host_attribution")
    result = analyzer.cadence([10.0, 10.1, 10.2])
    assert abs(result["hz"] - 10.0) < 1e-12
    assert abs(result["gap_max_ms"] - 100.0) < 1e-9


def test_counter_reset_never_creates_negative_scheduler_work():
    capture = load("capture_host_attribution")
    assert capture.counter_delta(12, 10) == 2
    assert capture.counter_delta(2, 10) == 0
    assert capture.nanoseconds_to_milliseconds(2_500_000) == 2.5


def test_timestamp_alignment_threshold_and_unique_pid():
    analyzer = load("analyze_host_attribution")
    records = [{"time": "10.0", "wall": "99"}, {"time": "10.2", "wall": "101"}]
    assert analyzer.first_threshold(records, "wall", 100) is records[1]
    assert analyzer.nearest(records, "time", 10.19) is records[1]
    assert analyzer.unique_pid({4: {"cmdline": "/pkg/node"}}, "/pkg/node") == 4
    try:
        analyzer.unique_pid({4: {"cmdline": "/pkg/node"}, 5: {"cmdline": "/pkg/node"}}, "/pkg/node")
    except ValueError:
        pass
    else:
        raise AssertionError("ambiguous executable ownership must fail closed")
