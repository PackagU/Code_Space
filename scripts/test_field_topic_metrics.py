#!/usr/bin/env python3
"""Pure offline tests for field topic interval metrics."""

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "scripts/field_topic_metrics.py"


def load_module():
    spec = importlib.util.spec_from_file_location("field_topic_metrics", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    mod = load_module()
    series = mod.ArrivalSeries()
    for index in range(11):
        arrival = 1_000_000_000 + index * 100_000_000
        series.add(arrival, arrival - 5_000_000)
    report = series.report()
    assert report["count"] == 11
    assert abs(report["measured_rate_hz"] - 10.0) < 1e-9
    assert report["gap_ms_p99"] == 100.0
    assert report["age_ms_p95"] == 5.0
    assert report["header_regressions"] == 0

    series.add(2_100_000_000, 500_000_000)
    assert series.report()["header_regressions"] == 1
    try:
        series.add(2_000_000_000)
    except ValueError:
        pass
    else:
        raise AssertionError("arrival clock regression accepted")
    print("P07 field topic metrics tests passed")


if __name__ == "__main__":
    main()
