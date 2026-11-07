"""Smoke test de co-simulación baseline."""

import pytest

from pure_pursuit.twin.sim import Sim


def test_baseline_short_run():
    sim = Sim(7, preset="baseline", max_time_s=20.0)
    result = sim.run()
    m = result.metrics
    assert m["frames_processed"] >= 5
    dist = m["distance_forward_mm"] + m["distance_backward_mm"]
    assert dist > 100.0
    assert "total_time_s" in m
    assert "mean_corner_time_s" in m
    assert "finished" in m
    assert result.trace
    assert any("ang=" in (r.get("ack") or "") for r in result.trace)
