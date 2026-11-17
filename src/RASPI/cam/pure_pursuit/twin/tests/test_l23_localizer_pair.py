"""Reject a false lateral echo without inventing a pose correction."""
import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit.localizer import Localizer


def _sample(tc, heading):
    loc = Localizer({}, None, 1000.0, 0.0, heading)
    left = loc._expect(loc.x, C.SENSOR_MOUNTS["tof_left"], "tof")[0]
    right = loc._expect(loc.x, C.SENSOR_MOUNTS["tof_right"], "tof")[0]
    # After an unobserved corner the lateral covariance can be large.
    loc.P[:2, :2] = np.eye(2) * 300**2
    # Same transient seen in s7: left outside its trusted range, short right echo.
    ack = f"ACK:V2,tc={tc},px=0,py=0,yaw=0,tL={left+80},tR={right-200}"
    loc.update(ack)
    return loc


@pytest.mark.parametrize("heading", [0.0, 30.0])
@pytest.mark.parametrize("tc", [3, 4])
def test_incompatible_pair_does_not_move_pose(monkeypatch, heading, tc):
    monkeypatch.setattr(C, "US_DESDE_TOF", True)
    monkeypatch.setattr(C, "LOC_L23_PAIR_TOL_MM", None)
    unguarded = _sample(tc, heading)
    monkeypatch.setattr(C, "LOC_L23_PAIR_TOL_MM", 100)
    guarded = _sample(tc, heading)
    assert abs(unguarded.x[0] - 1000) > 100
    assert np.allclose(guarded.x[:2], [1000, 0])


@pytest.mark.parametrize("tc", [0, 2, 12])
def test_pair_guard_does_not_change_other_phases(monkeypatch, tc):
    monkeypatch.setattr(C, "LOC_L23_PAIR_TOL_MM", None)
    original = _sample(tc, 0)
    monkeypatch.setattr(C, "LOC_L23_PAIR_TOL_MM", 100)
    guarded = _sample(tc, 0)
    assert np.allclose(guarded.x, original.x)
    assert np.allclose(guarded.P, original.P)


def test_consistent_pair_still_corrects_lateral_bias(monkeypatch):
    monkeypatch.setattr(C, "LOC_L23_PAIR_TOL_MM", 100)
    truth = Localizer({}, None, 1000, 0, 0)
    loc = Localizer({}, None, 1050, 0, 0)
    left = truth._expect(truth.x, C.SENSOR_MOUNTS["tof_left"], "tof")[0]
    right = truth._expect(truth.x, C.SENSOR_MOUNTS["tof_right"], "tof")[0]
    loc.update(f"ACK:V2,tc=4,px=0,py=0,yaw=0,tL={left},tR={right}")
    assert abs(loc.x[0] - 1000) < 20
    assert loc.n_used > 0
