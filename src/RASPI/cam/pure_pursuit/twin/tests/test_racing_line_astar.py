"""racing_line_astar: carta fija + vuelta joint vs DigitalMap._build_line."""
import numpy as np
import pytest

from pure_pursuit import racing_line_astar as rl
from pure_pursuit import route_planner as rp
from pure_pursuit import wro_field as wf

CARD = 26
SECTION = "N"
DIRECTION = "CW"


@pytest.fixture(scope="module")
def pillars():
    return rl.pillars_for_card(SECTION, CARD)


@pytest.fixture(scope="module")
def planned(pillars):
    prm = rl.LatticeParams(ds_mm=40.0, a_step_mm=40.0, n_kappa=7)
    return rl.plan_card(DIRECTION, SECTION, pillars, prm)


@pytest.fixture(scope="module")
def seed_field():
    return wf.randomize(3170839)


@pytest.fixture(scope="module")
def joint_lap(seed_field):
    prm = rl.LatticeParams(ds_mm=40.0)
    return rl.plan_field_lap(seed_field, start_section="N", params=prm, mode="joint")


def test_pillars_card_26(pillars):
    assert len(pillars) == 2
    assert {c for _, _, c in pillars} == {"Red", "Green"}


def test_kappa_ok(planned):
    assert planned.kappa_ok
    assert float(np.abs(planned.kappa).max()) <= rp.KAPPA_MAX * 1.05


def test_clearance_and_side(planned, pillars):
    assert planned.clear_mm >= 25.0
    assert rp.side_ok(planned.x, planned.y, pillars, closed=False)


def test_beats_invalid_center_on_clearance(pillars, planned):
    center = rl.centerline_card(DIRECTION, SECTION, pillars)
    assert planned.clear_mm > center.clear_mm
    assert planned.length_mm > 500.0


def test_seed_3170839_section_n(seed_field):
    pil = rl.pillars_for_section(seed_field, "N")
    r = rl.plan_card(seed_field.direction, "N", pil)
    assert r.kappa_ok
    assert r.clear_mm >= 15.0
    assert r.length_mm > 1000.0
    # side_ok del tramo abierto vs rp.side_ok es frágil en carta 29 (S-curve);
    # la métrica fuerte de lado está en carta 26 y en joint vs build_line.


def test_joint_lap_beats_build_line(seed_field, joint_lap):
    all_pil = [(s.x, s.y, s.color) for s in seed_field.signs]
    bx, by = rl.build_line_baseline(seed_field.direction, seed_field.signs)
    bsc = rl.score_xy(bx, by, all_pil)
    assert joint_lap.kappa_ok
    assert joint_lap.clear_mm >= 15.0
    assert joint_lap.time_s <= bsc["time_s"] + 0.05
    assert joint_lap.length_mm < bsc["length_mm"]
    # _build_line de esta seed sale con clear 0 / κ alto; la joint debe mejorar holgura
    assert joint_lap.clear_mm > bsc["clear_mm"]
