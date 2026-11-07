"""route_planner: lado de paso, holgura, curvatura, continuidad y comparación
contra la línea actual de DigitalMap._build_line (tableros de wro_field.randomize,
las 36 cartas)."""
import math

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit import digital_map as dmod
from pure_pursuit import route_planner as rp
from pure_pursuit import wro_field as wf

SEEDS = (1, 2, 3, 4, 5, 6, 7, 8)


def _board(seed):
    f = wf.randomize(seed)
    return f, [(s.x, s.y, s.color) for s in f.signs]


@pytest.fixture(scope="module")
def plans():
    out = {}
    for sd in SEEDS:
        f, pil = _board(sd)
        out[sd] = (f, pil, rp.plan_lap(f.direction, pil))
    return out


def test_flag_off_by_default():
    assert C.ROUTE_PLANNER is False


def test_side_of_pass_respected(plans):
    for sd, (f, pil, r) in plans.items():
        assert rp.side_ok(r.x, r.y, pil), sd


def test_clearance_footprint(plans):
    for sd, (f, pil, r) in plans.items():
        c = rp.clearances(r.x, r.y, pil)
        assert c["pillar"] >= 20.0, (sd, c)
        assert c["island"] >= 40.0, (sd, c)
        assert c["wall"] >= 20.0, (sd, c)


def test_curvature_limit(plans):
    for sd, (f, pil, r) in plans.items():
        assert r.kappa_ok, sd
        assert np.abs(r.kappa).max() <= rp.KAPPA_MAX, (sd, np.abs(r.kappa).max())
        # estimador independiente (rumbo en +-40 mm), con holgura por el muestreo
        assert rp.path_stats(r.x, r.y)["kmax"] <= rp.KAPPA_MAX * 1.2, sd


def test_continuity_and_closure(plans):
    for sd, (f, pil, r) in plans.items():
        p = r.xy()
        seg = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)   # incluye el cierre
        assert seg.max() < 1.15 * rp.PlannerParams().out_ds_mm, sd
        hd = np.radians(r.heading_deg)
        dpsi = np.abs((np.diff(np.append(hd, hd[0])) + math.pi) % (2 * math.pi) - math.pi)
        assert dpsi.max() <= seg.max() * rp.KAPPA_MAX * 1.3, (sd, dpsi.max())   # G1
        dk = np.abs(np.diff(np.append(r.kappa, r.kappa[0])))
        assert dk.max() <= 0.6 * rp.KAPPA_MAX, (sd, dk.max())                    # sin saltos de k


def test_start_station_and_direction(plans):
    f, pil, _ = plans[1]
    r = rp.plan_lap(f.direction, pil, start_section="W", start_along_mm=1000.0)
    al, lat = rp.section_along("W", f.direction, r.x[0], r.y[0])
    assert abs(al - 1000.0) < 50.0 and abs(lat) < 400.0
    assert r.s[0] == 0.0 and np.all(np.diff(r.s) > 0)


def test_empty_board_is_symmetric_ccw_cw():
    a = rp.plan_lap("CW", [])
    b = rp.plan_lap("CCW", [])
    assert abs(a.length_mm - b.length_mm) < 1.0
    assert 4400 < a.length_mm < 5200       # ~4.7 m: apex cerca de la isla


def _baseline(direction, signs, a0=1200.0):
    """Línea de _build_line encadenada (una instantánea por recta, ver docstring)."""
    dm = dmod.DigitalMap()
    if dm.direction is None:
        dm.set_direction(direction)
    dm.confirmed = {(s.section, s.seat): s.color for s in signs}
    dm.in_stall = False
    dm._aligned = True
    order = list(wf.SECTIONS_CW) if direction == "CW" else list(reversed(wf.SECTIONS_CW))
    lats = [0.0] * 4
    pieces = [None] * 4
    for _ in range(2):
        for k, sec in enumerate(order):
            nxt = order[(k + 1) % 4]
            x, y, h = dmod._lane_pose(sec, direction, a0, lats[k])
            dm.section, dm.pose_xy, dm.heading = sec, (x, y), h
            dm.along_mm, dm.lat_mm = dm._section_frame(x, y, sec)
            line = dm._build_line()
            cut = next(i for i, (px, py) in enumerate(line)
                       if i > 5 and dm._section_frame(px, py, nxt)[0] >= a0 - 1e-6
                       and abs(dm._section_frame(px, py, nxt)[1]) < 400)
            pieces[k] = line[:cut]
            lats[(k + 1) % 4] = dm._section_frame(*line[cut], nxt)[1]
    a = np.array([p for pc in pieces for p in pc])
    return a[:, 0], a[:, 1]


def test_better_than_current_line(plans):
    wins_len = wins_k = 0
    for sd, (f, pil, r) in plans.items():
        bx, by = _baseline(f.direction, f.signs)
        sb, sp = rp.path_stats(bx, by), rp.path_stats(r.x, r.y)
        wins_len += sp["length"] < sb["length"]
        wins_k += sp["krms"] < sb["krms"] and sp["flips"] <= sb["flips"]
        assert rp.lap_time_s(r.x, r.y, 309.0) < rp.lap_time_s(bx, by, 309.0), sd
    assert wins_len == len(plans) and wins_k == len(plans)


def test_compute_time(plans):
    ms = [r.solve_ms for _, _, r in plans.values()]
    print(f"route_planner: {np.mean(ms):.0f} ms medio, {max(ms):.0f} ms máx por vuelta")
    assert max(ms) < 5000.0


def test_digital_map_hook(monkeypatch):
    f, pil = _board(1)
    dm = dmod.DigitalMap()
    dm.set_direction(f.direction)
    dm.confirmed = {(s.section, s.seat): s.color for s in f.signs}
    dm.in_stall = False
    dm._aligned = True
    sec = dm.section_of(1)
    dm.section = sec
    x, y, h = dmod._lane_pose(sec, f.direction, 800.0, 0.0)
    dm.pose_xy, dm.heading = (x, y), h
    dm.along_mm, dm.lat_mm = dm._section_frame(x, y, sec)
    monkeypatch.setattr(C, "ROUTE_PLANNER", True, raising=False)
    line = dm._planned_line()
    assert len(line) > 20 and dm._steer_ok
    assert math.hypot(line[0][0] - x, line[0][1] - y) < 400.0
    key = dm._route_key
    dm._planned_line()
    assert dm._route_key == key            # no replanifica sin cambios
