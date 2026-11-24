"""Síntesis OpenWallMap: pasillos asimétricos, 3 esquinas ⇒ listo, L y R."""

from __future__ import annotations

import math

import pytest

from pure_pursuit.open_map import (
    OpenWallMap,
    expected_tof_mm,
    field_inner_box_from_corridors,
)
from pure_pursuit.track_map import TrackGeometry
from pure_pursuit.wro_field import OUTER_HALF_MM

# Pasillos distintos N/S vs E/W para comprobar rectángulo (no cuadrado 1 m).
CORRIDORS_L = {"S": 800.0, "E": 1200.0, "N": 800.0, "W": 1200.0}
TRUTH_BOX = field_inner_box_from_corridors(CORRIDORS_L)
TOL_MM = 50.0


def _section_line(
    section: str,
    inner_box: tuple[float, float, float, float],
    drive: str = "L",
) -> tuple[float, float, float]:
    """Punto en el centro del pasillo y rumbo (deg campo)."""
    x0, y0, x1, y1 = inner_box
    cx0, cy0 = x0 - 1500.0, y0 - 1500.0
    cx1, cy1 = x1 - 1500.0, y1 - 1500.0
    oh = OUTER_HALF_MM
    geom = TrackGeometry(drive)
    h = geom.section_frame(geom._order.index(section))[2]  # type: ignore[attr-defined]
    if section == "S":
        y = (cy0 + (-oh)) / 2.0
        x = 0.0
    elif section == "N":
        y = (cy1 + oh) / 2.0
        x = 0.0
    elif section == "E":
        x = (cx1 + oh) / 2.0
        y = 0.0
    else:
        x = (cx0 + (-oh)) / 2.0
        y = 0.0
    return x, y, h


def _inner_half_x(inner_box: tuple[float, float, float, float]) -> float:
    x0, _, x1, _ = inner_box
    return (x1 - x0) / 2.0


def _feed_straight(
    wm: OpenWallMap,
    section: str,
    inner_box: tuple[float, float, float, float],
    steps: int = 40,
    spacing_mm: float = 12.0,
) -> None:
    """Muestras a lo largo de la recta sin entrar en las esquinas (|along| ≤ tramo interior)."""
    x0, y0, h = _section_line(section, inner_box, wm.drive_dir)
    odom = 0.0
    dx, dy = math.cos(math.radians(h)), math.sin(math.radians(h))
    hx = _inner_half_x(inner_box)
    hy = (inner_box[3] - inner_box[1]) / 2.0
    half_along = hx if section in ("S", "N") else hy
    travel = min(0.65 * 2.0 * half_along, spacing_mm * (steps - 1))
    for i in range(steps):
        t_ms = i * 20.0
        s = -travel / 2.0 + travel * (i / max(steps - 1, 1))
        px = x0 + dx * s
        py = y0 + dy * s
        pose = (px, py, h)
        t_l = expected_tof_mm(pose, "L", inner_box)
        t_r = expected_tof_mm(pose, "R", inner_box)
        t_f = expected_tof_mm(pose, "F", inner_box)
        odom += spacing_mm
        wm.update_straight(t_ms, pose, t_l, t_r, t_f, odom)


def _run_lap1(wm: OpenWallMap, inner_box: tuple[float, float, float, float], drive: str) -> None:
    order = ("S", "E", "N", "W") if drive == "L" else ("S", "W", "N", "E")
    for k, sec in enumerate(order):
        assert wm.current_section == sec
        _feed_straight(wm, sec, inner_box)
        wm.on_corner_complete(k + 1)
    wm.on_corner_complete(4)


def test_three_corners_ready_and_infer_fourth():
    wm = OpenWallMap(drive_dir="L")
    for sec in ("S", "E", "N"):
        assert wm.current_section == sec
        _feed_straight(wm, sec, TRUTH_BOX)
        wm.on_corner_complete({"S": 1, "E": 2, "N": 3}[sec])
    assert wm.ready
    assert wm._sections["W"].frozen
    assert wm._sections["W"].inferred
    box = wm.inner_box_mm()
    for i in range(4):
        assert box[i] == pytest.approx(TRUTH_BOX[i], abs=TOL_MM)


def test_four_corners_refine_inner_box_left():
    wm = OpenWallMap(drive_dir="L")
    _run_lap1(wm, TRUTH_BOX, "L")
    assert wm.frozen
    box = wm.inner_box_mm()
    hx, hy = wm.inner_half_mm()
    assert hx == pytest.approx(300.0, abs=TOL_MM)
    assert hy == pytest.approx(700.0, abs=TOL_MM)
    for i in range(4):
        assert box[i] == pytest.approx(TRUTH_BOX[i], abs=TOL_MM)


def test_four_corners_drive_right():
    corridors = {"S": 900.0, "W": 1100.0, "N": 900.0, "E": 1100.0}
    truth = field_inner_box_from_corridors(corridors)
    wm = OpenWallMap(drive_dir="R")
    _run_lap1(wm, truth, "R")
    box = wm.inner_box_mm()
    for i in range(4):
        assert box[i] == pytest.approx(truth[i], abs=TOL_MM)


def test_expected_tof_matches_corridor_width_on_south():
    inner = TRUTH_BOX
    x, y, h = _section_line("S", inner, "L")
    d_in = expected_tof_mm((x, y, h), "L", inner)
    d_out = expected_tof_mm((x, y, h), "R", inner)
    assert d_in is not None and d_out is not None
    span = 2 * 35.2  # SENSOR_MOUNTS en config
    corridor = d_in + d_out + (2 * 35.2)
    assert corridor == pytest.approx(CORRIDORS_L["S"], abs=40.0)


def test_reset_and_geometry_summary():
    wm = OpenWallMap()
    wm.reset("R", "S")
    _feed_straight(wm, "S", TRUTH_BOX, steps=10)
    g = wm.geometry()
    assert g["drive_dir"] == "R"
    assert "sections" in g and g["sections"]["S"]["along_mm"] > 0