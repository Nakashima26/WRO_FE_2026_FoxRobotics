"""Síntesis OpenWallMap: pasillos WRO 600/1000, snap, 3 esquinas ⇒ listo."""

from __future__ import annotations

import math

import numpy as np
import pytest

from pure_pursuit.open_map import (
    ESP_MAP_ANCHO_SLOW_CM,
    ESP_MAP_ANGOSTO_SLOW_CM,
    OPEN_CORRIDOR_MM,
    OpenWallMap,
    esp_consistent_corridors_cm,
    esp_map_recta_pwm,
    expected_tof_mm,
    field_inner_box_from_corridors,
    open_layout_from_seed,
    snap_corridor_mm,
)
from pure_pursuit.track_map import TrackGeometry, field_heading_to_twin
from pure_pursuit.twin.params import ToFParams
from pure_pursuit.twin.sensors import Pose, ToF
from pure_pursuit.twin.world import World, collision
from pure_pursuit.wro_field import OUTER_HALF_MM, randomize

# Patrón legal: N/S estrechos, E/W anchos.
CORRIDORS_L = {"S": 600.0, "E": 1000.0, "N": 600.0, "W": 1000.0}
TRUTH_BOX = field_inner_box_from_corridors(CORRIDORS_L)
TOL_MM = 55.0


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
    corridor_bias_mm: float = 0.0,
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
        if corridor_bias_mm and t_l is not None and t_r is not None:
            t_l += corridor_bias_mm / 2.0
            t_r += corridor_bias_mm / 2.0
        odom += spacing_mm
        wm.update_straight(t_ms, pose, t_l, t_r, t_f, odom)


def _run_lap1(wm: OpenWallMap, inner_box: tuple[float, float, float, float], drive: str) -> None:
    order = ("S", "E", "N", "W") if drive == "L" else ("S", "W", "N", "E")
    for k, sec in enumerate(order):
        assert wm.current_section == sec
        _feed_straight(wm, sec, inner_box)
        wm.on_corner_complete(k + 1)
    wm.on_corner_complete(4)


def test_esp_map_recta_pwm_lap2_wide_vs_narrow():
    m = (60, 100, 60, 100)
    assert esp_map_recta_pwm(m, 4) == (255, ESP_MAP_ANGOSTO_SLOW_CM)
    assert esp_map_recta_pwm(m, 5) == (255, ESP_MAP_ANCHO_SLOW_CM)
    assert esp_map_recta_pwm(m, 3) == (255, ESP_MAP_ANGOSTO_SLOW_CM)
    assert esp_map_recta_pwm((0, 0, 0, 0), 6) == (255, ESP_MAP_ANGOSTO_SLOW_CM)


@pytest.mark.parametrize("noisy,expect", [(580.0, 600.0), (620.0, 600.0), (950.0, 1000.0), (1050.0, 1000.0)])
def test_snap_corridor_mm(noisy, expect):
    assert snap_corridor_mm(noisy) == expect


def test_three_corners_ready_without_inferring_fourth():
    wm = OpenWallMap(drive_dir="L")
    for sec in ("S", "E", "N"):
        assert wm.current_section == sec
        _feed_straight(wm, sec, TRUTH_BOX)
        wm.on_corner_complete({"S": 1, "E": 2, "N": 3}[sec])
    assert wm.ready
    assert not wm._sections["W"].frozen
    assert wm._sections["W"].inferred is False
    g = wm.geometry()
    assert g["sections"]["S"]["kind"] == "600"
    assert g["sections"]["E"]["kind"] == "1000"
    assert g["sections"]["N"]["kind"] == "600"
    assert g["narrow_mask"]["S"] is True
    assert g["narrow_mask"]["E"] is False


def test_noisy_corridors_snap_after_three_turns():
    inner_noisy = field_inner_box_from_corridors({"S": 580.0, "E": 1020.0, "N": 615.0, "W": 1000.0})
    wm = OpenWallMap(drive_dir="L")
    for sec in ("S", "E", "N"):
        _feed_straight(wm, sec, inner_noisy, corridor_bias_mm=15.0 if sec == "N" else 0.0)
        wm.on_corner_complete({"S": 1, "E": 2, "N": 3}[sec])
    assert wm.ready
    assert wm._sections["S"].corridor_mm == pytest.approx(OPEN_CORRIDOR_MM[0], abs=1.0)
    assert wm._sections["E"].corridor_mm == pytest.approx(OPEN_CORRIDOR_MM[1], abs=1.0)
    assert wm._sections["N"].corridor_mm == pytest.approx(OPEN_CORRIDOR_MM[0], abs=1.0)


def test_four_corners_refine_inner_box_left():
    wm = OpenWallMap(drive_dir="L")
    _run_lap1(wm, TRUTH_BOX, "L")
    assert wm.frozen
    box = wm.inner_box_mm()
    hx, hy = wm.inner_half_mm()
    assert hx == pytest.approx(500.0, abs=TOL_MM)
    assert hy == pytest.approx(900.0, abs=TOL_MM)
    for i in range(4):
        assert box[i] == pytest.approx(TRUTH_BOX[i], abs=TOL_MM)


def test_four_corners_drive_right():
    corridors = {"S": 1000.0, "W": 600.0, "N": 1000.0, "E": 600.0}
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
    corridor = d_in + d_out + (2 * 35.2)
    assert corridor == pytest.approx(CORRIDORS_L["S"], abs=40.0)


def test_reset_and_geometry_summary():
    wm = OpenWallMap()
    wm.reset("R", "S")
    _feed_straight(wm, "S", TRUTH_BOX, steps=10)
    g = wm.geometry()
    assert g["drive_dir"] == "R"
    assert "sections" in g and g["sections"]["S"]["along_mm"] > 0
    assert "narrow_mask" in g


def test_real_log_is_reconciled_like_esp():
    raw = (1290, 620, 780, 1270)
    assert tuple(snap_corridor_mm(v) for v in raw) == (1000, 600, 600, 1000)
    assert esp_consistent_corridors_cm(raw) == (100, 100, 100, 100)
    assert esp_consistent_corridors_cm((800, 600, 1000, 1000)) == (100, 60, 100, 60)
    assert ESP_MAP_ANCHO_SLOW_CM == 70


@pytest.mark.parametrize("drive,start", [("L", "E"), ("R", "W")])
def test_world_tof_recovers_seeded_rectangular_layout(drive, start):
    truth = open_layout_from_seed(3)
    assert truth["S"] == truth["N"] == 600
    assert truth["E"] == truth["W"] == 1000
    box = field_inner_box_from_corridors(truth)
    field = randomize(3)
    field.signs, field.barriers = [], []
    world = World(field, inner_box_mat=box)
    tof = ToF(ToFParams(half_fov_deg=0, noise_sigma_mm=0, noise_rel=0,
                        black_wall_max_mm=3000, short_map_ids=(),
                        status_ok_prob_by_id=(1, 1, 1, 1)), np.random.default_rng(3))
    wm = OpenWallMap(drive_dir=drive, start_section=start)
    for turn in range(1, 5):
        section = wm.current_section
        x, y, h = _section_line(section, box, drive)
        pose = Pose(x, y, field_heading_to_twin(h))
        t = float(turn)
        left = tof.read_mm(0, world, pose, t)
        right = tof.read_mm(1, world, pose, t)
        assert left + right + 70.4 == pytest.approx(truth[section], abs=2)
        wm.update_straight(t * 1000, (x, y, h), left, right, None, turn * 1000)
        wm.on_corner_complete(turn)
    assert wm.frozen
    assert {sec: entry["corridor_mm"] for sec, entry in wm.geometry()["sections"].items()} == truth
    assert wm.inner_box_mm() == box

    assert collision(world, 0, -700, 90, 120, 80, 20) == "isla"
    assert collision(World(field), 0, -700, 90, 120, 80, 20) is None
    assert collision(world, 700, 0, 0, 120, 80, 20) is None


def test_missing_corridors_keep_map_incomplete():
    wm = OpenWallMap(start_section="W")
    for turn in range(1, 5):
        wm.on_corner_complete(turn)
    assert not wm.ready and not wm.frozen
