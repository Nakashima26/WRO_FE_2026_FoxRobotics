"""Regresión del voto de color con pies BEV sintéticos del gemelo."""

import math

import pytest

from pure_pursuit import digital_map as dm


def _observe(world, color, true_pose, heading):
    """Invierte la proyección del centro de una lata al pie en el BEV."""
    h = math.radians(heading)
    dx, dy = world[0] - true_pose[0], world[1] - true_pose[1]
    fwd = dx * math.sin(h) + dy * math.cos(h)
    lat = dx * math.cos(h) - dy * math.sin(h)
    ray = math.hypot(fwd - dm._CAM_AHEAD_MM, lat)
    fwd -= dm._FOOT_TO_CENTER_MM * (fwd - dm._CAM_AHEAD_MM) / ray
    lat -= dm._FOOT_TO_CENTER_MM * lat / ray
    return (
        dm.C.ROBOT_BEV_X + lat / dm.C.MM_PER_PX,
        dm.C.ROBOT_BEV_Y - (fwd - dm._BEV_AHEAD_MM) / dm.C.MM_PER_PX,
        color,
    )


@pytest.fixture
def track(monkeypatch):
    monkeypatch.setattr(dm.C, "DIGITAL_MAP_DIRECTION", "CW")
    monkeypatch.setattr(dm.C, "DIGITAL_MAP_PARKING", "N")
    monkeypatch.setattr(dm.C, "DIGITAL_MAP_SEED_CONFIRMED", False, raising=False)
    m = dm.DigitalMap()
    m.section, m._tc, m.in_stall = "W", 3, False
    m._deferred = None
    m.pose_xy, m._view_heading = (-900.0, -1050.0), 0.0
    return m


@pytest.mark.parametrize("protected, expected", [(False, "Red"), (True, "Green")])
@pytest.mark.parametrize("heading", [0.0, 24.0])
def test_red_from_other_row_does_not_replace_green(monkeypatch, track, protected, expected, heading):
    monkeypatch.setattr(dm.C, "DIGITAL_MAP_RECHECK_COLOR", protected)
    key = ("W", "T2")
    green = _observe(track._seat_xy(*key), "Green", track.pose_xy, track._view_heading)
    for _ in range(12):
        track._vote([green])
    assert track.confirmed[key] == "Green"

    # La pose estimada sale de la curva corrida 110 mm hacia dentro.
    # El rojo de T4 cae entre filas tanto de frente como todavía en ángulo.
    track._view_heading = heading
    red = _observe(track._seat_xy("W", "T4"), "Red", (-1010.0, -1050.0), heading)
    for _ in range(18):
        track._vote([red])
    assert track.confirmed[key] == expected


def test_close_votes_can_correct_color_once_per_frame(monkeypatch, track):
    monkeypatch.setattr(dm.C, "DIGITAL_MAP_RECHECK_COLOR", True)
    key = ("W", "T2")
    center = track._seat_xy(*key)
    green = _observe(center, "Green", track.pose_xy, track._view_heading)
    red = _observe(center, "Red", track.pose_xy, track._view_heading)
    for _ in range(12):
        track._vote([green])
    for _ in range(13):
        track._vote([red, red])
    assert track.confirmed[key] == "Green"
    track._vote([red])
    assert track.confirmed[key] == "Red"


def test_x_seat_released_when_other_color_seen_on_t_seat(monkeypatch, track):
    """orillas368: el rojo quedó en W/X1 y la carta tiraba el verde de W/T2 para siempre."""
    monkeypatch.setattr(dm.C, "DIGITAL_MAP_RECHECK_COLOR", False)
    track._frozen = lambda: True
    track.confirmed[("W", "X1")] = "Red"
    green = _observe(track._seat_xy("W", "T2"), "Green", track.pose_xy, track._view_heading)
    for _ in range(12):
        track._vote([green])
    assert ("W", "X1") not in track.confirmed
    assert track.confirmed.get(("W", "T2")) == "Green"
