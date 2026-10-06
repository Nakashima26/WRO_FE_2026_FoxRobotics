"""collision() optimizada (floats Python, aristas cacheadas) == referencia de 54031cd/96dfc54."""

from __future__ import annotations

import numpy as np
import pytest

from pure_pursuit.twin import world as W
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.world import (
    SIGN_MM,
    World,
    _hits_polygon,
    _point_in_island,
    _point_outside_field,
    _segments_cross,
    body_corners,
    collision,
)
from pure_pursuit.wro_field import randomize


def _collision_ref(world, x, y, heading_deg, length_mm, width_mm, rear_overhang_mm, ignore=()):
    """Copia literal de collision en la base (world.py:257-298)."""
    corners = body_corners(x, y, heading_deg, length_mm, width_mm, rear_overhang_mm)
    corner_list = [tuple(c) for c in corners]
    for cx, cy in corner_list:
        if _point_outside_field(cx, cy) and "pared exterior" not in ignore:
            return "pared exterior"
        if _point_in_island(cx, cy):
            return "isla"
    edges = list(zip(corner_list, corner_list[1:] + corner_list[:1]))
    track = world.field
    polys = (track.inner,) if "pared exterior" in ignore else (track.outer, track.inner)
    for poly in polys:
        wall = [(float(p[0]), float(p[1])) for p in poly]
        wall_edges = list(zip(wall, wall[1:] + wall[:1]))
        for a, b in edges:
            for c, d in wall_edges:
                if _segments_cross(a, b, c, d):
                    return "pared"
    half = SIGN_MM / 2.0
    for sign in track.signs:
        box = [
            (sign.x - half, sign.y - half),
            (sign.x + half, sign.y - half),
            (sign.x + half, sign.y + half),
            (sign.x - half, sign.y + half),
        ]
        if _hits_polygon(corner_list, box):
            return f"señal {sign.color} {sign.section}/{sign.seat}"
    for box in track.barriers:
        if _hits_polygon(corner_list, box):
            return "cajón magenta"
    return None


def _poses(field, rng):
    pts = [(rng.uniform(-1500, 1500), rng.uniform(-1500, 1500), rng.uniform(0, 360)) for _ in range(4000)]
    centers = [(s.x, s.y) for s in field.signs]
    for box in field.barriers:
        centers += [(float(p[0]), float(p[1])) for p in box]
    for cx, cy in centers:
        for _ in range(200):
            pts.append((cx + rng.uniform(-250, 250), cy + rng.uniform(-250, 250), rng.uniform(0, 360)))
    for _ in range(300):
        v = rng.choice([-1500.0, 1500.0, -500.0, 500.0]) + rng.uniform(-60, 60)
        u = rng.uniform(-1500, 1500)
        xy = (v, u) if rng.random() < 0.5 else (u, v)
        pts.append((xy[0], xy[1], rng.uniform(0, 360)))
    return pts


@pytest.mark.parametrize("seed", range(2, 14))
def test_collision_equiv(seed):
    field = randomize(seed)
    w = World(field)
    rng = np.random.default_rng(7)
    v = TwinParams().vehicle
    dims = (v.length_mm, v.width_mm, v.rear_overhang_mm)
    for x, y, h in _poses(field, rng):
        for ign in ((), ("pared exterior",)):
            assert collision(w, x, y, h, *dims, ignore=ign) == _collision_ref(w, x, y, h, *dims, ignore=ign)


def test_zero_division_fallback(monkeypatch):
    field = randomize(2)
    w = World(field)
    v = TwinParams().vehicle
    dims = (v.length_mm, v.width_mm, v.rear_overhang_mm)
    orig = W._point_in_poly
    state = {"raised": False}

    def wrapped(px, py, poly):
        if not state["raised"] and type(px) is float:
            state["raised"] = True
            raise ZeroDivisionError
        return orig(px, py, poly)

    monkeypatch.setattr(W, "_point_in_poly", wrapped)
    rng = np.random.default_rng(7)
    for x, y, h in _poses(field, rng)[:1500]:
        assert collision(w, x, y, h, *dims) == _collision_ref(w, x, y, h, *dims)
    # el wrapper solo dispara si alguna pose llega a _point_in_poly con floats
    assert state["raised"]
