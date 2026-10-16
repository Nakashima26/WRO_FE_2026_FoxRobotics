"""Arranque forzado con la línea naranja (route_mode.RouteMode._line_vote)."""

import numpy as np

from pure_pursuit import config as C
from pure_pursuit import wro_field as W
from pure_pursuit.route_mode import RouteMode


class _Dm:
    direction = "CW"


def _rm(orange):
    rm = RouteMode.__new__(RouteMode)
    rm.orange = orange
    rm._line_d = []
    rm.axis = W.w_axis("N")
    return rm


def test_line_vote_recovers_delta():
    delta, fwd = 300.0, 350.0
    (sx, sy), (ex, ey) = W.orange_segments()[0]
    # Punto de la cinta cerca de su punta; el look-point del carro está δ atrás (marco supuesto).
    mx, my = sx + 0.1 * (ex - sx), sy + 0.1 * (ey - sy)
    ax, ay = W.w_axis("N")
    px, py = mx - delta * ax, my - delta * ay
    pose = (px, py - fwd, 0.0)                      # heading 0: el look-point queda fwd mm en +y
    # Recta naranja con pendiente que cruza la columna central a fwd mm delante del eje trasero
    vx, vy, x0 = 1.0, 0.5, 0.0
    by = float(C.ROBOT_BEV_Y) - (fwd - float(C.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM)) / C.MM_PER_PX
    y0 = by - (float(C.ROBOT_BEV_X) - x0) * vy / vx
    rm = _rm({"seen": True, "line": (vx, vy, x0, y0)})
    rm._line_vote(_Dm(), pose)
    assert len(rm._line_d) == 1
    assert abs(rm._line_d[0] - delta) <= 1.0


def test_line_vote_ignores_unseen_or_dead_reckoned():
    pose = (0.0, 0.0, 0.0)
    line = (1.0, 0.5, 0.0, 155.0)
    for o in ({"seen": False, "line": line},
              {"seen": True, "line": line, "dead_reckoned": True},
              {"seen": True, "line": None}):
        rm = _rm(o)
        rm._line_vote(_Dm(), pose)
        assert rm._line_d == []
