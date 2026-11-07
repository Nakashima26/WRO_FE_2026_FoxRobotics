"""Planificador de esquina continuo (vueltas 2–3, mapa conocido).

El apex / hold / release salen del color y (along, side) de la lata que
manda — tipicamente la 1ª de la siguiente recta — con techo si esta termina
en rojo o es cajón. No vota open/closed/island fijos.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from . import racing_line_astar as rl
from .route_planner import pillars_from_seats


_NEXT = {
    "CW": {"N": "E", "E": "S", "S": "W", "W": "N"},
    "CCW": {"N": "W", "W": "S", "S": "E", "E": "N"},
}

# Piecewise side_pass (marco + = isla) → apex_lat.
# Verde al medio (−120) → −80, no full-open.
_SIDE_APEX = (
    (-280.0, -260.0),
    (-120.0, -80.0),
    (0.0, 40.0),
    (120.0, 110.0),
    (240.0, 170.0),
)
_CLOSED_SOFT = 40.0  # techo: no más afuera que esto si rojo trailing / cajón
_APEX_MIN, _APEX_MAX = -260.0, 170.0


@dataclass
class CornerPlan:
    mode: str
    section: str
    next_section: str
    direction: str
    release_along_mm: float
    hold_dF_cm: float
    entry_lat_mm: float
    apex_lat_mm: float
    exit_lat_mm: float
    time_s: float
    clear_mm: float
    cost: float
    hint: str
    solve_ms: float = 0.0
    scored: dict | None = None

    @property
    def opens_out(self) -> bool:
        """Arco hacia pared exterior (apex del lado afuera en marco isla+)."""
        return self.apex_lat_mm * _sign_lat(self.direction) < -80.0

    @property
    def gf_cm(self) -> float:
        """Umbral frontal sugerido para gf= del V2."""
        from . import config as C
        if self.opens_out:
            return float(getattr(C, "GR_GF_OUTER_CM", 35))
        return float(getattr(C, "GR_GF_INNER_CM", 60))


def next_section(direction: str, section: str) -> str:
    return _NEXT[direction][section]


def _sign_lat(direction: str) -> float:
    """CCW espeja el lateral (+isla en CW → − en CCW)."""
    return 1.0 if direction == "CW" else -1.0


def _norm_pillars(pillars) -> list[tuple[float, float, str]]:
    if isinstance(pillars, dict):
        return pillars_from_seats(pillars)
    return rl._norm_pillars(pillars) if pillars else []


def _lerp_piecewise(x: float, table: tuple[tuple[float, float], ...]) -> float:
    if x <= table[0][0]:
        return table[0][1]
    if x >= table[-1][0]:
        return table[-1][1]
    for i in range(len(table) - 1):
        x0, y0 = table[i]
        x1, y1 = table[i + 1]
        if x0 <= x <= x1:
            u = 0.0 if x1 <= x0 else (x - x0) / (x1 - x0)
            return y0 + u * (y1 - y0)
    return table[-1][1]


def _cans_on_section(direction: str, sec: str, pillars
                     ) -> list[tuple[float, float, str]]:
    """(along, side_pass, color) ordenados por along."""
    from .digital_map import DigitalMap
    from .wro_field import SEATS, section_to_world

    pil = _norm_pillars(pillars)
    cans = []
    for px, py, col in pil:
        best = None
        best_d = 1e9
        for sid, (h, w) in SEATS.items():
            sx, sy = section_to_world(sec, h, w)
            d = math.hypot(px - sx, py - sy)
            if d < best_d:
                best_d, best = d, (sx, sy, col)
        if best is None or best_d > 120.0:
            continue
        sx, sy, col = best
        dm = DigitalMap.__new__(DigitalMap)
        dm.direction = direction
        sa, sr = dm._section_frame(sx, sy, sec)
        side = sr + 200.0 if col == "Red" else sr - 200.0
        side = max(-320.0, min(320.0, side))
        cans.append((sa, side, col))
    cans.sort()
    return cans


def _next_lead(direction: str, next_sec: str, pillars
               ) -> tuple[float, float, str] | None:
    """(along, side_pass, color) de la 1ª lata, o None."""
    cans = _cans_on_section(direction, next_sec, pillars)
    if not cans:
        return None
    return cans[0]


def _last_can(direction: str, section: str, pillars
              ) -> tuple[float, float, str] | None:
    cans = _cans_on_section(direction, section, pillars)
    if not cans:
        return None
    return cans[-1]


def _next_lead_side(direction: str, next_sec: str, pillars
                    ) -> tuple[float, float] | None:
    """Compat: (along, side) de la 1ª lata."""
    lead = _next_lead(direction, next_sec, pillars)
    return None if lead is None else (lead[0], lead[1])


def _last_can_side(direction: str, section: str, pillars
                   ) -> float | None:
    """Compat: lado de paso de la última lata."""
    last = _last_can(direction, section, pillars)
    return None if last is None else last[1]


def apex_from_side(side_inner: float) -> float:
    """side en marco +isla → apex_lat (mismo marco)."""
    return _lerp_piecewise(side_inner, _SIDE_APEX)


def _mode_label(apex_inner: float) -> str:
    if apex_inner < -100.0:
        return "open"
    if apex_inner > 150.0:
        return "island"
    if apex_inner < 20.0:
        return "mid"
    return "closed"


def suggest_mode(direction: str, section: str, pillars,
                 parking: str | None = None) -> str:
    """Label derivado del arco continuo (solo log / tests)."""
    plan = plan_corner(direction, section, pillars, parking=parking)
    return plan.mode


def _clearance_at_apex(direction: str, section: str, apex_signed: float,
                      pillars, params: rl.LatticeParams | None = None) -> float:
    """Clearance barato en el vértice de esquina con lateral apex."""
    prm = params or rl.LatticeParams(ds_mm=40.0, a_step_mm=40.0, n_kappa=7)
    pil = _norm_pillars(pillars)
    i0, _i_c, i1, c, t, nrm, _ = rl.section_index_range(direction, section, prm.ds_mm)
    idx = rl._unwrap_indices(i0, i1, len(c))
    if len(idx) < 2:
        return 1e9
    i_corner = idx[max(1, len(idx) * 2 // 3)]
    x, y, th = rl._pose(c, t, nrm, i_corner, apex_signed, 0.0)
    cl, _ = rl._clearance_pose(x, y, th, pil, prm)
    return float(cl)


def plan_corner(direction: str, section: str, pillars,
                params: rl.LatticeParams | None = None,
                *, parking: str | None = None) -> CornerPlan:
    """Arco continuo para la esquina al final de `section`."""
    if direction not in _NEXT or section not in _NEXT[direction]:
        raise ValueError(f"esquina inválida: {direction}/{section}")
    t0 = time.perf_counter()
    sgn = _sign_lat(direction)
    nxt = next_section(direction, section)
    lead = _next_lead(direction, nxt, pillars)
    last = _last_can(direction, section, pillars)

    # Lata que manda: lead de la siguiente si está cerca; si no, centro suave.
    if lead is not None and lead[0] <= 1300.0:
        side_raw = lead[1]
        # Si la lead está lejos en along pero hay última en esta, mezcla leve.
        side_inner = side_raw * sgn
    else:
        side_inner = 0.0

    apex_inner = apex_from_side(side_inner)
    # Rojo al final de ESTA recta: apex/entry cerrados (no abrirse en rojo).
    # El exit sí puede seguir al verde de la siguiente.
    red_trail = last is not None and last[2] == "Red"
    if red_trail:
        apex_inner = max(apex_inner, _CLOSED_SOFT)
    # Cajón: más estricto (ni full-open ni island agresivo).
    if parking and section == parking:
        apex_inner = max(apex_inner, _CLOSED_SOFT)
        apex_inner = min(apex_inner, 120.0)
        red_trail = True

    apex_inner = max(_APEX_MIN, min(_APEX_MAX, apex_inner))
    apex_signed = apex_inner * sgn

    # Clearance barato: si choca, empujar 40 mm hacia el centro del carril.
    clear = _clearance_at_apex(direction, section, apex_signed, pillars, params)
    if clear < 25.0:
        apex_inner = apex_inner - 40.0 if apex_inner > 0 else apex_inner + 40.0
        apex_inner = max(_APEX_MIN, min(_APEX_MAX, apex_inner))
        if red_trail:
            apex_inner = max(apex_inner, _CLOSED_SOFT)
        apex_signed = apex_inner * sgn
        clear = _clearance_at_apex(direction, section, apex_signed, pillars, params)

    entry_inner = 0.55 * apex_inner
    if red_trail:
        entry_inner = max(entry_inner, _CLOSED_SOFT)
    exit_inner = 0.75 * apex_inner
    # Salida sigue a la lead (verde medio → exit suave afuera, no full-open).
    if lead is not None and lead[0] <= 1300.0:
        lead_apex = apex_from_side(lead[1] * sgn)
        # Mezcla hacia la lead; con rojo trailing no bajar de −120 (medio).
        floor = -120.0 if red_trail else _APEX_MIN
        exit_inner = max(floor, min(exit_inner, 0.35 * exit_inner + 0.65 * lead_apex))
    u_inner = max(0.0, min(1.0, (apex_inner + 260.0) / 430.0))
    hold = 42.0 + u_inner * (62.0 - 42.0)
    release = 2550.0 + u_inner * (2360.0 - 2550.0)
    mode = _mode_label(apex_inner)
    hint = mode
    scored = {
        "side_inner": side_inner,
        "apex_inner": apex_inner,
        "lead": None if lead is None else {
            "along": lead[0], "side": lead[1], "color": lead[2],
        },
        "last": None if last is None else {
            "along": last[0], "side": last[1], "color": last[2],
        },
        "clear_mm": clear,
    }

    return CornerPlan(
        mode=mode, section=section,
        next_section=nxt, direction=direction,
        release_along_mm=float(release),
        hold_dF_cm=float(hold),
        entry_lat_mm=float(entry_inner * sgn),
        apex_lat_mm=float(apex_signed),
        exit_lat_mm=float(exit_inner * sgn),
        time_s=0.0, clear_mm=float(clear), cost=0.0,
        hint=hint,
        solve_ms=(time.perf_counter() - t0) * 1e3,
        scored=scored,
    )


def plan_corner_from_confirmed(direction: str, section: str, confirmed,
                               *, parking: str | None = None,
                               params: rl.LatticeParams | None = None) -> CornerPlan:
    """confirmed: dict {(sec, seat): color} de DigitalMap."""
    return plan_corner(direction, section, confirmed, params, parking=parking)
