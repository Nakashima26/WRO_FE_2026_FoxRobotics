"""
Línea de carrera por carta (vueltas 2–3): lattice Frenet + DP / Hybrid-A*.

Unidad: una recta (pilares de la carta) + arco de esquina siguiente hasta
salir alineado a la recta siguiente. No planifica parking ni salida de cajón.

El progreso a lo largo de la referencia es monótono → el grafo es un DAG y
el óptimo en la malla es DP forward. La frontera de entrada al arco usa el
cost-to-go de un DP backward en la esquina (heurística admissible de A* /
rejoin). Curvatura bicicleta: κ_max = tan(44.54°)/113 ≈ 1/115 (mismo
equivalente que config.MAX_WHEEL_STEER_DEG / route_planner).

Métrica: T ≈ ∫ ds/v(κ) (scrub del twin) − bonus de holgura + |Δκ|.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, replace

import numpy as np

from . import route_planner as rp
from .route_planner import (
    ARC_R,
    ISLAND_HALF,
    KAPPA_MAX,
    OUTER_HALF,
    SCRUB_COEFF,
    WHEELBASE_MM,
    DELTA_MAX_RAD,
    _PILLAR_CIRC,
    _norm_pillars,
    reference_loop,
)

_NEXT = {
    "CW": {"N": "E", "E": "S", "S": "W", "W": "N"},
    "CCW": {"N": "W", "W": "S", "S": "E", "E": "N"},
}

# Segmentos CW de _reference_cw: L,A,L,A,L,A,L,A — recta N=0, E=2, S=4, W=6.
_CW_STRAIGHT_SEG = {"N": 0, "E": 2, "S": 4, "W": 6}


@dataclass(frozen=True)
class LatticeParams:
    ds_mm: float = 40.0
    a_step_mm: float = 40.0
    a_min_mm: float = -380.0
    a_max_mm: float = 380.0
    psi_step_deg: float = 5.0
    psi_max_deg: float = 40.0
    n_kappa: int = 7
    kappa_max: float = KAPPA_MAX
    kappa_margin: float = 0.85
    disc_radius_mm: float = 72.0
    disc_offsets_mm: tuple = (0.0, 60.0, 120.0)
    pillar_margin_mm: float = 25.0
    wall_margin_mm: float = 40.0
    w_clear: float = 5.0e-4          # s/mm de holgura (cap abajo)
    clear_cap_mm: float = 100.0
    w_dk: float = 0.10               # s por |Δκ|/κ_max
    v0_mm_s: float = 309.0
    exit_psi_tol_deg: float = 10.0
    exit_a_band_mm: float = 160.0
    hard_clear_mm: float = 0.0       # rechazar aristas con clear < esto


@dataclass
class CardRoute:
    """Tramo de una carta (abierto: no cierra el anillo)."""
    x: np.ndarray
    y: np.ndarray
    heading_deg: np.ndarray
    kappa: np.ndarray
    s: np.ndarray
    a: np.ndarray
    section: str
    direction: str
    length_mm: float
    time_s: float
    clear_mm: float
    kappa_ok: bool
    solve_ms: float = 0.0
    method: str = "dp"
    card: int | None = None

    def xy(self) -> np.ndarray:
        return np.stack([self.x, self.y], axis=1)


@dataclass
class LapRoute:
    """Vuelta = 4 cartas encadenadas (abierta al cierre; no incluye U ni parking)."""
    x: np.ndarray
    y: np.ndarray
    heading_deg: np.ndarray
    kappa: np.ndarray
    s: np.ndarray
    direction: str
    sections: tuple
    cards: list
    length_mm: float
    time_s: float
    clear_mm: float
    kappa_ok: bool
    solve_ms: float = 0.0
    method: str = "chain"

    def xy(self) -> np.ndarray:
        return np.stack([self.x, self.y], axis=1)


def pillars_for_card(section: str, card: int) -> list[tuple[float, float, str]]:
    """Pilares (x, y, color) de una carta del reglamento en una recta."""
    from .wro_field import CARDS, SEATS, section_to_world
    if not (1 <= card <= 36):
        raise ValueError(f"carta fuera de 1..36: {card}")
    out = []
    for seat, color in CARDS[card - 1]:
        h, w = SEATS[seat]
        x, y = section_to_world(section, h, w)
        out.append((float(x), float(y), color))
    return out


def pillars_for_section(field, section: str) -> list[tuple[float, float, str]]:
    return [(s.x, s.y, s.color) for s in field.signs if s.section == section]


def section_order(direction: str, start_section: str = "N") -> list[str]:
    """Orden de las 4 rectas desde start_section en el sentido de marcha."""
    if start_section not in _CW_STRAIGHT_SEG:
        raise ValueError(start_section)
    out = []
    s = start_section
    for _ in range(4):
        out.append(s)
        s = _NEXT[direction][s]
    return out


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def frenet_from_pose(c, t, nrm, i: int, x: float, y: float, heading_rad: float
                     ) -> tuple[float, float]:
    """(a, psi) en la estación i a partir de pose mundo."""
    a = (x - float(c[i, 0])) * float(nrm[i, 0]) + (y - float(c[i, 1])) * float(nrm[i, 1])
    th_ref = math.atan2(float(t[i, 0]), float(t[i, 1]))
    return a, _wrap_pi(heading_rad - th_ref)


def _seg_edges_cw() -> np.ndarray:
    half, rad = ISLAND_HALF, ARC_R
    lane = 1000.0
    segs = [
        ("L", 2 * half),
        ("A", math.pi / 2 * rad),
        ("L", 2 * half),
        ("A", math.pi / 2 * rad),
        ("L", 2 * half),
        ("A", math.pi / 2 * rad),
        ("L", 2 * half),
        ("A", math.pi / 2 * rad),
    ]
    lens = [s[1] for s in segs]
    return np.concatenate([[0.0], np.cumsum(lens)]), float(sum(lens)), lane


def section_index_range(direction: str, section: str, ds_mm: float = 40.0
                        ) -> tuple[int, int, int, np.ndarray, np.ndarray, np.ndarray, float]:
    """(i0, i_corner, i1, c, t, nrm, ds) para recta `section` + esquina siguiente.

    i0..i_corner = recta; i_corner..i1 = arco; i1 exclusivo.
    """
    if section not in _CW_STRAIGHT_SEG:
        raise ValueError(section)
    c, t, nrm, total = reference_loop(direction, ds_mm)
    n = len(c)
    ds = total / n
    edges, loop_len, _ = _seg_edges_cw()
    seg = _CW_STRAIGHT_SEG[section]
    # En CW: s_cw crece con el índice. En CCW la polilínea está invertida.
    s0, s_arc, s1 = edges[seg], edges[seg + 1], edges[seg + 2]
    if direction == "CW":
        def to_i(s):
            return int(round((s % loop_len) / ds)) % n
        i0, i_c, i1 = to_i(s0), to_i(s_arc), to_i(s1)
    else:
        # CCW: índice 0 ≈ fin del loop CW; s_cw = loop_len - i*ds (aprox).
        def to_i(s_cw):
            return int(round(((loop_len - s_cw) % loop_len) / ds)) % n
        i0, i_c, i1 = to_i(s1), to_i(s_arc), to_i(s0)
    return i0, i_c, i1, c, t, nrm, ds


def _unwrap_indices(i0: int, i1: int, n: int) -> list[int]:
    """Índices de i0 inclusive a i1 exclusive, permitiendo wrap."""
    if i0 == i1:
        return []
    out = []
    i = i0
    guard = 0
    while i != i1 and guard < n + 2:
        out.append(i)
        i = (i + 1) % n
        guard += 1
    return out


def _kappa_ref(t: np.ndarray, idx: list[int]) -> np.ndarray:
    """Curvatura de la referencia en cada estación del tramo (abierto)."""
    kref = np.zeros(len(idx))
    for j in range(len(idx) - 1):
        t0, t1 = t[idx[j]], t[idx[j + 1]]
        cross = t0[0] * t1[1] - t0[1] * t1[0]
        dot = float(np.clip(t0 @ t1, -1.0, 1.0))
        dpsi = math.atan2(cross, dot)
        # ds ≈ |ángulo| * R en arco; en recta ~0
        # κ = dψ/ds; ds lo pone el llamador — aquí devolvemos dψ y se divide luego
        kref[j] = dpsi
    return kref


def _pose(c, t, nrm, i: int, a: float, psi: float):
    x = float(c[i, 0] + a * nrm[i, 0])
    y = float(c[i, 1] + a * nrm[i, 1])
    th = math.atan2(float(t[i, 0]), float(t[i, 1])) + psi
    return x, y, th


def _disc_centers(x: float, y: float, th: float, offsets) -> list[tuple[float, float]]:
    fx, fy = math.sin(th), math.cos(th)
    return [(x + o * fx, y + o * fy) for o in offsets]


def _clearance_pose(x, y, th, pillars, prm: LatticeParams,
                    soft_pillars: list | None = None) -> tuple[float, float]:
    """(clear_hard, clear_soft) en mm. Negativo = violación.

    soft_pillars: lookahead (siguiente carta); solo penaliza, no bloquea.
    Lado WRO solo con |along|≤250 mm.
    """
    R = prm.disc_radius_mm
    clear = math.inf
    soft = math.inf
    fx, fy = math.sin(th), math.cos(th)
    discs = _disc_centers(x, y, th, prm.disc_offsets_mm)

    def _body(qx, qy):
        wall = OUTER_HALF - max(abs(qx), abs(qy)) - R - prm.wall_margin_mm
        cx = min(max(qx, -ISLAND_HALF), ISLAND_HALF)
        cy = min(max(qy, -ISLAND_HALF), ISLAND_HALF)
        if abs(qx) <= ISLAND_HALF and abs(qy) <= ISLAND_HALF:
            isl = -min(ISLAND_HALF - abs(qx), ISLAND_HALF - abs(qy)) - R - prm.wall_margin_mm
        else:
            isl = math.hypot(qx - cx, qy - cy) - R - prm.wall_margin_mm
        return min(wall, isl)

    def _pillar_clear(qx, qy, px, py, col, apply_side: bool) -> float:
        d = math.hypot(qx - px, qy - py) - R - _PILLAR_CIRC - prm.pillar_margin_mm
        if not apply_side:
            return d
        # Solo pilares cerca del carro (along±250 y |lat|≤400). Sin el tope
        # lateral, un pilar en otra recta con el mismo along (p.ej. mismo x)
        # invalidaba todo el arranque.
        along = (px - x) * fx + (py - y) * fy
        left = fx * (py - y) - fy * (px - x)
        if abs(along) > 250.0 or abs(left) > 400.0:
            return d
        need = R + _PILLAR_CIRC + prm.pillar_margin_mm
        side = (left - need) if col == "Red" else (-left - need)
        return min(d, side)

    for qx, qy in discs:
        clear = min(clear, _body(qx, qy))
        for px, py, col in pillars:
            clear = min(clear, _pillar_clear(qx, qy, px, py, col, True))
        for px, py, col in (soft_pillars or ()):
            soft = min(soft, _pillar_clear(qx, qy, px, py, col, True))
    return float(clear), float(soft)


def _step_time(ds: float, kappa: float, v0: float) -> float:
    d = math.atan(WHEELBASE_MM * kappa)
    v = v0 * max(1.0 - SCRUB_COEFF * (d / DELTA_MAX_RAD) ** 2, 0.2)
    return ds / v


def _path_time_s(x: np.ndarray, y: np.ndarray, v0: float) -> float:
    """Tiempo de tramo abierto (no enrolla el último punto con el primero)."""
    if len(x) < 2:
        return 0.0
    p = np.stack([x, y], axis=1)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    # κ por cuerda central en el interior
    t1 = p[1:-1] - p[:-2]
    t2 = p[2:] - p[1:-1]
    ang = np.arctan2(t1[:, 0] * t2[:, 1] - t1[:, 1] * t2[:, 0],
                     np.sum(t1 * t2, axis=1))
    ds_k = 0.5 * (np.linalg.norm(t1, axis=1) + np.linalg.norm(t2, axis=1))
    kap = np.zeros(len(p))
    kap[1:-1] = ang / np.maximum(ds_k, 1e-9)
    # κ por segmento = promedio de nodos
    k_seg = 0.5 * (kap[:-1] + kap[1:])
    d = np.arctan(WHEELBASE_MM * k_seg)
    v = v0 * np.maximum(1.0 - SCRUB_COEFF * (d / DELTA_MAX_RAD) ** 2, 0.2)
    return float(np.sum(seg / v))


def _bins(prm: LatticeParams):
    kmax = prm.kappa_max * prm.kappa_margin
    n_k = prm.n_kappa if prm.n_kappa % 2 == 1 else prm.n_kappa + 1
    kappas = np.linspace(-kmax, kmax, n_k)
    a_vals = np.arange(prm.a_min_mm, prm.a_max_mm + 0.5 * prm.a_step_mm, prm.a_step_mm)
    n_psi = int(round(2 * prm.psi_max_deg / prm.psi_step_deg)) + 1
    psi_vals = np.radians(np.linspace(-prm.psi_max_deg, prm.psi_max_deg, n_psi))
    return kappas, a_vals, psi_vals


def _quantize(a, psi, k_idx, a_vals, psi_vals):
    ai = int(np.clip(np.argmin(np.abs(a_vals - a)), 0, len(a_vals) - 1))
    pi = int(np.clip(np.argmin(np.abs(psi_vals - psi)), 0, len(psi_vals) - 1))
    return ai, pi, int(k_idx)


def _edge_cost(ds, k_new, k_old, clear, prm: LatticeParams,
               soft_clear: float = math.inf) -> float | None:
    """Costo de arista, o None si la pose viola holgura dura."""
    if clear < prm.hard_clear_mm:
        return None
    t = _step_time(ds, k_new, prm.v0_mm_s)
    kmax = max(prm.kappa_max * prm.kappa_margin, 1e-9)
    t += prm.w_dk * abs(k_new - k_old) / kmax
    t -= prm.w_clear * min(max(clear, 0.0), prm.clear_cap_mm)
    # lookahead: pagar violación, no cortar el DAG
    if soft_clear < 0.0:
        t += 0.02 * (-soft_clear)
    else:
        t -= 0.25 * prm.w_clear * min(soft_clear, prm.clear_cap_mm)
    return t


def _integrate(a, psi, k_new, dpsi_ref, ds: float):
    """Un paso Frenet: dψ_ref = cambio de rumbo de la referencia en el tramo."""
    k_ref = dpsi_ref / max(ds, 1e-9)
    a2 = a + ds * math.sin(psi)
    psi2 = psi + ds * (k_new - k_ref)
    return a2, psi2


def _corner_cost_to_go(idx_corner: list[int], c, t, nrm, dpsi_ref, pillars, prm,
                      kappas, a_vals, psi_vals, soft_pillars=None) -> dict:
    """DP backward en la esquina: estado (j, ai, pi, ki) → costo a la meta.

    j=0 es la primera estación del arco; j=len-1 la salida.
    """
    J = len(idx_corner)
    if J < 2:
        return {}
    Na, Np, Nk = len(a_vals), len(psi_vals), len(kappas)
    INF = 1e18
    # cost[j][ai,pi,ki]
    cost = [np.full((Na, Np, Nk), INF) for _ in range(J)]
    parent = [np.full((Na, Np, Nk), -1, dtype=np.int32) for _ in range(J)]

    i_end = idx_corner[-1]
    for ai in range(Na):
        for pi in range(Np):
            if abs(math.degrees(psi_vals[pi])) > prm.exit_psi_tol_deg:
                continue
            if abs(a_vals[ai]) > prm.exit_a_band_mm:
                continue
            for ki in range(Nk):
                x, y, th = _pose(c, t, nrm, i_end, a_vals[ai], psi_vals[pi])
                cl, _ = _clearance_pose(x, y, th, pillars, prm, soft_pillars)
                if cl < prm.hard_clear_mm:
                    continue
                cost[-1][ai, pi, ki] = 0.0

    for j in range(J - 2, -1, -1):
        i = idx_corner[j]
        ds = math.hypot(c[idx_corner[j + 1], 0] - c[i, 0],
                        c[idx_corner[j + 1], 1] - c[i, 1])
        ds = max(ds, 1.0)
        dpr = dpsi_ref[j]
        for ai in range(Na):
            for pi in range(Np):
                for ki in range(Nk):
                    a, psi, k_old = a_vals[ai], psi_vals[pi], kappas[ki]
                    x, y, th = _pose(c, t, nrm, i, a, psi)
                    cl0, _ = _clearance_pose(x, y, th, pillars, prm, soft_pillars)
                    if cl0 < prm.hard_clear_mm:
                        continue
                    for kn in range(Nk):
                        a2, psi2 = _integrate(a, psi, kappas[kn], dpr, ds)
                        if a2 < prm.a_min_mm or a2 > prm.a_max_mm:
                            continue
                        if abs(psi2) > math.radians(prm.psi_max_deg) + 1e-6:
                            continue
                        ai2, pi2, _ = _quantize(a2, psi2, kn, a_vals, psi_vals)
                        nxt = cost[j + 1][ai2, pi2, kn]
                        if nxt >= INF / 2:
                            continue
                        x2, y2, th2 = _pose(c, t, nrm, idx_corner[j + 1],
                                            a_vals[ai2], psi_vals[pi2])
                        cl, soft = _clearance_pose(x2, y2, th2, pillars, prm,
                                                   soft_pillars)
                        ec = _edge_cost(ds, kappas[kn], k_old, cl, prm, soft)
                        if ec is None:
                            continue
                        tot = ec + nxt
                        if tot < cost[j][ai, pi, ki]:
                            cost[j][ai, pi, ki] = tot
                            parent[j][ai, pi, ki] = ai2 + Na * (pi2 + Np * kn)
    table = {"cost": cost, "parent": parent, "idx": idx_corner,
             "a_vals": a_vals, "psi_vals": psi_vals, "kappas": kappas}
    return table


def _reconstruct_corner(table, ai, pi, ki) -> list[tuple[int, int, int, int]]:
    """Lista (j, ai, pi, ki) desde entrada de esquina hasta salida."""
    cost, parent = table["cost"], table["parent"]
    Na, Np = len(table["a_vals"]), len(table["psi_vals"])
    path = [(0, ai, pi, ki)]
    j = 0
    while j < len(cost) - 1:
        code = int(parent[j][ai, pi, ki])
        if code < 0:
            break
        kn = code // (Na * Np)
        rem = code % (Na * Np)
        pi2 = rem // Na
        ai2 = rem % Na
        j += 1
        ai, pi, ki = ai2, pi2, kn
        path.append((j, ai, pi, ki))
    return path


def plan_card(direction: str, section: str, pillars,
              params: LatticeParams | None = None,
              start_state: tuple[float, float, float] | None = None,
              *, strict_start: bool = False,
              soft_pillars=None) -> CardRoute:
    """Óptimo en malla para una carta: A* en recta + DP cost-to-go en esquina.

    start_state: (a_mm, psi_rad, kappa) en el marco Frenet del inicio del tramo.
    soft_pillars: pilares de la carta siguiente (penalización, no hard).
    Elige el mejor entre A*+DP y DP joint (lado WRO, clear, tiempo).
    """
    prm = params or LatticeParams()
    pil = _norm_pillars(pillars)
    soft = _norm_pillars(soft_pillars) if soft_pillars else None
    t0 = time.perf_counter()
    i0, i_c, i1, c, t, nrm, _ds_nom = section_index_range(
        direction, section, prm.ds_mm)
    n = len(c)
    idx = _unwrap_indices(i0, i1, n)
    if len(idx) < 4:
        raise RuntimeError(f"tramo vacío: {direction} {section}")
    # Separar recta / esquina
    try:
        j_c = idx.index(i_c)
    except ValueError:
        j_c = max(1, len(idx) * 2 // 3)
    idx_straight = idx[: j_c + 1]
    idx_corner = idx[j_c:]
    if len(idx_corner) < 2:
        idx_corner = idx[-max(3, len(idx) // 4):]
        idx_straight = idx[: len(idx) - len(idx_corner) + 1]

    dpsi_all = _kappa_ref(t, idx)
    # dpsi por tramo j -> j+1 indexado en idx
    dpsi_s = dpsi_all[: len(idx_straight) - 1]
    dpsi_c = _kappa_ref(t, idx_corner)

    kappas, a_vals, psi_vals = _bins(prm)
    Na, Np, Nk = len(a_vals), len(psi_vals), len(kappas)

    corner_tbl = _corner_cost_to_go(
        idx_corner, c, t, nrm, dpsi_c, pil, prm, kappas, a_vals, psi_vals, soft)

    # A* en la recta: estado (j, ai, pi, ki); j índice en idx_straight.
    INF = 1e18
    Js = len(idx_straight)

    def h_corner(ai, pi, ki):
        if not corner_tbl:
            return 0.0
        return float(corner_tbl["cost"][0][ai, pi, ki])

    gscore: dict = {}
    parent: dict = {}
    pq: list = []
    i_start = idx_straight[0]
    starts = _seed_starts(start_state, a_vals, psi_vals, kappas, c, t, nrm,
                          i_start, pil, prm, strict=strict_start)
    for ai, pi, ki in starts:
        key = (0, ai, pi, ki)
        gscore[key] = 0.0
        heapq.heappush(pq, (h_corner(ai, pi, ki), 0.0, 0, ai, pi, ki))

    best_goal = None
    best_f = INF
    r_astar = None

    if pq:
        while pq:
            f, g, j, ai, pi, ki = heapq.heappop(pq)
            if g > gscore.get((j, ai, pi, ki), INF) + 1e-12:
                continue
            if j == Js - 1:
                hc = h_corner(ai, pi, ki)
                if hc < INF / 2 and g + hc < best_f:
                    best_f = g + hc
                    best_goal = (j, ai, pi, ki, g)
                continue
            i = idx_straight[j]
            i2 = idx_straight[j + 1]
            ds = max(math.hypot(c[i2, 0] - c[i, 0], c[i2, 1] - c[i, 1]), 1.0)
            dpr = dpsi_s[j] if j < len(dpsi_s) else 0.0
            a, psi, k_old = a_vals[ai], psi_vals[pi], kappas[ki]
            for kn in range(Nk):
                a2, psi2 = _integrate(a, psi, kappas[kn], dpr, ds)
                if a2 < prm.a_min_mm or a2 > prm.a_max_mm:
                    continue
                if abs(psi2) > math.radians(prm.psi_max_deg) + 1e-6:
                    continue
                ai2, pi2, _ = _quantize(a2, psi2, kn, a_vals, psi_vals)
                x2, y2, th2 = _pose(c, t, nrm, i2, a_vals[ai2], psi_vals[pi2])
                cl, scl = _clearance_pose(x2, y2, th2, pil, prm, soft)
                ec = _edge_cost(ds, kappas[kn], k_old, cl, prm, scl)
                if ec is None:
                    continue
                ng = g + ec
                key = (j + 1, ai2, pi2, kn)
                hc = h_corner(ai2, pi2, kn)
                if hc >= INF / 2 and j + 1 == Js - 1:
                    continue
                if ng < gscore.get(key, INF):
                    gscore[key] = ng
                    parent[key] = (j, ai, pi, ki)
                    heapq.heappush(pq, (ng + (0.0 if hc >= INF / 2 else hc), ng,
                                       j + 1, ai2, pi2, kn))

        if best_goal is not None:
            j, ai, pi, ki, g_s = best_goal
            straight_states = [(j, ai, pi, ki)]
            key = (j, ai, pi, ki)
            while key in parent:
                key = parent[key]
                straight_states.append(key)
            straight_states.reverse()
            corner_path = _reconstruct_corner(corner_tbl, ai, pi, ki)
            states_idx, states_a, states_psi, states_k = [], [], [], []
            for (jj, aai, ppi, kki) in straight_states:
                states_idx.append(idx_straight[jj])
                states_a.append(a_vals[aai])
                states_psi.append(psi_vals[ppi])
                states_k.append(kappas[kki])
            for (jj, aai, ppi, kki) in corner_path[1:]:
                states_idx.append(idx_corner[jj])
                states_a.append(a_vals[aai])
                states_psi.append(psi_vals[ppi])
                states_k.append(kappas[kki])
            r_astar = _pack_route(states_idx, states_a, states_psi, states_k,
                                  c, t, nrm, direction, section, pil, prm, t0,
                                  method="astar+dp")

    r_dp = None
    for st, strict in ((start_state, strict_start),
                       (start_state, False),
                       (None, False)):
        try:
            r_dp = _plan_card_forward_dp(direction, section, pil, prm, t0,
                                         start_state=st, strict_start=strict,
                                         soft_pillars=soft)
            break
        except RuntimeError:
            if st is None:
                raise
            continue
    cands = [r for r in (r_astar, r_dp) if r is not None]
    if not cands:
        raise RuntimeError(f"sin ruta en {direction}/{section}")
    return _pick_best_card(cands, pil)


def _seed_starts(start_state, a_vals, psi_vals, kappas, c, t, nrm, i_start,
                 pil, prm: LatticeParams, *, strict: bool = False
                 ) -> list[tuple[int, int, int]]:
    """Estados iniciales (ai, pi, ki) factibles.

    strict=True (encadenado): solo el vecindario del empalme; si choca, se
    acepta el bin más cercano igual (el tramo anterior ya comprometió la pose).
    """
    Na, Np, Nk = len(a_vals), len(psi_vals), len(kappas)
    out: list[tuple[int, int, int]] = []
    if start_state is None:
        pi0 = int(np.argmin(np.abs(psi_vals - 0.0)))
        ki0 = int(np.argmin(np.abs(kappas - 0.0)))
        for ai in range(Na):
            x0, y0, th0 = _pose(c, t, nrm, i_start, a_vals[ai], psi_vals[pi0])
            cl, _ = _clearance_pose(x0, y0, th0, pil, prm)
            if cl < prm.hard_clear_mm:
                continue
            out.append((ai, pi0, ki0))
        return out
    a0, psi0, k0 = start_state
    ai0, pi0, _ = _quantize(a0, psi0, 0, a_vals, psi_vals)
    ki0 = int(np.argmin(np.abs(kappas - k0)))
    # Encadenado: ±2 bins (~80 mm) para absorber el marco Frenet al cambiar de recta.
    rad = 2 if strict else 1
    for dai in range(-rad, rad + 1):
        for dpi in range(-rad, rad + 1):
            ai = int(np.clip(ai0 + dai, 0, Na - 1))
            pi = int(np.clip(pi0 + dpi, 0, Np - 1))
            x0, y0, th0 = _pose(c, t, nrm, i_start, a_vals[ai], psi_vals[pi])
            cl, _ = _clearance_pose(x0, y0, th0, pil, prm)
            if cl < prm.hard_clear_mm:
                continue
            out.append((ai, pi, ki0))
    if not out and not strict:
        out.append((ai0, pi0, ki0))
    # strict sin pose factible: devolver vacío → el llamador afloja
    return out


def _plan_card_forward_dp(direction, section, pil, prm, t0,
                          start_state=None, *, strict_start: bool = False,
                          soft_pillars=None) -> CardRoute:
    """DP forward en todo el tramo (fallback / candidato joint)."""
    i0, _i_c, i1, c, t, nrm, _ = section_index_range(direction, section, prm.ds_mm)
    idx = _unwrap_indices(i0, i1, len(c))
    dpsi = _kappa_ref(t, idx)
    kappas, a_vals, psi_vals = _bins(prm)
    Na, Np, Nk = len(a_vals), len(psi_vals), len(kappas)
    J = len(idx)
    INF = 1e18
    cost = [np.full((Na, Np, Nk), INF) for _ in range(J)]
    parent = [np.full((Na, Np, Nk), -1, dtype=np.int32) for _ in range(J)]
    seeds = _seed_starts(start_state, a_vals, psi_vals, kappas,
                         c, t, nrm, idx[0], pil, prm, strict=strict_start)
    if not seeds:
        raise RuntimeError(
            f"DP infactible en {direction}/{section} (arranque)")
    for ai0, pi0, ki0 in seeds:
        cost[0][ai0, pi0, ki0] = 0.0

    for j in range(J - 1):
        i, i2 = idx[j], idx[j + 1]
        ds = max(math.hypot(c[i2, 0] - c[i, 0], c[i2, 1] - c[i, 1]), 1.0)
        dpr = dpsi[j]
        for ai in range(Na):
            for pi in range(Np):
                for ki in range(Nk):
                    g = cost[j][ai, pi, ki]
                    if g >= INF / 2:
                        continue
                    a, psi, k_old = a_vals[ai], psi_vals[pi], kappas[ki]
                    for kn in range(Nk):
                        a2, psi2 = _integrate(a, psi, kappas[kn], dpr, ds)
                        if a2 < prm.a_min_mm or a2 > prm.a_max_mm:
                            continue
                        if abs(psi2) > math.radians(prm.psi_max_deg) + 1e-6:
                            continue
                        ai2, pi2, _ = _quantize(a2, psi2, kn, a_vals, psi_vals)
                        x2, y2, th2 = _pose(c, t, nrm, i2, a_vals[ai2], psi_vals[pi2])
                        cl, scl = _clearance_pose(x2, y2, th2, pil, prm, soft_pillars)
                        ec = _edge_cost(ds, kappas[kn], k_old, cl, prm, scl)
                        if ec is None:
                            continue
                        ng = g + ec
                        if ng < cost[j + 1][ai2, pi2, kn]:
                            cost[j + 1][ai2, pi2, kn] = ng
                            parent[j + 1][ai2, pi2, kn] = ai + Na * (pi + Np * ki)

    # Meta: |psi| y |a| en banda; si no, cualquier estado alcanzable.
    best = INF
    end = None
    for ai in range(Na):
        for pi in range(Np):
            if abs(math.degrees(psi_vals[pi])) > prm.exit_psi_tol_deg:
                continue
            if abs(a_vals[ai]) > prm.exit_a_band_mm:
                continue
            for ki in range(Nk):
                if cost[-1][ai, pi, ki] < best:
                    best = cost[-1][ai, pi, ki]
                    end = (ai, pi, ki)
    if end is None or best >= INF / 2:
        flat = cost[-1].reshape(-1)
        end_i = int(np.argmin(flat))
        if flat[end_i] >= INF / 2:
            raise RuntimeError(
                f"DP infactible en {direction}/{section} (sin camino en la malla)")
        ki = end_i % Nk
        rem = end_i // Nk
        # flat index = ((ai * Np) + pi) * Nk + ki  — reshape (Na,Np,Nk)
        ai = end_i // (Np * Nk)
        rem = end_i % (Np * Nk)
        pi = rem // Nk
        ki = rem % Nk
        end = (ai, pi, ki)

    ai, pi, ki = end
    states = [(J - 1, ai, pi, ki)]
    for j in range(J - 1, 0, -1):
        code = int(parent[j][ai, pi, ki])
        if code < 0:
            break
        # packed as ai + Na*(pi + Np*ki)
        ki_prev = code // (Na * Np)
        rem = code % (Na * Np)
        pi_prev = rem // Na
        ai_prev = rem % Na
        ai, pi, ki = ai_prev, pi_prev, ki_prev
        states.append((j - 1, ai, pi, ki))
    states.reverse()

    states_idx = [idx[j] for (j, _, _, _) in states]
    states_a = [a_vals[ai] for (_, ai, _, _) in states]
    states_psi = [psi_vals[pi] for (_, _, pi, _) in states]
    states_k = [kappas[ki] for (_, _, _, ki) in states]
    return _pack_route(states_idx, states_a, states_psi, states_k, c, t, nrm,
                       direction, section, pil, prm, t0, method="dp")


def _pack_route(states_idx, states_a, states_psi, states_k, c, t, nrm,
                direction, section, pil, prm, t0, method) -> CardRoute:
    xs, ys, hds, ks, ss, aa = [], [], [], [], [], []
    s_acc = 0.0
    for j, i in enumerate(states_idx):
        a, psi, kap = states_a[j], states_psi[j], states_k[j]
        x, y, th = _pose(c, t, nrm, i, a, psi)
        if j:
            s_acc += math.hypot(x - xs[-1], y - ys[-1])
        xs.append(x)
        ys.append(y)
        hds.append(math.degrees(th) % 360.0)
        ks.append(kap)
        ss.append(s_acc)
        aa.append(a)
    x = np.asarray(xs)
    y = np.asarray(ys)
    T = _path_time_s(x, y, prm.v0_mm_s)
    cl = rp.clearances(x, y, pil, closed=False)
    clear = float(min(cl["pillar"], cl["wall"], cl["island"]))
    kmax = float(np.max(np.abs(ks))) if len(ks) else 0.0
    k_lim = prm.kappa_max * prm.kappa_margin * 1.05
    return CardRoute(
        x=x, y=y, heading_deg=np.asarray(hds), kappa=np.asarray(ks),
        s=np.asarray(ss), a=np.asarray(aa), section=section, direction=direction,
        length_mm=float(s_acc), time_s=float(T), clear_mm=clear,
        kappa_ok=kmax <= prm.kappa_max * 1.05,
        solve_ms=(time.perf_counter() - t0) * 1e3, method=method,
    )


def centerline_card(direction: str, section: str, pillars,
                    params: LatticeParams | None = None) -> CardRoute:
    """Línea media del carril en el mismo tramo (baseline a=0)."""
    prm = params or LatticeParams()
    pil = _norm_pillars(pillars)
    t0 = time.perf_counter()
    i0, _ic, i1, c, t, nrm, _ = section_index_range(direction, section, prm.ds_mm)
    idx = _unwrap_indices(i0, i1, len(c))
    states_a = [0.0] * len(idx)
    states_psi = [0.0] * len(idx)
    # κ de referencia
    dpsi = _kappa_ref(t, idx)
    ks = []
    for j in range(len(idx)):
        if j < len(dpsi):
            i, i2 = idx[j], idx[min(j + 1, len(idx) - 1)]
            ds = max(math.hypot(c[i2, 0] - c[i, 0], c[i2, 1] - c[i, 1]), 1.0)
            ks.append(dpsi[j] / ds)
        else:
            ks.append(0.0)
    return _pack_route(idx, states_a, states_psi, ks, c, t, nrm,
                       direction, section, pil, prm, t0, method="center")


def score_route(route: CardRoute | LapRoute) -> dict:
    """Métrica compacta para comparar líneas del mismo tramo."""
    return {
        "time_s": route.time_s,
        "length_mm": route.length_mm,
        "clear_mm": route.clear_mm,
        "kmax": float(np.abs(route.kappa).max()) if len(route.kappa) else 0.0,
        "kappa_ok": route.kappa_ok,
        "method": route.method,
        "solve_ms": route.solve_ms,
    }


def _rank_card(route: CardRoute, pillars) -> tuple:
    """Orden total: lado WRO → clear≥25 → menos tiempo → más holgura."""
    side = rp.side_ok(route.x, route.y, pillars, closed=False)
    clear_ok = route.clear_mm >= 25.0
    return (side, clear_ok, -route.time_s, route.clear_mm, route.kappa_ok)


def _pick_best_card(cands: list[CardRoute], pillars) -> CardRoute:
    if not cands:
        raise RuntimeError("sin candidatos de ruta")
    if len(cands) == 1:
        return cands[0]
    best = max(cands, key=lambda r: _rank_card(r, pillars))
    others = [r.method for r in cands if r is not best]
    if others:
        best.method = f"{best.method}>" + ",".join(others)
    best.solve_ms = max(r.solve_ms for r in cands)
    return best


def plan_lap_cards(direction: str, pillars_by_section: dict,
                   start_section: str = "N",
                   params: LatticeParams | None = None,
                   card_numbers: dict | None = None) -> LapRoute:
    """Encadena las 4 cartas (recta+esquina).

    Cada carta ve pilares propios + los de la siguiente (lookahead de esquina),
    para que la salida no deje un a incompatible con el primer paso WRO.
    El arranque de la carta k>0 es estricto (pose final de k-1).
    """
    prm = params or LatticeParams()
    t0 = time.perf_counter()
    order = section_order(direction, start_section)
    pieces: list[CardRoute] = []
    start_state = None
    all_pil = []
    for sec in order:
        nxt = _NEXT[direction][sec]
        own = _norm_pillars(pillars_by_section.get(sec, []))
        look = _norm_pillars(pillars_by_section.get(nxt, []))
        all_pil.extend(own)
        r = plan_card(direction, sec, own, prm, start_state=start_state,
                      strict_start=start_state is not None,
                      soft_pillars=look)
        if card_numbers and sec in card_numbers:
            r.card = card_numbers[sec]
        pieces.append(r)
        i0, _ic, _i1, c, t, nrm, _ = section_index_range(direction, nxt, prm.ds_mm)
        a, psi = frenet_from_pose(c, t, nrm, i0, float(r.x[-1]), float(r.y[-1]),
                                  math.radians(float(r.heading_deg[-1])))
        start_state = (a, psi, float(r.kappa[-1]))

    # Coser: si el gap del nudo es grande, interpolar (no saltar el obstáculo)
    xs, ys, hds, ks, ss = [], [], [], [], []
    s_acc = 0.0

    def _push(x, y, hd, kap):
        nonlocal s_acc
        if xs:
            s_acc += math.hypot(x - xs[-1], y - ys[-1])
        xs.append(x)
        ys.append(y)
        hds.append(hd)
        ks.append(kap)
        ss.append(s_acc)

    for pi, r in enumerate(pieces):
        if pi > 0:
            x_prev, y_prev = xs[-1], ys[-1]
            gap = math.hypot(float(r.x[0]) - x_prev, float(r.y[0]) - y_prev)
            if gap > 30.0:
                n_ins = max(2, int(gap / 40.0))
                for k in range(1, n_ins):
                    u = k / n_ins
                    _push((1 - u) * x_prev + u * float(r.x[0]),
                          (1 - u) * y_prev + u * float(r.y[0]),
                          float(r.heading_deg[0]), float(r.kappa[0]))
                j0 = 0
            else:
                j0 = 1
        else:
            j0 = 0
        for j in range(j0, len(r.x)):
            _push(float(r.x[j]), float(r.y[j]),
                  float(r.heading_deg[j]), float(r.kappa[j]))

    x = np.asarray(xs)
    y = np.asarray(ys)
    T = _path_time_s(x, y, prm.v0_mm_s)
    cl = rp.clearances(x, y, all_pil, closed=False)
    clear = float(min(cl["pillar"], cl["wall"], cl["island"]))
    kmax = float(np.max(np.abs(ks))) if ks else 0.0
    methods = "+".join(p.method.split(">")[0] for p in pieces)
    return LapRoute(
        x=x, y=y, heading_deg=np.asarray(hds), kappa=np.asarray(ks),
        s=np.asarray(ss), direction=direction, sections=tuple(order),
        cards=pieces, length_mm=float(s_acc), time_s=float(T), clear_mm=clear,
        kappa_ok=kmax <= prm.kappa_max * 1.05 and all(p.kappa_ok for p in pieces),
        solve_ms=(time.perf_counter() - t0) * 1e3, method=f"chain:{methods}",
    )


def plan_lap_joint(direction: str, pillars,
                   start_section: str = "N",
                   params: LatticeParams | None = None) -> LapRoute:
    """Una vuelta continua: DP forward en todo el anillo (sin empalme carta-a-carta).

    Evita el gap N→E del chain local. La polilínea es la que PP seguiría en
    vueltas 2–3 con el mapa congelado.
    """
    prm = params or LatticeParams()
    pil = _norm_pillars(pillars)
    t0 = time.perf_counter()
    c, t, nrm, total = reference_loop(direction, prm.ds_mm)
    n = len(c)
    i0, _ic, _i1, _, _, _, _ = section_index_range(direction, start_section, prm.ds_mm)
    # Una vuelta: i0, i0+1, … sin repetir el cierre (abierta; clear closed=False)
    idx = [(i0 + k) % n for k in range(n)]
    dpsi = _kappa_ref(t, idx)
    kappas, a_vals, psi_vals = _bins(prm)
    Na, Np, Nk = len(a_vals), len(psi_vals), len(kappas)
    J = len(idx)
    INF = 1e18
    cost = [np.full((Na, Np, Nk), INF) for _ in range(J)]
    parent = [np.full((Na, Np, Nk), -1, dtype=np.int32) for _ in range(J)]

    for ai0, pi0, ki0 in _seed_starts(None, a_vals, psi_vals, kappas,
                                      c, t, nrm, idx[0], pil, prm):
        cost[0][ai0, pi0, ki0] = 0.0
    if not np.isfinite(cost[0]).any() or cost[0].min() >= INF / 2:
        raise RuntimeError(f"DP joint infactible al arranque ({direction}/{start_section})")

    for j in range(J - 1):
        i, i2 = idx[j], idx[j + 1]
        ds = max(math.hypot(c[i2, 0] - c[i, 0], c[i2, 1] - c[i, 1]), 1.0)
        dpr = dpsi[j] if j < len(dpsi) else 0.0
        for ai in range(Na):
            for pi in range(Np):
                for ki in range(Nk):
                    g = cost[j][ai, pi, ki]
                    if g >= INF / 2:
                        continue
                    a, psi, k_old = a_vals[ai], psi_vals[pi], kappas[ki]
                    for kn in range(Nk):
                        a2, psi2 = _integrate(a, psi, kappas[kn], dpr, ds)
                        if a2 < prm.a_min_mm or a2 > prm.a_max_mm:
                            continue
                        if abs(psi2) > math.radians(prm.psi_max_deg) + 1e-6:
                            continue
                        ai2, pi2, _ = _quantize(a2, psi2, kn, a_vals, psi_vals)
                        x2, y2, th2 = _pose(c, t, nrm, i2, a_vals[ai2], psi_vals[pi2])
                        cl, _ = _clearance_pose(x2, y2, th2, pil, prm)
                        ec = _edge_cost(ds, kappas[kn], k_old, cl, prm)
                        if ec is None:
                            continue
                        ng = g + ec
                        if ng < cost[j + 1][ai2, pi2, kn]:
                            cost[j + 1][ai2, pi2, kn] = ng
                            parent[j + 1][ai2, pi2, kn] = ai + Na * (pi + Np * ki)

    # Meta: preferir |psi| chico y a cercano al arranque (casi periódico)
    best = INF
    end = None
    for ai in range(Na):
        for pi in range(Np):
            if abs(math.degrees(psi_vals[pi])) > prm.exit_psi_tol_deg:
                continue
            for ki in range(Nk):
                g = cost[-1][ai, pi, ki]
                if g >= INF / 2:
                    continue
                # bonus suave por cerrar lateral cerca del arranque libre (a≈0)
                g_close = g + 1e-4 * abs(a_vals[ai])
                if g_close < best:
                    best = g_close
                    end = (ai, pi, ki)
    if end is None:
        flat = cost[-1].reshape(-1)
        end_i = int(np.argmin(flat))
        if flat[end_i] >= INF / 2:
            raise RuntimeError(f"DP joint infactible ({direction})")
        ai = end_i // (Np * Nk)
        rem = end_i % (Np * Nk)
        pi = rem // Nk
        ki = rem % Nk
        end = (ai, pi, ki)

    ai, pi, ki = end
    states = [(J - 1, ai, pi, ki)]
    for j in range(J - 1, 0, -1):
        code = int(parent[j][ai, pi, ki])
        if code < 0:
            break
        ki_prev = code // (Na * Np)
        rem = code % (Na * Np)
        pi_prev = rem // Na
        ai_prev = rem % Na
        ai, pi, ki = ai_prev, pi_prev, ki_prev
        states.append((j - 1, ai, pi, ki))
    states.reverse()

    states_idx = [idx[j] for (j, _, _, _) in states]
    states_a = [a_vals[ai] for (_, ai, _, _) in states]
    states_psi = [psi_vals[pi] for (_, _, pi, _) in states]
    states_k = [kappas[ki] for (_, _, _, ki) in states]
    card = _pack_route(states_idx, states_a, states_psi, states_k, c, t, nrm,
                       direction, start_section, pil, prm, t0, method="joint-dp")
    # Empacar como LapRoute (una sola pieza)
    return LapRoute(
        x=card.x, y=card.y, heading_deg=card.heading_deg, kappa=card.kappa,
        s=card.s, direction=direction, sections=tuple(section_order(direction, start_section)),
        cards=[card], length_mm=card.length_mm, time_s=card.time_s,
        clear_mm=card.clear_mm, kappa_ok=card.kappa_ok,
        solve_ms=(time.perf_counter() - t0) * 1e3, method="joint-dp",
    )


def plan_field_lap(field, start_section: str | None = None,
                   params: LatticeParams | None = None,
                   *, mode: str = "joint") -> LapRoute:
    """Vuelta sobre un Field (mapa ya conocido).

    mode='joint' (default): DP en el anillo completo — mejor continuidad.
    mode='chain': 4 cartas cosidas (puede dejar gaps; útil para debug).
    """
    start = start_section or "N"
    if mode == "chain":
        by_sec = {sec: pillars_for_section(field, sec) for sec in _CW_STRAIGHT_SEG}
        return plan_lap_cards(field.direction, by_sec, start, params,
                              card_numbers=dict(field.cards))
    pil = [(s.x, s.y, s.color) for s in field.signs]
    return plan_lap_joint(field.direction, pil, start, params)


def build_line_baseline(direction: str, signs, a0: float = 1200.0
                        ) -> tuple[np.ndarray, np.ndarray]:
    """Línea original DigitalMap._build_line encadenada (mapa confirmado).

    Misma construcción que twin/tests/test_route_planner._baseline: una
    instantánea por recta con asientos ya en confirmed (vueltas 2–3).
    """
    from . import digital_map as dmod
    from . import wro_field as wf

    dm = dmod.DigitalMap()
    if dm.direction is None:
        dm.set_direction(direction)
    dm.confirmed = {(s.section, s.seat): s.color for s in signs}
    dm.in_stall = False
    dm._aligned = True
    order = list(wf.SECTIONS_CW) if direction == "CW" else list(reversed(wf.SECTIONS_CW))
    lats = [0.0] * 4
    pieces: list = [None] * 4
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
    a = np.array([p for pc in pieces for p in pc], dtype=float)
    return a[:, 0], a[:, 1]


def score_xy(x, y, pillars, v0: float = 309.0, closed: bool = False) -> dict:
    """Métricas sobre una polilínea arbitraria (p.ej. _build_line)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    pil = _norm_pillars(pillars)
    T = _path_time_s(x, y, v0) if not closed else rp.lap_time_s(x, y, v0, closed=True)
    cl = rp.clearances(x, y, pil, closed=closed)
    clear = float(min(cl["pillar"], cl["wall"], cl["island"]))
    st = rp.path_stats(x, y, closed=closed)
    return {
        "time_s": float(T),
        "length_mm": float(st["length"]),
        "clear_mm": clear,
        "kmax": float(st["kmax"]),
        "kappa_ok": float(st["kmax"]) <= KAPPA_MAX * 1.2,
        "side_ok": rp.side_ok(x, y, pil, closed=closed),
        "method": "build_line",
    }


# Motor twin: v ≈ 3.5*(PWM-25) en recta (params.MotorParams).
def v0_from_pwm(pwm: float, k: float = 3.5, pwm0: float = 25.0) -> float:
    return max(0.0, k * (pwm - pwm0))


def pwm_from_v0(v0: float, k: float = 3.5, pwm0: float = 25.0) -> float:
    return pwm0 + v0 / k


def _path_kappa(x, y, closed: bool = False) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(xr, yr, kappa) remuestreados a 10 mm."""
    xr, yr, _ = rp.resample(np.asarray(x, float), np.asarray(y, float), 10.0, closed)
    p = np.stack([xr, yr], axis=1)
    if closed:
        t1 = p - np.roll(p, 4, axis=0)
        t2 = np.roll(p, -4, axis=0) - p
    else:
        t1 = np.zeros_like(p)
        t2 = np.zeros_like(p)
        t1[4:] = p[4:] - p[:-4]
        t2[:-4] = p[4:] - p[:-4]
        t1[:4] = t1[4]
        t2[-4:] = t2[-5]
    ang = np.arctan2(t1[:, 0] * t2[:, 1] - t1[:, 1] * t2[:, 0], np.sum(t1 * t2, axis=1))
    kap = ang / 40.0
    return xr, yr, kap


def path_speed_stats(x, y, v0: float, closed: bool = False) -> dict:
    """v(κ) scrub a lo largo del camino: min/mean y a_lat máx ≈ v²κ."""
    xr, yr, kap = _path_kappa(x, y, closed)
    d = np.arctan(WHEELBASE_MM * kap)
    v = v0 * np.maximum(1.0 - SCRUB_COEFF * (d / DELTA_MAX_RAD) ** 2, 0.2)
    a_lat = np.abs(v * v * kap)   # mm/s²
    return {
        "v_min": float(v.min()),
        "v_mean": float(v.mean()),
        "v_max": float(v.max()),
        "a_lat_max": float(a_lat.max()),
        "kmax": float(np.abs(kap).max()),
    }


def speed_profile(x, y, v_straight: float, a_lat_max: float = 800.0,
                  a_accel: float = 400.0, a_brake: float = 600.0,
                  closed: bool = False) -> dict:
    """Perfil v(s): tope en curva por a_lat, en recta por v_straight; rampa accel/freno.

    a_accel/a_brake en mm/s² (longitudinal). En lona floja el freno suele ser
    peor que el motor; defaults prudentes.
    Devuelve tiempo de vuelta con ese perfil vs scrub a v_straight constante.
    """
    xr, yr, kap = _path_kappa(x, y, closed)
    n = len(xr)
    if n < 3:
        return {"time_s": 0.0, "time_scrub_s": 0.0}
    # ds entre muestras
    dxy = np.hypot(np.diff(xr), np.diff(yr))
    if closed:
        dxy = np.append(dxy, math.hypot(xr[0] - xr[-1], yr[0] - yr[-1]))
    else:
        dxy = np.append(dxy, dxy[-1])

    # Tope local: v ≤ sqrt(a_lat/|κ|) y ≤ v_straight
    eps = 1e-6
    v_lim = np.minimum(v_straight, np.sqrt(a_lat_max / np.maximum(np.abs(kap), eps)))
    v_lim = np.maximum(v_lim, 50.0)   # piso para no clavar a 0

    # Forward: no superar accel
    v = np.zeros(n)
    v[0] = min(v_lim[0], v_straight)
    for i in range(n - 1):
        v_next = math.sqrt(max(0.0, v[i] ** 2 + 2.0 * a_accel * dxy[i]))
        v[i + 1] = min(v_lim[i + 1], v_next)
    if closed:
        # una pasada más para cerrar el anillo
        for i in range(n - 1):
            v_next = math.sqrt(max(0.0, v[i] ** 2 + 2.0 * a_accel * dxy[i]))
            v[i + 1] = min(v_lim[i + 1], v_next, v[i + 1])

    # Backward: respetar freno hacia las curvas
    for i in range(n - 2, -1, -1):
        v_prev = math.sqrt(max(0.0, v[i + 1] ** 2 + 2.0 * a_brake * dxy[i]))
        v[i] = min(v[i], v_lim[i], v_prev)
    if closed:
        for i in range(n - 2, -1, -1):
            v_prev = math.sqrt(max(0.0, v[i + 1] ** 2 + 2.0 * a_brake * dxy[i]))
            v[i] = min(v[i], v_lim[i], v_prev)

    # Tiempo: trapecio en cada segmento
    v_seg = 0.5 * (v + np.roll(v, -1)) if closed else 0.5 * (v[:-1] + v[1:])
    ds = dxy if closed else dxy[:-1]
    if not closed:
        v_seg = 0.5 * (v[:-1] + v[1:])
        ds = dxy[:-1]
    T = float(np.sum(ds / np.maximum(v_seg, 1.0)))

    # Scrub constante a v_straight (misma geometría)
    d = np.arctan(WHEELBASE_MM * kap)
    v_sc = v_straight * np.maximum(1.0 - SCRUB_COEFF * (d / DELTA_MAX_RAD) ** 2, 0.2)
    if closed:
        v_sc_seg = 0.5 * (v_sc + np.roll(v_sc, -1))
        T_sc = float(np.sum(dxy / np.maximum(v_sc_seg, 1.0)))
    else:
        v_sc_seg = 0.5 * (v_sc[:-1] + v_sc[1:])
        T_sc = float(np.sum(dxy[:-1] / np.maximum(v_sc_seg, 1.0)))

    return {
        "time_s": T,
        "time_scrub_s": T_sc,
        "v_min": float(v.min()),
        "v_mean": float(v.mean()),
        "v_max": float(v.max()),
        "v_straight": float(v_straight),
        "a_lat_max_used": float(a_lat_max),
        "a_lat_peak": float(np.max(np.abs(v * v * kap))),
        "frac_straight": float(np.mean(v >= 0.95 * v_straight)),
        "kmax": float(np.abs(kap).max()),
    }


def sweep_v0_joint(direction: str, pillars, start_section: str = "N",
                   v0s: list[float] | None = None,
                   clear_min_mm: float = 15.0,
                   a_lat_max: float | None = None,
                   params: LatticeParams | None = None) -> list[dict]:
    """Replan joint a cada v0. Factible = κ_ok ∧ clear≥mín [∧ a_lat≤tope].

    a_lat ≈ v²κ (mm/s²). Piso WRO = lona lisa → μ bajo (~0.08–0.12);
    default del CLI ~800 mm/s² (≈0.08 g). Sin eso la geometría casi no corta.
    """
    base = params or LatticeParams()
    if v0s is None:
        v0s = [v0_from_pwm(p) for p in
               (95, 110, 120, 130, 140, 150, 160, 170, 180, 200, 220)]
    rows = []
    for v0 in v0s:
        prm = replace(base, v0_mm_s=float(v0))
        row = {"v0": float(v0), "pwm_eq": pwm_from_v0(v0), "ok": False}
        try:
            lap = plan_lap_joint(direction, pillars, start_section, prm)
            st = path_speed_stats(lap.x, lap.y, v0, closed=False)
            ok = bool(lap.kappa_ok and lap.clear_mm >= clear_min_mm)
            if a_lat_max is not None and st["a_lat_max"] > a_lat_max:
                ok = False
            row.update({
                "ok": ok,
                "time_s": lap.time_s,
                "length_mm": lap.length_mm,
                "clear_mm": lap.clear_mm,
                "kmax": float(np.abs(lap.kappa).max()) if len(lap.kappa) else 0.0,
                "kappa_ok": lap.kappa_ok,
                "v_min": st["v_min"],
                "v_mean": st["v_mean"],
                "a_lat_max": st["a_lat_max"],
                "solve_ms": lap.solve_ms,
                "method": lap.method,
            })
        except Exception as e:
            row["error"] = f"{type(e).__name__}: {e}"
        rows.append(row)
    return rows


def max_v0_feasible(rows: list[dict]) -> dict | None:
    """Mayor v0 con ok=True (None si ninguno)."""
    good = [r for r in rows if r.get("ok")]
    if not good:
        return None
    return max(good, key=lambda r: r["v0"])

