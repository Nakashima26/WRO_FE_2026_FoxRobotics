"""
Planificador offline de la ruta de UNA vuelta (WRO FE 2026, reto de obstáculos).

Módulo puro: solo numpy (sin cv2, sin twin). Se calcula UNA vez, cuando los
asientos del mapa digital quedan confirmados, y el pure pursuit solo la sigue.

Modelo
------
La pista es un anillo de 1000 mm de ancho (isla de 1000x1000 mm, paredes en
+-1500 mm). La referencia es la línea media del carril con las esquinas
redondeadas (arcos de 500 mm alrededor de la esquina de la isla), muestreada
a ds uniforme. El camino es p_i = c_i + a_i n_i (n = normal izquierda del
sentido de marcha) y la incógnita es el desplazamiento lateral a_i, PERIÓDICO
en toda la vuelta (las 3 vueltas son idénticas). Así las 4 rectas y las 4
curvas quedan encadenadas sin condiciones de frontera explícitas: (d, psi) a
la entrada de cada recta es una consecuencia del óptimo global, con
continuidad G1/G2 por construcción.

Problema (convexo en cada iteración, el color ya fija el lado de cada pilar):

    min   sum_i w_k h_i k_i^2  +  mu * longitud
    s.a.  k_i en [-kmax, kmax]                    (|k| <= 1/radio mínimo)
          pilares: el carro pasa por el lado que manda el color
          paredes exteriores e isla con holgura

El carro se modela como 3 discos de radio R (72 mm) a 0, 60 y 120 mm delante
del eje trasero (cubren el rectángulo de 169x132 mm), de modo que la nariz
también respeta pilares y paredes cuando el carro va en diagonal.
La curvatura y la longitud se linealizan/cuadratizan alrededor de la
iteración anterior (SQP, ~6 iteraciones); cada QP se resuelve con ADMM
(estilo OSQP) en numpy puro.

Convenciones: x este, y norte, mm. heading en grados de brújula (0 = +y,
90 = +x, como wro_field/digital_map). curvatura > 0 = giro a la IZQUIERDA.
Colores: "Red" se pasa con el pilar a la IZQUIERDA del carro, "Green" a la
derecha (wro_field: rojo por la derecha, verde por la izquierda).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace

import numpy as np

# ── Pista (mm) ────────────────────────────────────────────────────────────
OUTER_HALF = 1500.0          # pared exterior
ISLAND_HALF = 500.0          # isla de 1000x1000
LANE_CENTER = 1000.0         # línea media del carril
ARC_R = 500.0                # arco de la referencia en cada esquina
PILLAR_HALF = 25.0           # señal 50x50

# ── Carro (config.py: GEOMETRÍA DEL VEHÍCULO) ─────────────────────────────
CAR_REAR_MM = 21.8
CAR_FRONT_MM = 147.6
CAR_HALF_W_MM = 66.0
KAPPA_MAX = 1.0 / 115.0      # radio mínimo al centro del eje trasero

_PILLAR_CIRC = PILLAR_HALF * math.sqrt(2.0)
_COLORS = ("Red", "Green")


@dataclass(frozen=True)
class PlannerParams:
    ds_mm: float = 40.0                          # malla de la optimización
    out_ds_mm: float = 20.0                      # paso de la ruta entregada
    disc_radius_mm: float = 72.0                 # cubre 169x132 con 3 discos
    disc_offsets_mm: tuple = (0.0, 60.0, 120.0)  # delante del eje trasero
    pillar_margin_mm: float = 25.0               # extra sobre disco + pilar
    wall_margin_mm: float = 50.0
    kappa_max: float = KAPPA_MAX
    kappa_margin: float = 0.85                   # usa 85 % de la curvatura máx.
    w_curv: float = 3.0e4                        # mm^2 (curvatura vs longitud)
    w_len: float = 1.0
    sqp_iters: int = 6
    tighten_mm: float = 0.5                      # holgura numérica de ADMM


@dataclass
class Route:
    """Una vuelta. Arrays alineados; el último punto NO repite el primero."""
    x: np.ndarray
    y: np.ndarray
    heading_deg: np.ndarray
    kappa: np.ndarray            # 1/mm, + = izquierda
    s: np.ndarray                # mm recorridos desde el primer punto
    direction: str
    length_mm: float
    solve_ms: float = 0.0
    iterations: int = 0
    max_violation_mm: float = 0.0
    kappa_ok: bool = True

    def points(self) -> list[tuple[float, float, float, float]]:
        return [(float(a), float(b), float(c), float(d))
                for a, b, c, d in zip(self.x, self.y, self.heading_deg, self.kappa)]

    def xy(self) -> np.ndarray:
        return np.stack([self.x, self.y], axis=1)


# ── Referencia (línea media redondeada) ───────────────────────────────────

def _reference_cw(n: int) -> tuple[np.ndarray, np.ndarray, float]:
    """n estaciones a arco uniforme, sentido horario, desde (-500, 1000)."""
    half, rad = ISLAND_HALF, ARC_R
    lane = LANE_CENTER
    segs = [
        ("L", (-half, lane), (1.0, 0.0), 2 * half),
        ("A", (half, half), 90.0, rad),
        ("L", (lane, half), (0.0, -1.0), 2 * half),
        ("A", (half, -half), 0.0, rad),
        ("L", (half, -lane), (-1.0, 0.0), 2 * half),
        ("A", (-half, -half), -90.0, rad),
        ("L", (-lane, -half), (0.0, 1.0), 2 * half),
        ("A", (-half, half), 180.0, rad),
    ]
    lens = [s[3] if s[0] == "L" else math.pi / 2 * s[3] for s in segs]
    total = float(sum(lens))
    edges = np.concatenate([[0.0], np.cumsum(lens)])
    s_all = np.arange(n) * (total / n)
    c = np.zeros((n, 2))
    t = np.zeros((n, 2))
    for k, seg in enumerate(segs):
        m = (s_all >= edges[k]) & (s_all < edges[k + 1] + (1e-9 if k == 7 else 0.0))
        ls = s_all[m] - edges[k]
        if seg[0] == "L":
            p0, d = seg[1], seg[2]
            c[m, 0] = p0[0] + d[0] * ls
            c[m, 1] = p0[1] + d[1] * ls
            t[m] = d
        else:
            ctr, th0, r = seg[1], seg[2], seg[3]
            th = np.radians(th0) - ls / r
            c[m, 0] = ctr[0] + r * np.cos(th)
            c[m, 1] = ctr[1] + r * np.sin(th)
            t[m, 0] = np.sin(th)
            t[m, 1] = -np.cos(th)
    return c, t, total


def reference_loop(direction: str, ds_mm: float = 40.0):
    """(c, t, n, L): estaciones, tangentes, normales izquierdas y largo (mm)."""
    if direction not in ("CW", "CCW"):
        raise ValueError(f"sentido inválido: {direction!r}")
    _, _, total = _reference_cw(8)
    n = int(round(total / ds_mm))
    c, t, total = _reference_cw(n)
    if direction == "CCW":
        c, t = c[::-1].copy(), -t[::-1].copy()
    nrm = np.stack([-t[:, 1], t[:, 0]], axis=1)
    return c, t, nrm, total


# ── Pilares ───────────────────────────────────────────────────────────────

def pillars_from_seats(confirmed) -> list[tuple[float, float, str]]:
    """{(sección, asiento): color} o [(sección, asiento, color)] -> [(x, y, color)].

    Los mismos asientos que wro_field.SEATS / DigitalMap.confirmed."""
    from .wro_field import SEATS, section_to_world
    items = confirmed.items() if isinstance(confirmed, dict) else (
        ((s, a), c) for s, a, c in confirmed)
    out = []
    for (sec, seat), color in items:
        h, w = SEATS[seat]
        x, y = section_to_world(sec, h, w)
        out.append((float(x), float(y), color))
    return out


def _norm_pillars(pillars) -> list[tuple[float, float, str]]:
    out = []
    for p in pillars:
        if hasattr(p, "x") and hasattr(p, "color"):       # wro_field.Sign
            x, y, col = p.x, p.y, p.color
        else:
            x, y, col = p
        if col not in _COLORS:
            raise ValueError(f"color inválido: {col!r}")
        out.append((float(x), float(y), col))
    return out


# ── QP: punto interior primal-dual (Mehrotra), numpy puro ────────────────

def _solve_qp(P, q, A, lo, hi, x0=None, iters=40, tol=1e-4):
    """min 1/2 x'Px + q'x  s.a. lo <= Ax <= hi (cotas +-inf permitidas).

    Devuelve (x, residuo primal en unidades de A). Infeasible-start, así que
    no necesita un punto inicial factible; si el QP es infactible el residuo
    primal queda grande y el llamador lo detecta."""
    n = P.shape[0]
    fin_hi = np.isfinite(hi)
    fin_lo = np.isfinite(lo)
    G = np.vstack([A[fin_hi], -A[fin_lo]])
    h = np.concatenate([hi[fin_hi], -lo[fin_lo]])
    m = len(h)
    x = np.zeros(n) if x0 is None else x0.copy()
    if m == 0:
        x = np.linalg.solve(P + 1e-9 * np.eye(n), -q)
        return x, 0.0
    s = np.maximum(h - G @ x, 1.0)
    lam = np.ones(m)
    res = np.inf
    for _ in range(iters):
        rd = P @ x + q + G.T @ lam
        rp = G @ x + s - h
        mu = float(s @ lam) / m
        res = float(max(np.max(rp), 0.0)) if False else float(np.max(np.abs(rp)))
        if res < tol and mu < tol and np.max(np.abs(rd)) < 1e-3:
            break
        d = lam / s
        Hm = P + (G.T * d) @ G + 1e-10 * np.eye(n)
        try:
            L = np.linalg.cholesky(Hm)
        except np.linalg.LinAlgError:
            Hm = Hm + 1e-6 * np.eye(n)
            L = np.linalg.cholesky(Hm)

        def solve(rc):
            rhs = -rd + G.T @ ((rc - lam * rp) / s)
            dx = np.linalg.solve(L.T, np.linalg.solve(L, rhs))
            ds = -rp - G @ dx
            dl = (-rc - lam * ds) / s
            return dx, ds, dl

        def maxstep(v, dv):
            neg = dv < 0
            return float(min(1.0, np.min(-v[neg] / dv[neg]))) if neg.any() else 1.0

        dxa, dsa, dla = solve(s * lam)
        ap = maxstep(s, dsa)
        ad = maxstep(lam, dla)
        mua = float((s + ap * dsa) @ (lam + ad * dla)) / m
        sig = (mua / max(mu, 1e-30)) ** 3
        dx, ds, dl = solve(s * lam + dsa * dla - sig * mu)
        ap = min(1.0, 0.99 * maxstep(s, ds) if (ds < 0).any() else 1.0)
        ad = min(1.0, 0.99 * maxstep(lam, dl) if (dl < 0).any() else 1.0)
        x = x + ap * dx
        s = s + ap * ds
        lam = lam + ad * dl
    return x, res


# ── Construcción del QP por iteración ────────────────────────────────────

def _rows(idx, g, k, c, nrm):
    """Filas que dan g . (q_i - 0) con q = p_i + k_i (p_{i+1}-p_{i-1}).

    Devuelve (R, const): g . q = R @ alpha + const.  g: (len(idx), 2)."""
    n = len(c)
    ip = (idx + 1) % n
    im = (idx - 1) % n
    R = np.zeros((len(idx), n))
    r = np.arange(len(idx))
    np.add.at(R, (r, idx), np.sum(g * nrm[idx], axis=1))
    np.add.at(R, (r, ip), k * np.sum(g * nrm[ip], axis=1))
    np.add.at(R, (r, im), -k * np.sum(g * nrm[im], axis=1))
    const = np.sum(g * (c[idx] + k[:, None] * (c[ip] - c[im])), axis=1)
    return R, const


def _positions(alpha, c, nrm):
    return c + alpha[:, None] * nrm


def _curv_rows(p, c, nrm):
    """kappa_i = r_i . alpha + b_i, linealizada en el camino actual p."""
    n = len(c)
    a = p - np.roll(p, 1, axis=0)          # chord i-1 -> i
    b = np.roll(p, -1, axis=0) - p         # chord i -> i+1
    la = np.maximum(np.linalg.norm(a, axis=1), 1e-6)
    lb = np.maximum(np.linalg.norm(b, axis=1), 1e-6)
    tc = a / la[:, None] + b / lb[:, None]
    tc /= np.maximum(np.linalg.norm(tc, axis=1), 1e-9)[:, None]
    nc = np.stack([-tc[:, 1], tc[:, 0]], axis=1)
    h = 0.5 * (la + lb)
    inv = 1.0 / (0.5 * (la + lb) ** 2)     # 1/h^2 con h medio
    idx = np.arange(n)
    ip, im = (idx + 1) % n, (idx - 1) % n
    R = np.zeros((n, n))
    np.add.at(R, (idx, im), np.sum(nc * nrm[im], axis=1) * inv)
    np.add.at(R, (idx, idx), -2.0 * np.sum(nc * nrm[idx], axis=1) * inv)
    np.add.at(R, (idx, ip), np.sum(nc * nrm[ip], axis=1) * inv)
    const = np.sum(nc * (c[im] - 2 * c + c[ip]), axis=1) * inv
    return R, const, h


def _length_terms(p, c, nrm):
    """Gradiente y Hessiana (Taylor 2º orden) de la longitud en alpha_k."""
    n = len(c)
    d = np.roll(p, -1, axis=0) - p
    ln = np.maximum(np.linalg.norm(d, axis=1), 1e-6)
    e = d / ln[:, None]
    m = np.stack([-e[:, 1], e[:, 0]], axis=1)
    idx = np.arange(n)
    ip = (idx + 1) % n
    # d_i(alpha) = dc_i + n_{i+1} a_{i+1} - n_i a_i
    Ge = np.zeros((n, n))
    Gm = np.zeros((n, n))
    np.add.at(Ge, (idx, ip), np.sum(e * nrm[ip], axis=1))
    np.add.at(Ge, (idx, idx), -np.sum(e * nrm[idx], axis=1))
    np.add.at(Gm, (idx, ip), np.sum(m * nrm[ip], axis=1))
    np.add.at(Gm, (idx, idx), -np.sum(m * nrm[idx], axis=1))
    grad = Ge.sum(axis=0)
    H = Gm.T @ (Gm / ln[:, None])
    return grad, H, ln


def _build_constraints(p, c, t, nrm, pillars, prm: PlannerParams):
    """Filas (R, lo, hi) lineales en alpha, linealizadas en el camino p."""
    n = len(c)
    chord = np.linalg.norm(np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0), axis=1)
    chord = np.maximum(chord, 1e-6)
    R_d = prm.disc_radius_mm
    rows, los, his = [], [], []
    allidx = np.arange(n)
    inf = np.inf
    for f in prm.disc_offsets_mm:
        k = f / chord
        q_cur = p + k[:, None] * (np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0))
        c_ref = c + f * t
        # paredes exteriores: e . q <= 1500 - (R + margen)
        wmax = OUTER_HALF - (R_d + prm.wall_margin_mm)
        for e in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
            ev = np.array(e)
            near = allidx[(c_ref @ ev) > OUTER_HALF - 800.0]
            if len(near) == 0:
                continue
            g = np.tile(ev, (len(near), 1))
            R, const = _rows(near, g, k[near], c, nrm)
            rows.append(R)
            los.append(np.full(len(near), -inf))
            his.append(wmax - const)
        # isla: plano tangente en el punto más cercano del camino actual
        cp = np.clip(q_cur, -ISLAND_HALF, ISLAND_HALF)
        dv = q_cur - cp
        dist = np.linalg.norm(dv, axis=1)
        near = allidx[dist < 700.0]
        if len(near):
            g = dv[near] / np.maximum(dist[near], 1e-6)[:, None]
            bad = dist[near] < 1e-6
            if bad.any():
                g[bad] = nrm[near][bad] * (-1.0)
            R, const = _rows(near, g, k[near], c, nrm)
            need = np.sum(g * cp[near], axis=1) + R_d + prm.wall_margin_mm
            rows.append(R)
            los.append(need - const)
            his.append(np.full(len(near), inf))
        # pilares: lado por color, en el marco de la estación
        rc = R_d + _PILLAR_CIRC + prm.pillar_margin_mm
        for (px, py, col) in pillars:
            P = np.array([px, py])
            rel = P - c
            u = np.sum(rel * t, axis=1) - f              # a lo largo, desde el disco
            v = np.sum(rel * nrm, axis=1)                # lateral del pilar
            sel = allidx[(np.abs(u) < rc) & (np.abs(v) < 450.0)]
            if len(sel) == 0:
                continue
            w = np.sqrt(np.maximum(rc * rc - u[sel] ** 2, 0.0))
            R, const = _rows(sel, nrm[sel], k[sel], c, nrm)
            # n_i . q - n_i . c_i  es el lateral del disco en el marco de i
            lat_c = np.sum(nrm[sel] * c[sel], axis=1)
            if col == "Red":       # pilar a la izquierda: lateral <= v - w
                hi = lat_c + v[sel] - w - const
                lo = np.full(len(sel), -inf)
            else:                  # verde: lateral >= v + w
                lo = lat_c + v[sel] + w - const
                hi = np.full(len(sel), inf)
            rows.append(R)
            los.append(lo)
            his.append(hi)
    R = np.vstack(rows) if rows else np.zeros((0, n))
    lo = np.concatenate(los) if los else np.zeros(0)
    hi = np.concatenate(his) if his else np.zeros(0)
    return R, lo, hi


def _smooth_curv(x, y, k):
    """Curvatura con signo (+ izquierda): cambio de rumbo entre cuerdas de k muestras
    a cada lado, sobre su distancia media (misma ventana que path_stats)."""
    p = np.stack([x, y], axis=1)
    t1 = p - np.roll(p, k, axis=0)
    t2 = np.roll(p, -k, axis=0) - p
    ang = np.arctan2(t1[:, 0] * t2[:, 1] - t1[:, 1] * t2[:, 0], np.sum(t1 * t2, axis=1))
    dist = 0.5 * (np.linalg.norm(t1, axis=1) + np.linalg.norm(t2, axis=1))
    return ang / np.maximum(dist, 1e-9)


def _heading_curv(x, y):
    """heading de brújula (deg) y curvatura con signo (+ izquierda) en polilínea cerrada."""
    p = np.stack([x, y], axis=1)
    a = p - np.roll(p, 1, axis=0)
    b = np.roll(p, -1, axis=0) - p
    t = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    heading = np.degrees(np.arctan2(t[:, 0], t[:, 1])) % 360.0
    la = np.maximum(np.linalg.norm(a, axis=1), 1e-9)
    lb = np.maximum(np.linalg.norm(b, axis=1), 1e-9)
    lc = np.maximum(np.linalg.norm(t, axis=1), 1e-9)
    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    kappa = 2.0 * cross / (la * lb * lc)
    return heading, kappa


# ── API ───────────────────────────────────────────────────────────────────

def plan_lap(direction: str, pillars, params: PlannerParams | None = None,
             start_section: str | None = None, start_along_mm: float = 0.0) -> Route:
    """Ruta de una vuelta.

    pillars: lista de (x_mm, y_mm, color) o wro_field.Sign, o el dict
        {(sección, asiento): color} de DigitalMap.confirmed (con
        pillars_from_seats). Los asientos no confirmados simplemente no
        restringen.
    start_section/start_along_mm: si se dan, la lista arranca en la estación
        de la línea media más cercana a ese punto de la recta (along desde la
        pared de atrás, como DigitalMap); si no, arranca en (-500, 1000).
    """
    prm = params or PlannerParams()
    if isinstance(pillars, dict):
        pillars = pillars_from_seats(pillars)
    pil = _norm_pillars(pillars)
    t0 = time.perf_counter()
    c, t, nrm, total = reference_loop(direction, prm.ds_mm)
    n = len(c)

    alpha = np.zeros(n)
    state = None
    kmax = prm.kappa_max * prm.kappa_margin
    kappa_ok = True
    iters = 0
    p = _positions(alpha, c, nrm)
    for it in range(prm.sqp_iters):
        iters = it + 1
        Rk, bk, h = _curv_rows(p, c, nrm)
        grad, Hl, _ = _length_terms(p, c, nrm)
        wk = prm.w_curv
        # costo: wk * sum h (Rk a + bk)^2 + w_len * (grad.(a-ak) + 1/2 (a-ak)'Hl(a-ak))
        Pm = 2.0 * wk * (Rk.T * h) @ Rk + prm.w_len * Hl
        qv = 2.0 * wk * (Rk.T * h) @ bk + prm.w_len * (grad - Hl @ alpha)
        Rc, lo_c, hi_c = _build_constraints(p, c, t, nrm, pil, prm)
        lo_c = lo_c + np.where(np.isfinite(lo_c), prm.tighten_mm, 0.0)
        hi_c = hi_c - np.where(np.isfinite(hi_c), prm.tighten_mm, 0.0)
        res = np.inf
        val0 = Rc @ alpha
        keep = np.minimum(val0 - lo_c, hi_c - val0) < 120.0
        kk = np.abs(bk + Rk @ alpha) > 0.5 * kmax
        for use_k in (True, False):
            for _round in range(5):
                Rs, ls, hs = Rc[keep], lo_c[keep], hi_c[keep]
                if use_k:
                    A = np.vstack([Rk[kk], Rs])
                    lo = np.concatenate([(-kmax - bk)[kk], ls])
                    hi = np.concatenate([(kmax - bk)[kk], hs])
                else:
                    A, lo, hi = Rs, ls, hs
                sc = np.maximum(np.linalg.norm(A, axis=1), 1e-12)
                x, res = _solve_qp(Pm, qv, A / sc[:, None], lo / sc, hi / sc, x0=alpha)
                v = Rc @ x
                bad = (v < lo_c - 0.2) | (v > hi_c + 0.2)
                kb = (np.abs(Rk @ x + bk) > kmax + 1e-7) if use_k else kk
                if not ((bad & ~keep).any() or (use_k and (kb & ~kk).any())):
                    break
                keep |= bad
                kk |= kb
            if res < 1e-2 or not use_k:
                kappa_ok = use_k and res < 1e-2
                break
        step = np.max(np.abs(x - alpha))
        alpha = x
        p = _positions(alpha, c, nrm)
        if step < 1.0:
            break

    # Hermite cúbico cerrado (tangente por diferencia central) sobre las estaciones y remuestreo a arco uniforme:
    # el consumidor (pure pursuit) recibe puntos equiespaciados, con rumbo y
    # curvatura calculados sobre la curva suave (no sobre la malla del QP).
    sub = 10
    p1, p2 = p, np.roll(p, -1, axis=0)
    tg = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    tg /= np.maximum(np.linalg.norm(tg, axis=1), 1e-9)[:, None]      # tangente unitaria
    lch = np.linalg.norm(p2 - p1, axis=1)[:, None, None]             # cuerda i -> i+1
    m1, m2 = tg[:, None], np.roll(tg, -1, axis=0)[:, None]
    u = (np.arange(sub) / sub)[None, :, None]
    dense = ((2 * u ** 3 - 3 * u ** 2 + 1) * p1[:, None] + (u ** 3 - 2 * u ** 2 + u) * lch * m1
             + (-2 * u ** 3 + 3 * u ** 2) * p2[:, None] + (u ** 3 - u ** 2) * lch * m2)
    dense = dense.reshape(-1, 2)
    dseg = np.linalg.norm(np.roll(dense, -1, axis=0) - dense, axis=1)
    length = float(dseg.sum())
    sd = np.concatenate([[0.0], np.cumsum(dseg)])
    m = int(round(length / prm.out_ds_mm))
    so = np.arange(m) * (length / m)
    closed = np.vstack([dense, dense[:1]])
    x_ = np.interp(so, sd, closed[:, 0])
    y_ = np.interp(so, sd, closed[:, 1])
    heading, _ = _heading_curv(x_, y_)
    kappa = _smooth_curv(x_, y_, 2)
    s_all = so
    n = m
    p = np.stack([x_, y_], axis=1)

    i0 = 0
    if start_section is not None:
        i0 = _nearest_station(p, direction, start_section, start_along_mm)
    order = (np.arange(n) + i0) % n
    s_out = (s_all[order] - s_all[order][0]) % length
    route = Route(x=x_[order], y=y_[order], heading_deg=heading[order],
                  kappa=kappa[order], s=s_out, direction=direction,
                  length_mm=length)
    kpk = float(np.abs(route.kappa).max())
    if kpk > prm.kappa_max and prm.kappa_margin > 0.55:
        # La curva suavizada pasó el límite aunque las estaciones del QP no:
        # se aprieta el margen y se repite (casi nunca hace falta).
        again = plan_lap(direction, pil, replace(
            prm, kappa_margin=prm.kappa_margin * prm.kappa_max / kpk * 0.97),
            start_section, start_along_mm)
        again.solve_ms += route.solve_ms
        return again
    route.iterations = iters
    route.kappa_ok = kappa_ok
    route.solve_ms = (time.perf_counter() - t0) * 1e3
    route.max_violation_mm = max(0.0, -min_clearance(route, pil) if pil else 0.0)
    return route


def section_along(section: str, direction: str, x: float, y: float) -> tuple[float, float]:
    """(along desde la pared de atrás, lateral derecha) como DigitalMap._section_frame."""
    hd = {"N": 90.0, "E": 180.0, "S": 270.0, "W": 0.0}[section]
    if direction == "CCW":
        hd = (hd + 180.0) % 360.0
    rad = math.radians(hd)
    fx, fy = math.sin(rad), math.cos(rad)
    cx, cy = {"N": (0.0, 1000.0), "E": (1000.0, 0.0),
              "S": (0.0, -1000.0), "W": (-1000.0, 0.0)}[section]
    ox, oy = cx - fx * OUTER_HALF, cy - fy * OUTER_HALF
    dx, dy = x - ox, y - oy
    return dx * fx + dy * fy, dx * math.cos(rad) - dy * math.sin(rad)


def _nearest_station(p, direction, section, along):
    a_l = np.array([section_along(section, direction, x, y) for x, y in p])
    cost = np.abs(a_l[:, 0] - along) + np.where(np.abs(a_l[:, 1]) > 500.0, 1e6, 0.0)
    return int(np.argmin(cost))


# ── Verificación geométrica exacta (huella del carro) ────────────────────

def _car_outline(step=11.0):
    xs = np.arange(-CAR_REAR_MM, CAR_FRONT_MM + 1e-6, step)
    ys = np.arange(-CAR_HALF_W_MM, CAR_HALF_W_MM + 1e-6, step)
    pts = [(a, -CAR_HALF_W_MM) for a in xs] + [(a, CAR_HALF_W_MM) for a in xs]
    pts += [(-CAR_REAR_MM, b) for b in ys] + [(CAR_FRONT_MM, b) for b in ys]
    return np.array(pts)         # (adelante, derecha)


def _outline_world(x, y, heading_deg):
    out = _car_outline()
    rad = np.radians(heading_deg)
    fx, fy = np.sin(rad), np.cos(rad)
    rx, ry = np.cos(rad), -np.sin(rad)
    wx = x[:, None] + fx[:, None] * out[None, :, 0] + rx[:, None] * out[None, :, 1]
    wy = y[:, None] + fy[:, None] * out[None, :, 0] + ry[:, None] * out[None, :, 1]
    return wx, wy


def resample(x, y, step=10.0, closed=True):
    """Polilínea a paso uniforme (interpolación lineal)."""
    p = np.stack([x, y], axis=1)
    if closed:
        p = np.vstack([p, p[:1]])
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    m = max(int(s[-1] // step), 2)
    si = np.linspace(0.0, s[-1], m, endpoint=not closed)
    return np.interp(si, s, p[:, 0]), np.interp(si, s, p[:, 1]), si


def pose_heading(x, y, closed=True):
    p = np.stack([x, y], axis=1)
    t = (np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)) if closed else np.gradient(p, axis=0)
    return np.degrees(np.arctan2(t[:, 0], t[:, 1])) % 360.0


def clearances(x, y, pillars, closed=True) -> dict:
    """Holgura mínima (mm) de la huella del carro a pilares, pared exterior e isla.

    x, y: camino del eje trasero (se remuestrea a 10 mm). Negativo = choque."""
    xr, yr, _ = resample(np.asarray(x, float), np.asarray(y, float), 10.0, closed)
    hd = pose_heading(xr, yr, closed)
    wx, wy = _outline_world(xr, yr, hd)
    wall = float(np.min(OUTER_HALF - np.maximum(np.abs(wx), np.abs(wy))))
    ax = np.maximum(np.abs(wx) - ISLAND_HALF, 0.0)
    ay = np.maximum(np.abs(wy) - ISLAND_HALF, 0.0)
    inside = (np.abs(wx) < ISLAND_HALF) & (np.abs(wy) < ISLAND_HALF)
    isl = np.hypot(ax, ay)
    isl = np.where(inside, -np.minimum(ISLAND_HALF - np.abs(wx), ISLAND_HALF - np.abs(wy)), isl)
    out = {"wall": wall, "island": float(isl.min()), "pillar": math.inf}
    for (px, py, _c) in pillars:
        dx = np.maximum(np.abs(wx - px) - PILLAR_HALF, 0.0)
        dy = np.maximum(np.abs(wy - py) - PILLAR_HALF, 0.0)
        out["pillar"] = min(out["pillar"], float(np.hypot(dx, dy).min()))
    return out


def min_clearance(route: Route, pillars) -> float:
    c = clearances(route.x, route.y, pillars)
    return min(c["pillar"], c["wall"], c["island"])


def side_ok(x, y, pillars, closed=True) -> bool:
    """El carro pasa cada pilar por el lado que manda su color."""
    xr, yr, _ = resample(np.asarray(x, float), np.asarray(y, float), 5.0, closed)
    hd = np.radians(pose_heading(xr, yr, closed))
    tx, ty = np.sin(hd), np.cos(hd)
    for (px, py, col) in pillars:
        d = np.hypot(xr - px, yr - py)
        i = int(np.argmin(d))
        left = tx[i] * (py - yr[i]) - ty[i] * (px - xr[i])   # >0: pilar a la izquierda
        if (col == "Red") != (left > 0):
            return False
    return True


def path_stats(x, y, closed=True) -> dict:
    """Longitud, curvatura máx/RMS y cambios de signo, igual para cualquier polilínea.

    La curvatura sale del cambio de rumbo en +-40 mm sobre el camino remuestreado
    a 10 mm; los cambios de signo ignoran |k| < 1/2000 mm^-1 (ruido)."""
    xr, yr, si = resample(np.asarray(x, float), np.asarray(y, float), 10.0, closed)
    p = np.stack([xr, yr], axis=1)
    L = float(np.sum(np.linalg.norm(np.diff(np.vstack([p, p[:1]]) if closed else p, axis=0), axis=1)))
    k = 4
    a = np.roll(p, k, axis=0)
    b = np.roll(p, -k, axis=0)
    t1 = p - a
    t2 = b - p
    ang = np.arctan2(t1[:, 0] * t2[:, 1] - t1[:, 1] * t2[:, 0], np.sum(t1 * t2, axis=1))
    kap = ang / (10.0 * k)
    if not closed:
        kap[:k] = 0.0
        kap[-k:] = 0.0
    sg = np.sign(kap[np.abs(kap) > 1.0 / 2000.0])
    flips = int(np.sum(sg[1:] != sg[:-1])) + (int(sg[0] != sg[-1]) if closed and len(sg) else 0)
    return {"length": L, "kmax": float(np.abs(kap).max()),
            "krms": float(np.sqrt(np.mean(kap ** 2))), "flips": flips}


# ── Modelo de tiempo (velocidad por curvatura) ────────────────────────────
WHEELBASE_MM = 113.0
DELTA_MAX_RAD = math.radians(44.54)
SCRUB_COEFF = 0.35                  # twin/params.py (supuesto)
V0_LINEAR_MM_S = (309.0, 332.0)     # PWM 120: regla de 3 / modelo 3.5(PWM-25)
# T = int ds/v(k) ~ (1/v0) [L + SCRUB (WB/dmax)^2 int k^2 ds]  =>  w_curv de minimo tiempo
W_CURV_MIN_TIME = SCRUB_COEFF * (WHEELBASE_MM / DELTA_MAX_RAD) ** 2   # ~7.4e3 mm^2


def lap_time_s(x, y, v0_mm_s: float, closed: bool = True) -> float:
    """T = integral ds / v(k), v = v0 (1 - 0.35 (atan(WB k)/dmax)^2).

    k de path_stats (cambio de rumbo en +-40 mm sobre 10 mm), misma para toda ruta."""
    xr, yr, _ = resample(np.asarray(x, float), np.asarray(y, float), 10.0, closed)
    p = np.stack([xr, yr], axis=1)
    k = 4
    t1 = p - np.roll(p, k, axis=0)
    t2 = np.roll(p, -k, axis=0) - p
    ang = np.arctan2(t1[:, 0] * t2[:, 1] - t1[:, 1] * t2[:, 0], np.sum(t1 * t2, axis=1))
    kap = ang / (10.0 * k)
    d = np.arctan(WHEELBASE_MM * kap)
    v = v0_mm_s * np.maximum(1.0 - SCRUB_COEFF * (d / DELTA_MAX_RAD) ** 2, 0.2)
    ds = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)
    return float(np.sum(ds / v))
