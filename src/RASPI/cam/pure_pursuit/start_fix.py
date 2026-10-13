"""Arranque ciego: a qué altura de la recta quedó el cajón.

El carro arranca dentro del cajón, pegado a la pared exterior de la recta de
salida, pero los jueces pueden correr el cajón a lo largo de esa recta (pasó en
el nacional 2026). Se conocen la recta, el rumbo (paralelo a la pared) y la
distancia a la pared exterior; NO se conoce el avance.

La Pi arranca suponiendo el cajón lo más ATRÁS posible en el sentido de
carrera (CW: w0 = PARK_W_MIN, CCW: w0 = PARK_W_MAX). Así, mientras no sepa
dónde está, el carro cree ir más atrás de lo que va: la ruta dobla tarde,
nunca antes de la esquina (contra la isla).

El corrimiento δ (mm a lo largo de +w de la recta: verdad = supuesto + δ·w)
se estima con un histograma sobre todos los δ posibles. Cada lectura nueva de
los sonares (frontal y laterales) vota por los δ en los que pared, isla,
maderas del cajón o una señal posible explicarían esa distancia. El sonar
frontal alcanza la pared del fondo (≤119 cm, timeout de pulseIn del firmware)
en la segunda mitad de la recta: ahí el histograma se colapsa en un pico.

Las maderas del cajón se mueven CON el carro (están donde arrancó), así que se
proyectan sin corrimiento; paredes, isla y asientos son del tapete y se
proyectan corridos δ. Una señal sin resolver puede estar o no: si en una
hipótesis alguna señal posible cae en un cono ANCHO (±21°) antes que la pared,
cualquier lectura entre esa señal y la pared vale _P_SIGN en esa hipótesis (qué
señal toca el borde del cono no se sabe modelar fino: semillas 1 y 8, la N/T4 a
14.7° del eje fijó −191 mm). Así una lectura de señal nunca decide entre
hipótesis (los asientos están cada 500 mm: la T1 vista en un δ es la X1 en
otro); solo pared, isla y maderas deciden.

Solo con el ESP en S/C/R y sin giro rápido: en la maniobra de salida (INICIO)
las lecturas en transición (EMA, mediana, sonar vs pose) fijaban mal el δ.
"""

from __future__ import annotations

import math

import numpy as np

from . import config as C
from .localizer import _BARRIER, _ISLAND, _SIGN, _WALL, _ack_num, _rect_segments, raycast
from .wro_field import (
    INNER_HALF_MM,
    OUTER_HALF_MM,
    PARK_W_MAX,
    PARK_W_MIN,
    PARKING_FORBIDDEN_SEATS,
    SEATS,
    SECTIONS_CW,
    SIGN_MM,
    _barrier_corners,
    section_to_world,
    w_axis,
)

_US_MAX_MM = 1190.0          # leerDistancia: timeout 7000 us ≈ 119 cm, sin eco = 200
_US_HALF_CONE = 15.0
_SIGN_HALF_CONE = 21.0       # cono para "una señal posible podría contestar"
_WORLD_RAY_STEP = 1.5        # paredes e isla (planas y largas)
_CAR_RAY_STEP = 0.5          # maderas del cajón: 20 mm de grueso (a 3° se escapaban, semilla 8)
_US_MAX_INC = 20.0
_P_MISS = 0.15               # el sonar no contesta aunque haya pared
_EPS = 0.03                  # lectura que el modelo no explica (fantasma)
_P_SIGN = 0.4                # una señal posible está de verdad en el haz
_MAX_YAW_STEP = 2.5          # grados entre lecturas: más = girando, no se usa
_SENSORS = (("dF", "us_front"), ("dL", "us_left"), ("dR", "us_right"))


def guess_w0(direction: str) -> float:
    """Cajón supuesto: el más atrás en el sentido de carrera."""
    return PARK_W_MIN if direction == "CW" else PARK_W_MAX


def _segs(rects):
    if not rects:
        return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0, int)
    a = np.array([s[0] for s in rects], float)
    b = np.array([s[1] for s in rects], float)
    k = np.array([s[2] for s in rects], int)
    return a, b, k


def _box_hits(o, d):
    """Primer impacto contra la pared exterior (desde adentro) y la isla (desde
    afuera), cuadrados alineados al tapete: (distancia, tipo, |cos| de incidencia).
    Igual que raycast() sobre esos 8 segmentos pero analítico (~10x más rápido)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        # pared exterior: salida del cuadrado ±OUTER
        tx = np.where(d[:, 0] > 0, OUTER_HALF_MM - o[:, 0], -OUTER_HALF_MM - o[:, 0]) * inv[:, 0]
        ty = np.where(d[:, 1] > 0, OUTER_HALF_MM - o[:, 1], -OUTER_HALF_MM - o[:, 1]) * inv[:, 1]
        tx = np.where(np.isfinite(tx), tx, np.inf)
        ty = np.where(np.isfinite(ty), ty, np.inf)
        t_out = np.minimum(tx, ty)
        c_out = np.where(tx <= ty, np.abs(d[:, 0]), np.abs(d[:, 1]))
        # isla: entrada al cuadrado ±INNER
        a1 = (-INNER_HALF_MM - o) * inv
        a2 = (INNER_HALF_MM - o) * inv
        lo = np.where(np.isnan(a1), -np.inf, np.minimum(a1, a2))
        hi = np.where(np.isnan(a1), np.inf, np.maximum(a1, a2))
    t_near = np.maximum(lo[:, 0], lo[:, 1])
    t_far = np.minimum(hi[:, 0], hi[:, 1])
    hit = (t_near <= t_far) & (t_near > 1e-6)
    t_isl = np.where(hit, t_near, np.inf)
    c_isl = np.where(lo[:, 0] >= lo[:, 1], np.abs(d[:, 0]), np.abs(d[:, 1]))
    isl = t_isl < t_out
    return (np.where(isl, t_isl, t_out), np.where(isl, _ISLAND, _WALL), np.where(isl, c_isl, c_out))


def _first_hit(origins, dirs, groups):
    """Primer impacto entre varios grupos de segmentos: (d, tipo, |cos|)."""
    best_d = np.full(len(origins), np.inf)
    best_k = np.full(len(origins), -1)
    best_c = np.zeros(len(origins))
    for segs in groups:
        if len(segs[0]) == 0:
            continue
        d, k, c = raycast(origins, dirs, segs)
        sel = d < best_d
        best_d = np.where(sel, d, best_d)
        best_k = np.where(sel, k, best_k)
        best_c = np.where(sel, c, best_c)
    return best_d, best_k, best_c


class StartFix:
    """Histograma del corrimiento δ del cajón (ver docstring del módulo)."""

    def __init__(self, parking: str, direction: str, w0_guess: float | None = None,
                 step_mm: float = 5.0, margin_mm: float = 40.0) -> None:
        self.parking = parking
        self.direction = direction
        self.w0_guess = guess_w0(direction) if w0_guess is None else float(w0_guess)
        # El carro está fijo respecto al cajón: δ = w0 real - w0 supuesto.
        lo, hi = PARK_W_MIN - self.w0_guess, PARK_W_MAX - self.w0_guess
        self.deltas = np.arange(lo - margin_mm, hi + margin_mm + 0.5 * step_mm, step_mm)
        self.logp = np.zeros(len(self.deltas))
        self.axis = np.array(w_axis(parking), float)
        self.n_inform = 0
        self.n_frames = 0
        self.last_odom = None
        self.fixed: float | None = None
        o, i = OUTER_HALF_MM, INNER_HALF_MM
        self._static = _segs(_rect_segments(-o, -o, o, o, _WALL) + _rect_segments(-i, -i, i, i, _ISLAND))
        self._car = _segs(self._barrier_rects(self.w0_guess))
        self._seat_pts = np.zeros((0, 2))
        self.set_possible_signs(None)

    # ── mapa ──────────────────────────────────────────────────────────────
    def _barrier_rects(self, w0):
        out = []
        for box in _barrier_corners(self.parking, w0):
            xs = [p[0] for p in box]
            ys = [p[1] for p in box]
            out += _rect_segments(min(xs), min(ys), max(xs), max(ys), _BARRIER)
        return out

    def set_possible_signs(self, seats) -> None:
        """Asientos (del tapete) donde puede haber señal; None = todos los legales.
        En la recta del cajón solo la fila interior."""
        if seats is None:
            seats = [(sec, s) for sec in SECTIONS_CW for s in SEATS
                     if not (sec == self.parking and s in PARKING_FORBIDDEN_SEATS)]
        self._seat_pts = np.array([section_to_world(sec, *SEATS[s]) for sec, s in seats], float).reshape(-1, 2)

    def set_car_signs(self, pts) -> None:
        """Señales vistas por la cámara en el marco supuesto (se mueven con el carro)."""
        h = SIGN_MM / 2.0
        rects = []
        for x, y in pts:
            rects += _rect_segments(x - h, y - h, x + h, y + h, _SIGN)
        self._car = _segs(self._barrier_rects(self.w0_guess) + rects)

    # ── modelo de cada sonar ──────────────────────────────────────────────
    @staticmethod
    def _sensor(pose, mount):
        x, y, hd = pose
        h = math.radians(hd)
        right, fwd, ddeg = mount
        ox = x + fwd * math.sin(h) + right * math.cos(h)
        oy = y + fwd * math.cos(h) - right * math.sin(h)
        return np.array([ox, oy]), hd + ddeg

    @staticmethod
    def _valid_min(d, k, c, n_cells, n_rays):
        inc = np.degrees(np.arccos(np.clip(c, 0.0, 1.0)))
        ok = np.isfinite(d) & ((k == _SIGN) | (inc <= _US_MAX_INC))
        return np.where(ok, d, np.inf).reshape(n_cells, n_rays).min(axis=1)

    def _expected_us(self, pose, mount) -> np.ndarray:
        """Lectura esperada (mm, inf = sin eco) del sonar para cada δ, sin
        señales posibles: paredes e isla (corridas δ) y lo que va con el carro
        (maderas del cajón y la señal que vio la cámara)."""
        o, axis_deg = self._sensor(pose, mount)
        J = len(self.deltas)
        ang = np.radians(axis_deg + np.arange(-_US_HALF_CONE, _US_HALF_CONE + 0.01, _WORLD_RAY_STEP))
        dirs = np.stack([np.sin(ang), np.cos(ang)], 1)
        R = len(dirs)
        orig = np.repeat(o[None, :] + self.deltas[:, None] * self.axis[None, :], R, axis=0)
        d, k, c = _box_hits(orig, np.tile(dirs, (J, 1)))
        world = self._valid_min(d, k, c, J, R)
        car = np.inf
        if len(self._car[0]):
            ang = np.radians(axis_deg + np.arange(-_US_HALF_CONE, _US_HALF_CONE + 0.01, _CAR_RAY_STEP))
            dirs = np.stack([np.sin(ang), np.cos(ang)], 1)
            d, k, c = raycast(np.broadcast_to(o, dirs.shape), dirs, self._car)
            car = float(self._valid_min(d, k, c, 1, len(dirs))[0])
        e = np.minimum(world, car)
        return np.where(e <= _US_MAX_MM, e, np.inf)

    def _sign_dist(self, pose, mount, half_cone: float = _SIGN_HALF_CONE) -> np.ndarray:
        """Para cada δ, distancia a la señal posible más cercana que cabe en un
        cono ancho (analítico: ancho angular de la señal, sin muestrear rayos)."""
        if not len(self._seat_pts):
            return np.full(len(self.deltas), np.inf)
        o, axis_deg = self._sensor(pose, mount)
        rel = (self._seat_pts[None, :, :] - self.deltas[:, None, None] * self.axis[None, None, :]
               - o[None, None, :])                                            # (J,S,2)
        r = np.hypot(rel[..., 0], rel[..., 1])
        bearing = np.degrees(np.arctan2(rel[..., 0], rel[..., 1]))
        off = np.abs((bearing - axis_deg + 180.0) % 360.0 - 180.0)
        half_w = np.degrees(np.arcsin(np.clip((SIGN_MM / math.sqrt(2.0)) / np.maximum(r, 1.0), 0.0, 1.0)))
        ok = (off <= half_cone + half_w) & (r - SIGN_MM / 2.0 <= _US_MAX_MM)
        return np.where(ok, r - SIGN_MM / 2.0, np.inf).min(axis=1)

    @staticmethod
    def _lik(z, e) -> np.ndarray:
        if z is None:
            return np.where(np.isfinite(e), np.where(e > 1100.0, 0.6, _P_MISS), 1.0)
        sig = 15.0 + 0.015 * z
        g = np.exp(-0.5 * ((z - np.where(np.isfinite(e), e, 1e9)) / sig) ** 2)
        return (1.0 - _EPS) * g + _EPS

    # ── paso por frame ────────────────────────────────────────────────────
    def update(self, ack: str | None, pose) -> None:
        """pose: eje trasero (x, y, rumbo) en el marco SUPUESTO, del mismo ACK."""
        if self.fixed is not None or not ack:
            return
        od = (_ack_num(ack, "px"), _ack_num(ack, "py"), _ack_num(ack, "yaw"))
        if None in od or od == self.last_odom:
            return
        prev, self.last_odom = self.last_odom, od
        est = ack.split("est=", 1)[1][:1] if "est=" in ack else None
        if est not in ("S", "C", "R"):
            return
        if prev is None or abs((od[2] - prev[2] + 180.0) % 360.0 - 180.0) > _MAX_YAW_STEP:
            return
        self.n_frames += 1
        for key, mname in _SENSORS:
            raw = _ack_num(ack, key)
            if raw is None:
                continue
            z = raw * 10.0
            z = None if (raw <= 0 or z >= _US_MAX_MM) else z
            mount = C.SENSOR_MOUNTS[mname]
            e0 = self._expected_us(pose, mount)
            lik = self._lik(z, e0)
            if z is not None:
                es = self._sign_dist(pose, mount)
                maybe = (np.isfinite(es) & (z >= es - 60.0)
                         & (~np.isfinite(e0) | (z < e0 - 30.0)))
                lik = np.where(maybe, np.maximum(lik, _P_SIGN), lik)
            if lik.max() > 3.0 * lik.min():
                self.n_inform += 1
            self.logp += np.log(lik)
        self.logp -= self.logp.max()

    def posterior(self) -> np.ndarray:
        p = np.exp(self.logp - self.logp.max())
        return p / p.sum()

    def estimate(self, win_mm: float = 20.0):
        """(δ, masa en ±win alrededor del máximo). El máximo se busca en el
        histograma suavizado: en una meseta plana argmax daba el primer índice
        (semilla 16: -127 con la verdad en -36)."""
        p = self.posterior()
        k = np.exp(-0.5 * (np.arange(-6, 7) * (self.deltas[1] - self.deltas[0]) / 15.0) ** 2)
        j = int(np.argmax(np.convolve(p, k / k.sum(), mode="same")))
        sel = np.abs(self.deltas - self.deltas[j]) <= win_mm
        m = float(p[sel].sum())
        d = float((p[sel] * self.deltas[sel]).sum() / max(m, 1e-12))
        return d, m

    def mean(self) -> float:
        """Media del histograma (respaldo cuando no hay pico)."""
        p = self.posterior()
        return float((p * self.deltas).sum())

    def converged(self) -> float | None:
        """δ si el histograma ya tiene un solo pico angosto."""
        if self.fixed is not None:
            return self.fixed
        if self.n_inform < int(getattr(C, "ROUTE_FIX_MIN_INFORM", 4)):
            return None
        d, m = self.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))
        if m >= float(getattr(C, "ROUTE_FIX_MASS", 0.97)):
            return d
        return None
