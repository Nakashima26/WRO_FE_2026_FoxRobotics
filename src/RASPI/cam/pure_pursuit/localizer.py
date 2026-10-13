"""Localización en el tapete: odometría del ESP + distancias contra el mapa conocido.

EKF sobre la pose del eje trasero (x, y, rumbo), en mm y grados del tapete
(0 = +y, 90 = +x). Predicción: los incrementos de odometría del ESP (px, py,
yaw del ACK, encoder + IMU). Corrección: cada sensor de distancia del ACK
(ultrasónicos dL/dR/dF en cm, ToF tL/tR/tB/tF en mm) contra la distancia que
predice un ray-casting sobre el mapa COMPLETO (paredes, isla, señales y maderas
del cajón). Una medición solo se usa si el rayo esperado pega en pared o isla
(superficie plana conocida) con buen ángulo; si lo más cercano es una señal o
una madera, se descarta. Así una señal o el cajón nunca se confunden con la
pared (lo que hacía digital_map._wall_fix).

Salida para el ESP: la transformación odometría -> tapete (tx, ty, th) con
mundo = (tx, ty) + px·F + py·L, F = (sin th, cos th), L = (-cos th, sin th),
rumbo = th - yaw. Las correcciones solo mueven esa transformación.
"""

from __future__ import annotations

import math
import re

import numpy as np

from . import config as C
from .wro_field import INNER_HALF_MM, OUTER_HALF_MM, SEATS, SIGN_MM, _barrier_corners, section_to_world

# tipo de superficie
_WALL, _ISLAND, _SIGN, _BARRIER = 0, 1, 2, 3

# (nombre en el ACK, montaje en config.SENSOR_MOUNTS, escala a mm, cono, alcance máx)
_US_HALF_CONE = 15.0
_US_MAX_INC = 20.0
_SENSORS = (
    ("dL", "us_left", 10.0, "us"),
    ("dR", "us_right", 10.0, "us"),
    ("dF", "us_front", 10.0, "us"),
    ("tL", "tof_left", 1.0, "tof"),
    ("tR", "tof_right", 1.0, "tof"),
    ("tB", "tof_rear", 1.0, "tof"),
    ("tF", "tof_front", 1.0, "tof"),
)


def _ack_num(ack: str, key: str):
    m = re.search(r"(?:^|,)" + key + r"=(-?[\d.]+)", ack)
    return float(m.group(1)) if m else None


def _rect_segments(x0, y0, x1, y1, kind):
    return [((x0, y0), (x1, y0), kind), ((x1, y0), (x1, y1), kind),
            ((x1, y1), (x0, y1), kind), ((x0, y1), (x0, y0), kind)]


def build_segments(seats: dict, parking: str | None, park_w: float | None = None):
    segs = []
    o, i = OUTER_HALF_MM, INNER_HALF_MM
    segs += _rect_segments(-o, -o, o, o, _WALL)
    segs += _rect_segments(-i, -i, i, i, _ISLAND)
    h = SIGN_MM / 2.0
    for (sec, seat), _color in seats.items():
        if seat not in SEATS:
            continue
        sx, sy = section_to_world(sec, *SEATS[seat])
        segs += _rect_segments(sx - h, sy - h, sx + h, sy + h, _SIGN)
    if parking is not None:
        for box in (_barrier_corners(parking) if park_w is None else _barrier_corners(parking, park_w)):
            xs = [p[0] for p in box]
            ys = [p[1] for p in box]
            segs += _rect_segments(min(xs), min(ys), max(xs), max(ys), _BARRIER)
    a = np.array([s[0] for s in segs], float)
    b = np.array([s[1] for s in segs], float)
    k = np.array([s[2] for s in segs], int)
    return a, b, k


def raycast(origins: np.ndarray, dirs: np.ndarray, segs, max_mm: float = 4000.0):
    """Distancia, tipo y |cos| de incidencia del primer impacto de cada rayo."""
    a, b, kind = segs
    e = b - a                                            # (S,2)
    ox, oy = origins[:, 0:1], origins[:, 1:2]            # (R,1)
    dx, dy = dirs[:, 0:1], dirs[:, 1:2]
    den = dx * e[None, :, 1] - dy * e[None, :, 0]        # (R,S)
    wx = a[None, :, 0] - ox
    wy = a[None, :, 1] - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (wx * e[None, :, 1] - wy * e[None, :, 0]) / den
        u = (wx * dy - wy * dx) / den
    ok = (np.abs(den) > 1e-9) & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
    t = np.where(ok, t, np.inf)
    j = np.argmin(t, axis=1)
    d = t[np.arange(len(t)), j]
    en = e[j] / np.linalg.norm(e[j], axis=1, keepdims=True)
    cos_inc = np.abs(dirs[:, 0] * en[:, 1] - dirs[:, 1] * en[:, 0])     # |dir · normal|
    hit = np.isfinite(d) & (d <= max_mm)
    return np.where(hit, d, np.inf), np.where(hit, kind[j], -1), cos_inc


class Localizer:
    def __init__(self, seats: dict, parking: str | None, x: float, y: float, heading_deg: float,
                 park_w: float | None = None):
        self.park_w = park_w
        self.segs = build_segments(seats, parking, park_w)
        self.x = np.array([x, y, math.radians(heading_deg)], float)
        # El rumbo del BNO085 es bueno (deriva ~0.5°/min): poca incertidumbre de
        # rumbo, así una sola medición rara no lo gira varios grados.
        self.P = np.diag([30.0 ** 2, 30.0 ** 2, math.radians(0.5) ** 2])
        self.odom = None                 # (px, py, yaw) del último ACK usado
        self.last = {}
        self.n_used = 0
        self.n_rej = 0

    def set_map(self, seats: dict, unknown, parking: str | None, park_w: float | None = None) -> None:
        """Señales conocidas + asientos sin resolver (tapan el rayo igual)."""
        if park_w is not None:
            self.park_w = park_w
        allseats = dict(seats)
        for key in unknown or ():
            allseats.setdefault(key, "?")
        self.segs = build_segments(allseats, parking, self.park_w)

    def shift(self, dx: float, dy: float) -> None:
        """Cambio de marco (arranque ciego resuelto): la pose se corre (dx, dy)."""
        self.x[0] += dx
        self.x[1] += dy

    # Transformación odometría -> tapete para el ESP.
    def transform(self):
        if self.odom is None:
            return None
        px, py, yaw = self.odom
        th = math.degrees(self.x[2]) + yaw
        r = math.radians(th)
        fx, fy = math.sin(r), math.cos(r)
        lx, ly = -math.cos(r), math.sin(r)
        tx = self.x[0] - (px * fx + py * lx)
        ty = self.x[1] - (px * fy + py * ly)
        return tx, ty, th

    def pose(self):
        return float(self.x[0]), float(self.x[1]), math.degrees(self.x[2]) % 360.0

    def _predict(self, px, py, yaw):
        if self.odom is None:
            self.odom = (px, py, yaw)
            return
        p0x, p0y, y0 = self.odom
        dpx, dpy = px - p0x, py - p0y
        # incremento de odometría al tapete con la transformación actual
        th = self.x[2] + math.radians(y0)
        fx, fy = math.sin(th), math.cos(th)
        lx, ly = -math.cos(th), math.sin(th)
        self.x[0] += dpx * fx + dpy * lx
        self.x[1] += dpx * fy + dpy * ly
        self.x[2] -= math.radians(yaw - y0)
        self.odom = (px, py, yaw)
        ds = math.hypot(dpx, dpy)
        dyaw = abs(yaw - y0)
        q_xy = (0.02 * ds) ** 2 + 0.2
        q_th = (math.radians(0.6) * ds / 1000.0) ** 2 + (math.radians(0.01) * dyaw) ** 2 + math.radians(0.01) ** 2
        # Error de rumbo -> error de posición al avanzar: w = R(θ+yaw0)·Δp,
        # ∂wx/∂θ = wy, ∂wy/∂θ = -wx.
        wx = dpx * fx + dpy * lx
        wy = dpx * fy + dpy * ly
        F = np.eye(3)
        F[0, 2] = wy
        F[1, 2] = -wx
        self.P = F @ self.P @ F.T + np.diag([q_xy, q_xy, q_th])

    def _expect(self, state, mount, kind_sensor, segs=None):
        """Distancia esperada (mm), tipo de superficie y |cos| de incidencia."""
        x, y, h = state
        right, fwd, ddeg = mount
        ox = x + fwd * math.sin(h) + right * math.cos(h)
        oy = y + fwd * math.cos(h) - right * math.sin(h)
        if kind_sensor == "us":
            offs = np.arange(-_US_HALF_CONE, _US_HALF_CONE + 0.1, 3.0)
        else:
            offs = np.array([0.0])
        ang = h + np.radians(ddeg + offs)
        dirs = np.stack([np.sin(ang), np.cos(ang)], 1)
        orig = np.broadcast_to(np.array([ox, oy]), dirs.shape)
        d, k, c = raycast(orig, dirs, self.segs if segs is None else segs)
        if kind_sensor == "us":
            inc_ok = (k == _SIGN) | (np.degrees(np.arccos(np.clip(c, 0, 1))) <= _US_MAX_INC)
            d = np.where(inc_ok, d, np.inf)
        i = int(np.argmin(d))
        return float(d[i]), int(k[i]), float(c[i])

    def _shifted_segs(self, dx: float, dy: float):
        """Mapa visto desde un carro corrido (dx, dy): las maderas se corren con él."""
        a, b, k = self.segs
        m = (k == _BARRIER)[:, None]
        sv = np.array([dx, dy])
        return a + m * sv, b + m * sv, k

    def update(self, ack: str | None, correct: bool = True, shifts=None) -> None:
        """Predicción con la odometría del ACK; con correct, corrección con sensores.

        shifts: corrimientos (dx, dy) posibles de la pose (arranque ciego sin
        resolver): solo se usa una lectura si su valor esperado no cambia con
        ellos (pared lateral, madera del cajón), nunca la pared del fondo."""
        if not ack:
            return
        px, py, yaw = _ack_num(ack, "px"), _ack_num(ack, "py"), _ack_num(ack, "yaw")
        if px is None or py is None or yaw is None:
            return
        if self.odom is not None and (px, py, yaw) == self.odom:
            return                      # ACK repetido: nada nuevo
        self._predict(px, py, yaw)
        if not correct:
            return
        for key, mname, scale, kind in _SENSORS:
            raw = _ack_num(ack, key)
            if raw is None or raw <= 0:
                continue
            z = raw * scale
            if kind == "us" and (z < 30.0 or z > 1500.0):
                continue
            if kind == "tof" and (z < 20.0 or z > 240.0):
                continue
            mount = C.SENSOR_MOUNTS[mname]
            zh, surf, cosi = self._expect(self.x, mount, kind)
            if surf not in (_WALL, _ISLAND) or not math.isfinite(zh):
                continue
            if shifts:
                stable = True
                for dx, dy in shifts:
                    st = self.x.copy()
                    st[0] += dx
                    st[1] += dy
                    z2, surf2, _c2 = self._expect(st, mount, kind, self._shifted_segs(dx, dy))
                    if surf2 != surf or not math.isfinite(z2) or abs(z2 - zh) > 8.0:
                        stable = False
                        break
                if not stable:
                    continue
            if kind == "tof" and cosi < math.cos(math.radians(35.0)):
                continue
            # Jacobiano numérico
            H = np.zeros(3)
            ok = True
            for j, eps in ((0, 2.0), (1, 2.0), (2, math.radians(0.5))):
                s2 = self.x.copy()
                s2[j] += eps
                z2, surf2, _c = self._expect(s2, mount, kind)
                if surf2 != surf or not math.isfinite(z2):
                    ok = False
                    break
                H[j] = (z2 - zh) / eps
            if not ok:
                continue
            sig = (8.0 + 0.01 * z) if kind == "us" else (6.0 + 0.01 * z)
            R = sig * sig
            nu = z - zh
            S = float(H @ self.P @ H + R)
            if nu * nu / S > 9.0:                  # 3 sigma
                self.n_rej += 1
                continue
            K = self.P @ H / S
            self.x = self.x + K * nu
            self.P = (np.eye(3) - np.outer(K, H)) @ self.P
            self.n_used += 1
            self.last[key] = (round(z), round(zh))
