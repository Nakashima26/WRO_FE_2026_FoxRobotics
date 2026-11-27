"""Localización en el tapete: odometría del ESP + distancias contra el mapa conocido.

EKF sobre la pose del eje trasero (x, y, rumbo), en mm y grados del tapete
(0 = +y, 90 = +x). Predicción: los incrementos de odometría del ESP (px, py,
yaw del ACK, encoder + IMU). Corrección: cada sensor de distancia del ACK
(ultrasónicos dL/dR/dF en cm, ToF tL/tR/tB/tF en mm) contra la distancia que
predice un ray-casting sobre el mapa COMPLETO (paredes, isla, señales y maderas
del cajón). Una medición solo se usa si el rayo esperado pega en pared o isla
(superficie plana conocida) con buen ángulo; si lo más cercano es una señal o
una madera, se descarta. En carrera, la guarda opt-in comprueba también el
cono del ToF para descartar ecos de obstáculos conocidos fuera del eje.
El visor de maniobras también
usa caras de las maderas con el cajón fijado.

Salida para el ESP: la transformación odometría -> tapete (tx, ty, th) con
mundo = (tx, ty) + px·F + py·L, F = (sin th, cos th), L = (-cos th, sin th),
rumbo = th - yaw. Las correcciones solo mueven esa transformación.
"""

from __future__ import annotations

import functools
import math
import os
import re

import numpy as np

from . import config as C
from .wro_field import INNER_HALF_MM, OUTER_HALF_MM, SEATS, SIGN_MM, _barrier_corners, section_to_world, w_axis

# tipo de superficie
_WALL, _ISLAND, _SIGN, _BARRIER = 0, 1, 2, 3

# (nombre en el ACK, montaje en config.SENSOR_MOUNTS, escala a mm, cono, alcance máx)
_US_HALF_CONE = 15.0
_US_MAX_INC = 20.0
_US_OFFS = np.arange(-_US_HALF_CONE, _US_HALF_CONE + 0.1, 3.0)   # rayos del cono (_expect), constantes
_TOF_OFFS = np.array([0.0])
_TOF_CONE_OFFS = np.radians(np.linspace(-13.5, 13.5, 19))
_SENSORS = (
    ("dL", "us_left", 10.0, "us"),
    ("dR", "us_right", 10.0, "us"),
    ("dF", "us_front", 10.0, "us"),
    ("tL", "tof_left", 1.0, "tof"),
    ("tR", "tof_right", 1.0, "tof"),
    ("tB", "tof_rear", 1.0, "tof"),
    ("tF", "tof_front", 1.0, "tof"),
)


@functools.cache
def _ack_re(key: str):
    return re.compile(r"(?:^|,)" + key + r"=(-?[\d.]+)")


def _ack_num(ack: str, key: str):
    m = _ack_re(key).search(ack)
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


def raycast(origins: np.ndarray, dirs: np.ndarray, segs, max_mm: float = 4000.0, *, face=False):
    """Distancia, tipo y |cos| del primer impacto; face devuelve índice y fracción del segmento."""
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
    if face:
        return np.where(hit, d, np.inf), j, u[np.arange(len(t)), j], cos_inc
    return np.where(hit, d, np.inf), np.where(hit, kind[j], -1), cos_inc


class Localizer:
    def __init__(self, seats: dict, parking: str | None, x: float, y: float, heading_deg: float,
                 park_w: float | None = None):
        self.park_w = park_w
        self.park_axis = w_axis(parking) if parking is not None else None
        self.segs = build_segments(seats, parking, park_w)
        self.occlusion_segs = self.segs
        self.x = np.array([x, y, math.radians(heading_deg)], float)
        # El rumbo del BNO085 es bueno (deriva ~0.5°/min): poca incertidumbre de
        # rumbo, así una sola medición rara no lo gira varios grados.
        self.P = np.diag([30.0 ** 2, 30.0 ** 2, math.radians(0.5) ** 2])
        self.odom = None                 # (px, py, yaw) del último ACK usado
        self.last = {}
        self.n_used = 0
        self.n_rej = 0
        self.n_reanchor = 0
        self._rej = {}
        self._prev_z = {}
        self._prev_ack_yaw = None
        self.anchor_segs = None          # señales reales (DigitalMap.confirmed) como referencia del ToF lateral
        self._anchor_key = None
        self.n_sign = 0

    def set_map(self, seats: dict, unknown, parking: str | None, park_w: float | None = None) -> None:
        """Señales conocidas + asientos sin resolver (tapan el rayo igual)."""
        if park_w is not None:
            self.park_w = park_w
        allseats = dict(seats)
        for key in unknown or ():
            allseats.setdefault(key, "?")
        self.park_axis = w_axis(parking) if parking is not None else None
        self.segs = build_segments(allseats, parking, self.park_w)
        if C.LOC_RACE_TOF_OCCLUSION_MM is not None:
            self.occlusion_segs = build_segments(seats, parking, self.park_w)

    def set_anchors(self, seats: dict) -> None:
        """Señales confirmadas por el mapa (sin fantasmas de fila) que el ToF lateral puede usar de referencia."""
        key = tuple(sorted(seats))
        if key == self._anchor_key:
            return
        self._anchor_key = key
        if not seats:
            self.anchor_segs = None
            return
        a, b, k = build_segments(seats, None)
        m = k == _SIGN
        self.anchor_segs = (a[m], b[m], k[m])

    def _sign_cone(self, state, mount) -> float:
        """Distancia a la cara de señal más cercana dentro del cono del ToF (el VL53 devuelve lo más cercano)."""
        x, y, h = state
        right, fwd, angle = mount
        origin = (x + fwd * math.sin(h) + right * math.cos(h), y + fwd * math.cos(h) - right * math.sin(h))
        half = float(getattr(C, "LOC_TOF_SIGN_HALF_DEG", 7.0))   # ROI 8 del firmware: cono ~14° total (twin/params.py)
        ang = h + math.radians(angle) + np.radians(np.linspace(-half, half, 11))
        dirs = np.column_stack((np.sin(ang), np.cos(ang)))
        d, _k, _c = raycast(np.broadcast_to(origin, dirs.shape), dirs, self.anchor_segs)
        return float(np.min(d))

    def _sign_corner_active(self, ack: str) -> bool:
        """Esquina temprana o giro rápido: un ToF lateral contra bloque observa mal el rumbo."""
        tc_lim = getattr(C, "LOC_SIGN_CORNER_TC_MAX", None)
        tc = _ack_num(ack, "tc")
        corner = tc_lim is not None and tc is not None and int(tc) <= int(tc_lim)
        ystep = getattr(C, "LOC_SIGN_TURN_YAW_DEG", None)
        yaw = _ack_num(ack, "yaw")
        fast = False
        if ystep is not None and yaw is not None and self._prev_ack_yaw is not None:
            fast = abs(yaw - self._prev_ack_yaw) > float(ystep)
        return corner or fast

    def _sign_update(self, key, mount, z, zs, ack) -> None:
        """Corrección EKF con el eco de una señal conocida (orillas162: 369 ecos cortos con señal en el cono,
        residuo mediano 11 mm; donde la pose decía 267 mm al bloque el ToF lo tenía a 30)."""
        nu = z - zs
        if self._sign_corner_active(ack):
            cap = getattr(C, "LOC_SIGN_CORNER_MAX_INNOV_MM", None)
            if cap is not None and abs(nu) > float(cap):
                self.n_rej += 1
                return
        H = np.zeros(3)
        for j, eps in ((0, 2.0), (1, 2.0), (2, math.radians(0.5))):
            s2 = self.x.copy()
            s2[j] += eps
            z2 = self._sign_cone(s2, mount)
            if not math.isfinite(z2):
                return
            H[j] = (z2 - zs) / eps
        sig = float(getattr(C, "LOC_TOF_SIGN_SIGMA_MM", 15.0)) + 0.02 * z
        S = float(H @ self.P @ H + sig * sig)
        if nu * nu / S > 9.0:
            self.n_rej += 1
            return
        K = self.P @ H / S
        if getattr(C, "LOC_SIGN_CORNER_NO_YAW", None) and self._sign_corner_active(ack):
            keys = getattr(C, "LOC_SIGN_CORNER_KEYS", None)
            key_ok = keys is None or key in keys
            nu_min = getattr(C, "LOC_SIGN_CORNER_YAW_NU_MIN", None)
            nu_ok = nu_min is None or abs(nu) > float(nu_min)
            if key_ok and nu_ok:
                K[2] = 0.0
        dx = K * nu
        self.x = self.x + dx
        self.P = (np.eye(3) - np.outer(K, H)) @ self.P
        self.n_used += 1
        self.n_sign += 1
        self.last[key] = (round(z), round(zs))
        log_mm = getattr(C, "LOC_LOG_UPD_MM", None)
        if log_mm is not None and math.hypot(dx[0], dx[1]) >= float(log_mm):
            print(f"[LOCUPD] {key} z={z:.0f} esp={zs:.0f} nu={nu:+.0f} sup=bloque "
                  f"dx={dx[0]:+.0f} dy={dx[1]:+.0f} dh={math.degrees(dx[2]):+.2f} "
                  f"pose=({self.x[0]:.0f},{self.x[1]:.0f},{math.degrees(self.x[2]):.0f}) tc={_ack_num(ack, 'tc')}",
                  flush=True)

    def along_update(self, axis, r: float, sigma_mm: float) -> np.ndarray:
        """Medida directa del avance: la pose va `r` mm atrás (r > 0) a lo largo de `axis` (vector unitario).
        Corrección EKF escalar con H = (ax, ay, 0): reparte con la P actual y la achica (una cinta vista)."""
        H = np.array([float(axis[0]), float(axis[1]), 0.0])
        S = float(H @ self.P @ H + sigma_mm * sigma_mm)
        K = self.P @ H / S
        dx = K * float(r)
        self.x = self.x + dx
        self.P = (np.eye(3) - np.outer(K, H)) @ self.P
        self.n_used += 1
        return dx

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

    def pose(self, ack: str | None = None):
        """Pose actual, o proyección odométrica al ACK nuevo sin modificar el filtro."""
        if ack and self.odom is not None:
            px, py, yaw = (_ack_num(ack, key) for key in ("px", "py", "yaw"))
            if px is not None and py is not None and yaw is not None:
                tx, ty, th = self.transform()
                r = math.radians(th)
                sin_th, cos_th = math.sin(r), math.cos(r)
                return (tx + px * sin_th - py * cos_th, ty + px * cos_th + py * sin_th,
                        (th - yaw) % 360.0)
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
        # 2026-11-09: el (2 %·ds)² por paso crece como raíz del recorrido (σ ~6 mm en 2 m), y el
        # error real del carro es sistemático (~19 cm en la recta norte de la 13:49). Con P tan chica
        # el gate de 3σ tiraba TODAS las lecturas laterales buenas y la pose nunca se corregía.
        # LOC_Q_XY_PER_MM: varianza (mm²) que se suma por mm recorrido (2 -> σ ~65 mm en 2 m sin paredes).
        q_lin = getattr(C, "LOC_Q_XY_PER_MM", None)
        if q_lin is not None:
            q_xy += float(q_lin) * ds
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
        offs = _US_OFFS if kind_sensor == "us" else _TOF_OFFS
        ang = h + np.radians(ddeg + offs)
        dirs = np.stack([np.sin(ang), np.cos(ang)], 1)
        orig = np.broadcast_to(np.array([ox, oy]), dirs.shape)
        d, k, c = raycast(orig, dirs, self.segs if segs is None else segs)
        if kind_sensor == "us":
            inc_ok = (k == _SIGN) | (np.degrees(np.arccos(np.clip(c, 0, 1))) <= _US_MAX_INC)
            d = np.where(inc_ok, d, np.inf)
        i = int(np.argmin(d))
        return float(d[i]), int(k[i]), float(c[i])

    def _occlusion(self, mount, expected, reading):
        """Error del eco respecto a la primera cara de obstáculo visible en cada rayo."""
        x, y, h = self.x
        right, fwd, angle = mount
        origin = (x + fwd * math.sin(h) + right * math.cos(h),
                  y + fwd * math.cos(h) - right * math.sin(h))
        angles = h + math.radians(angle) + _TOF_CONE_OFFS
        dirs = np.column_stack((np.sin(angles), np.cos(angles)))
        distances, surfaces, _ = raycast(
            np.broadcast_to(origin, dirs.shape), dirs, self.occlusion_segs, expected)
        obstacle = (surfaces == _SIGN) | (surfaces == _BARRIER)
        if not np.any(obstacle):
            return math.inf
        return float(np.min(np.abs(distances[obstacle] - reading)))

    def _shifted_segs(self, dx: float, dy: float):
        """Mapa visto desde un carro corrido (dx, dy): las maderas se corren con él."""
        a, b, k = self.segs
        m = (k == _BARRIER)[:, None]
        sv = np.array([dx, dy])
        return a + m * sv, b + m * sv, k

    def update_maneuver(self, ack: str) -> None:
        """ToF de maniobra para el visor: corrige posición y conserva rumbo IMU."""
        px, py, yaw = (_ack_num(ack, key) for key in ("px", "py", "yaw"))
        if px is None or py is None or yaw is None or self.odom == (px, py, yaw):
            return
        self._predict(px, py, yaw)
        a, b, surface = self.segs
        # La U de parking usa estado E y pnf 20..27 en el firmware actual.
        phase = _ack_num(ack, "pnf")
        turning = ",est=G," in ack or (phase is not None and 20 <= phase <= 27)
        h = self.x[2]
        fwd = np.array([math.sin(h), math.cos(h)])
        right = np.array([fwd[1], -fwd[0]])
        for key, name, _, kind in _SENSORS:
            if kind != "tof":
                continue
            z = _ack_num(ack, key)
            if z is None or z <= 0.0:
                continue
            if key == "tF":
                z -= C.LOC_MANEUVER_FRONT_OFFSET_MM
            limit = C.LOC_TOF_MAX_MM.get(key, 240.0)
            if not 20.0 <= z <= limit:
                continue
            lateral, longitudinal, angle = C.SENSOR_MOUNTS[name]
            origin = self.x[:2] + longitudinal * fwd + lateral * right
            rad = h + math.radians(angle)
            direction = np.array([math.sin(rad), math.cos(rad)])
            (expected,), (j,), (u,), (cosi,) = raycast(
                origin[None, :], direction[None, :], self.segs, face=True)
            if (surface[j] not in (_WALL, _BARRIER) or cosi <= math.cos(math.radians(35.0))
                    or not 20.0 <= expected <= limit):
                continue
            if surface[j] == _WALL and key in ("tL", "tR") and not turning and expected > 180.0:
                continue  # durante el escaneo, la madera puede tapar parte del eco lejano
            if surface[j] == _BARRIER and expected > 200.0:
                continue
            cone = rad + np.radians([-7.0, 7.0])
            rim = np.column_stack((np.sin(cone), np.cos(cone)))
            _, faces, _, _ = raycast(np.broadcast_to(origin, rim.shape), rim, self.segs, face=True)
            if np.any(faces != j):
                continue
            edge = b[j] - a[j]
            # Al borde, el cono del ToF puede mezclar la cara y el fondo.
            length = np.linalg.norm(edge)
            if min(u, 1.0 - u) * length < min(0.35 * length, max(12.0, 0.08 * expected)):
                continue
            normal = np.array([edge[1], -edge[0]])
            if surface[j] == _BARRIER and key != "tF" and normal @ self.park_axis != 0.0:
                # Las caras cuyo plano depende de w0 requieren el frontal calibrado.
                continue
            H = np.zeros(3)
            H[:2] = -normal / (direction @ normal)
            sigma = (C.LOC_MANEUVER_WOOD_SIGMA_MM if surface[j] == _BARRIER else 12.0) + 0.02 * z
            innovation = z - expected
            variance = float(H @ self.P @ H + sigma ** 2)
            if abs(innovation) > C.LOC_MANEUVER_MAX_INNOV_MM or innovation ** 2 > 9.0 * variance:
                self.n_rej += 1
                continue
            gain = self.P @ H / variance
            gain[2] = 0.0
            gain *= 0.25  # varios ACK pueden llevar ecos del mismo intervalo de muestreo
            self.x += gain * innovation
            residual = np.eye(3) - np.outer(gain, H)
            self.P = residual @ self.P @ residual.T + sigma ** 2 * np.outer(gain, gain)
            self.n_used += 1
            self.last[key] = (round(z), round(expected))

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
        if yaw is not None:
            self._prev_ack_yaw = yaw
        if not correct:
            return
        sin_us = getattr(C, "US_DESDE_TOF", False)
        tol = C.LOC_RACE_TOF_OCCLUSION_MM
        race_tc = _ack_num(ack, "tc") if tol is not None else None
        race = shifts is None and race_tc is not None and -1 <= race_tc < 12
        pair_bad = False
        pair_tol = getattr(C, "LOC_L23_PAIR_TOL_MM", None)
        tc = _ack_num(ack, "tc") if pair_tol is not None else None
        if tc is not None and 3 <= tc < 12:
            zl, zr = _ack_num(ack, "tL"), _ack_num(ack, "tR")
            if zl is not None and zr is not None and 20 <= zl <= 800 and 20 <= zr <= 800:
                el, sl, cl = self._expect(self.x, C.SENSOR_MOUNTS["tof_left"], "tof")
                er, sr, cr = self._expect(self.x, C.SENSOR_MOUNTS["tof_right"], "tof")
                # Between parallel island/outer walls their summed range is
                # independent of lateral pose. A short echo must not move it.
                pair_bad = ({sl, sr} == {_WALL, _ISLAND}
                            and min(cl, cr) >= math.cos(math.radians(35.0))
                            and abs((zl + zr) - (el + er)) > float(pair_tol))
                if pair_bad:
                    print(f"[LOC] par lateral incompatible medido={zl+zr:.0f} "
                          f"esperado={el+er:.0f}", flush=True)
        for key, mname, scale, kind in _SENSORS:
            if pair_bad and key in ("tL", "tR"):
                continue
            if kind == "us" and sin_us:
                continue   # dL/dR/dF son los mismos ToF (config.US_DESDE_TOF)
            raw = _ack_num(ack, key)
            if raw is None or raw <= 0:
                continue
            z = raw * scale
            prev = self._prev_z.get(key)
            self._prev_z[key] = z
            jump = getattr(C, "LOC_SENSOR_JUMP_MM", None)
            if jump is not None and prev is not None and abs(z - prev) > float(jump):
                continue   # primera lectura tras un salto: puede ser eco de transición
            if kind == "us" and (z < 30.0 or z > 1500.0):
                continue
            if kind == "tof" and (z < 20.0 or z > float(getattr(C, "LOC_TOF_MAX_MM", {}).get(key, 240.0))):
                continue
            mount = C.SENSOR_MOUNTS[mname]
            smax = getattr(C, "LOC_TOF_SIGN_MAX_MM", None)
            if (smax is not None and kind == "tof" and key in ("tL", "tR") and self.anchor_segs is not None
                    and not shifts and z <= float(smax)):
                zs = self._sign_cone(self.x, mount)
                if zs <= float(smax):
                    # Pared/isla/madera sin señales: con la señal en el rayo central la "pared" era la señal misma
                    # (zs == zw) y el eco se tiraba justo en las pasadas pegadas (orillas164 S/X2: z=29 esperado 105).
                    a_, b_, k_ = self.segs
                    m_ = k_ != _SIGN
                    zw = self._expect(self.x, mount, kind, (a_[m_], b_[m_], k_[m_]))[0]
                    # La señal queda delante de la pared en el cono y la lectura se parece más a ella que a la pared.
                    if zs < zw - 60.0 and abs(z - zs) < abs(z - zw):
                        self._sign_update(key, mount, z, zs, ack)
                        continue
            zh, surf, cosi = self._expect(self.x, mount, kind)
            if surf not in (_WALL, _ISLAND) or not math.isfinite(zh):
                continue
            if (kind == "tof" and getattr(C, "LOC_TOF_EXPECT_GATE", False)
                    and zh > float(getattr(C, "LOC_TOF_EXPECT_MAX_MM", {}).get(
                        key, getattr(C, "LOC_TOF_MAX_MM", {}).get(key, 240.0)))):
                continue   # la superficie queda más lejos que el rango útil: la lectura puede venir saturada
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
            if kind == "tof" and cosi < math.cos(math.radians(float(getattr(C, "LOC_TOF_MAX_INC_DEG", 35.0)))):
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
            far0 = getattr(C, "LOC_TOF_FAR_FROM_MM", None)
            if kind == "tof" and far0 is not None and z > float(far0):
                sig += float(getattr(C, "LOC_TOF_FAR_SIGMA_PER_MM", 0.0)) * (z - float(far0))
            f_sig = getattr(C, "LOC_TOF_FRONT_SIGMA_MM", None)
            if key == "tF" and f_sig is not None:
                sig = max(sig, float(f_sig))
            R = sig * sig
            nu = z - zh
            obl_deg = getattr(C, "LOC_TOF_OBLIQUE_DEG", None)
            obl_nu = getattr(C, "LOC_TOF_OBLIQUE_MAX_NU_MM", None)
            if (kind == "tof" and obl_deg is not None and obl_nu is not None
                    and cosi < math.cos(math.radians(float(obl_deg))) and nu < -float(obl_nu)):
                self._rej.pop(key, None)
                self.n_rej += 1
                continue   # eco oblicuo mucho más corto que lo esperado: no informa la pose (orillas243/252)
            S = float(H @ self.P @ H + R)
            # El eco corto compatible con un obstáculo no informa sobre la pared ni
            # debe alimentar su historial de reanclaje.
            if (race and kind == "tof" and nu < -35.0
                    and self._occlusion(mount, zh, z) <= tol):
                self._rej.pop(key, None)
                self.n_rej += 1
                continue
            if nu * nu / S > 9.0:                  # 3 sigma
                # Reanclaje: si el mismo sensor sigue diciendo lo mismo varias lecturas seguidas
                # (cajón mal fijado: la pose va ~40 cm corrida y el gate tiraba justo la lectura
                # que la arreglaba), se abre la P y se acepta.
                n_re = getattr(C, "LOC_REANCHOR_N", None)
                hist = self._rej.setdefault(key, [])
                hist.append(nu)
                del hist[:-int(n_re or 1)]
                if not (n_re and len(hist) >= int(n_re) and surf == _WALL
                        and max(hist) - min(hist) <= float(getattr(C, "LOC_REANCHOR_SPREAD_MM", 40.0))
                        and (min(hist) > 0 or max(hist) < 0)):
                    self.n_rej += 1
                    continue
                pm = float(getattr(C, "LOC_REANCHOR_P_MM", 400.0))
                self.P[:2, :2] += np.eye(2) * pm * pm
                S = float(H @ self.P @ H + R)
                self.n_reanchor += 1
                print(f"[LOC] reancla {key} nu={nu:+.0f} z={z:.0f} esperado={zh:.0f} "
                      f"pose=({self.x[0]:.0f},{self.x[1]:.0f})", flush=True)
            self._rej.pop(key, None)
            K = self.P @ H / S
            dx = K * nu
            self.x = self.x + dx
            self.P = (np.eye(3) - np.outer(K, H)) @ self.P
            self.n_used += 1
            self.last[key] = (round(z), round(zh))
            log_mm = getattr(C, "LOC_LOG_UPD_MM", None)
            if log_mm is not None and math.hypot(dx[0], dx[1]) >= float(log_mm):
                print(f"[LOCUPD] {key} z={z:.0f} esp={zh:.0f} nu={nu:+.0f} "
                      f"sup={'pared' if surf == _WALL else 'isla'} cos={cosi:.2f} "
                      f"dx={dx[0]:+.0f} dy={dx[1]:+.0f} dh={math.degrees(dx[2]):+.2f} "
                      f"pose=({self.x[0]:.0f},{self.x[1]:.0f},{math.degrees(self.x[2]):.0f}) tc={_ack_num(ack, 'tc')}",
                      flush=True)
