"""
Mapa digital vacío del tapete.

El carro sabe el cajón y el sentido. La altura en la recta sale del sonar
frontal (pared de adelante) y del ToF de atrás (pared de atrás, solo cuando
está cerca: la pared negra se apaga a ~25 cm). Entre medias se arrastra con
la odometría del ACK.

La cámara no dibuja la línea. Solo vota en cuál de los 6 asientos de la
recta (T1 T2 T3 T4 X1 X2) cayó la lata. Una recta solo puede quedar como
una carta del sorteo WRO. Confirmado el asiento, la lata se queda en ese punto. Solo salta a otro
de los 6 si la vista cae mucho más cerca de ese. La línea es el centro del
carril corrido al lado de paso: verde a la izquierda, rojo a la derecha.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from . import config as C
from .wro_field import (
    CARDS,
    INNER_HALF_MM,
    OUTER_HALF_MM,
    PARKING_FORBIDDEN_SEATS,
    SEATS,
    SECTIONS_CW,
    _barrier_corners,
    _heading,
    section_to_world,
    stall_start_xy,
)

_LANE_MM = 1000.0
_SPAN_MM = OUTER_HALF_MM * 2.0          # pared a pared, 3000
_FRONT_MOUNT_MM = C.SENSOR_MOUNTS["us_front"][1]    # sonar frontal, desde el eje trasero
_REAR_MOUNT_MM = -C.SENSOR_MOUNTS["tof_rear"][1]    # ToF trasero, detrás del eje
# La hoja BEV nace en el eje delantero; la pose del mapa es el eje trasero.
_BEV_AHEAD_MM = float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0))
_CAM_AHEAD_MM = float(getattr(C, "CAMERA_FWD_MM", 140.0))  # cámara, desde el eje trasero
# El pie de la lata es la cara que mira a la cámara; el centro queda detrás.
_FOOT_TO_CENTER_MM = float(getattr(C, "SIGN_FOOT_TO_CENTER_MM", 25.0))
# Sonar lateral derecho (der, adelante) desde el eje trasero; el izquierdo es espejo.
_US_SIDE_MOUNT = (abs(C.SENSOR_MOUNTS["us_right"][0]), C.SENSOR_MOUNTS["us_right"][1])
# La isla ocupa este tramo del along de cada recta (pared de atrás = 0).
_ISLAND_A0 = OUTER_HALF_MM - INNER_HALF_MM
_ISLAND_A1 = OUTER_HALF_MM + INNER_HALF_MM
# _wall_fix: rumbo máx. contra la recta, compuerta, ganancia y tope por frame.
_WALL_FIX_MAX_DEG = 12.0
_WALL_GATE_MM = 150.0
_WALL_GAIN = 0.3
_WALL_STEP_MM = 20.0
# Rumbo por pendiente de pared: tramo mínimo, ganancia y tope por ventana.
_YAW_WIN_MM = 400.0
_YAW_GAIN = 0.6
_YAW_STEP_DEG = 3.0
_REAR_WALL_MAX_MM = 250.0               # pared negra, el ToF no ve más lejos
_CLEAR_MM = 130.0                       # del centro de la lata al centro del carro
_LAT_CAP_MM = 220.0
_SEAT_GATE_MM = 160.0
_VOTES = 4
_STEP_MM = 80.0
# Radio del redondeo de la esquina que dobla el PP (verde en la boca).
_KNEE_MM = 260.0
# Hasta dónde se confía en que la lata vista es la confirmada (ver _pull_known).
_PULL_GATE_MM = 170.0
_COL = {"T4": "L", "T2": "L", "X2": "M", "X1": "M", "T3": "R", "T1": "R"}
# Cada carta ocupa uno de estos conjuntos. Ninguna mezcla dos asientos de
# la misma columna, ni tres latas, ni X1/X2 con otra lata.
_CARD_SEAT_SETS = list({frozenset(s for s, _c in card) for card in CARDS})
# Hay que ver la lata mucho más cerca del otro asiento para soltar el actual.
_JUMP_MM = 120.0


def _ack_num(ack: str | None, key: str) -> float | None:
    if not ack:
        return None
    idx = ack.find(key + "=")
    if idx < 0:
        return None
    try:
        return float(ack[idx + len(key) + 1:].split(",")[0])
    except ValueError:
        return None


def _lane_pose(section: str, direction: str, along_mm: float, lat_mm: float) -> tuple[float, float, float]:
    """along: mm desde la pared de atrás. lat: + a la derecha del sentido."""
    h = _heading(section, direction)
    rad = math.radians(h)
    fx, fy = math.sin(rad), math.cos(rad)
    rx, ry = math.cos(rad), -math.sin(rad)
    # Centro del carril en la pared de atrás.
    centers = {"N": (0.0, _LANE_MM), "E": (_LANE_MM, 0.0),
               "S": (0.0, -_LANE_MM), "W": (-_LANE_MM, 0.0)}
    cx, cy = centers[section]
    ox, oy = cx - fx * OUTER_HALF_MM, cy - fy * OUTER_HALF_MM
    x = ox + fx * along_mm + rx * lat_mm
    y = oy + fy * along_mm + ry * lat_mm
    return x, y, h


def _seat_world(section: str) -> list[tuple[str, float, float]]:
    return [(sid, *section_to_world(section, h, w)) for sid, (h, w) in SEATS.items()]


class DigitalMap:
    def __init__(self) -> None:
        self.direction = str(getattr(C, "DIGITAL_MAP_DIRECTION", "CW"))
        self.parking = str(getattr(C, "DIGITAL_MAP_PARKING", "W"))
        self.section = self.parking
        self.along_mm = self._stall_along()
        self.lat_mm = 0.0
        self.heading = _heading(self.section, self.direction)
        self.in_stall = True
        self._od: float | None = None
        self._tc = 0
        self._votes: dict[tuple[str, str], dict[str, int]] = {}
        self.confirmed: dict[tuple[str, str], str] = {}
        self._best: dict[tuple[str, str], float] = {}
        self.pose_xy = self._stall_xy()
        self.line_world: list[tuple[float, float]] = []
        self._view_heading = self.heading
        self._aligned = False
        self._line_shifts: list[float] = []
        self._odom0: tuple[float, float] | None = None
        # Corrección acumulada por latas conocidas (mundo, mm).
        self._pose_fix = (0.0, 0.0)
        # Corrección de rumbo (grados, + horario) por la pendiente de la pared.
        self._yaw_fix = 0.0
        self._odom_last = (0.0, 0.0)
        self._dr_xy = (0.0, 0.0)
        self._dr_step = (0.0, 0.0)
        # Ventana de muestras (recorrido, lateral a estima, lateral por pared).
        self._wall_win: list[tuple[float, float, float]] = []
        self._wall_win_sec: str | None = None
        self._wall_rej: list[tuple[float, float]] = []
        self._dr_s = 0.0
        self._dr_lat = 0.0
        self._yaw_step = 0.0
        self._at_corner = False
        self._orange_y: float | None = None
        self._has_pass = False
        self._latched: dict[str, tuple[float, float]] = {}
        self._dF: float | None = None
        self._pp_turn_tc: int | None = None
        self._tpr = 12

    def _stall_along(self) -> float:
        # El cajón está fuera de la pared, del lado de atrás del w de la recta.
        # along negativo = todavía no entró al carril.
        return -200.0

    def _stall_xy(self) -> tuple[float, float]:
        # Misma cifra que el arranque en cajón (wro_field.stall_start_xy): no
        # lee la pose real. Es el EJE TRASERO (lo que integra el odómetro).
        return stall_start_xy(self.parking, self.direction)

    def section_of(self, turns: int) -> str:
        i = SECTIONS_CW.index(self.parking)
        step = 1 if self.direction == "CW" else -1
        return SECTIONS_CW[(i + step * turns) % 4]

    def update(self, ack: str | None, feet: list[tuple[float, float, str]],
               orange: dict | None = None) -> None:
        """feet: (bev_x, bev_y, color). orange: la cinta, para saber que la recta se acaba."""
        tc = _ack_num(ack, "tc")
        est = None
        if ack and "est=" in ack:
            est = ack.split("est=", 1)[1][:1]
        turns = int(tc) if tc is not None else self._tc
        if turns != self._tc:
            self._tc = turns
            self.section = self.section_of(turns)
            self.in_stall = False
            self.lat_mm = 0.0
        if est in ("S", "C", "R") and self.in_stall:
            self.in_stall = False
            self.section = self.section_of(turns)

        self._dF = _ack_num(ack, "dF")
        tpr = _ack_num(ack, "tpr")
        if tpr is not None:
            self._tpr = int(tpr)
        self._pose_from_ack(ack)
        if getattr(C, "DIGITAL_MAP_WALL_FIX", False) and not self.in_stall:
            h = math.radians(_heading(self.section, self.direction))
            wx, wy = self._dr_step
            self._dr_s += abs(wx * math.sin(h) + wy * math.cos(h))
            self._dr_lat += wx * math.cos(h) - wy * math.sin(h)
            if self._wall_win_sec != self.section:
                self._wall_win, self._wall_win_sec = [], self.section
                self._dr_lat = 0.0
            if est in ("S", "C", "R"):
                self._wall_fix(ack)
        self._note_orange(orange)
        if not self.in_stall:
            self.along_mm, self.lat_mm = self._section_frame(
                self.pose_xy[0], self.pose_xy[1], self.section)
        self._aligned = (not self.in_stall) and self._odom0 is not None

        self._vote(feet)
        if not self.in_stall:
            self.along_mm, self.lat_mm = self._section_frame(
                self.pose_xy[0], self.pose_xy[1], self.section)
        self.line_world = self._build_line()

    def _pose_from_ack(self, ack: str | None) -> None:
        """Uniciclo: px/py ya integran ds del odómetro con el yaw de la IMU.

        px crece hacia adelante del arranque y py a la izquierda. El arranque
        mira en el sentido de carrera de la recta del cajón (antes estaba fijo
        a cajón W / CW = norte, y en las otras semillas el mapa giraba mal).
        El rumbo del twin es el opuesto del yaw (CW).
        """
        px = _ack_num(ack, "px")
        py = _ack_num(ack, "py")
        yaw = _ack_num(ack, "yaw")
        if px is None or py is None or yaw is None:
            return
        if self._odom0 is None:
            self._odom0 = (px, py)
            self._odom_last = (px, py)
            self._dr_xy = self._stall_xy()
        h0 = _heading(self.parking, self.direction)
        r0 = math.radians(h0)
        fx, fy = math.sin(r0), math.cos(r0)       # adelante del arranque
        lx, ly = -math.cos(r0), math.sin(r0)      # izquierda del arranque
        dpx, dpy = px - self._odom_last[0], py - self._odom_last[1]
        self._odom_last = (px, py)
        # Incremental: cada tramo del odómetro se gira con la corrección de
        # rumbo vigente (_yaw_fix, 0 sin DIGITAL_MAP_WALL_FIX = igual que antes).
        wx, wy = fx * dpx + lx * dpy, fy * dpx + ly * dpy
        if self._yaw_fix:
            c, s_ = math.cos(math.radians(self._yaw_fix)), math.sin(math.radians(self._yaw_fix))
            wx, wy = wx * c + wy * s_, wy * c - wx * s_
        self._dr_xy = (self._dr_xy[0] + wx, self._dr_xy[1] + wy)
        self._dr_step = (wx, wy)
        self.pose_xy = (self._dr_xy[0] + self._pose_fix[0],
                        self._dr_xy[1] + self._pose_fix[1])
        new_h = (h0 - yaw + self._yaw_fix) % 360.0
        self._yaw_step = abs((new_h - self._view_heading + 180.0) % 360.0 - 180.0)
        self._view_heading = new_h
        self.heading = self._view_heading

    def _note_orange(self, orange: dict | None) -> None:
        self._orange_y = None
        self._at_corner = False
        if not orange or not orange.get("seen"):
            return
        # La naranja arrastrada a ciegas (dead reckoning) es la esquina que
        # ya quedó atrás: con ella toda lata de adelante "pasaba la cinta" y
        # votaba en la recta siguiente (3170839, E/X2 nunca se asentó).
        if orange.get("dead_reckoned"):
            return
        ny = orange.get("near_y")
        if ny is None:
            return
        self._orange_y = float(ny)
        fwd = (C.ROBOT_BEV_Y - float(ny)) * C.MM_PER_PX
        # La cinta pegada al carro es el final de esta recta: toca girar.
        self._at_corner = fwd < 350.0

    def _along_of(self, x: float, y: float, section: str) -> float:
        x0, y0, h = _lane_pose(section, self.direction, 0.0, 0.0)
        dx, dy = x - x0, y - y0
        rad = math.radians(h)
        return dx * math.sin(rad) + dy * math.cos(rad)

    def _section_frame(self, x: float, y: float, section: str) -> tuple[float, float]:
        """(along desde la pared de atrás, derecha del sentido)."""
        x0, y0, h = _lane_pose(section, self.direction, 0.0, 0.0)
        dx, dy = x - x0, y - y0
        rad = math.radians(h)
        along = dx * math.sin(rad) + dy * math.cos(rad)
        right = dx * math.cos(rad) - dy * math.sin(rad)
        return along, right

    def _vote(self, feet: list[tuple[float, float, str]]) -> None:
        self._live = []
        if not feet:
            return
        px, py = self.pose_xy
        h = math.radians(self._view_heading)
        sections = [self.section]
        if not self.in_stall:
            sections.append(self.section_of(self._tc + 1))
        catalog: list[tuple[str, str, float, float]] = []
        for sec in sections:
            for sid, x, y in _seat_world(sec):
                along, right = self._section_frame(x, y, sec)
                catalog.append((sec, sid, along, right))
        for bx, by, color in feet:
            if color not in ("Red", "Green"):
                continue
            lat = (bx - C.ROBOT_BEV_X) * C.MM_PER_PX
            fwd = (C.ROBOT_BEV_Y - by) * C.MM_PER_PX
            if fwd < 80.0 or fwd > 2200.0 or abs(lat) > 1400.0:
                continue
            # Hoja -> eje trasero, y del pie al centro de la lata (sobre el
            # rayo de la cámara). Sin esto la lata caía ~1 fila más cerca.
            fwd += _BEV_AHEAD_MM
            ray = math.hypot(fwd - _CAM_AHEAD_MM, lat)
            if ray > 1e-6:
                fwd += _FOOT_TO_CENTER_MM * (fwd - _CAM_AHEAD_MM) / ray
                lat += _FOOT_TO_CENTER_MM * lat / ray
            wx = px + fwd * math.sin(h) + lat * math.cos(h)
            wy = py + fwd * math.cos(h) - lat * math.sin(h)
            # Delante de la naranja sigue esta recta. Pasada la cinta, es la siguiente.
            past = self._orange_y is not None and by < self._orange_y - 12.0
            ranked = []
            for sec, sid, sa, sr in catalog:
                if past and sec == self.section:
                    continue
                if self._orange_y is not None and not past and sec != self.section:
                    continue
                fa, fr = self._section_frame(wx, wy, sec)
                ranked.append((math.hypot(fa - sa, fr - sr), sec, sid, fr, sr, fa, sa))
            if not ranked:
                continue
            ranked.sort(key=lambda t: t[0])
            near = [
                c for c in ranked
                if c[0] <= 320.0 and self._seat_ok(c[1], c[2], color, wx, wy)
            ]
            if not near:
                continue
            dist, sec, sid, fr, sr, fa, sa = near[0]
            if len(near) > 1 and near[1][0] - dist < 80.0:
                a, b = near[0], near[1]
                pick = a if abs(a[4] - a[3]) <= abs(b[4] - b[3]) else b
                dist, sec, sid, fr, sr, fa, sa = pick
            stuck = self._stick_seat(sec, sid, color, wx, wy)
            if stuck != sid:
                sid = stuck
                sx, sy = self._seat_xy(sec, sid)
                dist = math.hypot(wx - sx, wy - sy)
                sa, sr = self._section_frame(sx, sy, sec)
                fr = sr
            if dist < 260.0 and abs(fr - sr) < 110.0:
                side = sr + 200.0 if color == "Red" else sr - 200.0
                side = max(-260.0, min(260.0, side))
                self._live.append((sec, sa, side))
            self._pull_known(wx, wy, color)
            key = (sec, sid)
            self._best[key] = min(self._best.get(key, 9999.0), dist)
            bucket = self._votes.setdefault(key, {"Red": 0, "Green": 0})
            bucket[color] += 1
            other = "Red" if color == "Green" else "Green"
            if (bucket[color] >= 3 and self._best[key] <= 160.0
                    and bucket[color] >= bucket[other] + 2
                    and self._seat_ok(sec, sid, color, wx, wy)):
                self._keep_closer_row(sec, sid, color)
        for sec, sa, side in self._live:
            if side >= -40.0:
                continue
            prev = self._latched.get(sec)
            if prev is None:
                self._latched[sec] = (sa, side)
            elif side < prev[1]:
                self._latched[sec] = (prev[0], side)

    def _keep_closer_row(self, sec: str, sid: str, color: str) -> None:
        """T1 y T3 son la misma altura, solo cambia la fila. Se queda la más cercana."""
        col = _COL[sid]
        mine = self._best.get((sec, sid), 9999.0)
        for other, ocol in _COL.items():
            if other == sid or ocol != col:
                continue
            rival = (sec, other)
            theirs = self._best.get(rival, 9999.0)
            if theirs + 80.0 < mine:
                return
            if mine + 80.0 < theirs:
                self.confirmed.pop(rival, None)
            elif rival in self.confirmed:
                return
        key = (sec, sid)
        if self.confirmed.get(key) != color:
            self.confirmed[key] = color
            print(f"[DMAP] {sec}/{sid} {color}", flush=True)

    def _seat_xy(self, sec: str, sid: str) -> tuple[float, float]:
        return next((x, y) for name, x, y in _seat_world(sec) if name == sid)

    def _xy_of(self, sec: str, sid: str) -> tuple[float, float]:
        return self._seat_xy(sec, sid)

    def _stick_seat(self, sec: str, sid: str, color: str, wx: float, wy: float) -> str:
        """La lata no sale de su asiento. Solo salta a otro si queda mucho más cerca."""
        sx, sy = self._seat_xy(sec, sid)
        d_new = math.hypot(wx - sx, wy - sy)
        best_old: str | None = None
        best_d = 1e9
        for osid in _COL:
            if osid == sid:
                continue
            key = (sec, osid)
            if key not in self._votes and key not in self.confirmed:
                continue
            if self.confirmed.get(key) not in (None, color):
                continue
            ox, oy = self._seat_xy(sec, osid)
            d = math.hypot(wx - ox, wy - oy)
            if d < best_d:
                best_d, best_old = d, osid
        if best_old is None:
            return sid
        if best_d > d_new + 250.0:
            return sid
        if d_new + _JUMP_MM < best_d:
            self._move_seat((sec, best_old), (sec, sid), color)
            return sid
        return best_old

    def _move_seat(self, old: tuple[str, str], new: tuple[str, str], color: str) -> None:
        if old == new or new in self.confirmed:
            return
        for store in (self._votes, self._best):
            if old in store and new not in store:
                store[new] = store.pop(old)
            else:
                store.pop(old, None)
        was = self.confirmed.pop(old, None)
        if was is not None:
            self.confirmed[new] = color
            print(f"[DMAP] {old[0]}/{old[1]} -> {new[0]}/{new[1]} {color}", flush=True)

    def _allows(self, sec: str, seats: set[str]) -> bool:
        """El conjunto cabe en alguna carta. En el cajón, solo la fila interior."""
        if not seats:
            return True
        if sec == self.parking and seats & set(PARKING_FORBIDDEN_SEATS):
            return False
        frozen = frozenset(seats)
        return any(frozen <= legal for legal in _CARD_SEAT_SETS)

    def _x2_taken(self, color: str, sec: str) -> bool:
        """Las cartas 9 y 10 son únicas: un solo X2 de cada color en el tapete."""
        for (s, sid), col in self.confirmed.items():
            if sid == "X2" and col == color and s != sec:
                return True
        return False

    def _seat_ok(self, sec: str, sid: str, color: str, wx: float, wy: float) -> bool:
        """Este asiento no rompe la carta de la recta."""
        if sec == self.parking and sid in PARKING_FORBIDDEN_SEATS:
            return False
        if sid == "X2" and self._x2_taken(color, sec):
            return False
        have = {s for (sc, s) in self.confirmed if sc == sec}
        if sid in have:
            return True
        rivals = {s for s in have if _COL[s] == _COL[sid]}
        if not self._allows(sec, (have - rivals) | {sid}):
            return False
        if not rivals:
            return True
        sx, sy = self._seat_xy(sec, sid)
        rx, ry = self._seat_xy(sec, next(iter(rivals)))
        return math.hypot(wx - sx, wy - sy) + 50.0 < math.hypot(wx - rx, wy - ry)

    def _pull_known(self, wx: float, wy: float, color: str) -> None:
        """Corrige la pose con la lata confirmada de ese color más cercana.

        Antes solo con el asiento votado a < 100 mm: tras un giro el odómetro
        ya trae 70–110 mm de error (semillas 1, 2, 6), la lata caía en la otra
        fila y nunca corregía. Se busca entre las confirmadas de esta recta y
        solo si no hay otra del mismo color que pueda ser la vista.
        """
        sec = self.section
        # El ACK es del frame anterior: girando, la vista sale rotada unos
        # grados y la lata cae 5–10 cm al lado (corregía hacia el lado malo).
        if self.in_stall or self._yaw_step > 3.0:
            return
        cands = sorted(
            (math.hypot(wx - x, wy - y), sid)
            for (s, sid), col in self.confirmed.items() if s == sec and col == color
            for x, y in [self._seat_xy(s, sid)]
        )
        if not cands or cands[0][0] > _PULL_GATE_MM:
            return
        if len(cands) > 1 and cands[1][0] < cands[0][0] + 120.0:
            return
        self._pull_pose(wx, wy, sec, cands[0][1])

    def _pull_pose(self, wx: float, wy: float, sec: str, sid: str) -> None:
        """La lata conocida corrige el odómetro. Solo con el carro ya derecho."""
        if self._odom0 is None:
            return
        sec_h = _heading(sec, self.direction)
        err = (self._view_heading - sec_h + 180.0) % 360.0 - 180.0
        if abs(err) > 45.0:
            return
        sx, sy = next((x, y) for name, x, y in _seat_world(sec) if name == sid)
        ex, ey = (sx - wx) * 0.3, (sy - wy) * 0.3
        mag = math.hypot(ex, ey)
        if mag < 4.0:
            return
        if mag > 35.0:
            ex, ey = ex * 35.0 / mag, ey * 35.0 / mag
        self._pose_fix = (self._pose_fix[0] + ex, self._pose_fix[1] + ey)
        self.pose_xy = (self.pose_xy[0] + ex, self.pose_xy[1] + ey)

    def _occupied(self) -> list[tuple[str, float, float]]:
        """Asientos con lata (confirmada o con votos): (recta, x, y)."""
        keys = set(self.confirmed) | {k for k, v in self._votes.items() if v["Red"] or v["Green"]}
        return [(sec, *self._seat_xy(sec, sid)) for sec, sid in keys]

    def _yaw_from_wall(self, lw: float) -> None:
        """Rumbo por la pendiente: si a estima el lateral cambia distinto que
        contra la pared, el gyro trae un sesgo (escala ~1 %: ~10° en 3 vueltas)."""
        win = self._wall_win
        if win and self._dr_s - win[-1][0] > 250.0:
            win.clear()
        win.append((self._dr_s, self._dr_lat, lw))
        if win[-1][0] - win[0][0] < _YAW_WIN_MM:
            return
        n = len(win)
        ms = sum(w[0] for w in win) / n
        mr = sum(w[1] - w[2] for w in win) / n
        sxx = sum((w[0] - ms) ** 2 for w in win)
        if n < 6 or sxx < 1.0:
            win.clear()
            return
        b = sum((w[0] - ms) * (w[1] - w[2] - mr) for w in win) / sxx
        win.clear()
        step = math.degrees(math.atan(b)) * _YAW_GAIN
        step = max(-_YAW_STEP_DEG, min(_YAW_STEP_DEG, step))
        self._yaw_fix -= step

    def _wall_fix(self, ack: str | None) -> None:
        """Corrige la pose con los sonares contra las paredes conocidas.

        Lateral: el lateral del lado de afuera siempre ve la pared exterior;
        el de adentro solo frente a la isla. Longitudinal: el frontal contra la
        pared del fondo de la recta. Solo con el carro casi derecho, sin lata
        conocida entre el sonar y la pared, y si la lectura cae cerca de lo
        esperado (si no, lo que vio es una lata o la punta de la isla). Con
        encoder la deriva es de ~50–100 mm por giro y ninguna lata la corregía
        en las rectas sin latas confirmadas (semillas 13, 20: 350–430 mm).
        """
        if self._yaw_step > 3.0:
            return
        sec = self.section
        e = (self._view_heading - _heading(sec, self.direction) + 180.0) % 360.0 - 180.0
        if abs(e) > _WALL_FIX_MAX_DEG:
            return
        er = math.radians(e)
        ce, se = math.cos(er), math.sin(er)
        a, l = self._section_frame(self.pose_xy[0], self.pose_xy[1], sec)
        occ = []
        for osec, x, y in self._occupied():
            sa, sl = self._section_frame(x, y, sec)
            occ.append((sa, sl))
        outer_right = self.direction == "CCW"
        da = dl = 0.0
        # ── Lateral ──
        meas = []
        strong = False
        mr, mf = _US_SIDE_MOUNT
        # Los dos sonares suman el ancho del carril frente a la isla: es pared a
        # pared (una lata no da esa suma) y vale aunque la deriva pase la compuerta.
        dLw, dRw = _ack_num(ack, "dL"), _ack_num(ack, "dR")
        s_al0 = a + mf * ce
        if (dLw is not None and dRw is not None and 2.0 < dLw < 110.0 and 2.0 < dRw < 110.0
                and _ISLAND_A0 + 150.0 <= s_al0 <= _ISLAND_A1 - 150.0
                and abs(10.0 * (dLw + dRw) + 2.0 * mr * ce - _LANE_MM) < 40.0):
            l_both = 0.5 * ((-_LANE_MM / 2.0 + 10.0 * dLw + mr * ce)
                            + (_LANE_MM / 2.0 - 10.0 * dRw - mr * ce)) - mf * se
            if abs(l_both - l) < 2.0 * _LANE_MM / 5.0:
                meas.append(l_both)
                strong = abs(l_both - l) >= _WALL_GATE_MM
                self._wall_rej.clear()
        for key, side in (("dL", -1.0), ("dR", 1.0)):
            if meas:
                break
            d = _ack_num(ack, key)
            if d is None or d <= 2.0 or d >= 110.0:
                continue
            d *= 10.0
            s_lat = l + side * mr * ce + mf * se
            s_al = a + mf * ce - side * mr * se
            is_outer = (side > 0) == outer_right
            if not is_outer and not (_ISLAND_A0 + 150.0 <= s_al <= _ISLAND_A1 - 150.0):
                continue
            if not (300.0 <= s_al <= _SPAN_MM - 300.0):
                continue
            wall = side * _LANE_MM / 2.0
            if any(abs(sa - s_al) < 160.0 and min(s_lat, wall) - 30.0 < sl < max(s_lat, wall) + 30.0
                   for sa, sl in occ):
                continue
            # El cono toma la distancia perpendicular (|e| < medio cono).
            l_meas = wall - side * (d + mr * ce) - mf * se
            if abs(l_meas - l) < _WALL_GATE_MM:
                meas.append(l_meas)
                if is_outer:
                    self._wall_rej.clear()
            elif is_outer:
                # Fuera de la compuerta: si la pared exterior insiste con la misma
                # cifra varios frames seguidos, la deriva es real (no una lata).
                rej = self._wall_rej
                if rej and self._dr_s - rej[-1][0] > 150.0:
                    rej.clear()
                rej.append((self._dr_s, l_meas - l))
                n_rej = C.fr(6)
                if len(rej) >= n_rej:
                    inn = [r[1] for r in rej[-n_rej:]]
                    mu = sum(inn) / float(n_rej)
                    if max(inn) - min(inn) < 50.0 and abs(mu) < 450.0:
                        meas.append(l + mu)
                        strong = True
                        rej.clear()
        if meas:
            lw = sum(meas) / len(meas)
            dl = (lw - l) * (0.6 if strong else _WALL_GAIN)
            if getattr(C, "DIGITAL_MAP_WALL_YAW", True):
                self._yaw_from_wall(lw)
        # ── Longitudinal ──
        d = _ack_num(ack, "dF")
        if d is not None and 5.0 < d < 110.0 and abs(e) <= 8.0:
            d *= 10.0
            s_al = a + _FRONT_MOUNT_MM * ce
            s_lat = l + _FRONT_MOUNT_MM * se
            reach = _SPAN_MM - s_al
            cone = math.tan(math.radians(17.0))
            blocked = any(s_al < sa < _SPAN_MM and abs(sl - s_lat) < 60.0 + (sa - s_al) * cone
                          for sa, sl in occ)
            # La punta de la isla (along 2000) también cae en el cono.
            inner = 1.0 if not outer_right else -1.0
            if s_al < _ISLAND_A1:
                gap = _LANE_MM / 2.0 - inner * s_lat
                if gap < 40.0 + (_ISLAND_A1 - s_al) * cone:
                    blocked = True
            # La siguiente recta no tiene latas en este tramo; su fila exterior
            # sí puede caer en el cono si va pegado a la isla (ya en occ).
            if not blocked:
                a_meas = _SPAN_MM - d - _FRONT_MOUNT_MM * ce
                if abs(a_meas - a) < _WALL_GATE_MM and reach > 0.0:
                    da = (a_meas - a) * _WALL_GAIN
        if not da and not dl:
            return
        da = max(-_WALL_STEP_MM, min(_WALL_STEP_MM, da))
        if not strong:
            dl = max(-_WALL_STEP_MM, min(_WALL_STEP_MM, dl))
        h = math.radians(_heading(sec, self.direction))
        ex = da * math.sin(h) + dl * math.cos(h)
        ey = da * math.cos(h) - dl * math.sin(h)
        self._pose_fix = (self._pose_fix[0] + ex, self._pose_fix[1] + ey)
        self.pose_xy = (self.pose_xy[0] + ex, self.pose_xy[1] + ey)

    def _next_section(self, sec: str) -> str:
        i = SECTIONS_CW.index(sec)
        step = 1 if self.direction == "CW" else -1
        return SECTIONS_CW[(i + step) % 4]

    def _cans_on(self, sec: str, along0: float, along1: float, latch: bool = True) -> list[tuple[float, float]]:
        """(altura, lado de paso) de las latas confirmadas en ese tramo."""
        out = []
        for (s, sid), color in self.confirmed.items():
            if s != sec:
                continue
            sx, sy = self._xy_of(sec, sid)
            sa, sr = self._section_frame(sx, sy, sec)
            if sa < along0 - 80.0 or sa > along1 + 40.0:
                continue
            side = sr + 200.0 if color == "Red" else sr - 200.0
            for osid, ox, oy in _seat_world(sec):
                if osid == sid or _COL.get(osid) != _COL.get(sid):
                    continue
                _, osr = self._section_frame(ox, oy, sec)
                if abs(osr - side) < 130.0:
                    side = osr + 90.0 if color == "Red" else osr - 90.0
            side = max(-320.0, min(320.0, side))
            out.append((sa, side))
        if latch:
            latched = self._latched.get(sec)
            if latched is not None and not any(abs(latched[0] - c[0]) < 250.0 for c in out):
                out.append(latched)
        taken = [c[0] for c in out]
        for lsec, sa, side in getattr(self, "_live", ()):
            if lsec != sec or sa < along0 - 80.0 or sa > along1 + 40.0:
                continue
            if any(abs(sa - csa) < 200.0 for csa in taken):
                continue
            out.append((sa, side))
        out.sort()
        return out

    def _put(self, pts, shifts, sec: str, along: float, shift: float) -> None:
        x, y, _ = _lane_pose(sec, self.direction, along, shift)
        x = max(-1320.0, min(1320.0, x))
        y = max(-1320.0, min(1320.0, y))
        if pts and abs(pts[-1][0] - x) < 20.0 and abs(pts[-1][1] - y) < 20.0:
            return
        pts.append((x, y))
        shifts.append(shift)

    def _ease(self, sec: str, along: float) -> float:
        """Lado de paso de la lata que toca. Verde a la izquierda, rojo a la derecha."""
        cans = self._cans_on(sec, 0.0, 3200.0)
        if not cans:
            return 0.0

        def smooth(u: float) -> float:
            u = max(0.0, min(1.0, u))
            return u * u * (3.0 - 2.0 * u)

        def onto(sa: float, side: float) -> float:
            enter, done = sa - 1100.0, sa - 80.0
            if along <= enter:
                return 0.0
            if along >= done:
                return side
            return side * smooth((along - enter) / (done - enter))

        if along <= cans[0][0]:
            return onto(cans[0][0], cans[0][1])
        if along >= cans[-1][0]:
            side = cans[-1][1]
            # Verde: se queda a la izquierda el resto de la recta. Volver al
            # centro mete la línea en el siguiente verde, que todavía no se vio.
            if side < -40.0:
                return side
            back = cans[-1][0] + 120.0
            if along <= back:
                return side
            u = smooth((along - back) / 480.0)
            return side + (0.0 - side) * u
        prev = cans[0]
        nxt = cans[-1]
        for can in cans:
            if can[0] <= along:
                prev = can
            else:
                nxt = can
                break
        span = nxt[0] - prev[0]
        hold, arrive = prev[0] + span * 0.25, prev[0] + span * 0.7
        if span >= 600.0:
            # Cambio de lado (verde -> rojo) en 1 m: con 25–70 % el cruce era
            # de 45 cm para 60 cm de lado a lado (~53°) y el carro rozaba la
            # segunda lata (3170839, S/T4 -> S/T1). Se arranca apenas sale la
            # cola de la primera y se llega 15 cm antes de la segunda.
            hold, arrive = prev[0] + 120.0, nxt[0] - 150.0
        if along <= hold:
            return prev[1]
        if along >= arrive:
            return nxt[1]
        return prev[1] + (nxt[1] - prev[1]) * smooth((along - hold) / (arrive - hold))

    def _left_pass(self, sec: str) -> tuple[float, float] | None:
        """Primera lata de la recta si se pasa por la izquierda (verde)."""
        cans = self._cans_on(sec, 0.0, 3200.0, latch=False)
        if not cans or cans[0][1] >= -40.0:
            return None
        return cans[0]

    def _green_ahead(self) -> tuple[float, float] | None:
        """(cuánto falta, lado) hasta el lado izquierdo del verde siguiente."""
        if not getattr(C, "DIGITAL_MAP_STEER", False) or self.in_stall or not self._aligned:
            return None
        nxt = self._next_section(self.section)
        lead = self._left_pass(nxt)
        if lead is None:
            return None
        corner = _lane_pose(self.section, self.direction, 2500.0, 0.0)
        na, _ = self._section_frame(corner[0], corner[1], nxt)
        hx, hy, _ = _lane_pose(nxt, self.direction, na, lead[1])
        against_wall = max(abs(hx), abs(hy)) > 1250.0
        hx = max(-1100.0, min(1100.0, hx))
        hy = max(-1100.0, min(1100.0, hy))
        dx, dy = hx - self.pose_xy[0], hy - self.pose_xy[1]
        h = math.radians(self.heading)
        fwd = dx * math.sin(h) + dy * math.cos(h)
        return fwd, against_wall

    def blocks_turn(self) -> bool:
        """Sigue bloqueado hasta que el arco del giro cabe a la izquierda del verde."""
        if self._tc >= self._tpr:
            # Vuelta 12 cerrada: no hay más esquinas. Con prio=1 el ESP nunca
            # arrancaba la U del estacionamiento (vigilarUturn espera libre).
            return False
        if self._pp_turn_tc == self._tc and not self.in_stall:
            return True
        ahead = self._green_ahead()
        if ahead is None:
            return False
        # Antes aquí se soltaba "ya en la esquina y del lado de afuera". Falso
        # con el verde en la primera columna de la recta siguiente (3170839,
        # S/T4): el giro rápido desde la exterior sale por dentro de la lata.
        block = ahead[0] > (480.0 if ahead[1] else 260.0)
        # Ya cerca de la pared y sin soltar: el giro rápido no alcanza (la
        # ventana es 40-95 cm y el ESP tarda ~15 cm en entrar). La esquina
        # la dobla la línea del mapa con prio=1 hasta que el ESP la cuente
        # por el gyro (GIRO_PI_CUENTA_DEG) y suba tc.
        pp_cm = float(getattr(C, "DIGITAL_MAP_PP_TURN_CM", 0.0))
        if block and pp_cm > 0.0 and self._dF is not None and 0.0 < self._dF <= pp_cm:
            self._pp_turn_tc = self._tc
            print(f"[DMAP] esquina por PP tc={self._tc} dF={self._dF:.0f}", flush=True)
        return block

    def _inner_lead(self, sec: str) -> bool:
        """La primera lata de la recta se pasa por el lado de adentro."""
        cans = self._cans_on(sec, 0.0, 3200.0, latch=False)
        if not cans or cans[0][0] > 1300.0:
            return False
        side = cans[0][1]
        # CW gira a la derecha: adentro = derecha (+). CCW al revés.
        return side > 40.0 if self.direction == "CW" else side < -40.0

    def holds_for_center(self) -> bool:
        """Giro rápido todavía no: el arco saldría pegado a la isla.

        El giro rápido dispara en cuanto el frontal baja de 95 cm y con el
        radio mínimo (~8 cm de avance) sale a ~10 cm de la isla. Se aguanta
        hasta DIGITAL_MAP_TURN_HOLD_CM (centro del carril), o hasta
        DIGITAL_MAP_TURN_HOLD_INNER_CM si la primera lata de la siguiente se
        pasa por dentro (lado de paso, pero no pegado a la isla).
        """
        hold = self._hold_cm()
        if hold <= 0.0 or not getattr(C, "DIGITAL_MAP_STEER", False) or self._tc >= self._tpr:
            return False
        if self.in_stall or not self._aligned or self._dF is None or self._dF <= 0.0:
            return False
        if self.along_mm < 1500.0:
            return False
        err = (self.heading - _heading(self.section, self.direction) + 180.0) % 360.0 - 180.0
        if abs(err) > 30.0:
            return False
        return self._dF > hold

    def _hold_cm(self) -> float:
        if self._inner_lead(self._next_section(self.section)):
            return float(getattr(C, "DIGITAL_MAP_TURN_HOLD_INNER_CM", 0.0))
        return float(getattr(C, "DIGITAL_MAP_TURN_HOLD_CM", 0.0))

    def _hold_along(self) -> float | None:
        """Altura de la recta donde se suelta el giro (para que la línea no doble antes)."""
        hold = self._hold_cm()
        if hold <= 0.0:
            return None
        return _SPAN_MM - _FRONT_MOUNT_MM - hold * 10.0

    def needs_line(self) -> bool:
        """El carro va del lado malo de la lata. El gyro lo deja irse derecho."""
        if not getattr(C, "DIGITAL_MAP_STEER", False) or self.in_stall or not self._aligned:
            return False
        target = self._ease(self.section, self.along_mm + 180.0)
        if target < -40.0:
            return self.lat_mm > target + 70.0
        # Rojo igual (3: sale del giro encima de la fila del rojo y el gyro
        # sigue derecho hasta pegarle).
        if target > 40.0:
            return self.lat_mm < target - 70.0
        return False

    def hold_pasado(self) -> bool:
        """No enderezar: RECUPERANDO se iría derecho y el giro cortaría el verde."""
        # Tampoco si todavía no llega al lado de paso de la lata que viene:
        # RECUPERANDO lo endereza a medio camino y se la lleva (semilla 3).
        if self.needs_line():
            return True
        ahead = self._green_ahead()
        if ahead is None:
            return False
        nxt = self._next_section(self.section)
        along, lat = self._section_frame(self.pose_xy[0], self.pose_xy[1], nxt)
        lead = self._left_pass(nxt)
        if lead is None:
            return False
        on_left = lat <= lead[1] + 80.0
        past = along >= lead[0] - 40.0
        return not (on_left and past)

    def _build_line(self) -> list[tuple[float, float]]:
        """Entra al lado de paso y a la esquina con una curva, no con un escalón."""
        if self.in_stall:
            self._line_shifts = []
            self._has_pass = False
            self._steer_ok = False
            return []
        join = 2500.0
        sec = self.section
        along = self.along_mm
        if along > join + 40.0:
            sec = self._next_section(sec)
            along, _ = self._section_frame(self.pose_xy[0], self.pose_xy[1], sec)
        along = min(max(along, 0.0), join - 40.0)
        step = 50.0
        pts: list[tuple[float, float]] = []
        shifts: list[float] = []
        _, car_lat = self._section_frame(self.pose_xy[0], self.pose_xy[1], sec)
        corner = _lane_pose(sec, self.direction, join, 0.0)
        nxt = self._next_section(sec)
        na, _ = self._section_frame(corner[0], corner[1], nxt)
        # Ya pasada la zona de esta recta: si la siguiente abre con verde,
        # no se vuelve al centro (eso deja la línea del lado contrario).
        # Se aguanta el lado de esta recta y la curva sube a la izquierda
        # del verde antes de doblar.
        lead = self._left_pass(nxt)
        mine = self._cans_on(sec, 0.0, 3200.0)
        end = join - 420.0
        hold_at = self._hold_along() if lead is None and sec == self.section else None
        if hold_at is not None:
            # Recto hasta el punto de giro: con prio=1 el PP seguiría la curva
            # y doblaría antes de tiempo.
            end = max(end, hold_at + 100.0)
        knee = None
        if lead is not None:
            # La esquina la dobla el PP (prio=1). Vértice = cruce del lado del
            # carro en esta recta con el lado de paso del verde en la otra;
            # se redondea con _KNEE_MM. Antes el arco salía del centro del
            # carril: con el carro del lado de adentro la curva terminaba
            # detrás de él y el PP seguía derecho a la pared.
            sh2 = max(-320.0, min(320.0, lead[1]))
            ka = _lane_pose(sec, self.direction, 0.0, car_lat)
            kb = _lane_pose(nxt, self.direction, 0.0, sh2)
            hs = math.radians(_heading(sec, self.direction))
            fsx, fsy = math.sin(hs), math.cos(hs)
            t_k = (kb[0] - ka[0]) * fsx + (kb[1] - ka[1]) * fsy
            knee = (ka[0] + fsx * t_k, ka[1] + fsy * t_k)
            end = t_k - _KNEE_MM
        a = along
        while a < end:
            target = self._ease(sec, a)
            if lead is not None and mine and a >= mine[-1][0]:
                # No regresar al rojo: la línea de adelante se queda del lado
                # del carro para poder subir a la izquierda del verde.
                target = car_lat
            u = min(1.0, max(0.0, (a - along) / 280.0))
            u = u * u * (3.0 - 2.0 * u)
            self._put(pts, shifts, sec, a, car_lat + (target - car_lat) * u)
            a += step
        sh0 = car_lat if lead is not None else self._ease(sec, max(a - 40.0, along))
        p0 = _lane_pose(sec, self.direction, max(a - 40.0, along), sh0)
        hrad = math.radians(_heading(sec, self.direction))
        rx, ry = math.cos(hrad), -math.sin(hrad)
        if lead is not None and knee is not None:
            bow = knee
            kn_a, _ = self._section_frame(knee[0], knee[1], nxt)
            p2 = _lane_pose(nxt, self.direction, kn_a + _KNEE_MM, sh2)
            p2 = (p2[0], p2[1])
        else:
            sh2 = self._ease(nxt, na + 700.0)
            p2 = _lane_pose(nxt, self.direction, na + 700.0, sh2)
            right_amt = max(0.0, (p2[0] - p0[0]) * rx + (p2[1] - p0[1]) * ry)
            bow = (p0[0] + rx * right_amt, p0[1] + ry * right_amt)
        for i in range(1, 13):
            t = i / 12.0
            u = 1.0 - t
            x = u * u * p0[0] + 2 * u * t * bow[0] + t * t * p2[0]
            y = u * u * p0[1] + 2 * u * t * bow[1] + t * t * p2[1]
            if pts and abs(pts[-1][0] - x) < 12 and abs(pts[-1][1] - y) < 12:
                continue
            x = max(-1320.0, min(1320.0, x))
            y = max(-1320.0, min(1320.0, y))
            pts.append((x, y))
            shifts.append(max(abs(sh0), abs(sh2)) * math.sin(t * math.pi))
        b = na + 700.0
        if lead is not None and knee is not None:
            b = self._section_frame(p2[0], p2[1], nxt)[0] + step
        while b < na + 1100.0:
            if lead is not None and b < lead[0] + 220.0:
                sh = lead[1]
            else:
                sh = self._ease(nxt, b)
            self._put(pts, shifts, nxt, b, sh)
            b += step
        self._line_shifts = shifts
        self._has_pass = any(abs(s) > 12.0 for s in shifts)
        nxt = self._next_section(sec)
        self._steer_ok = any(s == sec or s == nxt for (s, _sid) in self.confirmed)
        return pts

    def _target_lat(self) -> float:
        """Lado de paso del asiento confirmado más cercano por delante."""
        best = None
        h = math.radians(self.heading)
        fx, fy = math.sin(h), math.cos(h)
        rx, ry = math.cos(h), -math.sin(h)
        px, py = self.pose_xy
        for (sec, sid), color in self.confirmed.items():
            if sec != self.section:
                continue
            sx, sy = self._xy_of(sec, sid)
            dx, dy = sx - px, sy - py
            fwd = dx * fx + dy * fy
            if fwd < 0.0:
                continue
            lat = dx * rx + dy * ry
            if best is None or fwd < best[0]:
                best = (fwd, lat, color)
        if best is None:
            return 0.0
        _, lat, color = best
        shift = lat + _CLEAR_MM if color == "Red" else lat - _CLEAR_MM
        return float(max(-_LAT_CAP_MM, min(_LAT_CAP_MM, shift)))

    def line_bev(self) -> list[tuple[int, int]]:
        """La línea del mapa, en la hoja de 400 px. Vacía si no hay nada que seguir."""
        if not self._aligned or not self._steer_ok or len(self.line_world) < 4:
            return []
        px, py = self.pose_xy
        h = math.radians(self.heading)
        out = []
        for wx, wy in self.line_world:
            dx, dy = wx - px, wy - py
            # La hoja nace en el eje delantero.
            fwd = dx * math.sin(h) + dy * math.cos(h) - _BEV_AHEAD_MM
            right = dx * math.cos(h) - dy * math.sin(h)
            bx = int(round(C.ROBOT_BEV_X + right / C.MM_PER_PX))
            by = int(round(C.ROBOT_BEV_Y - fwd / C.MM_PER_PX))
            if fwd < -80.0 or fwd > 900.0:
                continue
            if -160 <= bx < C.BEV_W + 160 and -40 <= by <= C.ROBOT_BEV_Y + 20:
                out.append((max(0, min(C.BEV_W - 1, bx)), max(0, min(C.ROBOT_BEV_Y - 1, by))))
        return out

    def render(self, size: int = 520) -> np.ndarray:
        img = np.full((size, size, 3), (232, 224, 208), np.uint8)
        scale = (size - 24) / _SPAN_MM

        def px(x: float, y: float) -> tuple[int, int]:
            return (int(round(size / 2 + x * scale)), int(round(size / 2 - y * scale)))

        def poly(pts, color, thick=1):
            p = np.array([px(a, b) for a, b in pts], np.int32)
            cv2.polylines(img, [p], True, color, thick, cv2.LINE_AA)

        half_o, half_i = OUTER_HALF_MM, INNER_HALF_MM
        outer = [(half_o, -half_o), (half_o, half_o), (-half_o, half_o), (-half_o, -half_o)]
        inner = [(half_i, -half_i), (half_i, half_i), (-half_i, half_i), (-half_i, -half_i)]
        poly(outer, (40, 40, 40), 2)
        poly(inner, (40, 40, 40), 2)
        for box in _barrier_corners(self.parking):
            poly(box, (180, 0, 180), 2)

        for sec in SECTIONS_CW:
            for sid, x, y in _seat_world(sec):
                col = self.confirmed.get((sec, sid))
                votes = self._votes.get((sec, sid))
                if col is None and votes:
                    col = "Red" if votes["Red"] > votes["Green"] else "Green"
                    if max(votes.values()) <= 0:
                        col = None
                if col == "Red":
                    solid = (sec, sid) in self.confirmed
                    cv2.circle(img, px(x, y), 5, (0, 0, 220), -1 if solid else 1, cv2.LINE_AA)
                elif col == "Green":
                    solid = (sec, sid) in self.confirmed
                    cv2.circle(img, px(x, y), 5, (0, 170, 0), -1 if solid else 1, cv2.LINE_AA)
                else:
                    cv2.circle(img, px(x, y), 4, (150, 150, 150), 1, cv2.LINE_AA)

        if len(self.line_world) >= 2:
            pts = np.array([px(x, y) for x, y in self.line_world], np.int32)
            cv2.polylines(img, [pts], False, (0, 180, 220), 2, cv2.LINE_AA)

        cx, cy = self.pose_xy
        rad = math.radians(self.heading)
        tip = px(cx + 80 * math.sin(rad), cy + 80 * math.cos(rad))
        # Chasis a escala: pose_xy es el eje trasero.
        fx, fy = math.sin(rad), math.cos(rad)
        rx, ry = fy, -fx
        hw = C.ROBOT_WIDTH_MM / 2.0
        body = [(cx + a * fx + b * rx, cy + a * fy + b * ry)
                for a, b in ((-C.REAR_OVERHANG_MM, -hw), (-C.REAR_OVERHANG_MM, hw),
                             (C.ROBOT_LENGTH_MM - C.REAR_OVERHANG_MM, hw),
                             (C.ROBOT_LENGTH_MM - C.REAR_OVERHANG_MM, -hw))]
        poly(body, (30, 30, 30), 1)
        cv2.circle(img, px(cx, cy), 3, (30, 30, 30), -1, cv2.LINE_AA)
        cv2.arrowedLine(img, px(cx, cy), tip, (30, 30, 30), 1, tipLength=0.4)
        cv2.putText(
            img, f"{self.section}  {self.along_mm:.0f}mm",
            (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 40, 40), 1, cv2.LINE_AA,
        )
        if not self.confirmed:
            cv2.putText(img, "sin latas", (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (80, 80, 80), 1, cv2.LINE_AA)
        return img
