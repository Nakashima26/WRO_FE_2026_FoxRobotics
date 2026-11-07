"""
Mapa de pista WRO obstáculos: geometría, localización y mapa de señales.

Convención de heading de campo (FieldPose): grados CCW desde +x (este).
  0° = este, 90° = norte. Distinto del twin (0° = norte, CW+); convertir con
  field_heading_to_twin / field_heading_from_twin.

Secciones: índice 0 = recta sur (y ∈ [-1500,-500], |x|≤500). Numeración en
orden de marcha: dir 'L' → S,E,N,W; dir 'R' → S,W,N,E.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Literal

import numpy as np

from . import config as C
from .wro_field import (
    INNER_HALF_MM,
    OUTER_HALF_MM,
    PARKING_FORBIDDEN_SEATS,
    SEATS,
    SIGN_MM,
    section_to_world,
    orange_segments,
    blue_segments,
)

# ── Tunables (config.py puede sobreescribir) ────────────────────────────────

# El programa oficial no corre las señales del cajón: prohíbe T3/T4/X2 en esa
# recta (PARKING_FORBIDDEN_SEATS). Se deja el corrimiento por si una sede lo hace.
SEAT_SHIFT_PARKING_MM = getattr(C, "SEAT_SHIFT_PARKING_MM", 0.0)
PARKING_SECTION_INDEX = getattr(C, "PARKING_SECTION_INDEX", 0)
SIGN_CONFIRM_SCORE = getattr(C, "SIGN_CONFIRM_SCORE", 2.0)
SIGN_GATE_ALONG_MM = getattr(C, "SIGN_GATE_ALONG_MM", 150.0)
SIGN_GATE_LAT_MM = getattr(C, "SIGN_GATE_LAT_MM", 110.0)
SIGN_CAMERA_AHEAD_MM = getattr(C, "SIGN_CAMERA_AHEAD_MM", 60.0)
SIGN_FOOT_TO_CENTER_MM = getattr(C, "SIGN_FOOT_TO_CENTER_MM", 25.0)
SIGN_PASSED_MARGIN_MM = getattr(C, "SIGN_PASSED_MARGIN_MM", 30.0)
SIGN_USE_CARD_PRIOR = getattr(C, "SIGN_USE_CARD_PRIOR", True)
LOC_LATERAL_GAIN = getattr(C, "LOC_LATERAL_GAIN", 0.3)
LOC_ALONG_GAIN = getattr(C, "LOC_ALONG_GAIN", 0.3)
LOC_HEADING_GAIN = getattr(C, "LOC_HEADING_GAIN", 0.0)
LOC_LATERAL_HEADING_MAX_DEG = getattr(C, "LOC_LATERAL_HEADING_MAX_DEG", 15.0)
LOC_ALONG_HEADING_MAX_DEG = getattr(C, "LOC_ALONG_HEADING_MAX_DEG", 10.0)
FUSION_US_TOF_MAX_DIFF_MM = getattr(C, "FUSION_US_TOF_MAX_DIFF_MM", 40.0)
FUSION_TOF_WEIGHT = getattr(C, "FUSION_TOF_WEIGHT", 0.8)
GATE_US_MM = getattr(C, "GATE_US_MM", 120.0)
GATE_TOF_MM = getattr(C, "GATE_TOF_MM", 60.0)
MM_PER_PX = getattr(C, "MM_PER_PX", 2.0)
ROBOT_BEV_X = getattr(C, "ROBOT_BEV_X", 200.0)
ROBOT_BEV_Y = getattr(C, "ROBOT_BEV_Y", 380.0)
BEV_ORIGIN_AHEAD_MM = getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0)

WRO_SECTIONS = ("S", "W", "N", "E")
# Salida en la recta sur: con vueltas a la izquierda se va al este y la que
# sigue es la recta este; con vueltas a la derecha, la oeste.
DRIVE_ORDER_L = ("S", "E", "N", "W")
DRIVE_ORDER_R = ("S", "W", "N", "E")

ColorName = Literal["red", "green"]
SideStatus = Literal["ok", "us_only", "tof_only", "inconsistent", "none"]


def field_heading_to_twin(h_field_deg: float) -> float:
    return (90.0 - h_field_deg) % 360.0


def field_heading_from_twin(h_twin_deg: float) -> float:
    return (90.0 - h_twin_deg) % 360.0


def _norm_h(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _color_norm(c: str) -> ColorName:
    return "green" if c.lower().startswith("g") else "red"


@dataclass
class FieldPose:
    x_mm: float
    y_mm: float
    heading_deg: float  # CCW desde +x (este)


@dataclass(frozen=True)
class SeatDef:
    section_index: int
    wro_section: str
    seat_id: str
    x_mm: float
    y_mm: float


def _shift_toward_island(wro_sec: str, x: float, y: float, dist: float) -> tuple[float, float]:
    if wro_sec == "N":
        return x, y - dist
    if wro_sec == "S":
        return x, y + dist
    if wro_sec == "E":
        return x - dist, y
    return x + dist, y


@dataclass
class TrackGeometry:
    """Paredes y asientos en frame campo (origen centro, +x este, +y norte)."""

    drive_dir: str = "L"
    parking_section: int = PARKING_SECTION_INDEX
    seat_shift_parking_mm: float = SEAT_SHIFT_PARKING_MM

    def __post_init__(self) -> None:
        self.drive_dir = self.drive_dir.upper()
        if self.drive_dir not in ("L", "R"):
            raise ValueError("drive_dir L|R")
        self._order = DRIVE_ORDER_L if self.drive_dir == "L" else DRIVE_ORDER_R
        self.outer_half = OUTER_HALF_MM
        self.inner_half = INNER_HALF_MM
        self._seats: list[SeatDef] = []
        for si, wsec in enumerate(self._order):
            for seat_id, (h, w) in SEATS.items():
                if si == self.parking_section and seat_id in PARKING_FORBIDDEN_SEATS:
                    continue
                x, y = section_to_world(wsec, h, w)
                if si == self.parking_section and self.seat_shift_parking_mm:
                    x, y = _shift_toward_island(wsec, x, y, self.seat_shift_parking_mm)
                self._seats.append(SeatDef(si, wsec, seat_id, x, y))

    @property
    def walls_outer(self) -> list[tuple[float, float]]:
        h = self.outer_half
        return [(h, -h), (h, h), (-h, h), (-h, -h)]

    @property
    def walls_inner(self) -> list[tuple[float, float]]:
        h = self.inner_half
        return [(h, -h), (h, h), (-h, h), (-h, -h)]

    def seats_all(self) -> list[SeatDef]:
        return list(self._seats)

    def seats_of_section(self, section_index: int) -> list[SeatDef]:
        return [s for s in self._seats if s.section_index == section_index]

    def wro_section_name(self, section_index: int) -> str:
        return self._order[section_index]

    def start_field_pose(self) -> FieldPose:
        h = 0.0 if self.drive_dir == "L" else 180.0
        return FieldPose(0.0, -1000.0, h)

    def section_of(self, x: float, y: float) -> int | None:
        if self._in_corner(x, y):
            return None
        for si, wsec in enumerate(self._order):
            if self._in_straight(wsec, x, y):
                return si
        return None

    def corner_of(self, x: float, y: float) -> int | None:
        if not self._in_corner(x, y):
            return None
        # 0=SE, 1=SW, 2=NW, 3=NE (índice por cuadrante)
        if x >= 0 and y < 0:
            return 0
        if x < 0 and y < 0:
            return 1
        if x < 0 and y >= 0:
            return 2
        return 3

    def section_frame(
        self, section_index: int
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Origen en centro de recta, u_along (unit), u_lat (hacia isla)."""
        wsec = self._order[section_index]
        if wsec == "S":
            o = np.array([0.0, -1000.0])
            u_along = np.array([1.0, 0.0]) if self.drive_dir == "L" else np.array([-1.0, 0.0])
            u_lat = np.array([0.0, 1.0])
        elif wsec == "N":
            o = np.array([0.0, 1000.0])
            u_along = np.array([-1.0, 0.0]) if self.drive_dir == "L" else np.array([1.0, 0.0])
            u_lat = np.array([0.0, -1.0])
        elif wsec == "E":
            o = np.array([1000.0, 0.0])
            u_along = np.array([0.0, 1.0]) if self.drive_dir == "L" else np.array([0.0, -1.0])
            u_lat = np.array([-1.0, 0.0])
        else:
            o = np.array([-1000.0, 0.0])
            u_along = np.array([0.0, -1.0]) if self.drive_dir == "L" else np.array([0.0, 1.0])
            u_lat = np.array([1.0, 0.0])
        return o, u_along, math.degrees(math.atan2(u_along[1], u_along[0]))

    def along_lateral(
        self, section_index: int, x: float, y: float
    ) -> tuple[float, float]:
        o, u_along, _ = self.section_frame(section_index)
        _, u_lat, _ = self._lat_from_section(section_index)
        p = np.array([x, y]) - o
        return float(np.dot(p, u_along)), float(np.dot(p, u_lat))

    def _lat_from_section(self, section_index: int) -> tuple[np.ndarray, np.ndarray, float]:
        wsec = self._order[section_index]
        o, u_along, ah = self.section_frame(section_index)
        if wsec == "S":
            u_lat = np.array([0.0, 1.0])
        elif wsec == "N":
            u_lat = np.array([0.0, -1.0])
        elif wsec == "E":
            u_lat = np.array([-1.0, 0.0])
        else:
            u_lat = np.array([1.0, 0.0])
        return o, u_lat, ah

    def _in_straight(self, wsec: str, x: float, y: float) -> bool:
        ih, oh = self.inner_half, self.outer_half
        if wsec == "S":
            return abs(x) <= ih and -oh <= y <= -ih
        if wsec == "N":
            return abs(x) <= ih and ih <= y <= oh
        if wsec == "E":
            return ih <= x <= oh and abs(y) <= ih
        return -oh <= x <= -ih and abs(y) <= ih

    def _in_corner(self, x: float, y: float) -> bool:
        ih, oh = self.inner_half, self.outer_half
        in_ring = abs(x) <= oh and abs(y) <= oh
        in_hole = abs(x) < ih and abs(y) < ih
        if not in_ring or in_hole:
            return False
        return not any(self._in_straight(w, x, y) for w in WRO_SECTIONS)

    def orange_lines(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        return orange_segments()

    def blue_lines(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        return blue_segments()


@dataclass
class TrackFrame:
    """ODOM (x adelante, y izq, yaw CCW+) ↔ campo."""

    geometry: TrackGeometry
    pose_field: FieldPose = field(default_factory=lambda: FieldPose(0, -1000, 0))
    _odom_ref: tuple[float, float, float] | None = None
    _field_ref: FieldPose | None = None
    _R: np.ndarray | None = None

    def set_from_odom(
        self, x_od: float, y_od: float, yaw_od: float,
        start: FieldPose | None = None,
    ) -> None:
        """La pose ODOM dada (yaw del ESP incluido, que no tiene por qué ser 0 al
        armar) corresponde a `start` en el campo."""
        start = start or self.geometry.start_field_pose()
        self.pose_field = FieldPose(start.x_mm, start.y_mm, start.heading_deg)
        self._odom_ref = (x_od, y_od, yaw_od)
        self._field_ref = FieldPose(start.x_mm, start.y_mm, start.heading_deg)
        self._R = self._rot_odom_to_field(start.heading_deg - yaw_od)

    def shift(self, dx_mm: float, dy_mm: float) -> None:
        """Corrige la pose de arranque (p. ej. con Localizer.snap_lateral)."""
        if self._field_ref is not None:
            self._field_ref.x_mm += dx_mm
            self._field_ref.y_mm += dy_mm

    def odom_to_field(
        self, x_od: float, y_od: float, yaw_od: float
    ) -> FieldPose:
        if self._odom_ref is None or self._R is None or self._field_ref is None:
            self.set_from_odom(x_od, y_od, yaw_od)
        ox, oy, oyaw = self._odom_ref
        fr = self._field_ref
        d = np.array([x_od - ox, y_od - oy])
        p = self._R @ d + np.array([fr.x_mm, fr.y_mm])
        h = _norm_h(fr.heading_deg + (yaw_od - oyaw))
        self.pose_field = FieldPose(float(p[0]), float(p[1]), h)
        return self.pose_field

    def apply_field_pose(self, pose: FieldPose) -> None:
        self.pose_field = pose

    def init_lateral_from_sides(
        self,
        d_left_mm: float | None,
        d_right_mm: float | None,
        corridor_half_mm: float = 500.0,
    ) -> None:
        """Centro de pasillo si falta un lado."""
        if d_left_mm is not None and d_right_mm is not None:
            lat = (d_right_mm - d_left_mm) * 0.5
        else:
            lat = 0.0
        sec = self.geometry.section_of(self.pose_field.x_mm, self.pose_field.y_mm)
        if sec is None:
            sec = 0
        _, u_lat, _ = self.geometry._lat_from_section(sec)
        self.pose_field.x_mm += u_lat[0] * lat
        self.pose_field.y_mm += u_lat[1] * lat

    @staticmethod
    def _rot_odom_to_field(start_h_field: float) -> np.ndarray:
        a = math.radians(start_h_field)
        return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


@dataclass
class SensorMount:
    right_mm: float
    fwd_mm: float
    dir_deg: float  # 0=adelante, +90=derecha


@dataclass
class SensorReading:
    name: str
    kind: Literal["us", "tof"]
    mount: SensorMount
    half_angle_deg: float
    value_mm: float | None


def _ray_seg(
    ox: float, oy: float, dx: float, dy: float, ax: float, ay: float, bx: float, by: float
) -> float | None:
    v2x, v2y = bx - ax, by - ay
    v3x, v3y = -dy, dx
    denom = v2x * v3x + v2y * v3y
    if abs(denom) < 1e-12:
        return None
    v1x, v1y = ox - ax, oy - ay
    cross = v2x * v1y - v2y * v1x
    t1 = cross / denom
    dot = v1x * v3x + v1y * v3y
    t2 = dot / denom
    if t1 >= 0 and 0.0 <= t2 <= 1.0:
        return t1
    return None


class _WallCaster:
    def __init__(self, geom: TrackGeometry) -> None:
        self._polys = [geom.walls_outer, geom.walls_inner]

    def cast(
        self, ox: float, oy: float, dx: float, dy: float, max_mm: float
    ) -> float:
        best = max_mm
        for poly in self._polys:
            n = len(poly)
            for i in range(n):
                a = poly[i]
                b = poly[(i + 1) % n]
                t = _ray_seg(ox, oy, dx, dy, a[0], a[1], b[0], b[1])
                if t is not None and t < best:
                    best = t
        return best


# Montajes (mm desde el eje trasero; dir 0 = adelante, +90 = derecha). Fuente
# única: config.SENSOR_MOUNTS (el twin lee lo mismo). Medirlos en el carro.
DEFAULT_MOUNTS = {
    name: SensorMount(float(r), float(f), float(d))
    for name, (r, f, d) in C.SENSOR_MOUNTS.items()
}
_SIDE_DEFAULT = {
    "left": ("us_left", "us"),
    "right": ("us_right", "us"),
    "front": ("us_front", "us"),
    "rear": ("tof_rear", "tof"),
}
US_HALF_ANGLE_DEG = getattr(C, "US_HALF_ANGLE_DEG", 15.0)


class Localizer:
    def __init__(self, geometry: TrackGeometry) -> None:
        self.geometry = geometry
        self.pose = geometry.start_field_pose()
        self.side_status: dict[str, SideStatus] = {
            "left": "none",
            "right": "none",
            "front": "none",
            "rear": "none",
        }
        self.last_innov: dict[str, float | None] = {k: None for k in self.side_status}
        self._caster = _WallCaster(geometry)
        self._lat_innov_ema = 0.0

    def predict(self, dx_fwd: float, dy_left: float, dyaw_deg: float) -> None:
        h0 = math.radians(self.pose.heading_deg)
        c, s = math.cos(h0), math.sin(h0)
        dx_f = dx_fwd * c - dy_left * s
        dy_f = dx_fwd * s + dy_left * c
        self.pose.x_mm += dx_f
        self.pose.y_mm += dy_f
        self.pose.heading_deg = _norm_h(self.pose.heading_deg + dyaw_deg)

    def update(
        self,
        readings: list[SensorReading],
        signmap: SignMap | None = None,
    ) -> FieldPose:
        by_side: dict[str, list[SensorReading]] = {}
        for r in readings:
            side = self._classify_side(r)
            by_side.setdefault(side, []).append(r)

        for side in ("left", "right", "front", "rear"):
            innov, status, ray = self._fuse_side(by_side.get(side, []), side, signmap)
            self.side_status[side] = status
            self.last_innov[side] = innov
            if innov is not None:
                self._apply_innov(side, innov, ray, signmap)
        return self.pose

    def corner_anchor(self, d_front_mm: float) -> None:
        r = SensorReading("us_front", "us", DEFAULT_MOUNTS["us_front"],
                          US_HALF_ANGLE_DEG, d_front_mm)
        pred = self._predict(r)
        if pred is None:
            return
        innov = d_front_mm - pred
        if abs(innov) > 2.0 * GATE_US_MM:
            return
        self._apply_innov("front", innov, self._ray_world(r.mount)[2:], None, gain_scale=0.5)

    def snap_lateral(
        self,
        readings: list[SensorReading],
        max_width_err_mm: float = 120.0,
        max_shift_mm: float = 450.0,
    ) -> bool:
        """Arranque: corre la pose de lado sin compuerta (en la salida el carro puede
        estar donde sea a lo ancho del pasillo). Un carro movido δ a la derecha lee δ
        menos a la derecha y δ más a la izquierda; si la suma de las innovaciones no
        cuadra con el ancho del pasillo (señal en un cono) no se toca nada."""
        if self._axis_section() is None:
            return False
        innov: dict[str, float] = {}
        for kind in ("tof", "us"):
            for r in readings:
                if r.kind != kind or r.value_mm is None:
                    continue
                side = self._classify_side(r)
                if side not in ("left", "right") or side in innov:
                    continue
                pred = self._predict(r)
                if pred is not None:
                    innov[side] = r.value_mm - pred
        if "left" not in innov or "right" not in innov:
            return False
        if abs(innov["left"] + innov["right"]) > max_width_err_mm:
            return False
        shift = 0.5 * (innov["left"] - innov["right"])
        if abs(shift) > max_shift_mm:
            return False
        h = math.radians(self.pose.heading_deg)
        self.pose.x_mm += shift * math.sin(h)
        self.pose.y_mm -= shift * math.cos(h)
        return True

    def _classify_side(self, r: SensorReading) -> str:
        d = r.mount.dir_deg % 360
        if 45 <= d < 135:
            return "right"
        if 135 <= d < 225:
            return "rear"
        if 225 <= d < 315:
            return "left"
        return "front"

    def _ray_world(
        self, mount: SensorMount, off_deg: float = 0.0
    ) -> tuple[float, float, float, float]:
        """Origen y dirección (campo) del rayo de un sensor. dir 0 = adelante, +90 = derecha."""
        h = math.radians(self.pose.heading_deg)
        fx, fy = math.cos(h), math.sin(h)
        rx, ry = math.sin(h), -math.cos(h)
        ox = self.pose.x_mm + mount.fwd_mm * fx + mount.right_mm * rx
        oy = self.pose.y_mm + mount.fwd_mm * fy + mount.right_mm * ry
        a = math.radians(mount.dir_deg + off_deg)
        return ox, oy, math.cos(a) * fx + math.sin(a) * rx, math.cos(a) * fy + math.sin(a) * ry

    def _predict(self, r: SensorReading) -> float | None:
        """Lectura esperada contra las paredes: el HC-SR04 da el eco más cercano de
        su cono; el ToF, el rayo central."""
        offs = (np.linspace(-r.half_angle_deg, r.half_angle_deg, 7)
                if r.kind == "us" else (0.0,))
        best = None
        for off in offs:
            ox, oy, dx, dy = self._ray_world(r.mount, float(off))
            d = self._caster.cast(ox, oy, dx, dy, 4000.0)
            if d < 3999.0 and (best is None or d < best):
                best = d
        return best

    def _predict_range(self, side: str, signmap: SignMap | None = None) -> float | None:
        name, kind = _SIDE_DEFAULT[side]
        half = US_HALF_ANGLE_DEG if kind == "us" else 0.0
        return self._predict(SensorReading(name, kind, DEFAULT_MOUNTS[name], half, None))

    def _fuse_side(
        self,
        rs: list[SensorReading],
        side: str,
        signmap: SignMap | None,
    ) -> tuple[float | None, SideStatus, tuple[float, float] | None]:
        """(innovación, estado, dirección del rayo). Sonar y ToF se comparan por su
        innovación contra la predicción de CADA uno (están montados distinto)."""
        cands: list[tuple[SensorReading, float]] = []
        for r in rs:
            if r.value_mm is None:
                continue
            pred = self._predict(r)
            if pred is not None:
                cands.append((r, r.value_mm - pred))
        us = next((c for c in cands if c[0].kind == "us"), None)
        tof = next((c for c in cands if c[0].kind == "tof"), None)
        if us is None and tof is None:
            return None, "none", None
        if us is not None and tof is not None:
            if abs(us[1] - tof[1]) <= FUSION_US_TOF_MAX_DIFF_MM:
                w = FUSION_TOF_WEIGHT
                return (w * tof[1] + (1 - w) * us[1], "ok",
                        self._ray_world(tof[0].mount)[2:])
            # No cuadran: el cono del sonar suele estar viendo una señal o la boca
            # de la esquina. El ToF (haz angosto) manda si cae dentro de su compuerta.
            if abs(tof[1]) <= GATE_TOF_MM:
                return tof[1], "tof_only", self._ray_world(tof[0].mount)[2:]
            if abs(us[1]) <= GATE_US_MM:
                return us[1], "us_only", self._ray_world(us[0].mount)[2:]
            return None, "inconsistent", None
        r, innov = us if us is not None else tof
        gate = GATE_US_MM if r.kind == "us" else GATE_TOF_MM
        if abs(innov) <= gate:
            return innov, ("us_only" if r.kind == "us" else "tof_only"), self._ray_world(r.mount)[2:]
        return None, "none", None

    def _axis_section(self) -> int | None:
        """Recta cuyos ejes valen para corregir: la que contiene al carro o, en una
        esquina, la que está alineada con su heading."""
        sec = self.geometry.section_of(self.pose.x_mm, self.pose.y_mm)
        if sec is not None:
            return sec
        if self.geometry.corner_of(self.pose.x_mm, self.pose.y_mm) is None:
            return None
        return min(range(4), key=lambda si: abs(_norm_h(
            self.pose.heading_deg - self.geometry.section_frame(si)[2])))

    def _apply_innov(
        self,
        side: str,
        innov: float,
        ray: tuple[float, float] | None,
        signmap: SignMap | None,
        gain_scale: float = 1.0,
    ) -> None:
        sec = self._axis_section()
        if sec is None or ray is None:
            return
        _o, u_along, axis_h = self.geometry.section_frame(sec)
        _o2, u_lat, _ = self.geometry._lat_from_section(sec)
        herr = _norm_h(self.pose.heading_deg - axis_h)
        if side in ("left", "right"):
            if abs(herr) >= LOC_LATERAL_HEADING_MAX_DEG:
                return
            axis, gain = u_lat, LOC_LATERAL_GAIN
            if LOC_HEADING_GAIN > 0:
                self._lat_innov_ema = 0.8 * self._lat_innov_ema + 0.2 * innov
                self.pose.heading_deg = _norm_h(
                    self.pose.heading_deg - LOC_HEADING_GAIN * self._lat_innov_ema * 0.01
                )
        else:
            if abs(herr) >= LOC_ALONG_HEADING_MAX_DEG:
                return
            if signmap is not None and self._sign_blocks_beam(side, signmap):
                return
            axis, gain = u_along, LOC_ALONG_GAIN
        proj = ray[0] * axis[0] + ray[1] * axis[1]
        if abs(proj) < 0.5:
            return
        # Leer más lejos de lo esperado = el carro está más atrás sobre su rayo.
        step = -gain * gain_scale * innov * proj
        self.pose.x_mm += float(axis[0]) * step
        self.pose.y_mm += float(axis[1]) * step

    def _sign_blocks_beam(self, side: str, signmap: SignMap) -> bool:
        # Simplificado: si hay señal confirmada cerca del rayo frontal, no corregir along.
        if side != "front":
            return False
        sec = self.geometry.section_of(self.pose.x_mm, self.pose.y_mm)
        if sec is None:
            return False
        for st in signmap.signs(sec, "confirmed"):
            along, lat = self.geometry.along_lateral(sec, st["x"], st["y"])
            my_along, my_lat = self.geometry.along_lateral(sec, self.pose.x_mm, self.pose.y_mm)
            if along > my_along and abs(lat - my_lat) < 80:
                return True
        return False


@dataclass
class _SeatState:
    red: float = 0.0
    green: float = 0.0
    empty: float = 0.0

    def best(self) -> tuple[ColorName | None, float]:
        if self.red <= 0 and self.green <= 0:
            return None, 0.0
        if self.red >= self.green:
            return "red", self.red
        return "green", self.green


def _range_weight(dist_mm: float) -> float:
    if dist_mm <= 400:
        return 1.0
    if dist_mm >= 900:
        return 0.0
    if dist_mm <= 800:
        return 1.0 - 0.7 * (dist_mm - 400) / 400
    return 0.3 * (900 - dist_mm) / 100


class SignMap:
    def __init__(self, geometry: TrackGeometry) -> None:
        self.geometry = geometry
        self._seat_def = {(s.section_index, s.seat_id): s for s in geometry.seats_all()}
        self._seat_states: dict[tuple[int, str], _SeatState] = {}
        self._free: list[dict[str, Any]] = []
        self._card_single_section: int | None = None

    def set_single_section(self, section_index: int) -> None:
        self._card_single_section = section_index

    def observe(
        self,
        detections: list[tuple[float, float, str]],
        pose_field: FieldPose,
        t: float,
        in_view_fn: Callable[[int, str], bool] | None = None,
    ) -> None:
        h_twin = field_heading_to_twin(pose_field.heading_deg)
        matched_seats: set[tuple[int, str]] = set()
        for bx, by, color in detections:
            c = _color_norm(color)
            wx, wy = self._bev_foot_to_field(bx, by, pose_field, h_twin)
            si, seat_id, dist = self._associate(wx, wy)
            w = _range_weight(dist)
            if w <= 0:
                continue
            if si is not None and seat_id is not None:
                key = (si, seat_id)
                st = self._seat_states.setdefault(key, _SeatState())
                if c == "red":
                    st.red += w
                else:
                    st.green += w
                matched_seats.add(key)
                self._apply_card_prior(si, seat_id, c)
            else:
                self._add_free(wx, wy, c, w, t)

        self._negative_evidence(pose_field, h_twin, matched_seats, in_view_fn)

    def _bev_foot_to_field(
        self, bx: float, by: float, pose: FieldPose, h_twin: float
    ) -> tuple[float, float]:
        right_mm = (bx - ROBOT_BEV_X) * MM_PER_PX
        fwd_mm = (ROBOT_BEV_Y - by) * MM_PER_PX + BEV_ORIGIN_AHEAD_MM
        cam_fwd = SIGN_CAMERA_AHEAD_MM
        dx = fwd_mm - cam_fwd
        dy = right_mm
        ln = math.hypot(dx, dy)
        if ln < 1e-6:
            cx, cy = fwd_mm, right_mm
        else:
            cx = fwd_mm + SIGN_FOOT_TO_CENTER_MM * dx / ln
            cy = right_mm + SIGN_FOOT_TO_CENTER_MM * dy / ln
        return self._robot_to_field(cx, cy, pose, h_twin)

    def _robot_to_field(
        self, fwd: float, right: float, pose: FieldPose, h_twin: float
    ) -> tuple[float, float]:
        h = math.radians(h_twin)
        x = pose.x_mm + fwd * math.sin(h) + right * math.cos(h)
        y = pose.y_mm + fwd * math.cos(h) - right * math.sin(h)
        return x, y

    def _associate(self, x: float, y: float) -> tuple[int | None, str | None, float]:
        si = self.geometry.section_of(x, y)
        if si is None:
            si = self._nearest_section(x, y)
        if si is None:
            return None, None, 1e9
        best_id = None
        best_d = 1e9
        o, u_along, _ = self.geometry.section_frame(si)
        _, u_lat, _ = self.geometry._lat_from_section(si)
        p = np.array([x, y]) - o
        al, lat = float(np.dot(p, u_along)), float(np.dot(p, u_lat))
        for seat in self.geometry.seats_of_section(si):
            p2 = np.array([seat.x_mm, seat.y_mm]) - o
            al2, lat2 = float(np.dot(p2, u_along)), float(np.dot(p2, u_lat))
            if abs(al - al2) > SIGN_GATE_ALONG_MM or abs(lat - lat2) > SIGN_GATE_LAT_MM:
                continue
            d = math.hypot(al - al2, lat - lat2)
            if d < best_d:
                best_d = d
                best_id = seat.seat_id
        if best_id is None:
            return si, None, math.hypot(x, y)
        return si, best_id, best_d

    def _nearest_section(self, x: float, y: float) -> int | None:
        best, bd = None, 1e9
        for si in range(4):
            o, _, _ = self.geometry.section_frame(si)
            d = math.hypot(x - o[0], y - o[1])
            if d < bd:
                bd, best = d, si
        return best

    def _apply_card_prior(self, si: int, seat_id: str, color: ColorName) -> None:
        if not SIGN_USE_CARD_PRIOR:
            return
        if self._card_single_section is not None and si != self._card_single_section:
            if seat_id == "X2":
                st = self._seat_states.setdefault((si, seat_id), _SeatState())
                st.red = st.green = 0.0

    def _add_free(self, x: float, y: float, color: ColorName, w: float, t: float) -> None:
        for tr in self._free:
            if math.hypot(tr["x"] - x, tr["y"] - y) < 120:
                tr["x"] = 0.7 * tr["x"] + 0.3 * x
                tr["y"] = 0.7 * tr["y"] + 0.3 * y
                tr["w"] += w * 0.5
                tr["color"] = color
                return
        self._free.append({"x": x, "y": y, "color": color, "w": w, "t": t})

    def _negative_evidence(
        self,
        pose: FieldPose,
        h_twin: float,
        matched: set[tuple[int, str]],
        in_view_fn: Callable[[int, str], bool] | None,
    ) -> None:
        sec = self.geometry.section_of(pose.x_mm, pose.y_mm)
        if sec is None:
            return
        for seat in self.geometry.seats_of_section(sec):
            key = (sec, seat.seat_id)
            if key in matched:
                continue
            if in_view_fn and not in_view_fn(sec, seat.seat_id):
                continue
            if not self._in_reliable_wedge(seat, pose, h_twin):
                continue
            if self._occluded_by_closer_sign(seat, pose, h_twin):
                continue
            st = self._seat_states.setdefault(key, _SeatState())
            st.empty += 0.2

    def _in_reliable_wedge(self, seat: SeatDef, pose: FieldPose, h_twin: float) -> bool:
        dx = seat.x_mm - pose.x_mm
        dy = seat.y_mm - pose.y_mm
        cam_x = pose.x_mm + SIGN_CAMERA_AHEAD_MM * math.sin(math.radians(h_twin))
        cam_y = pose.y_mm + SIGN_CAMERA_AHEAD_MM * math.cos(math.radians(h_twin))
        vx = seat.x_mm - cam_x
        vy = seat.y_mm - cam_y
        dist = math.hypot(vx, vy)
        if dist < 200 or dist > 650:
            return False
        body_fwd = (math.sin(math.radians(h_twin)), math.cos(math.radians(h_twin)))
        dot = vx * body_fwd[0] + vy * body_fwd[1]
        cos_lim = math.cos(math.radians(25))
        if dot <= 0 or dot / dist < cos_lim:
            return False
        return True

    def _occluded_by_closer_sign(self, seat: SeatDef, pose: FieldPose, h_twin: float) -> bool:
        """Una señal ya vista tapa al asiento si está más cerca de la cámara y en
        su línea de vista (dentro del ancho angular de la señal)."""
        hr = math.radians(h_twin)
        fx, fy = math.sin(hr), math.cos(hr)
        cam_x = pose.x_mm + SIGN_CAMERA_AHEAD_MM * fx
        cam_y = pose.y_mm + SIGN_CAMERA_AHEAD_MM * fy
        sx, sy = seat.x_mm - cam_x, seat.y_mm - cam_y
        d_seat = math.hypot(sx, sy)
        a_seat = math.atan2(sx, sy)
        for (si, sid), st in self._seat_states.items():
            col, sc = st.best()
            if col is None or sc < 0.5:
                continue
            sdef = self._seat_def.get((si, sid))
            if sdef is None or sdef is seat:
                continue
            vx, vy = sdef.x_mm - cam_x, sdef.y_mm - cam_y
            d = math.hypot(vx, vy)
            if d < 1.0 or d >= d_seat - 50.0 or vx * fx + vy * fy <= 0.0:
                continue
            half = math.atan2(SIGN_MM, d)
            if abs(_norm_h(math.degrees(math.atan2(vx, vy) - a_seat))) < math.degrees(half):
                return True
        return False

    def _state_label(self, st: _SeatState) -> str:
        col, sc = st.best()
        if col and sc >= SIGN_CONFIRM_SCORE:
            other = st.green if col == "red" else st.red
            if other <= 0.3 * sc:
                return "confirmed"
        if col and sc > 0.3:
            return "tentative"
        if st.empty > 1.0 and (st.red + st.green) < 0.5:
            return "empty"
        return "unknown"

    def seats(self, section_index: int) -> list[dict[str, Any]]:
        out = []
        for seat in self.geometry.seats_of_section(section_index):
            st = self._seat_states.get((section_index, seat.seat_id), _SeatState())
            col, sc = st.best()
            out.append(
                {
                    "seat": seat.seat_id,
                    "x": seat.x_mm,
                    "y": seat.y_mm,
                    "color": col,
                    "score": sc,
                    "empty": st.empty,
                    "state": self._state_label(st),
                }
            )
        return out

    def signs(
        self, section_index: int, min_state: str = "tentative"
    ) -> list[dict[str, Any]]:
        order = {"unknown": 0, "empty": 0, "tentative": 1, "confirmed": 2}
        need = order.get(min_state, 1)
        out = []
        for s in self.seats(section_index):
            if s["state"] == "empty":
                continue
            if order.get(s["state"], 0) >= need and s["color"] is not None:
                out.append(s)
        return out

    def query_bev(
        self, pose_field: FieldPose, sections: list[int] | None = None
    ) -> list[tuple[float, float, str, float]]:
        if sections is None:
            sections = list(range(4))
        h_twin = field_heading_to_twin(pose_field.heading_deg)
        out: list[tuple[float, float, str, float]] = []
        for si in sections:
            for s in self.signs(si, "tentative"):
                if s["color"] is None:
                    continue
                bx, by = self._field_center_to_bev(s["x"], s["y"], pose_field, h_twin)
                out.append((bx, by, s["color"], s["score"]))
        return out

    def _field_center_to_bev(
        self, cx: float, cy: float, pose: FieldPose, h_twin: float
    ) -> tuple[float, float]:
        h = math.radians(h_twin)
        dx = cx - pose.x_mm
        dy = cy - pose.y_mm
        fwd = dx * math.sin(h) + dy * math.cos(h)
        right = dx * math.cos(h) - dy * math.sin(h)
        cam_fwd = SIGN_CAMERA_AHEAD_MM
        vx, vy = fwd - cam_fwd, right
        ln = math.hypot(vx, vy)
        if ln > 1e-6:
            fwd -= SIGN_FOOT_TO_CENTER_MM * vx / ln
            right -= SIGN_FOOT_TO_CENTER_MM * vy / ln
        bx = ROBOT_BEV_X + right / MM_PER_PX
        by = ROBOT_BEV_Y - (fwd - BEV_ORIGIN_AHEAD_MM) / MM_PER_PX
        return bx, by

    def passed(self, section_index: int, seat_id: str, pose_field: FieldPose) -> bool:
        seat = next(
            s
            for s in self.geometry.seats_of_section(section_index)
            if s.seat_id == seat_id
        )
        along_s, _ = self.geometry.along_lateral(section_index, seat.x_mm, seat.y_mm)
        along_r, _ = self.geometry.along_lateral(section_index, pose_field.x_mm, pose_field.y_mm)
        o, u_along, _ = self.geometry.section_frame(section_index)
        return along_r > along_s + SIGN_PASSED_MARGIN_MM

    def summary(self) -> str:
        n_conf = sum(
            1
            for k, st in self._seat_states.items()
            if self._state_label(st) == "confirmed"
        )
        return f"SignMap conf={n_conf} free={len(self._free)}"

    def to_dict(self) -> dict[str, Any]:
        seats = {}
        for (si, sid), st in self._seat_states.items():
            col, sc = st.best()
            seats[f"{si}:{sid}"] = {
                "red": st.red,
                "green": st.green,
                "empty": st.empty,
                "color": col,
                "score": sc,
                "state": self._state_label(st),
            }
        return {"seats": seats, "free": self._free}
