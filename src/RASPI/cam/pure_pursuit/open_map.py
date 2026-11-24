"""
Mapeo de paredes interiores — Open Challenge (WRO Future Engineers 2026).

Regla FE Open: cada pasillo entre pared interior y exterior es **solo 600 mm o
1000 mm** (±100 mm en la Final Internacional). No hay anchos continuos; las
medidas ToF se **ajustan (snap)** al par legal más cercano.

Geometría
---------
- Paredes exteriores fijas en ±OUTER_HALF_MM (1500 mm desde el centro del tapete).
- Isla interior centrada; **cada una de las cuatro caras** puede ser 600 o 1000 mm
  de forma **independiente** (no se asume que N=S ni E=W).
- En una recta con rumbo estable y |error de rumbo| pequeño:
    corridor ≈ d_inner + d_outer + SENSOR_LAT_SPAN_MM
  (distancia entre montajes laterales de los ToF), o bien
    inner_half_along_lat = OUTER_HALF − corridor
  para esa sección.
- Tras cada esquina completada se congela la sección recién recorrida (pasillo
  con snap a 600/1000).
- Con **3 esquinas** hay 3 pasillos medidos ⇒ ``ready``; la cuarta cara queda sin
  medir hasta la 4.ª esquina (no se copia el ancho del lado opuesto).
- Tras la 4.ª esquina se refinan las cuatro caras con las muestras de la vuelta.

Coordenadas: frame campo como ``track_map.FieldPose`` (origen centro, +x este,
+y norte). ``inner_box_mm`` usa convención ``analiza_vuelta.INTERIOR``
(x0,y0,x1,y1) con origen esquina exterior suroeste del tapete (0…3000 mm).

Integración futura: lap 1 alimenta ``OpenWallMap`` desde ACK/odom; laps 2–3
consumen ``inner_box_mm`` / ``inner_half_mm`` en el planificador (stubs en PP).
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Any, Literal

from . import config as C
from .track_map import (
    DRIVE_ORDER_L,
    DRIVE_ORDER_R,
    FieldPose,
    SensorMount,
    TrackGeometry,
    _ray_seg,
)
from .wro_field import MAT_MM, OUTER_HALF_MM

Direction = Literal["L", "R"]
SectionId = Literal["S", "E", "N", "W"]

MAT_CENTER_MM = MAT_MM / 2.0
DEFAULT_OUTER_HALF_MM = OUTER_HALF_MM

# analiza_vuelta SENSORES (fallback si no hay SENSOR_MOUNTS)
_FALLBACK_MOUNTS = {
    "tof_left": SensorMount(-60.0, 100.0, 270.0),
    "tof_right": SensorMount(60.0, 100.0, 90.0),
    "tof_front": SensorMount(0.0, 180.0, 0.0),
}

TOF_MIN_MM = 20.0
TOF_MAX_MM = 3500.0
HEADING_ALIGN_DEG = 15.0
CORRIDOR_EMA_ALPHA = 0.12
INNER_COORD_EMA_ALPHA = 0.12
SENSOR_LAT_SPAN_MM = abs(
    C.SENSOR_MOUNTS["tof_right"][0] - C.SENSOR_MOUNTS["tof_left"][0]
) if hasattr(C, "SENSOR_MOUNTS") else 120.0

# WRO 2026 Open Challenge: únicos anchos de pasillo permitidos (mm).
OPEN_CORRIDOR_MM = (600.0, 1000.0)
OPEN_CORRIDOR_IF_TOL_MM = 100.0
_IF_CORRIDOR_CANDIDATES_MM = (500.0, 600.0, 700.0, 900.0, 1000.0, 1100.0)
_OPEN_SNAP_MID_MM = 800.0


# Paridad con ``RondaAbierta.ino`` (MAP_ANCHO_*, MAP_ANGOSTO_*) tras esquina 4.
ESP_MAP_ANCHO_PWM = 255
ESP_MAP_ANGOSTO_PWM = 255
ESP_MAP_ANCHO_SLOW_CM = 100
ESP_MAP_ANGOSTO_SLOW_CM = 90


def esp_corridor_cm_from_mm(corridor_mm: float) -> int:
    """60 o 100 cm, mismo umbral que ``mapCongelarSeccion`` (80 cm)."""
    return 60 if snap_corridor_mm(corridor_mm) == OPEN_CORRIDOR_MM[0] else 100


def esp_map_recta_pwm(
    map_pasillo_cm: tuple[int, int, int, int] | list[int],
    turns_completed: int,
    *,
    turns_per_race: int = 12,
) -> tuple[int, int]:
    """
  PWM recto y umbral frontal (cm) para ``turnsCompleted`` en RondaAbierta.

  Vuelta 1 (turns < 4) o MAP incompleto → (255, 90). Vueltas 2–3 usan
  ``map_pasillo_cm[turns_completed % 4]``.
    """
    if turns_completed < 4 or turns_completed >= turns_per_race:
        return ESP_MAP_ANGOSTO_PWM, ESP_MAP_ANGOSTO_SLOW_CM
    if any(p <= 0 for p in map_pasillo_cm):
        return ESP_MAP_ANGOSTO_PWM, ESP_MAP_ANGOSTO_SLOW_CM
    sec = turns_completed % 4
    if map_pasillo_cm[sec] == 100:
        return ESP_MAP_ANCHO_PWM, ESP_MAP_ANCHO_SLOW_CM
    return ESP_MAP_ANGOSTO_PWM, ESP_MAP_ANGOSTO_SLOW_CM


def snap_corridor_mm(raw: float) -> float:
    """
    Ajusta una medida cruda al pasillo legal 600 o 1000 mm.

    Primero acerca a un candidato IF (500…1100); si está dentro de
    ``OPEN_CORRIDOR_IF_TOL_MM``, pliega por umbral 800 mm. Si no, umbral directo.
    """
    best = min(_IF_CORRIDOR_CANDIDATES_MM, key=lambda c: abs(raw - c))
    if abs(raw - best) <= OPEN_CORRIDOR_IF_TOL_MM:
        raw = best
    return OPEN_CORRIDOR_MM[0] if raw < _OPEN_SNAP_MID_MM else OPEN_CORRIDOR_MM[1]


def open_layout_from_seed(seed: int) -> dict[SectionId, float]:
    """Cuatro pasillos 600|1000 independientes (espíritu lanzamiento de moneda)."""
    import random

    rng = random.Random(seed)
    order: tuple[SectionId, ...] = ("S", "E", "N", "W")
    return {
        s: OPEN_CORRIDOR_MM[0] if rng.getrandbits(1) else OPEN_CORRIDOR_MM[1] for s in order
    }


def _mounts() -> dict[str, SensorMount]:
    if hasattr(C, "SENSOR_MOUNTS"):
        return {
            "tof_left": SensorMount(*C.SENSOR_MOUNTS["tof_left"]),
            "tof_right": SensorMount(*C.SENSOR_MOUNTS["tof_right"]),
            "tof_front": SensorMount(*C.SENSOR_MOUNTS["tof_front"]),
        }
    return dict(_FALLBACK_MOUNTS)


def _norm_h(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _section_heading(geom: TrackGeometry, section: SectionId) -> float:
    idx = geom._order.index(section)
    _, _, ah = geom.section_frame(idx)
    return ah


def _inner_outer_keys(heading_deg: float, geom: TrackGeometry, section: SectionId) -> tuple[str, str]:
    """ToF interior (hacia la isla) y exterior en esta recta."""
    idx = geom._order.index(section)
    _, u_lat, _ = geom._lat_from_section(idx)
    h = math.radians(heading_deg)
    fx, fy = math.cos(h), math.sin(h)
    lx, ly = -math.sin(h), math.cos(h)
    if lx * u_lat[0] + ly * u_lat[1] > 0:
        return "tof_left", "tof_right"
    return "tof_right", "tof_left"


def _ray_world(
    pose: FieldPose, mount: SensorMount, off_deg: float = 0.0
) -> tuple[float, float, float, float]:
    h = math.radians(pose.heading_deg)
    fx, fy = math.cos(h), math.sin(h)
    rx, ry = math.sin(h), -math.cos(h)
    ox = pose.x_mm + mount.fwd_mm * fx + mount.right_mm * rx
    oy = pose.y_mm + mount.fwd_mm * fy + mount.right_mm * ry
    a = math.radians(mount.dir_deg + off_deg)
    dx = math.cos(a) * fx + math.sin(a) * rx
    dy = math.cos(a) * fy + math.sin(a) * ry
    return ox, oy, dx, dy


def _ema(prev: float | None, new: float, alpha: float) -> float:
    if prev is None:
        return new
    return (1.0 - alpha) * prev + alpha * new


def _wall_coord_from_inner_tof(
    pose: FieldPose,
    section: SectionId,
    d_inner_mm: float,
    mount: SensorMount,
) -> float:
    """Posición de la pared interior en el eje correspondiente (x o y mundo)."""
    ox, oy, dx, dy = _ray_world(pose, mount)
    # intersección con vertical u horizontal según sección
    if section in ("S", "N"):
        # pared horizontal y = const
        if abs(dy) < 1e-6:
            return pose.y_mm
        t = d_inner_mm  # rayo central, distancia euclídea ≈ avance en y si lateral
        # proyección exacta: distancia medida es a lo largo del rayo
        t_along = d_inner_mm
        wy = oy + dy * (t_along / max(math.hypot(dx, dy), 1e-9))
        return wy
    ox2, oy2, dx2, dy2 = ox, oy, dx, dy
    if abs(dx2) < 1e-6:
        return pose.x_mm
    wx = ox2 + dx2 * (d_inner_mm / max(math.hypot(dx2, dy2), 1e-9))
    return wx


@dataclass
class _SectionState:
    section: SectionId
    d_inner: list[float] = field(default_factory=list)
    d_outer: list[float] = field(default_factory=list)
    corridor_mm: float | None = None
    inner_coord_mm: float | None = None  # y para S/N, x para E/W (pared interior)
    along_mm: float = 0.0
    frozen: bool = False
    inferred: bool = False


@dataclass
class OpenWallMap:
    """Estima el rectángulo interior con ToF + odometría en la vuelta 1."""

    drive_dir: Direction = "L"
    outer_half_mm: float = DEFAULT_OUTER_HALF_MM
    start_section: SectionId = "S"

    _geom: TrackGeometry = field(init=False)
    _mounts: dict[str, SensorMount] = field(default_factory=_mounts)
    _sections: dict[SectionId, _SectionState] = field(init=False)
    _order: tuple[SectionId, ...] = field(init=False)
    _turn_index: int = 0
    _current: SectionId = field(init=False)
    _odom_accum_mm: float = 0.0
    _last_odom_mm: float | None = None
    _inferred_faces: set[SectionId] = field(default_factory=set)

    def __post_init__(self) -> None:
        self.drive_dir = self.drive_dir.upper()  # type: ignore[assignment]
        if self.drive_dir not in ("L", "R"):
            raise ValueError("drive_dir must be L or R")
        self._geom = TrackGeometry(drive_dir=self.drive_dir)
        self._order = tuple(self._geom._order)  # type: ignore[attr-defined]
        self._sections = {s: _SectionState(s) for s in self._order}
        self.reset(self.drive_dir, self.start_section)

    def reset(self, direction: Direction = "L", start_section: SectionId = "S") -> None:
        self.drive_dir = direction.upper()  # type: ignore[assignment]
        self.start_section = start_section
        self._geom = TrackGeometry(drive_dir=self.drive_dir)
        self._order = tuple(self._geom._order)  # type: ignore[attr-defined]
        self._sections = {s: _SectionState(s) for s in self._order}
        if start_section not in self._order:
            raise ValueError(f"start_section {start_section} not in drive order")
        self._current = start_section
        self._turn_index = 0
        self._odom_accum_mm = 0.0
        self._last_odom_mm = None
        self._inferred_faces.clear()

    @property
    def current_section(self) -> SectionId:
        return self._current

    @property
    def frozen(self) -> bool:
        return all(st.frozen for st in self._sections.values())

    @property
    def ready(self) -> bool:
        """True con 3 esquinas: tres pasillos medidos y ajustados (sin copiar el 4.º)."""
        n_frozen = sum(1 for st in self._sections.values() if st.frozen and not st.inferred)
        return n_frozen >= 3

    def _snap_section(self, st: _SectionState) -> None:
        if st.corridor_mm is None:
            return
        st.corridor_mm = snap_corridor_mm(st.corridor_mm)
        st.inner_coord_mm = self._coord_from_corridor(st.section, st.corridor_mm)

    def _expected_heading(self) -> float:
        return _section_heading(self._geom, self._current)

    def update_straight(
        self,
        t_ms: float,
        pose_xy_yaw: tuple[float, float, float],
        tof_L_mm: float | None,
        tof_R_mm: float | None,
        tof_F_mm: float | None,
        odom_mm: float,
    ) -> None:
        del t_ms, tof_F_mm  # reservados para filtros / giro
        x, y, yaw = pose_xy_yaw
        pose = FieldPose(x, y, yaw)
        st = self._sections[self._current]
        if st.frozen:
            return

        h_exp = self._expected_heading()
        if abs(_norm_h(yaw - h_exp)) > HEADING_ALIGN_DEG:
            return

        if self._last_odom_mm is not None:
            st.along_mm += max(0.0, odom_mm - self._last_odom_mm)
        elif odom_mm > 0:
            st.along_mm += float(odom_mm)
        self._last_odom_mm = odom_mm

        inner_key, outer_key = _inner_outer_keys(yaw, self._geom, self._current)
        tof_map = {"tof_left": tof_L_mm, "tof_right": tof_R_mm}
        d_in = tof_map.get(inner_key)
        d_out = tof_map.get(outer_key)

        if d_in is not None and TOF_MIN_MM <= d_in <= TOF_MAX_MM:
            din = float(d_in)
            if st.d_inner:
                med = statistics.median(st.d_inner)
                if abs(din - med) > 80.0:
                    din = None  # esquina u otra pared en el cono
            if din is not None:
                st.d_inner.append(din)
                mount_in = self._mounts[inner_key]
                wc = _wall_coord_from_inner_tof(pose, self._current, din, mount_in)
                st.inner_coord_mm = _ema(st.inner_coord_mm, wc, INNER_COORD_EMA_ALPHA)

        if d_out is not None and TOF_MIN_MM <= d_out <= TOF_MAX_MM:
            dout = float(d_out)
            if st.d_outer:
                med = statistics.median(st.d_outer)
                if abs(dout - med) > 80.0:
                    dout = None
            if dout is not None:
                st.d_outer.append(dout)

        if st.d_inner and st.d_outer:
            corridor = statistics.median(st.d_inner) + statistics.median(st.d_outer) + SENSOR_LAT_SPAN_MM
            st.corridor_mm = _ema(st.corridor_mm, corridor, CORRIDOR_EMA_ALPHA)
        elif d_in is not None and st.corridor_mm is None and not st.d_outer:
            wc = st.inner_coord_mm
            if wc is not None:
                cor = self.outer_half_mm - abs(wc)
                st.corridor_mm = _ema(st.corridor_mm, cor, CORRIDOR_EMA_ALPHA)

    def on_corner_complete(self, turn_index: int) -> None:
        """Finaliza la recta actual y avanza a la siguiente sección."""
        self._turn_index = turn_index
        st = self._sections[self._current]
        if not st.frozen:
            if st.corridor_mm is None and st.d_inner and st.d_outer:
                st.corridor_mm = statistics.median(st.d_inner) + statistics.median(st.d_outer) + SENSOR_LAT_SPAN_MM
            elif st.corridor_mm is None and st.d_inner:
                if st.inner_coord_mm is not None:
                    st.corridor_mm = self.outer_half_mm - abs(st.inner_coord_mm)
                else:
                    st.corridor_mm = 2 * statistics.median(st.d_inner) + SENSOR_LAT_SPAN_MM
            if st.inner_coord_mm is None and st.corridor_mm is not None:
                st.inner_coord_mm = self._coord_from_corridor(self._current, st.corridor_mm)
            self._snap_section(st)
            st.frozen = True

        # avanzar sección según orden de marcha
        idx = self._order.index(self._current)
        if idx + 1 < len(self._order):
            self._current = self._order[idx + 1]
        self._last_odom_mm = None
        if turn_index >= 4:
            self._refine_inferred()

    def _coord_from_corridor(self, section: SectionId, corridor_mm: float) -> float:
        oh = self.outer_half_mm
        if section == "S":
            return -oh + corridor_mm
        if section == "N":
            return oh - corridor_mm
        if section == "E":
            return oh - corridor_mm
        return -oh + corridor_mm

    def _refine_inferred(self) -> None:
        for st in self._sections.values():
            if st.d_inner and st.d_outer:
                c = statistics.median(st.d_inner) + statistics.median(st.d_outer) + SENSOR_LAT_SPAN_MM
                st.corridor_mm = c
                self._snap_section(st)

    def inner_half_mm(self) -> tuple[float, float]:
        """Semiejes interiores (half_x, half_y) en frame centro."""
        xs: list[float] = []
        ys: list[float] = []
        for sec, st in self._sections.items():
            if st.inner_coord_mm is None:
                continue
            if sec in ("E", "W"):
                xs.append(abs(st.inner_coord_mm))
            else:
                ys.append(abs(st.inner_coord_mm))
        hx = statistics.median(xs) if xs else self.outer_half_mm - 500.0
        hy = statistics.median(ys) if ys else self.outer_half_mm - 500.0
        return hx, hy

    def inner_box_mm(self) -> tuple[float, float, float, float]:
        """(x0,y0,x1,y1) estilo analiza_vuelta en tapete 0…3000 (solo caras medidas)."""
        vals: dict[str, float] = {}
        for sec, st in self._sections.items():
            if st.frozen and st.inner_coord_mm is not None and not st.inferred:
                vals[sec] = st.inner_coord_mm
        if len(vals) < 2:
            hx, hy = self.inner_half_mm()
            cx0, cy0 = -hx, -hy
            cx1, cy1 = hx, hy
            return (
                MAT_CENTER_MM + cx0,
                MAT_CENTER_MM + cy0,
                MAT_CENTER_MM + cx1,
                MAT_CENTER_MM + cy1,
            )
        if "W" in vals and "E" in vals:
            x_w, x_e = vals["W"], vals["E"]
        elif "W" in vals:
            x_w, x_e = vals["W"], vals["W"] + 1.0
        elif "E" in vals:
            x_w, x_e = vals["E"] - 1.0, vals["E"]
        else:
            x_w, x_e = -500.0, 500.0
        if "S" in vals and "N" in vals:
            y_s, y_n = vals["S"], vals["N"]
        elif "S" in vals:
            y_s, y_n = vals["S"], vals["S"] + 1.0
        elif "N" in vals:
            y_s, y_n = vals["N"] - 1.0, vals["N"]
        else:
            y_s, y_n = -500.0, 500.0
        x0 = MAT_CENTER_MM + min(x_w, x_e)
        x1 = MAT_CENTER_MM + max(x_w, x_e)
        y0 = MAT_CENTER_MM + min(y_s, y_n)
        y1 = MAT_CENTER_MM + max(y_s, y_n)
        return x0, y0, x1, y1

    def geometry(self) -> dict[str, Any]:
        hx, hy = self.inner_half_mm()
        x0, y0, x1, y1 = self.inner_box_mm()
        narrow_mask: dict[SectionId, bool] = {}
        for s, st in self._sections.items():
            if st.corridor_mm is not None:
                snapped = snap_corridor_mm(st.corridor_mm)
                narrow_mask[s] = snapped == OPEN_CORRIDOR_MM[0]
        return {
            "drive_dir": self.drive_dir,
            "outer_half_mm": self.outer_half_mm,
            "inner_half_mm": (hx, hy),
            "inner_box_mm": (x0, y0, x1, y1),
            "ready": self.ready,
            "frozen": self.frozen,
            "turn_index": self._turn_index,
            "narrow_mask": narrow_mask,
            "sections": {
                s: {
                    "corridor_mm": st.corridor_mm,
                    "kind": (
                        "600"
                        if st.corridor_mm is not None and snap_corridor_mm(st.corridor_mm) == OPEN_CORRIDOR_MM[0]
                        else "1000"
                        if st.corridor_mm is not None
                        else None
                    ),
                    "inner_coord_mm": st.inner_coord_mm,
                    "along_mm": st.along_mm,
                    "frozen": st.frozen,
                    "inferred": st.inferred,
                }
                for s, st in self._sections.items()
            },
        }


def _inner_outer_segments(
    inner_box: tuple[float, float, float, float] | None,
    outer_half: float,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Segmentos para raycast: outer e inner rects en coords centro."""
    oh = outer_half
    outer = [
        (oh, -oh, -oh, oh),
        (-oh, oh, oh, oh),
        (-oh, oh, -oh, -oh),
        (-oh, -oh, oh, -oh),
    ]
    segs_a: list[tuple[float, float]] = []
    segs_b: list[tuple[float, float]] = []
    for x0, y0, x1, y1 in outer:
        segs_a.append((x0, y0))
        segs_b.append((x1, y1))
    if inner_box is not None:
        x0, y0, x1, y1 = inner_box
        cx0, cy0 = x0 - MAT_CENTER_MM, y0 - MAT_CENTER_MM
        cx1, cy1 = x1 - MAT_CENTER_MM, y1 - MAT_CENTER_MM
        inner = [
            (cx0, cy0, cx1, cy0),
            (cx1, cy0, cx1, cy1),
            (cx1, cy1, cx0, cy1),
            (cx0, cy1, cx0, cy0),
        ]
        for xa, ya, xb, yb in inner:
            segs_a.append((xa, ya))
            segs_b.append((xb, yb))
    return segs_a, segs_b


def expected_tof_mm(
    pose_xy_yaw: tuple[float, float, float],
    sensor: Literal["L", "R", "F"],
    inner_box_mat: tuple[float, float, float, float] | None,
    outer_half_mm: float = DEFAULT_OUTER_HALF_MM,
) -> float | None:
    """
    Distancia esperada del rayo central del ToF contra outer+inner.
    ``inner_box_mat`` en convención analiza (0…3000); None solo pared exterior.
    """
    mounts = _mounts()
    key = {"L": "tof_left", "R": "tof_right", "F": "tof_front"}[sensor]
    pose = FieldPose(pose_xy_yaw[0], pose_xy_yaw[1], pose_xy_yaw[2])
    ox, oy, dx, dy = _ray_world(pose, mounts[key])
    segs_a, segs_b = _inner_outer_segments(inner_box_mat, outer_half_mm)
    best = 4000.0
    for i, (ax, ay) in enumerate(segs_a):
        bx, by = segs_b[i]
        t = _ray_seg(ox, oy, dx, dy, ax, ay, bx, by)
        if t is not None and t < best:
            best = t
    return best if best < 3999.0 else None


def field_inner_box_from_corridors(
    corridors: dict[SectionId, float],
    outer_half: float = DEFAULT_OUTER_HALF_MM,
) -> tuple[float, float, float, float]:
    """Utilidad: pasillos por sección → INTERIOR mat."""
    coords: dict[SectionId, float] = {}
    for sec, cor in corridors.items():
        if sec == "S":
            coords[sec] = -outer_half + cor
        elif sec == "N":
            coords[sec] = outer_half - cor
        elif sec == "E":
            coords[sec] = outer_half - cor
        else:
            coords[sec] = -outer_half + cor
    xw, xe = coords.get("W", -coords.get("E", -500)), coords.get("E", -coords.get("W", 500))
    ys, yn = coords.get("S", -coords.get("N", -500)), coords.get("N", -coords.get("S", 500))
    if "E" in coords and "W" not in coords:
        xw = -coords["E"]
    if "W" in coords and "E" not in coords:
        xe = -coords["W"]
    if "S" in coords and "N" not in coords:
        yn = -coords["S"]
    if "N" in coords and "S" not in coords:
        ys = -coords["N"]
    return (
        MAT_CENTER_MM + min(xw, xe),
        MAT_CENTER_MM + min(ys, yn),
        MAT_CENTER_MM + max(xw, xe),
        MAT_CENTER_MM + max(ys, yn),
    )