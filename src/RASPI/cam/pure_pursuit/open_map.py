"""
Mapeo de paredes interiores — Open Challenge (WRO Future Engineers 2026).

Regla FE Open: cada pasillo entre pared interior y exterior es **solo 600 mm o
1000 mm** (±100 mm en la Final Internacional). No hay anchos continuos; las
medidas ToF se **ajustan (snap)** al par legal más cercano.

Geometría
---------
- Paredes exteriores fijas en ±OUTER_HALF_MM (1500 mm desde el centro del tapete).
- Modelo de isla rectangular centrada. Cada eje elige un nominal de forma
  independiente (S=N y E=W). La tolerancia IF se trata como incertidumbre de
  medición. La igualdad de caras opuestas es una hipótesis del modelo de pista.
- En una recta con rumbo estable y |error de rumbo| pequeño:
  se proyectan los rayos y sus montajes sobre el eje de las caras.
  La distancia entre impactos proyectados da el ancho del pasillo:
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
SENSOR_LAT_SPAN_MM = abs(
    C.SENSOR_MOUNTS["tof_right"][0] - C.SENSOR_MOUNTS["tof_left"][0]
) if hasattr(C, "SENSOR_MOUNTS") else 120.0

# WRO 2026 Open Challenge: únicos anchos de pasillo permitidos (mm).
OPEN_CORRIDOR_MM = (600.0, 1000.0)
OPEN_CORRIDOR_IF_TOL_MM = 100.0
_OPEN_SNAP_MID_MM = 800.0


# Paridad con ``RondaAbierta.ino`` (MAP_ANCHO_*, MAP_ANGOSTO_*) tras esquina 4.
ESP_MAP_ANCHO_PWM = 255
ESP_MAP_ANGOSTO_PWM = 255
ESP_MAP_ANCHO_SLOW_CM = 70
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
    """Proyecta al nominal más cercano. El empate a 800 mm va a 1000 mm."""
    return OPEN_CORRIDOR_MM[0] if raw < _OPEN_SNAP_MID_MM else OPEN_CORRIDOR_MM[1]


def esp_consistent_corridors_cm(raw_mm: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
    """Reconcilia opuestos en orden de giro, como RondaAbierta.

    Gana la lectura más alejada de 800 mm. Ante empate entre dos clases
    se elige el ancho menor para mantener el perfil prudente.
    """
    def choose(a: float, b: float) -> int:
        stronger = max((a, b), key=lambda x: (abs(x - _OPEN_SNAP_MID_MM), -x))
        return esp_corridor_cm_from_mm(stronger)

    a, b = choose(raw_mm[0], raw_mm[2]), choose(raw_mm[1], raw_mm[3])
    return a, b, a, b


def open_layout_from_seed(seed: int) -> dict[SectionId, float]:
    """Elige un ancho por eje de la isla centrada."""
    import random

    rng = random.Random(seed)
    south, east = (OPEN_CORRIDOR_MM[rng.getrandbits(1)] for _ in range(2))
    return {"S": south, "E": east, "N": south, "W": east}


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


def _wall_hit_axis(pose: FieldPose, section: SectionId, distance: float, mount: SensorMount) -> float:
    ox, oy, dx, dy = _ray_world(pose, mount)
    return oy + dy * distance if section in ("S", "N") else ox + dx * distance


@dataclass
class _SectionState:
    section: SectionId
    paired_widths: list[float] = field(default_factory=list)
    single_widths: list[float] = field(default_factory=list)
    raw_corridor_mm: float | None = None
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
        return all(st.frozen and st.corridor_mm is not None for st in self._sections.values())

    @property
    def ready(self) -> bool:
        """True con 3 esquinas: tres pasillos medidos y ajustados (sin copiar el 4.º)."""
        n_frozen = sum(1 for st in self._sections.values() if st.frozen and st.corridor_mm is not None and not st.inferred)
        return n_frozen >= 3

    def _snap_section(self, st: _SectionState) -> None:
        if st.corridor_mm is None:
            return
        st.raw_corridor_mm = st.corridor_mm
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
        self._last_odom_mm = odom_mm

        inner_key, outer_key = _inner_outer_keys(yaw, self._geom, self._current)
        tof_map = {"tof_left": tof_L_mm, "tof_right": tof_R_mm}
        d_in = tof_map.get(inner_key)
        d_out = tof_map.get(outer_key)

        if d_in is None or not TOF_MIN_MM <= d_in <= TOF_MAX_MM:
            return
        inner = _wall_hit_axis(pose, self._current, d_in, self._mounts[inner_key])
        sign = 1 if self._current in ("N", "E") else -1
        if d_out is not None and TOF_MIN_MM <= d_out <= TOF_MAX_MM:
            outer = _wall_hit_axis(pose, self._current, d_out, self._mounts[outer_key])
            width, samples = sign * (outer - inner), st.paired_widths
        else:
            width, samples = self.outer_half_mm - sign * inner, st.single_widths
        if 0 < width < self.outer_half_mm:
            samples.append(width)

    def on_corner_complete(self, turn_index: int) -> None:
        """Finaliza la recta actual y avanza a la siguiente sección."""
        if self._turn_index == 4 or turn_index != self._turn_index + 1:
            return
        self._turn_index = turn_index
        st = self._sections[self._current]
        samples = st.paired_widths or st.single_widths
        if samples:
            st.corridor_mm = statistics.median(samples)
            self._snap_section(st)
        st.frozen = True

        self._last_odom_mm = None
        if turn_index < 4:
            idx = self._order.index(self._current)
            self._current = self._order[(idx + 1) % len(self._order)]
        elif self.frozen:
            sections = [self._sections[section] for section in ("S", "E", "N", "W")]
            raw = tuple(st.raw_corridor_mm for st in sections)
            for face, cm in zip(sections, esp_consistent_corridors_cm(raw)):
                face.corridor_mm = cm * 10.0
                face.inner_coord_mm = self._coord_from_corridor(face.section, face.corridor_mm)

    def _coord_from_corridor(self, section: SectionId, corridor_mm: float) -> float:
        oh = self.outer_half_mm
        if section == "S":
            return -oh + corridor_mm
        if section == "N":
            return oh - corridor_mm
        if section == "E":
            return oh - corridor_mm
        return -oh + corridor_mm

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
        hx = statistics.median(xs) if xs else self.outer_half_mm - 1000.0
        hy = statistics.median(ys) if ys else self.outer_half_mm - 1000.0
        return hx, hy

    def inner_box_mm(self) -> tuple[float, float, float, float]:
        """(x0,y0,x1,y1) estilo analiza_vuelta para la isla centrada."""
        hx, hy = self.inner_half_mm()
        return MAT_CENTER_MM - hx, MAT_CENTER_MM - hy, MAT_CENTER_MM + hx, MAT_CENTER_MM + hy

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
                    "raw_corridor_mm": st.raw_corridor_mm,
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
        (oh, -oh, oh, oh),
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