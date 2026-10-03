"""
Runtime Pure Pursuit + MEMORIA DE OBSTÁCULOS — WRO Future Engineers.

Igual que runtime.py, pero con un mapa rodante disperso (obstacle_memory.py):
el robot recuerda las latas vistas y las arrastra hacia sí cuadro a cuadro usando
avance asumido (velocidad) + giro del IMU (anguloGyro que el ESP32 ahora regresa
en el ACK:V2).  Así la inflación de la lata no desaparece cuando ésta sale del
campo de visión, y el carro deja de cortarse sobre ella.

Diferencias vs runtime.py:
  • self.memory = ObstacleMemory()
  • parsea ang=<heading> del ACK:V2 del ESP32
  • antes de detect_centerline, fusiona detecciones nuevas con la memoria
  • el heading usado va con 1 frame de retraso (el ACK llega tras enviar) — irrelevante

Para correrlo:
  python -m pure_pursuit.runtime_nuevo
"""

import argparse
import functools
import math
import os
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# ── Importar infraestructura compartida de cam/ ──────────────────────────────
_CAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import Vision, open_camera
from wro_runtime import (
    ThreadedFrameGrabber,
    AsyncVideoWriter,
    SerialLink,
    resolve_output_path,
    CAM_FRAME_PATH,
)

from .bev import BEVTransformer
from .centerline import (
    bend_line_above, detect_centerline, draw_bev_debug,
    map_obstacle_to_bev, seen_above_obstacles,
)
from .digital_map import DigitalMap
from .corner_lines import OrangeLineTracker, TurnDirectionTracker, is_interior_pass
from .controller import PurePursuitController
from .obstacle_memory import ObstacleMemory
from .far_hint import FarHintManager
from .mid_turn import MidTurnObstacleDetector
from .bev_recorder import BevRecorder
from .color_corr import FloorColorCorrector
from . import config as C
from .odometry import Odometry, parse_ack
from .track_map import (
    DEFAULT_MOUNTS,
    FieldPose,
    Localizer,
    SensorReading,
    SignMap,
    TrackFrame,
    TrackGeometry,
    US_HALF_ANGLE_DEG,
    field_heading_to_twin,
)
from .map_view import render_map


# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PPConfig:
    # Cámara / captura
    cam_index:        int   = C.CAM_INDEX
    serial_port:      str   = C.SERIAL_PORT
    baudrate:         int   = C.BAUDRATE
    process_every_n:  int   = C.PROCESS_EVERY
    warmup_frames:    int   = C.WARMUP_FRAMES
    threaded_capture: bool  = True
    show_window:      bool  = True

    # Grabación
    record_orillas:  bool        = False
    record_output:   str | None  = None
    record_every_n:  int         = 6
    record_fps:      float       = 5.0

    # Vista en vivo (VNC) — CAM_FRAME_PATH, ver _write_cam_frame()
    cam_frame_every_n: int = 2   # cada cuántos frames procesados se actualiza
                                  # la captura para la vista remota; no afecta
                                  # el video grabado (record_every_n, aparte)

    # BEV
    calib_path: Path | None = None


@dataclass
class FrameResult:
    processed_frame: np.ndarray
    bev_frame: np.ndarray | None
    positions: dict
    steer_deg: float
    obs_norm: float
    pp_active: bool
    path_points: list
    lookahead_pt: tuple
    bev_obstacles: list
    bev_obstacles_beyond: list
    line_info: dict
    serial_msg: str
    serial_ack: str | None
    pasado: bool
    measured_pass: bool
    state: str
    fps: float
    timing_ms: dict
    bev_timing: dict
    new_obstacles: list
    new_obs_h: list
    map_pose: FieldPose | None = None
    map_status: dict | None = None
    map_geometry: TrackGeometry | None = None
    map_signmap: SignMap | None = None
    map_localizer: Localizer | None = None
    # Desplazamiento para dibujar el BEV ancho. El cálculo sigue en 400 px.
    bev_shift: tuple = (0, 0)
    # Mapa que el carro cree: medidas sí, latas solo las que ya vio.
    dig_map: np.ndarray | None = None

def _shift_line_info(info: dict, dx: int, dy: int) -> dict:
    """La naranja se calculó en la hoja de 400 px. El dibujo va en la ancha."""
    out = {}
    for color, item in info.items():
        item = dict(item)
        if item.get("near_y") is not None:
            item["near_y"] = item["near_y"] + dy
        line = item.get("line")
        if line is not None:
            vx, vy, x0, y0 = line
            item["line"] = (vx, vy, x0 + dx, y0 + dy)
        curve = item.get("_cv")
        if curve is not None:
            curve = dict(curve)
            mx, my = curve["mu"]
            curve["mu"] = (mx + dx, my + dy)
            item["_cv"] = curve
        out[color] = item
    return out

# ─────────────────────────────────────────────────────────────────────────────

def _parse_heading(ack: str) -> float | None:
    """Extrae ang=<float> de 'ACK:V2,ang=12.34'.  None si no está presente."""
    if not ack:
        return None
    idx = ack.find("ang=")
    if idx < 0:
        return None
    try:
        return float(ack[idx + 4:].split(",")[0])
    except (ValueError, IndexError):
        return None

def _parse_estado(ack: str) -> str | None:
    if not ack:
        return None
    idx = ack.find("est=")
    if idx < 0:
        return None
    val = ack[idx + 4: idx + 5]
    if val in ("C", "I", "E", "T"):   # CRUCERO / INICIO / ESTACIONANDO / TERMINANDO (ESP): la Pi los trata igual que SIGUIENDO
        return "S"          # (durante INICIO/ESTACIONANDO el ESP maneja solo; la Pi no hace nada especial)
    return val if val in ("G", "R", "S") else None


def _parse_inicio(ack: str) -> bool | None:
    """True si el ACK trae est=I (ESP en INICIO), False con cualquier otro est=,
    None si no trae est=. _parse_estado() colapsa I -> S; esto conserva el dato
    crudo solo para apagar el dead-reckon de la naranja (ver line_tracker.update)."""
    if not ack:
        return None
    idx = ack.find("est=")
    if idx < 0:
        return None
    return ack[idx + 4: idx + 5] == "I"


def _park_pink(frame_bgr):
    """(ratio, mask). ratio = fracción de píxeles magenta (pared del cajón) en el
    ROI, ignorando la banda superior C.PARK_PINK_ROI_TOP (fondo del venue). mask
    es del tamaño del frame (0 por encima del ROI), para dibujar el HUD. Ver
    `case INICIO` en PurePursuit.ino y el bloque INICIO de config.py."""
    if frame_bgr is None or getattr(frame_bgr, "size", 0) == 0:
        return 0.0, None
    hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    y0   = int(h * getattr(C, "PARK_PINK_ROI_TOP", 0.12))
    m = None
    for lo, hi in C.PARK_PINK_HSV:
        cur = cv2.inRange(hsv[y0:, :], lo, hi)
        m   = cur if m is None else (m | cur)
    if m is None:
        return 0.0, None
    ratio = float(np.count_nonzero(m)) / float(m.size)
    full  = np.zeros((h, w), np.uint8)
    full[y0:, :] = m
    return ratio, full

def _parse_direccion(ack: str) -> str | None:
    """dir= del ACK:V2 del ESP32: 'L'/'R' desde su 1er GIRANDO, '?' antes."""
    if not ack:
        return None
    idx = ack.find("dir=")
    if idx < 0:
        return None
    val = ack[idx + 4: idx + 5]
    return val if val in ("L", "R") else None

def _parse_tc(ack: str) -> int | None:
    """tc= del ACK:V2 del ESP32: giros completados (0..12)."""
    if not ack:
        return None
    idx = ack.find("tc=")
    if idx < 0:
        return None
    try:
        return int(ack[idx + 3:].split(",")[0])
    except (ValueError, IndexError):
        return None

def _parse_tpr(ack: str) -> int | None:
    """tpr= del ACK:V2 del ESP32: TURNS_PER_RACE del .ino (4 en tests, 12 en carrera)."""
    if not ack:
        return None
    idx = ack.find("tpr=")
    if idx < 0:
        return None
    try:
        return int(ack[idx + 4:].split(",")[0])
    except (ValueError, IndexError):
        return None

def _parse_pb(ack: str) -> bool | None:
    """pb= del ACK:V2 del ESP32: 1 si parkBuscando es True."""
    if not ack:
        return None
    idx = ack.find("pb=")
    if idx < 0:
        return None
    try:
        return ack[idx + 3: idx + 4] == "1"
    except (ValueError, IndexError):
        return None


def _map_safe(fallback):
    """El mapa es opcional: si truena se apaga hasta el próximo arm en vez de tumbar
    el loop del carro a media carrera. `fallback` recibe los mismos argumentos y
    debe regresar lo mismo que regresaría el mapa apagado."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(self, *args, **kwargs):
            if not self._map_failed:
                try:
                    return fn(self, *args, **kwargs)
                except Exception:
                    self._map_failed = True
                    print(f"[MAP] ERROR en {fn.__name__}, mapa apagado hasta el "
                          f"próximo arm:\n{traceback.format_exc()}", flush=True)
            return fallback(*args, **kwargs)
        return wrapper
    return deco


def _gray_map_panel(size: int) -> np.ndarray:
    return np.full((size, size, 3), (230, 230, 230), dtype=np.uint8)


class PPRuntime:
    """
    Runtime Pure Pursuit con memoria de obstáculos.
    Infraestructura idéntica a runtime.py / wro_runtime.py.
    """

    def __init__(self, cfg: PPConfig, *, vision=None, serial_link=None, bev=None):
        self.cfg = cfg

        # Visión
        self.vision     = vision if vision is not None else Vision(cfg.cam_index)
        self.bev        = bev if bev is not None else BEVTransformer(cfg.calib_path)
        self.controller = PurePursuitController()
        self.memory     = ObstacleMemory()
        self.far_hint   = FarHintManager()
        self.line_tracker = OrangeLineTracker()
        self.turn_dir_tracker = TurnDirectionTracker()
        # FASE 1 mid-turn: solo observa/registra (ver mid_turn.py). No actúa.
        self.mid_turn   = MidTurnObstacleDetector()
        # Corrección de color por el piso (otra iluminación), ver color_corr.py
        self.color_corr = FloorColorCorrector()
        self.digital = DigitalMap()
        self._dig_img = None

        # Estado de la memoria rodante
        self._last_heading: float | None = None
        self._lock_xy: tuple[float, float] | None = None  # cono primario fijado (>=2 conos)
        self._last_update_t: float | None = None
        self._prev_estado: str | None = None
        self._esp_inicio: bool = False   # último est= del ACK fue I (ver _parse_inicio)
        self._is_turning: bool = False
        self._turn_start_t: float | None = None
        self._turn_recovery_frames: int = 0
        self._pasado_hold: int = 0   # frames restantes repitiendo pasado=1
        self._pasado_from_measured: bool = False  # el pulso pasado en curso vino
                                                  # de _measured_recup_trigger
                                                  # (esquiva de ángulo REAL) — no
                                                  # se suprime cerca de la esquina
        self._turn_delay_frames: int = 0  # tras un rebase medido cerca de una
                                          # esquina: fuerza prio=1 estos frames
                                          # (post-RECUPERANDO) para que el giro
                                          # no entre ~0.5s antes del punto real
        self._ext_corner_hold: bool = False   # este frame se rescató un cono
                                              # EXTERIOR de la boca de la esquina
                                              # (ver CORNER_EXTERIOR_PASS_ENABLED)
        self._ext_corner_block: int = 0       # frames restantes forzando prio=1
                                              # tras soltar el rescate (el giro no
                                              # se suelta hasta que el cono está
                                              # de verdad atrás, no por arrastre)

        # ── Trigger de RECUPERANDO por ESTADO MEDIDO (ver _measured_recup_trigger) ──
        self._heading_ref: float | None = None   # heading de la recta al ARMARSE la esquiva
        self._dodge_armed: bool = False          # hubo una esquiva de verdad en curso
        self._recup_can_arm: bool = True         # gate: solo re-arma tras un peso bajo (esquiva nueva)
        self._recup_arm_streak: int = 0          # frames seguidos con peso alto (debounce de armado)
        self._recup_clear_count: int = 0         # frames seguidos con el path ya despejado
        self._recup_noghost_streak: int = 0      # frames sin ver color fresco (con esquiva armada + herr grande)
        self._g_streak: int = 0                  # est=G consecutivos (debounce de giro)
        self._last_recup_reason: str = "-"       # para overlay / journalctl

        # ── INICIO — salida del estacionamiento ───────────────────────────────
        # Se mide el rosa mientras está DESARMADO y se decide UNA vez al armar.
        # None = aún no decidido -> inicio=0. La Pi NO hace nada más.
        self._inicio_estacionamiento: bool | None = None
        self._pink_samples: list[float] = []

        # ── ESTACIONAMIENTO — búsqueda del cajón en la recta final ────────────
        self._tc: int = 0
        # TURNS_PER_RACE del ESP32 (tpr= en el ACK). 12 hasta que llegue el 1er ACK.
        self._tpr: int = 12
        self._park_buscando: bool = False
        self._park_state: int = 0         # 0=no busca, 1=viendo cajon, 2=cajon alineado (iniciar reversa)
        self._park_dist_cm: int = 0
        self._park_frames_seen: int = 0
        self._park_lost_frames: int = 0
        self._park_max_y_seen: int = 0

        self._corner_hint_until: float = 0.0   # es= del V2, ver _corner_hint()

        # ── Mapa de pista (shadow / control opcional) ───────────────────────
        self._odom = Odometry()
        self._map_geometry: TrackGeometry | None = None
        self._map_frame: TrackFrame | None = None
        self._map_localizer: Localizer | None = None
        self._map_signmap: SignMap | None = None
        self._map_field_pose: FieldPose | None = None
        self._map_last_odom = None
        self._map_pending: list[tuple[list, object, float]] = []
        self._map_status: dict = {}
        self._map_drive_resolved: str | None = None
        self._map_odom_at_arm = None
        self._map_snap_tries = 0
        self._map_failed = False
        self._map_log_counter = 0
        self._map_shadow_ms = 0.0
        self._map_render_ms = 0.0
        self._map_prev_odom = None

        # Serial
        self.serial_link = serial_link if serial_link is not None else SerialLink(cfg.serial_port, cfg.baudrate)

        # Captura / grabación
        self.loop_count    = 0
        self.frame_grabber = None
        self.video_writer  = None
        self.bev_recorder  = None   # BEV limpio cada frame (REC_BEV_CLEAN), ver bev_recorder.py
        self.record_count  = 0
        self.cam_frame_count = 0
        self.output_file   = (resolve_output_path(cfg.record_output)
                              if cfg.record_orillas else None)

        # Estado del loop principal (FPS / timing entre frames procesados)
        self._last_fps_time = 0.0
        self._fps_count     = 0
        self._fps           = 0.0
        self._t_prev_end    = 0.0
        self._timing_ms     = {"cap": 0.0, "vis": 0.0, "bev": 0.0, "ser": 0.0, "disp": 0.0, "rec": 0.0}

        try:
            self.vision.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        if self.bev.is_calibrated:
            print("[PP] BEV calibrado — Pure Pursuit + memoria activo.", flush=True)
        else:
            print("[PP] Sin calibración BEV — el robot irá recto.", flush=True)
            print(f"[PP] Corre: python -m pure_pursuit.calibrate", flush=True)

    # ── Serial ────────────────────────────────────────────────────────────────

    def _build_serial_message(self, obs_norm: float, state: str, n_mem_obs: int,
                               pasado: bool, interior: bool,
                               turn_block: bool = False) -> str:
        # turn_block: se acaba de pasar un cono exterior de esquina y todavía
        # NO se quiere soltar el giro (el cono podría estar al lado por
        # arrastre de memoria). Fuerza prio/mem para que el ESP32 mantenga
        # bloqueado detectarEsquina() unos frames más.
        has_obstacle = (n_mem_obs > 0) or turn_block
        mem_out = max(n_mem_obs, 1) if turn_block else n_mem_obs
        return (f"V2,obs={obs_norm:+.3f},turn=0,"
                f"state={state},prio={int(has_obstacle)},mem={mem_out},pp=1,"
                f"pasado={int(pasado)},intr={int(interior)},"
                f"inicio={int(bool(self._inicio_estacionamiento))},"
                f"park={self._park_state},pd={self._park_dist_cm}")

    # ── Mapa de pista ─────────────────────────────────────────────────────────

    @staticmethod
    def _track_map_active() -> bool:
        return bool(getattr(C, "TRACK_MAP_SHADOW", False)
                    or getattr(C, "TRACK_MAP_ENABLED", False))

    @staticmethod
    def _track_map_controls() -> bool:
        return bool(getattr(C, "TRACK_MAP_ENABLED", False))

    def _reset_map_state(self) -> None:
        self._odom = Odometry()
        self._map_geometry = None
        self._map_frame = None
        self._map_localizer = None
        self._map_signmap = None
        self._map_field_pose = None
        self._map_last_odom = None
        self._map_pending.clear()
        self._map_status = {}
        self._map_drive_resolved = None
        self._map_odom_at_arm = None
        self._map_snap_tries = 0
        self._map_failed = False
        self._map_prev_odom = None

    def _us_cm_from_fields(self, fields: dict[str, str], key: str) -> float | None:
        v = fields.get(key)
        if v is None:
            return None
        try:
            cm = float(v)
        except ValueError:
            return None
        # <= 0: sin eco / error del ESP; >= 190: tope del timeout (pasillo abierto)
        if cm <= 0.0 or cm >= 190.0:
            return None
        return cm

    def _tof_mm_from_fields(self, fields: dict[str, str], key: str) -> float | None:
        v = fields.get(key)
        if v is None:
            return None
        try:
            mm = float(v)
        except ValueError:
            return None
        if mm < 0:
            return None
        return mm

    def _build_map_readings(self, fields: dict[str, str]) -> list[SensorReading]:
        out: list[SensorReading] = []
        dL = self._us_cm_from_fields(fields, "dL")
        dR = self._us_cm_from_fields(fields, "dR")
        dF = self._us_cm_from_fields(fields, "dF")
        if dL is not None:
            out.append(SensorReading(
                "us_left", "us", DEFAULT_MOUNTS["us_left"], US_HALF_ANGLE_DEG, dL * 10.0))
        if dR is not None:
            out.append(SensorReading(
                "us_right", "us", DEFAULT_MOUNTS["us_right"], US_HALF_ANGLE_DEG, dR * 10.0))
        if dF is not None:
            out.append(SensorReading(
                "us_front", "us", DEFAULT_MOUNTS["us_front"], US_HALF_ANGLE_DEG, dF * 10.0))
        for key, name in (("tL", "tof_left"), ("tR", "tof_right"), ("tB", "tof_rear")):
            mm = self._tof_mm_from_fields(fields, key)
            if mm is not None:
                out.append(SensorReading(
                    name, "tof", DEFAULT_MOUNTS[name], 0.0, mm))
        return out

    def _resolve_map_drive_dir(
        self, fields: dict[str, str], ack: str | None,
    ) -> str | None:
        # Sin fuente confiable se espera: los sonares al armar solo dicen en qué
        # lado del pasillo pusieron el carro, no hacia dónde va la pista, y una
        # geometría espejeada ya no se corrige en toda la carrera.
        d = getattr(C, "CORNER_TURN_DIR_OVERRIDE", None)
        if d in ("L", "R"):
            return d
        d = fields.get("dir") if fields else None
        if d not in ("L", "R"):
            d = _parse_direccion(ack or "")
        if d in ("L", "R"):
            return d
        td = self.turn_dir_tracker.direction
        if td in ("L", "R"):
            return td
        return None

    def _ensure_map_geometry(self, drive_dir: str, odom_pose) -> None:
        if self._map_geometry is not None:
            return
        drive_dir = drive_dir.upper()
        self._map_geometry = TrackGeometry(drive_dir)
        self._map_localizer = Localizer(self._map_geometry)
        self._map_signmap = SignMap(self._map_geometry)
        self._map_frame = TrackFrame(self._map_geometry)
        if self._inicio_estacionamiento:
            # Dentro del cajón, pegado a la pared exterior de la recta de salida.
            w_c = float(getattr(C, "PARK_LOT_CENTER_W_MM", 1177.0))
            start = FieldPose(1500.0 - w_c, -(self._map_geometry.outer_half - 100.0),
                              0.0 if drive_dir == "L" else 180.0)
        else:
            start = self._map_geometry.start_field_pose()
        # La dirección puede llegar segundos después de armar: la pose arranca
        # donde la odometría dice que va el carro, no en la salida.
        ref = self._map_odom_at_arm or odom_pose
        self._map_frame.set_from_odom(ref.x_mm, ref.y_mm, ref.yaw_deg, start=start)
        now_pose = self._map_frame.odom_to_field(
            odom_pose.x_mm, odom_pose.y_mm, odom_pose.yaw_deg)
        self._map_localizer.pose = FieldPose(
            now_pose.x_mm, now_pose.y_mm, now_pose.heading_deg)
        self._map_field_pose = self._map_localizer.pose
        self._map_drive_resolved = drive_dir
        self._map_snap_tries = max(1, int(getattr(C, "MAP_SNAP_FRAMES", 30)))

    def _buffer_map_detections(self, dets: list, odom_pose, now: float) -> None:
        if dets:
            self._map_pending.append((list(dets), odom_pose, now))
            if len(self._map_pending) > 40:
                self._map_pending.pop(0)

    def _replay_map_pending(self) -> None:
        for dets, od_i, t in self._map_pending:
            self._map_signmap.observe(
                dets, self._map_frame.odom_to_field(od_i.x_mm, od_i.y_mm, od_i.yaw_deg), t)
        self._map_pending.clear()

    def _map_localization_good(self) -> bool:
        loc = self._map_localizer
        if loc is None or self._map_geometry is None or self._map_field_pose is None:
            return False
        ok_st = ("ok", "us_only", "tof_only")
        sides = ("left", "right")
        if not all(loc.side_status.get(s) in ok_st for s in sides):
            return False
        sec = self._map_geometry.section_of(
            self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        if sec is None:
            return False
        _, _, axis_h = self._map_geometry.section_frame(sec)
        herr = (self._map_field_pose.heading_deg - axis_h + 180.0) % 360.0 - 180.0
        return abs(herr) < 20.0

    def _map_heading_err_deg(self) -> float:
        if self._map_geometry is None or self._map_field_pose is None:
            return 0.0
        sec = self._map_geometry.section_of(
            self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        if sec is None:
            return 0.0
        _, _, axis_h = self._map_geometry.section_frame(sec)
        return (self._map_field_pose.heading_deg - axis_h + 180.0) % 360.0 - 180.0

    def _map_fusion_status_str(self) -> str:
        loc = self._map_localizer
        if loc is None:
            return "-"
        parts = []
        for side in ("L", "R", "F", "B"):
            key = {"L": "left", "R": "right", "F": "front", "B": "rear"}[side]
            parts.append(f"{side}:{loc.side_status.get(key, '?')[0]}")
        return "".join(parts)

    @_map_safe(lambda bev_obstacles, obstacle_conf: (bev_obstacles, obstacle_conf))
    def _map_apply_control_obstacles(
        self, bev_obstacles: list, obstacle_conf: list[float],
    ) -> tuple[list, list[float]]:
        if not self._track_map_controls() or self._map_signmap is None:
            return bev_obstacles, obstacle_conf
        if self._map_field_pose is None or self._map_geometry is None:
            return bev_obstacles, obstacle_conf
        sec = self._map_geometry.section_of(
            self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        if sec is None:
            return bev_obstacles, obstacle_conf
        dedup = float(getattr(C, "MAP_OBS_DEDUP_PX", 60.0))
        out_obs = list(bev_obstacles)
        out_conf = list(obstacle_conf)
        h_twin = field_heading_to_twin(self._map_field_pose.heading_deg)
        for s in self._map_signmap.signs(sec, "tentative"):
            if s["color"] is None:
                continue
            seat_id = s["seat"]
            if self._map_signmap.passed(sec, seat_id, self._map_field_pose):
                continue
            bx, by = self._map_signmap._field_center_to_bev(
                s["x"], s["y"], self._map_field_pose, h_twin)
            if by >= C.ROBOT_BEV_Y - 20:
                continue
            col = "Red" if s["color"] == "red" else "Green"
            clash = False
            for ox, oy, oc in out_obs:
                if oc != col:
                    continue
                if (ox - bx) ** 2 + (oy - by) ** 2 <= dedup ** 2:
                    clash = True
                    break
            if clash:
                continue
            cf = max(0.6, min(1.0, float(s.get("score", 1.0))))
            out_obs.append((bx, by, col))
            out_conf.append(cf)
        return out_obs, out_conf

    @_map_safe(lambda *args, **kwargs: None)
    def _map_classify_mine_beyond(
        self,
        bev_obstacles: list,
        bev_beyond: list,
        obstacle_conf: list[float],
    ) -> tuple[list, list, list[float]] | None:
        if not getattr(C, "TRACK_MAP_CLASSIFY", False):
            return None
        if not self._map_localization_good():
            return None
        if self._map_geometry is None or self._map_field_pose is None:
            return None
        cur = self._map_geometry.section_of(
            self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        if cur is None:
            return None
        h_twin = field_heading_to_twin(self._map_field_pose.heading_deg)
        mine, beyond, conf_out = [], [], []

        def _field_for_ob(ox, oy):
            right_mm = (ox - C.ROBOT_BEV_X) * C.MM_PER_PX
            fwd_mm = ((C.ROBOT_BEV_Y - oy) * C.MM_PER_PX
                      + C.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM)
            h = math.radians(h_twin)
            wx = (self._map_field_pose.x_mm
                  + fwd_mm * math.sin(h) + right_mm * math.cos(h))
            wy = (self._map_field_pose.y_mm
                  + fwd_mm * math.cos(h) - right_mm * math.sin(h))
            return wx, wy

        for i, (ox, oy, c) in enumerate(bev_obstacles):
            wx, wy = _field_for_ob(ox, oy)
            sec = self._map_geometry.section_of(wx, wy)
            cf = obstacle_conf[i] if i < len(obstacle_conf) else 1.0
            if sec is None or sec == cur:
                mine.append((ox, oy, c))
                conf_out.append(cf)
            else:
                beyond.append((ox, oy, c))
                conf_out.append(cf)
        return mine, beyond, conf_out

    @_map_safe(lambda: False)
    def _map_try_pasado(self) -> bool:
        if not getattr(C, "TRACK_MAP_PASADO", False):
            return False
        if not self._map_localization_good():
            return False
        if self._map_signmap is None or self._map_geometry is None:
            return False
        if self._map_field_pose is None:
            return False
        if abs(self._map_heading_err_deg()) < C.RECUP_MEAS_HEADING_DEG:
            return False
        sec = self._map_geometry.section_of(
            self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        if sec is None:
            return False
        for s in self._map_signmap.signs(sec, "confirmed"):
            if self._map_signmap.passed(sec, s["seat"], self._map_field_pose):
                return True
        return False

    def _map_seat_pass_side_inner(self, color: str) -> bool:
        drive = self._map_geometry.drive_dir if self._map_geometry else "L"
        col = color.lower()
        if drive == "L":
            return col.startswith("g")
        return col.startswith("r")

    @_map_safe(lambda: "")
    def _map_gr_gf_suffix(self) -> str:
        if not self._track_map_controls():
            return ""
        if self._map_geometry is None or self._map_field_pose is None:
            return ""
        sec = self._map_geometry.section_of(
            self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        if sec is None:
            return ""
        next_sec = (sec + 1) % 4
        gr_def = int(getattr(C, "GR_PI_LAP1", 0))
        gf_def = int(getattr(C, "GR_GF_DEFAULT_CM", 50))
        gf_in = int(getattr(C, "GR_GF_INNER_CM", 60))
        gf_out = int(getattr(C, "GR_GF_OUTER_CM", 35))
        if self._map_signmap is None:
            return f",gr={gr_def},gf={gf_def}"
        mapped = bool(self._map_signmap.signs(next_sec, "tentative"))
        if not mapped:
            return f",gr={gr_def},gf={gf_def}"
        seats = self._map_geometry.seats_of_section(next_sec)
        if not seats:
            return f",gr=1,gf={gf_def}"
        alongs = sorted(
            ((self._map_geometry.along_lateral(next_sec, st.x_mm, st.y_mm)[0], st)
             for st in seats),
            key=lambda a: a[0])
        min_al = alongs[0][0]
        near = [st for al, st in alongs if abs(al - min_al) < 80.0]
        sign_at_near = None
        for st in near:
            for s in self._map_signmap.signs(next_sec, "tentative"):
                if s["seat"] == st.seat_id and s["color"]:
                    sign_at_near = s
                    break
            if sign_at_near:
                break
        if sign_at_near is None:
            return f",gr=1,gf={gf_def}"
        gf = gf_in if self._map_seat_pass_side_inner(sign_at_near["color"]) else gf_out
        return f",gr=1,gf={gf}"

    @_map_safe(lambda *args, **kwargs: None)
    def _update_map_after_ack(
        self,
        serial_ack: str | None,
        now: float,
        armed: bool,
        new_obstacles: list,
        dt_s: float,
    ) -> None:
        if not armed or not self._track_map_active():
            return
        t0 = time.perf_counter()
        fields = parse_ack(serial_ack) if serial_ack else {}
        odom_pose = self._odom.update(fields, now, C.ROBOT_SPEED_MMS)
        if self._map_odom_at_arm is None:
            self._map_odom_at_arm = odom_pose
        yaw_rate = 0.0
        if self._map_last_odom is not None and dt_s > 1e-6:
            _, _, dyaw = Odometry.delta(self._map_last_odom, odom_pose)
            yaw_rate = abs(dyaw) / dt_s
        prev_odom = self._map_last_odom
        self._map_last_odom = odom_pose
        if self._map_geometry is None:
            drive = self._resolve_map_drive_dir(fields, serial_ack)
            if drive is None:
                self._buffer_map_detections(new_obstacles, odom_pose, now)
                self._map_shadow_ms = (time.perf_counter() - t0) * 1000.0
                return
            self._ensure_map_geometry(drive, odom_pose)
            prev_odom = None   # la pose nueva ya incluye todo lo recorrido
        loc = self._map_localizer
        if prev_odom is not None:
            dx, dy, dyaw = Odometry.delta(prev_odom, odom_pose)
            loc.predict(dx, dy, dyaw)
        readings = self._build_map_readings(fields)
        if self._map_snap_tries > 0:
            # Lo visto antes de ubicar al carro a lo ancho se guarda y entra al
            # mapa ya con la pose corregida (si no, cae en el asiento de al lado).
            self._map_snap_tries -= 1
            x0, y0 = loc.pose.x_mm, loc.pose.y_mm
            snapped = loc.snap_lateral(readings)
            if snapped:
                self._map_frame.shift(loc.pose.x_mm - x0, loc.pose.y_mm - y0)
                print(f"[MAP] ajuste lateral de arranque "
                      f"{math.hypot(loc.pose.x_mm - x0, loc.pose.y_mm - y0):.0f} mm",
                      flush=True)
            if snapped or self._map_snap_tries == 0:
                self._map_snap_tries = 0
                self._replay_map_pending()
        loc.update(readings, self._map_signmap)
        self._map_field_pose = loc.pose
        skip_obs = yaw_rate > float(getattr(C, "MAP_YAW_RATE_SKIP_DEG_S", 120.0))
        if self._map_snap_tries > 0:
            self._buffer_map_detections(new_obstacles, odom_pose, now)
        elif not skip_obs:
            self._map_signmap.observe(new_obstacles, self._map_field_pose, now)
        sec_i = "?"
        lat_mm = along_mm = 0.0
        if self._map_geometry and self._map_field_pose:
            sec = self._map_geometry.section_of(
                self._map_field_pose.x_mm, self._map_field_pose.y_mm)
            if sec is not None:
                sec_i = sec
                along_mm, lat_mm = self._map_geometry.along_lateral(
                    sec, self._map_field_pose.x_mm, self._map_field_pose.y_mm)
        self._map_status = {
            "section": sec_i,
            "lateral": lat_mm,
            "along": along_mm,
            "odom_source": odom_pose.source,
            "odom_quality": odom_pose.quality,
            "fusion": self._map_fusion_status_str(),
        }
        self._map_shadow_ms = (time.perf_counter() - t0) * 1000.0
        p = self._map_field_pose
        sm_sum = self._map_signmap.summary() if self._map_signmap else "-"
        if p is not None:
            print(
                f"[MAP] src={odom_pose.source} pose=({p.x_mm:.0f},{p.y_mm:.0f},"
                f"{p.heading_deg:.0f}) sec={sec_i} lat={lat_mm:.0f} "
                f"st={self._map_fusion_status_str()} {sm_sum}",
                flush=True,
            )
        self._map_log_counter += 1
        n_log = int(getattr(C, "MAP_LOG_COST_EVERY_N", 60))
        if n_log > 0 and self._map_log_counter % n_log == 0:
            print(f"[MAP] shadow_ms={self._map_shadow_ms:.2f} "
                  f"render_ms={self._map_render_ms:.2f}", flush=True)

    @_map_safe(_gray_map_panel)
    def render_map_panel(self, size: int) -> np.ndarray:
        t0 = time.perf_counter()
        geom = self._map_geometry
        pose = self._map_field_pose
        if geom is None or pose is None:
            img = _gray_map_panel(size)
            cv2.putText(img, "MAP: sin geometria", (10, size // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 80), 1)
        else:
            img = render_map(
                geom, self._map_signmap, pose, size=size,
                status=self._map_status,
            )
        self._map_render_ms = (time.perf_counter() - t0) * 1000.0
        return img

    def _green_ahead(self, obstacles) -> bool:
        """Verde todavía adelante y cerca. El giro rápido lo ignora y se clava
        al tope; gv=1 le pide abrir el arco para pasarlo por la izquierda."""
        y_min_ahead = 40.0
        max_mm = 360.0
        for item in obstacles or []:
            if len(item) < 3 or item[2] != "Green":
                continue
            dx = float(item[0]) - C.ROBOT_BEV_X
            dy = C.ROBOT_BEV_Y - float(item[1])
            if dy < y_min_ahead:
                continue
            if math.hypot(dx, dy) * C.MM_PER_PX > max_mm:
                continue
            return True
        return False

    def _corner_hint(self, line_info: dict, now: float) -> bool:
        """es= del V2: naranja vista cerca en los últimos CORNER_HINT_HOLD_S. Se
        apaga en cuanto el ESP entra al giro, para no arrastrarlo a la recta nueva."""
        if self._prev_estado == "G" or self._is_turning:
            self._corner_hint_until = 0.0
            return False
        o = line_info.get("Orange") or {}
        ny = o.get("near_y")
        if o.get("seen") and ny is not None and ny >= float(getattr(C, "CORNER_HINT_NEAR_Y", 120.0)):
            self._corner_hint_until = now + float(getattr(C, "CORNER_HINT_HOLD_S", 1.5))
        return now < self._corner_hint_until

    def _measured_recup_trigger(self, cl_stats: dict, bev_obstacles: list,
                                corner_soon: bool = False,
                                fresh_color: bool = True) -> bool:
        """
        Decide si el robot ACABA de rebasar un obstáculo de LADO (esquiva de
        ángulo) — el disparo de RECUPERANDO que el ancla "geom" nunca acertó.

        NO hace dead-reckoning propio: usa la posición que la memoria YA tiene de
        la lata (o.x/o.y — corregida por detecciones frescas mientras se ve, y
        arrastrada por el MISMO ego-movimiento que ve la centerline cuando no) y
        el heading real. Dispara cuando se cumplen las tres:

          1. Hubo una esquiva DE VERDAD en curso: el peso de esquiva de
             detect_centerline (cl_stats["weights"]) llegó a >= RECUP_MEAS_ARM_W
             en alguna fila junto al eje -> self._dodge_armed.
          2. Ya NO hay lata "estorbando" derecho adelante: ninguna Red/Green de
             la memoria está a la vez > AHEAD_TOL px adelante del eje Y a menos
             de CLEAR_PX px de lado (yendo recto el borde del carro la libra).
             Debe cumplirse RECUP_MEAS_CLEAR_FRAMES frames seguidos -- o solo
             RECUP_MEAS_CLEAR_FRAMES_CORNER (1) si corner_soon (naranja encima):
             el debounce de 3 llegaba tarde y el ESP32 metía un giro falso antes
             (orillas440). corner_soon lo pasa el llamador desde line_info Orange.
          3. El chasis quedó chueco vs la recta: |heading - heading_ref| >=
             RECUP_MEAS_HEADING_DEG. heading_ref se fija al ARMARSE (heading de
             la recta antes de empezar a rodear).

        (3) separa "esquiva suave con espacio" (heading chico -> NO dispara, PP +
        wall PID enderezan solos) de "latiguazo sin espacio" (heading grande ->
        dispara justo al terminar de rodear la lata).

        Devuelve True UNA sola vez por esquiva (se desarma al disparar).
        """
        if not getattr(C, "RECUP_MEAS_ENABLED", True):
            return False

        # OJO: cl_stats["weights"] mezcla el peso de esquiva de LATA con el peso
        # 1.0 que detect_centerline pone en filas SIN PISO (pared de frente,
        # proyección de tendencia). Esas ramas solo corren si NO hay lata de
        # color en la lista -> si no hay Red/Green, el peso junto al eje NO es de
        # esquiva y no debe armar nada.
        has_color_obs = any(c in ("Red", "Green") for _, _, c in bev_obstacles)
        weights = cl_stats.get("weights") or []
        # points[0] es la fila más cercana al eje; ROW_STEP px entre filas.
        n_near = max(1, int(C.RECUP_MEAS_NEAR_PX / C.CENTERLINE_ROW_STEP))
        max_w_near = max(weights[:n_near], default=0.0) if has_color_obs else 0.0

        # (1) ARMAR — LATCH. Una vez que el planner rodeó una lata de verdad
        # (max_w_near >= ARM_W), _dodge_armed queda en True y el chequeo de
        # disparo corre CADA frame -- NO se re-arma-y-retorna mientras el peso
        # siga alto (ese era el bug de orillas412: conf ~1.0 mantenía el peso
        # arriba toda la esquiva, así que el disparo solo se evaluaba tras el
        # prune de la lata -> igual de tarde que BEHIND_PAD). heading_ref se
        # siembra UNA vez, al armar.
        # Tras disparar, no re-armar hasta que el peso baje de ARM_W al menos un
        # frame -> exige una esquiva NUEVA (transición bajo->alto), no la misma
        # lata que sigue en memoria mientras el ESP32 endereza (evita re-disparo
        # de RECUPERANDO en cadena sobre el mismo obstáculo).
        # Debounce de armado: la esquiva tiene que estar en curso ARM_FRAMES
        # frames seguidos (peso alto) antes de armar. Un verde que se ve 1 solo
        # frame y se pierde (tras un giro, FOV rasante) ya NO arma -> no entra
        # a RECUPERANDO "super rápido" con la lata todavía enfrente (orillas417/8).
        if max_w_near >= C.RECUP_MEAS_ARM_W:
            self._recup_arm_streak = getattr(self, "_recup_arm_streak", 0) + 1
        else:
            self._recup_arm_streak = 0
            self._recup_can_arm = True
        if (self._recup_arm_streak >= getattr(C, "RECUP_MEAS_ARM_FRAMES", 3)
                and not self._dodge_armed
                and getattr(self, "_recup_can_arm", True)):
            self._dodge_armed = True
            self._recup_can_arm = False
            self._recup_clear_count = 0
            self._recup_noghost_streak = 0
            self._heading_ref = None   # se siembra abajo con el heading de ESTE frame

        # Siembra PEREZOSA de heading_ref: en cuanto haya heading y la esquiva
        # esté armada. Antes se sembraba SOLO en el frame de armado -> si se armó
        # sin heading (p.ej. el trigger llegó a correr desarmado, o el ACK aún no
        # traía ang=), quedaba en None para siempre y heading_err=0 -> measured
        # nunca disparaba (bug de orillas414).
        if (self._dodge_armed and self._heading_ref is None
                and self._last_heading is not None):
            self._heading_ref = self._last_heading

        heading_err = 0.0
        if self._last_heading is not None and self._heading_ref is not None:
            heading_err = (self._last_heading - self._heading_ref + 180.0) % 360.0 - 180.0

        # DIAG (solo log, sin efecto): estado de armado del trigger cada frame que
        # corre — cubre el hueco de que el early-return de abajo no registra
        # arm_streak / max_w_near cuando _dodge_armed aún es False.
        print(f"[RECUParm] w={max_w_near:.2f} streak={self._recup_arm_streak} "
              f"can_arm={int(self._recup_can_arm)} armed={int(self._dodge_armed)} "
              f"herr={heading_err:+.1f} "
              f"href={'-' if self._heading_ref is None else round(self._heading_ref)} "
              f"h={'-' if self._last_heading is None else round(self._last_heading)} "
              f"hascolor={int(has_color_obs)} clr={self._recup_clear_count} "
              f"csoon={int(bool(corner_soon))} fresh={int(fresh_color)} "
              f"noghost={getattr(self, '_recup_noghost_streak', 0)}", flush=True)

        if not self._dodge_armed:
            return False

        # (2) ¿alguna lata todavía estorbando adelante? SOLO por distancia
        # LONGITUDINAL (ry - oy). NO se usa la x de la lata: cuando la lata está
        # cerca de la cámara su proyección BEV se "unta" y o.x se queda pegada al
        # centro toda la esquiva (visto en pista: red x=232->179->193 mientras el
        # carro giraba 60°) -> con el término lateral "blocking" no se limpiaba
        # nunca y el disparo caía igual de tarde que el respaldo BEHIND_PAD.
        #
        # ahead_tol ESCALA con el giro: al enderezar desde herr grande el morro
        # barre un arco grande y lo puede meter en la lata si ésta sigue adelante
        # (orillas415: verde disparó a herr+59 con la lata 72px adelante -> morro
        # dentro del verde). Cuanto mayor el giro, más cerca del eje (o detrás)
        # debe estar la lata para disparar.
        ry = C.ROBOT_BEV_Y
        _tol0 = float(getattr(C, "RECUP_MEAS_AHEAD_TOL_PX", 90.0))
        _hlo  = float(getattr(C, "RECUP_MEAS_AHEAD_TOL_HARD_LO", 45.0))
        _hhi  = float(getattr(C, "RECUP_MEAS_AHEAD_TOL_HARD_HI", 68.0))
        _a = abs(heading_err)
        if _a <= _hlo:
            ahead_tol = _tol0                       # esquiva "normal": tol pleno
        elif _a >= _hhi:
            ahead_tol = 0.0                         # latiguazo extremo: lata al eje o detrás
        else:
            ahead_tol = _tol0 * (_hhi - _a) / (_hhi - _hlo)
        blocking = any(
            c in ("Red", "Green") and (ry - oy) > ahead_tol
            for (ox, oy, c) in bev_obstacles
        )
        # Bypass del ghost: si la lata que "estorba" no se ve FRESCA (la cámara
        # no la detectó) hace RECUP_MEAS_GHOST_CLEAR_FRAMES frames y el chasis ya
        # está chueco, el carro ya la rodeó -> su ghost de memoria dead-reckoned
        # no debe retener RECUPERANDO. orillas490: el verde salió del FOV a
        # y=291, su ghost avanzaba ~3px/frame y el ahead_tol se achica con el
        # yaw -> "blocking" eterno -> el ESP llegó a la esquina a 51° sin
        # enderezar.
        if not fresh_color and abs(heading_err) >= C.RECUP_MEAS_HEADING_DEG:
            self._recup_noghost_streak = getattr(self, "_recup_noghost_streak", 0) + 1
        else:
            self._recup_noghost_streak = 0
        ghost_stale = (self._recup_noghost_streak
                       >= getattr(C, "RECUP_MEAS_GHOST_CLEAR_FRAMES", 3))
        # Respaldo: si el planner tampoco rodea nada junto al eje, está despejado
        # aunque la memoria aún cargue la lata en algún lado raro.
        path_clear = (not blocking) or (max_w_near <= C.RECUP_MEAS_CLEAR_W) or ghost_stale

        if not path_clear:
            self._recup_clear_count = 0
            # DIAG 2026-09-09 (solo-log, sin efecto): geometría de cada lata de
            # color que retiene `rodeando`, para verificar la cláusula "ya cruzó
            # al lado interno del esquive" antes de implementarla:
            #   dx = ox - eje  (signo vs heading_err = de qué lado quedó)
            #   swept = |herr|>=HEADING_DEG y dx*herr>0  (cruzó, enderezar se aleja)
            _hd = C.RECUP_MEAS_HEADING_DEG
            _cones = []
            _swept = False
            for (ox, oy, c) in bev_obstacles:
                if c not in ("Red", "Green"):
                    continue
                dx = ox - C.ROBOT_BEV_X
                sw = abs(heading_err) >= _hd and dx * heading_err > 0.0
                _swept = _swept or sw
                _cones.append(f"{c[0]}dx={dx:+.0f},ahead={ry - oy:.0f},sw={int(sw)}")
            self._last_recup_reason = (
                f"rodeando herr={heading_err:+.0f} w={max_w_near:.2f} "
                f"atol={ahead_tol:.0f} swept={int(_swept)} [{' '.join(_cones)}]")
            return False
        self._recup_clear_count += 1

        # (3) DISPARAR en cuanto la lata quedó atrás Y el chasis está chueco de
        # verdad. El chequeo corre cada frame mientras se sigue "despejado" -->
        # aunque el heading TODAVÍA esté creciendo (mitad del latiguazo), dispara
        # en el frame en que cruza HEADING_DEG. Antes se desarmaba a la primera
        # de |herr|<HEADING_DEG con el path despejado y perdía el latiguazo que
        # aún no llegaba a 25° (visto en sim con la traza de la red de orillas412).
        _clear_req = (getattr(C, "RECUP_MEAS_CLEAR_FRAMES_CORNER", 1)
                      if corner_soon else C.RECUP_MEAS_CLEAR_FRAMES)
        if (self._recup_clear_count >= _clear_req
                and abs(heading_err) >= C.RECUP_MEAS_HEADING_DEG):
            self._last_recup_reason = (
                f"PASADO(medido) herr={heading_err:+.0f} "
                f"clr={self._recup_clear_count}")
            self._dodge_armed = False
            self._recup_clear_count = 0
            self._heading_ref = None
            return True

        # Despejado MUCHOS frames y el heading nunca llegó a HEADING_DEG ->
        # esquiva suave que se resolvió sola. Desarmar sin RECUPERANDO, y
        # permitir re-armar (la siguiente lata sí puede necesitar RECUPERANDO).
        # OLVIDAR la lata: si el path está despejado tanto tiempo y no hubo
        # latiguazo, la lata ya quedó al costado -> que la centerline deje de
        # rodearla (si no, un cono que se queda pegado al eje trasero, giro~0,
        # re-detectado, mantiene prio=1 y el steer oscilando -- visto lap 3
        # orillas420: R en y~321 falta+24 por decenas de frames).
        if self._recup_clear_count >= getattr(C, "RECUP_MEAS_GENTLE_FRAMES", 10):
            self._last_recup_reason = f"esquiva-suave-ok herr={heading_err:+.0f}"
            self._dodge_armed = False
            self._recup_can_arm = True
            self._recup_clear_count = 0
            self._heading_ref = None
            # Solo olvidar si NINGÚN cono sigue de verdad adelante (>40px del
            # eje): así no se borra un obstáculo que el carro tiene justo
            # enfrente y todavía debe rodear.
            if not any(c in ("Red", "Green") and (ry - oy) > 40.0
                       for (ox, oy, c) in bev_obstacles):
                self.memory.forget_color_obstacles()
        else:
            self._last_recup_reason = (
                f"despejado herr={heading_err:+.0f} clr={self._recup_clear_count}")
        return False

    # ── Captura ───────────────────────────────────────────────────────────────

    def _start_capture(self):
        print(f"[CAM] cap.isOpened()={self.vision.cap.isOpened()} "
              f"threaded={self.cfg.threaded_capture}", flush=True)
        if self.cfg.threaded_capture:
            self.frame_grabber = ThreadedFrameGrabber(self.vision.cap).start()
            print("[CAM] ThreadedFrameGrabber iniciado.", flush=True)

    def _read_frame(self):
        if self.frame_grabber is not None:
            return self.frame_grabber.read()
        return self.vision.cap.read()

    # ── Chequeo de cámara (ver CAM_CHECK_* en config) ─────────────────────────

    @staticmethod
    def _frame_negro(frame) -> bool:
        gray = cv2.cvtColor(frame[::4, ::4], cv2.COLOR_BGR2GRAY)
        return float(np.percentile(gray, 99)) < C.CAM_CHECK_P99_MIN

    def _camera_check(self) -> tuple[bool, str]:
        """Espera CAM_CHECK_FRAMES frames NUEVOS con imagen real."""
        t0 = time.monotonic()
        last_id, buenos, negros = -1, 0, 0
        while time.monotonic() - t0 < C.CAM_CHECK_TIMEOUT_S:
            if self.frame_grabber is not None:
                fid, _age = self.frame_grabber.freshness()
                if fid == 0 or fid == last_id:
                    time.sleep(0.01)
                    continue
                last_id = fid
            ret, frame = self._read_frame()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            if self._frame_negro(frame):
                negros += 1
            else:
                buenos += 1
                if buenos >= C.CAM_CHECK_FRAMES:
                    return True, f"{buenos} frames buenos en {time.monotonic() - t0:.1f}s"
        if buenos == 0 and negros == 0:
            return False, f"sin frames nuevos en {C.CAM_CHECK_TIMEOUT_S:.0f}s"
        return False, f"{buenos} buenos / {negros} negros en {C.CAM_CHECK_TIMEOUT_S:.0f}s"

    def _reopen_camera(self):
        if self.frame_grabber is not None:
            self.frame_grabber.stopped = True
            self.frame_grabber.thread.join(timeout=C.CAM_REOPEN_JOIN_S)
            if self.frame_grabber.thread.is_alive():
                # Soltar la cámara con otro hilo metido en cap.read() puede tumbar
                # GStreamer; mejor salir y que systemd relance el servicio.
                print("[CAM-CHECK] Hilo de captura colgado -> salgo (systemd reinicia).", flush=True)
                os._exit(3)
            self.frame_grabber = None
        try:
            self.vision.cap.release()
        except Exception as e:
            print(f"[CAM-CHECK] release falló: {e}", flush=True)
        time.sleep(C.CAM_REOPEN_WAIT_S)
        try:
            self.vision.cap = open_camera(self.cfg.cam_index)
            self.vision.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as e:
            print(f"[CAM-CHECK] No se pudo reabrir la cámara: {e}", flush=True)
            return
        self._start_capture()

    def _camera_reopen_and_gate(self):
        self._reopen_camera()
        self._camera_gate()

    def _camera_gate(self):
        """Bloquea (LED apagado) hasta que la cámara entregue imagen real."""
        intento = 0
        while True:
            intento += 1
            ok, why = self._camera_check()
            if ok:
                print(f"[CAM-CHECK] OK ({why}).", flush=True)
                return
            print(f"[CAM-CHECK] FALLA intento {intento}: {why} -> reabro la cámara.", flush=True)
            self._reopen_camera()

    def _maybe_record(self, frame: np.ndarray, fps: float,
                      bev: np.ndarray | None = None):
        if not self.cfg.record_orillas:
            return
        self.record_count += 1
        # BEV limpio de CADA frame (no 1 de cada record_every_n): el .avi del HUD
        # no permite re-probar la detección de la naranja offline.
        if bev is not None and getattr(C, "REC_BEV_CLEAN", False):
            if self.bev_recorder is None:
                _bp = self.output_file.with_name(self.output_file.stem + "_bev.bin")
                self.bev_recorder = BevRecorder(
                    str(_bp), quality=int(getattr(C, "REC_BEV_JPEG_QUALITY", 95)))
                print(f"[REC] BEV limpio en {_bp} (n={self.record_count})", flush=True)
            self.bev_recorder.write(self.record_count, bev)
        if self.record_count % max(1, self.cfg.record_every_n) != 0:
            return
        if frame is None:            # HUD no armado en este frame
            return
        if self.video_writer is None:
            out_fps = (max(self.cfg.record_fps, fps / max(1, self.cfg.record_every_n))
                       if fps > 0 else self.cfg.record_fps)
            self.video_writer = AsyncVideoWriter(
                str(self.output_file),
                frame.shape[1], frame.shape[0], out_fps,
            ).start()
            print(f"[REC] Grabando en {self.output_file}", flush=True)
        self.video_writer.write(frame.copy())

    def _write_cam_frame(self, frame: np.ndarray):
        # Throttle: esto es para la vista remota en vivo (VNC), no para el
        # video grabado -- no necesita actualizarse cada frame procesado.
        # Antes corría SIEMPRE, sin condición, comiéndose un encode JPEG +
        # escritura a disco síncrona en cada frame.
        self.cam_frame_count += 1
        if self.cam_frame_count % max(1, self.cfg.cam_frame_every_n) != 0:
            return
        if frame is None:            # HUD no armado en este frame
            return
        try:
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with open(CAM_FRAME_PATH, "wb") as f:
                f.write(buf.tobytes())
        except Exception:
            pass

    # ── Anotación ─────────────────────────────────────────────────────────────

    def _annotate(
        self,
        frame:        np.ndarray,
        steer_deg:    float,
        obs_norm:     float,
        pp_active:    bool,
        n_path_pts:   int,
        positions:    dict,
        serial_msg:   str,
        fps:          float,
        n_mem:        int,
        prune_reason: str = "-",
        timing_ms:    dict | None = None,
        bev_timing:   dict | None = None,
    ):
        lines = [
            f"fps={fps:.1f}  pp={'ON' if pp_active else 'OFF'}  pts={n_path_pts}",
            f"steer={steer_deg:+.1f} deg  obs={obs_norm:+.3f}",
            f"obs_R={len(positions.get('Red', []))}  obs_G={len(positions.get('Green', []))}  mem={n_mem}",
            f"tx: {serial_msg[:55]}",
            f"mem_prune: {prune_reason}",
            f"mem_closest: {self.memory.debug_closest()}",
            f"mem_all: {self.memory.debug_all()}",
        ]
        if timing_ms is not None:
            lines.append(
                "t(ms): cap={cap:.0f} vis={vis:.0f} bev={bev:.0f} "
                "ser={ser:.0f} disp={disp:.0f} rec={rec:.0f}".format(**timing_ms)
            )
        if bev_timing is not None:
            lines.append(
                "bev: warp={warp:.0f} proj={proj:.0f} mem={mem:.0f} "
                "line={line:.0f} dc={dc:.0f} ctrl={ctrl:.0f}".format(**bev_timing)
            )
        y = 22
        for txt in lines:
            cv2.putText(frame, txt, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)
            y += 22

    def _draw_park_pink(self, frame, ratio, mask):
        """HUD del rosa: un cuadrado sobre el magenta detectado + el ratio.
        SOLO se llama DESPUÉS del pipeline (nunca sobre el frame que va al BEV)."""
        col = (200, 0, 200)                       # magenta BGR
        thr = float(getattr(C, "PARK_PINK_RATIO_MIN", 0.45))
        if mask is not None:
            ys, xs = np.where(mask > 0)
            if xs.size > 200:
                cv2.rectangle(frame, (int(xs.min()), int(ys.min())),
                              (int(xs.max()), int(ys.max())), col, 2)
        txt = (f"PARK state={self._park_state} pink={ratio * 100:.1f}%"
               if (self._park_buscando or self._tc >= self._tpr)
               else f"PINK {ratio * 100:.0f}% / thr {thr * 100:.0f}%")
        cv2.putText(frame, txt,
                    (10, frame.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

    def _check_parking_search(self, frame_bgr, armed: bool):
        """Búsqueda activa del cajón magenta en la recta final (tc >= TURNS_PER_RACE del ESP)."""
        if not armed:
            return
        if not (self._park_buscando or self._tc >= self._tpr):
            return
        if self._park_state == 2:
            return  # ya disparó la orden de estacionamiento

        ratio, mask = _park_pink(frame_bgr)
        if mask is not None and ratio >= 0.005:
            ys, xs = np.where(mask > 0)
            if xs.size > 150:
                y_max = int(ys.max())
                self._park_frames_seen += 1
                self._park_lost_frames = 0
                self._park_max_y_seen = max(self._park_max_y_seen, y_max)
                self._park_dist_cm = max(0, int((frame_bgr.shape[0] - y_max) * 0.4))
                if self._park_state == 0:
                    self._park_state = 1
                    print(f"[PARK] Cajon detectado en recta! ratio={ratio*100:.1f}% y_max={y_max} frames={self._park_frames_seen}", flush=True)

                # Si el borde inferior del bloque magenta está abajo en el FOV (al lado del chasis)
                # O si el ratio creció mucho (cajón ocupando gran parte de la cámara)
                if y_max >= 420 or ratio >= 0.20:
                    self._park_state = 2
                    print(f"[PARK] Cajon alcanzado de frente (y_max={y_max}, ratio={ratio*100:.1f}%) -> PARK=2!", flush=True)
        else:
            if self._park_state == 1:
                self._park_lost_frames += 1
                # Solo declaramos que el carro lo rebasó si antes estuvo REALMENTE CERCA
                # (max_y >= 370 px, junto al parachoques frontal) y ahora se perdió por rebase lateral.
                if self._park_frames_seen >= 4 and self._park_max_y_seen >= 370 and self._park_lost_frames >= 3:
                    self._park_state = 2
                    print(f"[PARK] Cajon rebasado por el lado (seen={self._park_frames_seen}, max_y={self._park_max_y_seen}, lost={self._park_lost_frames}) -> PARK=2!", flush=True)
                elif self._park_lost_frames > 25:
                    # Se perdió estando lejos (ej. tapado temporalmente por un cono o maniobra):
                    # resetear para re-detectar cuando vuelva a verse más adelante.
                    print(f"[PARK] Cajon perdido a lo lejos (max_y={self._park_max_y_seen}), reseteando busqueda...", flush=True)
                    self._park_state = 0
                    self._park_frames_seen = 0
                    self._park_lost_frames = 0
                    self._park_max_y_seen = 0


    def arm(self, sleep_fn=time.sleep) -> bool:
        # READY 3x rápido (el ESP32 solo necesita 1 para salir de su
        # wait). NO bloquear ~0.7 s esperando ACK: el stream de V2
        # que arranca acto seguido mantiene vivo al ESP32, y este
        # ya NO rueda hasta el 1er V2 (ver piFirstV2Received .ino).
        for _ in range(3):
            self.serial_link.send_line("READY")
            sleep_fn(0.03)
        ready_ack = "ACK:READY" in (self.serial_link.try_readline() or "")
        self._last_update_t = None   # dt limpio para el 1er frame armado
        # La dirección de giro se infiere SOLO durante la corrida:
        # descartar cualquier latch del rato desarmado (el carro
        # pudo estar apuntando a una naranja que no es la del 1er
        # giro de la carrera).
        self.turn_dir_tracker.reset()
        self.mid_turn.reset()
        self._ext_corner_block = 0
        self._pasado_hold = 0
        self._pasado_from_measured = False
        self._turn_delay_frames = 0
        self._corner_hint_until = 0.0
        self._reset_map_state()
        print(f"[GPIO] GO — READY x3 enviado (ack={'sí' if ready_ack else '?'}).", flush=True)
        return ready_ack

    def process_frame(self, frame, now: float, armed: bool) -> FrameResult:
        # FPS contador
        self._fps_count += 1
        if now - self._last_fps_time >= 1.0:
            self._fps           = self._fps_count / (now - self._last_fps_time)
            self._fps_count     = 0
            self._last_fps_time = now

        # dt para la memoria rodante (tiempo entre frames procesados).
        # DESARMADO -> dt=0: el carro no se mueve, la memoria no debe
        # "marchar" las latas hacia el robot mientras esperamos el botón.
        if self._last_update_t is None or not armed:
            dt_s = 0.0
        else:
            dt_s = now - self._last_update_t
        self._last_update_t = now

        # Tiempo de captura: desde que terminó de procesar el frame
        # anterior hasta que este frame quedó listo para procesar
        # (incluye el bloqueo real de _read_frame() + el overhead de
        # frames saltados por process_every_n).
        self._timing_ms["cap"] = (now - self._t_prev_end) * 1000.0

        # ── Visión ──────────────────────────────────────────────────
        # Corrección de color por el piso ANTES de todo lo que usa
        # rangos HSV (conos, BEV/naranja/piso, rosa). Con la luz del
        # cuarto de pruebas las ganancias son ~1 y no cambia nada.
        frame = self.color_corr.process(frame)
        frame = cv2.flip(frame, 1)
        processed_frame, positions = self.vision.process_frame(frame)
        t_vis = time.perf_counter()
        self._timing_ms["vis"] = (t_vis - now) * 1000.0

        # ── INICIO: mide el rosa mientras está DESARMADO; al ARMAR
        # decide UNA vez si el carro arranca dentro del estacionamiento.
        # La Pi NO hace nada más: sigue el pipeline normal y el ESP32
        # ignora `obs` mientras dura su maniobra (PurePursuit.ino
        # `case INICIO`). El HUD se dibuja después, ver abajo.
        if not armed:
            self._pink_samples.append(_park_pink(processed_frame)[0])
            if len(self._pink_samples) > C.PARK_PINK_SAMPLES:
                self._pink_samples.pop(0)
        elif self._inicio_estacionamiento is None:
            if not self._pink_samples:
                self._pink_samples.append(_park_pink(processed_frame)[0])
            _avg    = float(np.mean(self._pink_samples))
            _forced = bool(getattr(C, "PARK_FORCE_INICIO", False))
            self._inicio_estacionamiento = _forced or (_avg >= C.PARK_PINK_RATIO_MIN)
            print(f"[INICIO] rosa avg={_avg:.2f} umbral={C.PARK_PINK_RATIO_MIN:.2f} "
                  f"n={len(self._pink_samples)}{' FORZADO' if _forced else ''} -> "
                  f"{'MANIOBRA DE SALIDA (inicio=1)' if self._inicio_estacionamiento else 'arranque normal'}",
                  flush=True)

        # ── Búsqueda activa de cajón en recta final (después del giro 12) ──
        self._check_parking_search(processed_frame, armed)

        # ── Pipeline Pure Pursuit ────────────────────────────────────
        steer_deg     = 0.0
        obs_norm      = 0.0
        lookahead_pt  = (float(C.ROBOT_BEV_X), float(C.ROBOT_BEV_Y))
        path_points   = []
        bev_frame     = None
        bev_obstacles = []
        obstacle_conf: list[float] = []   # alineado 1:1 con bev_obstacles
        pp_active     = False
        pasado        = False
        measured_pass = False
        cl_stats: dict = {}
        lookahead_eff = float(C.LOOKAHEAD_MAX_PX)   # diag: lookahead PP usado este frame
        line_info     = {"Orange": {"seen": False, "near_y": None}}
        new_obstacles = []
        serial_ack = getattr(self, "_prev_ack", None)
        new_obs_h     = []
        bev_obstacles_beyond = []
        interior      = False
        self._ext_corner_hold = False
        bev_timing    = {"warp": 0.0, "proj": 0.0, "mem": 0.0, "line": 0.0, "dc": 0.0, "ctrl": 0.0}

        if self.bev.is_calibrated:
            try:
                _t0 = time.perf_counter()
                bev_frame = self.bev.warp(processed_frame)
                # Se convierte UNA sola vez y se comparte con
                # detect_centerline() y line_tracker.update() -- antes
                # cada una convertía la misma imagen BGR->HSV por su
                # cuenta, duplicando trabajo cada frame.
                bev_hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV)
                _t1 = time.perf_counter()
                bev_timing["warp"] = (_t1 - _t0) * 1000.0

                # Proyectar obstáculos detectados al plano BEV, y
                # separar los que NO proyectaron (candidatos a hint lejano)
                new_obstacles = []
                new_obs_h     = []   # alto del bbox de CÁMARA, 1:1 con new_obstacles
                                     # (proximidad REAL; el BEV y miente con un
                                     # cono pasando la esquina -- ver el lock abajo)
                far_objects   = []   # (center_x, w, h, color) — fuera de BEV
                for color_name in ("Red", "Green"):
                    for obj in positions.get(color_name, []):
                        x, y, w, h = obj
                        result = map_obstacle_to_bev(self.bev, x, y, w, h)
                        if result is not None:
                            new_obstacles.append((result[0], result[1], color_name))
                            new_obs_h.append(float(h))
                        else:
                            # No proyectó (fuera de bev_in_bounds o
                            # cam_to_bev falló) → tratar como lejano
                            far_objects.append((x + w / 2.0, w, h, color_name))
                _t2 = time.perf_counter()
                bev_timing["proj"] = (_t2 - _t1) * 1000.0

                # Diag detección: lo que la CÁMARA ve (crudo) y si proyectó
                # al BEV. Para el verde que "no se ve tras el giro":
                # distinguir "no detectado" (cam=0) de "detectado pero no
                # proyecta" (cam>0, bev=0 -> far_objects).
                _camR = len(positions.get("Red", []))
                _camG = len(positions.get("Green", []))
                if _camR or _camG or new_obstacles or far_objects:
                    print(f"[DET] camR={_camR} camG={_camG} "
                          f"bev={[(round(a),round(b),c[0]) for a,b,c in new_obstacles]} "
                          f"far={[(round(fx),c[0]) for fx,_w,_h,c in far_objects]}",
                          flush=True)

                # ── Memoria rodante: apagada durante el giro para evitar fantasmas ──
                # (Solo con obstáculos BEV reales — el far_hint NO entra aquí)
                if self._is_turning:
                    bev_obstacles = []
                    obstacle_conf = []
                    # FASE 1 mid-turn: detección INSTANTÁNEA solo para
                    # observar/registrar. new_obstacles = proyección BEV
                    # cruda de ESTE frame (sin memoria rodante). NO toca
                    # bev_obstacles, steering, memoria ni el mensaje serial.
                    _mt_ev = self.mid_turn.update(new_obstacles, self._last_heading)
                    if _mt_ev is not None:
                        print(f"[MTURN] CONFIRMADO {_mt_ev.color} lado={_mt_ev.side} "
                              f"d={_mt_ev.dist_mm:.0f}mm bev=({_mt_ev.bev_x:.0f},{_mt_ev.bev_y:.0f}) "
                              f"frames={_mt_ev.frames}/{_mt_ev.window} "
                              f"gyro={_mt_ev.heading_deg:+.0f} wro_bias={_mt_ev.wro_bias:+d} "
                              f"(FASE1: no se actúa)", flush=True)
                else:
                    bev_obstacles = self.memory.update(
                        new_obstacles, dt_s, self._last_heading,
                        estado=self._prev_estado,
                        # steer del frame ANTERIOR (compute() aún no corre
                        # este frame) -> modelo de bicicleta del ancla geom.
                        steer_deg=self.controller._prev_steer_deg,
                    )
                    # Alineado 1:1 con bev_obstacles (mismo orden) --
                    # ver detect_centerline(obstacle_conf=).
                    obstacle_conf = list(self.memory.last_confidences)
                    if self.memory.last_reappear_rejects:
                        print("[MEMREJ] reaparicion imposible, lata nueva: "
                              + " ".join(f"{c[0]} visto({xs},{ys})->det({nx},{ny}) "
                                         f"lado={dx:+d} adelante={-dy:+d}"
                                         for c, xs, ys, nx, ny, dx, dy
                                         in self.memory.last_reappear_rejects),
                              flush=True)
                    # "PASADO y" de _prune: la lata cayó por DETRÁS del eje
                    # (rebase DE FRENTE) o salió por el borde inferior del
                    # BEV. Respaldo del trigger medido, que cubre el rebase
                    # de ÁNGULO. El pulso pasado=1 se finaliza más abajo
                    # (tras detect_centerline), ya OR-eado con el medido.
                    if self.memory.last_passed:
                        self._pasado_hold = max(self._pasado_hold,
                                                C.PASADO_HOLD_FRAMES)
                _t3 = time.perf_counter()
                bev_timing["mem"] = (_t3 - _t2) * 1000.0

                # ── Línea de esquina naranja — ver corner_lines.py. Usa
                # el tracker con persistencia (no detect_lines() cruda)
                # para no "bailar" entre el segmento ocluido y el
                # despejado frame a frame.
                # ds_px: px que la línea se acerca al robot este frame
                # (== el ds_px de la memoria) — para el dead-reckon de
                # near_y si se pierde cerca de la esquina (ORANGE_DR_*).
                _dr_ds_px = ((C.ROBOT_SPEED_MMS * dt_s) / C.MM_PER_PX
                             if dt_s > 0 else 0.0)
                # Mientras dura el giro/maniobra, re-armar el cooldown de
                # la naranja cada frame: el conteo ORANGE_POST_TURN_CD_FRAMES
                # arranca recién al TERMINAR la maniobra (la reversa dura
                # ~3s y al retroceder la cámara re-ve la línea del giro ->
                # latcheaba toda la recta nueva). Ver hold_cooldown().
                if self._is_turning:
                    self.line_tracker.hold_cooldown()
                # INICIO: sin dead-reckon de la naranja. El DR marcha la
                # línea hacia el carro a ROBOT_SPEED_MMS, pero en INICIO el
                # carro pivotea y luego va en REVERSA -> orillas863: la
                # línea estimada rebasó al rojo de la recta (268->362 px
                # mientras el cono se alejaba), lo fijó `beyond` y la cinta
                # provisional soltó la esquiva ya en SIGUIENDO. Las lecturas
                # REALES de la línea siguen clasificando igual.
                line_info = {"Orange": self.line_tracker.update(
                    bev_frame, bev_hsv=bev_hsv, ds_px=_dr_ds_px,
                    in_turn_cooldown=(self._turn_recovery_frames > 0
                                      or self._esp_inicio))}

                # ── Cooldown post-giro: justo al salir de un giro,
                # OrangeLineTracker se reseteó y apenas está re-
                # acumulando lecturas sobre la recta nueva — no confiar
                # en su clasificación todavía (ver TURN_RECOVERY_FRAMES).
                en_recuperacion_giro = self._turn_recovery_frames > 0
                if self._turn_recovery_frames > 0:
                    self._turn_recovery_frames -= 1

                # ── Filtrar obstáculos MÁS ALLÁ de la naranja: no deben
                # esquivarse todavía (están en la siguiente recta, no en
                # la mía) — antes de esto, detect_centerline() los
                # mezclaba con los reales y armaba rutas en zigzag
                # intentando satisfacer el lado de paso de un obstáculo
                # que ni siquiera es alcanzable aún. Sin línea visible,
                # o en el cooldown post-giro, no se filtra nada (todo
                # cuenta como mi recta — mismo comportamiento de siempre).
                bev_obstacles_beyond = []
                orange_info = line_info["Orange"]
                _map_split = None
                if (self._track_map_controls() and not en_recuperacion_giro
                        and not self._is_turning):
                    _map_split = self._map_classify_mine_beyond(
                        bev_obstacles, bev_obstacles_beyond, obstacle_conf)
                if _map_split is not None:
                    bev_obstacles, bev_obstacles_beyond, obstacle_conf = _map_split
                # has_provisional: la cinta ya está en ESTE frame pero la
                # estable aún no se confirma -> classify() solo puede decir
                # "beyond" (ver OrangeLineTracker.classify).
                elif ((orange_info["seen"] or self.line_tracker.has_provisional())
                        and not en_recuperacion_giro):
                    # ── Rescate de cono EXTERIOR pegado a la boca de la
                    # esquina: si la pista va a girar y el cono de la
                    # boca se pasa por el lado CONTRARIO al giro, hay
                    # que rodearlo ANTES de girar -> se trata como "mío"
                    # aunque caiga "más allá" de la naranja. Ver
                    # CORNER_EXTERIOR_PASS_ENABLED en config.py.
                    # turn_dir: override de config si está fijo (el
                    # equipo sabe la dirección al montar la pista), si
                    # no, la latcheada del tracker de visión (ya fija
                    # para cuando se llega a una esquina).
                    _turn_dir_eff = (getattr(C, "CORNER_TURN_DIR_OVERRIDE", None)
                                     or self.turn_dir_tracker.direction)
                    _oy_line = orange_info.get("near_y")
                    _corner_imminent = (
                        _oy_line is not None
                        and _oy_line >= getattr(C, "CORNER_EXT_PASS_NEAR_ORANGE_Y", 285.0))
                    _rescue_fn = None
                    if (getattr(C, "CORNER_EXTERIOR_PASS_ENABLED", False)
                            and _corner_imminent and _turn_dir_eff is not None):
                        _band = float(getattr(C, "CORNER_EXT_PASS_BAND_PX", 70.0))
                        def _rescue_fn(ox, oy, color, _td=_turn_dir_eff,
                                       _nyl=_oy_line, _b=_band):
                            # Exterior = el lado de paso WRO NO coincide
                            # con el giro. Y solo en la boca de la
                            # esquina (no metido en la recta siguiente).
                            if is_interior_pass(_td, color):
                                return False
                            if oy < _nyl - _b:
                                return False
                            self._ext_corner_hold = True
                            return True

                    # Clasificación PEGAJOSA por objeto (ver
                    # ObstacleMemory.classify_and_split()) -- una vez
                    # que un objeto se clasifica "mío" o "más allá",
                    # no se reevalúa frame a frame. Antes se
                    # reclasificaba cada frame contra la posición
                    # actual, y ruido momentáneo en esa posición
                    # (justo cuando la línea se estabiliza) podía
                    # voltear la clasificación de un objeto que ya se
                    # estaba esquivando a medias -- abandonando y
                    # retomando la esquiva a mitad de la maniobra
                    # (confirmado en pista).
                    bev_obstacles, bev_obstacles_beyond, obstacle_conf = (
                        self.memory.classify_and_split(
                            lambda ox, oy: self.line_tracker.classify(
                                ox, oy, C.ROBOT_BEV_X, C.ROBOT_BEV_Y
                            ),
                            rescue_fn=_rescue_fn,
                            allow_pending=not orange_info.get("dead_reckoned", False),
                            provisional=not orange_info["seen"],
                        )
                    )
                    if self.memory.last_prov_blocked:
                        print("[PROVBLOCK] naranja sin confirmar, lata vieja sigue mia: "
                              + " ".join(f"{c}({x:.0f},{y:.0f}) edad={a}"
                                         for x, y, c, a in self.memory.last_prov_blocked),
                              flush=True)

                # ── LOCK al obstáculo primario ── con >=2 conos la
                # centerline no puede satisfacer dos lados de paso
                # opuestos y `obs` oscila (orillas488: +0.47 <-> -0.60).
                # Se fija UNO: el resto sale de bev_obstacles (a beyond,
                # NO entra a la centerline) hasta que ése se pase.
                #  - criterio para FIJAR: bbox de cámara más grande = más
                #    cerca de verdad. El BEV y NO sirve: un cono pasando
                #    la esquina proyecta con y MAYOR (falso "más cerca")
                #    que el verde que tengo en la cara (orillas488:
                #    R y=258 > G y=246, y el verde es el cercano).
                #  - despues se sigue por POSICION (self._lock_xy) para no
                #    brincar si el otro cono gana y/bbox un frame por ruido.
                if len(bev_obstacles) >= 2:
                    def _cam_h(ox, oy):
                        bd, bh = (getattr(C, "LOCK_MATCH_RADIUS_PX", 70.0)) ** 2, 0.0
                        for (nx, ny, _c), nh in zip(new_obstacles, new_obs_h):
                            d = (nx - ox) ** 2 + (ny - oy) ** 2
                            if d < bd:
                                bd, bh = d, nh
                        return bh
                    _lr2 = (getattr(C, "LOCK_MATCH_RADIUS_PX", 70.0)) ** 2
                    _li = None
                    if self._lock_xy is not None:
                        _bd = _lr2
                        for _i, (_ox, _oy, _c) in enumerate(bev_obstacles):
                            _d = (_ox - self._lock_xy[0]) ** 2 + (_oy - self._lock_xy[1]) ** 2
                            if _d < _bd:
                                _bd, _li = _d, _i
                    if _li is None:                       # re-fijar: bbox más grande, empate -> mayor y
                        _li = max(range(len(bev_obstacles)),
                                  key=lambda k: (_cam_h(*bev_obstacles[k][:2]),
                                                 bev_obstacles[k][1]))
                    else:
                        # CAMBIO a un cono MUCHO más cercano y VISIBLE ahora
                        # mismo. Seguir por posición evita el vaivén entre dos
                        # conos parecidos (orillas488: 12px de diferencia), pero
                        # dejaba fuera un cono que reaparece encima del carro
                        # mientras el fijado es uno lejano: orillas828 vueltas
                        # 1 y 2 -- tras RECUPERANDO se fijó un verde a y=124-150
                        # y un frame después reapareció el rojo a y=280-284 (19cm,
                        # 10cm a la derecha); el LOCK siguió con el verde 5-9
                        # frames y el rojo no se esquivó.
                        #  - "mucho más cerca": >= LOCK_SWITCH_CLOSER_PX en y.
                        #  - visible: una detección de ESTE frame a <= VIS_PX (no
                        #    un cono estimado por la memoria que ya va al lado).
                        #  - todavía enfrente: y <= LOCK_SWITCH_MAX_Y.
                        _sw_dy  = getattr(C, "LOCK_SWITCH_CLOSER_PX", 60.0)
                        _sw_vis = getattr(C, "LOCK_SWITCH_VIS_PX", 25.0) ** 2
                        _sw_ym  = getattr(C, "LOCK_SWITCH_MAX_Y", C.ROBOT_BEV_Y - 60.0)
                        _ly = bev_obstacles[_li][1]
                        _sw = [k for k, (_ox, _oy, _c) in enumerate(bev_obstacles)
                               if k != _li and _oy - _ly >= _sw_dy and _oy <= _sw_ym
                               and any((nx - _ox) ** 2 + (ny - _oy) ** 2 <= _sw_vis
                                       for nx, ny, _nc in new_obstacles)]
                        if _sw:
                            _old = bev_obstacles[_li]
                            _li = max(_sw, key=lambda k: bev_obstacles[k][1])
                            print(f"[LOCK] CAMBIO a {bev_obstacles[_li][2]}@"
                                  f"({bev_obstacles[_li][0]:.0f},{bev_obstacles[_li][1]:.0f}): "
                                  f"{bev_obstacles[_li][1] - _old[1]:.0f}px más cerca que "
                                  f"{_old[2]}@({_old[0]:.0f},{_old[1]:.0f})", flush=True)
                    _lock = bev_obstacles[_li]
                    self._lock_xy = (_lock[0], _lock[1])
                    # Solo la primaria cuenta como "esquivada" para el
                    # PASADO por giro de _prune (ver _Obs.was_target).
                    self.memory.mark_target(*_lock)
                    _dropped = [o for j, o in enumerate(bev_obstacles) if j != _li]
                    bev_obstacles_beyond.extend(_dropped)
                    _lkc = obstacle_conf[_li] if _li < len(obstacle_conf) else 1.0
                    bev_obstacles, obstacle_conf = [_lock], [_lkc]
                    print(f"[LOCK] {_lock[2]}@({_lock[0]:.0f},{_lock[1]:.0f}) "
                          f"h={_cam_h(_lock[0], _lock[1]):.0f} difiere {len(_dropped)}",
                          flush=True)
                elif len(bev_obstacles) == 1:
                    self._lock_xy = (bev_obstacles[0][0], bev_obstacles[0][1])
                    self.memory.mark_target(*bev_obstacles[0])
                else:
                    self._lock_xy = None

                if not self._is_turning:
                    bev_obstacles, obstacle_conf = self._map_apply_control_obstacles(
                        bev_obstacles, obstacle_conf)

                # ── Dirección de giro: se infiere UNA SOLA VEZ (con
                # persistencia, ver TurnDirectionTracker) y se queda fija
                # toda la carrera. PRIMARIA: posición lateral de un
                # obstáculo "beyond". La pendiente (line/near_y) es solo
                # confirmación opcional (apagada por defecto). Solo
                # infiere DURANTE la corrida (armado): desarmado el
                # pipeline corre pero el carro no se mueve.
                turn_dir = self.turn_dir_tracker.update(
                    bev_obstacles_beyond if armed else [],
                    C.ROBOT_BEV_X,
                    line=(orange_info.get("line")
                          if (armed and not en_recuperacion_giro) else None),
                    near_y=(orange_info.get("near_y")
                            if (armed and not en_recuperacion_giro) else None),
                )

                # ── Interior/exterior del obstáculo actual (el más
                # cercano en mi recta): si pasar por su lado (regla de
                # color WRO) coincide con hacia dónde va a girar la
                # pista, el giro mismo ya resuelve el paso — no hace
                # falta que el ESP32 siga bloqueando detectarEsquina()
                # por él. Sin dirección confirmada, default seguro:
                # False (bloquea igual que siempre).
                interior = False
                if (getattr(C, "INTERIOR_PASS_ENABLED", True)
                        and bev_obstacles and turn_dir is not None):
                    closest = max(bev_obstacles, key=lambda o: o[1])
                    interior = is_interior_pass(turn_dir, closest[2])

                # ── DEBUG clasificación mine/beyond + turn-dir ──
                # (solo cuando hay algo "beyond" -> el caso que fijó mal
                # la dirección; ver TurnDirectionTracker).
                if bev_obstacles_beyond or self._ext_corner_hold:
                    print(f"[CLASS] mia={[(round(x),round(y),c) for x,y,c in bev_obstacles]} "
                          f"beyond={[(round(x),round(y),c) for x,y,c in bev_obstacles_beyond]} "
                          f"turn_dir={turn_dir} interior={interior} "
                          f"exthold={int(self._ext_corner_hold)} "
                          f"orange_near_y={orange_info.get('near_y')}", flush=True)
                _t4 = time.perf_counter()
                bev_timing["line"] = (_t4 - _t3) * 1000.0

                # Detectar centerline (con obstáculos recordados+nuevos,
                # ya sin los que quedaron más allá de la naranja).
                # Lo que la cámara sigue viendo por arriba de la hoja entra
                # solo aquí: no a la memoria ni al filtro de la naranja.
                path_points = detect_centerline(
                    bev_frame, bev_obstacles, bev_hsv=bev_hsv,
                    obstacle_conf=obstacle_conf, stats_out=cl_stats,
                )
                # La lata que sigue viéndose por arriba de la hoja abre solo
                # el tramo lejano. El punto del lookahead no se mueve.
                if not self._is_turning:
                    above = seen_above_obstacles(self.bev, positions)
                    if bev_obstacles:
                        mine = {c for _, _, c in bev_obstacles}
                        above = [a for a in above if a[2] in mine]
                    if above and path_points:
                        path_points = bend_line_above(path_points, above)
                        print("[LINEUP] " + " ".join(
                            f"{c}@({x:.0f},{y:.0f})" for x, y, c in above),
                              flush=True)
                _t5 = time.perf_counter()
                bev_timing["dc"] = (_t5 - _t4) * 1000.0

                # ── Trigger de RECUPERANDO por ESTADO MEDIDO ──────────
                # Sustituto del ancla "geom". Usa cl_stats["weights"] (lo
                # que el planner de verdad decidió esquivar este frame) +
                # el heading real -> sin dead-reckoning que derive.
                # SOLO armado: desarmado no hay heading (ACK=None) ni
                # movimiento, y si se armaba ahí quedaba sin heading_ref
                # para siempre (bug orillas414). Apagado también en el
                # giro y su cooldown.
                # `_prev_estado != "G"`: en cuanto el ESP32 entra a
                # GIRANDO resetea su anguloGyro a 0 -> el heading que
                # llega en el ACK SALTA ~90° de golpe -> heading_err se
                # dispara y measured firaba a herr=+89 en pleno giro
                # (orillas417). Con esto el trigger se apaga al PRIMER
                # est=G, sin esperar la confirmación de 2 frames.
                if (armed and not self._is_turning and self._prev_estado != "G"
                        and not en_recuperacion_giro):
                    _oy_cs = orange_info.get("near_y")
                    _corner_soon_meas = (
                        orange_info.get("seen") and _oy_cs is not None
                        and _oy_cs >= getattr(
                            C, "RECUP_SUPPRESS_NEAR_ORANGE_Y", 285.0))
                    measured_pass = self._measured_recup_trigger(
                        cl_stats, bev_obstacles,
                        corner_soon=bool(_corner_soon_meas),
                        fresh_color=bool(_camR or _camG),
                    )
                    if measured_pass:
                        self._pasado_hold = max(self._pasado_hold,
                                                C.PASADO_HOLD_FRAMES)
                        self._pasado_from_measured = True
                        # Olvida el cono YA: la centerline del PRÓXIMO
                        # frame deja de rodearlo -> el carro sale del
                        # arco en vez de seguir clavando el volante
                        # (comportamiento del modo "angle" viejo, que
                        # era lo que mantenía la esquiva fluida).
                        self.memory.forget_color_obstacles()
                        # Si el rebase medido fue CERCA de una esquina,
                        # retrasar el giro: tras RECUPERANDO el ESP32
                        # disparaba detectarEsquina() ~0.5s antes del
                        # punto real (reporte del usuario, orillas429).
                        # _turn_delay_frames fuerza prio=1 (giro
                        # bloqueado) esos frames, DESPUÉS de que
                        # RECUPERANDO termina, para que el carro avance
                        # recto hasta el punto correcto.
                        _ny_m = orange_info.get("near_y")
                        if (_ny_m is not None and _ny_m >=
                                getattr(C, "RECUP_SUPPRESS_NEAR_ORANGE_Y", 285.0)):
                            self._turn_delay_frames = getattr(
                                C, "RECUP_CORNER_TURN_DELAY_FRAMES", 8)

                feet = list(new_obstacles)
                if self.bev.is_calibrated and not self._is_turning:
                    feet.extend(seen_above_obstacles(self.bev, positions))
                self.digital.update(serial_ack, feet, line_info.get("Orange"))
                if len(path_points) >= C.MIN_PATH_PTS:
                    if getattr(C, "DIGITAL_MAP_STEER", False) and not self._is_turning:
                        mapped = self.digital.line_bev()
                        if len(mapped) >= C.MIN_PATH_PTS:
                            path_points = mapped
                    # Lookahead ADAPTATIVO: se acorta (~45 px) cuando hay
                    # una lata cerca -> la geometría pure-pursuit exige un
                    # steer más cerrado para el mismo path -> esquiva de
                    # inmediato en vez de entrar largo y comerse el cono.
                    # Sin obstáculos cerca vuelve a LOOKAHEAD_MAX_PX
                    # (trayectoria suave en recta/curva normal).
                    lookahead_eff = self.controller.adaptive_lookahead(
                        bev_obstacles, C.ROBOT_BEV_X, C.ROBOT_BEV_Y
                    )
                    steer_deg, lookahead_pt = self.controller.compute(
                        path_points, C.ROBOT_BEV_X, C.ROBOT_BEV_Y,
                        lookahead_px=lookahead_eff,
                        bev_obstacles=bev_obstacles,
                    )
                    pp_active = True

                # ── Hint direccional: solo suma al steer, nunca a prio/mem ──
                far_hint_deg = 0.0
                if C.FAR_HINT_ENABLED:
                    bev_obstacle_active = len(bev_obstacles) > 0
                    if not bev_obstacle_active:
                        far_hint_deg = self.far_hint.compute(far_objects)
                        if pp_active:
                            steer_deg = max(-C.MAX_STEER_DEG,
                                             min(C.MAX_STEER_DEG,
                                                 steer_deg + far_hint_deg))
                    else:
                        # Hay un obstáculo BEV real activo → el hint no
                        # debe competir con la esquiva geométrica precisa.
                        self.far_hint.reset_all()

                # ── Cono EXTERIOR de esquina: ir "completamente
                # vertical". La centerline, con la esquina abierta
                # adelante, puede curvar hacia ella y PP la seguiría ->
                # el carro arquearía hacia el giro antes de rebasar el
                # cono. Capar el steer mantiene el rumbo recto; la
                # esquiva de un cono YA exterior es suave y cabe dentro
                # del cap. 0 = sin cap.
                if self._ext_corner_hold and pp_active:
                    _cap = float(getattr(C, "CORNER_EXT_PASS_MAX_STEER_DEG", 0.0))
                    if _cap > 0.0:
                        steer_deg = max(-_cap, min(_cap, steer_deg))

                # ── Cap del steer de APROXIMACIÓN a un cono de color ──
                # El verde satura el steer desde lejos (pasa por su
                # izquierda -> ~2x el desplazamiento del rojo) y pivotea.
                # Mientras el cono esté LEJOS (y < _Y) se capa a lo que el
                # rojo ya hace bien -> arco, no pivote. Al acercarse se
                # suelta. NO si ya hay un cono CERCA. Ver config.
                _capd = float(getattr(
                    C, "CENTERLINE_COLOR_APPROACH_MAX_STEER_DEG", 0.0))
                if _capd > 0.0 and pp_active and bev_obstacles:
                    _capy = float(getattr(
                        C, "CENTERLINE_COLOR_APPROACH_Y", 300.0))
                    _far_color = any(
                        c in ("Red", "Green") and oy < _capy
                        for (ox, oy, c) in bev_obstacles)
                    _near_color = any(
                        c in ("Red", "Green") and oy >= _capy
                        for (ox, oy, c) in bev_obstacles)
                    if _far_color and not _near_color:
                        steer_deg = max(-_capd, min(_capd, steer_deg))
                        # baseline del slew = valor capado -> al soltar
                        # el cap (cono cerca) no hay salto, sigue rampa.
                        self.controller._prev_steer_deg = steer_deg

                if pp_active:
                    obs_norm = self.controller.normalize(steer_deg)
                bev_timing["ctrl"] = (time.perf_counter() - _t5) * 1000.0

            except Exception as e:
                import traceback
                print(f"[ERROR] {e}", flush=True)
                traceback.print_exc()
        else:
            self.far_hint.reset_all()

        # ── Finalizar el pulso pasado=1 ──────────────────────────────
        # OR de las dos vías: memory.last_passed (rebase de FRENTE, "PASADO
        # y" / borde) ya subió _pasado_hold arriba; el trigger MEDIDO
        # (rebase de ÁNGULO) lo subió tras detect_centerline. Aquí solo se
        # emite el hold y se decrementa -- una sola vez por frame, corra o
        # no el pipeline BEV.
        # SUPRIMIR si la esquina es inminente: con la línea naranja cerca
        # (near_y grande), el ESP32 está por girar. Un pasado=1 aquí lo
        # mete a RECUPERANDO -> en ese estado NO evalúa detectarEsquina()
        # -> el giro entra ~0.5s tarde y el carro se lleva el cono de la
        # recta siguiente (orillas420/421).
        # EXCEPCIÓN 2026-08-31 (RECUP_SUPPRESS_KEEP_MEASURED): si el
        # pulso vino del trigger MEDIDO (esquiva de ÁNGULO real, cono
        # rojo/verde en la misma recta cerca de la esquina), el chasis
        # quedó chueco y SÍ hace falta RECUPERANDO -> sin él, el carro
        # ladeado dispara un FALSO detectarEsquina() (ultrasónico lateral
        # lee "sin pared" por el yaw) y gira ~90° encima del ladeo
        # (reporte del usuario). Solo se suprime el pasado ESPURIO
        # (memory.last_passed / BEHIND_PAD head-on, sin esquiva de
        # ángulo), que era el caso de orillas420/421.
        _oy = line_info["Orange"].get("near_y")
        _corner_soon = (line_info["Orange"].get("seen")
                        and _oy is not None
                        and _oy >= getattr(C, "RECUP_SUPPRESS_NEAR_ORANGE_Y", 300.0))
        _keep_measured = (self._pasado_from_measured
                          and getattr(C, "RECUP_SUPPRESS_KEEP_MEASURED", True))
        if _corner_soon and self._pasado_hold > 0 and not _keep_measured:
            self._pasado_hold = 0
            self._pasado_from_measured = False
            self._last_recup_reason = f"pasado suprimido (esquina, oy={_oy:.0f})"
        elif _corner_soon and _keep_measured and self._pasado_hold > 0:
            self._last_recup_reason = (
                f"pasado(medido) NO suprimido cerca esquina (oy={_oy:.0f})")
        if armed and self._track_map_controls() and self._map_try_pasado():
            self._pasado_hold = max(self._pasado_hold, C.PASADO_HOLD_FRAMES)
        # Verde de la recta siguiente: no soltar "pasado" todavía. Si no,
        # RECUPERANDO endereza y el giro rápido corta por debajo del verde.
        if self.digital.hold_pasado() and self._pasado_hold > 0:
            self._pasado_hold = 0
            self._pasado_from_measured = False
        pasado = self._pasado_hold > 0
        if self._pasado_hold > 0:
            self._pasado_hold -= 1
            if self._pasado_hold == 0:
                self._pasado_from_measured = False

        # Sin línea válida → recto (obs=0).
        state = "pp_follow" if pp_active else "no_path"

        t_bev = time.perf_counter()
        self._timing_ms["bev"] = (t_bev - t_vis) * 1000.0

        # ── Construir y (si armado) enviar mensaje serial ────────────
        # ── Bloqueo de giro post-rescate de cono exterior ────────────
        # Mientras se rescata (exthold) se refresca el contador; al
        # soltarse, se mantiene prio=1 CORNER_EXT_PASS_TURN_BLOCK_FRAMES
        # frames más -> el ESP32 no gira hasta que el cono está de
        # verdad atrás, no por el arrastre de memoria que lo poda como
        # "PASADO" antes de tiempo.
        if self._ext_corner_hold:
            self._ext_corner_block = getattr(
                C, "CORNER_EXT_PASS_TURN_BLOCK_FRAMES", 10)
        elif self._ext_corner_block > 0:
            self._ext_corner_block -= 1

        # ── Retraso de giro post-RECUPERANDO cerca de esquina ────────
        # Solo consume el contador cuando RECUPERANDO YA terminó (no
        # `pasado` ni est=R) — forzar prio=1 durante RECUPERANDO lo
        # abortaría (el ESP32 sale a SIGUIENDO con piPriority). Una vez
        # enderezado, prio=1 estos frames bloquea detectarEsquina() ->
        # el carro avanza recto hasta el punto real del giro.
        _turn_hold = False
        if (self._turn_delay_frames > 0
                and not pasado and self._prev_estado != "R"):
            self._turn_delay_frames -= 1
            _turn_hold = True
        _turn_block = ((self._ext_corner_block > 0) or _turn_hold
                       or self.digital.blocks_turn() or self.digital.needs_line())

        serial_msg = self._build_serial_message(
            obs_norm, state, len(bev_obstacles), pasado, interior,
            turn_block=_turn_block,
        )
        serial_msg = serial_msg + self._map_gr_gf_suffix()
        if getattr(C, "CORNER_HINT_TO_ESP", False):
            serial_msg += f",es={int(self._corner_hint(line_info, now))}"
        serial_msg += f",gv={int(self._green_ahead(new_obstacles))}"
        if armed:
            self.serial_link.send_line(serial_msg)
            serial_ack = self.serial_link.try_readline()
        else:
            serial_ack = None   # desarmado: pipeline corre, carro quieto
        self._prev_ack = serial_ack

        heading = _parse_heading(serial_ack)
        if heading is not None:
            self._last_heading = heading

        # Dirección de giro AUTORITATIVA del ESP32 (dir= en el ACK, L/R
        # desde su 1er GIRANDO). Cubre esquinas 2-12; corrige cualquier
        # estimación de visión.
        _esp_dir = _parse_direccion(serial_ack)
        if _esp_dir is not None:
            self.turn_dir_tracker.set_esp_direction(_esp_dir)

        _inicio_now = _parse_inicio(serial_ack)
        if _inicio_now is not None:
            self._esp_inicio = _inicio_now

        _tc_now = _parse_tc(serial_ack)
        if _tc_now is not None:
            self._tc = _tc_now

        _tpr_now = _parse_tpr(serial_ack)
        if _tpr_now is not None and _tpr_now > 0:
            self._tpr = _tpr_now

        _pb_now = _parse_pb(serial_ack)
        if _pb_now is not None:
            self._park_buscando = _pb_now

        estado_now = _parse_estado(serial_ack)
        if estado_now is not None:
            # Debounce: un est=G ESPURIO (ACK con ruido, "est=G fantasma
            # tras verde") ya no dispara el wipe de memoria a media
            # esquiva. Un giro real manda est=G muchos frames seguidos;
            # se exigen TURN_EST_G_CONFIRM_FRAMES consecutivos.
            if estado_now == "G":
                self._g_streak += 1
                # Desarmar la esquiva medida al PRIMER est=G (sin esperar
                # confirmación): el ESP32 ya reseteó anguloGyro -> el
                # heading del ACK saltó ~90° -> heading_err es basura y
                # measured firaría espurio (orillas417 herr=+89).
                self._dodge_armed = False
                self._recup_can_arm = True
                self._recup_clear_count = 0
                self._heading_ref = None
            else:
                self._g_streak = 0
            g_confirmed = self._g_streak >= C.TURN_EST_G_CONFIRM_FRAMES

            if g_confirmed and not self._is_turning:
                # Empieza el giro físico -> vaciar YA y apagar la memoria.
                self.memory.reset()
                self.line_tracker.reset()   # la línea ya quedó atrás, no aplica a la recta nueva
                self.mid_turn.reset()       # FASE 1: historia limpia para este giro
                self._is_turning   = True
                self._turn_start_t = now
                # La esquiva (si había) muere con el giro.
                self._dodge_armed = False
                self._recup_can_arm = True
                self._recup_clear_count = 0
                self._heading_ref = None
                print(f"[MEM] Giro detectado (est=G x{self._g_streak}) — "
                      f"memoria de obstáculos desactivada.", flush=True)
            elif estado_now != "G" and self._is_turning:
                # Terminó el giro -> la memoria ya está vacía (no se tocó), arranca
                # limpia con las detecciones frescas de este frame.
                self._is_turning = False
                self._turn_recovery_frames = C.TURN_RECOVERY_FRAMES
                self._heading_ref = None   # ref fresca para la esquiva de la recta nueva
                print("[MEM] Giro terminado — memoria de obstáculos reactivada.", flush=True)
                _mt_last = self.mid_turn.last_sighting
                print(f"[MTURN] fin de giro — {'nada confirmado' if _mt_last is None else _mt_last}",
                      flush=True)
            self._prev_estado = estado_now

        # Red de seguridad: si por un ACK perdido/atorado el ESP32 nunca
        # reporta salir de "G", no dejar la memoria apagada para siempre.
        if (self._is_turning and self._turn_start_t is not None
                and (now - self._turn_start_t) > C.TURN_TIMEOUT_S):
            self._is_turning = False
            print("[MEM] Timeout de giro — memoria de obstáculos reactivada por seguridad.", flush=True)

        log_line = ("TX: " if armed else "TX(desarmado): ") + serial_msg
        if serial_ack:
            log_line += f" | RX: {serial_ack}"
        print(log_line, flush=True)

        print(f"[LINEA] Orange={ {k: v for k, v in line_info['Orange'].items() if not k.startswith('_')} }",
              flush=True)
        _ln = line_info["Orange"].get("line")
        _vy = None if _ln is None else round(float(_ln[1]))
        print(f"[DIR] fija={self.turn_dir_tracker.direction} "
              f"ovr={getattr(C, 'CORNER_TURN_DIR_OVERRIDE', None)} "
              f"vy={_vy} interior={interior} "
              f"ext_corner_hold={int(self._ext_corner_hold)} "
              f"turn_block={self._ext_corner_block} "
              f"turn_delay={self._turn_delay_frames}", flush=True)
        # Vuelca el estado interno de la memoria rodante a stdout (antes
        # solo iba al HUD de pantalla vía _annotate). Permite medir en
        # journalctl cuántos frames se arrastra un obstáculo (falta=+Npx
        # = aún enfrente) antes de soltarse, y si `pasado` salió por
        # cruce real de y (PASADO) o por decaimiento de confianza
        # (BAJA_CONF, que NO manda recuperando).
        print(f"[MEMDBG] closest={self.memory.debug_closest()} "
              f"prune={self.memory.last_prune_reason} "
              f"all={self.memory.debug_all()}", flush=True)
        print(f"[RECUP] {self._last_recup_reason} "
              f"armed={int(self._dodge_armed)} clr={self._recup_clear_count} "
              f"href={'-' if self._heading_ref is None else round(self._heading_ref)} "
              f"pasado={int(pasado)} measured={int(measured_pass)}", flush=True)
        # Diag esquiva: steer final (deg) que se manda, lookahead usado,
        # y n_obstáculos. Para ver si el carro ARQUEA (steer moderado, la
        # y del cono avanza en [MEMDBG]) o PIVOTEA (steer al tope, y clavada).
        print(f"[PPDIAG] steer={steer_deg:+.1f}deg obs={obs_norm:+.3f} "
              f"lka={lookahead_eff:.0f} nobs={len(bev_obstacles)}", flush=True)
        # FASE 1 mid-turn: estado por frame SOLO mientras dura el giro.
        if self._is_turning:
            print(f"[MTURN] {self.mid_turn.status_str()}", flush=True)

        t_ser = time.perf_counter()
        self._timing_ms["ser"] = (t_ser - t_bev) * 1000.0

        self._update_map_after_ack(
            serial_ack, now, armed, new_obstacles, dt_s)

        shown = bev_frame
        shift = (0, 0)
        self._dig_img = self.digital.render()
        warp_wide = getattr(self.bev, "warp_wide", None)
        if warp_wide is not None and processed_frame is not None:
            shown = warp_wide(processed_frame)
            dx, dy = self.bev.wide_shift
            shift = (dx, dy)
            path_points = [(x + dx, y + dy) for x, y in path_points]
            if lookahead_pt is not None:
                lookahead_pt = (lookahead_pt[0] + dx, lookahead_pt[1] + dy)
            bev_obstacles = [(x + dx, y + dy, c) for x, y, c in bev_obstacles]
            bev_obstacles_beyond = [(x + dx, y + dy, c) for x, y, c in bev_obstacles_beyond]
            line_info = _shift_line_info(line_info, dx, dy)
        return FrameResult(
            processed_frame=processed_frame,
            bev_frame=shown,
            positions=positions,
            steer_deg=steer_deg,
            obs_norm=obs_norm,
            pp_active=pp_active,
            path_points=path_points,
            lookahead_pt=lookahead_pt,
            bev_obstacles=bev_obstacles,
            bev_obstacles_beyond=bev_obstacles_beyond,
            line_info=line_info,
            serial_msg=serial_msg,
            serial_ack=serial_ack,
            pasado=pasado,
            measured_pass=measured_pass,
            state=state,
            fps=self._fps,
            timing_ms=self._timing_ms,
            bev_timing=bev_timing,
            new_obstacles=new_obstacles,
            new_obs_h=new_obs_h,
            map_pose=self._map_field_pose,
            map_status=dict(self._map_status) if self._map_status else None,
            map_geometry=self._map_geometry,
            map_signmap=self._map_signmap,
            map_localizer=self._map_localizer,
            bev_shift=shift,
            dig_map=self._dig_img,
        )

    # ── Loop principal ────────────────────────────────────────────────────────

    def run(self, on_ready=None, should_start=None, should_record=None,
            on_camera_lost=None):
        # BLOQUEA hasta que el UART está listo (antes era no-op y el loop
        # arrancaba mandando V2 al vacío ~1.5 s -> el ESP32 rodaba en su
        # fallback wall-PID = "el carro avanza y no le entran datos").
        if self.serial_link.open():
            print("[SERIAL] UART listo.", flush=True)
        self._start_capture()

        if C.CAM_CHECK_ENABLED:
            # Antes de LISTO (el LED lo prende should_start(), que no se llama
            # hasta el loop de abajo): la cámara tiene que dar imagen real.
            print("[CAM-CHECK] Verificando que la cámara entregue imagen...", flush=True)
            self._camera_gate()
        else:
            # Warmup de cámara (esperar exposición automática estabilizada).
            print(f"[INFO] Calentando cámara ({self.cfg.warmup_frames} frames)...", flush=True)
            warmed = 0
            while warmed < self.cfg.warmup_frames:
                ret, _ = self._read_frame()
                if ret:
                    warmed += 1
                else:
                    time.sleep(0.01)
        print("[INFO] Cámara estabilizada.", flush=True)
        _cam_last_id  = -1
        _cam_negros   = 0

        # ── ARRANQUE EN CALIENTE ──────────────────────────────────────────────
        # El loop de abajo corre YA (visión, detección, centerline, memoria,
        # grabación) pero DESARMADO: no manda comandos al ESP32 hasta que
        # should_start() da True (botón + start-delay). Así, al soltar GO:
        #   • la lata de enfrente ya está detectada y en memoria
        #   • el steer ya rampó a su valor (slew) -> esquiva desde el frame 1
        #   • el .avi ya lleva ~1 s grabando
        # en vez de arrancar la cámara/pipeline en frío con el carro encima.
        armed = False
        ready_ack = False

        print("[INFO] Pure Pursuit + memoria runtime iniciado (desarmado). ESC para detener.", flush=True)

        self._last_fps_time = time.perf_counter()
        self._fps_count     = 0
        self._fps           = 0.0
        self._t_prev_end    = time.perf_counter()   # para medir tiempo de captura entre iteraciones
        self._timing_ms     = {"cap": 0.0, "vis": 0.0, "bev": 0.0, "ser": 0.0, "disp": 0.0, "rec": 0.0}

        try:
            while True:
                ret, frame = self._read_frame()
                if not ret:
                    time.sleep(0.01)
                    continue

                # ── Vigilancia de cámara mientras espera el botón ────────────
                # Solo DESARMADO: sin frames nuevos o frames negros -> LED
                # apagado, botón pendiente cancelado y se repite el chequeo.
                if C.CAM_CHECK_ENABLED and not armed and self.frame_grabber is not None:
                    _fid, _age = self.frame_grabber.freshness()
                    _caida = None
                    if _age > C.CAM_STALE_S:
                        _caida = f"sin frames nuevos hace {_age:.1f}s"
                    elif _fid != _cam_last_id:
                        _cam_last_id = _fid
                        _cam_negros = _cam_negros + 1 if self._frame_negro(frame) else 0
                        if _cam_negros >= C.CAM_BLACK_FRAMES:
                            _caida = f"{_cam_negros} frames negros seguidos"
                    if _caida is not None:
                        print(f"[CAM-CHECK] Cámara caída antes del GO: {_caida}.", flush=True)
                        if on_camera_lost is not None:
                            on_camera_lost()
                        self._camera_reopen_and_gate()
                        _cam_last_id, _cam_negros = -1, 0
                        continue

                self.loop_count += 1
                if self.loop_count % self.cfg.process_every_n != 0:
                    time.sleep(0.001)
                    continue

                # ── Armado: botón + start-delay ──────────────────────────────
                if not armed and (should_start is None or should_start()):
                    ready_ack = self.arm()
                    armed = True
                    if on_ready is not None:
                        on_ready()

                result = self.process_frame(frame, time.perf_counter(), armed)
                t_ser = time.perf_counter()

                # ── Display ──────────────────────────────────────────────────
                # HUD (texto + BEV debug + resize + hstack): solo se CONSUME en los frames que de
                # verdad se escriben -- vista VNC (cada cam_frame_every_n), .avi (cada record_every_n)
                # o ventana. Armarlo en TODOS costaba ~14 ms/frame (~18% del loop a 14 fps). Los
                # 'want_*' espian los contadores SIN incrementarlos (_maybe_record/_write_cam_frame
                # los incrementan igual que antes), asi que el frame que se escribe es el mismo.
                _do_rec = (should_record is None or should_record())
                _want_cam = ((self.cam_frame_count + 1) % max(1, self.cfg.cam_frame_every_n)) == 0
                _want_rec = (_do_rec and self.cfg.record_orillas
                             and ((self.record_count + 1) % max(1, self.cfg.record_every_n)) == 0)
                combined = None
                if self.cfg.show_window or _want_cam or _want_rec:
                    self._annotate(result.processed_frame, result.steer_deg, result.obs_norm,
                                result.pp_active, len(result.path_points), result.positions,
                                result.serial_msg, result.fps, len(result.bev_obstacles),
                                self.memory.last_prune_reason, self._timing_ms, result.bev_timing)

                    # HUD del rosa — DESPUÉS de todo el pipeline (el BEV ya se
                    # calculó arriba con el frame limpio), así que dibujar acá no
                    # puede tocar la visión. Muestra el HUD cuando está DESARMADO
                    # o cuando está buscando el cajón en la recta final.
                    if not armed or self._park_buscando or self._tc >= self._tpr:
                        try:
                            _pr_h, _pm_h = _park_pink(result.processed_frame)
                            self._draw_park_pink(result.processed_frame, _pr_h, _pm_h)
                        except Exception:
                            pass

                    if result.bev_frame is not None:
                        bev_debug = draw_bev_debug(
                            result.bev_frame, result.path_points, result.lookahead_pt,
                            result.bev_obstacles, result.steer_deg, result.pp_active,
                            line_info=result.line_info,
                            bev_obstacles_beyond=result.bev_obstacles_beyond,
                        )
                        bev_h = result.processed_frame.shape[0]
                        bev_small = cv2.resize(bev_debug, (bev_h, bev_h))
                        combined = np.hstack([result.processed_frame, bev_small])
                    else:
                        combined = result.processed_frame
                    if (getattr(C, "MAP_VIEW_IN_HUD", False)
                            and self._track_map_active()
                            and combined is not None):
                        map_h = result.processed_frame.shape[0]
                        map_panel = self.render_map_panel(map_h)
                        combined = np.hstack([combined, map_panel])

                t_disp = time.perf_counter()
                if combined is not None:            # 'disp' = ultimo costo REAL del HUD armado
                    self._timing_ms["disp"] = (t_disp - t_ser) * 1000.0

                if self.cfg.show_window:
                    cv2.imshow("WRO Pure Pursuit + Memoria", combined)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

                # Grabación al .avi: solo DESDE que se picó el botón (incluye el
                # start-delay + la run). El pipeline desarmado corre antes pero
                # NO se graba -> el archivo no acumula el rato de "esperando
                # botón". El _write_cam_frame (vista VNC) sí corre siempre.
                if _do_rec:
                    self._maybe_record(combined, result.fps, bev=result.bev_frame)
                self._write_cam_frame(combined)   # ← ahora manda cámara + BEV/ruta

                self._t_prev_end = time.perf_counter()
                self._timing_ms["rec"] = (self._t_prev_end - t_disp) * 1000.0

        finally:
            if self.frame_grabber is not None:
                self.frame_grabber.stop()
            self.vision.cap.release()
            if self.video_writer is not None:
                self.video_writer.stop()
            if self.bev_recorder is not None:
                self.bev_recorder.stop()
            cv2.destroyAllWindows()
            self.serial_link.close()


# ─────────────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Pure Pursuit + memoria runtime — WRO Future Engineers"
    )
    p.add_argument("--cam-index",        type=int,   default=C.CAM_INDEX)
    p.add_argument("--serial-port",      type=str,   default=C.SERIAL_PORT)
    p.add_argument("--baudrate",         type=int,   default=C.BAUDRATE)
    p.add_argument("--process-every",    type=int,   default=C.PROCESS_EVERY)
    p.add_argument("--calib-path",       type=str,   default=None,
                   help="Ruta a bev_calib.npz (default: pure_pursuit/bev_calib.npz)")
    p.add_argument("--no-window",        action="store_true")
    p.add_argument("--threaded-capture", action="store_true", default=True)
    p.add_argument("--no-threaded-capture", action="store_true")
    p.add_argument("--record-orillas",   action="store_true")
    p.add_argument("--record-output",    type=str,   default=None)
    p.add_argument("--record-every",     type=int,   default=6)
    p.add_argument("--record-fps",       type=float, default=5.0)
    p.add_argument("--cam-frame-every",  type=int,   default=2,
                   help="Cada cuántos frames procesados se actualiza la captura para vista remota (VNC)")
    p.add_argument("--start-delay",      type=float, default=1.0,
                   help="Segundos de espera entre el botón y el arranque (quitar la mano). 0 = sin espera")
    return p.parse_args()


def main():
    # ── SIGTERM/SIGINT -> salida limpia ──────────────────────────────────────
    # systemctl stop/restart manda SIGTERM; por defecto Python termina el
    # proceso SIN correr los bloques finally -> el video (y el serial) quedan
    # sin cerrar. Convertir la señal en KeyboardInterrupt hace que el
    # try/finally de run() sí corra y el AsyncVideoWriter finalice el archivo.
    import signal as _signal

    def _term(_signum, _frame):
        raise KeyboardInterrupt

    _signal.signal(_signal.SIGTERM, _term)
    _signal.signal(_signal.SIGINT, _term)

    # ── GPIO: solo SETUP aquí. El botón se sondea DENTRO del loop (should_start)
    # con el pipeline ya corriendo en caliente -> al soltar GO la esquiva ya
    # está calculada, en vez de "empieza a inicializar ~2 s mientras el carro
    # ya está rodando y se come 10-15 cm antes del primer steer".
    _gpio = None
    try:
        import RPi.GPIO as GPIO
        _gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(27, GPIO.OUT)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(27, GPIO.LOW)   # LED APAGADO mientras inicializa (cámara, warmup)
        print("[GPIO] Inicializando... (LED apagado)", flush=True)
    except ImportError:
        print("[GPIO] RPi.GPIO no disponible, omitiendo espera de botón.", flush=True)

    _ss = {"btn_t": None, "announced": False}

    def should_start():
        """Predicado NO bloqueante: True cuando el botón fue presionado y pasó
        el start-delay. Mientras devuelve False, run() sigue corriendo TODO el
        pipeline (visión, detección, centerline, grabación) pero SIN mandar
        comandos al ESP32 -> al soltar GO la esquiva ya está calculada y el
        video ya está grabando, en vez de arrancar la cámara/detección en frío
        con el carro ya encima del obstáculo."""
        if _gpio is None:
            return True
        if not _ss["announced"]:
            # Pipeline ya corriendo en caliente -> el carro puede arrancar en
            # cuanto se pique el botón. LED ENCENDIDO = "listo, pícale".
            _gpio.output(27, _gpio.HIGH)
            print("[GPIO] LISTO — LED encendido. Esperando botón GPIO17...", flush=True)
            _ss["announced"] = True
        if _ss["btn_t"] is None:
            if _gpio.input(17) == _gpio.HIGH:
                _ss["btn_t"] = time.time()
                d = max(0.0, float(getattr(args, "start_delay", 1.0)))
                print(f"[GPIO] Botón detectado. GO en {d:.1f}s (quita la mano)...", flush=True)
            return False
        d = max(0.0, float(getattr(args, "start_delay", 1.0)))
        return (time.time() - _ss["btn_t"]) >= d

    def led_on():
        # Se llama en GO (armado). El LED ya está encendido desde "LISTO";
        # esto solo lo asegura por si acaso.
        if _gpio is not None:
            _gpio.output(27, _gpio.HIGH)
            print("[GPIO] Armado — corriendo.", flush=True)

    def should_record():
        # Empezar a grabar el .avi cuando se pica el botón (antes del
        # start-delay), no durante el idle de "esperando botón".
        return _ss["btn_t"] is not None

    def on_camera_lost():
        # La cámara se cayó ANTES del GO: LED apagado (ya no está listo), se
        # cancela un botón pendiente y should_start() vuelve a anunciar LISTO
        # (y prender el LED) solo cuando el chequeo de cámara pase otra vez.
        _ss["announced"] = False
        _ss["btn_t"] = None
        if _gpio is not None:
            _gpio.output(27, _gpio.LOW)
            print("[GPIO] LED apagado: cámara sin imagen.", flush=True)

    args = parse_args()

    threaded = True
    if args.no_threaded_capture:
        threaded = False

    cfg = PPConfig(
        cam_index        = args.cam_index,
        serial_port      = args.serial_port,
        baudrate         = args.baudrate,
        process_every_n  = max(1, args.process_every),
        threaded_capture = threaded,
        show_window      = not args.no_window,
        record_orillas   = bool(args.record_orillas),
        record_output    = args.record_output,
        record_every_n   = max(1, args.record_every),
        record_fps       = max(1.0, args.record_fps),
        cam_frame_every_n = max(1, args.cam_frame_every),
        calib_path       = Path(args.calib_path) if args.calib_path else None,
    )

    runtime = PPRuntime(cfg)   # abre la cámara AQUÍ, antes del botón
    try:
        runtime.run(on_ready=led_on, should_start=should_start,
                    should_record=should_record, on_camera_lost=on_camera_lost)
    except KeyboardInterrupt:
        # SIGTERM (systemctl stop/restart) o Ctrl-C: run() ya corrió su
        # finally (cerró video/serial). Salir sin escupir traceback.
        print("[INFO] Detenido (SIGTERM/SIGINT).", flush=True)


if __name__ == "__main__":
    main()
