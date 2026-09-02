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
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# ── Importar infraestructura compartida de cam/ ──────────────────────────────
_CAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import Vision
from wro_runtime import (
    ThreadedFrameGrabber,
    AsyncVideoWriter,
    SerialLink,
    resolve_output_path,
    CAM_FRAME_PATH,
)

from .bev import BEVTransformer
from .centerline import detect_centerline, map_obstacle_to_bev, draw_bev_debug
from .corner_lines import OrangeLineTracker, TurnDirectionTracker, is_interior_pass
from .controller import PurePursuitController
from .obstacle_memory import ObstacleMemory
from .far_hint import FarHintManager
from .mid_turn import MidTurnObstacleDetector
from . import config as C


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
    if val == "C":          # CRUCERO (ESP): la Pi lo trata igual que SIGUIENDO
        return "S"
    return val if val in ("G", "R", "S") else None

def _parse_direccion(ack: str) -> str | None:
    """dir= del ACK:V2 del ESP32: 'L'/'R' desde su 1er GIRANDO, '?' antes."""
    if not ack:
        return None
    idx = ack.find("dir=")
    if idx < 0:
        return None
    val = ack[idx + 4: idx + 5]
    return val if val in ("L", "R") else None


class PPRuntime:
    """
    Runtime Pure Pursuit con memoria de obstáculos.
    Infraestructura idéntica a runtime.py / wro_runtime.py.
    """

    def __init__(self, cfg: PPConfig):
        self.cfg = cfg

        # Visión
        self.vision     = Vision(cfg.cam_index)
        self.bev        = BEVTransformer(cfg.calib_path)
        self.controller = PurePursuitController()
        self.memory     = ObstacleMemory()
        self.far_hint   = FarHintManager()
        self.line_tracker = OrangeLineTracker()
        self.turn_dir_tracker = TurnDirectionTracker()
        # FASE 1 mid-turn: solo observa/registra (ver mid_turn.py). No actúa.
        self.mid_turn   = MidTurnObstacleDetector()

        # Estado de la memoria rodante
        self._last_heading: float | None = None
        self._lock_xy: tuple[float, float] | None = None  # cono primario fijado (>=2 conos)
        self._last_update_t: float | None = None
        self._prev_estado: str | None = None
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
        self._recup_lock_xy: tuple[float, float] | None = None  # cono que ARMÓ la
        #   esquiva en curso. El chequeo "¿ya lo rodeé?" se hace SOLO contra este
        #   cono, buscándolo en mia+beyond -> si su clasificación cambia a beyond
        #   a media esquiva (2º cono corrompe la naranja, orillas496) NO se pierde
        #   ni el trigger lo da por rodeado. "Me quedo con el cono que esquivo."
        self._recup_can_arm: bool = True         # gate: solo re-arma tras un peso bajo (esquiva nueva)
        self._recup_arm_streak: int = 0          # frames seguidos con peso alto (debounce de armado)
        self._recup_clear_count: int = 0         # frames seguidos con el path ya despejado
        self._recup_noghost_streak: int = 0      # frames sin ver color fresco (con esquiva armada + herr grande)
        self._g_streak: int = 0                  # est=G consecutivos (debounce de giro)
        self._last_recup_reason: str = "-"       # para overlay / journalctl

        # Serial
        self.serial_link = SerialLink(cfg.serial_port, cfg.baudrate)

        # Captura / grabación
        self.loop_count    = 0
        self.frame_grabber = None
        self.video_writer  = None
        self.record_count  = 0
        self.cam_frame_count = 0
        self.output_file   = (resolve_output_path(cfg.record_output)
                              if cfg.record_orillas else None)

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
                f"pasado={int(pasado)},intr={int(interior)}")

    # ── Trigger de RECUPERANDO por ESTADO MEDIDO ──────────────────────────────

    def _measured_recup_trigger(self, cl_stats: dict, bev_obstacles: list,
                                corner_soon: bool = False,
                                fresh_color: bool = True,
                                obs_beyond: list | None = None) -> bool:
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
            # Latch el cono que armó esta esquiva (el color más cercano al eje en
            # bev_obstacles -- post-LOCK es el primario). A partir de aquí el
            # chequeo de "rodeado" lo sigue por posición en mia+beyond.
            _armc = [(ox, oy) for (ox, oy, c) in bev_obstacles
                     if c in ("Red", "Green")]
            self._recup_lock_xy = (
                min(_armc, key=lambda p: abs(p[0] - C.ROBOT_BEV_X))
                if _armc else None)

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
              f"noghost={getattr(self, '_recup_noghost_streak', 0)} "
              f"lock={'-' if self._recup_lock_xy is None else f'{self._recup_lock_xy[0]:.0f},{self._recup_lock_xy[1]:.0f}'}",
              flush=True)

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
        # "Me quedo con el cono que esquivo": si hay un cono latcheado (lo hubo al
        # armar), el bloqueo se evalúa SOLO contra él, buscándolo en mia+beyond
        # -> aunque su clasificación cambie a beyond a media esquiva sigue
        # bloqueando hasta que de verdad quede atrás. Un cono NUEVO que solo
        # aparece (recta siguiente) ya no retiene RECUPERANDO (orillas496).
        _all_color = [t for t in (list(bev_obstacles) + list(obs_beyond or []))
                      if t[2] in ("Red", "Green")]
        _locked_ahead = None
        if self._recup_lock_xy is not None:
            _lr2 = getattr(C, "LOCK_MATCH_RADIUS_PX", 70.0) ** 2
            _bd = _lr2
            for (ox, oy, c) in _all_color:
                _d = (ox - self._recup_lock_xy[0]) ** 2 + (oy - self._recup_lock_xy[1]) ** 2
                if _d < _bd:
                    _bd, _locked_ahead = _d, (ox, oy)
        if self._recup_lock_xy is not None:
            if _locked_ahead is not None:
                self._recup_lock_xy = _locked_ahead      # seguir al cono
                blocking = (ry - _locked_ahead[1]) > ahead_tol
            else:
                blocking = False                         # el cono latcheado ya no está -> rodeado
        else:
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
        # PERO: si el cono LATCHEADO sigue adelante, eso MANDA -- el respaldo por
        # peso bajo no aplica (el peso cayó porque el cono se fue a beyond, no
        # porque el carro lo haya rodeado -- orillas496).
        if self._recup_lock_xy is not None and blocking:
            path_clear = False
        else:
            path_clear = (not blocking) or (max_w_near <= C.RECUP_MEAS_CLEAR_W) or ghost_stale

        if not path_clear:
            self._recup_clear_count = 0
            self._last_recup_reason = (
                f"rodeando herr={heading_err:+.0f} w={max_w_near:.2f} "
                f"atol={ahead_tol:.0f}")
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
            self._recup_lock_xy = None
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
            self._recup_lock_xy = None
            # Solo olvidar si NINGÚN cono sigue de verdad adelante (>40px del
            # eje): así no se borra un obstáculo que el carro tiene justo
            # enfrente y todavía debe rodear. Se mira mia+beyond -- un cono que
            # se fue a beyond a media esquiva NO debe borrarse (orillas496).
            if not any(c in ("Red", "Green") and (ry - oy) > 40.0
                       for (ox, oy, c) in _all_color):
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

    def _maybe_record(self, frame: np.ndarray, fps: float):
        if not self.cfg.record_orillas:
            return
        self.record_count += 1
        if self.record_count % max(1, self.cfg.record_every_n) != 0:
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

    # ── Loop principal ────────────────────────────────────────────────────────

    def run(self, on_ready=None, should_start=None, should_record=None):
        # BLOQUEA hasta que el UART está listo (antes era no-op y el loop
        # arrancaba mandando V2 al vacío ~1.5 s -> el ESP32 rodaba en su
        # fallback wall-PID = "el carro avanza y no le entran datos").
        if self.serial_link.open():
            print("[SERIAL] UART listo.", flush=True)
        self._start_capture()

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

        last_fps_time = time.perf_counter()
        fps_count     = 0
        fps           = 0.0
        t_prev_end    = time.perf_counter()   # para medir tiempo de captura entre iteraciones
        timing_ms     = {"cap": 0.0, "vis": 0.0, "bev": 0.0, "ser": 0.0, "disp": 0.0, "rec": 0.0}

        try:
            while True:
                ret, frame = self._read_frame()
                if not ret:
                    time.sleep(0.01)
                    continue

                self.loop_count += 1
                if self.loop_count % self.cfg.process_every_n != 0:
                    time.sleep(0.001)
                    continue

                # ── Armado: botón + start-delay ──────────────────────────────
                if not armed and (should_start is None or should_start()):
                    # READY 3x rápido (el ESP32 solo necesita 1 para salir de su
                    # wait). NO bloquear ~0.7 s esperando ACK: el stream de V2
                    # que arranca acto seguido mantiene vivo al ESP32, y este
                    # ya NO rueda hasta el 1er V2 (ver piFirstV2Received .ino).
                    for _ in range(3):
                        self.serial_link.send_line("READY")
                        time.sleep(0.03)
                    ready_ack = "ACK:READY" in (self.serial_link.try_readline() or "")
                    armed = True
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
                    if on_ready is not None:
                        on_ready()
                    print(f"[GPIO] GO — READY x3 enviado (ack={'sí' if ready_ack else '?'}).", flush=True)

                # FPS contador
                fps_count += 1
                now = time.perf_counter()
                if now - last_fps_time >= 1.0:
                    fps           = fps_count / (now - last_fps_time)
                    fps_count     = 0
                    last_fps_time = now

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
                timing_ms["cap"] = (now - t_prev_end) * 1000.0

                # ── Visión ──────────────────────────────────────────────────
                frame = cv2.flip(frame, 1)
                processed_frame, positions = self.vision.process_frame(frame)
                t_vis = time.perf_counter()
                timing_ms["vis"] = (t_vis - now) * 1000.0

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
                        line_info = {"Orange": self.line_tracker.update(bev_frame, bev_hsv=bev_hsv)}

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
                        if orange_info["seen"] and not en_recuperacion_giro:
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
                                )
                            )

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
                            _lock = bev_obstacles[_li]
                            self._lock_xy = (_lock[0], _lock[1])
                            _dropped = [o for j, o in enumerate(bev_obstacles) if j != _li]
                            bev_obstacles_beyond.extend(_dropped)
                            _lkc = obstacle_conf[_li] if _li < len(obstacle_conf) else 1.0
                            bev_obstacles, obstacle_conf = [_lock], [_lkc]
                            print(f"[LOCK] {_lock[2]}@({_lock[0]:.0f},{_lock[1]:.0f}) "
                                  f"h={_cam_h(_lock[0], _lock[1]):.0f} difiere {len(_dropped)}",
                                  flush=True)
                        elif len(bev_obstacles) == 1:
                            self._lock_xy = (bev_obstacles[0][0], bev_obstacles[0][1])
                        else:
                            self._lock_xy = None

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
                        # ya sin los que quedaron más allá de la naranja)
                        path_points = detect_centerline(
                            bev_frame, bev_obstacles, bev_hsv=bev_hsv,
                            obstacle_conf=obstacle_conf, stats_out=cl_stats,
                        )
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
                                obs_beyond=bev_obstacles_beyond,
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

                        if len(path_points) >= C.MIN_PATH_PTS:
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
                pasado = self._pasado_hold > 0
                if self._pasado_hold > 0:
                    self._pasado_hold -= 1
                    if self._pasado_hold == 0:
                        self._pasado_from_measured = False

                # Sin línea válida → recto (obs=0).
                state = "pp_follow" if pp_active else "no_path"

                t_bev = time.perf_counter()
                timing_ms["bev"] = (t_bev - t_vis) * 1000.0

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
                _turn_block = (self._ext_corner_block > 0) or _turn_hold

                serial_msg = self._build_serial_message(
                    obs_norm, state, len(bev_obstacles), pasado, interior,
                    turn_block=_turn_block,
                )
                if armed:
                    self.serial_link.send_line(serial_msg)
                    serial_ack = self.serial_link.try_readline()
                else:
                    serial_ack = None   # desarmado: pipeline corre, carro quieto

                heading = _parse_heading(serial_ack)
                if heading is not None:
                    self._last_heading = heading

                # Dirección de giro AUTORITATIVA del ESP32 (dir= en el ACK, L/R
                # desde su 1er GIRANDO). Cubre esquinas 2-12; corrige cualquier
                # estimación de visión.
                _esp_dir = _parse_direccion(serial_ack)
                if _esp_dir is not None:
                    self.turn_dir_tracker.set_esp_direction(_esp_dir)

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
                        self._recup_lock_xy = None
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
                        self._recup_lock_xy = None
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

                print(f"[LINEA] Orange={line_info['Orange']}", flush=True)
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
                timing_ms["ser"] = (t_ser - t_bev) * 1000.0

                # ── Display ──────────────────────────────────────────────────
                self._annotate(processed_frame, steer_deg, obs_norm,
                            pp_active, len(path_points), positions,
                            serial_msg, fps, len(bev_obstacles),
                            self.memory.last_prune_reason, timing_ms, bev_timing)

                if bev_frame is not None:
                    bev_debug = draw_bev_debug(
                        bev_frame, path_points, lookahead_pt,
                        bev_obstacles, steer_deg, pp_active,
                        line_info=line_info,
                        bev_obstacles_beyond=bev_obstacles_beyond,
                    )
                    bev_h = processed_frame.shape[0]
                    bev_small = cv2.resize(bev_debug, (bev_h, bev_h))
                    combined = np.hstack([processed_frame, bev_small])
                else:
                    combined = processed_frame

                t_disp = time.perf_counter()
                timing_ms["disp"] = (t_disp - t_ser) * 1000.0

                if self.cfg.show_window:
                    cv2.imshow("WRO Pure Pursuit + Memoria", combined)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

                # Grabación al .avi: solo DESDE que se picó el botón (incluye el
                # start-delay + la run). El pipeline desarmado corre antes pero
                # NO se graba -> el archivo no acumula el rato de "esperando
                # botón". El _write_cam_frame (vista VNC) sí corre siempre.
                if should_record is None or should_record():
                    self._maybe_record(combined, fps)
                self._write_cam_frame(combined)   # ← ahora manda cámara + BEV/ruta

                t_prev_end = time.perf_counter()
                timing_ms["rec"] = (t_prev_end - t_disp) * 1000.0

        finally:
            if self.frame_grabber is not None:
                self.frame_grabber.stop()
            self.vision.cap.release()
            if self.video_writer is not None:
                self.video_writer.stop()
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
                    should_record=should_record)
    except KeyboardInterrupt:
        # SIGTERM (systemctl stop/restart) o Ctrl-C: run() ya corrió su
        # finally (cerró video/serial). Salir sin escupir traceback.
        print("[INFO] Detenido (SIGTERM/SIGINT).", flush=True)


if __name__ == "__main__":
    main()
