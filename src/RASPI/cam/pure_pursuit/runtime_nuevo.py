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
    return val if val in ("G", "R", "S") else None


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

        # Estado de la memoria rodante
        self._last_heading: float | None = None
        self._last_update_t: float | None = None
        self._prev_estado: str | None = None
        self._is_turning: bool = False
        self._turn_start_t: float | None = None
        self._turn_recovery_frames: int = 0
        self._pasado_hold: int = 0   # frames restantes repitiendo pasado=1

        # ── Trigger de RECUPERANDO por ESTADO MEDIDO (ver _measured_recup_trigger) ──
        self._heading_ref: float | None = None   # heading de la recta al ARMARSE la esquiva
        self._dodge_armed: bool = False          # hubo una esquiva de verdad en curso
        self._recup_clear_count: int = 0         # frames seguidos con el path ya despejado
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
                               pasado: bool, interior: bool) -> str:
        has_obstacle = n_mem_obs > 0   # basado en memoria real, no en steer
        return (f"V2,obs={obs_norm:+.3f},turn=0,"
                f"state={state},prio={int(has_obstacle)},mem={n_mem_obs},pp=1,"
                f"pasado={int(pasado)},intr={int(interior)}")

    # ── Trigger de RECUPERANDO por ESTADO MEDIDO ──────────────────────────────

    def _measured_recup_trigger(self, cl_stats: dict, bev_obstacles: list) -> bool:
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
             Debe cumplirse RECUP_MEAS_CLEAR_FRAMES frames seguidos.
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

        # heading_ref: se siembra al armarse la esquiva.
        if (self._heading_ref is None and self._last_heading is not None
                and max_w_near >= C.RECUP_MEAS_ARM_W):
            self._heading_ref = self._last_heading

        heading_err = 0.0
        if self._last_heading is not None and self._heading_ref is not None:
            heading_err = (self._last_heading - self._heading_ref + 180.0) % 360.0 - 180.0

        # (1) armar — "hubo una lata que el planner rodeó"
        if max_w_near >= C.RECUP_MEAS_ARM_W:
            self._dodge_armed = True
            self._recup_clear_count = 0
            self._last_recup_reason = (
                f"armado w={max_w_near:.2f} herr={heading_err:+.0f}")
            return False

        if not self._dodge_armed:
            return False

        # (2) ¿alguna lata todavía estorbando derecho adelante? (posición MEDIDA)
        rx, ry = C.ROBOT_BEV_X, C.ROBOT_BEV_Y
        clear_px  = float(getattr(C, "OBS_MEM_GEOM_CLEAR_PX",
                                  C.OBS_INFLATE_R + 35))
        ahead_tol = float(getattr(C, "RECUP_MEAS_AHEAD_TOL_PX", 30.0))
        blocking = False
        for (ox, oy, color) in bev_obstacles:
            if color not in ("Red", "Green"):
                continue
            if (ry - oy) > ahead_tol and abs(ox - rx) < clear_px:
                blocking = True
                break
        # Respaldo: si el planner tampoco rodea nada junto al eje, está despejado
        # aunque la memoria aún cargue la lata en algún lado raro.
        path_clear = (not blocking) or (max_w_near <= C.RECUP_MEAS_CLEAR_W)

        if not path_clear:
            self._recup_clear_count = 0
            return False
        self._recup_clear_count += 1
        if self._recup_clear_count < C.RECUP_MEAS_CLEAR_FRAMES:
            return False

        # (3) ¿vale la pena enderezar?
        if abs(heading_err) >= C.RECUP_MEAS_HEADING_DEG:
            self._last_recup_reason = (
                f"PASADO(medido) herr={heading_err:+.0f} "
                f"clr={self._recup_clear_count} block=0")
            self._dodge_armed = False
            self._recup_clear_count = 0
            self._heading_ref = None
            return True

        # Path despejado pero el chasis casi recto -> esquiva suave que se
        # resolvió sola. Desarmar en silencio, SIN RECUPERANDO.
        self._last_recup_reason = f"esquiva-suave-ok herr={heading_err:+.0f}"
        self._dodge_armed = False
        self._recup_clear_count = 0
        self._heading_ref = None
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
                line_info     = {"Orange": {"seen": False, "near_y": None}}
                bev_obstacles_beyond = []
                interior      = False
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
                        far_objects   = []   # (center_x, w, h, color) — fuera de BEV
                        for color_name in ("Red", "Green"):
                            for obj in positions.get(color_name, []):
                                x, y, w, h = obj
                                result = map_obstacle_to_bev(self.bev, x, y, w, h)
                                if result is not None:
                                    new_obstacles.append((result[0], result[1], color_name))
                                else:
                                    # No proyectó (fuera de bev_in_bounds o
                                    # cam_to_bev falló) → tratar como lejano
                                    far_objects.append((x + w / 2.0, w, h, color_name))
                        _t2 = time.perf_counter()
                        bev_timing["proj"] = (_t2 - _t1) * 1000.0

                        # ── Memoria rodante: apagada durante el giro para evitar fantasmas ──
                        # (Solo con obstáculos BEV reales — el far_hint NO entra aquí)
                        if self._is_turning:
                            bev_obstacles = []
                            obstacle_conf = []
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
                                    )
                                )
                            )

                        # ── Dirección de giro: se infiere UNA SOLA VEZ (con
                        # persistencia, ver TurnDirectionTracker) de la posición
                        # lateral de un obstáculo visto más allá de la naranja,
                        # y se queda fija toda la carrera.
                        turn_dir = self.turn_dir_tracker.update(
                            bev_obstacles_beyond, C.ROBOT_BEV_X
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
                        if bev_obstacles_beyond:
                            print(f"[CLASS] mia={[(round(x),round(y),c) for x,y,c in bev_obstacles]} "
                                  f"beyond={[(round(x),round(y),c) for x,y,c in bev_obstacles_beyond]} "
                                  f"turn_dir={turn_dir} interior={interior} "
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
                        # Apagado durante el giro y su cooldown (memoria/ línea
                        # aún inestables, ninguna esquiva puede estar en curso).
                        if not self._is_turning and not en_recuperacion_giro:
                            measured_pass = self._measured_recup_trigger(
                                cl_stats, bev_obstacles
                            )
                            if measured_pass:
                                self._pasado_hold = max(self._pasado_hold,
                                                        C.PASADO_HOLD_FRAMES)

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
                pasado = self._pasado_hold > 0
                if self._pasado_hold > 0:
                    self._pasado_hold -= 1

                # Sin línea válida → recto (obs=0).
                state = "pp_follow" if pp_active else "no_path"

                t_bev = time.perf_counter()
                timing_ms["bev"] = (t_bev - t_vis) * 1000.0

                # ── Construir y (si armado) enviar mensaje serial ────────────
                serial_msg = self._build_serial_message(
                    obs_norm, state, len(bev_obstacles), pasado, interior
                )
                if armed:
                    self.serial_link.send_line(serial_msg)
                    serial_ack = self.serial_link.try_readline()
                else:
                    serial_ack = None   # desarmado: pipeline corre, carro quieto

                heading = _parse_heading(serial_ack)
                if heading is not None:
                    self._last_heading = heading

                estado_now = _parse_estado(serial_ack)
                if estado_now is not None:
                    # Debounce: un est=G ESPURIO (ACK con ruido, "est=G fantasma
                    # tras verde") ya no dispara el wipe de memoria a media
                    # esquiva. Un giro real manda est=G muchos frames seguidos;
                    # se exigen TURN_EST_G_CONFIRM_FRAMES consecutivos.
                    if estado_now == "G":
                        self._g_streak += 1
                    else:
                        self._g_streak = 0
                    g_confirmed = self._g_streak >= C.TURN_EST_G_CONFIRM_FRAMES

                    if g_confirmed and not self._is_turning:
                        # Empieza el giro físico -> vaciar YA y apagar la memoria.
                        self.memory.reset()
                        self.line_tracker.reset()   # la línea ya quedó atrás, no aplica a la recta nueva
                        self._is_turning   = True
                        self._turn_start_t = now
                        # La esquiva (si había) muere con el giro.
                        self._dodge_armed = False
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
                print(f"[DIR] fija={self.turn_dir_tracker.direction} interior={interior}", flush=True)
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
