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

    def run(self, on_ready=None):
        self.serial_link.open()
        self._start_capture()

        # Warmup de cámara (esperar exposición automática estabilizada)
        print(f"[INFO] Calentando cámara ({self.cfg.warmup_frames} frames)...", flush=True)
        warmed = 0
        while warmed < self.cfg.warmup_frames:
            ret, _ = self._read_frame()
            if ret:
                warmed += 1
            else:
                time.sleep(0.01)
        print("[INFO] Cámara estabilizada.", flush=True)

        self.serial_link.send_line("READY")
        print("[INFO] READY enviado al ESP32.", flush=True)

        if on_ready is not None:
            on_ready()

        print("[INFO] Pure Pursuit + memoria runtime iniciado. ESC para detener.", flush=True)

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

                # FPS contador
                fps_count += 1
                now = time.perf_counter()
                if now - last_fps_time >= 1.0:
                    fps           = fps_count / (now - last_fps_time)
                    fps_count     = 0
                    last_fps_time = now

                # dt para la memoria rodante (tiempo entre frames procesados)
                if self._last_update_t is None:
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
                            )
                            # Alineado 1:1 con bev_obstacles (mismo orden) --
                            # ver detect_centerline(obstacle_conf=).
                            obstacle_conf = list(self.memory.last_confidences)
                            # Evento de un solo frame: un obstáculo quedó detrás
                            # del robot recién en este update -> "ya lo pasé de
                            # verdad", no "dejé de verlo". Dispara RECUPERANDO.
                            pasado = self.memory.last_passed
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
                        if bev_obstacles and turn_dir is not None:
                            closest = max(bev_obstacles, key=lambda o: o[1])
                            interior = is_interior_pass(turn_dir, closest[2])
                        _t4 = time.perf_counter()
                        bev_timing["line"] = (_t4 - _t3) * 1000.0

                        # Detectar centerline (con obstáculos recordados+nuevos,
                        # ya sin los que quedaron más allá de la naranja)
                        path_points = detect_centerline(
                            bev_frame, bev_obstacles, bev_hsv=bev_hsv, obstacle_conf=obstacle_conf
                        )
                        _t5 = time.perf_counter()
                        bev_timing["dc"] = (_t5 - _t4) * 1000.0

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

                # Sin línea válida → recto (obs=0).
                state = "pp_follow" if pp_active else "no_path"

                t_bev = time.perf_counter()
                timing_ms["bev"] = (t_bev - t_vis) * 1000.0

                # ── Construir y enviar mensaje serial ────────────────────────
                serial_msg = self._build_serial_message(
                    obs_norm, state, len(bev_obstacles), pasado, interior
                )
                self.serial_link.send_line(serial_msg)
                serial_ack = self.serial_link.try_readline()

                heading = _parse_heading(serial_ack)
                if heading is not None:
                    self._last_heading = heading

                estado_now = _parse_estado(serial_ack)
                if estado_now is not None:
                    if estado_now == "G" and self._prev_estado != "G":
                        # Empieza el giro físico -> vaciar YA y apagar la memoria.
                        self.memory.reset()
                        self.line_tracker.reset()   # la línea ya quedó atrás, no aplica a la recta nueva
                        self._is_turning   = True
                        self._turn_start_t = now
                        print("[MEM] Giro detectado — memoria de obstáculos desactivada.", flush=True)
                    elif estado_now != "G" and self._is_turning:
                        # Terminó el giro -> la memoria ya está vacía (no se tocó), arranca
                        # limpia con las detecciones frescas de este frame.
                        self._is_turning = False
                        self._turn_recovery_frames = C.TURN_RECOVERY_FRAMES
                        print("[MEM] Giro terminado — memoria de obstáculos reactivada.", flush=True)
                    self._prev_estado = estado_now

                # Red de seguridad: si por un ACK perdido/atorado el ESP32 nunca
                # reporta salir de "G", no dejar la memoria apagada para siempre.
                if (self._is_turning and self._turn_start_t is not None
                        and (now - self._turn_start_t) > C.TURN_TIMEOUT_S):
                    self._is_turning = False
                    print("[MEM] Timeout de giro — memoria de obstáculos reactivada por seguridad.", flush=True)

                log_line = f"TX: {serial_msg}"
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

                self._maybe_record(processed_frame, fps)
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
    return p.parse_args()


def main():
    # ── GPIO: LED de encendido + esperar botón (igual que wro_runtime.py) ────
    _gpio = None
    try:
        import RPi.GPIO as GPIO
        _gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(27, GPIO.OUT)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(27, GPIO.HIGH)
        print("[GPIO] LED encendido — Pi prendida.", flush=True)
        print("[GPIO] Esperando botón en GPIO17...", flush=True)
        while GPIO.input(17) == GPIO.LOW:
            time.sleep(0.05)
        print("[GPIO] Botón detectado. Iniciando...", flush=True)
    except ImportError:
        print("[GPIO] RPi.GPIO no disponible, omitiendo espera de botón.", flush=True)

    def led_on():
        if _gpio is not None:
            _gpio.output(27, _gpio.HIGH)
            print("[GPIO] LED encendido — sistema listo.", flush=True)

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

    runtime = PPRuntime(cfg)
    runtime.run(on_ready=led_on)


if __name__ == "__main__":
    main()
