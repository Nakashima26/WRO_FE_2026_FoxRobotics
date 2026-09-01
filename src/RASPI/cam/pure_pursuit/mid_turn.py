"""
Detector INSTANTÁNEO de obstáculos durante el giro (estado GIRANDO del ESP32).

Contexto
--------
Mientras el ESP32 gira, runtime_nuevo apaga la memoria rodante
(obstacle_memory.py): su modelo de ego-movimiento asume avance a
ROBOT_SPEED_MMS y en el pivote el carro casi no traslada -> arrastra las latas
a posiciones equivocadas -> "fantasmas". Por eso durante el giro bev_obstacles
se fuerza a [] y el carro queda ciego a un cono que aparezca en la recta nueva
(caso real de descalificación: verde pegado a la salida del giro que no se
alcanzó a detectar a tiempo).

Este detector NO usa memoria ni dead-reckoning. Solo mira las proyecciones BEV
CRUDAS de cada frame (new_obstacles, que runtime_nuevo ya calcula) y exige que
una lata del mismo color aparezca en una posición consistente varios frames
seguidos, dentro de un ROI cercano y hacia adelante, para "confirmarla". Un
fantasma de un frame no persiste en la misma posición; una lata real sí.

FASE 1 (este archivo)
---------------------
SOLO observa y registra (línea [MTURN] a journalctl). NO toca el steering, NO
manda nada nuevo al ESP32, NO siembra la memoria rodante. Es para medir en
pista si la detección mid-turn es fiable ANTES de cablearla al firmware:
  Fase 2 -> campo serial `mturn` + modulación del lock del servo en GIRANDO
  Fase 3 -> sembrar la memoria rodante al terminar el giro (continuidad)

Convención BEV (ver bev.py): el robot está en (ROBOT_BEV_X, ROBOT_BEV_Y),
"apuntando hacia arriba"; Y decrece hacia adelante. dx > 0 = derecha del robot.
`heading_deg` es el anguloGyro que el ESP32 regresa en el ACK:V2 — durante
GIRANDO arranca en 0 y crece, así que |heading_deg| = avance del giro en grados.
"""

from collections import deque
from dataclasses import dataclass
import math

from . import config as C


@dataclass
class MidTurnSighting:
    """Una lata confirmada durante el giro. FASE 1: el llamador solo la registra."""
    color:       str    # 'Red' | 'Green'
    side:        str    # 'L' | 'R' | '?'  — lado observado respecto al eje del robot
    dist_mm:     float  # distancia real robot -> lata al confirmar
    bev_x:       float
    bev_y:       float
    frames:      int    # frames del window que respaldaron la confirmación
    window:      int
    heading_deg: float  # avance del giro (|anguloGyro|) al confirmar
    wro_bias:    int     # +1 = pasar por la DERECHA (Rojo), -1 = por la IZQUIERDA
                         # (Verde). FASE 1: informativo; NO se envía al ESP32.


class MidTurnObstacleDetector:
    """Ver docstring del módulo. Uso:

        det = MidTurnObstacleDetector()
        # al entrar a GIRANDO:
        det.reset()
        # cada frame de GIRANDO:
        ev = det.update(new_obstacles, last_heading)
        if ev is not None:
            log(ev)          # FASE 1: solo registrar
    """

    def __init__(self):
        self.window       = int(getattr(C, "MIDTURN_WINDOW", 4))
        self.confirm_n    = int(getattr(C, "MIDTURN_CONFIRM_FRAMES", 3))
        self.roi_max_mm   = float(getattr(C, "MIDTURN_ROI_MAX_MM", 280.0))
        self.roi_half_deg = float(getattr(C, "MIDTURN_ROI_HALF_ANGLE_DEG", 45.0))
        self.pos_tol_px   = float(getattr(C, "MIDTURN_POS_TOL_PX", 50.0))
        self.min_gyro_deg = float(getattr(C, "MIDTURN_MIN_GYRO_DEG", 25.0))
        self.side_dead_px = float(getattr(C, "MIDTURN_SIDE_DEADBAND_PX", 24.0))

        self._buf: deque = deque(maxlen=self.window)   # por frame: [(color, bx, by), ...] en ROI
        self._last_heading: float | None = None
        self._emitted_keys: set = set()                # clusters ya reportados este giro
        self.last_sighting: MidTurnSighting | None = None
        self._roi_counts = {"Red": 0, "Green": 0}      # del frame más reciente (diag)

    def reset(self):
        """Llamar al ENTRAR a un giro (y al armar). Un giro nuevo = historia limpia."""
        self._buf.clear()
        self._last_heading = None
        self._emitted_keys = set()
        self.last_sighting = None
        self._roi_counts = {"Red": 0, "Green": 0}

    # ── ROI: cercano y hacia adelante ────────────────────────────────────────
    def _roi_metrics(self, bx: float, by: float) -> tuple[float, float] | None:
        """(dist_mm, ang_deg) si (bx,by) cae en el ROI; None si queda fuera."""
        dx = bx - C.ROBOT_BEV_X
        dy = C.ROBOT_BEV_Y - by                       # + = adelante del robot
        if dy <= 1.0:
            return None                              # al costado o atrás
        dist_mm = math.hypot(dx, dy) * C.MM_PER_PX
        if dist_mm > self.roi_max_mm:
            return None
        ang = math.degrees(math.atan2(dx, dy))        # 0 = al frente, + = derecha
        if abs(ang) > self.roi_half_deg:
            return None
        return dist_mm, ang

    # ── Update ───────────────────────────────────────────────────────────────
    def update(self, new_obstacles, heading_deg: float | None) -> MidTurnSighting | None:
        """
        new_obstacles : lista de (bev_x, bev_y, color) — proyección BEV CRUDA de
                        ESTE frame (SIN memoria rodante). runtime_nuevo ya la
                        calcula en el pipeline; aquí se pasa tal cual.
        heading_deg   : anguloGyro del ESP32 (avance del giro), o None.

        Devuelve un MidTurnSighting el frame en que un cluster se confirma por
        primera vez este giro; None en los demás frames.
        """
        self._last_heading = heading_deg
        self._roi_counts = {"Red": 0, "Green": 0}

        # Gate por avance del giro: los primeros grados apuntan a la esquina
        # misma / a la recta que se deja atrás -> un cono ahí es el que YA se
        # rebasó, no el de la recta nueva. Igual se apila [] para envejecer
        # observaciones viejas del buffer.
        if heading_deg is None or abs(heading_deg) < self.min_gyro_deg:
            self._buf.append([])
            return None

        kept: list[tuple[str, float, float]] = []
        for item in (new_obstacles or []):
            bx, by, color = float(item[0]), float(item[1]), item[2]
            if color not in ("Red", "Green"):
                continue
            if self._roi_metrics(bx, by) is None:
                continue
            kept.append((color, bx, by))
            self._roi_counts[color] += 1
        self._buf.append(kept)

        if len(self._buf) < self.confirm_n:
            return None

        # Anclar en las latas del frame más reciente (las más fiables); para
        # cada una contar en cuántos frames del window hay una lata del MISMO
        # color a <= pos_tol_px. Reportar la MÁS CERCANA que llegue a confirm_n
        # y que no se haya reportado ya este giro.
        anchors = sorted(
            kept,
            key=lambda a: math.hypot(a[1] - C.ROBOT_BEV_X, C.ROBOT_BEV_Y - a[2]),
        )
        for color, ax, ay in anchors:
            matched: list[tuple[float, float]] = []
            for frame_obs in self._buf:
                best = None
                for (c, bx, by) in frame_obs:
                    if c != color:
                        continue
                    d = math.hypot(bx - ax, by - ay)
                    if d <= self.pos_tol_px and (best is None or d < best[0]):
                        best = (d, bx, by)
                if best is not None:
                    matched.append((best[1], best[2]))
            if len(matched) < self.confirm_n:
                continue

            mbx = sum(p[0] for p in matched) / len(matched)
            mby = sum(p[1] for p in matched) / len(matched)
            key = (color, round(mbx / self.pos_tol_px), round(mby / self.pos_tol_px))
            if key in self._emitted_keys:
                continue
            self._emitted_keys.add(key)

            dx = mbx - C.ROBOT_BEV_X
            dist_mm = math.hypot(dx, C.ROBOT_BEV_Y - mby) * C.MM_PER_PX
            side = "?" if abs(dx) < self.side_dead_px else ("R" if dx > 0 else "L")
            self.last_sighting = MidTurnSighting(
                color=color, side=side, dist_mm=dist_mm,
                bev_x=mbx, bev_y=mby, frames=len(matched), window=self.window,
                heading_deg=float(heading_deg),
                wro_bias=(+1 if color == "Red" else -1),
            )
            return self.last_sighting
        return None

    # ── Diagnóstico por frame (línea [MTURN] mientras dura el giro) ───────────
    def status_str(self) -> str:
        h = "-" if self._last_heading is None else f"{self._last_heading:+.0f}"
        last = self.last_sighting
        conf = "-" if last is None else f"{last.color}/{last.side}@{last.dist_mm:.0f}mm"
        return (f"buf={len(self._buf)}/{self.window} "
                f"roiR={self._roi_counts['Red']} roiG={self._roi_counts['Green']} "
                f"gyro={h} confirmado={conf}")
