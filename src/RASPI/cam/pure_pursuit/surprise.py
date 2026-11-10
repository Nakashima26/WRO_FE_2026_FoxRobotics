"""Regla sorpresa 2026: altura observada -> color de conducción.

No recibe el Field del twin. La única geometría disponible es la calibración
BEV y el montaje de la cámara; los colores visibles se conservan en el informe.
"""
from dataclasses import dataclass
import math

import numpy as np

from . import config as C


def opposite(color):
    return {"Red": "Green", "Green": "Red"}[color]


def ground_visible(image, box):
    """Apoyo de piso bajo el bloque; descarta, p. ej., la cima tras una pared.

    Es una compuerta conservadora, no una prueba de ausencia de toda oclusión.
    Se aplica a nuevas clasificaciones especiales, no borra una ya recordada.
    """
    x, y, w, h = map(int, box)
    rows, cols = image.shape[:2]
    patch = image[min(rows,y+h+1):min(rows,y+h+4), max(0,x+w//3):min(cols,x+2*w//3)]
    if not patch.size:
        return False
    hi = patch.max(axis=2).astype(float)
    lo = patch.min(axis=2).astype(float)
    return bool(np.mean((hi >= 70) & ((hi-lo) <= .45*np.maximum(hi,1))) >= .6)


def validate_reference(samples):
    table = np.asarray(samples, dtype=float)
    if table.size == 0:
        return
    if (table.ndim != 2 or table.shape[1] != 2 or len(table) < 2
            or not np.isfinite(table).all() or np.any(table[:,1] <= 0)
            or np.any(np.diff(table[:,0]) <= 0)):
        raise ValueError("SURPRISE_NORMAL_HEIGHT_SAMPLES requiere pares ordenados (distancia BEV mm, alto normal px)")


def estimate_height(bev, box, frame_shape, *, camera_height=None, camera_ground=None, allow_top_clipped=False, allow_side_clipped=False):
    """Altura por metrología de una vista, sin asumir distancia por tamaño.

    G proyecta el piso. La columna vertical q cumple G*Cxy + Hcam*q=0
    (el centro óptico se proyecta a cero). Así P(X,Y,z)=G(X,Y,1)+z*q.
    El pie y la cima del bbox aproximan una vertical sobre la cara visible.
    Rechaza recortes por defecto, objetos pequeños y geometría degenerada.
    Los flags de recorte solo se usan para una cota de altura o para reconocer
    una señal normal con margen estrecho.
    """
    x, y, w, h = box
    rows, cols = frame_shape[:2]
    if w < 5 or h < 6 or ((x <= 1 or x+w >= cols-1) and not allow_side_clipped) or (y <= 1 and not allow_top_clipped) or y+h >= rows-1:
        return None
    if getattr(bev, "H", None) is None or getattr(bev, "H_inv", None) is None:
        return None
    mm = C.MM_PER_PX
    ground_to_bev = np.array([[1/mm, 0, C.ROBOT_BEV_X],
                              [0, -1/mm, C.ROBOT_BEV_Y], [0, 0, 1]])
    foot = np.asarray(bev.H) @ np.array([x+(w-1)/2, y+h-1, 1.0])
    if abs(foot[2]) < 1e-9:
        return None
    foot /= foot[2]
    reference = C.SURPRISE_NORMAL_HEIGHT_SAMPLES
    if reference and camera_height is None and not allow_top_clipped:
        table = np.asarray(reference, dtype=float)
        distance = (C.ROBOT_BEV_Y-foot[1])*mm
        if not table[0,0] <= distance <= table[-1,0]:
            return None
        normal_px = float(np.interp(distance, table[:,0], table[:,1]))
        return 100.0*h/normal_px
    hc = float(C.CAMERA_HEIGHT_MM if camera_height is None else camera_height)
    if hc <= 0 or not math.isfinite(hc):
        return None
    if camera_ground is None:
        camera_ground = (C.CAMERA_RIGHT_MM, C.CAMERA_FWD_MM-C.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM)
    g = np.asarray(bev.H_inv) @ ground_to_bev
    ground = np.array([(foot[0]-C.ROBOT_BEV_X)*mm, (C.ROBOT_BEV_Y-foot[1])*mm, 1.0])
    b = g @ ground
    q = -(g @ np.array([*camera_ground, 1.0])) / hc
    # El centro horizontal del bbox puede moverse entre la cara superior y
    # la base (cubo visto de costado). Su fila superior sí mide la cima:
    # resolver solo esa coordenada evita rechazar señales en los bordes.
    top_y = float(y)
    denominator = top_y*q[2]-q[1]
    if abs(denominator) < 1e-12:
        return None
    z = float((b[1]-top_y*b[2])/denominator)
    if not math.isfinite(z) or not 20 <= z <= 180:
        return None
    return z


@dataclass
class _Track:
    point: tuple
    visible: str
    candidate: int | None = None
    votes: int = 0
    height: int | None = None
    last: int = 0
    seat: tuple | None = None


class SurpriseInterpreter:
    """Confirma altura antes de publicar color; recuerda la señal por asiento.

    Las observaciones ambiguas no se publican como si fueran normales. Una
    clasificación confirmada puede reutilizarse en un recorte posterior. Los
    bloques especiales requieren varios votos. Un normal a 100±8 mm se
    publica inmediatamente para conservar la reacción durante la salida.
    """
    def __init__(self):
        validate_reference(C.SURPRISE_NORMAL_HEIGHT_SAMPLES)
        self.tracks = []
        self.frame = 0
        self.seats = {}
        self.observations = []

    def reset(self):
        self.__init__()

    def interpret(self, positions, bev, shape, digital=None, image=None):
        from .digital_map import feet_world, _seat_world
        self.frame += 1
        self.observations = []
        out = {c: [] for c in positions}
        used = set()
        for color in ("Red", "Green"):
            for box in positions.get(color, ()):
                x, y, w, h = box
                # Si el pie sale del cuadro, no inventar su posición usando
                # el borde de la imagen: ObstacleMemory sigue con la anterior.
                foot = bev.cam_to_bev(x+(w-1)/2, y+h-1) if y+h < shape[0]-1 else None
                point = None
                seat = None
                if foot is not None:
                    point = ((foot[0]-C.ROBOT_BEV_X)*C.MM_PER_PX,
                             (C.ROBOT_BEV_Y-foot[1])*C.MM_PER_PX)
                    if digital is not None and digital.direction is not None:
                        world = feet_world([(*foot, color)], digital.pose_xy, digital._view_heading)
                        if world:
                            point = world[0][:2]
                            ranked = sorted((math.hypot(point[0]-sx, point[1]-sy), (sec, sid))
                                            for sec in ("N", "E", "S", "W") for sid, sx, sy in _seat_world(sec))
                            if ranked[0][0] < 160 and ranked[1][0]-ranked[0][0] > 40:
                                seat = ranked[0][1]
                height = estimate_height(bev, box, shape)
                if height is None and (x <= 1 or x+w >= shape[1]-1):
                    partial = estimate_height(bev, box, shape, allow_side_clipped=True)
                    if partial is not None and abs(partial-100) <= 8:
                        height = partial
                clipped_tall = False
                if height is None and y <= 1 and not C.SURPRISE_NORMAL_HEIGHT_SAMPLES:
                    lower = estimate_height(bev, box, shape, allow_top_clipped=True)
                    clipped_tall = lower is not None and lower >= 125
                candidate = 150 if clipped_tall else None if height is None else min((50, 100, 150), key=lambda z: abs(z-height))
                if candidate is not None and not clipped_tall and abs(candidate-height) > float(C.SURPRISE_HEIGHT_TOL_MM):
                    candidate = None
                supported = image is None or ground_visible(image, box)
                if candidate in (50, 150) and not supported:
                    candidate = None
                track = None
                if point is not None:
                    choices = [(math.dist(point, t.point), i, t) for i, t in enumerate(self.tracks)
                               if i not in used and t.visible == color and self.frame-t.last <= 5
                               and (t.seat is None or seat is None or t.seat == seat)]
                    if choices:
                        distance, i, nearest = min(choices, key=lambda item: item[0])
                        if distance < 140:
                            track = nearest
                            used.add(i)
                    if track is None:
                        track = _Track(point, color)
                        self.tracks.append(track)
                        used.add(len(self.tracks)-1)
                    track.point, track.last, track.seat = point, self.frame, seat
                    if seat in self.seats:
                        saved_color, saved_height = self.seats[seat]
                        if saved_color == color:
                            track.height = saved_height
                    if candidate is not None and (supported or track.height in (None, candidate)):
                        track.votes = track.votes+1 if candidate == track.candidate else 1
                        track.candidate = candidate
                        required = 1 if candidate == 100 and track.height in (None, 100) and abs(height-100) <= 8 else int(C.SURPRISE_CONFIRM_FRAMES)
                        if track.votes >= required:
                            if candidate in (50, 150) and track.height != candidate:
                                measured = round(height, 1) if height is not None else ">=125"
                                print(f"[SURPRISE] visible={color} driving={opposite(color)} "
                                      f"height={candidate} estimate={measured} seat={seat}", flush=True)
                            track.height = candidate
                            if seat is not None:
                                self.seats[seat] = (color, candidate)
                    else:
                        track.votes = 0
                        track.candidate = None
                confirmed = track.height if track is not None else None
                logical = opposite(color) if confirmed in (50, 150) else color if confirmed == 100 else None
                self.observations.append(dict(box=tuple(box), visible=color, driving=logical,
                                              height_mm=height, confirmed_mm=confirmed, seat=seat, ground_support=supported,
                                              foot_distance_mm=((C.ROBOT_BEV_Y-foot[1])*C.MM_PER_PX if foot is not None else None)))
                position_valid = foot is not None and (confirmed == 100 or (x > 1 and x+w < shape[1]-1))
                self.observations[-1]["position_valid"] = position_valid
                if logical is not None and position_valid:
                    out.setdefault(logical, []).append(box)
        self.tracks = [t for t in self.tracks if self.frame-t.last <= 5]
        return out
