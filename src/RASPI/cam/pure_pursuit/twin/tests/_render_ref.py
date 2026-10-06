"""Funciones libres de render extraídas de camera.py (HEAD, 37b6bd0; idéntico a 54031cd) para tests de equivalencia."""

from __future__ import annotations

import math

import cv2
import numpy as np

from ...wro_field import (
    INNER_HALF_MM,
    LINE_MM,
    OUTER_HALF_MM,
    SIGN_MM,
    blue_segments,
    orange_segments,
)
# Colores y paleta: se leen de camera.py (no se reinventan) para que la
# referencia sea bit-exacta con render_camera. No modifica camera.py: solo
# importa constantes de solo lectura ya definidas ahí.
from ..camera import (
    _PALETTE,
    BLUE_BGR,
    GREEN_BGR,
    MAGENTA_BGR,
    ORANGE_BGR,
    RED_BGR,
    WALL_BGR,
)

# Paleta id: 0 cielo, 1 piso, 2 pared, 3 rojo, 4 verde, 5 magenta
_WALL_TILE_MM = 100.0


def _project(cam: object, cam_pts: np.ndarray) -> np.ndarray:
    """Puntos en el frame de la cámara (z adelante) → píxeles."""
    x = cam_pts[:, 0]
    y = cam_pts[:, 1]
    z = cam_pts[:, 2]
    if cam.params.equidistant:
        theta = np.arctan2(np.hypot(x, y), z)
        radius = cam._fx * theta
        rho = np.hypot(x, y)
        scale = np.ones_like(rho)
        nz = rho > 1e-8
        scale[nz] = radius[nz] / rho[nz]
        return np.column_stack([cam._cx + scale * x, cam._cy + scale * y])
    return np.column_stack(
        [cam._cx + cam._fx * x / z, cam._cy + cam._fy * y / z]
    )


def _to_cam(cam: object, pts, ox, oy, oz, sh, ch):
    """Transforma puntos de mundo a frame de cámara."""
    pts = np.asarray(pts, dtype=np.float64)
    dx = pts[:, 0] - ox
    dy = pts[:, 1] - oy
    dz = pts[:, 2] - oz
    right = dx * ch - dy * sh
    fwd = dx * sh + dy * ch
    robot = np.column_stack([right, fwd, dz])
    return robot @ cam._robot_to_cam.T


def _ribbon_to_cam(cam: object, seg, half, ox, oy, oz, sh, ch):
    """Convierte una cinta (segmento de línea con ancho) al frame de cámara."""
    return _to_cam(cam, _ribbon(seg, half), ox, oy, oz, sh, ch)


def _ribbon(seg, half):
    """Genera los 4 vértices de una cinta rectangular alrededor de un segmento."""
    a, b = seg
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy) or 1.0
    px, py = -dy / length * half, dx / length * half
    z = 1.0
    return (
        (ax + px, ay + py, z),
        (bx + px, by + py, z),
        (bx - px, by - py, z),
        (ax - px, ay - py, z),
    )


def _clip_near(cam_pts, zmin=8.0):
    """Recorta polígono en 3D contra plano z = zmin (near clip)."""
    out = []
    n = len(cam_pts)
    for i in range(n):
        a = cam_pts[i]
        b = cam_pts[(i + 1) % n]
        a_in = a[2] >= zmin
        b_in = b[2] >= zmin
        if a_in and b_in:
            out.append(b)
        elif a_in != b_in:
            t = (zmin - a[2]) / (b[2] - a[2])
            hit = a + t * (b - a)
            out.append(hit)
            if b_in:
                out.append(b)
    return out


def _fill_face(cam: object, img, cam_pts, color):
    """Dibuja un polígono recortado proyectado al frame de imagen."""
    poly = _clip_near(cam_pts)
    if len(poly) < 3:
        return
    poly = np.asarray(poly, dtype=np.float64)
    uv = _project(cam, poly)
    if np.any(np.abs(uv) > 20000):
        return
    cv2.fillConvexPoly(img, np.round(uv).astype(np.int32), color)


def reference_render(cam: object, track, robot_x: float, robot_y: float, heading_deg: float) -> np.ndarray:
    """Render de referencia (copia literal de camera.py:185-266)."""
    from ..world import robot_to_world

    p = cam.params
    h = math.radians(heading_deg)
    sh, ch = math.sin(h), math.cos(h)
    ox, oy = robot_to_world(
        p.right_mm,
        p.forward_from_rear_axle_mm,
        robot_x,
        robot_y,
        heading_deg,
    )

    # Distancia al piso por rayo, y de ahí el punto del tapete. El giro del
    # heading se aplica al punto, no al rayo: una sola rotación por frame.
    t = cam._floor_scale * np.float32(p.height_mm)
    finite = cam._floor_down & (t >= np.float32(1e-3))
    rr = t * cam._ray_right
    ff = t * cam._ray_fwd
    gx = np.zeros(t.shape, dtype=np.float32)
    gy = np.zeros(t.shape, dtype=np.float32)
    gx[finite] = np.float32(ox) + ff[finite] * np.float32(sh) + rr[finite] * np.float32(ch)
    gy[finite] = np.float32(oy) + ff[finite] * np.float32(ch) - rr[finite] * np.float32(sh)
    in_mat = (np.abs(gx) <= OUTER_HALF_MM) & (np.abs(gy) <= OUTER_HALF_MM)
    in_island = (np.abs(gx) < INNER_HALF_MM) & (np.abs(gy) < INNER_HALF_MM)
    floor = finite & in_mat & ~in_island
    island = finite & in_island
    best_id = np.zeros(t.shape, dtype=np.uint8)
    best_id = np.where(floor, np.uint8(1), best_id)
    best_id = np.where(island, np.uint8(2), best_id)
    img = _PALETTE[best_id]

    oz = p.height_mm
    half_line = LINE_MM / 2.0
    for seg, color in (
        *((s, ORANGE_BGR) for s in orange_segments()),
        *((s, BLUE_BGR) for s in blue_segments()),
    ):
        _fill_face(cam, img, _ribbon_to_cam(cam, seg, half_line, ox, oy, oz, sh, ch), color)

    faces = []
    # Pared en tramos: con el quad entero (3 m) la profundidad media del
    # pintor quedaba por delante de la madera del cajón pegada a ella y la
    # tapaba (CCW: el rosa del INICIO caía de 0.29 a 0.22). Los cortes caen
    # también en los cantos de las maderas, si no el tramo que las cruza
    # sigue tapando una esquina.
    for axis, value, a0, a1, b0, b1 in cam._wall_quads():
        n = max(1, int(math.ceil((a1 - a0) / _WALL_TILE_MM)))
        cuts = {a0 + k * (a1 - a0) / n for k in range(n + 1)}
        for box in track.barriers:
            for pt in box:
                if a0 < pt[1 - axis] < a1:
                    cuts.add(pt[1 - axis])
        cuts = sorted(cuts)
        for c0, c1 in zip(cuts[:-1], cuts[1:]):
            corners = cam._quad_corners(axis, value, c0, c1, b0, b1)
            cam_pts = _to_cam(cam, corners, ox, oy, oz, sh, ch)
            if np.all(cam_pts[:, 2] < 8.0):
                continue
            faces.append((float(np.mean(cam_pts[:, 2])), cam_pts, WALL_BGR))
    half = SIGN_MM / 2.0
    for sign in track.signs:
        color = RED_BGR if sign.color == "Red" else GREEN_BGR
        for corners in cam._box_quads(
            sign.x - half,
            sign.x + half,
            sign.y - half,
            sign.y + half,
            p.sign_height_mm,
        ):
            cam_pts = _to_cam(cam, corners, ox, oy, oz, sh, ch)
            faces.append((float(np.mean(cam_pts[:, 2])), cam_pts, color))
    for box in track.barriers:
        xs = [pt[0] for pt in box]
        ys = [pt[1] for pt in box]
        for corners in cam._box_quads(
            min(xs), max(xs), min(ys), max(ys), p.sign_height_mm
        ):
            cam_pts = _to_cam(cam, corners, ox, oy, oz, sh, ch)
            faces.append((float(np.mean(cam_pts[:, 2])), cam_pts, MAGENTA_BGR))
    for _depth, cam_pts, color in sorted(faces, key=lambda item: item[0], reverse=True):
        _fill_face(cam, img, cam_pts, color)
    return img
