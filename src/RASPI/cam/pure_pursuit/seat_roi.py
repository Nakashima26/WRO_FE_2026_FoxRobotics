"""ROI de un asiento de señal en la imagen de perspectiva y su color.

Proyección inversa de feet_world (digital_map.py): la posición del asiento en el
tapete -> (adelante, derecha) desde el eje trasero -> píxel BEV -> H_inv -> imagen.
Solo medición (ROUTE_* no la usa para conducir).
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from . import config as C
from .wro_field import SEATS, section_to_world
from vision import COLOR_RANGES

_IMG_W, _IMG_H = 640, 480      # resolución de proceso de la cámara
_SIGN_HALF_MM = 25.0           # media base de la señal (50 mm)


def _floor_px(H_inv, fwd: float, lat: float) -> tuple[float, float]:
    """(adelante, derecha) desde el eje trasero -> píxel de imagen. Inversa de feet_world."""
    bx = C.ROBOT_BEV_X + lat / C.MM_PER_PX
    by = C.ROBOT_BEV_Y - (fwd - C.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM) / C.MM_PER_PX
    pt = np.array([[[bx, by]]], dtype=np.float64)
    u, v = cv2.perspectiveTransform(pt, H_inv)[0, 0]
    return float(u), float(v)


def _sign_geom(H_inv, pose, sx: float, sy: float):
    """(u, v, w) del punto de apoyo del asiento y ancho en px, sin mirar la imagen. None si no hay geometría."""
    x, y, hd = pose
    h = math.radians(hd)
    dx, dy = sx - x, sy - y
    fwd = dx * math.sin(h) + dy * math.cos(h)          # + = adelante del eje trasero
    lat = dx * math.cos(h) - dy * math.sin(h)          # + = derecha
    cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
    rx, ry = fwd - cam, lat                            # rayo cámara -> centro
    ray = math.hypot(rx, ry)
    if ray < 1.0:
        return None
    # feet_world: centro = pie + 25 mm sobre el mismo rayo desde la cámara
    r_foot = ray - _SIGN_HALF_MM
    if r_foot <= 1.0:
        return None
    fx, fy = cam + rx * r_foot / ray, ry * r_foot / ray
    ux, uy = rx / ray, ry / ray
    px_, py_ = -uy * _SIGN_HALF_MM, ux * _SIGN_HALF_MM   # base, perpendicular al rayo
    u1, v1 = _floor_px(H_inv, fx + px_, fy + py_)
    u2, v2 = _floor_px(H_inv, fx - px_, fy - py_)
    u, v = _floor_px(H_inv, fx, fy)
    return u, v, abs(u1 - u2)


def seat_span(H_inv, pose, sx: float, sy: float):
    """(roi, w): ROI del asiento (sin recortar a la imagen si queda dentro) y ancho en px.
    None si el punto de apoyo cae fuera de la imagen o la señal mide < 3 px."""
    g = _sign_geom(H_inv, pose, sx, sy)
    if g is None:
        return None
    u, v, w = g
    if w < 3.0 or not (0.0 <= u < _IMG_W and 0.0 <= v < _IMG_H):
        return None
    x0, x1 = max(0.0, u - w / 2), min(float(_IMG_W), u + w / 2)
    y0, y1 = max(0.0, v - 2 * w), min(float(_IMG_H), v)
    return (int(x0), int(y0), int(x1), int(y1)), w


def seat_roi(H_inv, pose, sx: float, sy: float):
    """(u0, v0, u1, v1) de la señal en la imagen de perspectiva, o None."""
    span = seat_span(H_inv, pose, sx, sy)
    return None if span is None else span[0]


def classify(frame_bgr, roi):
    """("Red"|"Green"|"Empty", frac_red, frac_green) sobre la ROI."""
    x0, y0, x1, y1 = roi
    sub = frame_bgr[y0:y1, x0:x1]
    if sub.size == 0:
        return "Empty", 0.0, 0.0
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    n = float(hsv.shape[0] * hsv.shape[1])
    fr = fg = 0.0
    for color, ranges in COLOR_RANGES.items():
        mask = np.bitwise_or.reduce([cv2.inRange(hsv, lo, hi) for lo, hi in ranges])
        frac = float(np.count_nonzero(mask)) / n
        if color == "Red":
            fr = frac
        else:
            fg = frac
    thr = float(getattr(C, "SEAT_ROI_MIN_FRAC", 0.25))
    if fr >= thr and fr > fg:
        return "Red", fr, fg
    if fg >= thr and fg > fr:
        return "Green", fr, fg
    return "Empty", fr, fg


def _col_geom(H_inv, pose, sec: str, col_seats: list, margin_mm: float):
    """(bbox sin recortar, w_far, w_near) de la columna, o None si una esquina queda detrás de la cámara."""
    x, y, hd = pose
    h = math.radians(hd)
    cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
    half = _SIGN_HALF_MM + margin_mm
    w_c = SEATS[col_seats[0]][1]
    h_lo = min(SEATS[t][0] for t in col_seats) - half
    h_hi = max(SEATS[t][0] for t in col_seats) + half
    us, vs = [], []
    for hh in (h_lo, h_hi):
        for ww in (w_c - half, w_c + half):
            wx, wy = section_to_world(sec, hh, ww)
            dx, dy = wx - x, wy - y
            fwd = dx * math.sin(h) + dy * math.cos(h)
            lat = dx * math.cos(h) - dy * math.sin(h)
            if fwd <= cam:
                return None
            u, v = _floor_px(H_inv, fwd, lat)
            us.append(u)
            vs.append(v)
    ws = []
    for t in col_seats:
        sx, sy = section_to_world(sec, *SEATS[t])
        g = _sign_geom(H_inv, pose, sx, sy)
        if g is None:
            return None
        ws.append((g[1], g[2]))             # (v, w): v menor = más lejos
    w_far = min(ws)[1]
    w_near = max(ws)[1]
    return (min(us), min(vs), max(us), max(vs)), w_far, w_near


def column_window(H_inv, pose, sec: str, col_seats: list, margin_mm: float):
    """((u0, v0, u1, v1), entera) de la columna en la imagen, recortada; o None.
    entera=False si la ventana sin recortar sale de la imagen."""
    g = _col_geom(H_inv, pose, sec, col_seats, margin_mm)
    if g is None:
        return None
    (u0, v_min, u1, v1), w_far, _ = g
    v0 = v_min - 3.0 * w_far                 # altura de la señal por encima de la base más lejana
    if u1 < 0 or u0 >= _IMG_W or v1 < 0 or v0 >= _IMG_H:
        return None
    entera = u0 >= 0.0 and v0 >= 0.0 and u1 <= _IMG_W and v1 <= _IMG_H
    return (int(max(0.0, u0)), int(max(0.0, v0)), int(min(float(_IMG_W), u1)), int(min(float(_IMG_H), v1))), entera


def column_sign_area(H_inv, pose, sec: str, col_seats: list, margin_mm: float):
    """Área en px de una señal en el asiento más cercano de la columna (ancho × 2.9 × ancho)."""
    g = _col_geom(H_inv, pose, sec, col_seats, margin_mm)
    return None if g is None else g[2] * 2.9 * g[2]


def _seg_hits_square(x0: float, y0: float, x1: float, y1: float, half: float) -> bool:
    """True si el segmento (x0,y0)-(x1,y1) toca el cuadrado |x|,|y| <= half (Liang-Barsky)."""
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 + half), (dx, half - x0), (-dy, y0 + half), (dy, half - y0)):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        r = q / p
        if p < 0.0:
            if r > t1:
                return False
            t0 = max(t0, r)
        else:
            if r < t0:
                return False
            t1 = min(t1, r)
    return True


def island_hidden(pose, sec: str, col_seats: list, margin_mm: float, half_mm: float) -> bool:
    """True si la isla central (cuadrado de media half_mm) tapa algún punto de la huella de la columna.

    Rayos desde la cámara a las 4 esquinas de la huella ampliada y a su centro; la pared de la isla
    (100 mm) tapa todo lo que queda detrás de su huella con la cámara a 97 mm.
    """
    x, y, hd = pose
    h = math.radians(hd)
    cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
    cx, cy = x + math.sin(h) * cam, y + math.cos(h) * cam
    half = _SIGN_HALF_MM + margin_mm
    w_c = SEATS[col_seats[0]][1]
    h_lo = min(SEATS[t][0] for t in col_seats) - half
    h_hi = max(SEATS[t][0] for t in col_seats) + half
    pts = [section_to_world(sec, hh, ww) for hh in (h_lo, h_hi) for ww in (w_c - half, w_c + half)]
    pts.append(section_to_world(sec, (h_lo + h_hi) / 2.0, w_c))
    return any(_seg_hits_square(cx, cy, px, py, half_mm) for px, py in pts)


# auto-prueba: segmentos contra el cuadrado de media 500 (la isla del tapete)
assert _seg_hits_square(0, 0, 1000, 0, 500)          # sale del cuadrado por la derecha
assert _seg_hits_square(-1000, 100, 1000, 100, 500)  # cruza de lado a lado
assert _seg_hits_square(0, 0, 400, 0, 500)           # empieza dentro: cuenta como tocado
assert not _seg_hits_square(600, 0, 1000, 0, 500)    # queda fuera
assert not _seg_hits_square(-1000, 600, 1000, 600, 500)  # pasa por encima
assert _seg_hits_square(-1000, -1000, 1000, 1000, 500)   # diagonal por el centro
assert not _seg_hits_square(600, 600, 1000, 1000, 500)   # esquina fuera


def classify_window(frame_bgr, win, sign_px_area: float, color_ranges=None):
    """("Red"|"Green"|"Empty"|"Mixed", n_red, n_green) sobre la ventana de la columna.
    color_ranges: rangos HSV del runtime (Vision.color_ranges); None -> COLOR_RANGES."""
    x0, y0, x1, y1 = win
    sub = frame_bgr[y0:y1, x0:x1]
    if sub.size == 0 or sign_px_area <= 0:
        return "Empty", 0, 0
    ranges = color_ranges or COLOR_RANGES
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    counts = {}
    for color in ("Red", "Green"):
        mask = np.bitwise_or.reduce([cv2.inRange(hsv, lo, hi) for lo, hi in ranges[color]])
        counts[color] = int(np.count_nonzero(mask))
    nr, ng = counts["Red"], counts["Green"]
    thr = float(getattr(C, "SEAT_COL_MIN_FRAC", 0.3)) * sign_px_area
    if nr < 0.5 * thr and ng < 0.5 * thr:
        return "Empty", nr, ng
    if nr >= thr and nr > 2 * ng:
        return "Red", nr, ng
    if ng >= thr and ng > 2 * nr:
        return "Green", nr, ng
    return "Mixed", nr, ng
