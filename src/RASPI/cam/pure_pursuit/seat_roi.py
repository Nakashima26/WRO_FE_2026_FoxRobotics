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


def seat_span(H_inv, pose, sx: float, sy: float):
    """(roi, w): ROI del asiento (sin recortar a la imagen si queda dentro) y ancho en px.
    None si el punto de apoyo cae fuera de la imagen o la señal mide < 3 px."""
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
    w = abs(u1 - u2)
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
