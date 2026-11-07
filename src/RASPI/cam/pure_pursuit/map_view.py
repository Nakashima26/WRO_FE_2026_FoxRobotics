"""
Vista 2D del mapa (campo + señales + robot). Fondo estático cacheado.
"""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from . import config as C
from .track_map import (
    FieldPose,
    SignMap,
    TrackGeometry,
    field_heading_to_twin,
)
from .wro_field import OUTER_HALF_MM

MM_PER_PX = getattr(C, "MM_PER_PX", 2.0)
# Fuente única: config.py (los 210x140 del README están obsoletos).
ROBOT_LEN_MM = C.ROBOT_LENGTH_MM
ROBOT_WID_MM = C.ROBOT_WIDTH_MM
CAM_FOV_DEG = getattr(C, "CAM_FOV_DEG", 62.0)
BEV_W = getattr(C, "BEV_W", 400)
BEV_H = getattr(C, "BEV_H", 400)

_BG_CACHE: dict[tuple, np.ndarray] = {}


def _world_to_img(x: float, y: float, size: int, scale: float) -> tuple[int, int]:
    cx = cy = size // 2
    return int(round(cx + x * scale)), int(round(cy - y * scale))


def _build_static(geometry: TrackGeometry, size: int) -> tuple[np.ndarray, float]:
    key = (geometry.drive_dir, geometry.parking_section, size)
    if key in _BG_CACHE:
        scale = (size - 20) / (2 * OUTER_HALF_MM)
        return _BG_CACHE[key].copy(), scale
    scale = (size - 20) / (2 * OUTER_HALF_MM)
    img = np.full((size, size, 3), (245, 240, 230), dtype=np.uint8)

    def draw_poly(poly, color, thick=1):
        pts = np.array([_world_to_img(p[0], p[1], size, scale) for p in poly], np.int32)
        cv2.polylines(img, [pts], True, color, thick, cv2.LINE_AA)

    draw_poly(geometry.walls_outer, (40, 40, 40), 2)
    draw_poly(geometry.walls_inner, (40, 40, 40), 2)
    for a, b in geometry.orange_lines():
        p1 = _world_to_img(a[0], a[1], size, scale)
        p2 = _world_to_img(b[0], b[1], size, scale)
        cv2.line(img, p1, p2, (0, 140, 255), 1, cv2.LINE_AA)
    for a, b in geometry.blue_lines():
        p1 = _world_to_img(a[0], a[1], size, scale)
        p2 = _world_to_img(b[0], b[1], size, scale)
        cv2.line(img, p1, p2, (255, 120, 0), 1, cv2.LINE_AA)
    for seat in geometry.seats_all():
        px, py = _world_to_img(seat.x_mm, seat.y_mm, size, scale)
        cv2.circle(img, (px, py), 2, (160, 160, 160), -1, cv2.LINE_AA)
    _BG_CACHE[key] = img.copy()
    return img, scale


def _draw_robot(
    img: np.ndarray,
    pose: FieldPose,
    size: int,
    scale: float,
) -> None:
    h_twin = field_heading_to_twin(pose.heading_deg)
    h = math.radians(h_twin)
    cx, cy = pose.x_mm, pose.y_mm
    half_w = ROBOT_WID_MM / 2
    half_l = ROBOT_LEN_MM / 2
    corners = [
        (-half_w, -half_l),
        (half_w, -half_l),
        (half_w, half_l),
        (-half_w, half_l),
    ]
    pts = []
    for r, f in corners:
        wx = cx + f * math.sin(h) + r * math.cos(h)
        wy = cy + f * math.cos(h) - r * math.sin(h)
        pts.append(_world_to_img(wx, wy, size, scale))
    cv2.fillPoly(img, [np.array(pts, np.int32)], (80, 180, 80), cv2.LINE_AA)
    tip_x = cx + half_l * math.sin(h)
    tip_y = cy + half_l * math.cos(h)
    c0 = _world_to_img(cx, cy, size, scale)
    c1 = _world_to_img(tip_x, tip_y, size, scale)
    cv2.arrowedLine(img, c0, c1, (0, 80, 0), 2, tipLength=0.25)


def _bev_corners_field(pose: FieldPose) -> list[tuple[float, float]]:
    h_twin = field_heading_to_twin(pose.heading_deg)
    h = math.radians(h_twin)
    out = []
    for bx, by in (
        (0, 0),
        (BEV_W, 0),
        (BEV_W, BEV_H),
        (0, BEV_H),
    ):
        right = (bx - C.ROBOT_BEV_X) * MM_PER_PX
        fwd = (C.ROBOT_BEV_Y - by) * MM_PER_PX + getattr(
            C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0
        )
        wx = pose.x_mm + fwd * math.sin(h) + right * math.cos(h)
        wy = pose.y_mm + fwd * math.cos(h) - right * math.sin(h)
        out.append((wx, wy))
    return out


def render_map(
    geometry: TrackGeometry,
    signmap: SignMap | None,
    pose_field: FieldPose,
    size: int = 400,
    detections: list[tuple[float, float, str]] | None = None,
    show_fov: bool = True,
    show_bev: bool = True,
    path: list[tuple[float, float]] | None = None,
    status: dict[str, Any] | None = None,
) -> np.ndarray:
    img, scale = _build_static(geometry, size)
    if path:
        pts = [_world_to_img(x, y, size, scale) for x, y in path]
        if len(pts) >= 2:
            cv2.polylines(img, [np.array(pts, np.int32)], False, (200, 100, 50), 1)

    if signmap is not None:
        for si in range(4):
            for s in signmap.seats(si):
                px, py = _world_to_img(s["x"], s["y"], size, scale)
                col = s["color"]
                st = s["state"]
                if st == "empty":
                    cv2.circle(img, (px, py), 5, (140, 140, 140), 1, cv2.LINE_AA)
                elif col == "red":
                    inten = min(255, int(80 + 40 * s["score"]))
                    cv2.circle(img, (px, py), 5, (0, 0, inten), -1, cv2.LINE_AA)
                elif col == "green":
                    inten = min(255, int(80 + 40 * s["score"]))
                    cv2.circle(img, (px, py), 5, (0, inten, 0), -1, cv2.LINE_AA)
        for tr in signmap.to_dict().get("free", []):
            px, py = _world_to_img(tr["x"], tr["y"], size, scale)
            cv2.drawMarker(img, (px, py), (100, 100, 200), cv2.MARKER_CROSS, 6, 1)

    _draw_robot(img, pose_field, size, scale)

    if show_bev:
        poly = _bev_corners_field(pose_field)
        pts = np.array([_world_to_img(x, y, size, scale) for x, y in poly], np.int32)
        cv2.polylines(img, [pts], True, (180, 120, 60), 1, cv2.LINE_AA)

    if show_fov:
        h_twin = field_heading_to_twin(pose_field.heading_deg)
        h = math.radians(h_twin)
        cam_a = getattr(C, "SIGN_CAMERA_AHEAD_MM", 60.0)
        ox = pose_field.x_mm + cam_a * math.sin(h)
        oy = pose_field.y_mm + cam_a * math.cos(h)
        half = math.radians(CAM_FOV_DEG / 2)
        r = 700.0
        for ang in (h - half, h + half):
            ex = ox + r * math.sin(ang)
            ey = oy + r * math.cos(ang)
            p0 = _world_to_img(ox, oy, size, scale)
            p1 = _world_to_img(ex, ey, size, scale)
            cv2.line(img, p0, p1, (120, 80, 200), 1, cv2.LINE_AA)

    if detections:
        h_twin = field_heading_to_twin(pose_field.heading_deg)
        h = math.radians(h_twin)
        for bx, by, _c in detections:
            right = (bx - C.ROBOT_BEV_X) * MM_PER_PX
            fwd = (C.ROBOT_BEV_Y - by) * MM_PER_PX + getattr(
                C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0
            )
            wx = pose_field.x_mm + fwd * math.sin(h) + right * math.cos(h)
            wy = pose_field.y_mm + fwd * math.cos(h) - right * math.sin(h)
            px, py = _world_to_img(wx, wy, size, scale)
            cv2.circle(img, (px, py), 3, (255, 0, 255), -1, cv2.LINE_AA)

    if status:
        sec = status.get("section", "?")
        lat = status.get("lateral", 0)
        along = status.get("along", 0)
        src = status.get("odom_source", "")
        qual = status.get("odom_quality", "")
        fusion = status.get("fusion", "")
        txt = f"S{sec} a={along:.0f} l={lat:.0f} {src}/{qual} {fusion}"
        cv2.putText(img, txt, (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 20, 20), 1)

    return img
