"""Campo WRO: segmentos 2D y ray casting vectorizado."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ..wro_field import INNER_HALF_MM, OUTER_HALF_MM, SIGN_MM

if TYPE_CHECKING:
    from ..wro_field import Field

MaterialName = str


@dataclass
class Segment:
    a: np.ndarray
    b: np.ndarray
    material: MaterialName
    normal: np.ndarray
    object_index: int


def robot_to_world(
    right_mm: float,
    forward_mm: float,
    robot_x: float,
    robot_y: float,
    heading_deg: float,
) -> tuple[float, float]:
    h = math.radians(heading_deg)
    x = robot_x + forward_mm * math.sin(h) + right_mm * math.cos(h)
    y = robot_y + forward_mm * math.cos(h) - right_mm * math.sin(h)
    return x, y


def body_corners(
    x: float,
    y: float,
    heading_deg: float,
    length_mm: float,
    width_mm: float,
    rear_overhang_mm: float,
) -> np.ndarray:
    """Esquinas mundo (4,2) en orden cerrado."""
    y0 = -rear_overhang_mm
    y1 = y0 + length_mm
    half_w = width_mm / 2.0
    local = np.array(
        [
            [-half_w, y0],
            [half_w, y0],
            [half_w, y1],
            [-half_w, y1],
        ],
        dtype=np.float64,
    )
    h = math.radians(heading_deg)
    sh, ch = math.sin(h), math.cos(h)
    rot = np.array([[ch, sh], [-sh, ch]], dtype=np.float64)
    world = local @ rot.T + np.array([x, y])
    return world


def _poly_edges(poly: list, material: MaterialName, object_index: int) -> list[Segment]:
    pts = np.asarray(poly, dtype=np.float64)
    if len(pts) < 2:
        return []
    segs: list[Segment] = []
    n = len(pts)
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        edge = b - a
        ln = float(np.linalg.norm(edge))
        if ln < 1e-9:
            continue
        # Normal hacia el exterior del borde (CCW polígono → derecha del edge)
        tangent = edge / ln
        normal = np.array([tangent[1], -tangent[0]], dtype=np.float64)
        segs.append(Segment(a=a, b=b, material=material, normal=normal, object_index=object_index))
    return segs


def _square_edges(half: float, material: MaterialName, object_index: int) -> list[Segment]:
    poly = [
        (half, -half),
        (half, half),
        (-half, half),
        (-half, -half),
    ]
    return _poly_edges(poly, material, object_index)


class World:
    """Geometría 2D del tapete para sensores y choques."""

    def __init__(self, field: Field) -> None:
        self.field = field
        self.segments: list[Segment] = []
        idx = 0
        self.segments.extend(_square_edges(OUTER_HALF_MM, "wall", idx))
        idx += 1
        self.segments.extend(_square_edges(INNER_HALF_MM, "wall", idx))
        idx += 1
        for bi, box in enumerate(field.barriers):
            self.segments.extend(_poly_edges(box, "magenta", 100 + bi))
        half = SIGN_MM / 2.0
        for si, sign in enumerate(field.signs):
            box = [
                (sign.x - half, sign.y - half),
                (sign.x + half, sign.y - half),
                (sign.x + half, sign.y + half),
                (sign.x - half, sign.y + half),
            ]
            self.segments.extend(_poly_edges(box, "sign", 200 + si))

        self._a = np.stack([s.a for s in self.segments], axis=0)
        self._b = np.stack([s.b for s in self.segments], axis=0)
        self._normals = np.stack([s.normal for s in self.segments], axis=0)
        self._materials = [s.material for s in self.segments]
        self._obj_idx = np.array([s.object_index for s in self.segments], dtype=np.int32)

    def cast(
        self,
        origins: np.ndarray,
        dirs: np.ndarray,
        max_range: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        origins (N,2), dirs (N,2) unitarios.
        Retorna dist (N,), material index (N,) int, hit normal (N,2), object index (N,).
        """
        origins = np.asarray(origins, dtype=np.float64)
        dirs = np.asarray(dirs, dtype=np.float64)
        n_rays = origins.shape[0]
        n_segs = len(self.segments)

        best_t = np.full(n_rays, np.inf, dtype=np.float64)
        best_mat = np.full(n_rays, -1, dtype=np.int32)
        best_n = np.zeros((n_rays, 2), dtype=np.float64)
        best_obj = np.full(n_rays, -1, dtype=np.int32)

        mat_to_i = {"wall": 0, "magenta": 1, "sign": 2}

        # Vectorized: shape (n_segs, 2) for segment endpoints
        a_all = self._a  # (n_segs, 2)
        b_all = self._b  # (n_segs, 2)
        v2_all = b_all - a_all  # (n_segs, 2)

        # Perpendicular to each ray direction: (n_rays, 2)
        v3 = np.stack([-dirs[:, 1], dirs[:, 0]], axis=1)  # (n_rays, 2)

        # Vectorize over all rays and segments: (n_rays, n_segs)
        # denom[ray, seg] = v2[seg] . v3[ray]
        denom = np.dot(v2_all, v3.T)  # (n_segs, n_rays) -> transpose to (n_rays, n_segs)
        denom = denom.T  # Now (n_rays, n_segs)

        # v1[ray, seg] = origins[ray] - a[seg]
        # Broadcasting: origins (n_rays, 2), a_all (n_segs, 2)
        # Reshape for broadcasting: origins (n_rays, 1, 2) - a_all (1, n_segs, 2) = (n_rays, n_segs, 2)
        v1 = origins[:, np.newaxis, :] - a_all[np.newaxis, :, :]  # (n_rays, n_segs, 2)

        # cross[ray, seg] = v2[seg] x v1[ray, seg]
        cross = v2_all[np.newaxis, :, 0] * v1[:, :, 1] - v2_all[np.newaxis, :, 1] * v1[:, :, 0]  # (n_rays, n_segs)

        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = np.where(np.abs(denom) > 1e-12, cross / denom, np.inf)  # (n_rays, n_segs)
            # dot_v1_v3[ray, seg] = v1[ray, seg] . v3[ray]
            dot_v1_v3 = v1[:, :, 0] * v3[:, np.newaxis, 0] + v1[:, :, 1] * v3[:, np.newaxis, 1]  # (n_rays, n_segs)
            t2 = np.where(np.abs(denom) > 1e-12, dot_v1_v3 / denom, np.inf)  # (n_rays, n_segs)

        # hit[ray, seg]: whether this ray hits this segment
        hit = (t1 >= 0.0) & (t1 <= max_range) & (t2 >= 0.0) & (t2 <= 1.0)  # (n_rays, n_segs)
        t_hit = np.where(hit, t1, np.inf)  # (n_rays, n_segs)

        # For each ray, find closest segment
        closest_seg = np.argmin(t_hit, axis=1)  # (n_rays,) - first on ties
        closest_t = t_hit[np.arange(n_rays), closest_seg]  # (n_rays,)

        # Update best values where hit
        closer = closest_t < best_t
        best_t = np.where(closer, closest_t, best_t)

        # Material index, object index, normal for closest segment
        mat_names = np.array(self._materials)  # (n_segs,)
        best_mat_vals = np.array([mat_to_i.get(m, 0) for m in mat_names])  # (n_segs,) -> int
        best_mat = np.where(closer, best_mat_vals[closest_seg], best_mat)

        best_obj = np.where(closer, self._obj_idx[closest_seg], best_obj)

        # Normal from closest segment
        closest_normals = self._normals[closest_seg]  # (n_rays, 2)
        best_n[closer] = closest_normals[closer]

        dist = np.where(np.isfinite(best_t), best_t, max_range)
        no_hit = best_mat < 0
        dist = np.where(no_hit, max_range, dist)
        return dist, best_mat, best_n, best_obj

    def material_name(self, mat_index: int) -> str:
        return ("wall", "magenta", "sign")[mat_index] if 0 <= mat_index < 3 else "wall"


# ── Choque (misma semántica que chassis_twin.collision) ─────────────────────


def _segments_cross(a, b, c, d) -> bool:
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1 = cross(c, d, a)
    d2 = cross(c, d, b)
    d3 = cross(a, b, c)
    d4 = cross(a, b, d)
    return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and (
        (d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)
    )


def _point_in_island(x: float, y: float) -> bool:
    return abs(x) < INNER_HALF_MM and abs(y) < INNER_HALF_MM


def _point_outside_field(x: float, y: float) -> bool:
    return abs(x) > OUTER_HALF_MM or abs(y) > OUTER_HALF_MM


def _point_in_poly(px: float, py: float, poly) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (
            px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def _hits_polygon(corners, poly) -> bool:
    if any(_point_in_poly(x, y, poly) for x, y in corners):
        return True
    if any(_point_in_poly(x, y, corners) for x, y in poly):
        return True
    edges = list(zip(corners, corners[1:] + corners[:1]))
    other = list(zip(poly, poly[1:] + poly[:1]))
    return any(_segments_cross(a, b, c, d) for a, b in edges for c, d in other)


def collision(
    world: World,
    x: float,
    y: float,
    heading_deg: float,
    length_mm: float,
    width_mm: float,
    rear_overhang_mm: float,
) -> str | None:
    corners = body_corners(x, y, heading_deg, length_mm, width_mm, rear_overhang_mm)
    corner_list = [tuple(c) for c in corners]
    for cx, cy in corner_list:
        if _point_outside_field(cx, cy):
            return "pared exterior"
        if _point_in_island(cx, cy):
            return "isla"
    edges = list(zip(corner_list, corner_list[1:] + corner_list[:1]))
    track = world.field
    for poly in (track.outer, track.inner):
        wall = [(float(p[0]), float(p[1])) for p in poly]
        wall_edges = list(zip(wall, wall[1:] + wall[:1]))
        for a, b in edges:
            for c, d in wall_edges:
                if _segments_cross(a, b, c, d):
                    return "pared"
    half = SIGN_MM / 2.0
    for sign in track.signs:
        box = [
            (sign.x - half, sign.y - half),
            (sign.x + half, sign.y - half),
            (sign.x + half, sign.y + half),
            (sign.x - half, sign.y + half),
        ]
        if _hits_polygon(corner_list, box):
            return f"señal {sign.color} {sign.section}/{sign.seat}"
    for box in track.barriers:
        if _hits_polygon(corner_list, box):
            return "cajón magenta"
    return None
