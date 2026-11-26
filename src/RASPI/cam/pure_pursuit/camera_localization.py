"""Mediciones geométricas de cámara en el marco (derecha, adelante)."""

import math

import cv2
import numpy as np

from . import config as C
from .digital_map import feet_world


def _ground(points, scale=1):
    return (np.asarray(points) * scale - [C.ROBOT_BEV_X, C.ROBOT_BEV_Y]) * [
        C.MM_PER_PX, -C.MM_PER_PX] + [0, C.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM]


def wall_edges(frame, bev):
    """Devuelve (distancia, ángulo de la normal hacia el negro, extremos).

    Busca transiciones de piso claro a pared negra en una BEV gris a media
    resolución. El relleno neutro evita tratar el borde de la proyección
    como pared.
    """
    transform = np.diag([0.5, 0.5, 1.0]) @ bev.H
    gray = cv2.warpPerspective(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), transform,
                               (C.BEV_W // 2, C.BEV_H // 2), borderValue=95)
    dark = cv2.inRange(gray, 0, 72)
    edges = cv2.Canny(dark, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 23, minLineLength=30, maxLineGap=6)
    if lines is None:
        return []
    segments = lines.reshape(-1, 2, 2)
    lengths = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1)
    walls = []
    for index in np.argsort(-lengths)[:8]:
        endpoints = segments[index]
        dx, dy = (endpoints[1] - endpoints[0]) / lengths[index]
        normal = np.array([-dy, dx])
        samples = np.linspace(*endpoints, num=11)[1:-1]
        sides = np.rint(samples[None] + np.array([-3, 3])[:, None, None] * normal).astype(int)
        x, y = sides[..., 0], sides[..., 1]
        if not ((x >= 0) & (x < gray.shape[1]) & (y >= 0) & (y < gray.shape[0])).all():
            continue
        a, b = gray[y, x]
        if np.count_nonzero((a < 73) & (b > 108)) >= 7:
            normal = -normal
        elif np.count_nonzero((b < 73) & (a > 108)) < 7:
            continue
        normal[1] *= -1
        ground = _ground(endpoints, scale=2)
        distance = float(ground.mean(axis=0) @ normal)
        if distance >= 55:
            walls.append((distance, math.atan2(*normal), ground))
    return walls


def sign_observations(obstacles):
    """Centra aproximadamente el pie de la cara visible de cada lata."""
    return [(math.hypot(right, forward), math.atan2(right, forward), color)
            for _, _, color, forward, right in feet_world(obstacles, (0, 0), 0)
            if math.hypot(right, forward) <= 750]
