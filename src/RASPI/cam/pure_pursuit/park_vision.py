"""Referencia longitudinal del cajón a partir del pie de las dos maderas."""

import math
from collections import deque

import cv2
import numpy as np

from . import config as C
from .digital_map import feet_world
from .wro_field import (
    OUTER_HALF_MM, PARK_BARRIER_THICK_MM, PARK_GAP_MM, SEATS, _heading, world_to_section,
)


class ParkVision:
    def __init__(self, match_mm=90):
        self.match_mm = match_mm
        self.samples = (deque(maxlen=12), deque(maxlen=12))
        self.estimate = None

    def update(self, mask, bev, parking, direction, pose, far_along):
        if mask is None or not bev.is_calibrated:
            return
        yaw = math.radians(pose[2])
        lane = math.radians(_heading(parking, direction))
        if math.cos(yaw - lane) < 0.96:
            return
        h, w = world_to_section(parking, *pose[:2])
        along = (w - OUTER_HALF_MM) * (1 if direction == "CW" else -1)
        if not 225 < h < 650 or not 150 < far_along - along < 1150:
            return

        # Cada columna del borde inferior aporta un píxel de contacto con el
        # piso. Ajustar las caras transversales en el BEV, no el bbox del color.
        found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        thickness, gap = PARK_BARRIER_THICK_MM, PARK_GAP_MM
        for contour in found:
            if cv2.contourArea(contour) < 120:
                continue
            uv = contour[:, 0]
            u, v = uv.T
            bottom = np.full(mask.shape[1], -1, dtype=np.int32)
            np.maximum.at(bottom, u, v)
            columns = np.flatnonzero((bottom > 0) & (bottom < mask.shape[0] - 2))
            columns = columns[(columns > 1) & (columns < mask.shape[1] - 2)]
            if len(columns) < 12:
                continue
            image = np.column_stack((columns, bottom[columns])).astype(np.float32)
            points = cv2.perspectiveTransform(image[None], bev.H)[0]
            fwd = (C.ROBOT_BEV_Y - points[:, 1]) * C.MM_PER_PX + C.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM
            side = (points[:, 0] - C.ROBOT_BEV_X) * C.MM_PER_PX
            angle = yaw - lane
            wood_along = along + fwd * math.cos(angle) - side * math.sin(angle)
            depth = h + (fwd * math.sin(angle) + side * math.cos(angle)) * (1 if direction == "CW" else -1)
            ok = (fwd >= 160) & (fwd <= 720) & (depth >= 35) & (depth <= 190)
            for i, expected in enumerate((far_along - 2 * thickness - gap, far_along - thickness)):
                selected = ok & (np.abs(wood_along - expected) <= self.match_mm)
                x, y = depth[selected], wood_along[selected]
                if len(x) < 9 or np.ptp(x) < 35:
                    continue
                slope, offset = np.polyfit(x - 150, y, 1)
                residual = y - slope * (x - 150) - offset
                if abs(slope) > 0.16 or np.median(np.abs(residual)) > 12:
                    continue
                bucket = self.samples[i]
                if not bucket or abs(along - bucket[-1][0]) >= 12:
                    bucket.append((along, offset - expected))

        if min(map(len, self.samples)) < 3:
            return
        innovations = [np.array(bucket)[:, 1] for bucket in self.samples]
        medians = [float(np.median(values)) for values in innovations]
        if abs(medians[0] - medians[1]) > 22:
            return
        if any(np.median(np.abs(values - center)) > 15 for values, center in zip(innovations, medians)):
            return
        estimate = far_along + 0.5 * sum(medians)
        if self.estimate is None:
            print(f"[PARKREF BEV] along={estimate:.1f} prior={far_along:.1f}", flush=True)
        self.estimate = estimate


class ParkLandmark:
    """Desplazamiento longitudinal observado respecto a un asiento confirmado."""

    def __init__(self):
        self.samples = deque(maxlen=6)

    def update(self, feet, confirmed, parking, direction, pose):
        lane = _heading(parking, direction)
        if abs((pose[2] - lane + 180) % 360 - 180) > 12:
            self.samples.clear()
            return None
        sign = 1 if direction == "CW" else -1
        h, w = world_to_section(parking, *pose[:2])
        if not 220 < h < 650 or not 610 < w < 2390:
            self.samples.clear()
            return None
        seats = [(sid, *SEATS[sid], color) for (section, sid), color in confirmed.items()
                 if section == parking and sid in SEATS]
        observations = []
        for x, y, color, fwd, _ in feet_world(feet, pose[:2], pose[2]):
            if not 230 < fwd < 680:
                continue
            depth, observed = world_to_section(parking, x, y)
            matches = [(sid, (known - observed) * sign) for sid, seat_h, known, expected in seats
                       if color == expected and abs(depth - seat_h) <= 55 and abs(known - observed) <= 65]
            if len(matches) == 1:
                observations.append(matches[0])
        along = w * sign
        if len(observations) == 1:
            sid, error = observations[0]
            if self.samples and (sid != self.samples[-1][0] or along - self.samples[-1][1] > 200):
                self.samples.clear()
            if not self.samples or along - self.samples[-1][1] >= 16:
                self.samples.append((sid, along, error))
        if len(self.samples) < 4 or not 0 <= along - self.samples[-1][1] <= 170:
            return None
        innovations = [error for _, _, error in self.samples]
        if max(innovations) - min(innovations) > 24:
            return None
        return float(np.median(innovations))
