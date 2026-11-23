"""Estimate the stall position from repeated passes of its two wooden barriers."""

import math

import numpy as np

from . import config as C
from .localizer import _ack_num
from .start_fix import StartFix
from .wro_field import (
    PARK_BARRIER_INTO_MM, PARK_BARRIER_THICK_MM, PARK_GAP_MM,
    PARK_W_MAX, PARK_W_MIN, w_axis, world_to_section,
)


class ParkRefine:
    def __init__(self, section, direction, prior):
        self.section = section
        self.direction = direction
        self.prior = prior
        self.axis = np.array(w_axis(section))
        self.readings = {}
        self.last = None

    def update(self, tc, ack, pose, loc):
        """Add a fresh ToF return projected from the corrected pose of its ACK."""
        if not 3 <= tc <= 11 or loc.odom is None or loc.odom == self.last:
            return False
        self.last = loc.odom
        h, w = world_to_section(self.section, *pose[:2])
        angle = math.radians(pose[2])
        progress = float(self.axis @ [math.sin(angle), math.cos(angle)])
        if not 215 <= h <= 700 or abs(progress) < 0.86 or not 780 <= w <= 2230:
            return False
        key, mount = ("tR", "tof_right") if self.direction == "CCW" else ("tL", "tof_left")
        distance = _ack_num(ack, key)
        if distance is None or not 20 <= distance <= 470:
            return False
        origin, heading = StartFix._sensor(pose, C.SENSOR_MOUNTS[mount])
        ray = np.array([math.sin(math.radians(heading)), math.cos(math.radians(heading))])
        depth, along = world_to_section(self.section, *(origin + distance * ray))
        if abs(depth - PARK_BARRIER_INTO_MM) > 40:
            return False
        sigma = math.sqrt(20.0 ** 2 + float(self.axis @ loc.P[:2, :2] @ self.axis))
        # The corner counter can advance between the two boards.
        lap = (int(tc) + 1) // 4
        self.readings.setdefault(lap, []).append((along, sigma))
        return True

    def estimate(self):
        """One vote per pass. Require the known separation between the woods."""
        spacing = PARK_GAP_MM + PARK_BARRIER_THICK_MM
        half = PARK_BARRIER_THICK_MM / 2
        votes = []
        for readings in self.readings.values():
            groups = []
            for w, sigma in sorted(readings):
                if not groups or w - groups[-1][0][0] > 65:
                    groups.append([])
                groups[-1].append((w, sigma))
            centers = [(float(np.median([w for w, _ in group])),
                        float(np.median([s for _, s in group]))) for group in groups]
            pairs = []
            for i, (a, sa) in enumerate(centers):
                for b, sb in centers[i + 1:]:
                    mismatch = abs(b - a - spacing)
                    w = (a + b - spacing) / 2 - half
                    if mismatch <= 35 and PARK_W_MIN - 20 <= w <= PARK_W_MAX + 20:
                        if abs(w - self.prior) <= 245:
                            pairs.append((w, max(sa, sb) ** 2 + mismatch ** 2))
            if len(pairs) == 1:
                votes.append(pairs[0])
        if not votes:
            return None
        center = float(np.median([w for w, _ in votes]))
        if len(votes) == 2 and abs(votes[0][0] - votes[1][0]) > 65:
            return None
        votes = [(w, var) for w, var in votes if abs(w - center) <= 45]
        precision = 1 / 85.0 ** 2
        value = (self.prior * precision + sum(w / var for w, var in votes)) / (
            precision + sum(1 / var for _, var in votes))
        return round(min(PARK_W_MAX, max(PARK_W_MIN, value))), len(votes)
