"""Locate both parking boards using lateral ToF returns on the parking straight."""

import math
from typing import NamedTuple

import numpy as np

from . import config as C
from .localizer import _ack_num
from .start_fix import StartFix
from .wro_field import PARK_BARRIER_INTO_MM, PARK_BARRIER_THICK_MM, PARK_GAP_MM, w_axis, world_to_section


class ParkingEstimate(NamedTuple):
    w: float
    far_along: float
    passes: int


class ParkRefine:
    def __init__(self, section, direction, prior):
        self.section = section
        self.direction = direction
        self.prior = prior
        self.axis = np.array(w_axis(section))
        self.sign = 1 if direction == "CW" else -1
        self.readings = {}
        self.last = None

    def update(self, tc, ack, pose, loc):
        """Project a fresh lateral return from the pose associated with its ACK."""
        if not 3 <= tc <= 12 or loc.odom is None or loc.odom == self.last:
            return False
        self.last = loc.odom
        h, w = world_to_section(self.section, *pose[:2])
        angle = math.radians(pose[2])
        progress = self.sign * float(self.axis @ [math.sin(angle), math.cos(angle)])
        if not 215 <= h <= 810 or progress < 0.86 or not 780 <= w <= 2230:
            return False
        key, mount = ("tR", "tof_right") if self.direction == "CCW" else ("tL", "tof_left")
        distance = _ack_num(ack, key)
        if distance is None or not 10 <= distance <= 640:
            return False
        origin, heading = StartFix._sensor(pose, C.SENSOR_MOUNTS[mount])
        ray = np.array([math.sin(math.radians(heading)), math.cos(math.radians(heading))])
        depth, along = world_to_section(self.section, *(origin + distance * ray))
        # Long returns often hit the board before the tip or underestimate
        # the range. Pairing and repeatability distinguish boards from clutter.
        if not PARK_BARRIER_INTO_MM - 100 <= depth <= PARK_BARRIER_INTO_MM + 80:
            return False
        if not self.prior - 330 <= along <= self.prior + PARK_GAP_MM + 330:
            return False
        lap = (int(tc) + 1) // 4
        self.readings.setdefault(lap, []).append((along, distance))
        return True

    def estimate(self):
        """Require a consistent pair, with extra corroboration for long returns."""
        thickness = PARK_BARRIER_THICK_MM
        spacing = PARK_GAP_MM + thickness
        votes = []
        for readings in self.readings.values():
            groups = []
            for along, distance in sorted(readings):
                if not groups or along - groups[-1][0][0] > 85:
                    groups.append([])
                groups[-1].append((along, distance))
            centers = [(float(np.median([w for w, _ in group])),
                        min(d for _, d in group)) for group in groups]
            candidates = [(a, b, max(da, db)) for i, (a, da) in enumerate(centers)
                          for b, db in centers[i + 1:]
                          if spacing - 45 <= b - a <= spacing + 80]
            if len(candidates) == 1:
                votes.append(candidates[0])
        if not votes:
            return None
        a0, b0 = np.median([(a, b) for a, b, _ in votes], axis=0)
        votes = [(a, b, d) for a, b, d in votes if abs(a - a0) <= 65 and abs(b - b0) <= 65]
        if not votes or (len(votes) < 2 and votes[0][2] > 470):
            return None
        # Use close pairs in preference to long, oblique returns.
        close = [(a, b) for a, b, d in votes if d <= 470]
        a, b = np.median(close or [(a, b) for a, b, _ in votes], axis=0)
        w = a - thickness / 2
        # The far edge uses the measured second board. The nominal gap is
        # only a pairing constraint and cannot shift the measured edge.
        far = b + thickness / 2 - 1500 if self.sign > 0 else 1500 - w
        return ParkingEstimate(round(w), round(far), len(votes))
