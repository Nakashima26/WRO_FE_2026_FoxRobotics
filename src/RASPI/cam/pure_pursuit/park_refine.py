"""Locate both parking boards using lateral ToF returns on the parking straight."""

import math
from typing import NamedTuple

import numpy as np

from . import config as C
from .localizer import _ack_num
from .start_fix import StartFix
from .wro_field import OUTER_HALF_MM, PARK_BARRIER_INTO_MM, PARK_BARRIER_THICK_MM, PARK_GAP_MM, w_axis, world_to_section


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

    def update(self, tc, ack, pose, odom):
        """Project a fresh lateral return from the pose associated with its ACK."""
        if not 3 <= tc <= 12 or odom is None or odom == self.last:
            return False
        self.last = odom
        h, w = world_to_section(self.section, *pose[:2])
        angle = math.radians(pose[2])
        progress = self.sign * float(self.axis @ [math.sin(angle), math.cos(angle)])
        if not 215 <= h <= 810 or progress < 0.86 or not 780 <= w <= 2230:
            return False
        key, mount = ("tR", "tof_right") if self.direction == "CCW" else ("tL", "tof_left")
        distance = _ack_num(ack, key)
        if distance is None or not 3 <= distance <= 640:
            return False
        origin, heading = StartFix._sensor(pose, C.SENSOR_MOUNTS[mount])
        ray = np.array([math.sin(math.radians(heading)), math.cos(math.radians(heading))])
        depth, along = world_to_section(self.section, *(origin + distance * ray))
        # Long returns often hit the board before the tip or underestimate
        # the range. Pairing and repeatability distinguish boards from clutter.
        if not PARK_BARRIER_INTO_MM - 100 <= depth <= PARK_BARRIER_INTO_MM + 95:
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
        centers_by_lap = {}
        for lap, readings in self.readings.items():
            groups = []
            for along, distance in sorted(readings):
                if not groups or along - groups[-1][0][0] > 85:
                    groups.append([])
                groups[-1].append((along, distance))
            centers = [(float(np.median([w for w, _ in group])),
                        min(d for _, d in group)) for group in groups]
            centers_by_lap[lap] = centers
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
        # Prefer the slow final pass when it has a complete close pair.
        final = centers_by_lap.get(3, ())
        close = [(a, b) for a, b, d in votes if d <= 470]
        a, b = np.median(close or [(a, b) for a, b, _ in votes], axis=0)
        final_close = [(f, d) for f, d in final if d <= 190]
        pairs = [(f, g) for i, (f, _) in enumerate(final_close)
                 for g, _ in final_close[i + 1:] if spacing - 45 <= g - f <= spacing + 80]
        if len(pairs) == 1 and max(abs(pairs[0][0] - a), abs(pairs[0][1] - b)) <= 150:
            a, b = pairs[0]
        elif len(final_close) == 1:
            f, _ = final_close[0]
            # The second board can be the only return on the slow pass.
            # Match it with the first board of a previously confirmed pair.
            if self.sign > 0:
                near = [g for lap, centers in centers_by_lap.items() if lap < 3
                        for g, _ in centers if abs(g - a) <= 65]
                if len(near) >= 2 and spacing - 45 <= f - np.median(near) <= spacing + 45:
                    b = f
            else:
                near = [g for lap, centers in centers_by_lap.items() if lap < 3
                        for g, _ in centers if abs(g - b) <= 65]
                if len(near) >= 2 and spacing - 45 <= np.median(near) - f <= spacing + 45:
                    a = f
        w = a - thickness / 2
        far = b + thickness / 2 - OUTER_HALF_MM if self.sign > 0 else OUTER_HALF_MM - w
        return ParkingEstimate(round(w), round(far), len(votes))
