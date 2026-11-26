"""Mide cámara y fusión sobre una recta del gemelo con verdad de terreno.

Desde src/RASPI/cam: python -m pure_pursuit.twin.tools.check_camloc
"""

import math
import time

import cv2
import numpy as np

from vision import Vision
from ... import config as C
from ...camera_localization import sign_observations, wall_edges
from ...centerline import map_obstacle_to_bev
from ...localizer import Localizer
from ...wro_field import OUTER_HALF_MM, randomize
from ..camera import CameraModel
from ..params import TwinParams


def stats(values):
    a = np.asarray(values)
    return f"n={len(a)} med={np.median(a):.2f} p95={np.percentile(a, 95):.2f}" if a.size else "n=0"


def measure():
    cv2.setNumThreads(1)
    cam, bev, _, _ = CameraModel.twin_camera_and_bev(TwinParams())
    vision = Vision(open_cam=False)
    wall_errors, angle_errors, sign_errors, bearing_errors, costs = ([] for _ in range(5))
    sign_samples, trajectory = [], []
    for seed in range(1, 7):
        field = randomize(seed)
        for sign in field.signs:
            for cardinal, range_mm, offset_deg in ((0, 220, -10), (90, 280, 0),
                                                  (180, 340, 10), (270, 400, -15)):
                heading = cardinal + offset_deg
                th = math.radians(cardinal)
                position = np.array([sign.x, sign.y]) - range_mm * np.array([math.sin(th), math.cos(th)])
                if not (600 < max(abs(position)) < 1420):
                    continue
                frame = cam.render_camera(field, *position, heading)
                _, positions = vision.detect_on(frame)
                feet = []
                for color, boxes in positions.items():
                    for box in boxes:
                        hit = map_obstacle_to_bev(bev, *box)
                        if hit is not None:
                            feet.append((*hit, color))
                observations = sign_observations(feet)
                if len(observations) != 1 or observations[0][2] != sign.color:
                    continue
                distance, angle, _ = observations[0]
                # Excluye vistas con otra lata delante en el eje de la cámara.
                others = [math.hypot(obj.x - position[0], obj.y - position[1])
                          for obj in field.signs if obj is not sign and obj.color == sign.color]
                if others and min(others) < 400:
                    continue
                sign_errors.append(abs(distance - range_mm))
                bearing_errors.append(abs(math.degrees(angle) + offset_deg))
                sign_samples.append((position, heading, observations, {(sign.section, sign.seat): sign.color}))

    field = randomize(2)
    field.signs.clear()
    field.barriers.clear()
    for index in range(61):
        x, y, heading = OUTER_HALF_MM - 240, -1050 + 32 * index, 6.0
        frame = cam.render_camera(field, x, y, heading)
        start = time.perf_counter()
        walls = wall_edges(frame, bev)
        costs.append((time.perf_counter() - start) * 1000)
        trajectory.append((x, y, heading, walls))
        east = [(d, a) for d, a, _ in walls
                if abs(math.degrees(a) - (90 - heading)) < 15]
        if east:
            distance, angle = east[0]
            wall_errors.append(abs(distance - (OUTER_HALF_MM - x)))
            angle_errors.append(abs(math.degrees(angle) - (90 - heading)))

    print("pared distancia mm", stats(wall_errors), "rumbo grados", stats(angle_errors))
    print("lata rango mm", stats(sign_errors), "bearing grados", stats(bearing_errors))
    print("pared costo ms", stats(costs))
    for offset in (80, 120, 160):
        for enabled in (False, True):
            C.LOC_CAM_WALL = enabled
            C.LOC_CAM_SIGN = enabled
            # Error inicial de abscisa + deriva acumulada de yaw y 1% encoder.
            x, y, heading, _ = trajectory[0]
            loc = Localizer({}, None, x - offset, y, heading + 4)
            truth = Localizer({}, None, x, y, heading)
            cross_track, yaw_error = [], []
            start_th = math.radians(heading)
            for i, (x, y, heading, walls) in enumerate(trajectory):
                truth.x[:2] = (x, y)
                distance = 32 * i * 1.01
                ack = f"px={distance * math.cos(start_th)},py={distance * math.sin(start_th)},yaw={-i * 0.05}"
                # Lectura ToF del mismo mapa físico, válida cuando entra en su alcance.
                for key, mount in (("tL", "tof_left"), ("tR", "tof_right")):
                    d, _, _ = truth._expect(truth.x, C.SENSOR_MOUNTS[mount], "tof")
                    if math.isfinite(d):
                        ack += f",{key}={d:.2f}"
                loc.update(ack)
                if i % 4 == 0:
                    loc.update_camera(walls, (), {})
                cross_track.append(abs(loc.x[0] - x))
                yaw_error.append(abs(math.degrees(loc.x[2]) - heading))
            print(f"sesgo={offset} cam={enabled} pared={loc.n_cam_wall} "
                  f"lateral {stats(cross_track)} final={cross_track[-1]:.1f} "
                  f"rumbo {stats(yaw_error)} final={yaw_error[-1]:.1f}")

    C.LOC_CAM_SIGN = True
    before, after = [], []
    accepted = 0
    for position, heading, observations, confirmed in sign_samples:
        loc = Localizer({}, None, position[0] + 80, position[1], heading)
        loc.P[:2, :2] = np.eye(2) * 80**2
        before.append(np.linalg.norm(loc.x[:2] - position))
        accepted += loc.update_camera((), observations, confirmed)[1]
        after.append(np.linalg.norm(loc.x[:2] - position))
    print(f"lata aceptadas={accepted} error XY antes {stats(before)} después {stats(after)}")


if __name__ == "__main__":
    measure()
