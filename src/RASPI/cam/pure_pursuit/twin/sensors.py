"""Modelos de sensores (HC-SR04, ToF, encoder, gyro)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .params import EncoderParams, GyroParams, ToFParams, UltrasonicParams, max_wheel_deg
from .world import World, robot_to_world


@dataclass
class Pose:
    x: float
    y: float
    heading_deg: float


def _body_dir_world(heading_deg: float, direction_deg: float) -> np.ndarray:
    """direction_deg: 0 adelante cuerpo, +90 derecha."""
    h = math.radians(heading_deg)
    fwd = np.array([math.sin(h), math.cos(h)], dtype=np.float64)
    right = np.array([math.cos(h), -math.sin(h)], dtype=np.float64)
    a = math.radians(direction_deg)
    d = math.cos(a) * fwd + math.sin(a) * right
    n = float(np.linalg.norm(d))
    return d / n if n > 1e-12 else fwd


class Ultrasonic:
    def __init__(self, params: UltrasonicParams, rng: np.random.Generator) -> None:
        self.params = params
        self.rng = rng
        self.last_debug: dict = {}

    def echo_us(self, sensor_id: int, world: World, pose: Pose) -> int:
        p = self.params
        mount = p.mounts[sensor_id]
        ox, oy = robot_to_world(mount.right_mm, mount.forward_mm, pose.x, pose.y, pose.heading_deg)
        origin = np.array([ox, oy], dtype=np.float64)

        angles = np.arange(
            mount.direction_deg - p.cone_half_deg,
            mount.direction_deg + p.cone_half_deg + 0.5 * p.ray_step_deg,
            p.ray_step_deg,
        )
        dirs_body = np.stack(
            [_body_dir_world(0.0, a) for a in angles],
            axis=0,
        )
        # Rotar al mundo
        h = math.radians(pose.heading_deg)
        rot = np.array([[math.sin(h), math.cos(h)], [math.cos(h), -math.sin(h)]], dtype=np.float64)
        # body: x right, y forward -> world: same as robot_to_world Jacobian
        dirs = np.column_stack(
            [
                dirs_body[:, 0] * math.cos(h) + dirs_body[:, 1] * math.sin(h),
                -dirs_body[:, 0] * math.sin(h) + dirs_body[:, 1] * math.cos(h),
            ]
        )
        norms = np.linalg.norm(dirs, axis=1, keepdims=True)
        dirs = dirs / np.maximum(norms, 1e-12)

        origins = np.broadcast_to(origin, dirs.shape)
        dists, mat_i, normals, _obj = world.cast(origins, dirs, p.max_range_mm)

        valid = np.zeros(len(angles), dtype=bool)
        for i in range(len(angles)):
            if mat_i[i] < 0 or dists[i] >= p.max_range_mm - 1e-6:
                continue
            mat = world.material_name(int(mat_i[i]))
            cos_inc = abs(float(np.dot(dirs[i], normals[i])))
            cos_inc = min(1.0, max(0.0, cos_inc))
            inc_deg = math.degrees(math.acos(cos_inc))
            if mat == "sign":
                valid[i] = True
            elif inc_deg <= p.max_incidence_deg:
                valid[i] = True

        if not np.any(valid):
            self.last_debug = {"sensor_id": sensor_id, "hits": 0, "d_mm": None}
            return 0

        d_mm = float(np.min(dists[valid]))
        noise = self.rng.normal(0.0, p.noise_sigma_mm)
        d_mm = max(0.0, d_mm * (1.0 + self.rng.normal(0.0, p.noise_rel)) + noise)
        echo = int(round(2.0 * d_mm / p.sound_mm_per_us))
        self.last_debug = {"sensor_id": sensor_id, "hits": int(np.sum(valid)), "d_mm": d_mm}
        return max(0, echo)


class ToF:
    def __init__(self, params: ToFParams, rng: np.random.Generator) -> None:
        self.params = params
        self.rng = rng
        # Estado POR sensor: el firmware lee los 4 en el mismo instante; con un
        # solo reloj compartido, 1..3 devolvían la lectura retenida del 0.
        self._last_sample_t: dict[int, float] = {}
        self._held_mm: dict[int, int] = {}
        self._valid_threshold = self._calibrate_threshold()

    def _calibrate_threshold(self) -> float:
        """Señal normalizada mínima: pared negra a black_wall_max_mm, incidencia normal."""
        d = self.params.black_wall_max_mm
        sig = self.params.reflectivity["wall"] * 1.0 / (d * d)
        return sig * 0.95

    def read_mm(self, sensor_id: int, world: World, pose: Pose, t: float) -> int:
        p = self.params
        if t - self._last_sample_t.get(sensor_id, -1e9) < p.sample_period_s - 1e-9:
            return self._held_mm.get(sensor_id, -1)
        self._last_sample_t[sensor_id] = t
        if not 0 <= sensor_id < len(p.mounts):
            self._held_mm[sensor_id] = -1
            return -1
        mount = p.mounts[sensor_id]
        ox, oy = robot_to_world(mount.right_mm, mount.forward_mm, pose.x, pose.y, pose.heading_deg)
        origin = np.array([ox, oy], dtype=np.float64)

        angles = np.arange(
            mount.direction_deg - p.half_fov_deg,
            mount.direction_deg + p.half_fov_deg + 0.5 * p.ray_step_deg,
            p.ray_step_deg,
        )
        dirs_body = np.stack([_body_dir_world(0.0, a) for a in angles], axis=0)
        h = math.radians(pose.heading_deg)
        dirs = np.column_stack(
            [
                dirs_body[:, 0] * math.cos(h) + dirs_body[:, 1] * math.sin(h),
                -dirs_body[:, 0] * math.sin(h) + dirs_body[:, 1] * math.cos(h),
            ]
        )
        norms = np.linalg.norm(dirs, axis=1, keepdims=True)
        dirs = dirs / np.maximum(norms, 1e-12)
        origins = np.broadcast_to(origin, dirs.shape)
        dists, mat_i, normals, _obj = world.cast(origins, dirs, p.max_range_mm)

        clusters: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for i in range(len(angles)):
            if mat_i[i] < 0:
                continue
            d = float(dists[i])
            if d > p.max_range_mm - 1e-3:
                continue
            mat = world.material_name(int(mat_i[i]))
            cos_inc = abs(float(np.dot(dirs[i], normals[i])))
            cos_inc = min(1.0, max(0.0, cos_inc))
            refl = p.reflectivity.get(mat, 0.04)
            signal = refl * cos_inc / (d * d)
            bucket_d = int(round(d / 50.0))
            key = (mat_i[i], bucket_d)
            clusters.setdefault(key, []).append((d, signal))

        if not clusters:
            self._held_mm[sensor_id] = -1
            return -1

        best_key = max(clusters, key=lambda k: sum(s for _, s in clusters[k]))
        pts = clusters[best_key]
        total_sig = sum(s for _, s in pts)
        mean_d = sum(d * s for d, s in pts) / total_sig
        norm_sig = total_sig / len(pts)

        if norm_sig < self._valid_threshold or mean_d > p.max_range_mm:
            self._held_mm[sensor_id] = -1
            return -1

        noise = self.rng.normal(0.0, p.noise_sigma_mm)
        out = mean_d * (1.0 + self.rng.normal(0.0, p.noise_rel)) + noise
        self._held_mm[sensor_id] = int(round(max(0.0, out)))
        return self._held_mm[sensor_id]


class Encoder:
    def __init__(self, params: EncoderParams, rng: np.random.Generator) -> None:
        self.params = params
        self.rng = rng
        self.count = 0
        self._frac = 0.0
        self._slip_bias = 1.0 + float(rng.normal(0.0, params.slip_bias_sigma))

    def update(self, ds_true: float, wheel_deg: float, accel: float, steering_max_deg: float) -> None:
        p = self.params
        slip = self._slip_bias
        slip += min(p.slip_c_acc_cap, p.slip_c_acc * abs(accel))
        if steering_max_deg > 1e-6:
            slip += p.slip_c_scrub * (abs(wheel_deg) / steering_max_deg) ** 2
        slip = max(0.0, slip)
        # Acumulador fraccional: redondear cada sub-paso de 1 ms sesgaba la cuenta.
        self._frac += ds_true * p.counts_per_mm * slip
        n = int(self._frac)
        self._frac -= n
        self.count += n


class Gyro:
    def __init__(self, params: GyroParams, rng: np.random.Generator) -> None:
        self.params = params
        self.rng = rng
        self._bias = float(rng.normal(0.0, params.bias_sigma_dps))
        self._scale = 1.0 + float(rng.normal(0.0, params.scale_error_sigma))
        self._last_t: float | None = None

    def read(self, true_rate_ccw_dps: float, t: float) -> float:
        p = self.params
        if self._last_t is not None:
            dt = max(0.0, t - self._last_t)
            self._bias += float(self.rng.normal(0.0, p.bias_walk_dps_per_sqrt_s * math.sqrt(dt)))
        self._last_t = t
        noisy = true_rate_ccw_dps * self._scale + self._bias
        noisy += float(self.rng.normal(0.0, p.white_noise_dps))
        return noisy
