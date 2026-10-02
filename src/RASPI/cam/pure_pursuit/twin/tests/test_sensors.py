"""Sensores contra geometría conocida."""

import math

import numpy as np
import pytest

from pure_pursuit.twin.params import TwinParams, max_wheel_deg
from pure_pursuit.twin.sensors import Encoder, Gyro, Pose, ToF, Ultrasonic
from pure_pursuit.twin.world import Segment, World
from pure_pursuit.wro_field import OUTER_HALF_MM, SIGN_MM, randomize


class _WallWorld:
    """Mundo mínimo: pared norte (segmento horizontal en y constante)."""

    def __init__(self, y_wall: float):
        a = np.array([-OUTER_HALF_MM, y_wall], dtype=np.float64)
        b = np.array([OUTER_HALF_MM, y_wall], dtype=np.float64)
        edge = b - a
        tangent = edge / np.linalg.norm(edge)
        normal = np.array([tangent[1], -tangent[0]])
        self.segments = [
            Segment(a=a, b=b, material="wall", normal=normal, object_index=0),
        ]
        self._a = np.stack([s.a for s in self.segments])
        self._b = np.stack([s.b for s in self.segments])
        self._normals = np.stack([s.normal for s in self.segments])
        self._materials = ["wall"]
        self._obj_idx = np.array([0], dtype=np.int32)
        self.field = None

    def cast(self, origins, dirs, max_range):
        w = World.__new__(World)
        w.segments = self.segments
        w._a = self._a
        w._b = self._b
        w._normals = self._normals
        w._materials = self._materials
        w._obj_idx = self._obj_idx
        return w.cast(origins, dirs, max_range)

    def material_name(self, idx):
        return "wall"


class _EastWallWorld(_WallWorld):
    """Pared este en x = +OUTER_HALF_MM."""

    def __init__(self, x_wall: float = OUTER_HALF_MM):
        a = np.array([x_wall, -OUTER_HALF_MM], dtype=np.float64)
        b = np.array([x_wall, OUTER_HALF_MM], dtype=np.float64)
        edge = b - a
        tangent = edge / np.linalg.norm(edge)
        normal = np.array([tangent[1], -tangent[0]])
        self.segments = [
            Segment(a=a, b=b, material="wall", normal=normal, object_index=0),
        ]
        self._a = np.stack([s.a for s in self.segments])
        self._b = np.stack([s.b for s in self.segments])
        self._normals = np.stack([s.normal for s in self.segments])
        self._materials = ["wall"]
        self._obj_idx = np.array([0], dtype=np.int32)
        self.field = None


@pytest.fixture
def rng():
    return np.random.default_rng(42)


def test_ultrasonic_perpendicular_wall(rng):
    params = TwinParams()
    us = Ultrasonic(params.ultrasonic, rng)
    y_wall = 1000.0
    dist_target = 500.0
    forward_mount = params.ultrasonic.mounts[2].forward_mm
    y_rear = y_wall - dist_target - forward_mount
    world = _WallWorld(y_wall)
    pose = Pose(0.0, y_rear, 0.0)
    echo = us.echo_us(2, world, pose)
    expected = 2.0 * dist_target / params.ultrasonic.sound_mm_per_us
    assert echo > 0
    assert abs(echo - expected) / expected < 0.02


def test_ultrasonic_yaw30(rng):
    params = TwinParams()
    us = Ultrasonic(params.ultrasonic, rng)
    y_wall = 800.0
    dist_n = 500.0
    forward_mount = params.ultrasonic.mounts[2].forward_mm
    y_rear = y_wall - dist_n - forward_mount
    world = _WallWorld(y_wall)
    pose = Pose(0.0, y_rear, 30.0)
    echo = us.echo_us(2, world, pose)
    d_mm = echo * params.ultrasonic.sound_mm_per_us / 2.0
    expected = dist_n / math.cos(math.radians(15.0))
    assert echo > 0
    assert abs(d_mm - expected) / expected < 0.08


def test_ultrasonic_yaw50_no_echo(rng):
    params = TwinParams()
    us = Ultrasonic(params.ultrasonic, rng)
    world = _WallWorld(900.0)
    pose = Pose(0.0, 300.0, 50.0)
    assert us.echo_us(2, world, pose) == 0


def test_ultrasonic_sign_before_wall(rng):
    params = TwinParams()
    us = Ultrasonic(params.ultrasonic, rng)
    field = randomize(7)
    world = World(field)
    half = SIGN_MM / 2.0
    sign = next(s for s in field.signs if s.color == "Red")
    y_rear = sign.y - half - 300.0 - params.ultrasonic.mounts[2].forward_mm
    pose = Pose(sign.x, y_rear, 0.0)
    echo = us.echo_us(2, world, pose)
    d_mm = echo * params.ultrasonic.sound_mm_per_us / 2.0
    assert 250.0 < d_mm < 360.0


def test_tof_black_wall_range(rng):
    params = TwinParams()
    tof = ToF(params.tof, rng)
    x_wall = OUTER_HALF_MM
    world = _EastWallWorld(x_wall)
    lateral = params.tof.mounts[1].right_mm
    for dist, valid in ((200.0, True), (400.0, False)):
        x_rear = x_wall - dist - lateral
        pose = Pose(x_rear, 0.0, 0.0)
        r = tof.read_mm(1, world, pose, float(dist))
        if valid:
            assert r > 0
            assert abs(r - dist) < 40.0
        else:
            assert r == -1


def test_tof_red_sign(rng):
    params = TwinParams()
    tof = ToF(params.tof, rng)
    field = randomize(3)
    world = World(field)
    sign = next(s for s in field.signs if s.color == "Red")
    x_rear = sign.x + 400.0
    pose = Pose(x_rear, sign.y, 0.0)
    r = tof.read_mm(0, world, pose, 1.0)
    assert r > 200


def test_encoder_slip(rng):
    params = TwinParams()
    enc = Encoder(params.encoder, rng)
    wmax = max_wheel_deg(params.steering)
    ds = 100.0
    enc.update(ds, 0.0, 0.0, wmax)
    expected = ds * params.encoder.counts_per_mm
    assert abs(enc.count - expected) / expected < 0.08


def test_gyro_rate(rng):
    params = TwinParams()
    g = Gyro(params.gyro, rng)
    true_rate = 45.0
    samples = [g.read(true_rate, i * 0.01) for i in range(200)]
    assert abs(np.mean(samples) - true_rate) < 5.0
