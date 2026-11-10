"""Deferred ToF poses preserve readings, sampling and random draws."""
import math

import numpy as np

from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.sensors import Pose, ToF
from pure_pursuit.twin.world import World
from pure_pursuit.wro_field import randomize


def test_deferred_pose_matches_eager_and_rng():
    p = TwinParams().tof
    eager_rng = np.random.default_rng(49)
    lazy_rng = np.random.default_rng(49)
    eager = ToF(p, eager_rng)
    lazy = ToF(p, lazy_rng)
    world = World(randomize(19))
    resolved = 0
    due = 0
    for i in range(500):
        t = i * .001
        pose = Pose(1250 + 50 * math.sin(i / 30), 1000 * math.sin(i / 150), i / 5)

        def get_pose():
            nonlocal resolved
            resolved += 1
            return pose

        for sensor_id in (0, 1, 2, 3, -1, 10):
            period = eager._sensor_value(p.sample_period_s_by_id, p.sample_period_s, sensor_id)
            if (0 <= sensor_id < len(p.mounts)
                    and t - eager._last_sample_t.get(sensor_id, -1e9) >= period - 1e-9):
                due += 1
            assert eager.read_mm(sensor_id, world, pose, t) == lazy.read_mm(sensor_id, world, get_pose, t)
            assert eager._last_sample_t == lazy._last_sample_t
            assert eager._held_mm == lazy._held_mm
            assert eager_rng.bit_generator.state == lazy_rng.bit_generator.state
    assert resolved == due
    assert 0 < resolved < 4 * 500


def test_held_reading_does_not_resolve_pose():
    p = TwinParams().tof
    tof = ToF(p, np.random.default_rng(4))
    world = World(randomize(7))
    first = tof.read_mm(0, world, Pose(1300, 0, 0), 0)
    state = tof.rng.bit_generator.state

    def unexpected():
        raise AssertionError('held/invalid readings must not request pose')

    assert tof.read_mm(0, world, unexpected, 0) == first
    assert tof.read_mm(-1, world, unexpected, 0) == -1
    assert tof.rng.bit_generator.state == state
