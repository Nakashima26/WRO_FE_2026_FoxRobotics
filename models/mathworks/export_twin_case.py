"""Export a WRO field and Python twin reference for the Simulink plant."""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/RASPI/cam"))

from pure_pursuit.twin.native_vehicle import NativeVehicle, _FIELDS
from pure_pursuit.twin.params import TwinParams, max_wheel_deg
from pure_pursuit.twin.sensors import Encoder
from pure_pursuit.twin.vehicle import Vehicle
from pure_pursuit.twin.world import World
from pure_pursuit.wro_field import randomize

DT = 0.001


def export_case(seed, output):
    params = TwinParams()
    field = randomize(seed, park_pos="random")
    world = World(field)
    vehicle = Vehicle(params, field.start_x, field.start_y, field.start_heading)
    native = NativeVehicle(params, vehicle.x, vehicle.y, vehicle.heading_deg)
    encoder = Encoder(params.encoder, np.random.default_rng(seed))
    ep = params.encoder
    mounts = np.array([
        (m.right_mm, m.forward_mm, m.direction_deg, limit)
        for m, limit in zip(params.tof.mounts, params.tof.max_range_mm_by_id, strict=True)
    ])

    commands, states, ranges = [], [], []
    steering_max = max_wheel_deg(params.steering)
    center = params.steering.servo_center_deg
    phases = [
        (700, center, 255, 1),
        (180, center - 25, 195, 1),
        (380, center + 25, 0, 1),
        (300, center + 25, 160, -1),
        (400, center, 0, 0),
    ]
    for count, servo, pwm, direction in phases:
        commands.extend([[servo, pwm, direction]] * count)
    commands.append(commands[-1])  # Final sample logs the state after the last update.

    for servo, pwm, direction in commands:
        states.append([*(float(getattr(vehicle, name)) for name in _FIELDS),
                       encoder._frac, encoder.count])
        heading = np.deg2rad(vehicle.heading_deg)
        sin_h, cos_h = np.sin(heading), np.cos(heading)
        right, forward, offset, limits = mounts.T
        origins = np.column_stack((vehicle.x + right*cos_h + forward*sin_h,
                                   vehicle.y - right*sin_h + forward*cos_h))
        angles = heading + np.deg2rad(offset)
        directions = np.column_stack((np.sin(angles), np.cos(angles)))
        distances, *_ = world._cast_python(origins, directions, max(limits))
        ranges.append(np.minimum(distances, limits).tolist())

        speed_before = vehicle.v
        vehicle._integrate_substep(DT, servo, pwm, direction)
        encoder.update(vehicle.v_wheel * DT, vehicle.last_wheel_deg,
                       (vehicle.v - speed_before) / DT, steering_max)

    assert any(row[7] and abs(row[3]) > abs(row[6]) for row in states)
    assert any(row[3] < 0 for row in states)
    assert any(row[-1] != 0 for row in states)
    case = {
        "field_seed": seed,
        "sample_s": DT,
        "plant_parameters": list(native._p),
        "encoder_parameters": [ep.counts_per_mm, encoder._slip_bias,
                               ep.slip_c_acc_cap, ep.slip_c_acc, ep.slip_c_scrub,
                               steering_max],
        "segments": [[*s.a, *s.b] for s in world.segments],
        "mounts": mounts.tolist(),
        "commands": commands,
        "states": states,
        "ranges": ranges,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(case, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    os.environ.setdefault("FOX_TWIN_CAST", "python")
    export_case(args.seed, args.output)
