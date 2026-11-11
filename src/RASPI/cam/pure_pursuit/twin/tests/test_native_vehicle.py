"""Native physics must preserve every state update of the Python reference."""
import dataclasses
import random
import pytest
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.vehicle import Vehicle
from pure_pursuit.twin.native_vehicle import NativeVehicle, _FIELDS

@pytest.mark.parametrize('variant', range(4))
def test_native_vehicle_exact(variant):
    p=TwinParams()
    if variant == 1:
        p.steering=dataclasses.replace(p.steering,table_right=((0,0),(20,25),(60,53)),table_left=((0,0),(30,35),(60,53)),backlash_deg=2,lag_tau_s=.04)
    elif variant == 2:
        p.motor=dataclasses.replace(p.motor,pwm0_rev=25,k_rev_mm_s_per_pwm=3,a_max_rev_mm_s2=800,coast_decel_mm_s2=300)
    elif variant == 3:
        p.motor=dataclasses.replace(p.motor,a_lat_left_mm_s2=300,a_lat_right_mm_s2=200,brake_lock_speed_mm_s=20)
    a=Vehicle(p,12,-50,27,v=700)
    b=NativeVehicle(p,12,-50,27,v=700)
    rng=random.Random(42)
    for i in range(20000):
        if i % 100 == 0:
            cmd=(rng.choice([.00001,.0005,.001]),rng.uniform(25,155),rng.choice([0,20,80,130,255]),rng.choice([-1,0,1]))
        a._integrate_substep(*cmd)
        b._integrate_substep(*cmd)
        assert tuple(getattr(a,k) for k in _FIELDS)==tuple(getattr(b,k) for k in _FIELDS), (i,cmd)

@pytest.mark.parametrize('variant', range(4))
@pytest.mark.parametrize('drain_period', [0, 1, 17])
def test_sil_native_advance_exact(variant, drain_period):
    """Whole SIL calls, including multi-step delays and sensor boundaries, agree bitwise."""
    import struct
    import numpy as np
    from pure_pursuit.twin.firmware.fw import FirmwareSIL
    from pure_pursuit.twin.sensors import Encoder
    from pure_pursuit.twin.params import max_wheel_deg

    p = TwinParams()
    if variant == 1:
        p.steering = dataclasses.replace(p.steering, backlash_deg=2, lag_tau_s=.04)
    elif variant == 2:
        p.motor = dataclasses.replace(p.motor, pwm0_rev=25, k_rev_mm_s_per_pwm=3,
                                      a_max_rev_mm_s2=800, coast_decel_mm_s2=300)
    elif variant == 3:
        p.motor = dataclasses.replace(p.motor, brake_lock_speed_mm_s=20,
                                      a_lat_left_mm_s2=300, a_lat_right_mm_s2=200)
    vehicles = [NativeVehicle(p, 12, -50, 27, v=700) for _ in range(2)]
    encoders = [Encoder(p.encoder, np.random.default_rng(42)) for _ in range(2)]
    firmwares = [FirmwareSIL() for _ in range(2)]
    from collections import deque
    histories = [deque(maxlen=20000), []]
    collisions = [[], []]
    sensor_reads = [[], []]
    last = [0., 0., 0.]  # ds, yaw, collision time
    maximum = max_wheel_deg(p.steering)
    clock, actuators = firmwares[0].enable_advance_snapshot()

    def advance(us):
        remaining = us / 1e6
        v, enc = vehicles[0], encoders[0]
        t = clock[0] / 1e6
        while remaining > 1e-12:
            sub = min(remaining, .001)
            remaining -= sub
            h0, v0 = v.heading_deg, v.v
            v._integrate_substep(sub, *actuators)
            last[0] = 0.0 + v.v * sub
            last[1] = -(v.heading_deg - h0) / sub
            enc.update(v.v_wheel * sub, v.last_wheel_deg, (v.v - v0) / sub, maximum)
            histories[0].append((t, v.x, v.y, v.heading_deg))
            if t - last[2] >= .005:
                last[2] = t
                collisions[0].append((t, v.x, v.y, v.heading_deg))

    def collide(_us):
        v = vehicles[1]
        collisions[1].append((firmwares[1].now_us / 1e6, v.x, v.y, v.heading_deg))

    def sync():
        if drain_period == 0:
            histories[1].refresh()
        elif drain_period == 1:
            histories[1].extend(firmwares[1].drain_history())

    e = firmwares[1].enable_native_vehicle(vehicles[1], encoders[1], maximum, collide, sync)

    if drain_period == 0:
        histories[1] = firmwares[1].history_view()

    def read(i, kind):
        v = vehicles[i]
        sensor_reads[i].append((kind, firmwares[i].now_us, v.x, v.y, v.heading_deg,
                               last[1] if i == 0 else e[3],
                               encoders[0].count if i == 0 else int(e[1])))
        return 0

    for i, fw in enumerate(firmwares):
        fw.set_callbacks(advance=advance if i == 0 else lambda us: None,
                         sonar=lambda sid, i=i: read(i, sid),
                         gyro=lambda i=i: read(i, 10),
                         encoder=lambda i=i: read(i, 11),
                         tof=lambda idx, i=i: read(i, 20 + idx))
        fw.push_serial2('READY')

    def bits(rows):
        return b''.join(struct.pack('!' + 'd' * len(row), *row) for row in rows)

    try:
        for fw in firmwares:
            fw.setup()
        for n in range(160):
            if n % 20 == 0:
                for fw in firmwares:
                    fw.push_serial2('V2,obs=+0.350,turn=0,state=pp_follow,prio=0,mem=0,'
                                    'pp=1,pasado=0,intr=0,inicio=0,park=0,pd=0')
            for fw in firmwares:
                fw.loop()
            assert bits([tuple(vehicles[0]._state)]) == bits([tuple(vehicles[1]._state)])
            if (n + 1) % (drain_period or 17) == 0 or n == 159:
                if drain_period not in (0, 1):
                    histories[1].extend(firmwares[1].drain_history())
                assert bits(histories[0]) == bits(histories[1])
                if drain_period == 0:
                    from pure_pursuit.twin.sim import _pose_at
                    times = [histories[0][0][0] - 1, histories[0][-1][0] + 1]
                    for row in list(histories[0])[::137]:
                        times.extend((row[0], row[0] + 0.0000001))
                    for t in times:
                        assert bits([_pose_at(histories[0], t)]) == bits([histories[1].pose_at(t)])
                if drain_period:
                    for h in histories:
                        h.clear()
            assert bits(collisions[0]) == bits(collisions[1])
            assert bits(sensor_reads[0]) == bits(sensor_reads[1])
            assert bits([(encoders[0]._frac, encoders[0].count, *last)]) == bits([
                (e[0], e[1], e[2], e[3], e[10])])
            for h in collisions + sensor_reads:
                h.clear()
    finally:
        for fw in firmwares:
            fw.close()
