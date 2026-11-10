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
