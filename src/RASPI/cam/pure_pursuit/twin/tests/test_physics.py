"""Dinámica del vehículo."""

import math

import pytest

from pure_pursuit.twin.params import TwinParams, wheel_deg_from_servo
from pure_pursuit.twin.vehicle import Vehicle


@pytest.fixture
def params():
    return TwinParams()


def test_pwm120_forward_speed(params):
    v = Vehicle(params, 0.0, 0.0, 0.0)
    v.servo_deg = 90.0
    for _ in range(3000):
        v.step(0.01, 90.0, 120.0, 1)
    assert 310.0 < v.v < 355.0


def test_full_lock_turn_radius(params):
    v = Vehicle(params, 0.0, 0.0, 0.0)
    st = params.steering
    servo = st.servo_min_deg
    wheel = wheel_deg_from_servo(servo, st)
    L = params.vehicle.wheelbase_mm
    v.v = 80.0
    v.servo_deg = servo
    x0, y0, h0 = v.x, v.y, v.heading_deg
    for _ in range(2000):
        v.step(0.001, servo, 0, 0)
    dx = v.x - x0
    dy = v.y - y0
    chord = math.hypot(dx, dy)
    dheading = abs(v.heading_deg - h0)
    if dheading > 5.0:
        R_meas = chord / (2.0 * math.sin(math.radians(dheading / 2.0)))
        R_pred = L / math.tan(math.radians(abs(wheel)))
        assert abs(R_meas - R_pred) / R_pred < 0.15


def test_reverse_opposite_steer(params):
    v = Vehicle(params, 0.0, 0.0, 0.0)
    servo = params.steering.servo_min_deg
    v.servo_deg = 90.0
    h0 = v.heading_deg
    for _ in range(500):
        v.step(0.01, servo, 100.0, -1)
    assert v.v < -50.0
    dh_fwd = v.heading_deg - h0
    v2 = Vehicle(params, 0.0, 0.0, 0.0)
    v2.servo_deg = 90.0
    h0b = v2.heading_deg
    for _ in range(500):
        v2.step(0.01, servo, 100.0, 1)
    dh_rev = v2.heading_deg - h0b
    assert dh_fwd * dh_rev < 0


def test_coast_decays(params):
    v = Vehicle(params, 0.0, 0.0, 0.0)
    for _ in range(500):
        v.step(0.01, 90.0, 120.0, 1)
    v0 = abs(v.v)
    for _ in range(800):
        v.step(0.01, 90.0, 0, 0)
    assert abs(v.v) < v0 * 0.2
