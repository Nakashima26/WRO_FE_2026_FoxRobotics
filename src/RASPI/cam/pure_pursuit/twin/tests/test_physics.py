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
    assert 260.0 < v.v < 310.0   # 12.0 V (2026-10-24); a 12.5 V eran ~315


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


def _straight(params, pwm, run_s, direction_after):
    """Recto a pwm run_s, luego freno (direction_after=1) o suelto (0). -> (v al cortar, mm real, mm encoder)."""
    v = Vehicle(params, 0.0, 0.0, 0.0, servo_deg=75.0)
    for _ in range(int(run_s / 0.001)):
        v.step(0.001, 75.0, pwm, 1)
    v_cut, y0, enc = v.v, v.y, 0.0
    for _ in range(3000):
        v.step(0.001, 75.0, 0, direction_after)
        enc += v.v_wheel * 0.001
    return v_cut, v.y - y0, enc


def test_brake_from_150_stops_short(params):
    # 2026-10-24: desde ~400 mm/s para en 6-7 mm y el encoder ve lo mismo.
    _, real, enc = _straight(params, 150, 1.5, 1)
    assert 4.0 < real < 12.0
    assert abs(real - enc) < 1.0


def test_brake_from_255_slides_past_encoder(params):
    # 2026-10-24: desde ~790 mm/s el encoder ve 13-23 mm y el carro patina 24-51 mm más.
    _, real, enc = _straight(params, 255, 1.0, 1)
    assert 10.0 < enc < 30.0
    assert 35.0 < real < 80.0


def test_coast_from_255(params):
    # 2026-10-24: suelto desde ~800-826 mm/s rueda 113-123 mm, desde 404 rueda 31 mm.
    v_cut, real, enc = _straight(params, 255, 1.0, 0)
    assert 95.0 < real < 140.0
    assert abs(real - enc) < 1.0


def test_backlash_straight_depends_on_side(params):
    # 2026-10-24: al 75 llegando desde 55 la rueda queda a la derecha, desde 95 a la izquierda.
    st = params.steering
    for desde, signo in ((55.0, 1.0), (95.0, -1.0)):
        v = Vehicle(params, 0.0, 0.0, 0.0, servo_deg=desde)
        v.step(0.5, 75.0, 0, 0)
        assert signo * v.wheel_deg > 0.5
        assert abs(v.wheel_deg) < 1.6
    assert st.servo_center_deg == 75.0


def test_left_turns_less_than_right(params):
    # 2026-10-24: con ±15 unidades, der 8.7°, izq 7.3°.
    st = params.steering
    der = wheel_deg_from_servo(st.servo_center_deg - 13.5, st)
    izq = -wheel_deg_from_servo(st.servo_center_deg + 13.5, st)
    assert abs(der - 8.7) < 0.2 and abs(izq - 7.3) < 0.2
