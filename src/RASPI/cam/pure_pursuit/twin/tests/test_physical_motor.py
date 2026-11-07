"""Motor físico (hoja de datos N20 + etapa LEGO) y tope de aceleración."""

from pure_pursuit.twin.params import TwinParams, physical_motor
from pure_pursuit.twin.vehicle import Vehicle


def test_physical_motor_gear_1_5_range():
    m = physical_motor(gear=1.5)
    assert 690.0 < m["k_mm_s_per_pwm"] * (255.0 - m["pwm0"]) < 740.0
    assert 0.04 < m["tau_drive_s"] < 0.08


def test_physical_motor_gear_1_0_faster_and_slower_to_start():
    m15 = physical_motor(gear=1.5)
    m10 = physical_motor(gear=1.0)
    v15 = m15["k_mm_s_per_pwm"] * (255.0 - m15["pwm0"])
    v10 = m10["k_mm_s_per_pwm"] * (255.0 - m10["pwm0"])
    assert v10 > v15
    assert m10["tau_drive_s"] > m15["tau_drive_s"]


def test_accel_cap_limits_first_substep():
    params = TwinParams()
    params.motor.a_max_mm_s2 = 500.0
    v = Vehicle(params, 0.0, 0.0, 0.0)
    v.step(0.001, 90.0, 200.0, 1)
    assert v.v <= 500.0 * 0.001 + 1e-9
