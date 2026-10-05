"""Servo 0-180 del carro nuevo (FOX_SERVO_180=1): unidades internas 30..160 ->
valor de servo 0-180 -> rueda. Supuestos sin medir: ruedas rectas en 90 y
±46.32° de rueda en 0/180, lineal y simétrico."""

import pytest

from pure_pursuit import config as C
from pure_pursuit.twin import sim as S
from pure_pursuit.twin.firmware.fw import servo_desde_duty
from pure_pursuit.twin.params import (
    SteeringParams,
    TwinParams,
    pulso_us_servo_180,
    servo_180_desde_interno,
    servo_interno_desde_180,
    steering_servo_180,
    wheel_deg_from_servo,
)
from pure_pursuit.twin.vehicle import Vehicle


def _duty(pulso_us: int) -> int:
    # escribirServo(): duty = (pulso * ((1 << 16) - 1)) / 20000, entero.
    return (pulso_us * 65535) // 20000


def _rueda_desde_interno(v: int, st: SteeringParams) -> float:
    s180 = servo_desde_duty(_duty(pulso_us_servo_180(v)))
    s180 = max(st.servo_min_deg, min(st.servo_max_deg, s180))   # recorte de vehicle.py
    return wheel_deg_from_servo(s180, st)


@pytest.mark.parametrize("interno, servo, rueda", [
    (30, 0.0, C.MAX_WHEEL_STEER_DEG),      # tope derecho (twin: + = derecha)
    (90, 90.0, 0.0),
    (160, 180.0, -C.MAX_WHEEL_STEER_DEG),  # tope izquierdo
])
def test_interno_a_servo_180_a_rueda(interno, servo, rueda):
    st = steering_servo_180()
    assert servo_180_desde_interno(interno) == pytest.approx(servo, abs=1e-9)
    assert pulso_us_servo_180(interno) == 500 + round(servo * 2000 / 180)
    # El duty de 16 bits trunca hasta ~0.3 µs: 0.03° de servo, 0.015° de rueda.
    assert _rueda_desde_interno(interno, st) == pytest.approx(rueda, abs=0.02)


def test_fuera_de_rango_queda_en_tope():
    # La U de la fase 24 en CCW pide 20 (ticks x70 a ambos lados).
    assert pulso_us_servo_180(20) == pulso_us_servo_180(30) == 500
    assert pulso_us_servo_180(175) == pulso_us_servo_180(160) == 2500


def test_misma_rueda_que_unidades_internas():
    # Mismo ángulo de rueda que el modelo de siempre (30..160) en todo el rango,
    # salvo el redondeo del pulso.
    st0, st = SteeringParams(), steering_servo_180()
    for v in range(30, 161):
        assert wheel_deg_from_servo(servo_180_desde_interno(v), st) == pytest.approx(
            wheel_deg_from_servo(float(v), st0), abs=1e-9)
        assert servo_interno_desde_180(servo_180_desde_interno(v)) == pytest.approx(v, abs=1e-9)


def _wheel_trace(st: SteeringParams, cmds: list[float], to_cmd, hold_ms: int = 20) -> list[float]:
    p = TwinParams()
    p.steering = st
    veh = Vehicle(p, 0.0, 0.0, 0.0)
    veh.servo_deg = to_cmd(90.0)
    out = []
    for c in cmds:
        for _ in range(hold_ms):
            veh.step(0.001, to_cmd(c), 0.0, 0)
            out.append(veh.wheel_deg)
    return out


def test_slew_proporcional_misma_tasa_de_rueda():
    cmds = [160, 160, 30, 30, 30, 125, 60, 90, 158, 31]
    ref = _wheel_trace(SteeringParams(), cmds, float)
    new = _wheel_trace(steering_servo_180("proporcional"), cmds, servo_180_desde_interno)
    assert new == pytest.approx(ref, abs=1e-6)
    # 600 °/s plano de valor de servo es más lento en rueda (~309 vs 397/463 °/s).
    slow = _wheel_trace(steering_servo_180("plano600"), cmds, servo_180_desde_interno)
    assert abs(slow[19]) < abs(ref[19])


def test_banda_muerta_misma_en_rueda():
    # Holds largos: el servo llega y se queda dentro de la banda muerta. Con 0.5
    # plano de valor de servo la rueda paraba más cerca del objetivo (0.257° vs
    # 0.386° der / 0.331° izq); escalada por lado queda igual que con 30..160.
    # (valores fuera de empates exactos |err| == banda, que dependen del redondeo)
    cmds = [58, 90, 125, 90, 30, 160, 31, 89, 91, 58.27, 57.63, 90.41, 159.63, 30.21, 89.83, 90.17]
    ref = _wheel_trace(SteeringParams(), cmds, float, hold_ms=300)
    new = _wheel_trace(steering_servo_180("proporcional"), cmds, servo_180_desde_interno, hold_ms=300)
    assert new == pytest.approx(ref, abs=1e-6)
    # 90 -> 58: el servo para a 58.2 (rueda 24.5496), no en 58 (24.7040).
    assert ref[299] == pytest.approx(24.5496, abs=1e-4)
    # Con 0.5 plano de valor de servo no coincide (era el error del shim).
    plano = steering_servo_180("proporcional")
    plano.deadband_left_deg = plano.deadband_right_deg = None
    viejo = _wheel_trace(plano, cmds, servo_180_desde_interno, hold_ms=300)
    assert max(abs(a - b) for a, b in zip(viejo, ref)) > 0.1


def test_preset_hw_nuevo_usa_servo_180(monkeypatch):
    monkeypatch.delenv("TWIN_SERVO_SLEW", raising=False)
    hw = S.Sim(1, preset="hw_nuevo")
    assert hw.servo_180 and hw.fw_defines["FOX_SERVO_180"] == "1"
    st = hw.params.steering
    assert (st.servo_min_deg, st.servo_max_deg) == (0.0, 180.0)
    assert st.slew_left_deg_per_s == pytest.approx(600.0 * 90 / 70)
    assert st.slew_right_deg_per_s == pytest.approx(900.0)
    gr = S.Sim(1, preset="giro_rapido")
    assert not gr.servo_180
    assert gr.params.steering == SteeringParams()
    monkeypatch.setenv("TWIN_SERVO_SLEW", "plano600")
    st6 = S.Sim(1, preset="hw_nuevo").params.steering
    assert st6.slew_left_deg_per_s is None and st6.slew_deg_per_s == 600.0
