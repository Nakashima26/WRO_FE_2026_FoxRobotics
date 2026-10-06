"""Servo 0-180 del carro nuevo (FOX_SERVO_180=1): unidades internas 30..160 ->
valor de servo 0-180 -> rueda. Supuestos sin medir: ruedas rectas en 90 y
±46.32° de rueda en 0/180, lineal y simétrico."""

import pytest

from pure_pursuit import config as C
from pure_pursuit.twin import sim as S
from pure_pursuit.twin.firmware.fw import servo_desde_duty
from pure_pursuit.twin.params import (
    BANDA_SG90,
    SteeringParams,
    TwinParams,
    pulso_us_servo_180,
    servo_180_desde_interno,
    servo_interno_desde_180,
    steering_servo_180,
    wheel_deg_from_servo,
)
from pure_pursuit.twin.vehicle import RetencionPulso, Vehicle, pulso_us_desde_valor


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
    for env in ("TWIN_SERVO_SLEW", *(e for e, _ in S._SERVO_ENV)):
        monkeypatch.delenv(env, raising=False)
    hw = S.Sim(1, preset="hw_nuevo")
    assert hw.servo_180 and hw.fw_defines["FOX_SERVO_180"] == "1"
    # Default desde T16sg90: el SG90 del carro nuevo.
    assert hw.servo_slew == "sg90"
    st = hw.params.steering
    assert (st.servo_min_deg, st.servo_max_deg) == (0.0, 180.0)
    assert st.slew_left_deg_per_s == st.slew_right_deg_per_s == 600.0
    assert st.refresh_hz == 50.0 and st.banda_al_borde
    monkeypatch.setenv("TWIN_SERVO_SLEW", "proporcional")
    stp = S.Sim(1, preset="hw_nuevo").params.steering
    assert stp.slew_left_deg_per_s == pytest.approx(600.0 * 90 / 70)
    assert stp.slew_right_deg_per_s == pytest.approx(900.0)
    assert stp.refresh_hz is None and not stp.banda_al_borde
    gr = S.Sim(1, preset="giro_rapido")
    assert not gr.servo_180
    assert gr.params.steering == SteeringParams()
    monkeypatch.setenv("TWIN_SERVO_SLEW", "plano600")
    st6 = S.Sim(1, preset="hw_nuevo").params.steering
    assert st6.slew_left_deg_per_s is None and st6.slew_deg_per_s == 600.0


# ── T16sg90: SG90 del carro nuevo (slew 600 valor-de-servo/s, banda ±0.45 al
# borde, pulso retenido a 50 Hz). Supuestos del datasheet, sin medir. ─────────

def _servo_solo(st: SteeringParams, inicio: float, cmd: float, pasos: int, dt: float = 0.001):
    """Posición del servo (valor de servo 0-180) tras cada sub-paso, sin pulso
    retenido (el comando llega al instante)."""
    p = TwinParams()
    p.steering = st
    veh = Vehicle(p, 0.0, 0.0, 0.0)
    veh.servo_deg = inicio
    out = []
    for _ in range(pasos):
        veh.step(dt, cmd, 0.0, 0)
        out.append(veh.servo_deg)
    return veh, out


def test_sg90_rueda_308_8_grados_s_y_150_ms_del_centro_al_tope():
    st = steering_servo_180("sg90")
    assert st.slew_left_deg_per_s == st.slew_right_deg_per_s == 600.0
    # 600 valor-de-servo/s · 46.32/90 = 308.8 °/s de rueda.
    assert 600.0 * st.gain_left / 90.0 == pytest.approx(308.8, abs=0.05)
    for tope in (180.0, 0.0):
        _, pos = _servo_solo(st, 90.0, tope, 200)
        # Tasa de rueda en pleno movimiento.
        r1 = wheel_deg_from_servo(pos[49], st)
        r0 = wheel_deg_from_servo(pos[48], st)
        assert abs(r1 - r0) / 0.001 == pytest.approx(308.8, abs=0.05)
        # 90 -> tope en 0.150 s nominal; para en el borde de la banda (tope ∓ 0.45),
        # a (90 - 0.45)/600 = 0.14925 s.
        borde = tope - BANDA_SG90 if tope > 90.0 else tope + BANDA_SG90
        llega = next(i for i, v in enumerate(pos) if v == pytest.approx(borde, abs=1e-9))
        assert (llega + 1) * 0.001 == pytest.approx(0.150, abs=0.001)
        assert all(v == pytest.approx(borde, abs=1e-9) for v in pos[llega:])
    # Con TWIN_SERVO_SLEW_DEG_S=500: 0.180 s.
    _, pos = _servo_solo(steering_servo_180("sg90", slew_deg_s=500.0), 90.0, 180.0, 250)
    llega = next(i for i, v in enumerate(pos) if v == pytest.approx(180.0 - BANDA_SG90, abs=1e-9))
    assert (llega + 1) * 0.001 == pytest.approx(0.180, abs=0.001)


def test_sg90_banda_045_simetrica_y_al_borde():
    st = steering_servo_180("sg90")
    assert st.deadband_left_deg == st.deadband_right_deg == pytest.approx(0.45)
    assert st.banda_al_borde
    # Rueda: ±0.45·46.32/90 = ±0.2316°.
    assert wheel_deg_from_servo(90.0 - 0.45, st) == pytest.approx(0.2316, abs=1e-4)
    for cmd, reposo in ((100.0, 99.55), (80.0, 80.45), (90.4, 90.0), (89.6, 90.0),
                        (90.46, 90.01), (89.54, 89.99)):
        _, pos = _servo_solo(st, 90.0, cmd, 100)
        assert pos[-1] == pytest.approx(reposo, abs=1e-9), cmd
    # El reposo no depende del sub-paso (antes 0.6/ms de paso > 0.5 de banda).
    for dt in (0.001, 0.0003, 0.00071):
        _, pos = _servo_solo(st, 90.0, 123.4, int(0.2 / dt), dt)
        assert pos[-1] == pytest.approx(123.4 - 0.45, abs=1e-9)


def _servo_con_retencion(escrituras: dict[int, float], fin_us: int, paso_us: int = 1000):
    """Corre RetencionPulso + Vehicle como Sim.run (advance): ledcAttach en t=0,
    escrituras {t_us: valor de servo} entre avances. Devuelve [(t1_us, visto, servo)]."""
    st = steering_servo_180("sg90")
    p = TwinParams()
    p.steering = st
    veh = Vehicle(p, 0.0, 0.0, 0.0)
    ret = RetencionPulso(st.refresh_hz)
    valor = None
    out = []
    t = 0
    while t < fin_us:
        if t in escrituras:
            valor = escrituras[t]
        t1 = t + paso_us
        for dur, visto in ret.tramos(t, t1, True, valor):
            cmd = veh.servo_deg if visto is None else visto
            rem = dur
            while rem > 1e-12:
                sub = min(rem, 0.001)
                rem -= sub
                veh.step(sub, cmd, 0.0, 0)
        out.append((t1, ret.visto, veh.servo_deg))
        t = t1
    return out


def test_retencion_escritura_a_mitad_de_periodo_llega_en_borde_mas_pulso():
    # Ciclos en 20, 40, 60 ms desde el ledcAttach (t=0); el de 0 sale sin pulso.
    # 90 escrito en t=0 -> visto desde 20 ms + 1.5 ms. 120 escrito en t=30 ms
    # (mitad del período) -> retenido en 40 ms, visto desde 40 + 1.8333 ms.
    out = _servo_con_retencion({0: 90.0, 30_000: 120.0}, 60_000, paso_us=100)
    pulso = pulso_us_desde_valor(120.0)
    assert pulso == pytest.approx(1833.333, abs=1e-3)
    for t1, visto, servo in out:
        if t1 < 21_500:
            assert visto is None and servo == 90.0, t1
        elif t1 < 40_000 + pulso:
            assert visto == 90.0 and servo == 90.0, t1
        else:
            assert visto == 120.0, t1
    # El servo empieza a moverse recién tras borde + pulso.
    mueve = next(t1 for t1, _, s in out if s != 90.0)
    assert 40_000 + pulso < mueve <= 40_000 + pulso + 100


def test_retencion_avance_largo_parte_el_paso_en_el_flanco():
    # Un avance de 50 ms que cruza dos bordes: el tramo se parte en el flanco de
    # bajada del primer pulso y la suma de duraciones es el intervalo.
    ret = RetencionPulso(50.0)
    ret.tramos(0, 1, True, 100.0)
    tr = ret.tramos(1, 50_001, True, 100.0)
    assert sum(d for d, _ in tr) == pytest.approx(0.05)
    p100 = pulso_us_desde_valor(100.0)
    assert tr[0][0] == pytest.approx((20_000 + p100 - 1) / 1e6)
    assert [v for _, v in tr] == [None, 100.0]
    assert ret.visto == 100.0


def test_retencion_dos_escrituras_en_un_periodo_solo_cuenta_la_ultima():
    out = _servo_con_retencion({0: 90.0, 21_000: 100.0, 35_000: 110.0}, 70_000, paso_us=100)
    vistos = [v for _, v, _ in out]
    assert 100.0 not in vistos
    assert 110.0 in vistos
    t110 = next(t1 for t1, v, _ in out if v == 110.0)
    assert t110 >= 40_000 + pulso_us_desde_valor(110.0)


def test_retencion_escritura_justo_en_el_borde_va_al_ciclo_siguiente():
    # La escritura en t = 40 ms ocurre entre el avance que termina en 40 ms (que
    # ya retuvo el valor viejo) y el siguiente: la ve el ciclo de 60 ms.
    out = _servo_con_retencion({0: 90.0, 40_000: 120.0}, 80_000, paso_us=1000)
    t120 = next(t1 for t1, v, _ in out if v == 120.0)
    assert t120 >= 60_000 + pulso_us_desde_valor(120.0)


def test_retencion_sin_attach_ni_pulso_queda_quieto():
    ret = RetencionPulso(50.0)
    # Antes del ledcAttach: sin bordes, visto None.
    assert ret.tramos(0, 30_000, False, None) == [(0.03, None)]
    assert ret.origen_us is None
    # Attach en 30 ms con duty 0 (sin pulso): los bordes corren y nadie ve nada.
    assert ret.tramos(30_000, 80_000, True, None) == [(0.05, None)]
    assert ret.origen_us == 30_000 and ret.visto is None
    # Primer valor en 80 ms: el borde de 90 ms lo retiene.
    ret.tramos(80_000, 100_000, True, 45.0)
    assert ret.visto == 45.0
    # Duty 0 otra vez: queda lo último visto.
    ret.tramos(100_000, 200_000, True, None)
    assert ret.visto == 45.0
    with pytest.raises(ValueError):
        ret.tramos(200_001, 210_000, True, None)   # intervalos no contiguos
    with pytest.raises(ValueError):
        RetencionPulso(0.0)


def _st180_dd2831f(slew: str) -> SteeringParams:
    """steering_servo_180(slew) de dd2831f, literal (antes de T16sg90)."""
    return SteeringParams(
        gain_left=C.MAX_WHEEL_STEER_DEG, gain_right=C.MAX_WHEEL_STEER_DEG,
        servo_center_deg=90.0, servo_min_deg=0.0, servo_max_deg=180.0,
        slew_deg_per_s=600.0,
        slew_left_deg_per_s=600.0 * 90.0 / 70.0 if slew == "proporcional" else None,
        slew_right_deg_per_s=600.0 * 90.0 / 60.0 if slew == "proporcional" else None,
        deadband_left_deg=0.5 * 90.0 / 70.0, deadband_right_deg=0.5 * 90.0 / 60.0,
    )


@pytest.mark.parametrize("slew", ["proporcional", "plano600"])
def test_proporcional_y_plano600_identicos_a_dd2831f(slew):
    st = steering_servo_180(slew)
    assert st == _st180_dd2831f(slew)
    assert st.refresh_hz is None and not st.banda_al_borde and st.centro_rueda_deg == 0.0
    # Mismos pasos del servo que con los parámetros de dd2831f.
    cmds = [180, 180, 0, 0, 90, 135.5, 44.1, 90.3, 89.2, 177.0, 2.0]
    assert _wheel_trace(st, cmds, float) == _wheel_trace(_st180_dd2831f(slew), cmds, float)


def test_banda_cero_no_congela():
    # Antes: con banda izq = 0, izq·db_der + der·db_izq <= db_izq·db_der daba
    # 0 <= 0 para cualquier movimiento del lado derecho y el servo no se movía.
    st = SteeringParams(servo_min_deg=0.0, servo_max_deg=180.0,
                        slew_left_deg_per_s=600.0, slew_right_deg_per_s=600.0,
                        deadband_left_deg=0.0, deadband_right_deg=0.75)
    _, pos = _servo_solo(st, 90.0, 60.0, 100)
    assert pos[-1] < 60.75
    _, pos = _servo_solo(st, 90.0, 120.0, 100)
    assert pos[-1] == 120.0
    # Las dos bandas en 0: llega exacto.
    st0 = SteeringParams(servo_min_deg=0.0, servo_max_deg=180.0,
                         slew_left_deg_per_s=600.0, slew_right_deg_per_s=600.0,
                         deadband_left_deg=0.0, deadband_right_deg=0.0)
    for cmd in (60.0, 120.0, 90.2):
        _, pos = _servo_solo(st0, 90.0, cmd, 100)
        assert pos[-1] == pytest.approx(cmd, abs=1e-12)
    # Con banda_al_borde también.
    stb = steering_servo_180("sg90")
    stb.deadband_left_deg = stb.deadband_right_deg = 0.0
    _, pos = _servo_solo(stb, 90.0, 60.0, 100)
    assert pos[-1] == pytest.approx(60.0, abs=1e-12)


def test_centro_rueda_y_tope_rueda():
    st = steering_servo_180("sg90", centro_rueda_deg=1.5)
    # + = izquierda: en el twin la rueda + es derecha.
    assert wheel_deg_from_servo(90.0, st) == pytest.approx(-1.5)
    assert wheel_deg_from_servo(0.0, st) == pytest.approx(C.MAX_WHEEL_STEER_DEG - 1.5)
    assert wheel_deg_from_servo(180.0, st) == pytest.approx(-C.MAX_WHEEL_STEER_DEG - 1.5)
    stp = steering_servo_180("proporcional", centro_rueda_deg=-0.7)
    assert wheel_deg_from_servo(90.0, stp) == pytest.approx(0.7)
    stt = steering_servo_180("sg90", tope_rueda_deg=40.0)
    assert wheel_deg_from_servo(0.0, stt) == pytest.approx(40.0)
    assert wheel_deg_from_servo(180.0, stt) == pytest.approx(-40.0)
    assert wheel_deg_from_servo(45.0, stt) == pytest.approx(20.0)
    for kw in ({"tope_rueda_deg": 0.0}, {"tope_rueda_deg": 95.0}):
        with pytest.raises(ValueError):
            steering_servo_180("sg90", **kw)
    with pytest.raises(ValueError):
        steering_servo_180("proporcional", slew_deg_s=500.0)
    with pytest.raises(ValueError):
        steering_servo_180("sg90", slew_deg_s=0.0)


def test_env_del_servo_en_hw_nuevo(monkeypatch):
    for env in ("TWIN_SERVO_SLEW", *(e for e, _ in S._SERVO_ENV)):
        monkeypatch.delenv(env, raising=False)
    monkeypatch.setenv("TWIN_SERVO_SLEW_DEG_S", "500")
    monkeypatch.setenv("TWIN_SERVO_CENTRO_RUEDA_DEG", "0.5")
    monkeypatch.setenv("TWIN_RUEDA_TOPE_DEG", "44")
    hw = S.Sim(1, preset="hw_nuevo")
    st = hw.params.steering
    assert st.slew_left_deg_per_s == st.slew_right_deg_per_s == 500.0
    assert st.centro_rueda_deg == 0.5 and st.gain_left == st.gain_right == 44.0
    assert hw.servo_env == {"TWIN_SERVO_SLEW_DEG_S": 500.0,
                            "TWIN_SERVO_CENTRO_RUEDA_DEG": 0.5,
                            "TWIN_RUEDA_TOPE_DEG": 44.0}
    # TWIN_SERVO_SLEW_DEG_S solo vale con sg90.
    monkeypatch.setenv("TWIN_SERVO_SLEW", "proporcional")
    with pytest.raises(ValueError):
        S.Sim(1, preset="hw_nuevo")
    # Sin FOX_SERVO_180 las variables no se leen.
    gr = S.Sim(1, preset="giro_rapido")
    assert gr.params.steering == SteeringParams() and gr.servo_env == {}
