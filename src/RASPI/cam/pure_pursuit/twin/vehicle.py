"""Dinámica longitudinal + bicicleta del Fox (eje trasero)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .params import (
    TwinParams,
    steady_speed_mm_s,
    wheel_deg_from_servo,
)


@dataclass
class StepInfo:
    ds_true_mm: float
    yaw_rate_ccw_dps: float
    accel_mm_s2: float


@dataclass
class Vehicle:
    params: TwinParams
    x: float
    y: float
    heading_deg: float
    v: float = 0.0
    servo_deg: float = 90.0

    @property
    def wheel_deg(self) -> float:
        return wheel_deg_from_servo(self.servo_deg, self.params.steering)

    def step(
        self,
        dt: float,
        servo_cmd_deg: float,
        pwm: float,
        direction: int,
    ) -> StepInfo:
        """
        direction: +1 adelante, -1 reversa, 0 coast.
        Integra con sub-pasos <= 1 ms.
        """
        if dt <= 0.0:
            return StepInfo(0.0, 0.0, 0.0)
        remaining = dt
        ds_total = 0.0
        h0 = self.heading_deg
        v0 = self.v
        max_sub = 0.001
        while remaining > 1e-12:
            sub = min(remaining, max_sub)
            remaining -= sub
            self._integrate_substep(sub, servo_cmd_deg, pwm, direction)
            ds_total += self.v * sub
        dt_eff = dt
        yaw_rate = -(self.heading_deg - h0) / dt_eff if dt_eff > 0 else 0.0
        accel = (self.v - v0) / dt_eff if dt_eff > 0 else 0.0
        return StepInfo(ds_total, yaw_rate, accel)

    def _integrate_substep(
        self,
        dt: float,
        servo_cmd_deg: float,
        pwm: float,
        direction: int,
    ) -> None:
        st = self.params.steering
        mot = self.params.motor
        veh = self.params.vehicle

        # Slew servo
        target = max(st.servo_min_deg, min(st.servo_max_deg, servo_cmd_deg))
        if st.banda_al_borde:
            self.servo_deg = _slew_al_borde(self.servo_deg, target, dt, st)
        elif _por_lado(st):
            if not _en_banda_muerta(self.servo_deg, target, st):
                self.servo_deg = _slew_por_lado(self.servo_deg, target, dt, st)
        else:
            err = target - self.servo_deg
            if abs(err) <= st.deadband_deg:
                pass
            else:
                step = st.slew_deg_per_s * dt
                if err > 0:
                    self.servo_deg = min(target, self.servo_deg + step)
                else:
                    self.servo_deg = max(target, self.servo_deg - step)

        wheel = wheel_deg_from_servo(self.servo_deg, st)
        v_ss = steady_speed_mm_s(pwm, wheel, mot, st)
        if direction > 0:
            v_target = v_ss
        elif direction < 0:
            v_target = -steady_speed_mm_s(pwm, wheel, mot, st)
        else:
            v_target = 0.0

        if direction == 0:
            tau = mot.tau_coast_s
            self.v += (v_target - self.v) * (dt / tau)
        else:
            tau = mot.tau_drive_s
            self.v += (v_target - self.v) * (dt / tau)

        wh = math.radians(wheel)
        if abs(wh) > 1e-9 and abs(veh.wheelbase_mm) > 1e-6:
            yaw_rate_rad = self.v / veh.wheelbase_mm * math.tan(wh)
            self.heading_deg += math.degrees(yaw_rate_rad * dt)
        else:
            yaw_rate_rad = 0.0

        h = math.radians(self.heading_deg)
        self.x += self.v * dt * math.sin(h)
        self.y += self.v * dt * math.cos(h)


def _por_lado(st) -> bool:
    return any(v is not None for v in (st.slew_left_deg_per_s, st.slew_right_deg_per_s,
                                       st.deadband_left_deg, st.deadband_right_deg))


def _en_banda_muerta(pos: float, target: float, st) -> bool:
    """Banda muerta por lado: el tramo [pos, target] se reparte en la parte sobre
    el centro (izquierda) y bajo el centro (derecha), cada una medida contra la
    banda de su lado: izq/db_izq + der/db_der <= 1. Con valor de servo 0-180 y
    bandas 0.5·90/70 y 0.5·90/60 es |err| <= 0.5 en unidades internas 30..160.
    Con una banda en 0, un tramo de ese lado nunca está en la banda (antes la
    forma multiplicada daba 0 <= 0 y congelaba el servo)."""
    c = st.servo_center_deg
    dbl = st.deadband_left_deg if st.deadband_left_deg is not None else st.deadband_deg
    dbr = st.deadband_right_deg if st.deadband_right_deg is not None else st.deadband_deg
    lo, hi = (pos, target) if pos <= target else (target, pos)
    izq = max(0.0, hi - max(lo, c))
    der = max(0.0, min(hi, c) - lo)
    if dbl > 0.0 and dbr > 0.0:
        return izq * dbr + der * dbl <= dbl * dbr
    frac = 0.0
    for tramo, db in ((izq, dbl), (der, dbr)):
        if tramo > 0.0:
            if db <= 0.0:
                return False
            frac += tramo / db
    return frac <= 1.0


def _banda_lado(target: float, st) -> float:
    """Banda muerta del lado del objetivo (> centro = izquierda)."""
    if target > st.servo_center_deg:
        return st.deadband_left_deg if st.deadband_left_deg is not None else st.deadband_deg
    return st.deadband_right_deg if st.deadband_right_deg is not None else st.deadband_deg


def _slew_al_borde(pos: float, target: float, dt: float, st) -> float:
    """Banda muerta al borde (banda_al_borde=True), independiente de dt.

    Si |err| > banda, el servo avanza hacia el borde de la banda (objetivo −
    signo(err)·banda) a la tasa del slew y para justo ahí: avanza
    min(paso, |err| − banda). Si |err| <= banda, queda quieto. Supuesto:
    histéresis de un servo analógico que apaga el motor al entrar en la banda
    (sin inercia ni rebote). La banda se toma del lado del objetivo y la tasa la
    da _slew_por_lado (cambia de lado al cruzar el centro)."""
    err = target - pos
    db = _banda_lado(target, st)
    if abs(err) <= db:
        return pos
    borde = target - db if err > 0.0 else target + db
    return _slew_por_lado(pos, borde, dt, st)


def _slew_por_lado(pos: float, target: float, dt: float, st) -> float:
    """Mueve pos hacia target en dt con la tasa del lado donde está el servo
    (> centro = izquierda); si cruza el centro, el resto del dt va a la tasa del
    otro lado. Con valor de servo 0-180 y tasas 600·90/70 y 600·90/60 la rueda
    se mueve igual que con 600 °/s en unidades internas 30..160."""
    c = st.servo_center_deg
    left = st.slew_left_deg_per_s if st.slew_left_deg_per_s is not None else st.slew_deg_per_s
    right = st.slew_right_deg_per_s if st.slew_right_deg_per_s is not None else st.slew_deg_per_s
    rem = dt
    while rem > 0.0 and pos != target:
        up = target > pos
        rate = left if (pos > c or (pos == c and up)) else right
        crosses = (pos < c < target) or (target < c < pos)
        limit = c if crosses else target
        need = abs(limit - pos) / rate
        if need <= rem:
            pos = limit
            rem -= need
        else:
            pos += rate * rem if up else -rate * rem
            rem = 0.0
    return pos


def pulso_us_desde_valor(valor: float) -> float:
    """Valor de servo 0-180 -> ancho de pulso (µs): inversa de fw.servo_desde_duty."""
    return 500.0 + valor * 2000.0 / 180.0


class RetencionPulso:
    """El servo ve el comando a la tasa del PWM (SteeringParams.refresh_hz), no al
    instante del ledcWrite.

    Supuestos (inferidos: la doc de ESP-IDF dice que el duty nuevo del LEDC "no toma
    efecto hasta el siguiente ciclo"; un servo analógico mide el pulso y actúa al
    bajar el flanco):
    - Los ciclos arrancan en t_k = t_attach + k/refresh_hz. t_attach es el instante
      del ledcAttach del servo en el SIL: el inicio del primer intervalo de tiempo en
      que el pin ya tiene frecuencia (entre dos avances el reloj no corre, así que
      es exacto).
    - En t_k el LEDC toma el último duty escrito ANTES de t_k; una escritura en el
      mismo instante t_k entra al ciclo siguiente. El ciclo k=0 sale con el duty 0
      que deja ledcAttach: sin pulso.
    - El servo ve ese valor desde t_k + pulso (flanco de bajada) hasta el siguiente
      pulso. Un ciclo con duty 0 no trae pulso: el servo sigue con lo último visto.
    - Antes del primer pulso visto (visto = None) el servo se queda donde está.
    """

    def __init__(self, refresh_hz: float) -> None:
        if not refresh_hz > 0.0:
            raise ValueError(f"refresh_hz debe ser > 0 (pedido: {refresh_hz})")
        self.periodo_us = 1e6 / refresh_hz
        self.origen_us: int | None = None
        self.visto: float | None = None
        self._k = 1
        self._t_us: int | None = None
        self._pendientes: list[tuple[float, float]] = []

    def tramos(
        self, t0_us: int, t1_us: int, attached: bool, valor: float | None,
    ) -> list[tuple[float, float | None]]:
        """Intervalo (t0, t1] en que el LEDC tiene `valor` (valor de servo 0-180;
        None = duty 0, sin pulso). Devuelve [(duración_s, visto)] en orden, que
        suman (t1 - t0)/1e6; visto = None: el servo no recibió ningún pulso aún.
        Solo se parte el intervalo cuando cambia lo visto."""
        if self._t_us is not None and t0_us != self._t_us:
            raise ValueError(f"intervalos no contiguos: {self._t_us} -> {t0_us}")
        self._t_us = t1_us
        if self.origen_us is None:
            if not attached:
                return [((t1_us - t0_us) / 1e6, self.visto)]
            self.origen_us = t0_us
        while True:
            tk = self.origen_us + self._k * self.periodo_us
            if tk > t1_us:
                break
            if valor is not None:
                self._pendientes.append((tk + pulso_us_desde_valor(valor), valor))
            self._k += 1
        out: list[tuple[float, float | None]] = []
        cur = float(t0_us)
        while self._pendientes and self._pendientes[0][0] <= t1_us:
            tv, v = self._pendientes.pop(0)
            if v == self.visto:
                continue   # mismo valor: no se parte el tramo
            if tv > cur:
                out.append(((tv - cur) / 1e6, self.visto))
                cur = tv
            self.visto = v
        if t1_us > cur:
            out.append(((t1_us - cur) / 1e6, self.visto))
        return out


def servo_from_ledc(ledc_duty: float, center: float = 90.0) -> float:
    """Mapa aproximado LEDC 0..255 → ángulo servo (solo utilidad de prueba)."""
    return center + (ledc_duty - 128.0) * (60.0 / 128.0)
