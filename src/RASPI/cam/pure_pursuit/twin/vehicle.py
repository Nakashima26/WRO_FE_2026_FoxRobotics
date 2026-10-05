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
        if _por_lado(st):
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
    bandas 0.5·90/70 y 0.5·90/60 es |err| <= 0.5 en unidades internas 30..160."""
    c = st.servo_center_deg
    dbl = st.deadband_left_deg if st.deadband_left_deg is not None else st.deadband_deg
    dbr = st.deadband_right_deg if st.deadband_right_deg is not None else st.deadband_deg
    lo, hi = (pos, target) if pos <= target else (target, pos)
    izq = max(0.0, hi - max(lo, c))
    der = max(0.0, min(hi, c) - lo)
    return izq * dbr + der * dbl <= dbl * dbr


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


def servo_from_ledc(ledc_duty: float, center: float = 90.0) -> float:
    """Mapa aproximado LEDC 0..255 → ángulo servo (solo utilidad de prueba)."""
    return center + (ledc_duty - 128.0) * (60.0 / 128.0)
