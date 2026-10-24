"""Dinámica longitudinal + bicicleta del Fox (eje trasero)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .params import (
    TwinParams,
    max_wheel_deg,
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
    # Posición efectiva de las ruedas en unidades de servo: sigue al cuerno con juego
    # (steering.backlash_deg). None = arranca igual que servo_deg.
    wheel_servo_deg: float | None = None
    # Velocidad de la llanta (lo que ve el encoder): igual a v salvo frenando con la llanta
    # bloqueada, cuando el carro patina y la llanta ya está parada.
    v_wheel: float = 0.0
    sliding: bool = False
    # wheel_servo_deg con el retraso de steering.lag_tau_s (lo que de verdad apuntan las ruedas).
    wheel_lag_deg: float | None = None

    def __post_init__(self) -> None:
        if self.wheel_servo_deg is None:
            self.wheel_servo_deg = self.servo_deg
        if self.wheel_lag_deg is None:
            self.wheel_lag_deg = self.wheel_servo_deg
        self.v_wheel = self.v

    @property
    def wheel_deg(self) -> float:
        return wheel_deg_from_servo(self.wheel_lag_deg, self.params.steering)

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
        # Llamado ~575k veces por corrida: params en locales y wheel_deg_from_servo /
        # steady_speed_mm_s en línea (mismas operaciones, mismos floats).
        st = self.params.steering
        mot = self.params.motor
        veh = self.params.vehicle

        # Slew servo
        hi = st.servo_max_deg
        lo = st.servo_min_deg
        target = servo_cmd_deg if servo_cmd_deg < hi else hi
        target = target if target > lo else lo
        servo = self.servo_deg
        err = target - servo
        if abs(err) > st.deadband_deg:
            step = st.slew_deg_per_s * dt
            if err > 0:
                servo = servo + step
                servo = servo if servo < target else target
            else:
                servo = servo - step
                servo = servo if servo > target else target
            self.servo_deg = servo

        # Juego: las ruedas no se mueven hasta que el cuerno se aleja más de backlash_deg.
        eff = self.wheel_servo_deg
        b = st.backlash_deg
        if servo - eff > b:
            eff = servo - b
            self.wheel_servo_deg = eff
        elif eff - servo > b:
            eff = servo + b
            self.wheel_servo_deg = eff
        if st.lag_tau_s > 0.0:
            eff = self.wheel_lag_deg + (eff - self.wheel_lag_deg) * min(1.0, dt / st.lag_tau_s)
        self.wheel_lag_deg = eff

        wheel = wheel_deg_from_servo(eff, st)
        braking = direction != 0 and pwm <= mot.pwm0
        if braking:
            # Freno en corto: desaceleración casi constante; rápido, la llanta se bloquea y patina.
            v0 = self.v
            if not self.sliding and abs(v0) > mot.brake_lock_speed_mm_s:
                self.sliding = True
            dec = (mot.brake_slide_decel_mm_s2 if self.sliding else mot.brake_decel_mm_s2) * dt
            vel = v0 - dec if v0 > dec else (v0 + dec if v0 < -dec else 0.0)
            self.v = vel
            if self.sliding:
                self.v_wheel -= self.v_wheel * (dt / mot.brake_wheel_tau_s)
                if abs(vel) < 1e-6:
                    self.sliding = False
                    self.v_wheel = 0.0
            else:
                self.v_wheel = vel
        elif direction == 0 and mot.coast_decel_mm_s2:
            dec = mot.coast_decel_mm_s2 * dt
            v0 = self.v
            vel = v0 - dec if v0 > dec else (v0 + dec if v0 < -dec else 0.0)
            self.v = vel
            self.v_wheel = vel
            self.sliding = False
        else:
            if direction == 0:
                v_target = 0.0
            else:
                v = mot.k_mm_s_per_pwm * (pwm - mot.pwm0)
                w_max = max_wheel_deg(st)
                if w_max > 1e-6:
                    v *= 1.0 - mot.scrub_coeff * (abs(wheel) / w_max) ** 2
                v_ss = max(0.0, v)
                v_target = v_ss if direction > 0 else -v_ss

            tau = mot.tau_coast_s if direction == 0 else mot.tau_drive_s
            dv = (v_target - self.v) * (dt / tau)
            amax = mot.a_max_mm_s2
            if direction != 0 and amax:
                lim = amax * dt
                if dv > lim:
                    dv = lim
                elif dv < -lim:
                    dv = -lim
            vel = self.v + dv
            self.v = vel
            self.v_wheel = vel
            self.sliding = False

        wh = _radians(wheel)
        if abs(wh) > 1e-9 and abs(veh.wheelbase_mm) > 1e-6:
            yaw_rate_rad = vel / veh.wheelbase_mm * _tan(wh)
            alat = mot.a_lat_left_mm_s2 if yaw_rate_rad > 0 else mot.a_lat_right_mm_s2
            if alat and abs(vel) > 1e-3 and abs(yaw_rate_rad * vel) > alat:   # subviraje: la curva se abre
                yaw_rate_rad = _copysign(alat / abs(vel), yaw_rate_rad)
            self.heading_deg += _degrees(yaw_rate_rad * dt)

        h = _radians(self.heading_deg)
        self.x += vel * dt * _sin(h)
        self.y += vel * dt * _cos(h)


_radians, _degrees, _sin, _cos, _tan, _copysign = math.radians, math.degrees, math.sin, math.cos, math.tan, math.copysign


def servo_from_ledc(ledc_duty: float, center: float = 90.0) -> float:
    """Mapa aproximado LEDC 0..255 → ángulo servo (solo utilidad de prueba)."""
    return center + (ledc_duty - 128.0) * (60.0 / 128.0)
