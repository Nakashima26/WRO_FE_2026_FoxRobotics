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


def servo_from_ledc(ledc_duty: float, center: float = 90.0) -> float:
    """Mapa aproximado LEDC 0..255 → ángulo servo (solo utilidad de prueba)."""
    return center + (ledc_duty - 128.0) * (60.0 / 128.0)
