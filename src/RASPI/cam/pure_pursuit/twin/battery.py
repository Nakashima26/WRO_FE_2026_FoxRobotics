"""Experimental pack and armature dynamics from the Simscape electrical study."""

from dataclasses import dataclass, field
from math import expm1, isfinite


@dataclass
class BatteryDrive:
    """Connect pack sag to the empirical motor map without replacing tire physics."""

    open_voltage_v: float = 11.88
    pack_resistance_ohm: float = 0.12
    auxiliary_current_a: float = 0.5
    armature_current_a: float = field(default=0.0, init=False)
    pack_current_a: float = field(default=0.0, init=False)
    terminal_voltage_v: float = field(default=0.0, init=False)
    min_terminal_voltage_v: float = field(default=0.0, init=False)
    peak_pack_current_a: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        if not all(isfinite(x) for x in (self.open_voltage_v, self.pack_resistance_ohm, self.auxiliary_current_a)):
            raise ValueError("Battery parameters must be finite.")
        if (self.open_voltage_v <= 0 or self.pack_resistance_ohm < 0 or self.auxiliary_current_a < 0
                or self.open_voltage_v <= self.pack_resistance_ohm * self.auxiliary_current_a):
            raise ValueError("The battery must support the nonnegative auxiliary load.")
        self.reset()

    def reset(self) -> None:
        self.armature_current_a = 0.0
        self.pack_current_a = self.auxiliary_current_a
        self.terminal_voltage_v = self.open_voltage_v - self.pack_resistance_ohm * self.pack_current_a
        self.min_terminal_voltage_v = self.terminal_voltage_v
        self.peak_pack_current_a = self.pack_current_a

    def advance(self, dt: float, pwm: float, direction: int, braking: bool, wheel_speed_mm_s: float) -> float:
        # Same electrical assumptions as averaged_drive.ssc. Use signed wheel speed
        # for the back EMF, including when the firmware commands a direction change.
        duty = min(max(pwm / 255, 0), 1) * direction if not braking else 0.0
        resistance = 83.0 if direction == 0 else 3.5
        resistance += self.pack_resistance_ohm * duty * duty
        motor_rad_s = wheel_speed_mm_s * 0.001 / 0.022 * 37.5
        supply = self.open_voltage_v - self.pack_resistance_ohm * self.auxiliary_current_a
        steady_current = (duty * supply - 0.0035 * motor_rad_s) / resistance
        # Exact RL update within each substep, with wheel speed held at its start value.
        self.armature_current_a += (steady_current - self.armature_current_a) * -expm1(-dt * resistance / 0.001)
        self.pack_current_a = self.auxiliary_current_a + duty * self.armature_current_a
        self.terminal_voltage_v = self.open_voltage_v - self.pack_resistance_ohm * self.pack_current_a
        self.min_terminal_voltage_v = min(self.min_terminal_voltage_v, self.terminal_voltage_v)
        self.peak_pack_current_a = max(self.peak_pack_current_a, self.pack_current_a)
        return max(0.0, self.terminal_voltage_v / self.open_voltage_v)
