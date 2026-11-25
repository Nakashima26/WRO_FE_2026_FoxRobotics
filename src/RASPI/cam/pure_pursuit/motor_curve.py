"""Curva N30 en piso, twin/calib/medidas_20261117."""

from bisect import bisect_right
from functools import lru_cache


_LOW_V = 11.88
_FULL_V = 12.5  # Referencia estimada para la sesión de carga plena.
_LOW = ((11, 0), (60, 265 * _LOW_V / _FULL_V), (100, 503),
        (150, 798), (180, 1062), (255, 1501))
_FULL = ((11 * _LOW_V / _FULL_V, 0), (60, 265), (100, 593),
         (140, 809), (180, 1077), (220, 1366), (255, 1556))


def _speed_at(curve, pwm):
    i = bisect_right(curve, (pwm, float("inf")))
    if i == len(curve):
        return curve[-1][1]
    x0, y0 = curve[i - 1]
    x1, y1 = curve[i]
    return y0 + (y1 - y0) * (pwm - x0) / (x1 - x0)


@lru_cache(maxsize=8)
def forward_curve(voltage):
    """Nodos PWM físico → mm/s, interpolados entre las dos sesiones."""
    blend = max(0.0, min(1.0, (voltage - _LOW_V) / (_FULL_V - _LOW_V)))
    scale = voltage / (_LOW_V + blend * (_FULL_V - _LOW_V))
    return tuple(
        (pwm, scale * ((1.0 - blend) * _speed_at(_LOW, pwm) + blend * _speed_at(_FULL, pwm)))
        for pwm in (11, 60, 100, 140, 150, 180, 220, 255)
    )
