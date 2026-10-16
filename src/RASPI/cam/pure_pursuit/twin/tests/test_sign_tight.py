"""Paso muy justo a una señal (route_mode._profile_pwm, ROUTE_PROFILE_SIGN_TIGHT_MM)."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from pure_pursuit import config as C
from pure_pursuit.route_mode import RouteMode

LEN = 4000.0
N = 201


def _mode(sign_offset_mm=130.0):
    """RouteMode sin __init__: ruta recta por y=0 (kappa 0), que empieza 200 mm antes de la señal.

    La señal está en (0, sign_offset_mm): la ruta pasa a esa distancia de su centro.
    """
    m = RouteMode.__new__(RouteMode)
    s = np.linspace(0.0, LEN, N)
    x = s - 200.0
    r = SimpleNamespace(kappa=np.zeros(N), s=s, x=x, y=np.zeros(N), length_mm=LEN)
    m.follower = SimpleNamespace(route=r, i=0)
    signs = np.array([[0.0, sign_offset_mm]])
    return m, signs


def _set(monkeypatch, **kw):
    for name, val in kw.items():
        monkeypatch.setattr(C, name, val)


def _base_cfg(monkeypatch, tight_mm):
    _set(
        monkeypatch,
        ROUTE_PROFILE=True,
        ROUTE_PROFILE_MIN_PWM=155,
        ROUTE_PROFILE_BASE_PWM=205,
        ROUTE_PROFILE_MAX_PWM=255,
        ROUTE_PROFILE_SIGN_MM=220.0,
        ROUTE_PROFILE_SIGN_NEAR_MM=150.0,
        ROUTE_PROFILE_SIGN_TIGHT_MM=tight_mm,
        ROUTE_PROFILE_SIGN_TIGHT_PWM=110,
        ROUTE_PROFILE_ISLAND_MM=None,
        ROUTE_PROFILE_BRAKE_FF_S=None,
        ROUTE_PROFILE_TIGHT_KAPPA=None,
    )


def test_tight_none_no_baja_de_min_pwm(monkeypatch):
    _base_cfg(monkeypatch, None)
    m, signs = _mode()
    assert m._profile_pwm(signs) >= 155


def test_tight_baja_hasta_tight_pwm(monkeypatch):
    _base_cfg(monkeypatch, 120.0)
    m, signs = _mode()
    pwm = m._profile_pwm(signs)
    assert 110 <= pwm < 155, pwm
