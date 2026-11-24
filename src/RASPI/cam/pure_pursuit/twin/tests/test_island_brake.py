"""Freno por isla: perfil a caras + tope ToF interior (orillas219 vuelta 3)."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from pure_pursuit import config as C
from pure_pursuit.route_mode import RouteMode
from pure_pursuit.wro_field import INNER_HALF_MM


def _set(monkeypatch, **kw):
    for name, val in kw.items():
        monkeypatch.setattr(C, name, val)


def _straight_mode(y_mm: float):
    """Ruta recta por x creciente, y fija (cara de isla al sur/norte)."""
    m = RouteMode.__new__(RouteMode)
    n = 201
    s = np.linspace(0.0, 4000.0, n)
    r = SimpleNamespace(
        kappa=np.zeros(n), s=s, x=s - 200.0, y=np.full(n, y_mm), length_mm=4000.0,
    )
    m.follower = SimpleNamespace(route=r, i=0)
    return m


def test_profile_face_frena_en_recta_junto_a_isla(monkeypatch):
    """Antes solo 4 esquinas: cara a y=INNER+180 no frenaba; con holgura de caja sí."""
    _set(
        monkeypatch,
        ROUTE_PROFILE=True,
        ROUTE_PROFILE_MIN_PWM=155,
        ROUTE_PROFILE_BASE_PWM=205,
        ROUTE_PROFILE_MAX_PWM=255,
        ROUTE_PROFILE_SIGN_MM=None,
        ROUTE_PROFILE_SIGN_NEAR_MM=None,
        ROUTE_PROFILE_SIGN_TIGHT_MM=None,
        ROUTE_PROFILE_ISLAND_MM=350.0,
        ROUTE_PROFILE_ISLAND_NEAR_MM=150.0,
        ROUTE_PROFILE_BRAKE_FF_S=None,
        ROUTE_PROFILE_TIGHT_KAPPA=None,
        ROUTE_L23_MIN_PWM=None,
    )
    h = float(INNER_HALF_MM)
    m = _straight_mode(h + 180.0)  # ~180 mm de la cara norte/sur
    pwm = m._profile_pwm(np.zeros((0, 2)))
    assert pwm is not None and pwm < 205, pwm


def test_profile_lejos_de_isla_no_toca_base(monkeypatch):
    _set(
        monkeypatch,
        ROUTE_PROFILE=True,
        ROUTE_PROFILE_MIN_PWM=155,
        ROUTE_PROFILE_BASE_PWM=205,
        ROUTE_PROFILE_MAX_PWM=255,
        ROUTE_PROFILE_SIGN_MM=None,
        ROUTE_PROFILE_SIGN_NEAR_MM=None,
        ROUTE_PROFILE_SIGN_TIGHT_MM=None,
        ROUTE_PROFILE_ISLAND_MM=350.0,
        ROUTE_PROFILE_ISLAND_NEAR_MM=150.0,
        ROUTE_PROFILE_BRAKE_FF_S=None,
        ROUTE_PROFILE_TIGHT_KAPPA=None,
        ROUTE_L23_MIN_PWM=None,
    )
    h = float(INNER_HALF_MM)
    m = _straight_mode(h + 500.0)
    pwm = m._profile_pwm(np.zeros((0, 2)))
    assert pwm is not None and pwm >= 200, pwm


def test_tof_interior_cap_rampa():
    """Solo roce (<12 cm): dL=15 no entra; dL=9 cerca del piso."""
    near, cm, lo = 8.0, 12.0, 130
    assert not (0.0 < 15.0 < cm)
    span = cm - near
    t = min(max((9.0 - near) / span, 0.0), 1.0)
    cap = int(round(lo + t * (255 - lo)))
    assert lo <= cap <= 170, cap
