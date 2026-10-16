"""Rampa de velocidad cerca de un asiento sin ver (route_mode._speed, ROUTE_UNSEEN_RAMP_MM)."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from pure_pursuit import config as C
from pure_pursuit.route_mode import RouteMode
from pure_pursuit.wro_field import SECTIONS_CW, SEATS, section_to_world

SEC = SECTIONS_CW[0]
SX, SY = section_to_world(SEC, *SEATS["X1"])


def _mode():
    """RouteMode sin __init__: ruta recta lejos del asiento (no lo toca), sin nada visto."""
    m = RouteMode.__new__(RouteMode)
    xs = np.linspace(SX - 2500.0, SX + 2500.0, 500)
    ys = np.full_like(xs, SY - 5000.0)
    m.route = SimpleNamespace(x=xs, y=ys, length_mm=5000.0)
    m.follower = SimpleNamespace(i=0)
    m.visible = set()
    m.stopped = 0
    return m


def _speed_at(dist_mm):
    """Asiento sin ver recto adelante a dist_mm de la cámara (heading 0 = +y)."""
    cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
    pose = (SX, SY - dist_mm - cam, 0.0)
    return _mode()._speed({(SEC, "X1")}, pose)


def _set(monkeypatch, **kw):
    for name, val in kw.items():
        monkeypatch.setattr(C, name, val)


def test_sin_rampa_es_escalon_a_unseen_pwm(monkeypatch):
    _set(monkeypatch, ROUTE_UNSEEN_RAMP_MM=None, ROUTE_UNSEEN_PWM=155, ROUTE_LAP1_PWM=255, ROUTE_LAP1_OPEN=False)
    assert _speed_at(1000.0) == 155


def test_rampa_dentro_de_la_distancia_da_unseen_pwm(monkeypatch):
    _set(monkeypatch, ROUTE_UNSEEN_RAMP_MM=900, ROUTE_UNSEEN_PWM=155, ROUTE_LAP1_PWM=255, ROUTE_LAP1_OPEN=True)
    assert _speed_at(850.0) == 155


def test_rampa_fuera_de_la_distancia_sube_sobre_unseen_pwm(monkeypatch):
    _set(monkeypatch, ROUTE_UNSEEN_RAMP_MM=900, ROUTE_UNSEEN_PWM=155, ROUTE_LAP1_PWM=255, ROUTE_LAP1_OPEN=True)
    assert _speed_at(1200.0) > 155


def test_rampa_monotona_no_decreciente_con_la_distancia(monkeypatch):
    _set(monkeypatch, ROUTE_UNSEEN_RAMP_MM=900, ROUTE_UNSEEN_PWM=155, ROUTE_LAP1_PWM=255, ROUTE_LAP1_OPEN=True)
    vals = [_speed_at(d) for d in range(800, 1301, 25)]
    assert all(b >= a for a, b in zip(vals, vals[1:])), vals
    assert vals[0] == 155 and vals[-1] <= 255
