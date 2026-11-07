"""Localizer: lectura de distancia que salta > LOC_SENSOR_JUMP_MM se ignora."""

from __future__ import annotations

from pure_pursuit import config as C
from pure_pursuit.localizer import Localizer


def _feed(loc, dfs):
    # Car en (1000, -1000) mirando +x: el ultrasónico frontal ve la pared exterior (x=1500).
    for i, dF in enumerate(dfs):
        loc.update(f"ACK:V2,dF={dF},px={i * 0.01:.2f},py=0,yaw=0")


def test_salto_de_sensor_se_ignora(monkeypatch):
    dfs = [35.2, 35.2, 35.2, 15.0]          # el último salta 200 mm: eco de transición
    a = Localizer({}, None, 1000.0, -1000.0, 90.0)
    _feed(a, dfs)                           # con el filtro (LOC_SENSOR_JUMP_MM = 100)
    monkeypatch.setattr(C, "LOC_SENSOR_JUMP_MM", None)
    b = Localizer({}, None, 1000.0, -1000.0, 90.0)
    _feed(b, dfs)                           # sin filtro
    assert a.n_rej < b.n_rej                # el filtro descarta el salto antes de la compuerta de innovación
