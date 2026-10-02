"""Aviso de esquina de la Pi al ESP32 (es= en el V2)."""

from __future__ import annotations

import time

from pure_pursuit import config as C
from pure_pursuit.twin.tests.test_runtime_step import _floor_frame, _make_runtime


def _orange(near_y):
    return {"Orange": {"seen": near_y is not None, "near_y": near_y}}


def test_v2_without_hint_by_default(monkeypatch):
    monkeypatch.setattr(C, "CORNER_HINT_TO_ESP", False, raising=False)
    rt, serial = _make_runtime()
    rt.process_frame(_floor_frame(), time.perf_counter(), armed=True)
    assert ",es=" not in serial.sent[-1]


def test_v2_carries_hint_when_enabled(monkeypatch):
    monkeypatch.setattr(C, "CORNER_HINT_TO_ESP", True, raising=False)
    rt, serial = _make_runtime()
    rt.process_frame(_floor_frame(), time.perf_counter(), armed=True)
    assert serial.sent[-1].endswith(",es=0")


def test_hint_holds_after_line_passes_under_car(monkeypatch):
    monkeypatch.setattr(C, "CORNER_HINT_NEAR_Y", 120.0, raising=False)
    monkeypatch.setattr(C, "CORNER_HINT_HOLD_S", 1.5, raising=False)
    rt, _serial = _make_runtime()
    assert not rt._corner_hint(_orange(80.0), 10.0)      # todavía lejos
    assert rt._corner_hint(_orange(300.0), 10.1)
    assert rt._corner_hint(_orange(None), 11.5)          # ya bajo el carro
    assert not rt._corner_hint(_orange(None), 11.7)


def test_hint_cleared_when_turn_starts(monkeypatch):
    rt, _serial = _make_runtime()
    assert rt._corner_hint(_orange(300.0), 10.0)
    rt._prev_estado = "G"
    assert not rt._corner_hint(_orange(None), 10.2)
    rt._prev_estado = "S"
    assert not rt._corner_hint(_orange(None), 10.3)
