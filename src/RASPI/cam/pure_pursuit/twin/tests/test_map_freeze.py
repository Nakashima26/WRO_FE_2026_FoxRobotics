"""DIGITAL_MAP_FREEZE_TC: una señal confirmada no salta de fila desde ese giro."""
from pure_pursuit import config as C
from pure_pursuit import digital_map as dmod


def test_move_seat_frozen_keeps_confirmed(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_FREEZE_TC", 4, raising=False)
    dm = dmod.DigitalMap()
    dm._tc = 5
    dm.confirmed = {("N", "X2"): "Green"}
    dm._move_seat(("N", "X2"), ("N", "X1"), "Green")
    assert dm.confirmed == {("N", "X2"): "Green"}


def test_move_seat_unfrozen_moves(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_FREEZE_TC", None, raising=False)
    dm = dmod.DigitalMap()
    dm._tc = 5
    dm.confirmed = {("N", "X2"): "Green"}
    dm._move_seat(("N", "X2"), ("N", "X1"), "Green")
    assert dm.confirmed == {("N", "X1"): "Green"}


def test_keep_closer_row_frozen_skips_rival_pop(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_FREEZE_TC", 4, raising=False)
    dm = dmod.DigitalMap()
    dm._tc = 5
    dm.confirmed = {("N", "T1"): "Red"}
    dm._best = {("N", "T1"): 900.0, ("N", "T3"): 100.0}
    dm._keep_closer_row("N", "T3", "Green")
    assert dm.confirmed == {("N", "T1"): "Red"}
