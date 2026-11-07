"""DIGITAL_MAP_ORANGE_BEHIND_MM: la cinta de la esquina de atrás no cuenta como la siguiente."""
from pure_pursuit import config as C
from pure_pursuit import digital_map as dmod


def _orange(dm, along):
    dm.in_stall = False
    dm.along_mm = along
    dm._note_orange({"seen": True, "near_y": C.ROBOT_BEV_Y - 50})


def test_orange_behind_ignored_below_threshold(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_ORANGE_BEHIND_MM", 1500, raising=False)
    dm = dmod.DigitalMap()
    _orange(dm, 400)
    assert dm._orange_y is None


def test_orange_ahead_kept_above_threshold(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_ORANGE_BEHIND_MM", 1500, raising=False)
    dm = dmod.DigitalMap()
    _orange(dm, 2100)
    assert dm._orange_y is not None


def test_orange_kept_when_flag_off(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_ORANGE_BEHIND_MM", None, raising=False)
    dm = dmod.DigitalMap()
    _orange(dm, 400)
    assert dm._orange_y is not None
