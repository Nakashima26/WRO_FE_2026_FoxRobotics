"""Latas repetidas que ningún asiento explica pasan al planificador como obstáculo suelto."""
from pure_pursuit import config as C
from pure_pursuit.digital_map import DigitalMap


def _dm(monkeypatch, on=True):
    monkeypatch.setattr(C, "DIGITAL_MAP_LOOSE", on)
    dm = DigitalMap()
    dm.set_direction("CCW")
    return dm


def test_loose_needs_votes_and_flag(monkeypatch):
    dm = _dm(monkeypatch)
    for i in range(2):
        dm._note_loose(-450.0 + i * 10, -300.0, "Red", 600.0)
    assert dm.loose_extra() == []
    dm._note_loose(-440.0, -300.0, "Red", 600.0)
    assert dm.loose_extra() == [(-450.0, -300.0, "Red")]
    off = _dm(monkeypatch, on=False)
    for _ in range(5):
        off._note_loose(-450.0, -300.0, "Red", 600.0)
    assert off.loose_extra() == []


def test_loose_dropped_when_seat_confirmed(monkeypatch):
    dm = _dm(monkeypatch)
    for _ in range(3):
        dm._note_loose(-880.0, -480.0, "Red", 600.0)
    assert dm.loose_extra()
    dm.confirmed[("W", "T2")] = "Red"   # W/T2 está en (-900, -500)
    assert dm.loose_extra() == []
    dm.confirmed[("W", "T2")] = "Green"  # otro color: sigue siendo un obstáculo distinto
    assert dm.loose_extra()


def test_loose_ignores_far_detections(monkeypatch):
    dm = _dm(monkeypatch)
    for _ in range(5):
        dm._note_loose(-450.0, -300.0, "Red", 1500.0)
    assert dm.loose_extra() == []
