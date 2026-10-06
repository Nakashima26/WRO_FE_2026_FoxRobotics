"""Esperar a pasarlo no aplica si el paso ya coincide con el giro."""

from pure_pursuit.digital_map import DigitalMap


def test_pass_matches_turn():
    m = DigitalMap()
    m.direction = "CW"
    assert m._pass_matches_turn(200.0) is True   # rojo + giro der
    assert m._pass_matches_turn(-200.0) is False  # verde + giro der → sí esperar
    m.direction = "CCW"
    assert m._pass_matches_turn(-200.0) is True  # verde + giro izq
    assert m._pass_matches_turn(200.0) is False  # rojo + giro izq → sí esperar


def test_blocks_turn_skips_ccw_green():
    m = DigitalMap()
    m.direction = "CCW"
    m._aligned = True
    m._tc, m._tpr = 0, 12
    # Sin esto, CCW seguiría bloqueando por verde-ahead.
    assert m.blocks_turn() is False


def test_needs_line_skips_interior(monkeypatch):
    from pure_pursuit import config as C
    monkeypatch.setattr(C, "DIGITAL_MAP_STEER", True, raising=False)
    m = DigitalMap()
    m.in_stall = False
    m._aligned = True
    m.direction = "CW"
    m.lat_mm = 0.0
    monkeypatch.setattr(m, "_ease", lambda *a, **k: 200.0)
    assert m.needs_line() is False
    monkeypatch.setattr(m, "_ease", lambda *a, **k: -200.0)
    assert m.needs_line() is True


def test_ease_holds_last_red(monkeypatch):
    m = DigitalMap()
    m.direction = "CW"
    monkeypatch.setattr(m, "_cans_on", lambda *a, **k: [(2000.0, 200.0)])
    assert m._ease("S", 2500.0) == 200.0


def test_ease_commits_pass_side_early(monkeypatch):
    m = DigitalMap()
    m.direction = "CW"
    monkeypatch.setattr(m, "_cans_on", lambda *a, **k: [(2000.0, 210.0)])
    assert m._ease("S", 400.0) == 210.0
    assert m._ease("S", 1990.0) == 210.0


def test_pass_side_ahead(monkeypatch):
    m = DigitalMap()
    m.in_stall = False
    m._aligned = True
    monkeypatch.setattr(m, "_ease", lambda *a, **k: 200.0)
    assert m.pass_side_ahead() is True
    monkeypatch.setattr(m, "_ease", lambda *a, **k: 0.0)
    assert m.pass_side_ahead() is False
    m.in_stall = True
    monkeypatch.setattr(m, "_ease", lambda *a, **k: 200.0)
    assert m.pass_side_ahead() is False
