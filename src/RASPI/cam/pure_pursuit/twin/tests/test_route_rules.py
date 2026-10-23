"""Reglas de asientos por recta (route_mode._rule_empty)."""

from __future__ import annotations

from pure_pursuit.route_mode import RouteMode, cte_urgent_holds
from pure_pursuit.wro_field import SECTIONS_CW, SEATS


def _all_but(sec, keep):
    return {(sec, s) for s in SEATS if s not in keep}


def test_cte_cuerda_larga_suelta_el_tope():
    """|cte| > 100 mm y la ruta dobla ≥ 25° a 500 mm, con holgura: no sostiene el 110."""
    assert cte_urgent_holds(-200.0, 40.0, 200.0, 600.0, 100.0) is False
    assert cte_urgent_holds(243.0, 5.0, 200.0, 600.0, 100.0) is True   # n3 s16, recta
    assert cte_urgent_holds(-49.0, 16.0, 200.0, 600.0, 100.0) is True  # n1 s8
    assert cte_urgent_holds(-200.0, 40.0, 30.0, 600.0, 100.0) is True


def test_x_confirmada_deja_vacios_los_demas_de_la_recta():
    """Una X va sola: los otros 5 asientos de su recta quedan vacíos, las demás rectas no."""
    sec = SECTIONS_CW[0]
    out = RouteMode()._rule_empty({(sec, "X1"): "Red"})
    assert out == _all_but(sec, {"X1"})


def test_t_en_columna_deja_vacio_el_otro_de_la_columna_y_las_x():
    """T1 (columna 2): T3 vacío (misma columna) y ambas X vacías; T2 y T4 siguen abiertos."""
    sec = SECTIONS_CW[1]
    out = RouteMode()._rule_empty({(sec, "T1"): "Green"})
    assert out == {(sec, "T3"), (sec, "X1"), (sec, "X2")}


def test_dos_senales_en_la_recta_dejan_vacios_los_restantes():
    """Máximo dos por recta: con T2 y T3 confirmados, T1, T4 y las X quedan vacíos."""
    sec = SECTIONS_CW[2]
    out = RouteMode()._rule_empty({(sec, "T2"): "Red", (sec, "T3"): "Green"})
    assert out == {(sec, "T1"), (sec, "T4"), (sec, "X1"), (sec, "X2")}


def test_recta_sin_senal_no_aplica_reglas():
    """Sin señales en una recta, _rule_empty no marca nada allí (la regla de los 6 vacíos es de _resolved)."""
    sec0, sec1 = SECTIONS_CW[0], SECTIONS_CW[1]
    out = RouteMode()._rule_empty({(sec0, "T2"): "Red"})
    assert not any(s == sec1 for (s, _t) in out)
    assert out and all(s == sec0 for (s, _t) in out)
