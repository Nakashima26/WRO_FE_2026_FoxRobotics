"""Modo ruta: planificador (ring_route), localizador y reglas de asientos (route_mode)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from pure_pursuit import ring_route as RR
from pure_pursuit.localizer import Localizer, build_segments, raycast
from pure_pursuit.route_mode import RouteMode
from pure_pursuit.wro_field import SEATS, randomize, section_to_world


def test_pursuit_kappa_no_supera_el_pico_de_la_polilinea():
    from pure_pursuit.route_mode import _pursuit_kappa
    f = randomize(1)
    r = RR.plan(f.direction, RR.signs_from_field(f), f.parking_section)
    assert np.abs(_pursuit_kappa(r, 200.0)).max() <= np.abs(r.kappa).max() + 1e-9


def test_margen_blando_solo_con_mapa_completo():
    from pure_pursuit.route_mode import _prm_for
    assert _prm_for([]).soft_clear_mm == 100.0
    assert _prm_for([("N", "T1")]).soft_clear_mm == 140.0


@pytest.mark.parametrize("seed", [1, 2, 9, 14, 3170839])
def test_plan_valida(seed):
    """Ruta factible, con holgura de la huella, radio y lado de paso correctos."""
    f = randomize(seed)
    r = RR.plan(f.direction, RR.signs_from_field(f), f.parking_section)
    rep = r.report
    assert rep["feasible"]
    assert rep["sign_mm"] >= 40.0
    assert rep["wall_mm"] >= 80.0
    assert rep["island_mm"] >= 80.0
    assert rep.get("barrier_mm", 1e9) >= 80.0
    assert rep["r_min_mm"] >= 150.0
    # lado de paso: rojo a la derecha del sentido, verde a la izquierda
    for s in f.signs:
        i = int(np.argmin(np.hypot(r.x - s.x, r.y - s.y)))
        h = math.radians(r.heading_deg[i])
        right = (s.x - r.x[i]) * math.cos(h) - (s.y - r.y[i]) * math.sin(h)
        # la señal queda a la IZQUIERDA del carro si se pasa por su derecha
        assert (right < 0) == (s.color == "Red"), (s.section, s.seat, s.color)


def test_plan_desconocidos_y_ancla():
    """Con asientos sin ver la ruta los esquiva; el ancla la hace pasar por el carro."""
    f = randomize(3)
    seats = RR.signs_from_field(f)
    known = dict(list(seats.items())[:2])
    unknown = [(sec, st) for sec in "NESW" for st in SEATS if (sec, st) not in known]
    r = RR.plan(f.direction, known, f.parking_section, unknown=unknown)
    assert r.report["feasible"]
    for sec, st in unknown:
        x, y = section_to_world(sec, *SEATS[st])
        assert np.hypot(r.x - x, r.y - y).min() > 66.0 + 25.0
    anchor = (r.x[100] + 25.0, r.y[100])
    r2 = RR.plan(f.direction, known, f.parking_section, unknown=unknown, anchor=anchor)
    assert np.hypot(r2.x - anchor[0], r2.y - anchor[1]).min() < 45.0


def test_follower_recto():
    """Sobre la ruta y alineado, el pure pursuit casi no gira en recta."""
    f = randomize(1)
    r = RR.plan(f.direction, RR.signs_from_field(f), f.parking_section)
    fol = RR.RouteFollower(r, 200.0)
    n = len(r.x)
    win = np.array([np.abs(r.kappa[(np.arange(30) + i) % n]).max() for i in range(n)])
    k = int(np.argmin(win))                     # 300 mm seguidos casi rectos
    assert win[k] < 1e-3
    wheel = fol.steer(r.x[k], r.y[k], r.heading_deg[k])
    assert abs(wheel) < 3.0


def test_raycast_pared():
    segs = build_segments({}, None)
    d, kind, cosi = raycast(np.array([[0.0, 1000.0]]), np.array([[0.0, 1.0]]), segs)
    assert d[0] == pytest.approx(500.0)
    assert kind[0] == 0 and cosi[0] == pytest.approx(1.0)


def test_localizer_corrige_lateral():
    """Odometría corrida 40 mm de lado: el ultrasónico lateral la corrige."""
    loc = Localizer({}, None, 0.0, 1040.0, 90.0)       # cree estar en y=1040; real y=1000
    ack0 = "ACK:V2,yaw=0.00,px=0,py=0"
    loc.update(ack0)
    for i in range(1, 30):
        # avanza hacia +x; dL = pared norte (y=1500) desde y=1000: 500 - 63.5 = 436.5 mm
        loc.update(f"ACK:V2,dL=43.6,yaw=0.00,px={i * 10},py=0")
    assert abs(loc.x[1] - 1000.0) < 15.0


class _DM:
    def __init__(self, conf, parking="S"):
        self.confirmed = conf
        self.parking = parking


def test_reglas_de_asientos():
    rm = RouteMode()
    conf, empty = rm._resolved(_DM({("N", "X1"): "Red"}))
    # X va sola: el resto de N vacío; asientos prohibidos del cajón vacíos
    assert all(("N", s) in empty for s in SEATS if s != "X1")
    assert ("S", "T3") in empty and ("S", "T4") in empty and ("S", "X2") in empty
    conf, empty = rm._resolved(_DM({("E", "T4"): "Green"}))
    # misma columna (T2) vacía, la otra columna sigue sin resolver
    assert ("E", "T2") in empty and ("E", "X1") in empty
    assert ("E", "T1") not in empty and ("E", "T3") not in empty
