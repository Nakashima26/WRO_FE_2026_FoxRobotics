"""DP de la ruta: la salida (d por estación) no cambia con la caché de geometría
ni con el kernel en C. El golden sale del código previo a esas optimizaciones."""
import os

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit import ring_route as RR
from pure_pursuit.route_mode import _route_params

GOLDEN = os.path.join(os.path.dirname(__file__), "data", "route_dp_golden.npz")
_SEATS_ALL = [(s, t) for s in "NESW" for t in RR.SEATS]


def _params(time_cost):
    """Parámetros del planificador como en el carro (perfil de la Pi)."""
    saved = (C.ROUTE_TIME_COST, C.ROUTE_PROFILE_BASE_PWM, C.ROUTE_PROFILE_A_LAT_MM_S2)
    try:
        C.ROUTE_TIME_COST = time_cost
        C.ROUTE_PROFILE_BASE_PWM = 205
        C.ROUTE_PROFILE_A_LAT_MM_S2 = 2000.0
        return _route_params()
    finally:
        C.ROUTE_TIME_COST, C.ROUTE_PROFILE_BASE_PWM, C.ROUTE_PROFILE_A_LAT_MM_S2 = saved


def _cases():
    """4 entradas del planificador: (direction, seats, parking, prm, unknown, start)."""
    seats2 = {("N", "T2"): "Red", ("E", "T3"): "Green", ("S", "X1"): "Red"}
    unknown2 = [k for k in _SEATS_ALL if k not in seats2][:14]
    return [
        ("CW", {}, "W", _params(True), _SEATS_ALL, None),
        ("CW", seats2, "W", _params(True), unknown2, None),
        ("CCW", {("N", "T1"): "Green", ("W", "T4"): "Red"}, "E", _params(True), [], None),
        ("CW", seats2, "W", _params(False), unknown2, None),
    ]


def _run(case):
    direction, seats, parking, prm, unknown, start = case
    return RR.plan(direction, seats, parking, prm=prm, start_section=start, unknown=unknown)


def _check_golden(native):
    g = np.load(GOLDEN)
    old = RR._NATIVE_DP
    RR._NATIVE_DP = native
    try:
        for i, case in enumerate(_cases()):
            r = _run(case)
            assert np.array_equal(r.d_stations, g[f"d_{i}"]), f"caso {i}: d_stations cambió"
            assert bool(r.report["feasible"]) == bool(g[f"feasible_{i}"]), f"caso {i}: feasible cambió"
    finally:
        RR._NATIVE_DP = old


def test_numpy_matches_golden():
    _check_golden(native=False)


def test_native_matches_golden():
    if RR._dp_kernel() is None:
        pytest.skip("fox_route_dp no disponible (biblioteca nativa)")
    _check_golden(native=True)


if __name__ == "__main__":
    ds, feas = [], []
    for case in _cases():
        r = _run(case)
        ds.append(np.asarray(r.d_stations))
        feas.append(bool(r.report["feasible"]))
    np.savez(GOLDEN, **{f"d_{i}": d for i, d in enumerate(ds)},
             **{f"feasible_{i}": np.array(f) for i, f in enumerate(feas)})
    print("golden", GOLDEN, "feasible", feas, "len", [len(d) for d in ds])
