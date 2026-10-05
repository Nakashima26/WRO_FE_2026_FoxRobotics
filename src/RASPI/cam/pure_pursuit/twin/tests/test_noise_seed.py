"""T16ens: semilla del ruido de sensores separada de la del escenario, y la
estadística de tools/ens.py / tools/ens_cmp.py sobre datos sintéticos."""

import importlib.util
import itertools
import json
import sys
import zlib
from pathlib import Path

import numpy as np
import pytest

from pure_pursuit.twin.sim import Sim

TOOLS = Path(__file__).resolve().parents[1] / "tools"


def _tools():
    """(ens, ens_cmp) de tools/ (se importan como módulos sueltos, igual que desde la CLI)."""
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    import ens as E
    import ens_cmp as EC
    return E, EC


def _prepark_tool():
    spec = importlib.util.spec_from_file_location("tools_prepark", TOOLS / "prepark.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _draws(sim: Sim, n: int = 8) -> np.ndarray:
    return sim.sensor_rng().normal(size=n)


def test_noise_seed_none_is_old_rng():
    s = Sim(14, preset="hw_nuevo")
    assert s.noise_seed is None and s.noise_key is None
    np.testing.assert_array_equal(_draws(s), np.random.default_rng(14).normal(size=8))
    # Mismo estado interno, no solo los primeros sorteos (None = bit a bit lo de antes).
    assert s.sensor_rng().bit_generator.state == np.random.default_rng(14).bit_generator.state


def test_noise_seed_int_tuple_and_validation():
    assert Sim(0, preset="hw_nuevo", noise_seed=3).noise_key == (3,)
    assert Sim(0, preset="hw_nuevo", noise_seed=np.int64(3)).noise_key == (3,)
    assert Sim(0, preset="hw_nuevo", noise_seed=(3, 7)).noise_key == (3, 7)
    assert Sim(0, preset="hw_nuevo", noise_seed=[3, 7]).noise_key == (3, 7)
    # noise_seed=k es SeedSequence(seed, spawn_key=(k,)), no default_rng([seed, k]).
    st = Sim(5, preset="hw_nuevo", noise_seed=(3, 7)).sensor_rng().bit_generator.state
    assert st == np.random.default_rng(np.random.SeedSequence(5, spawn_key=(3, 7))).bit_generator.state
    for bad in (-1, (1, -2), ()):
        with pytest.raises(ValueError):
            Sim(0, preset="hw_nuevo", noise_seed=bad)


def test_noise_seed_changes_sensor_rng():
    base = _draws(Sim(14, preset="hw_nuevo"))
    n0 = _draws(Sim(14, preset="hw_nuevo", noise_seed=0))
    n1 = _draws(Sim(14, preset="hw_nuevo", noise_seed=1))
    n2 = _draws(Sim(14, preset="hw_nuevo", noise_seed=2))
    np.testing.assert_array_equal(n1, _draws(Sim(14, preset="hw_nuevo", noise_seed=1)))
    # noise_seed=0 tampoco es la rng vieja (default_rng([seed, 0]) sí lo sería).
    for x, y in ((base, n0), (base, n1), (n0, n1), (n1, n2)):
        assert not np.allclose(x, y)
    # El ruido depende de (seed, noise_seed), no solo de noise_seed.
    assert not np.allclose(n1, _draws(Sim(15, preset="hw_nuevo", noise_seed=1)))


def test_noise_seed_tuple_substream():
    # prepark.py: Sim(0, ...) en todos los escenarios, subcorriente (k, crc32(nombre)).
    n1 = _draws(Sim(0, preset="hw_nuevo", noise_seed=1))
    assert np.array_equal(n1, _draws(Sim(0, preset="hw_nuevo", noise_seed=(1,))))
    a = _draws(Sim(0, preset="hw_nuevo", noise_seed=(1, 11)))
    b = _draws(Sim(0, preset="hw_nuevo", noise_seed=(1, 12)))
    assert not np.allclose(a, b) and not np.allclose(a, n1)
    np.testing.assert_array_equal(a, _draws(Sim(0, preset="hw_nuevo", noise_seed=(1, 11))))


def test_prepark_noise_key_per_scenario():
    PT = _prepark_tool()
    a, b = "CW_c1_p0", "CCW_c30_p4"
    assert PT.noise_key(None, a) is None                     # sin --noise-seed: lo de siempre
    assert PT.noise_key(2, a) == (2, zlib.crc32(a.encode()))
    assert PT.noise_key(2, a) != PT.noise_key(2, b)
    assert PT.noise_key(2, a) != PT.noise_key(3, a)
    # prepark usa Sim(0, ...) para todo: con la subcorriente cada escenario tiene su ruido,
    # y la misma (k, escenario) da el mismo ruido en A y en B.
    ra = _draws(Sim(0, preset="hw_nuevo", noise_seed=PT.noise_key(1, a)))
    rb = _draws(Sim(0, preset="hw_nuevo", noise_seed=PT.noise_key(1, b)))
    assert not np.allclose(ra, rb)
    np.testing.assert_array_equal(ra, _draws(Sim(0, preset="hw_nuevo", noise_seed=PT.noise_key(1, a))))


def test_wilson_and_mdd():
    E, _ = _tools()
    lo, hi = E.wilson(5, 10)
    assert lo == pytest.approx(0.2366, abs=1e-4) and hi == pytest.approx(0.7634, abs=1e-4)
    lo, hi = E.wilson(0, 10)
    assert lo == 0.0 and hi == pytest.approx(0.2775, abs=1e-4)
    lo, hi = E.wilson(14, 105)                               # ensamble base de la Mac
    assert (round(100 * lo, 1), round(100 * hi, 1)) == (8.1, 21.1)
    assert E.wilson(0, 0) == (0.0, 1.0)
    assert E.var_unbiased(2, 4) == pytest.approx(1 / 3)      # p(1-p)·n/(n-1)
    # k_needed es la inversa de mdd: con K = k_needed(var, d) la MDD es d.
    var = [0.24, 0.0, 0.16, 0.25]
    k = E.k_needed(var, 0.1)
    assert E.mdd(var, [k] * 4, [k] * 4) == pytest.approx(0.1)


def test_paired_test_is_exact_mcnemar():
    E, _ = _tools()
    # 6 pares discordantes todos para B: p = 2·(1/2)^6; los concordantes no cuentan.
    pairs = [("s", k, 0, 1) for k in range(6)] + [("t", k, 1, 1) for k in range(10)]
    assert E.paired_test(pairs) == (6, 0, pytest.approx(2 / 64))
    pairs = [("s", k, 0, 1) for k in range(5)] + [("s", 9, 1, 0)]
    assert E.paired_test(pairs) == (5, 1, pytest.approx(2 * 7 / 64))
    assert E.paired_test([("s", 1, 1, 1)]) == (0, 0, 1.0)
    # pair_runs: solo pares (escenario, k) presentes en los dos lados.
    A = {"x": [{"k": 1, "ok": True}, {"k": 2, "ok": False}], "y": [{"k": 1, "ok": True}]}
    B = {"x": [{"k": 2, "ok": True}, {"k": 3, "ok": True}]}
    pairs, lost = E.pair_runs(A, B)
    assert pairs == [("x", 2, 0, 1)] and lost == 2


def _strat_brute(a, nA, b, nB):
    """p bilateral de la permutación por escenario, enumerando a mano todas las
    asignaciones de etiquetas (equiprobables) dentro de cada escenario."""
    obs = sum(y / m - x / n for x, n, y, m in zip(a, nA, b, nB))
    per = []
    for x, n, y, m in zip(a, nA, b, nB):
        runs = [1] * (x + y) + [0] * (n + m - x - y)
        per.append([sum(runs[i] for i in ib) for ib in itertools.combinations(range(n + m), m)])
    hit = tot = 0
    for combo in itertools.product(*per):
        st = sum(bb / m - (x + y - bb) / n for bb, x, n, y, m in zip(combo, a, nA, b, nB))
        tot += 1
        hit += abs(st) >= abs(obs) - 1e-9
    return hit / tot


def test_strat_perm_exact_and_mc_match_bruteforce():
    E, _ = _tools()
    a, b = [0, 1, 2, 0], [2, 1, 2, 3]
    exact = E.strat_perm_test(a, [3] * 4, b, [3] * 4)
    assert exact == pytest.approx(_strat_brute(a, [3] * 4, b, [3] * 4), abs=1e-12)
    assert E.strat_perm_test([1, 2], [3, 3], [1, 2], [3, 3]) == pytest.approx(1.0)
    # nA distinto entre escenarios: Monte Carlo, contra la enumeración.
    a, nA, b, nB = [0, 1, 2], [2, 3, 3], [2, 2, 3], [2, 3, 3]
    mc = E.strat_perm_test(a, nA, b, nB, mc=200_000, seed=1)
    assert mc == pytest.approx(_strat_brute(a, nA, b, nB), abs=0.01)


def test_ens_load_prepark_kmax(tmp_path):
    E, _ = _tools()
    for k, oks in ((1, (True, True)), (2, (True, False)), (3, (False, False))):
        (tmp_path / f"n{k}").mkdir()
        rows = [{"name": n, "limpio": ok, "stop": "terminado", "contacto": "" if ok else "cajón@3.0",
                 "fuera_mm": 0.0} for n, ok in zip(("A", "B"), oks)]
        (tmp_path / f"n{k}" / "summary.json").write_text(json.dumps(rows), encoding="utf-8")
    kind, runs = E.load(tmp_path)
    assert kind == "prepark" and [r["k"] for r in runs["A"]] == [1, 2, 3]
    assert [r["end"] for r in runs["B"]] == ["ok", "cajón", "cajón"]
    _, runs2 = E.load(tmp_path, kmax=2)
    assert [r["k"] for r in runs2["A"]] == [1, 2] and sum(r["ok"] for r in runs2["B"]) == 1


def test_ens_cmp_verdicts():
    _, EC = _tools()

    def ens(rates, K):
        return {f"s{i}": [{"k": k, "ok": k <= round(p * K), "end": "x"} for k in range(1, K + 1)]
                for i, p in enumerate(rates)}
    A = ens([0.2] * 21, 5)
    assert EC.stats(A, A)["p_pair"] == 1.0
    assert EC.verdict(EC.stats(A, A)).startswith("NO DISTINGUIBLE")
    B = ens([1.0] * 21, 5)
    assert EC.verdict(EC.stats(A, B)).startswith("MEJOR")
    assert EC.verdict(EC.stats(B, A)).startswith("PEOR")
    # 21 seeds sueltas (K=1): nunca hay veredicto, aunque cambien todas.
    assert EC.verdict(EC.stats(ens([0.0] * 21, 1), ens([1.0] * 21, 1))).startswith("NO CONCLUYENTE")


def _field_key(f):
    return (f.direction, f.parking_section, f.start_x, f.start_y, f.start_heading,
            [(s.color, s.section, s.seat, s.x, s.y) for s in f.signs])


def test_noise_seed_keeps_field_changes_trace():
    runs = {ns: Sim(14, preset="baseline", max_time_s=2.0, noise_seed=ns).run() for ns in (None, 1, 2)}
    keys = {ns: _field_key(r.field) for ns, r in runs.items()}
    assert keys[None] == keys[1] == keys[2]
    tr = {ns: [(row["t"], row["x"], row["heading"], row["ack"]) for row in r.trace] for ns, r in runs.items()}
    assert tr[None] != tr[1]
    assert tr[1] != tr[2]
