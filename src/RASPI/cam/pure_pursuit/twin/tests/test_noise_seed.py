"""T16ens: semilla del ruido de sensores separada de la del escenario."""

import numpy as np

from pure_pursuit.twin.sim import Sim


def _draws(sim: Sim, n: int = 8) -> np.ndarray:
    return sim.sensor_rng().normal(size=n)


def test_noise_seed_none_is_old_rng():
    s = Sim(14, preset="hw_nuevo")
    assert s.noise_seed is None
    np.testing.assert_array_equal(_draws(s), np.random.default_rng(14).normal(size=8))


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


def test_ens_cmp_verdicts():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import ens_cmp as EC

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
