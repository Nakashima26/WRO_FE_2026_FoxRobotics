from pure_pursuit.seat_posterior import posterior
from pure_pursuit.wro_field import SEATS, SECTIONS_CW, randomize


def test_randomize_truth_stays_in_support():
    for start in ("centro", "cajon", "zona", "Z3", "Z4"):
        for seed in range(1, 101):
            field = randomize(seed, start=start)
            truth = {(s.section, s.seat): s.color for s in field.signs}
            all_keys = {(sec, seat) for sec in SECTIONS_CW for seat in SEATS}
            confirmed, empty = {}, set()
            # Recorrido parcial reproducible, intercalando vacíos y colores.
            keys = sorted(all_keys)
            keys = keys[seed % len(keys):] + keys[:seed % len(keys)]
            for seen in range(len(keys) + 1):
                result = posterior(field.direction, field.parking_section, confirmed, empty, start)
                assert result is not None, (start, seed, confirmed, empty)
                for key, (p, red, green) in result.items():
                    assert 0 <= p <= 1 and abs(p - red - green) < 1e-12
                    if key in truth:
                        assert p > 0
                        assert (red if truth[key] == "Red" else green) > 0
                    else:
                        assert p < 1
                if seen < len(keys):
                    key = keys[seen]
                    if key in truth:
                        confirmed[key] = truth[key]
                    else:
                        empty.add(key)


def test_conflicting_observations_have_no_support():
    assert posterior("CW", "W", {("W", "X2"): "Red"}, set()) is None
    assert posterior("CW", "W", {}, {("N", seat) for seat in SEATS}) is None
