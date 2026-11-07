"""Arranque ciego: cajón movido a lo largo de la recta (wro_field.park_w_of,
start_fix.StartFix)."""

import numpy as np

from pure_pursuit import config as C
from pure_pursuit import wro_field as W
from pure_pursuit.start_fix import StartFix, guess_w0


def test_world_to_section_roundtrip():
    rng = np.random.default_rng(0)
    for sec in W.SECTIONS_CW:
        for _ in range(5):
            h, w = rng.uniform(0, 1000), rng.uniform(0, 3000)
            h2, w2 = W.world_to_section(sec, *W.section_to_world(sec, h, w))
            assert abs(h - h2) < 1e-9 and abs(w - w2) < 1e-9
        x0, y0 = W.section_to_world(sec, 300, 1000)
        x1, y1 = W.section_to_world(sec, 300, 1001)
        assert np.allclose((x1 - x0, y1 - y0), W.w_axis(sec))


def test_park_pos_keeps_signs_and_moves_stall():
    a = W.randomize(77, start="cajon")
    b = W.randomize(77, start="cajon", park_pos="random")
    assert [(s.section, s.seat, s.color) for s in a.signs] == [(s.section, s.seat, s.color) for s in b.signs]
    assert a.parking_w == W.PARK_W_MIN
    assert W.PARK_W_MIN <= b.parking_w <= W.PARK_W_MAX
    assert b.parking_w == W.randomize(77, start="cajon", park_pos="random").parking_w
    # El carro se mueve con el cajón, a lo largo de +w de la recta.
    d = b.parking_w - a.parking_w
    ax, ay = W.w_axis(a.parking_section)
    assert np.allclose((b.start_x - a.start_x, b.start_y - a.start_y), (d * ax, d * ay), atol=1e-6)
    c = W.randomize(77, start="cajon", park_pos=1400)
    assert c.parking_w == 1400.0


def _drive(sf, sec, direction, true_delta, h_lane=350.0, est="S", seed=0):
    """Carro en el carril a h_lane avanzando por la recta; lecturas del modelo en el δ verdadero."""
    rng = np.random.default_rng(seed)
    hd = W._heading(sec, direction)
    j = int(np.argmin(np.abs(sf.deltas - true_delta)))
    sgn = 1.0 if direction == "CW" else -1.0
    w0 = sf.w0_guess + (150.0 if direction == "CW" else 0.0)
    for k in range(120):
        w = w0 + sgn * 8.0 * k                       # marco supuesto
        x, y = W.section_to_world(sec, h_lane, w)
        vals = {}
        for key, m in (("dF", "us_front"), ("dL", "us_left"), ("dR", "us_right")):
            e = sf._expected_us((x, y, hd), C.SENSOR_MOUNTS[m])[j]
            vals[key] = 200 if not np.isfinite(e) else int(round((e + rng.normal(0, 5)) / 10))
        ack = f"px={8.0 * k:.1f},py=0,yaw=0,dF={vals['dF']},dL={vals['dL']},dR={vals['dR']},est={est}"
        sf.update(ack, (x, y, hd))
        got = sf.converged()
        if got is not None:
            return got, k
    return None, None


def test_converges_cw_and_ccw():
    for sec, direction, true in (("N", "CW", 400.0), ("S", "CCW", -300.0), ("E", "CW", 0.0), ("W", "CCW", -690.0)):
        sf = StartFix(sec, direction)
        assert sf.w0_guess == guess_w0(direction)
        got, _k = _drive(sf, sec, direction, true)
        assert got is not None, (sec, direction)
        assert abs(got - true) <= 20.0, (sec, direction, got, true)


def test_ignores_readings_outside_s_c_r():
    sf = StartFix("N", "CW")
    got, _k = _drive(sf, "N", "CW", 400.0, est="I")
    assert got is None and sf.n_frames == 0


def test_box_hits_matches_raycast():
    from pure_pursuit.localizer import raycast
    from pure_pursuit.start_fix import _box_hits
    sf = StartFix("N", "CW")
    rng = np.random.default_rng(1)
    o = rng.uniform(-1490, 1490, (4000, 2))
    o = o[~((np.abs(o[:, 0]) < 510) & (np.abs(o[:, 1]) < 510))]
    a = rng.uniform(0, 2 * np.pi, len(o))
    for d in (np.stack([np.sin(a), np.cos(a)], 1),
              np.array([[1, 0], [0, 1], [-1, 0], [0, -1]] * len(o), float)[:len(o)]):
        d1, k1, c1 = raycast(o, d, sf._static)
        d2, k2, c2 = _box_hits(o, d)
        assert np.allclose(d1, d2) and (k1 == k2).all() and np.allclose(c1, c2)
