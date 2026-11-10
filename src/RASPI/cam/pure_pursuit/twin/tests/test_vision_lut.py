"""Vision.hsv_masks con LUT (config.VISION_LUT) == OR de cv2.inRange, bit a bit."""

import cv2
import numpy as np
import pytest

import vision as V
from vision import Vision


def _ref_masks(color_ranges, hsv):
    out = {}
    for color, ranges in color_ranges.items():
        m = None
        for lo, hi in ranges:
            r = cv2.inRange(hsv, lo, hi)
            m = r if m is None else (m | r)
        out[color] = m
    return out


def _same(vis, hsv):
    got = vis.hsv_masks(hsv)
    ref = _ref_masks(vis.color_ranges, hsv)
    assert got.keys() == ref.keys()
    for c in ref:
        if ref[c] is None:                       # color sin rangos
            assert got[c] is None
            continue
        assert got[c].dtype == ref[c].dtype and got[c].shape == ref[c].shape
        assert np.array_equal(got[c], ref[c]), c


# Todos los (H, S, V) de 8 bits: 4096x4096 px = el cubo completo.
_CUBE = np.stack(np.meshgrid(np.arange(256, dtype=np.uint8), np.arange(256, dtype=np.uint8),
                             np.arange(256, dtype=np.uint8), indexing="ij"), -1).reshape(4096, 4096, 3)

# Rangos de la Pi 4 y de la Pi 5 (vision.COLOR_RANGES elige según la cámara) y uno con
# naranja/rosa extra y un color sin rangos.
_SETS = {
    "actual": None,
    "pi4": {"Red": [((0, 150, 40), (5, 255, 200)), ((168, 170, 40), (179, 255, 235))],
            "Green": [((30, 35, 25), (85, 255, 255))]},
    "pi5": {"Red": [((0, 110, 40), (5, 255, 200)), ((168, 110, 40), (179, 255, 235))],
            "Green": [((30, 35, 25), (85, 255, 255))]},
    "extra": {"Red": [((0, 110, 40), (5, 255, 200)), ((168, 110, 40), (179, 255, 235))],
              "Green": [((30, 35, 25), (85, 255, 255))],
              "Orange": [((7, 85, 140), (18, 200, 255))],
              "Pink": [((140, 60, 60), (167, 255, 255)), ((0, 0, 0), (255, 255, 255))],
              "Nada": []},
}


@pytest.mark.parametrize("name", list(_SETS))
def test_lut_cubo_completo(name):
    vis = Vision(open_cam=False)
    if _SETS[name] is not None:
        vis.color_ranges = {c: [(np.array(lo), np.array(hi)) for lo, hi in rs] for c, rs in _SETS[name].items()}
    _same(vis, _CUBE)


def test_lut_se_rearma_si_cambian_los_rangos():
    vis = Vision(open_cam=False)
    rng = np.random.default_rng(0)
    hsv = rng.integers(0, 256, (120, 160, 3), dtype=np.uint8)
    _same(vis, hsv)
    vis.color_ranges["Green"][0][0][1] = 90          # en el sitio, como un recalibrado
    _same(vis, hsv)
    vis.color_ranges["Red"] = vis.color_ranges["Red"][:1]
    _same(vis, hsv)
    vis.color_ranges = {f"c{i}": [(np.array([i * 20, 0, 0]), np.array([i * 20 + 30, 255, 255]))] for i in range(9)}
    _same(vis, hsv)                                  # 9 rangos no caben en un byte: cv2.inRange


def test_detect_on_lut_igual_inrange_en_cuadros_del_gemelo():
    from pure_pursuit.twin.camera import CameraModel
    from pure_pursuit.twin.params import TwinParams
    from pure_pursuit.wro_field import randomize
    params = TwinParams()
    cam = CameraModel(params.camera, params)
    a, b = Vision(open_cam=False), Vision(open_cam=False)
    b.use_lut = False
    n_obj = 0
    for seed in (1, 2, 3):
        field = randomize(seed)
        for sg in field.signs:
            for dy in (-350.0, -600.0):
                frame = cam.render_camera(field, sg.x, sg.y + dy, 0.0)
                _same(a, cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
                fa, pa = a.detect_on(frame)
                fb, pb = b.detect_on(frame)
                assert pa == pb and np.array_equal(fa, fb)
                n_obj += sum(len(v) for v in pa.values())
    assert n_obj > 0
