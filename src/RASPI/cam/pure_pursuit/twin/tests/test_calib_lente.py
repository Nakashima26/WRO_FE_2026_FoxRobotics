"""Calibración de lente: ajuste sintético, persistencia con sig y uso en el twin.

Desde src/RASPI/cam:  PYTHONPATH=. python -m pytest pure_pursuit/twin/tests/test_calib_lente.py -q
"""

import cv2
import numpy as np

from pure_pursuit import bev
from pure_pursuit import calib_lente as cl
from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.params import CameraParams, TwinParams

W, H = 640, 480


def _synthetic_views(n=15, noise_px=0.1, seed=0):
    rng = np.random.default_rng(seed)
    K = np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 240.0], [0.0, 0.0, 1.0]])
    dist = np.array([-0.25, 0.07, 0.0, 0.0, 0.0])
    obj = cl.board_points(9, 6, 25.0).astype(np.float64)
    objs, imgs = [], []
    for _ in range(n):
        rvec = rng.uniform(-0.5, 0.5, 3)
        tvec = np.array([rng.uniform(-60, 60), rng.uniform(-45, 45), rng.uniform(300, 500)])
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        proj = proj.reshape(-1, 1, 2) + rng.normal(0, noise_px, (len(obj), 1, 2))
        objs.append(obj.astype(np.float32))
        imgs.append(proj.astype(np.float32))
    return objs, imgs


def test_calibrate_from_points_recovers_intrinsics():
    objs, imgs = _synthetic_views()
    res = cl.calibrate_from_points(objs, imgs, (W, H), alpha=1.0)
    assert res["rms"] < 0.3
    assert abs(res["K"][0, 0] - 300.0) / 300.0 < 0.02
    assert abs(res["dist"].ravel()[0] - (-0.25)) < 0.03
    assert len(res["per_view_err"]) == 15


def test_save_lens_sig_is_read_by_bev(tmp_path, monkeypatch):
    objs, imgs = _synthetic_views(n=15)
    res = cl.calibrate_from_points(objs, imgs, (W, H), alpha=1.0)
    path = tmp_path / "lens_calib.npz"
    sig = cl.save_lens(path, res, 1.0)
    monkeypatch.setenv("FOX_LENS_CALIB", str(path))
    assert bev._lens_sig() == sig
    monkeypatch.setenv("FOX_LENS_CALIB", "0")
    assert bev._lens_sig() == ""


def test_twin_camera_uses_measured_K():
    K = ((300.0, 0.0, 320.0), (0.0, 300.0, 240.0), (0.0, 0.0, 1.0))
    model = CameraModel(params=CameraParams(K=K), vehicle=TwinParams())
    assert model._fx == 300.0
    assert model._fy == 300.0
    assert model._cx == 320.0
    assert model._cy == 240.0


def test_tablero_imprimible_se_detecta():
    import cv2
    from pure_pursuit import calib_lente as L
    img = L.draw_board(9, 6, 25.0, dpi=60)
    ok, _ = cv2.findChessboardCorners(img, (9, 6))
    assert ok
