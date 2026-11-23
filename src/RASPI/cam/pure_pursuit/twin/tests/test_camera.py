"""Cámara sintética y calibración."""

import math
import time
from pathlib import Path

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit.twin.camera import OLD_CALIB_4_MM, CameraModel
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.wro_field import OUTER_HALF_MM, SIGN_MM, randomize
from vision import Vision


def test_render_sign_bev_projection():
    params = TwinParams()
    cam = CameraModel(params.camera, params)
    field = randomize(1)
    sign = field.signs[0]
    half = SIGN_MM / 2.0
    # A 45° una lata a más de ~13 cm se aplasta contra el borde de arriba
    # y el detector la tira por aspect ratio. 12 cm sigue siendo un blob.
    y_rear = sign.y - half - 120.0 - params.camera.forward_from_rear_axle_mm
    x_rear = sign.x
    heading = 0.0
    bev_tf, _ = cam.build_bev()
    vision = Vision(open_cam=False)
    frame = cam.render_camera(field, x_rear, y_rear, heading)
    _, positions = vision.detect_on(frame)
    found = None
    for color in ("Red", "Green"):
        for obj in positions.get(color, []):
            mapped = __import__(
                "pure_pursuit.centerline", fromlist=["map_obstacle_to_bev"]
            ).map_obstacle_to_bev(bev_tf, *obj)
            if mapped is not None:
                found = (mapped, color, sign)
                break
        if found:
            break
    assert found is not None
    (bev_x, bev_y), _color, sign = found
    # Cara cercana de la señal en mm desde eje delantero
    true_fwd = (sign.y - half) - (y_rear + params.vehicle.wheelbase_mm)
    true_lat = sign.x - x_rear
    exp_bev_x = C.ROBOT_BEV_X + true_lat / C.MM_PER_PX
    exp_bev_y = C.ROBOT_BEV_Y - true_fwd / C.MM_PER_PX
    err = math.hypot(bev_x - exp_bev_x, bev_y - exp_bev_y) * C.MM_PER_PX
    assert err < 30.0


def test_twin_pi_bev_floor_consistency(monkeypatch):
    """Piso en el gemelo → píxel → cam_to_bev del npz; debe coincidir con mm→BEV."""
    root = Path(__file__).resolve().parents[2]
    npz, lens = root / "bev_calib.npz", root / "lens_calib.npz"
    if not npz.exists() or not lens.exists():
        pytest.skip("bev_calib.npz o lens_calib.npz ausentes")
    monkeypatch.setenv("TWIN_LENS_CALIB", str(lens))
    monkeypatch.setenv("TWIN_CAMERA_FROM_CALIB", str(npz))
    twin = TwinParams()
    cam, bev, _, _ = CameraModel.twin_camera_and_bev(twin)
    errs: list[float] = []
    wb = twin.vehicle.wheelbase_mm
    behind = wb - cam.params.forward_from_rear_axle_mm
    for fwd in range(100, 1001, 40):
        for lat in range(-200, 201, 40):
            delta = np.array(
                [lat - cam.params.right_mm, fwd + behind, -cam.params.height_mm],
                dtype=np.float64,
            )
            pt_cam = delta @ cam._robot_to_cam.T
            if pt_cam[2] < 8.0:
                continue
            uv = cam._project(pt_cam.reshape(1, 3))[0]
            hit = bev.cam_to_bev(float(uv[0]), float(uv[1]))
            if hit is None:
                continue
            exp = __import__("pure_pursuit.bev", fromlist=["BEVTransformer"]).BEVTransformer._real_mm_to_bev_px(
                np.array([[lat, fwd]], dtype=np.float32)
            )[0]
            errs.append(math.hypot(hit[0] - exp[0], hit[1] - exp[1]) * C.MM_PER_PX)
    assert errs, "ningún punto de piso en rango"
    assert max(errs) < 3.0, f"max err {max(errs):.2f} mm"


def test_from_calib_npz_old_four_points():
    npz = Path(__file__).resolve().parents[2] / "bev_calib.npz"
    if not npz.exists():
        pytest.skip("bev_calib.npz no presente")
    model, report = CameraModel.from_calib_npz(
        npz, calib_points_mm=OLD_CALIB_4_MM, search_hfov=False
    )
    print(report)
    assert math.isfinite(report["height_mm"])
    assert math.isfinite(report["forward_from_rear_axle_mm"])


def test_render_performance():
    params = TwinParams()
    cam = CameraModel(params.camera, params)
    field = randomize(2)
    t0 = time.perf_counter()
    for _ in range(5):
        cam.render_camera(field, 0.0, 500.0, 0.0)
    dt = (time.perf_counter() - t0) / 5.0
    assert dt < 0.040
