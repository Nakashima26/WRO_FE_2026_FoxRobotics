"""Pose de la cámara a partir del bev_calib.npz (y lens_calib.npz si existe). No edita archivos.

Desde src/RASPI/cam (PYTHONPATH=.):
  python -m pure_pursuit.twin.tools.calib.calib_camara [--bev pure_pursuit/bev_calib.npz] [--lente pure_pursuit/lens_calib.npz]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from pure_pursuit import config as C
from pure_pursuit.twin.camera import CameraModel


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bev", default=str(C.CALIB_FILE))
    ap.add_argument("--lente", default=str(C.LENS_CALIB_FILE))
    a = ap.parse_args(argv)

    lente = Path(a.lente) if Path(a.lente).exists() else None
    _, r = CameraModel.from_calib_npz(a.bev, search_hfov=lente is None, lens_path=lente)

    print(f"lente: {lente if lente else 'sin lens_calib.npz (hfov buscado)'}")
    for k in ("height_mm", "tilt_deg", "yaw_deg", "roll_deg",
              "forward_from_rear_axle_mm", "right_from_rear_axle_mm", "reproj_err_px"):
        print(f"{k:28s} {r[k]:.2f}")
    print("\n# config.py sugerido")
    print(f"CAMERA_HEIGHT_MM = {r['height_mm']:.1f}")
    print(f"CAMERA_TILT_DEG = {r['tilt_deg']:.1f}")
    print(f"CAMERA_FWD_MM = {r['forward_from_rear_axle_mm']:.1f}")
    print(f"CAMERA_RIGHT_MM = {r['right_from_rear_axle_mm']:.1f}")

    if abs(r["yaw_deg"]) > 3 or abs(r["roll_deg"]) > 3:
        print("\nAVISO: la camara esta torcida, el twin no modela yaw/roll salvo con rotation_cam_to_robot")
    if r["reproj_err_px"] > 2:
        print("\nAVISO: error de reproyeccion alto, rehaz los 9 puntos")
    return 0


if __name__ == "__main__":
    sys.exit(main())
