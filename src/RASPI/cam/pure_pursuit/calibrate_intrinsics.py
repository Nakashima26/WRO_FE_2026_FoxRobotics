"""
Calibración de intrínsecos (K + distorsión) para la Pi Camera.

USO (desde src/RASPI/cam/):
  python -m pure_pursuit.calibrate_intrinsics

Imprime un chessboard (esquinas internas CHESSBOARD_INNER, default 9x6).
Muévelo por el FOV (cerca, lejos, bordes, inclinado). ESPACIO captura un
frame si ve el tablero. C calcula K+dist. S guarda cam_intrinsics.npz.
D borra el último. ESC sale.

Después corre `python -m pure_pursuit.calibrate` para rehacer la homografía
BEV SOBRE la imagen undistorsionada. Sin ese segundo paso el runtime NO
aplica undistort (mezclaría H vieja con K nueva).
"""

from __future__ import annotations

import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import open_camera
from . import config as C


def run(cam_index: int = C.CAM_INDEX) -> None:
    inner = tuple(getattr(C, "CHESSBOARD_INNER", (9, 6)))
    square = float(getattr(C, "CHESSBOARD_SQUARE_MM", 25.0))
    out_path = getattr(C, "INTRINSICS_FILE", C.CALIB_FILE.parent / "cam_intrinsics.npz")

    objp = np.zeros((inner[0] * inner[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:inner[0], 0:inner[1]].T.reshape(-1, 2) * square

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    cap = open_camera(cam_index)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             + cv2.CALIB_CB_NORMALIZE_IMAGE
             + cv2.CALIB_CB_FAST_CHECK)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print(f"[INTR] Chessboard interno {inner[0]}x{inner[1]}, cuadro {square:.1f} mm",
          flush=True)
    print("[INTR] ESPACIO = capturar si se ve el tablero | C = calibrar | "
          "S = guardar | D = borrar último | ESC = salir", flush=True)

    K = dist = newK = None
    rms = None
    cv2.namedWindow("Intrinsecos", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, inner, flags)
        vis = frame.copy()
        if found:
            cv2.drawChessboardCorners(vis, inner, corners, found)
        cv2.putText(vis, f"capturas={len(imgpoints)}  tablero={'SI' if found else 'no'}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if found else (0, 180, 255), 2)
        if rms is not None:
            cv2.putText(vis, f"rms={rms:.3f}px  C=recalcular  S=guardar",
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("Intrinsecos", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            print("[INTR] Cancelado.", flush=True)
            break

        if key == ord(' ') and found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            objpoints.append(objp.copy())
            imgpoints.append(corners2)
            print(f"[INTR] Captura {len(imgpoints)}", flush=True)

        elif key in (ord('d'), ord('D')):
            if imgpoints:
                objpoints.pop()
                imgpoints.pop()
                print(f"[INTR] Borrada. Quedan {len(imgpoints)}", flush=True)

        elif key in (ord('c'), ord('C')):
            if len(imgpoints) < 8:
                print("[INTR] Necesitas al menos 8 capturas bien repartidas.", flush=True)
                continue
            rms, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, image_size, None, None
            )
            newK, _roi = cv2.getOptimalNewCameraMatrix(
                K, dist, image_size, 0.0, image_size
            )
            print(f"[INTR] rms={rms:.4f}px  K=\n{K}\n dist={dist.ravel()}", flush=True)

        elif key in (ord('s'), ord('S')):
            if K is None:
                print("[INTR] Primero pulsa C para calibrar.", flush=True)
                continue
            np.savez(
                str(out_path),
                camera_matrix=K,
                dist_coeffs=dist,
                new_camera_matrix=newK,
                image_w=np.int32(image_size[0]),
                image_h=np.int32(image_size[1]),
                rms=np.float64(rms),
                chess_inner=np.array(inner, np.int32),
            )
            print(f"[INTR] Guardado {out_path}", flush=True)
            print("[INTR] Ahora: python -m pure_pursuit.calibrate  "
                  "(clics sobre la imagen ya undistorsionada).", flush=True)
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    import argparse
    p = argparse.ArgumentParser(description="Calibración de intrínsecos (chessboard)")
    p.add_argument("--cam-index", type=int, default=C.CAM_INDEX)
    args = p.parse_args()
    run(args.cam_index)


if __name__ == "__main__":
    main()
