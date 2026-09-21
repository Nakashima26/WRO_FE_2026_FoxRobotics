"""
Replay / self-test de la proyección BEV.

Los .avi/.mp4 de corrida son HUD: izquierda = cámara 640x480 (con overlays),
derecha = BEV YA warpeada. No se puede reestimar H desde ellos (no hay
marcadores de suelo ni frame crudo limpio). Sí se puede:

  • re-warpear el panel izquierdo con la H actual (y undistort si hay K)
  • comparar contra el BEV que quedó grabado a la derecha
  • self-test sintético: puntos de piso + barrel distortion, con vs sin undistort

USO (desde src/RASPI/cam/):
  python -m pure_pursuit.replay_calib --self-test
  python -m pure_pursuit.replay_calib --avi /ruta/orillasNNNN.avi --save-dir /tmp/bev_replay
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from .bev import BEVTransformer
from . import config as C


HUD_CAM_W = 640


def split_hud(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
    """HUD 1120x480 → (cámara 640x480, BEV 480x480) o (frame, None)."""
    h, w = frame.shape[:2]
    if w >= HUD_CAM_W + 16 and h == 480:
        return frame[:, :HUD_CAM_W], frame[:, HUD_CAM_W:]
    return frame, None


def _synthetic_intrinsics(w: int = 640, h: int = 480):
    fx = fy = 520.0
    K = np.array([[fx, 0, w / 2.0],
                  [0, fy, h / 2.0],
                  [0, 0, 1.0]], dtype=np.float64)
    # Barrel típico de módulo IMX219 / Pi Cam v2 (orden de magnitud).
    dist = np.array([0.22, -0.08, 0.0, 0.0, 0.0], dtype=np.float64)
    return K, dist


def _ground_to_cam(pts_mm: np.ndarray, K, dist, rvec, tvec) -> np.ndarray:
    xyz = np.zeros((len(pts_mm), 3), np.float32)
    xyz[:, 0] = pts_mm[:, 0]
    xyz[:, 1] = pts_mm[:, 1]
    img, _ = cv2.projectPoints(xyz, rvec, tvec, K, dist)
    return img.reshape(-1, 2)


def run_self_test() -> int:
    """
    Pose conocida: cámara a 130 mm de altura, pitch ~55° hacia el piso.
    Proyecta CALIB_REAL_MM con barrel, ajusta H con y sin undistort, mide
    error de una grilla densa en BEV (debería ser cuadrada y métrica).
    """
    w, h = 640, 480
    K, dist = _synthetic_intrinsics(w, h)
    # world: X derecha, Y adelante, Z arriba. OpenCV cam: X der, Y abajo, Z fwd.
    # Rotación: world → camera. Pitch down ~55° around X after swapping axes.
    rx = np.deg2rad(90.0 + 55.0)  # Z_world up → Y_cam down-ish, look at ground
    rvec = np.array([rx, 0.0, 0.0], dtype=np.float64)
    tvec = np.array([0.0, 80.0, 130.0], dtype=np.float64)

    src_dist = _ground_to_cam(C.CALIB_REAL_MM, K, dist, rvec, tvec)
    src_ideal = _ground_to_cam(C.CALIB_REAL_MM, K, np.zeros(5), rvec, tvec)

    if np.any(src_dist[:, 0] < 0) or np.any(src_dist[:, 0] >= w) \
            or np.any(src_dist[:, 1] < 0) or np.any(src_dist[:, 1] >= h):
        print("[SELF] Aviso: algún punto de CALIB_REAL_MM cayó fuera de 640x480 "
              "(pose sintética). El test sigue con los que entren.", flush=True)

    dst = BEVTransformer._real_mm_to_bev_px(C.CALIB_REAL_MM)

    def _fit_and_grid_err(src):
        H, _ = cv2.findHomography(src.astype(np.float32), dst, 0)
        if H is None:
            return None, 1e9
        # Grilla 100 mm, proyectar cam→BEV y medir desviación vs destino métrico.
        xs = np.linspace(-200, 200, 9)
        ys = np.linspace(80, 550, 8)
        gx, gy = np.meshgrid(xs, ys)
        mm = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float32)
        cam = _ground_to_cam(mm, K, dist, rvec, tvec)
        bev_gt = BEVTransformer._real_mm_to_bev_px(mm)
        bev_hat = cv2.perspectiveTransform(
            cam.reshape(-1, 1, 2).astype(np.float32), H
        ).reshape(-1, 2)
        err = np.linalg.norm(bev_hat - bev_gt, axis=1)
        return H, float(np.mean(err))

    H_raw, err_raw = _fit_and_grid_err(src_dist)

    # Undistort the distorted clicks, then fit H (lo que hará calibrate.py).
    src_und = cv2.undistortPoints(
        src_dist.reshape(-1, 1, 2).astype(np.float32), K, dist, P=K
    ).reshape(-1, 2)
    H_und, _ = cv2.findHomography(src_und.astype(np.float32), dst, 0)

    # Misma grilla: undistort cada proyección distorsionada, luego H_und.
    xs = np.linspace(-200, 200, 9)
    ys = np.linspace(80, 550, 8)
    gx, gy = np.meshgrid(xs, ys)
    mm = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float32)
    cam_d = _ground_to_cam(mm, K, dist, rvec, tvec)
    cam_u = cv2.undistortPoints(
        cam_d.reshape(-1, 1, 2).astype(np.float32), K, dist, P=K
    ).reshape(-1, 1, 2)
    bev_gt = BEVTransformer._real_mm_to_bev_px(mm)
    bev_hat_u = cv2.perspectiveTransform(cam_u, H_und).reshape(-1, 2)
    err_und = float(np.linalg.norm(bev_hat_u - bev_gt, axis=1).mean())

    # Reproyección de los 9 puntos de calib (overfit) — siempre baja.
    def _reproj(src, H):
        p = cv2.perspectiveTransform(src.reshape(-1, 1, 2).astype(np.float32), H)
        return float(np.linalg.norm(p.reshape(-1, 2) - dst, axis=1).mean())

    print("=== self-test IPM (sintético, barrel k1=0.22) ===", flush=True)
    print(f"  reproy 9 pts  SIN undistort: {_reproj(src_dist, H_raw):.3f} px BEV",
          flush=True)
    print(f"  reproy 9 pts  CON undistort: {_reproj(src_und, H_und):.3f} px BEV",
          flush=True)
    print(f"  error grilla  SIN undistort: {err_raw:.2f} px BEV  "
          f"(~{err_raw * C.MM_PER_PX:.1f} mm)  ← H absorbe distorsión solo en 9 pts",
          flush=True)
    print(f"  error grilla  CON undistort: {err_und:.2f} px BEV  "
          f"(~{err_und * C.MM_PER_PX:.1f} mm)",
          flush=True)
    print(f"  9 pts undist vs ideales (sin dist): "
          f"{np.linalg.norm(src_und - src_ideal, axis=1).mean():.3f} px cámara",
          flush=True)

    ok = err_und < err_raw
    if ok:
        print("[SELF] OK: undistort + H mejora la grilla fuera de los 9 clics.",
              flush=True)
        return 0
    print("[SELF] FAIL: undistort no mejoró — revisar pose sintética.", flush=True)
    return 1


def replay_avi(avi_path: str, save_dir: str | None, max_frames: int,
               every: int) -> int:
    cap = cv2.VideoCapture(avi_path)
    if not cap.isOpened():
        print(f"[REPLAY] no se pudo abrir {avi_path}", flush=True)
        return 2

    bev = BEVTransformer()
    if not bev.is_calibrated:
        print("[REPLAY] sin bev_calib.npz — no se puede re-warp.", flush=True)
        return 2

    out_dir = Path(save_dir) if save_dir else None
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)

    print("[REPLAY] El AVI es HUD: izq=cámara (con texto) der=BEV ya transformada.",
          flush=True)
    print("[REPLAY] Re-warpeo el panel izquierdo con la H actual. No reestimo H "
          "(no hay marcadores de calibración en las corridas).", flush=True)
    if bev.uses_undistort():
        print("[REPLAY] undistort ON (H se ajustó con intrínsecos).", flush=True)
    else:
        print("[REPLAY] undistort OFF — igual que la corrida original.", flush=True)

    n_saved = 0
    k = 0
    while n_saved < max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        k += 1
        if (k - 1) % max(1, every) != 0:
            continue
        cam, stored_bev = split_hud(frame)
        re_bev = bev.warp(cam)
        if re_bev is None:
            continue
        # No se corre detect_centerline sobre el HUD: el texto/bboxes
        # ensucian la máscara de piso y el path no es el de la corrida.
        cam_grid = bev.draw_ground_grid(cam)
        cam_h = cam.shape[0]
        left = cam_grid
        mid = cv2.resize(re_bev, (cam_h, cam_h))
        if stored_bev is not None:
            right = cv2.resize(stored_bev, (cam_h, cam_h))
            combo = np.hstack([left, mid, right])
            cap_txt = "cam+grilla | re-warp H actual | BEV grabado"
        else:
            combo = np.hstack([left, mid])
            cap_txt = "cam+grilla | re-warp H actual"
        cv2.putText(combo, cap_txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
        n_saved += 1
        if out_dir is not None:
            fname = out_dir / f"replay_{k:05d}.jpg"
            cv2.imwrite(str(fname), combo)
            print(f"[REPLAY] {fname}", flush=True)
        else:
            cv2.imshow("replay BEV", combo)
            if cv2.waitKey(30) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[REPLAY] frames={n_saved}", flush=True)
    return 0


def main():
    p = argparse.ArgumentParser(description="Replay / self-test de calibración BEV")
    p.add_argument("--avi", type=str, default=None,
                   help="HUD de corrida (.avi/.mp4). Panel izq = cámara.")
    p.add_argument("--save-dir", type=str, default=None,
                   help="Si se pasa, escribe JPGs en vez de ventana.")
    p.add_argument("--every", type=int, default=15,
                   help="Tomar 1 de cada N frames del AVI.")
    p.add_argument("--max-frames", type=int, default=8)
    p.add_argument("--self-test", action="store_true",
                   help="Prueba sintética undistort vs homografía cruda.")
    args = p.parse_args()

    rc = 0
    if args.self_test:
        rc = run_self_test()
    if args.avi:
        rc2 = replay_avi(args.avi, args.save_dir, args.max_frames, args.every)
        rc = rc or rc2
    if not args.self_test and not args.avi:
        p.print_help()
        rc = 2
    sys.exit(rc)


if __name__ == "__main__":
    main()
