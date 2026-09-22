"""
Test visual de BEV + centerline + Pure Pursuit.
No requiere ESP32 ni serial solo cámara y bev_calib.npz.

USO:
  python -m pure_pursuit.test_vision          # desde src/RASPI/cam/
  python -m pure_pursuit.test_vision --video archivo.mp4

CONTROLES:
  ESC   : salir
  S     : guardar captura del frame actual como bev_snapshot.jpg
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

_HERE    = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import Vision
from .bev        import BEVTransformer
from .centerline import detect_centerline, map_obstacle_to_bev, draw_bev_debug
from .controller import PurePursuitController
from . import config as C


def run(cam_index: int = C.CAM_INDEX, video_path: str | None = None) -> None:
    # ── BEV + controlador ─────────────────────────────────────────────────────
    bev        = BEVTransformer()
    controller = PurePursuitController()
    vision     = Vision(cam_index)

    # ── Cámara o video ────────────────────────────────────────────────────────
    if video_path:
        vision.cap.release()
        cap = cv2.VideoCapture(video_path)
        print(f"[TEST] Usando video: {video_path}", flush=True)
    else:
        cap = vision.cap
        print(f"[TEST] Usando cámara índice {cam_index} con GStreamer", flush=True)

    vision.cap = cap

    if not cap.isOpened():
        print("[TEST] ERROR: No se pudo abrir la fuente de video.", flush=True)
        return

    if not bev.is_calibrated:
        print("[TEST] ADVERTENCIA: sin bev_calib.npz  solo se muestra la cámara cruda.", flush=True)
        print("[TEST] Corre primero: python -m pure_pursuit.calibrate", flush=True)
    else:
        print("[TEST] BEV calibrado OK. Mostrando pipeline completo.", flush=True)

    print("[TEST] ESC para salir | S para guardar snapshot", flush=True)

    loop      = 0
    fps       = 0.0
    t_last    = time.perf_counter()
    fps_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            if video_path:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)   # reiniciar video
                continue
            time.sleep(0.01)
            continue
        loop += 1
        frame = cv2.flip(frame, 1)
        fps_count += 1
        now = time.perf_counter()
        if now - t_last >= 1.0:
            fps     = fps_count / (now - t_last)
            fps_count = 0
            t_last  = now

        # ── Detección de obstáculos ───────────────────────────────────────────
        processed, positions = vision.process_frame(frame)

        if not bev.is_calibrated:
            cv2.putText(processed, "Sin calibracion BEV corre calibrate.py",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("TEST Vision", processed)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ── Pipeline BEV ─────────────────────────────────────────────────────
        try:
            bev_frame = bev.warp(processed)

            bev_obstacles: list[tuple[float, float, str]] = []
            for color_name in ("Red", "Green"):
                for obj in positions.get(color_name, []):
                    x, y, w, h = obj
                    result = map_obstacle_to_bev(bev, x, y, w, h)
                    if result is not None:
                        bev_obstacles.append((result[0], result[1], color_name))

            path_points = detect_centerline(bev_frame, bev_obstacles)
        except Exception as e:
            import traceback
            print(f"[ERROR] {e}", flush=True)
            traceback.print_exc()
            continue

        steer_deg   = 0.0
        lookahead_pt = (float(C.ROBOT_BEV_X), float(C.ROBOT_BEV_Y))
        pp_active   = False

        if len(path_points) >= C.MIN_PATH_PTS:
            steer_deg, lookahead_pt = controller.compute(
                path_points, C.ROBOT_BEV_X, C.ROBOT_BEV_Y
            )
            pp_active = True

        # ── Dibujar BEV debug ─────────────────────────────────────────────────
        bev_debug = draw_bev_debug(
            bev_frame, path_points, lookahead_pt,
            bev_obstacles, steer_deg, pp_active,
        )

        # ── Overlay en frame de cámara ────────────────────────────────────────
        lines = [
            f"fps={fps:.1f}  pp={'ON' if pp_active else 'OFF'}  pts={len(path_points)}",
            f"steer={steer_deg:+.1f} deg",
            f"obs_R={len(positions.get('Red',[]))}  obs_G={len(positions.get('Green',[]))}",
        ]
        y_txt = 22
        for txt in lines:
            cv2.putText(processed, txt, (10, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            y_txt += 22

        # ── Mostrar lado a lado ───────────────────────────────────────────────
        cam_h = processed.shape[0]
        bev_small = cv2.resize(bev_debug, (cam_h, cam_h))
        combined  = np.hstack([processed, bev_small])
        cv2.imshow("TEST Vision - camara | BEV", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord('s') or key == ord('S'):
            fname = f"bev_snapshot_{loop:04d}.jpg"
            cv2.imwrite(fname, combined)
            print(f"[TEST] Guardado: {fname}", flush=True)

    cap.release()
    cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description="Test visual BEV + Pure Pursuit (sin serial)")
    p.add_argument("--cam-index", type=int, default=C.CAM_INDEX)
    p.add_argument("--video",     type=str, default=None,
                   help="Ruta a video .mp4 en lugar de cámara")
    args = p.parse_args()
    run(cam_index=args.cam_index, video_path=args.video)


if __name__ == "__main__":
    main()
