"""
Herramienta de calibración interactiva para BEV (Bird's Eye View).

USO:
  python -m pure_pursuit.calibrate          # desde src/RASPI/cam/
  python calibrate.py                       # desde pure_pursuit/

FLUJO:
  1. Coloca marcadores en el suelo en las posiciones EXACTAS definidas en
     config.py (CALIB_REAL_MM) — por defecto una grilla 3×3 (9 puntos)
     cubriendo cerca / media / lejos, cada una izquierda / centro / derecha.
     Imprime las posiciones al arrancar.
  2. Presiona 'C' para capturar el frame.
  3. Haz clic en cada marcador EN EL ORDEN mostrado en pantalla y consola.
     - Click IZQUIERDO: marca el punto actual.
     - Click DERECHO: deshace el último punto marcado (por si te equivocas).
     - Una ventana de "Zoom" muestra un acercamiento alrededor del cursor
       para ayudarte a clickear con precisión sub-píxel.
  4. Al completar todos los puntos aparece la vista BEV de preview en tiempo
     real, y en consola se imprime el ERROR DE REPROYECCIÓN de la calibración
     (qué tan bien la homografía explica tus clics). Si sale alto, revisa el
     punto señalado y rehaz.
  5. Presiona 'S' para guardar la calibración, 'R' para rehacer todo, ESC
     para salir.

La calibración se guarda en:  pure_pursuit/bev_calib.npz
"""

import os
import sys

import cv2
import numpy as np

# Agrega el directorio cam/ al path para poder importar vision.py
_HERE    = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import open_camera
from .bev import BEVTransformer
from . import config as C

# ── Etiquetas y colores de los puntos de calibración ─────────────────────────
# Se toman de config.py si existen (CALIB_POINT_LABELS), o se generan
# genéricamente para no romper si alguien usa una cantidad distinta de puntos.
N_POINTS = len(C.CALIB_REAL_MM)

if hasattr(C, "CALIB_POINT_LABELS") and len(C.CALIB_POINT_LABELS) == N_POINTS:
    POINT_LABELS = list(C.CALIB_POINT_LABELS)
else:
    POINT_LABELS = [f"P{i + 1}" for i in range(N_POINTS)]


def _make_point_colors(n: int) -> list[tuple[int, int, int]]:
    """Genera n colores distinguibles en BGR usando un colormap de OpenCV."""
    idx = np.linspace(0, 255, n).astype(np.uint8).reshape(-1, 1)
    colored = cv2.applyColorMap(idx, cv2.COLORMAP_TURBO)  # (n,1,3) BGR
    return [tuple(int(c) for c in colored[i, 0]) for i in range(n)]


POINT_COLORS = _make_point_colors(N_POINTS)

ZOOM_WIN      = "Calibracion BEV - Zoom"
ZOOM_HALF_PX  = 40    # radio (en px del frame original) que se recorta para el zoom
ZOOM_OUT_SIZE = 320   # tamaño de la ventana de zoom en píxeles


def _draw_instructions(frame: np.ndarray, clicks: list, done: bool) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]

    # Puntos ya clickeados
    for i, (px, py) in enumerate(clicks):
        cv2.circle(out, (px, py), 8, POINT_COLORS[i], -1)
        cv2.circle(out, (px, py), 8, (0, 0, 0), 2)
        cv2.putText(out, POINT_LABELS[i], (px + 10, py - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, POINT_COLORS[i], 2)

    # Siguiente punto esperado
    if not done:
        idx = len(clicks)
        msg = f"Clic en punto {POINT_LABELS[idx]}  ({idx + 1}/{N_POINTS})"
        cv2.putText(out, msg, (10, h - 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
        cv2.putText(out, "Click derecho = deshacer ultimo punto",
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 200), 1)
    else:
        cv2.putText(out, "S = guardar   R = rehacer   ESC = salir",
                    (10, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    return out


def _draw_zoom(frame: np.ndarray, cursor: tuple[int, int] | None,
               next_label: str | None) -> np.ndarray:
    """Recorte ampliado alrededor del cursor para clickear con más precisión."""
    h, w = frame.shape[:2]
    canvas = np.zeros((ZOOM_OUT_SIZE, ZOOM_OUT_SIZE, 3), dtype=np.uint8)

    if cursor is None:
        cv2.putText(canvas, "Mueve el mouse", (20, ZOOM_OUT_SIZE // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
        return canvas

    cx, cy = cursor
    x0, x1 = max(0, cx - ZOOM_HALF_PX), min(w, cx + ZOOM_HALF_PX)
    y0, y1 = max(0, cy - ZOOM_HALF_PX), min(h, cy + ZOOM_HALF_PX)
    crop = frame[y0:y1, x0:x1]
    if crop.size == 0:
        return canvas

    zoomed = cv2.resize(crop, (ZOOM_OUT_SIZE, ZOOM_OUT_SIZE), interpolation=cv2.INTER_NEAREST)

    # Retícula al centro (posición exacta donde caería el clic)
    cv2.line(zoomed, (ZOOM_OUT_SIZE // 2, 0), (ZOOM_OUT_SIZE // 2, ZOOM_OUT_SIZE), (0, 255, 0), 1)
    cv2.line(zoomed, (0, ZOOM_OUT_SIZE // 2), (ZOOM_OUT_SIZE, ZOOM_OUT_SIZE // 2), (0, 255, 0), 1)

    if next_label is not None:
        cv2.putText(zoomed, f"-> {next_label}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    return zoomed


def _draw_bev_preview(bev_img: np.ndarray, dst_pts: np.ndarray) -> np.ndarray:
    out = bev_img.copy()
    robot_x, robot_y = C.ROBOT_BEV_X, C.ROBOT_BEV_Y

    # Puntos destino esperados
    for i, (dx, dy) in enumerate(dst_pts):
        cv2.circle(out, (int(dx), int(dy)), 6, POINT_COLORS[i], -1)
        cv2.putText(out, str(i + 1), (int(dx) + 4, int(dy) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, POINT_COLORS[i], 2)

    # Robot
    cv2.circle(out, (robot_x, robot_y), 9, (255, 80, 0), -1)
    cv2.arrowedLine(out, (robot_x, robot_y), (robot_x, robot_y - 30),
                    (255, 255, 255), 2, tipLength=0.3)

    cv2.putText(out, "BEV preview", (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
    return out


def run_calibration(cam_index: int = C.CAM_INDEX) -> None:
    cap = open_camera(cam_index)

    # ── Paso 1: capturar frame ─────────────────────────────────────────────────
    print("Encuadra la cámara sobre el suelo y presiona 'C' para capturar.", flush=True)
    captured_frame = None
    cv2.namedWindow("Calibracion BEV - Captura", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        cv2.putText(frame, "Presiona C para capturar | ESC para salir",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.imshow("Calibracion BEV - Captura", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') or key == ord('C'):
            captured_frame = frame.copy()
            break
        if key == 27:
            cap.release()
            cv2.destroyAllWindows()
            return

    cv2.destroyWindow("Calibracion BEV - Captura")
    cv2.waitKey(100)

    # ── Paso 2: clic en los N puntos ──────────────────────────────────────────
    clicks: list[tuple[int, int]] = []
    cursor_pos: list[tuple[int, int] | None] = [None]   # mutable para el callback
    bev = BEVTransformer()
    dst_pts = BEVTransformer.expected_dst_pts()

    def on_mouse(event, x, y, flags, param):
        cursor_pos[0] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < N_POINTS:
            clicks.append((x, y))
            print(f"  [{len(clicks)}/{N_POINTS}] {POINT_LABELS[len(clicks) - 1]} -> ({x},{y})", flush=True)
        elif event == cv2.EVENT_RBUTTONDOWN and clicks:
            removed = clicks.pop()
            print(f"  Deshecho: {POINT_LABELS[len(clicks)]} ({removed[0]},{removed[1]})", flush=True)

    cv2.namedWindow("Calibracion BEV - Puntos", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Calibracion BEV - Preview BEV", cv2.WINDOW_NORMAL)
    cv2.namedWindow(ZOOM_WIN, cv2.WINDOW_NORMAL)
    cv2.waitKey(1)
    cv2.setMouseCallback("Calibracion BEV - Puntos", on_mouse)

    print(f"\nHaz clic en los {N_POINTS} marcadores en orden:", flush=True)
    for i, lbl in enumerate(POINT_LABELS):
        mm = C.CALIB_REAL_MM[i]
        print(f"  {lbl}: {mm[0]:+.0f} mm lateral, {mm[1]:.0f} mm adelante", flush=True)

    while True:
        done = len(clicks) == N_POINTS
        display = _draw_instructions(captured_frame, clicks, done)

        next_label = POINT_LABELS[len(clicks)] if not done else None
        zoom_view = _draw_zoom(captured_frame, cursor_pos[0], next_label)
        cv2.imshow(ZOOM_WIN, zoom_view)

        # Preview BEV en tiempo real al tener todos los puntos
        if done:
            src_pts = np.float32(clicks)
            try:
                bev.save(src_pts)
            except RuntimeError as e:
                print(e, flush=True)
                clicks.clear()
                continue

            ret, live_frame = cap.read()
            if ret:
                bev_img = bev.warp(live_frame)
                if bev_img is not None:
                    preview = _draw_bev_preview(bev_img, dst_pts)
                    cv2.imshow("Calibracion BEV - Preview BEV", preview)

        cv2.imshow("Calibracion BEV - Puntos", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') or key == ord('S'):
            if done:
                err = bev.last_reproj_err_px
                if err is not None and err > C.CALIB_MAX_MEAN_ERR_PX:
                    print(f"\n[AVISO] Guardaste con error de reproyección alto "
                          f"({err:.2f}px). Puedes rehacer con 'R' si quieres mejorarlo.",
                          flush=True)
                print(f"\n[OK] Calibración guardada en {C.CALIB_FILE}", flush=True)
                break
            else:
                print("Aún faltan puntos por seleccionar.", flush=True)

        elif key == ord('r') or key == ord('R'):
            clicks.clear()
            bev.H = None
            bev.H_inv = None
            print("Rehacer — haz clic en los puntos de nuevo.", flush=True)

        elif key == 27:
            print("Calibración cancelada.", flush=True)
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Calibración BEV para Pure Pursuit WRO")
    parser.add_argument("--cam-index", type=int, default=C.CAM_INDEX)
    args = parser.parse_args()
    run_calibration(args.cam_index)


if __name__ == "__main__":
    main()
