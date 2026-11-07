"""
Calibración de LENTE (matriz K + distorsión) con tablero de ajedrez.

USO:
  python -m pure_pursuit.calib_lente              # desde src/RASPI/cam/ (Pi, con cámara)
  python -m pure_pursuit.calib_lente --images DIR # offline: calibra desde jpgs en DIR

FLUJO:
  1. Imprime un tablero de --cols x --rows esquinas INTERNAS (cuadros de
     --square-mm) y pégalo plano sobre algo rígido.
  2. La cámara corre SIN enderezar. Mueve el tablero por todo el campo: cerca,
     lejos, esquinas, inclinado. ESPACIO captura cuando el tablero se detecta;
     si es casi la misma pose que una captura anterior (centro a <15 px y mismo
     tamaño) la rechaza.
  3. Con >= --min capturas presiona 'C': calibra e imprime el error de
     reproyección. 'U' alterna la vista enderezada, 'S' guarda, ESC sale.
  4. Guarda en pure_pursuit/lens_calib.npz (o --out). vision.open_camera lo
     aplica solo. Después REHAZ la calibración de 9 puntos: la anterior queda
     inválida.
"""

import argparse
import datetime as dt
import glob
import hashlib
import math
import os
import sys

import cv2
import numpy as np

# Mismo manejo de path que calibrate.py: vision.py vive en src/RASPI/cam/
_HERE    = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import open_camera
from . import config as C

FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
SUB_CRIT   = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
WIN        = "Calibracion lente"
TOO_CLOSE_PX = 15.0   # centro a menos de esto de una captura previa = misma pose


def board_points(cols, rows, square_mm):
    """Puntos 3D del tablero (z=0) en el mismo orden que findChessboardCorners."""
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return obj


def calibrate_from_points(obj_points, img_points, size, alpha=1.0):
    """K y distorsión (modelo de 5 coeficientes) + newK para el alpha dado."""
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    per_view = []
    for op, ip, rv, tv in zip(obj_points, img_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        per_view.append(float(np.linalg.norm(ip.reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean()))
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, size, alpha, size)
    return dict(K=K, dist=dist, newK=newK, rms=float(rms), per_view_err=per_view, size=tuple(size))


def save_lens(path, res, alpha):
    """Guarda la calibración. El sig identifica esta versión (bev_calib.npz lo guarda)."""
    sig = hashlib.sha1(np.round(res["K"], 3).tobytes() + np.round(res["dist"], 5).tobytes()
                       + str(alpha).encode()).hexdigest()[:10]
    np.savez(str(path), K=res["K"], dist=res["dist"], newK=res["newK"],
             size=np.array(res["size"]), rms=res["rms"], alpha=alpha, sig=sig)
    return sig


def print_result(res, alpha):
    K, d, newK = res["K"], res["dist"].ravel(), res["newK"]
    w, _ = res["size"]
    errs = res["per_view_err"]
    rms = res["rms"]
    worst = np.argsort(errs)[::-1][:3]
    print(f"\nRMS = {rms:.3f} px")
    print("peores vistas: " + ", ".join(f"#{i + 1} {errs[i]:.2f}px" for i in worst))
    print(f"fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
    print(f"k1={d[0]:+.4f} k2={d[1]:+.4f} p1={d[2]:+.5f} p2={d[3]:+.5f} k3={d[4]:+.4f}")
    print(f"HFOV de newK (alpha {alpha}): {math.degrees(2 * math.atan(w / 2 / newK[0, 0])):.1f} deg")
    guia = "excelente" if rms < 0.5 else "bien" if rms < 1.0 else "REHACER (fotos movidas o tablero doblado)"
    print(f"calidad (rms < 0.5 excelente, < 1.0 bien, > 1.0 rehacer): {guia}")
    print("\n" + "#" * 100)
    print("# AHORA REHAZ la calibracion de 9 puntos (python -m pure_pursuit.calibrate): la anterior queda invalida")
    print("#" * 100 + "\n", flush=True)


def _board_geom(corners):
    """Centro y diagonal (px) del tablero detectado."""
    pts = corners.reshape(-1, 2)
    return pts.mean(axis=0), float(np.linalg.norm(pts[0] - pts[-1]))


def run_offline(a):
    paths = sorted(p for ext in ("*.jpg", "*.png") for p in glob.glob(os.path.join(a.images, ext)))
    obj = board_points(a.cols, a.rows, a.square_mm)
    objs, imgs, size = [], [], None
    for p in paths:
        gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"no se pudo leer {p}", flush=True)
            continue
        size = size or gray.shape[::-1]
        if gray.shape[::-1] != size:
            print(f"tamaño distinto, se salta {os.path.basename(p)}", flush=True)
            continue
        found, corners = cv2.findChessboardCorners(gray, (a.cols, a.rows), FIND_FLAGS)
        if not found:
            print(f"sin tablero: {os.path.basename(p)}", flush=True)
            continue
        imgs.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUB_CRIT))
        objs.append(obj)
    print(f"{len(imgs)}/{len(paths)} imagenes con tablero", flush=True)
    if len(imgs) < a.min:
        print(f"faltan capturas: {len(imgs)}/{a.min}", flush=True)
        return 1
    res = calibrate_from_points(objs, imgs, size, a.alpha)
    print_result(res, a.alpha)
    sig = save_lens(a.out, res, a.alpha)
    print(f"[LENTE] guardado {a.out} (sig {sig})", flush=True)
    return 0


def run_live(a):
    cap = open_camera(C.CAM_INDEX, undistort=False)   # frames crudos: aquí se calibra
    obj = board_points(a.cols, a.rows, a.square_mm)
    save_dir = a.save_dir or os.path.join("runs", "calib_lente", dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(save_dir, exist_ok=True)
    objs, imgs, seen = [], [], []   # seen: (centro, diagonal) de cada captura aceptada
    res, undist = None, False
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    print("ESPACIO captura | C calibra | U preview enderezado | S guarda | ESC sale", flush=True)
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (a.cols, a.rows), FIND_FLAGS)
        if undist:
            view = cv2.undistort(frame, res["K"], res["dist"], None, res["newK"])
        else:
            view = frame.copy()
            if found:
                cv2.drawChessboardCorners(view, (a.cols, a.rows), corners, found)
        for c, _ in seen:   # pistas de cobertura: dónde ya hay capturas
            cv2.rectangle(view, (int(c[0]) - 10, int(c[1]) - 10), (int(c[0]) + 10, int(c[1]) + 10), (0, 255, 255), 1)
        cv2.putText(view, f"capturas {len(imgs)}/{a.min}  {'tablero OK' if found else 'sin tablero'}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow(WIN, view)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(" ") and found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUB_CRIT)
            c, s = _board_geom(corners)
            if any(np.linalg.norm(c - pc) < TOO_CLOSE_PX and abs(s - ps) < 0.15 * ps for pc, ps in seen):
                print("muy parecida, mueve el tablero", flush=True)
                continue
            seen.append((c, s))
            objs.append(obj)
            imgs.append(corners)
            cv2.imwrite(os.path.join(save_dir, f"cap_{len(imgs):03d}.jpg"), frame)
            print(f"captura {len(imgs)} aceptada", flush=True)
        elif key in (ord("c"), ord("C")):
            if len(imgs) < a.min:
                print(f"faltan capturas: {len(imgs)}/{a.min}", flush=True)
            else:
                res = calibrate_from_points(objs, imgs, gray.shape[::-1], a.alpha)
                print_result(res, a.alpha)
                undist = True
        elif key in (ord("u"), ord("U")) and res is not None:
            undist = not undist
        elif key in (ord("s"), ord("S")):
            if res is None:
                print("calibra primero (C)", flush=True)
            else:
                sig = save_lens(a.out, res, a.alpha)
                print(f"[LENTE] guardado {a.out} (sig {sig})", flush=True)
    cap.release()
    cv2.destroyAllWindows()


def draw_board(cols, rows, square_mm, dpi=300):
    """Tablero imprimible de (cols+1)x(rows+1) cuadros con 10 mm de margen blanco."""
    px = square_mm / 25.4 * dpi
    nx, ny, m = cols + 1, rows + 1, int(round(10 / 25.4 * dpi))
    img = np.full((int(round(ny * px)) + 2 * m, int(round(nx * px)) + 2 * m), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                x0, y0 = m + int(round(i * px)), m + int(round(j * px))
                img[y0:m + int(round((j + 1) * px)), x0:m + int(round((i + 1) * px))] = 0
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description="Calibración de lente con tablero de ajedrez")
    ap.add_argument("--cols", type=int, default=9, help="esquinas internas a lo ancho")
    ap.add_argument("--rows", type=int, default=6, help="esquinas internas a lo alto")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--alpha", type=float, default=1.0,
                    help="getOptimalNewCameraMatrix: 1 = no pierde campo de vision")
    ap.add_argument("--min", type=int, default=15, help="capturas mínimas para calibrar")
    ap.add_argument("--out", default=str(C.LENS_CALIB_FILE))
    ap.add_argument("--save-dir", default=None, help="carpeta de las capturas jpg (modo cámara)")
    ap.add_argument("--images", default=None, help="DIR con jpgs del tablero: calibra sin cámara")
    ap.add_argument("--tablero", default=None, help="solo escribe un PNG imprimible del tablero (300 dpi) y sale")
    a = ap.parse_args(argv)
    if a.tablero:
        cv2.imwrite(a.tablero, draw_board(a.cols, a.rows, a.square_mm))
        print(f"[LENTE] tablero en {a.tablero}: imprime al 100 % (sin ajustar a página) y mide un cuadro = {a.square_mm:g} mm")
        return 0
    if a.images:
        return run_offline(a)
    run_live(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
