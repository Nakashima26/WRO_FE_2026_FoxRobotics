"""
pick_color.py — Picker HSV simple (un color a la vez, min/max crudo).

Para calibrar conos rojo/verde, cinta naranja y pared rosa CON otra luz
(recomendado, es el que usa Pure Pursuit):

    python3 -m pure_pursuit.calibra_luz --gui

POR QUÉ:
  FLOOR_LOWER_BLUE / FLOOR_UPPER_BLUE / FLOOR_LOWER_ORANGE / FLOOR_UPPER_ORANGE
  en config.py son valores de arranque — dependen de tu cámara e iluminación.
  Este script te deja clickear/arrastrar sobre los pixeles reales de las
  líneas en tu tapete y te imprime el rango HSV exacto que cubriste.

USO:
  # Sobre la imagen BEV en vivo (recomendado — mismo espacio que usa
  # centerline.py, así ves exactamente lo que "ve" el algoritmo):
  python pick_color.py --bev

  # Sobre el frame crudo de cámara (si aún no calibraste el BEV):
  python pick_color.py

  # Sobre una imagen guardada:
  python pick_color.py --image ruta/a/foto.jpg

CONTROLES:
  Click izquierdo + arrastrar : muestrea un parche de pixeles bajo el cursor
                                 (acumula min/max HSV de todo lo que arrastres)
  'r'                          : reinicia la acumulación (empezar de cero)
  'p'                          : imprime el rango acumulado ahora mismo
  ESC / 'q'                    : imprime rango final (con margen) y sale

El HSV bajo el cursor se muestra en vivo en la esquina superior izquierda,
junto con el rango acumulado hasta el momento.
"""

import argparse
import sys
import os

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

PATCH_HALF = 3   # radio del parche muestreado alrededor del cursor (px)
MARGIN_H   = 3   # margen de seguridad agregado al rango final
MARGIN_SV  = 15


class Picker:
    def __init__(self):
        self.h_min, self.s_min, self.v_min = 255, 255, 255
        self.h_max, self.s_max, self.v_max = 0, 0, 0
        self.n_samples = 0
        self.dragging = False

    def reset(self):
        self.__init__()

    def add(self, hsv_patch: np.ndarray):
        if hsv_patch.size == 0:
            return
        h, s, v = hsv_patch[..., 0], hsv_patch[..., 1], hsv_patch[..., 2]
        self.h_min = min(self.h_min, int(h.min())); self.h_max = max(self.h_max, int(h.max()))
        self.s_min = min(self.s_min, int(s.min())); self.s_max = max(self.s_max, int(s.max()))
        self.v_min = min(self.v_min, int(v.min())); self.v_max = max(self.v_max, int(v.max()))
        self.n_samples += hsv_patch.shape[0] * hsv_patch.shape[1] if hsv_patch.ndim >= 2 else hsv_patch.shape[0]

    def has_data(self) -> bool:
        return self.n_samples > 0

    def print_range(self, with_margin: bool = True):
        if not self.has_data():
            print("[pick_color] Sin muestras todavía — arrastra sobre el color primero.")
            return
        if with_margin:
            h_lo = max(0,   self.h_min - MARGIN_H)
            h_hi = min(179, self.h_max + MARGIN_H)
            s_lo = max(0,   self.s_min - MARGIN_SV)
            s_hi = min(255, self.s_max + MARGIN_SV)
            v_lo = max(0,   self.v_min - MARGIN_SV)
            v_hi = min(255, self.v_max + MARGIN_SV)
        else:
            h_lo, h_hi = self.h_min, self.h_max
            s_lo, s_hi = self.s_min, self.s_max
            v_lo, v_hi = self.v_min, self.v_max

        print(f"\n[pick_color] {self.n_samples} px muestreados"
              f" (margen {'+' if with_margin else 'sin'} aplicado)")
        print(f"  crudo : H[{self.h_min}-{self.h_max}] S[{self.s_min}-{self.s_max}] V[{self.v_min}-{self.v_max}]")
        print(f"  LOWER = np.array([{h_lo}, {s_lo}, {v_lo}])")
        print(f"  UPPER = np.array([{h_hi}, {s_hi}, {v_hi}])\n")


def run(get_frame, bev_mode: bool):
    picker = Picker()
    win = "pick_color BEV" if bev_mode else "pick_color camara"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    state = {"frame": None, "hsv": None, "mouse": (0, 0)}

    def on_mouse(event, x, y, flags, param):
        state["mouse"] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            picker.dragging = True
        elif event == cv2.EVENT_LBUTTONUP:
            picker.dragging = False
        if picker.dragging and state["hsv"] is not None:
            h, w = state["hsv"].shape[:2]
            x0, x1 = max(0, x - PATCH_HALF), min(w, x + PATCH_HALF + 1)
            y0, y1 = max(0, y - PATCH_HALF), min(h, y + PATCH_HALF + 1)
            picker.add(state["hsv"][y0:y1, x0:x1])

    cv2.setMouseCallback(win, on_mouse)

    print("[pick_color] Arrastra con click izq. sobre el color a medir.")
    print("[pick_color] 'r'=reset  'p'=imprimir ahora  ESC/'q'=salir e imprimir\n")

    while True:
        frame = get_frame()
        if frame is None:
            continue
        state["frame"] = frame
        state["hsv"] = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        disp = frame.copy()
        mx, my = state["mouse"]
        h, w = disp.shape[:2]
        if 0 <= mx < w and 0 <= my < h:
            hv = state["hsv"][my, mx]
            cv2.circle(disp, (mx, my), PATCH_HALF, (0, 255, 255), 1)
            txt_live = f"cursor HSV=({hv[0]},{hv[1]},{hv[2]})"
        else:
            txt_live = "cursor HSV=(-,-,-)"

        if picker.has_data():
            txt_acc = (f"acum H[{picker.h_min}-{picker.h_max}] "
                       f"S[{picker.s_min}-{picker.s_max}] V[{picker.v_min}-{picker.v_max}]  "
                       f"n={picker.n_samples}")
        else:
            txt_acc = "acum: (arrastra para muestrear)"

        cv2.putText(disp, txt_live, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(disp, txt_acc,  (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 0), 2)
        cv2.putText(disp, "r=reset  p=imprimir  ESC/q=salir", (8, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            picker.reset()
            print("[pick_color] Reset.")
        elif key == ord('p'):
            picker.print_range()
        elif key == 27 or key == ord('q'):
            break

    cv2.destroyAllWindows()
    picker.print_range()


def main():
    ap = argparse.ArgumentParser(description="Selector HSV interactivo para calibrar colores de piso")
    ap.add_argument("--image", type=str, default=None, help="Ruta a imagen fija en vez de cámara en vivo")
    ap.add_argument("--bev", action="store_true", help="Mostrar en espacio BEV (requiere calibración ya guardada)")
    ap.add_argument("--cam-index", type=int, default=None)
    args = ap.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[pick_color] No se pudo leer {args.image}")
            sys.exit(1)
        run(lambda: img, bev_mode=False)
        return

    from vision import open_camera
    from bev import BEVTransformer
    from config import CAM_INDEX
    from color_corr import FloorColorCorrector

    cam_index = args.cam_index if args.cam_index is not None else CAM_INDEX
    cap = open_camera(cam_index)
    bev = BEVTransformer() if args.bev else None
    corr = FloorColorCorrector()
    if args.bev and not bev.is_calibrated:
        print("[pick_color] --bev pedido pero no hay bev_calib.npz — corre calibrate.py primero.")
        sys.exit(1)

    def get_frame():
        ret, frame = cap.read()
        if not ret:
            return None
        frame = corr.process(frame)
        frame = cv2.flip(frame, 1)
        if bev is not None:
            return bev.warp(frame)
        return frame

    try:
        run(get_frame, bev_mode=args.bev)
    finally:
        cap.release()


if __name__ == "__main__":
    main()
