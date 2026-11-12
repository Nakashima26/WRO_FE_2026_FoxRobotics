"""
mide_exposicion.py — Exposición FIJA de la cámara para la sede (Pi 5, Picamera2).

POR QUÉ
  Con la exposición automática, dentro del cajón (pared rosa oscura) la cámara sube la
  exposición y al salir el piso queda quemado ~1 s (orillas57, 2026-11-12: la cinta
  naranja no se vio a tiempo y el cajón se forzó 12 cm mal). Con exposición fija la
  imagen no cambia al salir. vision.open_camera usa camera_tuning/exposicion.json si existe.

USO (desde src/RASPI/cam/, con el servicio parado: sudo systemctl stop wro-runtime && sleep 4)

  # 1) Carro en la PISTA, fuera del cajón, viendo el pasillo. Mide lo que pone la
  #    automática ahí y lo guarda como fija:
  python3 -m pure_pursuit.mide_exposicion --guardar

  # 2) Comprobar con la fija (carro dentro del cajón: el rosa debe seguir pasando de 0.28):
  python3 -m pure_pursuit.mide_exposicion --probar

  Quitar la fija: borrar camera_tuning/exposicion.json (o FOX_CAM_EXPO=0).
  Después: sudo systemctl start wro-runtime
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

_CAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from pure_pursuit import config as C  # noqa: E402

_TUNING = os.path.join(_CAM_DIR, "camera_tuning", "imx219_noir_wro_pi5.json")
_GAINS = (1.10, 1.37)


def _abrir(expo):
    from picamera2 import Picamera2
    cam = Picamera2(tuning=Picamera2.load_tuning_file(_TUNING))
    controls = {"FrameRate": 40, "AwbEnable": False, "ColourGains": _GAINS}
    if expo:
        controls.update({"AeEnable": False, **expo})
    cam.configure(cam.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}, raw={"size": (1640, 1232)},
        buffer_count=4, controls=controls))
    cam.start()
    return cam


def _stats(frame) -> str:
    luma = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    top = int(frame.shape[0] * float(C.PARK_PINK_ROI_TOP))
    roi = hsv[top:]
    pink = np.zeros(roi.shape[:2], np.uint8)
    for lo, hi in C.PARK_PINK_HSV:
        pink |= cv2.inRange(roi, lo, hi)
    orange = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in C.LINE_ORANGE_HSV:
        orange |= cv2.inRange(hsv, lo, hi)
    p50, p99 = np.percentile(luma, [50, 99])
    return (f"luma p50={p50:.0f} p99={p99:.0f} quemado(>=250)={100 * (luma >= 250).mean():.1f}%  "
            f"rosa={(pink > 0).mean():.2f} (umbral {C.PARK_PINK_RATIO_MIN})  naranja px={int((orange > 0).sum())}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    p.add_argument("--guardar", action="store_true", help="guardar la medida en camera_tuning/exposicion.json")
    p.add_argument("--probar", action="store_true", help="abrir con la fija del json y mostrar cómo se ve")
    p.add_argument("--escala", type=float, default=1.0, help="multiplica el tiempo de exposición medido (p. ej. 0.8)")
    p.add_argument("--seg", type=float, default=2.0, help="segundos de medición")
    p.add_argument("--jpg", default="/tmp/exposicion.jpg", help="frame de muestra")
    p.add_argument("--archivo", default=os.path.join(_CAM_DIR, "camera_tuning", "exposicion.json"),
                   help="json de la exposición fija (el que lee vision.py)")
    a = p.parse_args()

    expo = None
    if a.probar:
        if not os.path.exists(a.archivo):
            sys.exit(f"No hay exposición fija (falta {a.archivo}).")
        with open(a.archivo) as f:
            d = json.load(f)
        expo = {"ExposureTime": int(d["ExposureTime"]), "AnalogueGain": float(d["AnalogueGain"])}
    cam = _abrir(expo)
    try:
        time.sleep(3.0)   # que la automática converja (o la fija se asiente)
        ts, gs, frame = [], [], None
        t0 = time.time()
        while time.time() - t0 < a.seg:
            frame = cam.capture_array("main")
            md = cam.capture_metadata()
            ts.append(md.get("ExposureTime", 0))
            gs.append(md.get("AnalogueGain", 0.0) * md.get("DigitalGain", 1.0))
        t, g = float(np.median(ts)), float(np.median(gs))
        print(f"[EXPO] {'fija' if expo else 'automática'}: ExposureTime={t:.0f} us (min {min(ts)} max {max(ts)}), "
              f"ganancia={g:.2f}")
        print(f"[EXPO] {_stats(frame)}")
        cv2.imwrite(a.jpg, frame)
        print(f"[EXPO] frame en {a.jpg}")
        if a.guardar:
            out = {"ExposureTime": int(round(t * a.escala)), "AnalogueGain": round(g, 3),
                   "_medido": time.strftime("%Y-%m-%d %H:%M:%S"), "_escala": a.escala}
            with open(a.archivo, "w") as f:
                json.dump(out, f, indent=1)
            print(f"[EXPO] guardado {a.archivo}: {out}")
    finally:
        cam.stop()
        cam.close()


if __name__ == "__main__":
    main()
