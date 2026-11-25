"""Hoja de cuadros del video de una corrida real (HUD + mapa) en tiempos de la telemetría.

Uso:
  python cuadros_run.py orillas256.avi orillas256_tele.jsonl 18.4:20.9[:0.3] [4.0:5.0 ...] -o hoja.jpg

Los tiempos son segundos desde el inicio del _tele.jsonl (los mismos que imprime
resumen_run.py). El video se graba a fps variable: se mapea por duración total.
Cada rango da una columna de cuadros; máx. 9 cuadros por rango.
"""
import argparse
import json

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("avi")
    ap.add_argument("tele")
    ap.add_argument("rangos", nargs="+", help="ini:fin[:paso] en s")
    ap.add_argument("-o", "--out", default="hoja.jpg")
    ap.add_argument("--ancho", type=int, default=800)
    a = ap.parse_args()

    lines = open(a.tele, encoding="utf-8").read().splitlines()
    t0 = json.loads(lines[0])["t"]
    dur = json.loads(lines[-1])["t"] - t0
    rec0 = 0.13   # el video arranca ~0.13 s después de la telemetría ([REC] Grabando)
    for l in lines[1:40]:
        for te, e in json.loads(l).get("ev", []):
            if "Grabando en" in e:
                rec0 = te - t0
    cap = cv2.VideoCapture(a.avi)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f)
    if not frames:
        raise SystemExit("video vacío")
    k = len(frames) / max(dur - rec0, 1e-3)
    h = int(a.ancho * frames[0].shape[0] / frames[0].shape[1])

    cols = []
    for r in a.rangos:
        p = [float(v) for v in r.split(":")]
        ini, fin = p[0], p[1]
        paso = p[2] if len(p) > 2 else max((fin - ini) / 8, 0.1)
        ts = list(np.arange(ini, fin + 1e-6, paso))[:9]
        col = []
        for t in ts:
            i = min(max(int((t - rec0) * k), 0), len(frames) - 1)
            f = cv2.resize(frames[i], (a.ancho, h))
            cv2.putText(f, f"t~{t:.1f}", (int(a.ancho * 0.36), 28), 0, 0.9, (0, 255, 255), 2)
            col.append(f)
        while len(col) < 9 and cols:
            col.append(np.zeros_like(col[0]))
        cols.append(np.vstack(col))
    n = max(c.shape[0] for c in cols)
    cols = [np.vstack([c, np.zeros((n - c.shape[0], c.shape[1], 3), np.uint8)]) for c in cols]
    cv2.imwrite(a.out, np.hstack(cols), [cv2.IMWRITE_JPEG_QUALITY, 80])
    print(a.out, f"{len(frames)} cuadros, {k:.2f} fps")


if __name__ == "__main__":
    main()
