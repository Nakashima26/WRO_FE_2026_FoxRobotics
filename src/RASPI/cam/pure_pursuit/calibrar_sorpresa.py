"""Tabla de píxeles de un bloque NORMAL de 100 mm, con fotos del propio carro.

Las fotos deben estar enderezadas como las del runtime, con la misma resolución,
lente y calibración BEV. Un solo bloque rojo/verde de 100 mm por foto; moverlo a
varias distancias y ángulos, con su base y cima visibles.

python -m pure_pursuit.calibrar_sorpresa fotos/*.png --out altura_normal.json
Copiar SURPRISE_NORMAL_HEIGHT_SAMPLES del resultado a carrera.json.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from vision import Vision
from . import config as C
from .bev import BEVTransformer
from .surprise import ground_visible, validate_reference


def build_reference(frames, bev):
    vision = Vision(open_cam=False)
    vision.min_contour_area = C.SURPRISE_MIN_CONTOUR_AREA
    bins = {}
    skipped = 0
    for frame in frames:
        if frame is None or frame.shape[:2] != (480,640):
            raise ValueError("Se requieren fotos de proceso de 640x480, como la calibración BEV")
        _, positions = vision.detect_on(frame)
        boxes = [box for group in positions.values() for box in group]
        if len(boxes) != 1:
            skipped += 1
            continue
        x,y,w,h = box = boxes[0]
        if x <= 1 or y <= 1 or x+w >= 639 or y+h >= 479 or not ground_visible(frame,box):
            skipped += 1
            continue
        foot = bev.cam_to_bev(x+(w-1)/2, y+h-1)
        if foot is None:
            raise ValueError("Se requiere una calibración BEV válida")
        d = (C.ROBOT_BEV_Y-foot[1])*C.MM_PER_PX
        bins.setdefault(round(d/50), []).append((d,h))
    table = [np.median(values,axis=0).tolist() for _,values in sorted(bins.items())]
    validate_reference(table)
    if len(table) < 3 or table[-1][0]-table[0][0] < 300:
        raise ValueError("Se necesitan al menos tres distancias, cubriendo al menos 300 mm")
    return {"SURPRISE_NORMAL_HEIGHT_SAMPLES": table,
            "_photos_skipped": skipped,
            "_note": "Solo fotos de bloques de 100 mm; distancia del pie desde el origen BEV, no desde el lente"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('photos',type=Path,nargs='+')
    parser.add_argument('--bev-calib',type=Path,default=C.CALIB_FILE)
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    bev = BEVTransformer(args.bev_calib)
    if not bev.is_calibrated:
        parser.error('Falta la calibración BEV')
    data = build_reference((cv2.imread(str(p)) for p in args.photos),bev)
    args.out.write_text(json.dumps(data,indent=2,ensure_ascii=False))
    print(f"{len(data['SURPRISE_NORMAL_HEIGHT_SAMPLES'])} distancias -> {args.out}")


if __name__ == '__main__':
    main()
