"""Saca la vista de arriba del carro (un .usdc) a twin/assets/car_top.json.

Uso (pxr no está en el .venv: pip install --target <dir> usd-core):
  PYTHONPATH=<dir> python car_top.py VehicleDirTest_V2.usdc
Cada pieza queda como su envolvente convexa vista desde arriba, en mm del
carro (x = derecha, y = adelante, origen en el eje trasero, como body_corners),
ordenadas de abajo hacia arriba para pintarlas en ese orden.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from pxr import Usd, UsdGeom, UsdShade

stage = Usd.Stage.Open(sys.argv[1])
assert UsdGeom.GetStageUpAxis(stage) == "Y" and UsdGeom.GetStageMetersPerUnit(stage) == 1.0
xf = UsdGeom.XformCache()
parts = []
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    pts = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float)
    m = np.array(xf.GetLocalToWorldTransform(prim))
    w = pts @ m[:3, :3] + m[3, :3]
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    rgb = (0.5, 0.5, 0.5)
    if mat:
        c = mat.ComputeSurfaceSource()[0].GetInput("diffuseColor")
        rgb = tuple(c.Get()) if c else rgb
    parts.append((prim.GetName() if prim.GetName() != "Mesh" else prim.GetParent().GetName(), w, rgb))

allw = np.vstack([w for _, w, _ in parts])
cx = (allw[:, 0].min() + allw[:, 0].max()) / 2
# Eje trasero = centro de las ruedas traseras; adelante es -Z (la cámara).
rear = np.vstack([w for n, w, _ in parts if n.startswith("RuedaAtras")])
z_axle = (rear[:, 2].min() + rear[:, 2].max()) / 2
out = []
for name, w, rgb in sorted(parts, key=lambda p: p[1][:, 1].max()):
    xy = np.c_[(w[:, 0] - cx) * 1000, (z_axle - w[:, 2]) * 1000].astype(np.float32)
    hull = cv2.convexHull(xy)
    if cv2.contourArea(hull) < 15:  # tornillería y pines: no se ven a 0.2 px/mm
        continue
    hull = cv2.approxPolyDP(hull, 0.5, True).reshape(-1, 2)
    bgr = [int(round(c * 255)) for c in rgb[::-1]]
    out.append({"name": name, "bgr": bgr, "pts": np.round(hull, 1).tolist()})
dst = Path(__file__).resolve().parents[1] / "assets" / "car_top.json"
dst.write_text(json.dumps(out, separators=(",", ":")))
print(dst, len(out), "piezas", "x", np.ptp(allw[:, 0]) * 1000, "z", np.ptp(allw[:, 2]) * 1000)
