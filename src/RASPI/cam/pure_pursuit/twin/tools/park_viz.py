"""Vista cenital del cajón con el carro en cada cambio de fase. Desde src/RASPI/cam:

  PYTHONPATH=. <venv>/python pure_pursuit/twin/tools/park_viz.py runs/<tag>/<escenario>

Lee trace.csv y fw_debug.log del escenario (salida de tools/prepark.py) y escribe
park_viz.png en la misma carpeta: rastro del eje trasero, carro al entrar a cada
fase (color por fase, etiqueta con t y fase) y el carro final en negro.
"""
from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, ".")

from pure_pursuit import config as C  # noqa: E402
from pure_pursuit.twin import prepark as PP  # noqa: E402
from pure_pursuit.twin.world import body_corners  # noqa: E402

_FASE_RE = re.compile(r"^\s*([\d.]+) PARK(?: fase (\d+)|: ATRAS PEGADO en fase (\d+))")
_COLORS = [(230, 25, 75), (60, 180, 75), (0, 130, 200), (245, 130, 48), (145, 30, 180),
           (70, 240, 240), (240, 50, 230), (128, 128, 0), (0, 0, 128), (170, 110, 40)]


def main(run_dir: str, scale: float = 1.2, margin: float = 260.0) -> Path:
    d = Path(run_dir)
    d_name, c, p = d.name.split("_")
    field, _ = PP.build(PP.Scenario(d_name, int(c.lstrip("c")), p))
    rows = list(csv.DictReader(open(d / "trace.csv", encoding="utf-8")))
    ts = np.array([float(r["t"]) for r in rows])
    xs = np.array([float(r["x"]) for r in rows])
    ys = np.array([float(r["y"]) for r in rows])
    hs = np.array([float(r["heading"]) for r in rows])
    fases = []
    for ln in open(d / "fw_debug.log", encoding="utf-8"):
        m = _FASE_RE.match(ln)
        if m:
            fases.append((float(m.group(1)), m.group(2) or "x", ln.split("PARK", 1)[1].strip()[:60]))

    bx = [q[0] for b in field.barriers for q in b]
    by = [q[1] for b in field.barriers for q in b]
    x0, x1 = min(bx) - margin, max(bx) + margin
    y0, y1 = min(by) - margin, max(by) + margin
    W, H = int((x1 - x0) * scale), int((y1 - y0) * scale)
    img = np.full((H + 20 * (len(fases) + 2), W, 3), 245, np.uint8)

    def px(x, y):
        return int((x - x0) * scale), int((y1 - y) * scale)

    def car(x, y, h, color, thick):
        cs = body_corners(x, y, h, C.ROBOT_LENGTH_MM, C.ROBOT_WIDTH_MM, C.REAR_OVERHANG_MM)
        cv2.polylines(img, [np.array([px(*q) for q in cs], np.int32)], True, color, thick, cv2.LINE_AA)
        cv2.circle(img, px(x, y), 2, color, -1)

    # Pared exterior: el lado del rectángulo de las maderas que toca el borde.
    for b in field.barriers:
        cv2.fillPoly(img, [np.array([px(*q) for q in b], np.int32)], (200, 0, 200))
    keep = (xs > x0) & (xs < x1) & (ys > y0) & (ys < y1)
    pts = np.array([px(x, y) for x, y in zip(xs[keep], ys[keep])], np.int32)
    if len(pts) > 1:
        cv2.polylines(img, [pts], False, (150, 150, 150), 1, cv2.LINE_AA)
    for i, (t, f, txt) in enumerate(fases):
        k = min(int(np.searchsorted(ts, t)), len(ts) - 1)
        col = _COLORS[i % len(_COLORS)]
        car(xs[k], ys[k], hs[k], col, 1)
        cv2.putText(img, f"{t:6.2f} {txt}", (4, H + 16 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1,
                    cv2.LINE_AA)
    car(xs[-1], ys[-1], hs[-1], (0, 0, 0), 2)
    cv2.putText(img, f"final t={ts[-1]:.2f} hdg={hs[-1]:.1f}", (4, H + 16 + 20 * len(fases)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)
    out = d / "park_viz.png"
    cv2.imwrite(str(out), img)
    return out


if __name__ == "__main__":
    print(main(sys.argv[1]))
