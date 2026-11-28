"""Radio de giro con el volante a tope (eje trasero), real o gemelo.

  python radio_tope.py runs/orillas409_tele.jsonl ...      # corridas reales: Δod / Δyaw
  python radio_tope.py lat.noindex/<tag>/n1/s*/trace.csv  # gemelo: Δs(x,y) / Δheading

Agrupa por (estado, adelante/reversa, lado del tope) y descarta los primeros 0.25 s de cada tramo
(el servo todavía viaja). 11-27, 399-410: reversa srv=1 ≈162 mm, srv=156 ≈250; el gemelo 107-136.
"""
import csv
import json
import math
import sys


def _tramos(rows):
    """rows: (t, est, srv, s_mm, yaw_deg, signo) con s acumulado; rinde (clave, radio)."""
    i = 0
    while i < len(rows):
        t0, est, srv = rows[i][:3]
        lado = "L" if srv >= 150 else "R" if srv <= 5 else None
        if lado is None or est not in ("E", "I"):
            i += 1
            continue
        j = i
        while j + 1 < len(rows) and rows[j + 1][1] == est and (rows[j + 1][2] >= 150 if lado == "L" else rows[j + 1][2] <= 5):
            j += 1
        seg = [r for r in rows[i:j + 1] if r[0] - t0 >= 0.25]
        if len(seg) >= 3:
            ds = seg[-1][3] - seg[0][3]
            dy = abs(seg[-1][4] - seg[0][4])
            if dy >= 10 and abs(ds) >= 30:
                yield (est, "rev" if seg[len(seg) // 2][5] < 0 else "fwd", lado), abs(ds) / math.radians(dy)
        i = j + 1


def leer(path):
    rows = []
    if path.endswith(".csv"):
        s = 0.0
        prev = None
        for r in csv.DictReader(open(path)):
            x, y = float(r["x"]), float(r["y"])
            if prev:
                s += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
            rows.append((float(r["t"]), r["est"], float(r["servo"]), s, float(r["heading"]), int(r["motor_dir"] or 0)))
        return rows
    prev_od = 0.0
    for line in open(path):
        try:
            sample = json.loads(line)["s"]
        except (ValueError, KeyError):
            continue
        a = sample.get("ack") or {}
        if a.get("od") is None or a.get("yaw") is None or a.get("srv") is None:
            continue
        od = float(a["od"])
        rows.append((sample["t"], a.get("est"), float(a["srv"]), od, float(a["yaw"]), -1 if od < prev_od else 1))
        prev_od = od
    return rows


def main(paths):
    res = {}
    for p in paths:
        for k, r in _tramos(leer(p)):
            res.setdefault(k, []).append(r)
    for k, v in sorted(res.items()):
        v.sort()
        print(*k, f"n={len(v)} mediana={v[len(v) // 2]:.0f} rango={v[0]:.0f}-{v[-1]:.0f}")


if __name__ == "__main__":
    assert next(_tramos([(0, "E", 1, 0, 0, -1), (0.3, "E", 1, -10, 0, -1), (0.4, "E", 1, -50, 20, -1),
                         (0.5, "E", 1, -110, 40, -1)]))[1] == 100 / math.radians(40)
    main(sys.argv[1:])
