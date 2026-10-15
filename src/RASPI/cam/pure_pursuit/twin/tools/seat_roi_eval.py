"""Evalúa la clasificación por ROI (líneas [ROI] de log.txt) contra la verdad (field.txt).

Uso: python seat_roi_eval.py <dir1> [<dir2> ...]
     python seat_roi_eval.py --col <dir1> [<dir2> ...]   (líneas [COL]: clasificación por columna)
Cada dir es la salida de drive.py: log.txt y pi.log con [ROI]/[ROI-OUT], y field.txt con la verdad.
Asiento sin línea en field.txt = vacío.
"""

import re
import sys
from collections import defaultdict
from pathlib import Path

BANDS = [(0, 500), (500, 1000), (1000, 1500), (1500, 2000), (2000, 2500), (2500, 3000), (3000, 3500)]
CLASSES = ["Red", "Green", "Empty"]
ROI_RE = re.compile(r"^\[ROI\] t=\d+ sec=(\w) seat=(\w+) d=(\d+) b=\S+ w=\S+ cls=(\w+) fr=\S+ fg=\S+ unk=(\d)")
OUT_RE = re.compile(r"^\[ROI-OUT\] t=\d+ sec=(\w) seat=(\w+) d=(\d+)")
FIELD_RE = re.compile(r"^(Red|Green) (\w)/(\w+) \(")
COL_RE = re.compile(r"^\[COL\] t=\d+ sec=(\w) col=(\d) d=(\d+) b=(\S+) yaw=(\S+) cls=(\w+) nr=\d+ ng=\d+ area=\S+ occ=(\d)")
COL_SEATS = {"0": ["T4", "T2"], "1": ["X2", "X1"], "2": ["T3", "T1"]}   # = _COL de route_mode.py


def load(d: Path):
    truth = {}
    for line in (d / "field.txt").read_text(encoding="utf-8").splitlines()[1:]:
        m = FIELD_RE.match(line)
        if m:
            truth[(m.group(2), m.group(3))] = m.group(1)
    rows, outs = [], []
    # las líneas [ROI] salen por pi.log (stdout de la Pi en el twin); log.txt en otros casos
    logs = [d / n for n in ("log.txt", "pi.log") if (d / n).exists()]
    lines = [ln for p in logs for ln in p.read_text(encoding="utf-8", errors="replace").splitlines()]
    for line in lines:
        m = ROI_RE.match(line)
        if m:
            sec, seat, dist, cls, unk = m.group(1), m.group(2), int(m.group(3)), m.group(4), int(m.group(5))
            rows.append((truth.get((sec, seat), "Empty"), cls, dist, unk, sec, seat))
            continue
        m = OUT_RE.match(line)
        if m:
            outs.append(int(m.group(3)))
    return rows, outs


def band(dist: int) -> int:
    return min(dist // 500, len(BANDS) - 1)


def main(dirs):
    rows, outs = [], []
    for d in map(Path, dirs):
        r, o = load(d)
        print(f"{d}: {len(r)} mediciones, {len(o)} fuera de imagen")
        rows += r
        outs += o
    per = defaultdict(list)
    for row in rows:
        per[band(row[2])].append(row)
    out_per = defaultdict(int)
    for dist in outs:
        out_per[band(dist)] += 1

    print()
    print(f"{'banda mm':>10} {'n':>6} {'acierto':>8} {'fuera':>6} {'peligr':>7}   matriz (filas verdad, cols predicho: Red Green Empty)")
    for i, (lo, hi) in enumerate(BANDS):
        items = per.get(i, [])
        n = len(items)
        ok = sum(1 for t, p, *_ in items if t == p)
        danger = sum(1 for t, p, *_ in items if t in ("Red", "Green") and p != t)
        acc = f"{100.0 * ok / n:.1f}%" if n else "-"
        cm = {(t, p): 0 for t in CLASSES for p in CLASSES}
        for t, p, *_ in items:
            cm[(t, p)] += 1
        mat = " | ".join(f"{t[:3]}: " + " ".join(f"{cm[(t, p)]:4d}" for p in CLASSES) for t in CLASSES)
        print(f"{lo:>4}-{hi:<5} {n:>6} {acc:>8} {out_per.get(i, 0):>6} {danger:>7}   {mat}")

    # sanity: señal real a menos de 800 mm
    near = [r for r in rows if r[2] < 800 and r[0] in ("Red", "Green")]
    near_ok = sum(1 for r in near if r[0] == r[1])
    print()
    print(f"sanity d<800 con señal real: {near_ok}/{len(near)} correctas")
    unk = sum(1 for r in rows if r[3] == 1)
    print(f"mediciones con unk=1 (sin resolver en el mapa): {unk}")


def load_col(d: Path):
    truth = {}
    for line in (d / "field.txt").read_text(encoding="utf-8").splitlines()[1:]:
        m = FIELD_RE.match(line)
        if m:
            truth[(m.group(2), m.group(3))] = m.group(1)
    rows = []
    logs = [d / n for n in ("log.txt", "pi.log") if (d / n).exists()]
    for p in logs:
        for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
            m = COL_RE.match(line)
            if not m or m.group(7) != "0":
                continue
            sec, col, dist = m.group(1), m.group(2), int(m.group(3))
            colors = {truth[(sec, t)] for t in COL_SEATS[col] if (sec, t) in truth}
            tr = "Empty" if not colors else colors.pop() if len(colors) == 1 else "Mixed"
            b, yaw = float(m.group(4)), float(m.group(5))
            rows.append((tr, m.group(6), dist, b, yaw))
    return rows


def col_table(title, rows):
    per = defaultdict(list)
    for r in rows:
        per[band(r[2])].append(r)
    print(title)
    print(f"{'banda mm':>10} {'n':>6} {'acierto':>8} {'peligr':>7}   matriz (filas verdad, cols predicho: Red Green Empty Mixed)")
    cls4 = CLASSES + ["Mixed"]
    for i, (lo, hi) in enumerate(BANDS):
        items = per.get(i, [])
        n = len(items)
        ok = sum(1 for t, p, *_ in items if t == p)
        danger = sum(1 for t, p, *_ in items if t in ("Red", "Green") and p != t)
        acc = f"{100.0 * ok / n:.1f}%" if n else "-"
        cm = defaultdict(int)
        for t, p, *_ in items:
            cm[(t, p)] += 1
        mat = " | ".join(f"{t[:3]}: " + " ".join(f"{cm[(t, p)]:4d}" for p in cls4) for t in cls4)
        print(f"{lo:>4}-{hi:<5} {n:>6} {acc:>8} {danger:>7}   {mat}")
    print()


def col_main(dirs):
    rows = []
    for d in map(Path, dirs):
        r = load_col(d)
        print(f"{d}: {len(r)} columnas (occ=0)")
        rows += r
    print()
    col_table("TODAS (occ=0)", rows)
    col_table("FILTRO |yaw|<=1.0 deg/frame y |b|<=40 deg", [r for r in rows if abs(r[4]) <= 1.0 and abs(r[3]) <= 40.0])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if sys.argv[1] == "--col":
        col_main(sys.argv[2:])
    else:
        main(sys.argv[1:])
