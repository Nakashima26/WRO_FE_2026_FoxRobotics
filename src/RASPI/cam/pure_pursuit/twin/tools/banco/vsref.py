"""Compara variantes contra una referencia corrida por corrida.

Uso: python vsref.py REF_DIR VAR_DIR [VAR_DIR...]   (dirs con cand_n1..5)
Tiempo de vuelta 1 geométrico: primer movimiento -> primer cruce del ángulo de salida.
"""
import collections
import csv
import json
import os
import statistics as st
import sys

import numpy as np

SEEDS = [str(i) for i in range(1, 42)] + ["3170839"]


def one(path):
    try:
        ls = [x for x in open(path + ".out", encoding="utf-8").read().splitlines() if x.startswith("STOP ")]
        stop, js = ls[-1].split(" ", 2)[1:]
        m = json.loads(js)
    except Exception:
        return dict(ok=False, why="noSTOP")
    bad = stop != "race_finished" or bool(m.get("collisions")) or bool(m.get("sign_contacts"))
    # Choque ya con las 3 vueltas hechas (TERMINANDO del bench): en la carrera ahí sigue la U, no cuenta.
    if stop == "collision" and len(m.get("lap_times_s") or []) >= 3:
        bad = False
    if bad:
        c = m.get("collisions") or []
        why = c[0].get("cause", "?") if c and isinstance(c[0], dict) else stop
        return dict(ok=False, why=why)
    rows = list(csv.DictReader(open(os.path.join(path, "trace.csv"), encoding="utf-8")))
    t = np.array([float(r["t"]) for r in rows])
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    sp = np.array([float(r["speed_mm_s"] or 0) for r in rows])
    ang = np.unwrap(np.arctan2(y, x))
    sgn = -1.0 if ang[-1] < ang[0] else 1.0
    t0 = t[np.argmax(sp > 1)]
    cr = []
    for k in (1, 2, 3):
        w = np.nonzero(sgn * (ang - ang[0]) >= 2 * np.pi * k)[0]
        cr.append(t[w[0]] if len(w) else np.nan)
    return dict(ok=True, total=m["total_time_s"], lap1=cr[0] - t0, loop2=cr[1] - cr[0], loop3=cr[2] - cr[1],
                exitI=m["time_per_state_s"].get("I", 0.0))


def load(d):
    return {(n, s): one(os.path.join(d, f"cand_n{n}", f"s{s}")) for n in range(1, 6) for s in SEEDS}


def main():
    ref = load(sys.argv[1])
    fref = collections.Counter(s for (n, s), r in ref.items() if not r["ok"])
    print(f"ref {sys.argv[1]}: fallas {sum(fref.values())}")
    for d in sys.argv[2:]:
        var = load(d)
        fvar = collections.Counter(s for (n, s), r in var.items() if not r["ok"])
        only_v = sum(1 for k in ref if ref[k]["ok"] and not var[k]["ok"])
        only_r = sum(1 for k in ref if var[k]["ok"] and not ref[k]["ok"])
        new2 = [s for s, c in fvar.items() if c >= 2 and fref[s] == 0]
        both = [k for k in ref if ref[k]["ok"] and var[k]["ok"]]
        dd = {f: [var[k][f] - ref[k][f] for k in both if not np.isnan(var[k][f] - ref[k][f])]
              for f in ("total", "lap1", "loop2", "loop3", "exitI")}
        med = {f: round(st.median(v), 2) for f, v in dd.items()}
        lap1 = round(st.median([var[k]["lap1"] for k in both]), 2)
        why = collections.Counter(var[k]["why"] for k in var if not var[k]["ok"])
        print(f"\n{os.path.basename(d)}: fallas {sum(fvar.values())}  solo-var {only_v}  solo-ref {only_r}  "
              f"seeds nuevas >=2 {new2}")
        print(f"   dt mediana vs ref: {med}   vuelta1 mediana {lap1} s  (n={len(both)})")
        print(f"   seeds que fallan: {dict(sorted(fvar.items(), key=lambda x: -x[1]))}")
        print(f"   causas: {dict(why.most_common(6))}")


if __name__ == "__main__":
    main()
