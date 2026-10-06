"""Harness corto de la SALIDA DEL CAJÓN (T15a), hw_nuevo.

Una corrida:  salida.py one <escenario> <dir> [extra_json]
Lote:         salida.py all <tag> [jobs] [escenarios…]   (desde src/RASPI/cam)

Escenario:
  CW:5 / CCW:27[:N]  -> campo armado a mano: sentido, carta de la recta del
                        cajón (1-6, 25-30) y recta del cajón (default N).
  s3170839           -> semilla estándar (randomize), truncada igual.
Por defecto el lote son las 10 cartas distintas × CW/CCW + las 21 semillas.
EXTRA (env o argumento) = {"pi": {...}, "fw": {...}, "def": {...}, "period": s,
"src": "head"} ("head" = .ino del commit HEAD, para medir la base).

Mide cada 5 ms (en el chequeo de choque del sim) la holgura mínima de la
huella completa (180×130 con voladizo) contra señales, maderas, pared exterior
e isla, y de qué lado cruza el carro cada señal de la recta del cajón.
"""
from __future__ import annotations

import concurrent.futures as cf
import csv
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

MAX_T = float(os.environ.get("SALIDA_MAX_T", "14"))
CARDS_PARK = (1, 2, 3, 4, 5, 6, 25, 26, 27, 30)   # 28≡26, 29≡27
SEEDS = list(range(1, 21)) + [3170839]


def default_scenarios() -> list[str]:
    return [f"{d}:{c}" for d in ("CW", "CCW") for c in CARDS_PARK] + [f"s{s}" for s in SEEDS]


# ── campo armado ─────────────────────────────────────────────────────────────
def build_field(direction: str, card: int, park: str = "N"):
    from pure_pursuit import wro_field as W
    order = list(W.SECTIONS_CW) if direction == "CW" else list(reversed(W.SECTIONS_CW))
    i = order.index(park)
    nxt, nxt2, prev = order[(i + 1) % 4], order[(i + 2) % 4], order[(i + 3) % 4]
    # Resto fijo: la recta siguiente con la señal única roja (X2), las otras dos
    # con cartas mixtas. Solo importa la primera esquina.
    cards = {park: card, nxt: W.CARD_SINGLE_RED, nxt2: 26, prev: 14}
    signs = []
    for sec, n in cards.items():
        for seat, color in W.CARDS[n - 1]:
            h, w = W.SEATS[seat]
            x, y = W.section_to_world(sec, h, w)
            signs.append(W.Sign(x, y, color, sec, seat))
    sx, sy = W.stall_start_xy(park, direction)
    return W.Field(
        seed=0, direction=direction, single_section=nxt, parking_section=park,
        cards=cards, signs=signs, barriers=W._barrier_corners(park),
        start_x=sx, start_y=sy, start_heading=W._heading(park, direction),
        start_mode="cajon",
    )


def to_hw(sec: str, x: float, y: float) -> tuple[float, float]:
    """Inversa de wro_field.section_to_world."""
    if sec == "N":
        return 1500.0 - y, x + 1500.0
    if sec == "S":
        return y + 1500.0, 1500.0 - x
    if sec == "E":
        return 1500.0 - x, 1500.0 - y
    return x + 1500.0, y + 1500.0


# ── geometría de holguras ────────────────────────────────────────────────────
def _seg_pt(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)


def poly_dist(P, Q) -> float:
    """Distancia entre polígonos convexos (0 si se tocan)."""
    from pure_pursuit.twin.world import _hits_polygon
    if _hits_polygon(P, Q):
        return 0.0
    eP = list(zip(P, P[1:] + P[:1])); eQ = list(zip(Q, Q[1:] + Q[:1]))
    return min(min(_seg_pt(p, a, b) for p in P for a, b in eQ),
               min(_seg_pt(q, a, b) for q in Q for a, b in eP))


# ── una corrida ──────────────────────────────────────────────────────────────
def run_one(scn: str, out: Path, extra: dict) -> None:
    from pure_pursuit.twin import sim as S
    from pure_pursuit.twin import world as Wd
    from pure_pursuit.twin.firmware import fw as F
    from pure_pursuit.wro_field import SIGN_MM

    out.mkdir(parents=True, exist_ok=True)
    if scn.startswith("s"):
        seed = int(scn[1:]); fld_fix = None
    else:
        parts = scn.split(":")
        seed = 1; fld_fix = build_field(parts[0], int(parts[1]), parts[2] if len(parts) > 2 else "N")
        S.randomize = lambda _seed, start="cajon": fld_fix

    dbg = open(out / "fw_debug.log", "w", encoding="utf-8")
    _orig = F.FirmwareSIL.pop_debug
    buf = {"t": ""}

    def pop2(self):
        s = _orig(self)
        if s:
            t = self.now_us / 1e6
            lines = (buf["t"] + s).split("\n")
            buf["t"] = lines.pop()
            for ln in lines:
                dbg.write(f"{t:8.3f} {ln}\n")
        return s
    F.FirmwareSIL.pop_debug = pop2

    st = {"field": None, "min": {}, "samples": []}
    _coll = S.collision

    def coll(world, x, y, hd, L, Wd_, ro, **kw):
        # **kw: Sim pasa ignore= desde 87532bb (T15b). Sin esto el TypeError
        # se lo traga el callback ctypes de advance: ni colisiones ni mínimos.
        fld = world.field
        st["field"] = fld
        P = [tuple(c) for c in Wd.body_corners(x, y, hd, L, Wd_, ro)]
        mins = st["min"]
        half = SIGN_MM / 2.0
        for s in fld.signs:
            if abs(s.x - x) > 700 or abs(s.y - y) > 700:
                continue
            box = [(s.x - half, s.y - half), (s.x + half, s.y - half), (s.x + half, s.y + half), (s.x - half, s.y + half)]
            k = f"sign {s.color[0]} {s.section}/{s.seat}"
            mins[k] = min(mins.get(k, 1e9), poly_dist(P, box))
        if "x0" not in st:
            st["x0"] = (x, y)
        moved = math.hypot(x - st["x0"][0], y - st["x0"][1]) > 15.0   # el arranque está a 2-5 mm a propósito
        for b in fld.barriers if moved else ():
            mins["stall"] = min(mins.get("stall", 1e9), poly_dist(P, [tuple(p) for p in b]))
        mins["wall"] = min(mins.get("wall", 1e9), min(1500.0 - max(abs(cx), abs(cy)) for cx, cy in P))
        isl = [(500.0, -500.0), (500.0, 500.0), (-500.0, 500.0), (-500.0, -500.0)]
        mins["island"] = min(mins.get("island", 1e9), poly_dist(P, isl))
        st["samples"].append((S.Sim._now_hack(), x, y, hd))
        return _coll(world, x, y, hd, L, Wd_, ro, **kw)

    S.collision = coll
    sim = S.Sim(seed, preset="hw_nuevo", log_dir=out, max_time_s=MAX_T,
                pi_overrides=extra.get("pi"), fw_overrides=extra.get("fw"),
                fw_defines=extra.get("def"), pi_period_s=extra.get("period"),
                fw_source=extra.get("src"))
    clock = {"fw": None}
    _fw_init = F.FirmwareSIL.__init__

    def fw_init(self, *a, **k):
        _fw_init(self, *a, **k); clock["fw"] = self
    F.FirmwareSIL.__init__ = fw_init
    S.Sim._now_hack = staticmethod(lambda: clock["fw"].now_us / 1e6 if clock["fw"] else 0.0)
    r = sim.run()
    dbg.close()
    fld = r.field
    m = dict(r.metrics); m.pop("corners", None)

    # Cruces de las señales de la recta del cajón (centro del chasis).
    park = fld.parking_section; sgn = 1.0 if fld.direction == "CW" else -1.0
    ahead = 146.5 - 90.0  # centro del chasis respecto al eje trasero
    cross = []
    signs = [s for s in fld.signs if s.section == park]
    prev = None
    for t, x, y, hd in st["samples"]:
        h_r = math.radians(hd)
        cx, cy = x + ahead * math.sin(h_r), y + ahead * math.cos(h_r)
        hh, ww = to_hw(park, cx, cy)
        if prev is not None and 0.0 <= hh <= 1000.0:     # solo dentro de la recta del cajón
            for s in signs:
                sh, sw = to_hw(park, s.x, s.y)
                if (prev[1] - sw) * (ww - sw) < 0:
                    fwd = (ww - prev[1]) * sgn > 0
                    right = (hh > sh) if fld.direction == "CW" else (hh < sh)
                    ok = right == (s.color == "Red")
                    cross.append({"t": round(t, 2), "sign": f"{s.color[0]} {s.seat}", "fwd": fwd,
                                  "side": "der" if right else "izq", "ok": ok, "h": round(hh)})
        prev = (hh, ww)
    t_tc1 = None
    for row in r.trace:
        a = row.get("ack") or ""
        if ",tc=" in a:
            try:
                if int(a.split(",tc=")[1].split(",")[0]) >= 1:
                    t_tc1 = round(row["t"], 2); break
            except ValueError:
                pass
    fwlog = (out / "fw_debug.log").read_text(encoding="utf-8")
    t_ini = None
    for ln in fwlog.splitlines():
        if "INICIO completado" in ln:
            t_ini = float(ln.split()[0]); break
    res = {
        "scn": scn, "dir": fld.direction, "park": park, "card": fld.cards[park],
        "stop": r.stop_reason, "hit": (m.get("collisions") or [{}])[0].get("cause") if m.get("collisions") else None,
        "hit_t": (m.get("collisions") or [{}])[0].get("t") if m.get("collisions") else None,
        "min": {k: round(v) for k, v in st["min"].items()}, "cross": cross,
        "t_inicio": t_ini, "t_tc1": t_tc1, "tc": m.get("turns_completed"),
    }
    (out / "salida.json").write_text(json.dumps(res, ensure_ascii=False), encoding="utf-8")
    (out / "field.txt").write_text(
        f"dir={fld.direction} parking={park} start=({fld.start_x:.0f},{fld.start_y:.0f},{fld.start_heading})\n"
        + "\n".join(f"{s.color} {s.section}/{s.seat} ({s.x:.0f},{s.y:.0f})" for s in fld.signs), encoding="utf-8")
    keys = sorted({k for row in r.trace for k in row})
    with open(out / "trace.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(r.trace)
    print("SALIDA", json.dumps(res, ensure_ascii=False))


# ── resumen ──────────────────────────────────────────────────────────────────
def summarize(d: Path, scns: list[str]) -> None:
    # Alcance: hasta 1 s después de completar el 1er giro (tc=1). Un choque
    # después se lista pero no cuenta (ya es carrera, no salida).
    print("| esc | dir/park | carta | choque | min señal recta | min otras | min cajón | min isla | cruces (señal lado ok) | t_ini | t_tc1 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    n = ok_n = 0
    for scn in scns:
        p = d / scn.replace(":", "_") / "salida.json"
        n += 1
        if not p.exists():
            print(f"| {scn} | ERROR |"); continue
        r = json.loads(p.read_text(encoding="utf-8"))
        mn = r["min"]
        park = r["park"]
        ms = min([v for k, v in mn.items() if k.startswith("sign") and k.split()[2].startswith(park + "/")] or [9999])
        mo = min([v for k, v in mn.items() if k.startswith("sign") and not k.split()[2].startswith(park + "/")] or [9999])
        cr = " ".join(f"{c['sign']}{'' if c['fwd'] else '(rev)'}:{c['side']}{'' if c['ok'] else '!'}" for c in r["cross"]) or "-"
        bad_side = any(c["fwd"] and not c["ok"] for c in r["cross"])
        late = r["hit"] and r["t_tc1"] is not None and r["hit_t"] > r["t_tc1"] + 1.0
        good = (not r["hit"] or late) and not bad_side
        ok_n += good
        hit = (f"{r['hit']}@{r['hit_t']:.2f}" + (" (fuera de alcance)" if late else "")) if r["hit"] else "-"
        print(f"| {scn} | {r['dir']}/{r['park']} | {r['card']} | {hit} | {ms} | {mo} | {mn.get('stall')} | {mn.get('island')} | {cr} | {r['t_inicio']} | {r['t_tc1']} |{'' if good else ' X'}")
    print(f"sin_contacto_y_lado_ok={ok_n}/{n}")


def main() -> None:
    mode = sys.argv[1]
    extra_env = json.loads(os.environ.get("EXTRA", "null") or "null") or {}
    if mode == "one":
        extra = json.loads(sys.argv[4]) if len(sys.argv) > 4 else extra_env
        run_one(sys.argv[2], Path(sys.argv[3]), extra or {})
        return
    if mode == "summ":
        d = Path(sys.argv[2]); scns = sys.argv[3:] or default_scenarios()
        summarize(d, scns); return
    tag = sys.argv[2]; jobs = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    scns = sys.argv[4:] or default_scenarios()
    d = Path("runs") / tag; d.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONUTF8="1", PYTHONPATH=".", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    t0 = time.time()

    def job(scn):
        sd = d / scn.replace(":", "_")
        with open(str(sd) + ".out", "w", encoding="utf-8") as fo:
            subprocess.run([sys.executable, __file__, "one", scn, str(sd), json.dumps(extra_env)],
                           stdin=subprocess.DEVNULL, stdout=fo, stderr=subprocess.STDOUT, env=env)
    with cf.ThreadPoolExecutor(jobs) as ex:
        list(ex.map(job, scns))
    print(f"[{time.time() - t0:.0f} s]")
    summarize(d, scns)


if __name__ == "__main__":
    main()
