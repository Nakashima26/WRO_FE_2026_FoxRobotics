#!/usr/bin/env python3
"""Línea óptima por carta / vuelta (vueltas 2–3, mapa ya conocido).

Compara contra DigitalMap._build_line y, si no revienta, contra plan_lap (QP).
En runtime solo se sigue la polilínea (PP); esto es generación offline.

Ejemplos (desde src/RASPI/cam):
  PYTHONPATH=. python pure_pursuit/twin/tools/line_card.py --card 26 --section N
  PYTHONPATH=. python pure_pursuit/twin/tools/line_card.py --seed 3170839 --lap
  PYTHONPATH=. python pure_pursuit/twin/tools/line_card.py --seed 3170839 --lap --mode chain
  PYTHONPATH=. python pure_pursuit/twin/tools/line_card.py --seed 3170839 --sweep-v0
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from pure_pursuit import corner_planner as cp
from pure_pursuit import racing_line_astar as rl
from pure_pursuit import route_planner as rp
from pure_pursuit import wro_field as wf


def _fmt(score: dict) -> str:
    side = score.get("side_ok")
    side_s = f"  side_ok={side}" if side is not None else ""
    return (f"T={score['time_s']:.3f}s  L={score['length_mm']:.0f}mm  "
            f"clear={score['clear_mm']:.1f}mm  kmax={score['kmax']:.5f}  "
            f"κ_ok={score['kappa_ok']}  [{score['method']}]  "
            f"{score.get('solve_ms', 0):.0f}ms{side_s}")


def _run_one(direction, section, pil, all_pillars, prm):
    center = rl.centerline_card(direction, section, pil, prm)
    planned = rl.plan_card(direction, section, pil, prm)
    print("center ", _fmt({**rl.score_route(center),
                           "side_ok": rp.side_ok(center.x, center.y, pil, closed=False)}))
    print("best   ", _fmt({**rl.score_route(planned),
                           "side_ok": rp.side_ok(planned.x, planned.y, pil, closed=False)}))
    try:
        lap = rp.plan_lap(direction, all_pillars)
        i0, _ic, i1, c, _t, _n, _ = rl.section_index_range(direction, section, prm.ds_mm)
        idx = rl._unwrap_indices(i0, i1, len(c))
        pref = c[idx]
        p = lap.xy()
        j0 = int(np.argmin(np.linalg.norm(p - pref[0], axis=1)))
        j1 = int(np.argmin(np.linalg.norm(p - pref[-1], axis=1)))
        if j1 < j0:
            j0, j1 = j1, j0
        x, y = lap.x[j0:j1 + 1], lap.y[j0:j1 + 1]
        sc = rl.score_xy(x, y, pil, prm.v0_mm_s)
        sc["method"] = "plan_lap slice"
        sc["solve_ms"] = lap.solve_ms
        print("qp     ", _fmt(sc))
    except Exception as e:
        print(f"qp      ERROR: {type(e).__name__}: {e}")


def _run_lap(field, prm, start_section: str, mode: str):
    signs = field.signs
    all_pil = [(s.x, s.y, s.color) for s in signs]
    print(f"seed={field.seed}  dir={field.direction}  cards={field.cards}  "
          f"start={start_section}  mode={mode}")
    for sec in rl.section_order(field.direction, start_section):
        pil = rl.pillars_for_section(field, sec)
        print(f"  {sec} card={field.cards[sec]} "
              f"{[(round(x), round(y), c) for x, y, c in pil]}")

    lap = rl.plan_field_lap(field, start_section=start_section, params=prm, mode=mode)
    if mode == "chain":
        for r in lap.cards:
            own = rl.pillars_for_section(field, r.section)
            print(f"card {r.section}#{r.card} ",
                  _fmt({**rl.score_route(r),
                        "side_ok": rp.side_ok(r.x, r.y, own, closed=False)}))

    print("LINE   ", _fmt({**rl.score_route(lap),
                           "side_ok": rp.side_ok(lap.x, lap.y, all_pil, closed=False)}))

    bx, by = rl.build_line_baseline(field.direction, signs)
    bsc = rl.score_xy(bx, by, all_pil, prm.v0_mm_s)
    print("BUILD  ", _fmt(bsc))

    dT = lap.time_s - bsc["time_s"]
    dC = lap.clear_mm - bsc["clear_mm"]
    print(f"delta(line-build_line)  T={dT:+.3f}s  clear={dC:+.1f}mm  "
          f"L={lap.length_mm - bsc['length_mm']:+.0f}mm")

    try:
        qp = rp.plan_lap(field.direction, all_pil)
        qsc = rl.score_xy(qp.x, qp.y, all_pil, prm.v0_mm_s, closed=True)
        qsc["method"] = "plan_lap"
        qsc["solve_ms"] = qp.solve_ms
        print("QP_LAP ", _fmt(qsc))
    except Exception as e:
        print(f"QP_LAP  ERROR: {type(e).__name__}: {e}")
    return 0


def _run_sweep_v0(field, start_section: str, clear_min: float, ds: float,
                  a_lat_max: float | None):
    all_pil = [(s.x, s.y, s.color) for s in field.signs]
    alat_s = f"a_lat<={a_lat_max:.0f}" if a_lat_max else "sin tope a_lat"
    print(f"seed={field.seed}  dir={field.direction}  start={start_section}  "
          f"clear_min={clear_min}mm  {alat_s}  (replan joint por v0)")
    print(f"{'pwm':>4} {'v0':>6} {'ok':>3} {'T':>7} {'L':>6} {'clear':>7} "
          f"{'vmin':>6} {'alat':>8} {'ms':>6}")
    prm = rl.LatticeParams(ds_mm=ds)
    rows = rl.sweep_v0_joint(field.direction, all_pil, start_section,
                             clear_min_mm=clear_min, a_lat_max=a_lat_max,
                             params=prm)
    for r in rows:
        if "error" in r:
            print(f"{r['pwm_eq']:4.0f} {r['v0']:6.0f} ERR {r['error'][:50]}")
            continue
        print(f"{r['pwm_eq']:4.0f} {r['v0']:6.0f} {'Y' if r['ok'] else 'n':>3} "
              f"{r['time_s']:7.2f} {r['length_mm']:6.0f} {r['clear_mm']:7.1f} "
              f"{r['v_min']:6.0f} {r['a_lat_max']:8.0f} {r['solve_ms']:6.0f}")
    best = rl.max_v0_feasible(rows)
    if best is None:
        print(f"MAX: ninguno factible con esos topes")
        return 1
    print(f"MAX factible: v0={best['v0']:.0f} mm/s  (~PWM {best['pwm_eq']:.0f})  "
          f"T={best['time_s']:.2f}s  clear={best['clear_mm']:.1f}mm  "
          f"v_min={best['v_min']:.0f}  a_lat_max={best['a_lat_max']:.0f} mm/s²")
    print("v0 = vel. recta modelo (3.5·(PWM-25)); T estimado scrub, no twin real. "
          "Firmware hoy velocidadMotor=160 → ~472 mm/s.")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--card", type=int, default=None)
    ap.add_argument("--section", default="N", choices=list(wf.SECTIONS_CW))
    ap.add_argument("--direction", default=None, choices=("CW", "CCW"))
    ap.add_argument("--lap", action="store_true",
                    help="vuelta completa y comparar vs _build_line")
    ap.add_argument("--corner", action="store_true",
                    help="plan de esquina open/closed/island al final de --section")
    ap.add_argument("--sweep-v0", action="store_true",
                    help="barrer v0 (replan joint) y reportar el máximo factible")
    ap.add_argument("--clear-min", type=float, default=15.0,
                    help="holgura mínima mm para ok en --sweep-v0")
    ap.add_argument("--a-lat-max", type=float, default=800.0,
                    help="tope a_lat=v²κ mm/s² en --sweep-v0 (0=sin tope); "
                         "default 800≈0.08g lona lisa")
    ap.add_argument("--mode", default="joint", choices=("joint", "chain"),
                    help="joint=DP anillo (default); chain=4 cartas cosidas")
    ap.add_argument("--start", default="N", choices=list(wf.SECTIONS_CW))
    ap.add_argument("--ds", type=float, default=40.0)
    args = ap.parse_args(argv)
    prm = rl.LatticeParams(ds_mm=args.ds)

    if args.sweep_v0:
        if args.seed is None:
            ap.error("--sweep-v0 requiere --seed")
        alat = None if args.a_lat_max <= 0 else args.a_lat_max
        return _run_sweep_v0(wf.randomize(args.seed), args.start, args.clear_min,
                             args.ds, alat)

    if args.lap:
        if args.seed is None:
            ap.error("--lap requiere --seed")
        return _run_lap(wf.randomize(args.seed), prm, args.start, args.mode)

    if args.corner:
        if args.seed is None:
            ap.error("--corner requiere --seed")
        field = wf.randomize(args.seed)
        direction = args.direction or field.direction
        pil = [(s.x, s.y, s.color) for s in field.signs]
        plan = cp.plan_corner(direction, args.section, pil,
                              parking=field.parking_section)
        print(f"seed={args.seed}  dir={direction}  corner {plan.section}→"
              f"{plan.next_section}")
        print(f"mode={plan.mode}  hint={plan.hint}  hold={plan.hold_dF_cm:.0f}cm  "
              f"release_along={plan.release_along_mm:.0f}mm")
        print(f"entry={plan.entry_lat_mm:.0f}  apex={plan.apex_lat_mm:.0f}  "
              f"exit={plan.exit_lat_mm:.0f}  T={plan.time_s:.3f}s  "
              f"clear={plan.clear_mm:.1f}mm  {plan.solve_ms:.0f}ms  gf={plan.gf_cm:.0f}")
        if plan.scored:
            for m, sc in plan.scored.items():
                ok = "Y" if sc.get("ok") else "n"
                print(f"  {m:7} {ok}  cost={sc.get('cost', float('inf')):.3f}  "
                      f"clear={sc.get('clear_mm', float('nan'))}")
        return 0

    if args.seed is not None:
        field = wf.randomize(args.seed)
        direction = args.direction or field.direction
        section = args.section
        pil = rl.pillars_for_section(field, section)
        all_pil = [(s.x, s.y, s.color) for s in field.signs]
        print(f"seed={args.seed}  dir={direction}  section={section}  "
              f"card={field.cards[section]}  "
              f"pillars={[(round(x), round(y), c) for x, y, c in pil]}")
        _run_one(direction, section, pil, all_pil, prm)
        return 0

    if args.card is not None:
        direction = args.direction or "CW"
        pil = rl.pillars_for_card(args.section, args.card)
        print(f"card={args.card}  dir={direction}  section={args.section}  "
              f"pillars={[(round(x), round(y), c) for x, y, c in pil]}")
        _run_one(direction, args.section, pil, pil, prm)
        return 0

    ap.error("indica --seed/--card o --seed --lap")
    return 2


if __name__ == "__main__":
    sys.exit(main())
