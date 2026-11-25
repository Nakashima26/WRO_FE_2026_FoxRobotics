#!/usr/bin/env python3
"""Desglose del tiempo de vuelta 1 por tc y limitador de velocidad (motor_pwm en trace)."""
from __future__ import annotations

import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

DT = 1.0 / 40.0

_CNR_HI = 162
_FLR_LO, _FLR_HI = 163, 215
_TOP_LO = 216


def _tc(ack: str) -> float | None:
    m = re.search(r"tc=([\d.]+)", ack or "")
    return float(m.group(1)) if m else None


def _lim(pwm: int, spd: float) -> str:
    if spd < 20:
        return "hold"
    if pwm <= _CNR_HI:
        return "corner_sign"
    if _FLR_LO <= pwm <= _FLR_HI:
        return "piso_kappa"
    if pwm >= _TOP_LO:
        return "unseen_upload"
    return "near_sign"


def _lap1_slice(rows: list[dict]) -> list[dict]:
    out: list[dict] = []
    for r in rows:
        if r.get("pi_frame") != "True":
            continue
        tc = _tc(r.get("ack") or "")
        if tc is None or tc >= 4:
            if tc is not None and tc >= 4 and out:
                break
            continue
        out.append(r)
    return out


def _analyze(seed_dir: Path) -> dict | None:
    tr = seed_dir / "trace.csv"
    if not tr.is_file():
        return None
    rows = list(csv.DictReader(open(tr, encoding="utf-8")))
    l1 = _lap1_slice(rows)
    if not l1:
        return None
    by_tc: dict[int, dict[str, float]] = defaultdict(lambda: defaultdict(float))
    by_lim: dict[str, float] = defaultdict(float)
    prev_tc = None
    for r in l1:
        tc = int(_tc(r.get("ack") or "") or 0)
        pwm = int(float(r.get("motor_pwm") or 0))
        spd = float(r.get("speed_mm_s") or 0)
        lim = _lim(pwm, spd)
        by_tc[tc][lim] += DT
        by_lim[lim] += DT
        prev_tc = tc
    return {"by_tc": {k: dict(v) for k, v in sorted(by_tc.items())}, "by_lim": dict(by_lim), "total_s": sum(by_lim.values())}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    run = Path(args.run_dir)
    seeds = sorted(d for d in run.iterdir() if d.is_dir() and d.name.startswith("s"))
    agg_tc: dict[int, dict[str, float]] = defaultdict(lambda: defaultdict(float))
    agg_lim: dict[str, float] = defaultdict(float)
    n = 0
    for sd in seeds:
        a = _analyze(sd)
        if not a:
            continue
        n += 1
        for tc, d in a["by_tc"].items():
            for lim, t in d.items():
                agg_tc[tc][lim] += t
        for lim, t in a["by_lim"].items():
            agg_lim[lim] += t
    if n == 0:
        raise SystemExit("sin trazas")
    scale = 1.0 / n
    lims = sorted(agg_lim, key=lambda k: -agg_lim[k])
    print(f"# v1 (tc<4), medias sobre {n} semillas, dt/cuadro={DT}s")
    print("| tc | " + " | ".join(lims) + " | total |")
    print("|---|---|" + "---|" * (len(lims) + 1))
    for tc in sorted(agg_tc):
        d = {k: v * scale for k, v in agg_tc[tc].items()}
        tot = sum(d.values())
        print(f"| {tc} | " + " | ".join(f"{d.get(l, 0):.2f}" for l in lims) + f" | {tot:.2f} |")
    tot_all = sum(agg_lim[k] * scale for k in lims)
    print("| ALL | " + " | ".join(f"{agg_lim[l] * scale:.2f}" for l in lims) + f" | {tot_all:.2f} |")
    if args.json:
        print(json.dumps({"n": n, "by_lim_mean": {l: agg_lim[l] * scale for l in lims}, "by_tc": agg_tc}, indent=2))


if __name__ == "__main__":
    main()