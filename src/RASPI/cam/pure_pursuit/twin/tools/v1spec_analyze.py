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
    print("\n# estrategia sugerida por tc (NOTA 1)")
    print("| tc | dominante | reactivo | especulativo | mapa |")
    print("|---|---|---|---|---|")
    _hint = {
        0: ("hold+piso (fix)", "—", "—", "ubica cajón"),
        1: ("corner_sign", "rectas con ROI/prov", "esquinas T ciegas", "confirmados"),
        2: ("corner_sign+piso", "asiento ya visible", "T sin votos", "half_seats"),
        3: ("piso_kappa+corner", "señal frontal vista", "últimas T", "freeze próximo"),
    }
    for tc in sorted(agg_tc):
        dom = max(agg_tc[tc], key=agg_tc[tc].get)
        r, e, m = _hint.get(tc, ("", "", "", ""))[1:]
        print(f"| {tc} | {dom} | {r} | {e} | {m} |")


def audit_visible_brakes(run_dir: Path) -> None:
    """Cuenta cuadros corner_sign (pwm<=162) en tc 1-3 con asiento ya en self.visible (proxy: prov en log)."""
    import re
    wasted = 0
    corner = 0
    seeds = sorted(d for d in run_dir.iterdir() if d.is_dir() and d.name.startswith("s"))
    for sd in seeds:
        tr = sd / "trace.csv"
        pl = sd / "pi.log"
        if not tr.is_file():
            continue
        prov_seats: set[str] = set()
        if pl.is_file():
            for line in open(pl, encoding="utf-8", errors="replace"):
                m = re.search(r"\[(?:ROI|PROV)\].*?([NSEW])/([TX]\d+)", line)
                if m:
                    prov_seats.add(f"{m.group(1)}/{m.group(2)}")
        in_corner = False
        for r in csv.DictReader(open(tr, encoding="utf-8")):
            if r.get("pi_frame") != "True":
                continue
            tc_m = re.search(r"tc=([\d.]+)", r.get("ack") or "")
            if not tc_m:
                continue
            tc = float(tc_m.group(1))
            if tc >= 4:
                break
            if tc < 1:
                continue
            pwm = int(float(r.get("motor_pwm") or 0))
            if pwm <= _CNR_HI and float(r.get("speed_mm_s") or 0) >= 20:
                corner += 1
                in_corner = True
            elif in_corner and pwm > _CNR_HI:
                in_corner = False
        # heurística débil sin estado visible en trace: solo reporta prov en log
        if prov_seats and corner:
            wasted += 1
    print(f"\n# audit esquinas (semillas con prov en log y corner_sign): {wasted}/{len(seeds)}")


if __name__ == "__main__":
    main()