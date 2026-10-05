"""Harness corto de estacionamiento (T15b). Desde src/RASPI/cam:

  PYTHONUTF8=1 PYTHONPATH=. <venv>/python pure_pursuit/twin/tools/prepark.py <tag> [opciones] < /dev/null

  --jobs N            procesos en paralelo (default 8)
  --perts p0,p1       perturbaciones (default todas, ver twin/prepark.PERTURB)
  --dirs CW,CCW       sentidos;  --cards 1,2,…  cartas de la recta del cajón
  --fw K=V,K=V        overrides de constantes del .ino (además de los del harness; recompila)
  --par K=V,K=V       constantes PARK_AJ del .ino vía sil_param (sin recompilar), p. ej.
                      --par PARK_POSTE2_MM=80,PARK_ENDEREZA_TOL_DEG=16
  --period S          periodo de la Pi (default el del preset hw_nuevo, 0.035 = 28 fps)
  --max-time S        tope de tiempo simulado (default 45)
  --noise-seed K      semilla del ruido de sensores (T16ens; default $TWIN_NOISE_SEED o, sin
                      definir, el ruido de siempre). Mismos escenarios, otro ruido; cada
                      escenario saca su subcorriente (K, crc32(nombre)). Sin definir, los
                      100 escenarios comparten una sola calibración de sensores (Sim(0)).

Escribe runs/<tag>/<escenario>/{pi.log,fw_debug.log,trace.csv} y runs/<tag>/summary.json;
imprime una tabla por escenario (contacto, mm fuera de la caja, error de rumbo,
paralelo según reglamento, tiempo) y el total limpio (sin contacto, terminado,
<= 5 mm fuera y paralelo).
"""
from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import sys
import zlib
from pathlib import Path

sys.path.insert(0, ".")

from pure_pursuit.twin import prepark as PP  # noqa: E402

# Lo que el harness fuerza en el firmware: arranque directo a estacionar (tc=12).
HARNESS_FW = {"PARK_TEST_RECTA_COMPLETA": "true"}


def run_one(args):
    sc, out, fw_over, period, max_time, par, ignore, noise_seed = args
    from pure_pursuit.twin import sim as S
    from pure_pursuit.twin.firmware import fw as F
    d = Path(out) / sc.name
    d.mkdir(parents=True, exist_ok=True)
    dbg = open(d / "fw_debug.log", "w", encoding="utf-8")
    orig = F.FirmwareSIL.pop_debug
    buf = {"t": ""}

    def pop(self):
        s = orig(self)
        if s:
            t = self.now_us / 1e6
            lines = (buf["t"] + s).split("\n")
            buf["t"] = lines.pop()
            for ln in lines:
                dbg.write(f"{t:8.3f} {ln}\n")
        return s
    F.FirmwareSIL.pop_debug = pop
    field, params = PP.build(sc)
    params.update(par)
    # Todos los escenarios usan Sim(0, ...). Sin --noise-seed, los 100 comparten la
    # rng default_rng(0): una sola "calibración" de sensores (slip_bias, bias/scale del
    # gyro) para todo el prepark, como siempre. Con --noise-seed k, cada escenario saca
    # su propia subcorriente spawn_key=(k, crc32(nombre)): las réplicas son
    # independientes entre escenarios y la misma k da el mismo ruido en A y en B.
    nkey = None if noise_seed is None else (int(noise_seed), zlib.crc32(sc.name.encode()))
    sim = S.Sim(0, preset="hw_nuevo", log_dir=d, fw_overrides={**HARNESS_FW, **fw_over},
                pi_period_s=period, max_time_s=max_time, field=field, fw_params=params,
                ignore_collisions=ignore, noise_seed=nkey)
    try:
        r = sim.run()
    finally:
        dbg.close()
    keys = sorted({k for row in r.trace for k in row})
    with open(d / "trace.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(r.trace)
    last = r.trace[-1]
    vp = sim.params.vehicle
    pm = PP.park_metrics(field, float(last["x"]), float(last["y"]), float(last["heading"]),
                         rear_oh=vp.rear_overhang_mm, length=vp.length_mm, width=vp.width_mm,
                         wheelbase=sim.params.vehicle.wheelbase_mm)
    col = r.metrics.get("collisions") or []
    cz = (f"{col[0].get('cause')}@{col[0].get('t')}" if col and isinstance(col[0], dict)
          else (str(col[0]) if col else ""))
    motivo = ""
    for ln in open(d / "fw_debug.log", encoding="utf-8"):
        if "TERMINADO:" in ln:
            motivo = ln.split("TERMINADO:", 1)[1].strip()
    res = {"name": sc.name, "dir": sc.direction, "card": sc.card, "pert": sc.pert,
           "stop": r.stop_reason, "contacto": cz, "t": r.metrics.get("total_time_s"),
           "motivo": motivo, **pm}
    pared = [c for c in r.metrics.get("ignored_contacts", []) if c["cause"] == "pared exterior"]
    res["pared"] = f"{pared[0]['max_mm']}mm@{pared[0]['t']}" if pared else ""
    res["limpio"] = (not cz and r.stop_reason in ("race_finished", "terminado")
                     and pm["fuera_mm"] <= 5.0 and pm["paralelo"])
    if noise_seed is not None:
        res["noise_seed"] = noise_seed
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--perts", default="")
    ap.add_argument("--dirs", default="CW,CCW")
    ap.add_argument("--cards", default="")
    ap.add_argument("--fw", default="")
    ap.add_argument("--par", default="")
    ap.add_argument("--period", type=float, default=None)
    ap.add_argument("--max-time", type=float, default=45.0)
    ap.add_argument("--solo-cajon", action="store_true",
                    help="tocar la pared exterior no detiene la corrida (columna 'pared': mm metidos@t)")
    _ns = os.environ.get("TWIN_NOISE_SEED", "").strip()
    ap.add_argument("--noise-seed", type=int, default=int(_ns) if _ns else None,
                    help="semilla del ruido de sensores (default $TWIN_NOISE_SEED; sin definir = el de siempre)")
    a = ap.parse_args()
    ignore = ("pared exterior",) if a.solo_cajon else ()
    perts = [p for p in a.perts.split(",") if p] or None
    cards = tuple(int(c) for c in a.cards.split(",") if c) or PP.STALL_CARDS
    fw_over = dict(kv.split("=", 1) for kv in a.fw.split(",") if kv)
    par = {k: float(v) for k, v in (kv.split("=", 1) for kv in a.par.split(",") if kv)}
    scs = PP.all_scenarios(perts, tuple(a.dirs.split(",")), cards)
    out = Path("runs") / a.tag
    out.mkdir(parents=True, exist_ok=True)
    # Una compilación antes de abrir el pool (si no, cada proceso compila la suya).
    from pure_pursuit.twin.firmware import build as B
    from pure_pursuit.twin import sim as S
    cfg = S.resolve_preset("hw_nuevo", fw_overrides={**HARNESS_FW, **fw_over})
    B.build(overrides=cfg["fw_overrides"], defines=cfg.get("fw_defines"), source=cfg["fw_source"])
    # maxtasksperchild=1: proceso nuevo por escenario (pop_debug parcheado, config de la Pi).
    with mp.Pool(a.jobs, maxtasksperchild=1) as pool:
        rows = pool.map(run_one, [(s, str(out), fw_over, a.period, a.max_time, par, ignore, a.noise_seed)
                                  for s in scs], chunksize=1)
    (out / "summary.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False), encoding="utf-8")
    print("| escenario | stop | contacto | pared | fuera_mm | rumbo_err | ruedas_dif | t | motivo |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['name']} | {r['stop']} | {r['contacto'] or '-'} | {r['pared'] or '-'} | {r['fuera_mm']} | {r['rumbo_err_deg']} | "
              f"{r['ruedas_dif_mm']} | {r['t']} | {r['motivo']}{' OK' if r['limpio'] else ''} |")
    n = len(rows)
    print(f"limpios={sum(r['limpio'] for r in rows)}/{n} sin_contacto={sum(not r['contacto'] for r in rows)}/{n} "
          f"fuera<=5={sum(r['fuera_mm'] <= 5 for r in rows)}/{n} paralelo={sum(r['paralelo'] for r in rows)}/{n} "
          f"roce_pared={sum(bool(r['pared']) for r in rows)}/{n}")


if __name__ == "__main__":
    main()
