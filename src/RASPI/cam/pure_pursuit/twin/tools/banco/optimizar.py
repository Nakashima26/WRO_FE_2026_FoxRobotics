"""Optimizador (mu,lambda)-ES de 8 perillas de ruta del gemelo (minimiza vueltas 1-3).

Reanudable: state.json se guarda tras cada evaluacion y la generacion parcial se retoma.
Uso: python optimizar.py --help
"""
import argparse
import copy
import json
import math
import os
import shutil
import subprocess
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import numpy as np

WK = Path(__file__).resolve().parent / "wk.sh"
DEFAULT_CAM = "/Users/jesse/Documents/GitHub/FoxRobotics-wt-isla/src/RASPI/cam"
SEEDS = list(range(1, 42)) + [3170839]
MIN_FREE = 20 * 2**30
ENV = {
    **os.environ,
    "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
    "PYTHONUTF8": "1", "PYTHONPATH": ".",
    "TWIN_FW_PARAMS": "serial2_us_per_char=21.7,serial_us_per_char=21.7",
    "TWIN_PI_UART_US_PER_BYTE": "21.7",
}
# (clave en cfg["pi"], minimo, maximo, entero)
PARAMS = [
    ("ROUTE_PROFILE_A_LAT_MM_S2", 2000, 2900, False),   # agarre medido 2400 izq / 2750 der (2026-10-21)
    ("ROUTE_PROFILE_A_BRAKE_MM_S2", 2000, 3500, False),
    ("ROUTE_PROFILE_BRAKE_FF_S", 0.0, 0.15, False),
    ("ROUTE_PROFILE_LATENCY_S", 0.06, 0.20, False),
    ("ROUTE_PROFILE_SIGN_MM", 180, 320, False),
    ("ROUTE_PROFILE_SIGN_NEAR_MM", 100, 220, False),
    ("ROUTE_BLIND_PWM", 95, 120, True),
    ("ROUTE_PROFILE_MIN_PWM", 190, 235, True),
]
KEYS = [p[0] for p in PARAMS]


def decode(x):
    """x normalizado [0,1]^8 -> dict de perillas."""
    pi = {}
    for v, (k, lo, hi, is_int) in zip(np.clip(x, 0, 1), PARAMS):
        val = lo + float(v) * (hi - lo)
        pi[k] = float(round(val)) if is_int else val
    # SIGN_NEAR debe quedar 20 mm por debajo de SIGN
    pi["ROUTE_PROFILE_SIGN_NEAR_MM"] = min(pi["ROUTE_PROFILE_SIGN_NEAR_MM"], pi["ROUTE_PROFILE_SIGN_MM"] - 20)
    return pi


def encode(pi):
    return [min(1.0, max(0.0, (pi[k] - lo) / (hi - lo))) for k, lo, hi, _ in PARAMS]


def parse_out(path):
    """Registro JSON de la linea STOP de un .out, o None si no existe."""
    try:
        lines = Path(path).read_text(errors="replace").splitlines()
    except OSError:
        return None
    for line in lines:
        if line.startswith("STOP "):
            try:
                return json.loads(line[line.index("{"):])
            except ValueError:
                return None
    return None


def ok_t(rec):
    """(ok, suma de vueltas 1-3); ok requiere al menos 3 vueltas."""
    laps = (rec or {}).get("lap_times_s") or []
    if len(laps) < 3:
        return (False, None)
    # roce de pared exterior ignorado por --solo-cajon: en el carro real sí choca. Cuenta en toda la
    # carrera (también U/cajón): el usuario no quiere que toque la pared nunca.
    if (rec or {}).get("ignored_contacts"):
        return (False, None)
    return (True, float(sum(laps[:3])))


def score(base, cand):
    """Puntuacion del candidato vs base; ambos son {(nz, seed): (ok, t)}."""
    keys = [k for k in cand if k in base]
    both = [k for k in keys if base[k][0] and cand[k][0]]
    d = [cand[k][1] - base[k][1] for k in both]
    new_fail = [k for k in keys if base[k][0] and not cand[k][0]]
    fails = [k for k in cand if not cand[k][0]]
    info = dict(
        mean=float(np.mean(d)) if d else None,
        median=float(np.median(d)) if d else None,
        n=len(d), new_fail=new_fail, fails=fails,
    )
    # semilla que falla NUEVA en 2+ ruidos: rechazo duro (las que ya fallan en la base no cuentan)
    if any(n >= 2 for n in Counter(s for _, s in new_fail).values()) or not d:
        return math.inf, info
    return float(np.mean(d)) + len(new_fail), info


def sample(st, lam, rng):
    cands = []
    for _ in range(lam):
        x = np.clip(np.array(st["mean"]) + st["sigma"] * rng.standard_normal(len(KEYS)), 0, 1)
        cands.append({"id": st["next_id"], "x": x.tolist()})
        st["next_id"] += 1
    return cands


def select(st, cands, mu):
    """Recombina los mu mejores con pesos log y ajusta sigma segun el mejor de la generacion."""
    order = sorted(cands, key=lambda c: c["score"])
    w = np.array([math.log(mu + 0.5) - math.log(i) for i in range(1, mu + 1)])
    w /= w.sum()
    st["mean"] = (w @ np.array([c["x"] for c in order[:mu]])).tolist()
    best = order[0]["score"]
    if st["prev_best"] is not None:
        st["sigma"] = min(0.3, max(0.03, st["sigma"] * (1.1 if best < st["prev_best"] else 0.9)))
    st["prev_best"] = best
    st["gen"] += 1


def save_state(out, st):
    tmp = out / "state.json.tmp"
    tmp.write_text(json.dumps(st, indent=1))
    os.replace(tmp, out / "state.json")


def make_cfg(ctx, x):
    pi = decode(x)
    cfg = copy.deepcopy(ctx.cfg)
    cfg["pi"].update(pi)
    return pi, json.dumps(cfg)


def run_bench(ctx, cid, noises, cfg_json):
    free = shutil.disk_usage(ctx.cam).free
    if free < MIN_FREE:
        print(f"Disco libre {free / 2**30:.1f} GB < 20 GB: paro")
        sys.exit(1)
    tag = f"opt/e{cid}"
    jobs = []
    for nz in noises:
        (ctx.cam / "runs" / tag / f"cand_n{nz}").mkdir(parents=True, exist_ok=True)
        jobs += [(nz, s) for s in SEEDS]

    def one(job):
        nz, s = job
        subprocess.run(["bash", str(WK), str(nz), f"{tag}/cand_n{nz}", "ruta_perfil", cfg_json, str(s)],
                       cwd=ctx.cam, env=ENV)

    with ThreadPoolExecutor(ctx.a.jobs) as ex:
        list(ex.map(one, jobs))


def read_results(ctx, cid, noises):
    d = ctx.cam / "runs" / "opt" / f"e{cid}"
    return {(nz, s): ok_t(parse_out(d / f"cand_n{nz}" / f"s{s}.out")) for nz in noises for s in SEEDS}


def fmt(v):
    return "n/a" if v is None else f"{v:.3f}"


def log_eval(ctx, c, stage, gen, s, info):
    rec = {"id": c["id"], "gen": gen, "params": decode(c["x"]), "x": c["x"],
           "score": s, "info": info, "stage": stage, "time": time.time()}
    with open(ctx.out / "log.jsonl", "a") as f:
        f.write(json.dumps(rec) + "\n")


def is_promo(ctx, c):
    return c["score"] <= ctx.a.promote and not c["info"]["new_fail"]


def screen(ctx, st, c):
    pi, cfg_json = make_cfg(ctx, c["x"])
    gen = st["cur"]["gen"]
    run_bench(ctx, c["id"], ctx.screen, cfg_json)
    s, info = score(ctx.base, read_results(ctx, c["id"], ctx.screen))
    c["score"], c["info"] = s, info
    log_eval(ctx, c, "screen", gen, s, info)
    if math.isfinite(s):
        st["best"] = sorted(st["best"] + [{"id": c["id"], "gen": gen, "score": s, "params": pi}],
                            key=lambda b: b["score"])[:10]
    save_state(ctx.out, st)
    params = " ".join(f"{k.replace('ROUTE_PROFILE_', '').replace('ROUTE_', '')}={v:g}" for k, v in pi.items())
    print(f"e{c['id']} g{gen} score={fmt(s)} mean={fmt(info['mean'])} fails={len(info['fails'])} {params}",
          flush=True)


def promote(ctx, st, c):
    _, cfg_json = make_cfg(ctx, c["x"])
    gen = st["cur"]["gen"]
    run_bench(ctx, c["id"], [n for n in ctx.full if n not in ctx.screen], cfg_json)
    fs, finfo = score(ctx.base, read_results(ctx, c["id"], ctx.full))
    c["full"] = {"score": fs, "info": finfo}
    log_eval(ctx, c, "full", gen, fs, finfo)
    (ctx.out / f"cand_e{c['id']}.json").write_text(cfg_json)
    save_state(ctx.out, st)
    print(f"PROMOVIDO e{c['id']} full score {fmt(fs)} mean={fmt(finfo['mean'])} "
          f"n={finfo['n']} fails={len(finfo['fails'])}", flush=True)


def main(argv=None):
    a = parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    base_cfg = json.loads(a.base_cfg.read_text())
    screen_n = [int(n) for n in a.noises.split()]
    full_n = [int(n) for n in a.full_noises.split()]
    base = {}
    for nz in sorted(set(screen_n) | set(full_n)):
        for s in SEEDS:
            base[(nz, s)] = ok_t(parse_out(a.cam / "runs" / a.base_tag / f"cand_n{nz}" / f"s{s}.out"))
    ctx = SimpleNamespace(a=a, cam=a.cam, out=a.out, base=base, cfg=base_cfg, screen=screen_n, full=full_n)

    sp = a.out / "state.json"
    st = json.loads(sp.read_text()) if sp.exists() else {
        "gen": 0, "mean": encode({k: base_cfg["pi"][k] for k in KEYS}), "sigma": a.sigma0,
        "next_id": 1, "best": [], "prev_best": None, "cur": None,
    }
    rng = np.random.default_rng()
    while st["gen"] < a.gens:
        if st["cur"] is None:
            st["cur"] = {"gen": st["gen"], "cands": sample(st, a.lam, rng)}
            save_state(a.out, st)
        for c in st["cur"]["cands"]:
            if "score" not in c:
                screen(ctx, st, c)
            if is_promo(ctx, c):
                if "full" not in c:
                    promote(ctx, st, c)
            else:
                shutil.rmtree(a.cam / "runs" / "opt" / f"e{c['id']}", ignore_errors=True)
        select(st, st["cur"]["cands"], a.mu)
        st["cur"] = None
        save_state(a.out, st)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="ES (mu,lambda) de las perillas de ruta del gemelo")
    p.add_argument("--cam", default=DEFAULT_CAM, help="cwd de los workers")
    p.add_argument("--base-cfg", help="config json base (default <cam>/runs/cfg/al26.json)")
    p.add_argument("--base-tag", default="al26")
    p.add_argument("--out", help="estado y log (default <cam>/runs/opt)")
    p.add_argument("--noises", default="1 2 3 4 5")
    p.add_argument("--full-noises", default=" ".join(str(n) for n in range(1, 21)))
    p.add_argument("--jobs", type=int, default=12)
    p.add_argument("--lam", type=int, default=6)
    p.add_argument("--mu", type=int, default=2)
    p.add_argument("--sigma0", type=float, default=0.15)
    p.add_argument("--gens", type=int, default=1000)
    p.add_argument("--promote", type=float, default=-0.12)
    a = p.parse_args(argv)
    a.cam = Path(a.cam)
    a.base_cfg = Path(a.base_cfg) if a.base_cfg else a.cam / "runs" / "cfg" / "al26.json"
    a.out = Path(a.out) if a.out else a.cam / "runs" / "opt"
    return a


if __name__ == "__main__":
    main()
