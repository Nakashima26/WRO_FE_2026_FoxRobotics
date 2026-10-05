"""Resumen de un ensamble de ruido de sensores (T16ens). Desde src/RASPI/cam:

  python pure_pursuit/twin/tools/ens.py runs/<tag> [--all] [--power] [--delta 0.10]

El tipo sale del contenido de runs/<tag> (lo arma tools/ens.sh):
  carrera  s<seed>_n<k>/ + s<seed>_n<k>.out      (drive.py con TWIN_NOISE_SEED=k)
  prepark  n<k>/summary.json                     (prepark.py --noise-seed k)

Éxito de una corrida: carrera = criterio de summ3.py (race_finished/terminado, sin
choque, <= 5 mm fuera del cajón); prepark = "limpio" de prepark.py (además paralelo).
Por escenario: éxitos/K, tc medio, REVERSA por corrida y tipos de fin. Agregado: tasa
con IC 95% de Wilson, suma_tc media, REVERSA por esquina recorrida, holgura casco-lata
(carrera: holg_todas / holg_1a de tools/holg.py), escenarios robustos (K/K, 0/K) y
monedas, y la diferencia mínima detectable (MDD) contra otro ensamble del mismo K
(prueba de tools/ens_cmp.py). --power: potencia simulada de esa prueba para varios K.
Comparar solo ensambles del mismo host.
"""
from __future__ import annotations

import argparse
import functools
import json
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import summ3  # noqa: E402
import holg as H  # noqa: E402

Z975 = 1.959963984540054   # alfa 0.05 bilateral
Z80 = 0.8416212335729143   # potencia 80%
RACE_DIR = re.compile(r"^s(\d+)_n(\d+)$")
RACE_DIR1 = re.compile(r"^s(\d+)$")
PP_DIR = re.compile(r"^n(\d+)$")
# Una línea por giro MANIOBRA en reversa (PurePursuit.ino, "-> MANIOBRA dir=... REVERSA").
REVERSA = re.compile(r"-> MANIOBRA dir=\w+ REVERSA")


# ---------------------------------------------------------------- carga
def _cause_kind(cause: str) -> str:
    if cause.startswith("señal"):
        return "señal"
    if cause.startswith("cajón"):
        return "cajón"
    return {"pared exterior": "pared_ext", "isla": "isla", "pared": "pared"}.get(cause, cause)


def race_end(r) -> str:
    if r is None:
        return "error"
    if r["ok"]:
        return "ok"
    col = r["col"]
    if col:
        return _cause_kind(col[0].get("cause", "") if isinstance(col[0], dict) else str(col[0]))
    if r["stop"] in ("race_finished", "terminado"):
        return "fuera"          # terminó sin choque pero > 5 mm fuera del cajón
    return r["stop"]            # max_time, stuck, ...


def prepark_end(r: dict) -> str:
    if r.get("limpio"):
        return "ok"
    if r.get("contacto"):
        return _cause_kind(r["contacto"].split("@")[0])
    if r.get("stop") not in ("race_finished", "terminado"):
        return r.get("stop") or "error"
    if r.get("fuera_mm", 0) > 5.0:
        return "fuera"
    return "no_paralelo"


def _seed_key(s: str):
    return (0, int(s)) if s.isdigit() else (1, s)


def load(d: Path):
    """-> (tipo, {escenario: [corrida, ...]}); corrida = dict k, ok, end, tc, rev."""
    d = Path(d)
    runs: dict[str, list[dict]] = defaultdict(list)
    race = [p for p in d.iterdir() if p.is_dir() and RACE_DIR.match(p.name)]
    if not race:   # corrida suelta de all.sh (s<seed>/ + s<seed>.out): una réplica, k=0
        race = [p for p in d.iterdir() if p.is_dir() and RACE_DIR1.match(p.name)
                and p.with_name(p.name + ".out").is_file()]
    if race:
        for p in race:
            mm = RACE_DIR.match(p.name)
            seed, k = mm.groups() if mm else (RACE_DIR1.match(p.name).group(1), "0")
            r = summ3.run_row(str(p))
            try:
                with open(p / "fw_debug.log", encoding="utf-8") as f:
                    rev = sum(1 for ln in f if REVERSA.search(ln))
            except OSError:
                rev = 0
            hh = H.run_holg(str(p), r) if r else None
            runs[seed].append({
                "k": int(k), "ok": bool(r and r["ok"]), "end": race_end(r),
                "tc": int(r["m"].get("turns_completed") or 0) if r else 0,
                "rev": rev, "dp": r["dp"] if r else "?",
                "h_all": hh[0] if hh else None, "h_1a": hh[1] if hh else None,
            })
        kind = "carrera"
    else:
        pp = [(int(m.group(1)), p / "summary.json") for p in d.iterdir()
              if (m := PP_DIR.match(p.name)) and (p / "summary.json").is_file()]
        if not pp and (d / "summary.json").is_file():   # prepark.py suelto: k=0
            pp = [(0, d / "summary.json")]
        for k, f in pp:
            for r in json.loads(f.read_text(encoding="utf-8")):
                runs[r["name"]].append({"k": k, "ok": bool(r.get("limpio")),
                                        "end": prepark_end(r), "tc": None, "rev": None, "dp": r.get("dir", ""),
                                        "h_all": None, "h_1a": None})
        kind = "prepark"
    if not runs:
        raise SystemExit(f"{d}: ni s<seed>_n<k>/ (carrera) ni n<k>/summary.json (prepark)")
    out = {s: sorted(v, key=lambda x: x["k"]) for s, v in sorted(runs.items(), key=lambda kv: _seed_key(kv[0]))}
    return kind, out


# ---------------------------------------------------------------- estadística
def wilson(s: int, n: int, z: float = Z975) -> tuple[float, float]:
    if n == 0:
        return 0.0, 1.0
    p = s / n
    den = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def var_unbiased(s: int, n: int) -> float:
    """Varianza de Bernoulli de un escenario, insesgada: p(1-p)·n/(n-1)."""
    if n < 2:
        return 0.25
    p = s / n
    return p * (1 - p) * n / (n - 1)


def mdd(var_i, nA_i, nB_i) -> float:
    """Diferencia mínima detectable (tasa media sobre escenarios, alfa 0.05 bilateral,
    potencia 80%) con la varianza por escenario var_i y nA_i/nB_i corridas."""
    N = len(var_i)
    v = sum(vi * (1.0 / a + 1.0 / b) for vi, a, b in zip(var_i, nA_i, nB_i))
    return (Z975 + Z80) * math.sqrt(v) / N


def k_needed(var_i, delta: float) -> float:
    """K (réplicas por escenario y por lado) para detectar delta de tasa media."""
    N = len(var_i)
    return 2.0 * (Z975 + Z80) ** 2 * sum(var_i) / (N * N * delta * delta)


@functools.lru_cache(maxsize=None)
def _hyper(T: int, n: int, nB: int) -> tuple[int, np.ndarray]:
    """P(b éxitos caen en B | T éxitos entre n corridas, nB de ellas en B)."""
    lo, hi = max(0, T - (n - nB)), min(T, nB)
    tot = math.comb(n, T)
    return lo, np.array([math.comb(nB, b) * math.comb(n - nB, T - b) / tot for b in range(lo, hi + 1)])


def strat_perm_test(a, nA, b, nB, mc: int = 200_000, seed: int = 0) -> float:
    """Permutación pareada por escenario: H0 = en cada escenario A y B son la misma
    moneda (la diferencia es ruido); se reparten al azar las etiquetas A/B entre las
    corridas de cada escenario, condicionado a sus éxitos totales. Estadístico
    Σ_i (b_i/nB_i - a_i/nA_i). Exacta (convolución de hipergeométricas) si todos los
    escenarios tienen el mismo nA y nB; si no, Monte Carlo. Devuelve p bilateral."""
    a, nA, b, nB = (np.asarray(x, dtype=int) for x in (a, nA, b, nB))
    T, n = a + b, nA + nB
    obs = float(np.sum(b / nB - a / nA))
    if len(set(nA.tolist())) == 1 and len(set(nB.tolist())) == 1:
        dist, off = np.array([1.0]), 0
        for Ti, ni, nBi in zip(T.tolist(), n.tolist(), nB.tolist()):
            lo, pmf = _hyper(Ti, ni, nBi)
            dist, off = np.convolve(dist, pmf), off + lo
        xs = off + np.arange(len(dist))                     # Σ b_i posibles
        nA0, nB0 = int(nA[0]), int(nB[0])
        stat = xs / nB0 - (T.sum() - xs) / nA0
        return float(min(1.0, dist[np.abs(stat) >= abs(obs) - 1e-9].sum()))
    rng = np.random.default_rng(seed)
    bs = rng.hypergeometric(T, n - T, nB, size=(mc, len(T)))
    stat = np.sum(bs / nB - (T - bs) / nA, axis=1)
    return float((1 + np.sum(np.abs(stat) >= abs(obs) - 1e-9)) / (mc + 1))


def sign_test(d) -> tuple[int, int, float]:
    pos = sum(1 for x in d if x > 0)
    neg = sum(1 for x in d if x < 0)
    n = pos + neg
    if n == 0:
        return 0, 0, 1.0
    k = min(pos, neg)
    return pos, neg, _sign_p(n, k)


@functools.lru_cache(maxsize=None)
def _sign_p(n: int, k: int) -> float:
    """p bilateral exacto de una binomial(n, 1/2) con min(pos, neg) = k."""
    if n == 0:
        return 1.0
    return min(1.0, 2.0 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def pair_runs(A: dict, B: dict):
    """Pares (escenario, réplica k) presentes en A y en B -> (keys, [(a, b) 0/1], sin_par).
    Diseño pareado: misma k = mismo TWIN_NOISE_SEED en ambos lados."""
    pairs, lost = [], 0
    for s in A:
        if s not in B:
            continue
        ka = {r["k"]: r["ok"] for r in A[s]}
        kb = {r["k"]: r["ok"] for r in B[s]}
        common = sorted(set(ka) & set(kb))
        lost += len(set(ka) ^ set(kb))
        pairs += [(s, k, int(ka[k]), int(kb[k])) for k in common]
    return pairs, lost


def paired_test(pairs) -> tuple[int, int, float]:
    """Prueba principal de ens_cmp: permutación de las etiquetas A/B dentro de cada par
    (escenario, k) = McNemar exacta sobre los pares discordantes. Estadístico Σ (b - a).
    -> (pares donde gana B, pares donde gana A, p bilateral)."""
    bw = sum(1 for *_, a, b in pairs if b > a)
    aw = sum(1 for *_, a, b in pairs if a > b)
    return bw, aw, _sign_p(bw + aw, min(bw, aw))


def _waterfill(p: np.ndarray, idx, U: float) -> np.ndarray:
    p = p.copy()
    left = U
    for _ in range(200):
        open_ = [i for i in idx if p[i] < 1.0 - 1e-12]
        if not open_ or left <= 1e-12:
            break
        inc = left / len(open_)
        for i in open_:
            add = min(inc, 1.0 - p[i])
            p[i] += add
            left -= add
    return p


def shifted(p: np.ndarray, model: str, delta: float) -> np.ndarray:
    """Tasas del lado B con una mejora media de `delta` (escenarios-equivalentes = delta·N).
    monedas: repartida en los escenarios con 0<p<1 (el caso más difícil de ver);
    uniforme: en todos los que no están en 1; robustas: round(delta·N) escenarios en 0 pasan a 1."""
    N = len(p)
    U = delta * N
    if model == "robustas":
        q = p.copy()
        zeros = [i for i in range(N) if p[i] <= 1e-12]
        for i in zeros[: int(round(U))]:
            q[i] = 1.0
        return q
    if model == "monedas":
        idx = [i for i in range(N) if 1e-12 < p[i] < 1 - 1e-12]
        if idx:
            return _waterfill(p, idx, U)
    return _waterfill(p, range(N), U)


def power(p: np.ndarray, K: int, model: str, delta: float, sims: int, rng, test: str = "pareada") -> float:
    """Fracción de simulaciones en que ens_cmp diría 'B MEJOR' (p<0.05 y B arriba) con
    corridas independientes Bernoulli(p_i) en A y Bernoulli(q_i) en B, K por escenario.
    test='pareada' (la principal, McNemar por (escenario, k)) o 'estratificada'."""
    q = shifted(p, model, delta)
    N = len(p)
    ks = [K] * N
    hits = 0
    for _ in range(sims):
        if test == "pareada":
            a = rng.random((N, K)) < p[:, None]
            b = rng.random((N, K)) < q[:, None]
            bw = int(np.sum(b & ~a))
            aw = int(np.sum(a & ~b))
            if bw > aw and _sign_p(bw + aw, aw) < 0.05:
                hits += 1
        else:
            a = rng.binomial(K, p)
            b = rng.binomial(K, q)
            if b.sum() > a.sum() and strat_perm_test(a, ks, b, ks) < 0.05:
                hits += 1
    return hits / sims


# ---------------------------------------------------------------- reporte
def scen_stats(runs: dict[str, list[dict]]):
    rows = []
    for s, rr in runs.items():
        n = len(rr)
        ok = sum(r["ok"] for r in rr)
        rows.append({"scen": s, "n": n, "ok": ok, "dp": rr[0]["dp"],
                     "tc": [r["tc"] for r in rr], "rev": [r["rev"] for r in rr],
                     "ends": Counter(r["end"] for r in rr), "ks": [r["k"] for r in rr]})
    return rows


def _fines(c: Counter) -> str:
    return " ".join(f"{k}×{v}" for k, v in sorted(c.items(), key=lambda kv: (-kv[1], kv[0])))


def report(d: Path, show_all: bool = False, do_power: bool = False, delta: float = 0.10, sims: int = 1000):
    kind, runs = load(d)
    rows = scen_stats(runs)
    N = len(rows)
    Ks = sorted({r["n"] for r in rows})
    K = Ks[-1]
    wall = Path(d) / "wall.txt"
    print(f"# {d} ({kind}): {N} escenarios, K={'/'.join(map(str, Ks))}"
          + (f", {wall.read_text().strip()}" if wall.is_file() else ""))
    race = kind == "carrera"
    if race:
        print("| seed | dir/park | éxito | tc medio | REVERSA/corrida | fines |")
        print("|---|---|---|---|---|---|")
    else:
        print("| escenario | éxito | fines |")
        print("|---|---|---|")
    for r in rows:
        if not race and not show_all and r["ok"] == r["n"]:
            continue
        if race:
            print(f"| {r['scen']} | {r['dp']} | {r['ok']}/{r['n']} | {np.mean(r['tc']):.1f} | "
                  f"{np.mean(r['rev']):.1f} | {_fines(r['ends'])} |")
        else:
            print(f"| {r['scen']} | {r['ok']}/{r['n']} | {_fines(r['ends'])} |")
    if not race and not show_all:
        print(f"(omitidos {sum(r['ok'] == r['n'] for r in rows)} escenarios con éxito {K}/{K}; --all los muestra)")

    S = sum(r["ok"] for r in rows)
    n_tot = sum(r["n"] for r in rows)
    lo, hi = wilson(S, n_tot)
    print()
    print(f"tasa de éxito = {S}/{n_tot} = {100 * S / n_tot:.1f}%  IC95 Wilson [{100 * lo:.1f}, {100 * hi:.1f}]"
          f"  (= {N * S / n_tot:.2f} escenarios-equivalentes de {N})")
    by_k: dict[int, list[int]] = defaultdict(list)
    for r in rows:
        for k, rr in zip(r["ks"], runs[r["scen"]]):
            by_k[k].append(rr["ok"])
    print("éxitos por réplica (lo que daría una sola pasada de N corridas): "
          + " ".join(f"n{k}={sum(v)}/{len(v)}" for k, v in sorted(by_k.items())))
    if race:
        tc_k = defaultdict(int)
        for r in rows:
            for k, t in zip(r["ks"], r["tc"]):
                tc_k[k] += t
        st = list(tc_k.values())
        rev = sum(sum(r["rev"]) for r in rows)
        tcs = sum(sum(r["tc"]) for r in rows)
        print(f"suma_tc media = {np.mean(st):.1f} (sd entre réplicas {np.std(st, ddof=1) if len(st) > 1 else 0:.1f}, "
              f"rango {min(st)}-{max(st)}; por réplica: {' '.join(f'n{k}={v}' for k, v in sorted(tc_k.items()))})")
        print(f"REVERSA por esquina recorrida = {rev}/{tcs} = {rev / tcs if tcs else 0:.3f}")
        hall, h1a = holg_lists(runs)
        if hall is not None:
            print("holgura casco-lata (mm, pasadas hacia adelante tc<12; lado malo = -1):")
            print("  " + H.fmt_dist("holg_todas", hall))
            print("  " + H.fmt_dist("holg_1a", h1a))
    fines = Counter()
    for r in rows:
        fines.update(r["ends"])
    print(f"fines (todas las corridas): {_fines(fines)}")
    rob_ok = [r["scen"] for r in rows if r["ok"] == r["n"]]
    rob_ko = [r["scen"] for r in rows if r["ok"] == 0]
    coins = [f"{r['scen']}({r['ok']}/{r['n']})" for r in rows if 0 < r["ok"] < r["n"]]
    print(f"robustos éxito ({len(rob_ok)}): {' '.join(rob_ok)}")
    print(f"robustos falla ({len(rob_ko)}): {' '.join(rob_ko) if race or len(rob_ko) < 30 else '...'}")
    print(f"monedas ({len(coins)}): {' '.join(coins)}")

    var_i = [var_unbiased(r["ok"], r["n"]) for r in rows]
    var_j = [((r["ok"] + 0.5) / (r["n"] + 1)) * (1 - (r["ok"] + 0.5) / (r["n"] + 1)) for r in rows]
    ns = [r["n"] for r in rows]
    m_pl = mdd(var_i, ns, ns)
    m_j = mdd(var_j, ns, ns)
    print(f"ruido: Σ var por escenario = {sum(var_i):.2f} (insesgada; Jeffreys {sum(var_j):.2f}, cuenta "
          f"monedas que con K={K} salieron 0/K o K/K)")
    print(f"MDD contra otro ensamble de K={K} (alfa 0.05, potencia 80%): ±{100 * m_pl:.1f} pts "
          f"= ±{N * m_pl:.2f} escenarios-equivalentes (Jeffreys: ±{100 * m_j:.1f} pts = ±{N * m_j:.2f})")
    print(f"K necesario para ±{100 * delta:.0f} pts (= {N * delta:.1f} escenarios-equiv.): "
          f"{k_needed(var_i, delta):.1f} (Jeffreys {k_needed(var_j, delta):.1f}) [aprox. normal]")
    if do_power:
        rng = np.random.default_rng(12345)
        p_pl = np.array([r["ok"] / r["n"] for r in rows])
        p_j = np.array([(r["ok"] + 0.5) / (r["n"] + 1) for r in rows])
        Klist = [3, 5, 8, 10, 15, 20]
        print()
        print(f"potencia simulada de ens_cmp (prueba pareada por (escenario, k), p<0.05, B mejor) para "
              f"+{100 * delta:.0f} pts, {sims} simulaciones por celda, corridas independientes:")
        print("| verdad | modelo de mejora | prueba | " + " | ".join(f"K={k}" for k in Klist) + " |")
        print("|---|---|---|" + "---|" * len(Klist))
        for tname, p in (("tasas medidas", p_pl), ("Jeffreys", p_j)):
            for model in ("monedas", "uniforme", "robustas"):
                q = shifted(p, model, delta)
                eff = 100 * (q.sum() - p.sum()) / N
                tests = ("pareada", "estratificada") if (tname, model) == ("tasas medidas", "monedas") else ("pareada",)
                for test in tests:
                    cells = [f"{power(p, k, model, delta, sims, rng, test):.2f}" for k in Klist]
                    print(f"| {tname} | {model} (+{eff:.1f} pts) | {test} | " + " | ".join(cells) + " |")
        print("falsa alarma 'B mejor' con B = A (mismas tasas; nominal <= 0.025): "
              + " ".join(f"K={k}:{power(p_pl, k, 'monedas', 0.0, sims, rng):.3f}" for k in (3, 5, 10)))
        if wall.is_file():
            try:
                w_s = float(wall.read_text().split()[1])
                print(f"costo de un ensamble (mismos jobs que este, escala lineal con K; este: K={K}, "
                      f"{w_s / 60:.1f} min): " + " ".join(f"K={k}:{w_s * k / K / 60:.0f} min" for k in Klist))
            except (IndexError, ValueError):
                pass
    return kind, runs


def holg_lists(runs: dict[str, list[dict]]):
    """(holg_todas, holg_1a) de todas las corridas; None si ninguna trae trace/field."""
    hall: list[float] = []
    h1a: list[float] = []
    have = False
    for rr in runs.values():
        for r in rr:
            if r.get("h_all") is not None:
                have = True
                hall += r["h_all"]
                h1a += r["h_1a"]
    return (hall, h1a) if have else (None, None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--all", action="store_true", help="prepark: mostrar también los escenarios K/K")
    ap.add_argument("--power", action="store_true", help="potencia simulada de ens_cmp para varios K")
    ap.add_argument("--delta", type=float, default=0.10, help="mejora a detectar (fracción de tasa)")
    ap.add_argument("--sims", type=int, default=1000)
    a = ap.parse_args()
    report(Path(a.dir), a.all, a.power, a.delta, a.sims)


if __name__ == "__main__":
    main()
