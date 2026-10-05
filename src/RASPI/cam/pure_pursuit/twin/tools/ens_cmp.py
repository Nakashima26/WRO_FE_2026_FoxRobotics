"""Comparación PAREADA de dos ensambles de ruido (T16ens). Desde src/RASPI/cam:

  python pure_pursuit/twin/tools/ens_cmp.py runs/<A> runs/<B>      (A = base, B = cambio)
  python pure_pursuit/twin/tools/ens_cmp.py runs/<A> --split       (falsa alarma: réplicas
                                                                    de A contra réplicas de A)

Mismo tipo en ambos (carrera o prepark, ver ens.py), mismo host y las mismas réplicas k
(TWIN_NOISE_SEED / --noise-seed 1..K) en ambos lados: la unidad es el par (escenario, k).
Prueba principal: permutación de las etiquetas A/B dentro de cada par = McNemar exacta
sobre los pares discordantes (H0: B es A más ruido). Secundarias: permutación por
escenario condicionada a sus éxitos (no usa el pareo por k) y prueba de signos sobre los
escenarios que cambian de tasa. Δ tasa con IC95 y MDD (alfa 0.05, potencia 80%) del K.
Carrera: además suma_tc media (permutación por escenario), REVERSA por esquina y las
distribuciones de holgura casco-lata (holg_todas / holg_1a, descriptivas).
VEREDICTO: MEJOR / PEOR (p<0.05) / NO DISTINGUIBLE; con una sola réplica por escenario
(K=1, p. ej. 21 seeds sueltas) siempre NO CONCLUYENTE.
"""
from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ens as E  # noqa: E402
import holg as H  # noqa: E402


def perm_mean_test(xa: list[list[float]], xb: list[list[float]], mc: int = 20_000, seed: int = 0):
    """Permutación por escenario de una métrica numérica por corrida (p. ej. tc).
    Estadístico Σ_i (media_B_i - media_A_i). Devuelve (estadístico, p bilateral)."""
    rng = np.random.default_rng(seed)
    obs = sum(np.mean(b) - np.mean(a) for a, b in zip(xa, xb))
    tot = np.zeros(mc)
    for a, b in zip(xa, xb):
        pool = np.asarray(list(a) + list(b), dtype=float)
        na = len(a)
        perm = rng.permuted(np.tile(pool, (mc, 1)), axis=1)
        tot += perm[:, na:].mean(axis=1) - perm[:, :na].mean(axis=1)
    p = (1 + np.sum(np.abs(tot) >= abs(obs) - 1e-9)) / (mc + 1)
    return float(obs), float(p)


def stats(A: dict, B: dict) -> dict:
    """Números de la comparación sobre los pares (escenario, k) comunes."""
    pairs, lost = E.pair_runs(A, B)
    keys = [s for s in A if s in B and any(p[0] == s for p in pairs)]
    common = {s: {p[1] for p in pairs if p[0] == s} for s in keys}
    A2 = {s: [r for r in A[s] if r["k"] in common[s]] for s in keys}
    B2 = {s: [r for r in B[s] if r["k"] in common[s]] for s in keys}
    N = len(keys)
    if N == 0:
        raise SystemExit("A y B no tienen ningún par (escenario, k) en común")
    a = [sum(r["ok"] for r in A2[s]) for s in keys]
    b = [sum(r["ok"] for r in B2[s]) for s in keys]
    nA = [len(A2[s]) for s in keys]
    nB = [len(B2[s]) for s in keys]
    d = [y / n2 - x / n1 for x, y, n1, n2 in zip(a, b, nA, nB)]
    var_pool = [E.var_unbiased(x + y, n1 + n2) for x, y, n1, n2 in zip(a, b, nA, nB)]
    se = float(np.sqrt(sum(v * (1.0 / n1 + 1.0 / n2) for v, n1, n2 in zip(var_pool, nA, nB)))) / N
    bw, aw, p_pair = E.paired_test(pairs)
    return {"keys": keys, "A": A2, "B": B2, "N": N, "a": a, "b": b, "nA": nA, "nB": nB, "d": d,
            "delta": float(np.mean(d)), "se": se, "mdd": E.mdd(var_pool, nA, nB),
            "pairs": pairs, "lost": lost, "bw": bw, "aw": aw, "p_pair": p_pair,
            "p_strat": E.strat_perm_test(a, nA, b, nB), "sign": E.sign_test(d),
            "kmin": min(nA + nB) if N else 0}


def verdict(st: dict) -> str:
    m, N = st["mdd"], st["N"]
    if st["kmin"] < 2:
        return ("NO CONCLUYENTE: una sola réplica por escenario (K=1) no separa un cambio del caos "
                "del twin; correr el ensamble con K>=5")
    if st["p_pair"] < 0.05:
        return f"{'MEJOR' if st['bw'] > st['aw'] else 'PEOR'} que el ruido (p = {st['p_pair']:.4f} < 0.05)"
    return (f"NO DISTINGUIBLE del ruido (p = {st['p_pair']:.4f}); con este K solo se ve una "
            f"diferencia de ±{100 * m:.1f} pts (±{N * m:.2f} escenarios-equiv.)")


def _holg_line(name: str, A: dict, B: dict, key: str) -> None:
    va = [x for rr in A.values() for r in rr if r.get(key) is not None for x in r[key]]
    vb = [x for rr in B.values() for r in rr if r.get(key) is not None for x in r[key]]
    print(f"  A {H.fmt_dist(name, va)}")
    print(f"  B {H.fmt_dist(name, vb)}")


def compare(kind: str, A: dict, B: dict, name_a: str, name_b: str) -> str:
    st = stats(A, B)
    keys, N = st["keys"], st["N"]
    miss = sorted(set(A) ^ set(B))
    if miss:
        print(f"AVISO: escenarios sin par (se ignoran): {' '.join(miss[:20])}{' ...' if len(miss) > 20 else ''}")
    if st["lost"]:
        print(f"AVISO: {st['lost']} corridas sin su réplica k en el otro lado (se ignoran; "
              f"usar las mismas k 1..K en A y B)")
    print(f"# A={name_a} (K={'/'.join(map(str, sorted(set(st['nA']))))})  B={name_b} "
          f"(K={'/'.join(map(str, sorted(set(st['nB']))))})  {kind}, {N} escenarios, "
          f"{len(st['pairs'])} pares (escenario, k)")
    print("| escenario | A | B | Δ tasa | fines A | fines B |")
    print("|---|---|---|---|---|---|")
    for s, x, y, n1, n2, dd in zip(keys, st["a"], st["b"], st["nA"], st["nB"], st["d"]):
        if abs(dd) < 1e-12:
            continue
        fa = E._fines(E.Counter(r["end"] for r in st["A"][s]))
        fb = E._fines(E.Counter(r["end"] for r in st["B"][s]))
        print(f"| {s} | {x}/{n1} | {y}/{n2} | {100 * dd:+.0f} | {fa} | {fb} |")
    same = sum(1 for dd in st["d"] if abs(dd) < 1e-12)
    print(f"({same} escenarios con la misma tasa en A y B no se listan)")

    SA, SB, TA, TB = sum(st["a"]), sum(st["b"]), sum(st["nA"]), sum(st["nB"])
    la, ha = E.wilson(SA, TA)
    lb, hb = E.wilson(SB, TB)
    delta, se, m = st["delta"], st["se"], st["mdd"]
    pos, neg, p_sign = st["sign"]
    print()
    print(f"A: {SA}/{TA} = {100 * SA / TA:.1f}% [{100 * la:.1f}, {100 * ha:.1f}]   "
          f"B: {SB}/{TB} = {100 * SB / TB:.1f}% [{100 * lb:.1f}, {100 * hb:.1f}]  (IC95 Wilson)")
    print(f"Δ tasa (B - A, media por escenario) = {100 * delta:+.1f} pts, IC95 "
          f"[{100 * (delta - E.Z975 * se):+.1f}, {100 * (delta + E.Z975 * se):+.1f}] "
          f"= {N * delta:+.2f} escenarios-equivalentes")
    print(f"prueba pareada por (escenario, k) (McNemar exacta, principal): B gana {st['bw']} pares, "
          f"A gana {st['aw']}, p = {st['p_pair']:.4f}")
    print(f"permutación por escenario (exacta, sin pareo por k): p = {st['p_strat']:.4f}")
    print(f"prueba de signos por escenario: {pos} suben, {neg} bajan, p = {p_sign:.4f}")
    print(f"MDD con este K (alfa 0.05, potencia 80%, varianza combinada): ±{100 * m:.1f} pts "
          f"= ±{N * m:.2f} escenarios-equivalentes")
    if kind == "carrera":
        tca = [[r["tc"] for r in st["A"][s]] for s in keys]
        tcb = [[r["tc"] for r in st["B"][s]] for s in keys]
        dtc, p_tc = perm_mean_test(tca, tcb)
        sa = sum(np.mean(x) for x in tca)
        sb = sum(np.mean(x) for x in tcb)
        print(f"suma_tc media: A {sa:.1f}  B {sb:.1f}  Δ {dtc:+.1f}  (permutación por escenario, p = {p_tc:.4f})")
        ra = sum(sum(r["rev"] for r in st["A"][s]) for s in keys)
        rb = sum(sum(r["rev"] for r in st["B"][s]) for s in keys)
        ca = sum(sum(x) for x in tca)
        cb = sum(sum(x) for x in tcb)
        print(f"REVERSA por esquina recorrida: A {ra}/{ca} = {ra / ca if ca else 0:.3f}   "
              f"B {rb}/{cb} = {rb / cb if cb else 0:.3f}")
        if any(r.get("h_all") is not None for rr in st["A"].values() for r in rr):
            print("holgura casco-lata (mm, pasadas hacia adelante tc<12; lado malo = -1; descriptiva):")
            _holg_line("holg_todas", st["A"], st["B"], "h_all")
            _holg_line("holg_1a", st["A"], st["B"], "h_1a")
    v = verdict(st)
    print(f"VEREDICTO: B {v}")
    return v


def split_alarm(kind: str, A: dict, name: str) -> None:
    """Falsa alarma con datos reales: todas las particiones de las réplicas de A en dos
    grupos disjuntos de m = K//2 (pareados en orden), comparadas como si fueran A y B.
    Las dos mitades son el MISMO código: toda salida MEJOR/PEOR es falsa alarma."""
    ks = sorted({r["k"] for rr in A.values() for r in rr})
    m = len(ks) // 2
    if m < 1:
        raise SystemExit("--split necesita K >= 2")
    seen, res = set(), []
    for ga in itertools.combinations(ks, m):
        rest = [k for k in ks if k not in ga]
        for gb in itertools.combinations(rest, m):
            key = frozenset((ga, gb))
            if key in seen:
                continue
            seen.add(key)
            ren_a = {k: j for j, k in enumerate(ga)}
            ren_b = {k: j for j, k in enumerate(gb)}
            XA = {s: [dict(r, k=ren_a[r["k"]]) for r in rr if r["k"] in ren_a] for s, rr in A.items()}
            XB = {s: [dict(r, k=ren_b[r["k"]]) for r in rr if r["k"] in ren_b] for s, rr in A.items()}
            st = stats(XA, XB)
            res.append((ga, gb, st))
    n = len(res)
    fa_pair = sum(st["p_pair"] < 0.05 for *_, st in res)
    fa_strat = sum(st["p_strat"] < 0.05 for *_, st in res)
    fa_verd = sum(verdict(st).startswith(("MEJOR", "PEOR")) for *_, st in res)
    print(f"# --split {name} ({kind}): K={len(ks)}, {n} particiones en dos mitades de {m} réplicas")
    print("| mitad A | mitad B | Δ pts | B gana | A gana | p pareada | p estratificada |")
    print("|---|---|---|---|---|---|---|")
    for ga, gb, st in res:
        print(f"| n{','.join(map(str, ga))} | n{','.join(map(str, gb))} | {100 * st['delta']:+.1f} | "
              f"{st['bw']} | {st['aw']} | {st['p_pair']:.4f} | {st['p_strat']:.4f} |")
    print(f"falsa alarma (p<0.05): pareada {fa_pair}/{n}, estratificada {fa_strat}/{n}; "
          f"veredicto MEJOR/PEOR {fa_verd}/{n} (nominal 0.05; las particiones comparten réplicas, "
          f"no son independientes){' [m=1: el veredicto es NO CONCLUYENTE por regla]' if m < 2 else ''}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("A")
    ap.add_argument("B", nargs="?")
    ap.add_argument("--split", action="store_true", help="réplicas de A contra réplicas de A (sin B)")
    a = ap.parse_args()
    kind, A = E.load(Path(a.A))
    if a.split:
        split_alarm(kind, A, a.A)
        return
    if not a.B:
        ap.error("falta B (o --split)")
    kind_b, B = E.load(Path(a.B))
    if kind_b != kind:
        raise SystemExit(f"tipos distintos: {kind} vs {kind_b}")
    compare(kind, A, B, a.A, a.B)


if __name__ == "__main__":
    main()
