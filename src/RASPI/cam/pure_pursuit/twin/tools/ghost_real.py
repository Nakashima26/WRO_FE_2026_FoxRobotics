"""Ghost real vs gemelo: los ACK de la ESP (misma firmware, mismo marco) alineados por tramo (tc + fracción de od).
uso: ghost_real.py orillasNNN_tele.jsonl "<dir_banco>/n1/s*/" [-o hoja.png] [--tc 7.5:9]
Correr el gemelo con TWIN_LAYOUT = el acomodo de esa corrida. Dibuja px/py, servo, vel, tL/tR/tF vs od
(real en rojo, semillas del gemelo en azul) e imprime por tc el tiempo de llegada y dónde se separan."""
import csv, glob, json, re, statistics as st, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

KEYS = ("od", "px", "py", "srv", "vel", "tL", "tR", "tF", "dF", "tc", "yaw")


def real(path):
    rows, t0 = [], None
    for l in open(path, errors="ignore"):
        try:
            s = json.loads(l).get("s") or {}
        except ValueError:
            continue
        a = s.get("ack")
        if not s.get("armed") or not isinstance(a, dict) or a.get("od") is None:
            continue
        t0 = s["t"] if t0 is None else t0
        rows.append({"t": s["t"] - t0, **{k: float(a[k]) for k in KEYS if a.get(k) is not None}})
    return rows


def twin(sd):
    rows, t0, prev = [], None, None
    for r in csv.DictReader(open(f"{sd}/trace.csv", encoding="utf-8")):
        if r["ack"] == prev:   # el trace va a 1 ms; el ACK cambia ~30 veces/s como en el real
            continue
        prev = r["ack"]
        a = dict(m.groups() for m in re.finditer(r"(\w+)=(-?[\d.]+)(?=,|$)", r["ack"] or ""))
        if "od" not in a or a.get("est") == "W":
            continue
        t = float(r["t"])
        t0 = t if t0 is None else t0
        rows.append({"t": t - t0, **{k: float(a[k]) for k in KEYS if k in a},
                     "x": float(r["x"]), "y": float(r["y"])})
    return rows


def tc_axis(rows):
    """x = tc + fracción del od recorrido en ese tramo (real y gemelo no miden igual el od); vel = dod/dt."""
    seg = {}
    for r in rows:
        seg.setdefault(int(r.get("tc", 0)), []).append(r["od"])
    for i, r in enumerate(rows):
        tc = int(r.get("tc", 0)); a, b = seg[tc][0], seg[tc][-1]
        r["x_tc"] = tc + ((r["od"] - a) / (b - a) if b > a else 0.0)
        j = max(0, i - 5)
        r["v"] = (r["od"] - rows[j]["od"]) / (r["t"] - rows[j]["t"]) if r["t"] > rows[j]["t"] else 0.0
    return rows


def splits(rows):
    out, prev = {}, 0
    for r in rows:
        tc = int(r.get("tc", prev))
        if tc > prev and tc not in out:
            out[tc] = r["t"]
        prev = tc
    return out


def main():
    args = [a for a in sys.argv[1:]]
    png = args[args.index("-o") + 1] if "-o" in args else "ghost.png"
    zoom = tuple(map(float, args[args.index("--tc") + 1].split(":"))) if "--tc" in args else None
    args = [a for i, a in enumerate(args) if a not in ("-o", "--tc") and (i == 0 or args[i - 1] not in ("-o", "--tc"))]
    R = tc_axis(real(args[0]))
    T = {sd.rstrip("/").split("/")[-1]: twin(sd) for sd in sorted(glob.glob(args[1])) if glob.glob(f"{sd}/trace.csv")}
    T = {k: tc_axis(v) for k, v in T.items() if v}
    fig, ax = plt.subplots(6, 1, figsize=(16, 22), gridspec_kw={"height_ratios": [3, 1, 1, 1, 1, 1]})
    for rows, kw in [(v, dict(color="tab:blue", lw=0.5, alpha=0.35)) for v in T.values()] + [(R, dict(color="red", lw=1.6))]:
        od = [r["x_tc"] for r in rows]
        ax[0].plot([r.get("px", 0) for r in rows], [r.get("py", 0) for r in rows], **kw)
        for i, k in enumerate(("srv", "v", "tL", "tR", "tF")):
            ax[i + 1].plot(od, [r.get(k, float("nan")) if r.get(k, -1) >= 0 else float("nan") for r in rows], **kw)
    for a in ax[1:]:
        a.set_xticks(range(0, 14)); a.grid(axis="x", color="0.7", lw=0.6)
        if zoom:
            a.set_xlim(*zoom); a.minorticks_on()
    ax[0].set_aspect("equal"); ax[0].set_title(f"pose ESP (px,py): real rojo vs gemelo azul ({len(T)} semillas)")
    for a, k in zip(ax[1:], ("servo", "vel mm/s", "tL mm", "tR mm", "tF mm")):
        a.set_ylabel(k)
    ax[-1].set_xlabel("tc + fracción del tramo")
    fig.tight_layout(); fig.savefig(png, dpi=80)
    sr, stw = splits(R), [splits(v) for v in T.values()]
    print(f"{'tc':>3} {'real s':>7} {'gemelo med':>10} {'Δ':>6}  {'srv real':>8} {'srv gem':>8}  llegan")
    for tc in range(1, 13):
        tw = [s[tc] for s in stw if tc in s]
        seg = lambda rows: [r["srv"] for r in rows if int(r.get("tc", -1)) == tc - 1 and "srv" in r]
        sv = [st.mean(seg(v)) for v in T.values() if seg(v)]
        rv = seg(R)
        print(f"{tc:>3} {sr.get(tc, float('nan')):7.2f} {st.median(tw) if tw else float('nan'):10.2f} "
              f"{(sr[tc] - st.median(tw)) if tc in sr and tw else float('nan'):+6.2f}  "
              f"{st.mean(rv) if rv else float('nan'):8.1f} {st.median(sv) if sv else float('nan'):8.1f}  {len(tw)}/{len(T)}")
    print("png:", png)


if __name__ == "__main__":
    main()
