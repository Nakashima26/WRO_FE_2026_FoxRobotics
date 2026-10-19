"""
Analiza un log de VueltaCalibracion: dibuja la ruta del carro sobre el mapa de
la pista y compara cada lectura de los ToF contra la distancia que DEBIÓ dar
según el mapa y la posición del carro (encoder + BNO).

Uso:
  python scripts/analiza_vuelta.py logs_tof/vuelta_20261019_120000.txt
  python scripts/analiza_vuelta.py LOG --no-mostrar     solo guarda los PNG
  python scripts/analiza_vuelta.py LOG --escala-encoder 1.2
      corrige un log grabado con la escala del encoder mal: el recorrido real
      fue 1.2 veces el que calculó el carro (cuentas que usó / cuentas reales)

Guarda las gráficas junto al log (LOG_mapa.png, ...) e imprime un resumen por
sensor y status.

Cada lectura se clasifica así (el ToF ve un cono, no una línea: a una pared en
ángulo le puede medir cualquier punto dentro del cono):
  ok     dentro de [lo más cerca, lo más lejos] que hay en el cono, ± TOL_MM
  corto  más cerca que cualquier pared del cono  -> fantasma
  largo  más lejos que la pared                   -> no vio la pared
  ambiguo el cono abarca más de AMBIGUO_MM (pared muy en ángulo o esquina): no cuenta

ANTES DE USARLO: mide SENSORES (dónde está cada ToF respecto al centro del eje
trasero) y revisa INTERIOR si las paredes de adentro no están en su lugar normal.
"""
import argparse
import os
import sys

import numpy as np

# ── Pista (mm). Origen: esquina exterior detrás del carro al arrancar ────────
PISTA_MM = 3000.0
# Cuadro interior (x0, y0, x1, y1). Normal: 1 m de pasillo en los 4 lados.
# En la ronda abierta se pueden mover; ponlas donde estén.
INTERIOR = (1000.0, 1000.0, 2000.0, 2000.0)

# ── ToF en el carro: (adelante mm, izquierda mm, ángulo °) desde el centro del
# eje trasero. ¡MIDE ESTO! Un error aquí sale como error en todas las lecturas.
SENSORES = {
    "F": (180.0,   0.0,   0.0),
    "L": (100.0,  60.0,  90.0),
    "R": (100.0, -60.0, -90.0),
}
CONO_DEG = 15.0        # ancho total del cono con ROI 8 (~27° con ROI 16)
TOL_MM = 60.0          # error que se le perdona a la posición estimada
# Si dentro del cono la pared va de muy cerca a muy lejos (pared casi paralela al
# rayo, o una esquina), cualquier lectura "cabe": esas no cuentan para el resumen.
AMBIGUO_MM = 400.0
RETRASO_MS = 25.0      # la medición dura 50 ms: la distancia es de ~25 ms antes de leerla
RANGO_MAX_MM = 4000.0

STATUS = {0: "válida", 1: "sigma", 2: "poca luz", 3: "mín. recortada", 4: "fuera de rango",
          5: "hardware", 6: "sin wrap check", 7: "wrap fail", 255: "nada"}


def paredes():
    """Segmentos (a, b) de todas las paredes."""
    P = PISTA_MM
    x0, y0, x1, y1 = INTERIOR
    caja = lambda a, b, c, d: [((a, b), (c, b)), ((c, b), (c, d)), ((c, d), (a, d)), ((a, d), (a, b))]
    seg = caja(0, 0, P, P) + caja(x0, y0, x1, y1)
    return np.array([s[0] for s in seg]), np.array([s[1] for s in seg])


def leer_log(ruta):
    P, T, E, cfg = [], [], [], []
    with open(ruta, encoding="utf-8", errors="replace") as f:
        for linea in f:
            linea = linea.strip()
            # La app del celular le pone hora al principio: se ignora lo de antes de "P,", "T,", ...
            for tag in ("P,", "T,", "E,", "#"):
                i = linea.find(tag)
                if i >= 0 and (i == 0 or linea[i - 1] == " "):
                    linea = linea[i:]
                    break
            c = linea.split(",")
            try:
                if c[0] == "P" and len(c) >= 6:
                    P.append((float(c[1]), float(c[2]), float(c[3]), float(c[4]), int(c[5])))
                elif c[0] == "T" and len(c) >= 7:
                    T.append((float(c[1]), c[2], float(c[3]), int(c[4]), float(c[5]), float(c[6])))
                elif c[0] == "E":
                    E.append((float(c[1]), ",".join(c[2:])))
                elif linea.startswith("#"):
                    cfg.append(linea)
            except (ValueError, IndexError):
                pass   # línea cortada por el BT
    if not P or not T:
        sys.exit("El log no tiene líneas P (posición) y T (ToF). ¿Es de VueltaCalibracion?")
    P = np.array(P)
    return P, T, E, cfg


def rayos(o, ang, a, b):
    """Distancia de cada rayo (origen o[N,2], ángulo ang[N] en rad) a la pared más cercana."""
    d = np.stack([np.cos(ang), np.sin(ang)], axis=-1)[:, None, :]   # N,1,2
    e = (b - a)[None, :, :]                                          # 1,S,2
    ap = a[None, :, :] - o[:, None, :]                               # N,S,2
    cruz = lambda u, v: u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    den = cruz(d, e)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = cruz(ap, e) / den
        u = cruz(ap, d) / den
    t = np.where((np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    return np.minimum(t.min(axis=1), RANGO_MAX_MM)


def analizar(P, T):
    tP, xP, yP, thP = P[:, 0], P[:, 1], P[:, 2], P[:, 3]
    a, b = paredes()
    filas = {k: [] for k in ("t", "sensor", "mm", "st", "sig", "amb", "ox", "oy", "ang", "c", "lo", "hi")}
    for t, s, mm, st, sig, amb in T:
        if s not in SENSORES:
            continue
        tm = t - RETRASO_MS
        if tm < tP[0] or tm > tP[-1]:
            continue
        x, y, th = np.interp(tm, tP, xP), np.interp(tm, tP, yP), np.interp(tm, tP, thP)
        adel, izq, ang_s = SENSORES[s]
        r = np.radians(th)
        ox = x + adel * np.cos(r) - izq * np.sin(r)
        oy = y + adel * np.sin(r) + izq * np.cos(r)
        for k, v in zip(("t", "sensor", "mm", "st", "sig", "amb", "ox", "oy", "ang"),
                        (t, s, mm, st, sig, amb, ox, oy, np.radians(th + ang_s))):
            filas[k].append(v)
    D = {k: np.array(v) for k, v in filas.items() if v}
    if not D:
        sys.exit("Ninguna lectura cae dentro del tiempo de las posiciones.")
    n = len(D["mm"])
    o = np.stack([D["ox"], D["oy"]], axis=1)
    # Cono: rayos cada ~1° dentro de ±CONO/2.
    offs = np.radians(np.linspace(-CONO_DEG / 2, CONO_DEG / 2, max(3, int(CONO_DEG) + 1)))
    dist = np.stack([rayos(o, D["ang"] + off, a, b) for off in offs], axis=1)
    D["c"] = dist[:, len(offs) // 2]
    D["lo"], D["hi"] = dist.min(axis=1), dist.max(axis=1)
    D["clase"] = np.where(D["hi"] - D["lo"] > AMBIGUO_MM, "ambiguo",
                 np.where(D["mm"] < D["lo"] - TOL_MM, "corto",
                 np.where(D["mm"] > D["hi"] + TOL_MM, "largo", "ok")))
    D["err"] = D["mm"] - D["c"]
    return D, n


def resumen(D, E, cfg):
    for c in cfg:
        if not c.startswith("# listo"):
            print(c)
    radios = [e[1] for e in E if "radio_medido" in e[1]]
    if radios:
        print("\nGiros:")
        for r in radios:
            print("  " + r)
    amb = (D["clase"] == "ambiguo").sum()
    print(f"\nPor sensor y status (esperado = lo que hay en el cono según el mapa;"
          f" {amb} lecturas ambiguas no cuentan):")
    print(f"  {'sensor':6s} {'status':18s} {'n':>5s} {'ok':>6s} {'corto':>6s} {'largo':>6s}"
          f" {'err med ok':>10s} {'señal ok p10':>12s} {'señal mal p90':>13s}")
    for s in SENSORES:
        for st in sorted(set(D["st"][D["sensor"] == s])):
            m = (D["sensor"] == s) & (D["st"] == st) & (D["clase"] != "ambiguo")
            if not m.any():
                continue
            ok, cor, lar = (m & (D["clase"] == k) for k in ("ok", "corto", "largo"))
            mal = cor | lar
            err = np.median(D["err"][ok]) if ok.any() else np.nan
            s_ok = np.percentile(D["sig"][ok], 10) if ok.any() else np.nan
            s_mal = np.percentile(D["sig"][mal], 90) if mal.any() else np.nan
            nm = m.sum()
            print(f"  {s:6s} {st:2d} {STATUS.get(st, '?'):15s} {nm:5d} {100*ok.sum()/nm:5.0f}%"
                  f" {100*cor.sum()/nm:5.0f}% {100*lar.sum()/nm:5.0f}% {err:9.0f}mm {s_ok:12.3f} {s_mal:13.3f}")
    print("\n  err med ok = medido - esperado en las buenas (si todas dan +30 mm, el sensor"
          "\n  tiene un offset o está mal medida su posición en SENSORES).")
    print("  señal: si 'señal ok p10' > 'señal mal p90', un umbral de señal entre los dos"
          "\n  separa pared real de fantasma para ese status.")
    print("\nFantasmas (corto) por sensor:")
    for s in SENSORES:
        m = (D["sensor"] == s) & (D["clase"] == "corto")
        if m.any():
            print(f"  {s}: {m.sum()} | medido {np.median(D['mm'][m]):.0f} mm (mediana),"
                  f" esperado {np.median(D['c'][m]):.0f} mm, status {sorted(set(D['st'][m].tolist()))}")


def graficas(P, D, E, base, mostrar):
    import matplotlib
    import warnings
    warnings.filterwarnings("ignore", message="No artists with labels")
    if not mostrar:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a, b = paredes()

    def mapa(ax, titulo):
        for p, q in zip(a, b):
            ax.plot([p[0], q[0]], [p[1], q[1]], color="k", lw=3)
        ax.set_aspect("equal")
        ax.set_xlim(-200, PISTA_MM + 200)
        ax.set_ylim(-200, PISTA_MM + 200)
        ax.set_title(titulo)
        ax.plot(P[:, 1], P[:, 2], color="0.6", lw=1)
        ax.plot(P[0, 1], P[0, 2], "k^", ms=9)

    # Donde "cayó" cada lectura: origen del sensor + distancia medida en su dirección.
    hx = D["ox"] + np.minimum(D["mm"], RANGO_MAX_MM) * np.cos(D["ang"])
    hy = D["oy"] + np.minimum(D["mm"], RANGO_MAX_MM) * np.sin(D["ang"])
    marc = {"F": "o", "L": "s", "R": "D"}
    colores = {"ok": "tab:green", "corto": "tab:red", "largo": "tab:orange", "ambiguo": "0.75"}

    # 1) Ruta + dónde cayó cada lectura, por clase.
    fig, ax = plt.subplots(figsize=(9, 9))
    mapa(ax, "Ruta y lecturas (verde ok, rojo fantasma, naranja no vio la pared, gris ambigua)")
    sc = ax.scatter(P[:, 1], P[:, 2], c=P[:, 0] / 1000, cmap="viridis", s=4, zorder=3)
    fig.colorbar(sc, ax=ax, shrink=0.7, label="tiempo (s)")
    for s, mk in marc.items():
        for cl, col in colores.items():
            m = (D["sensor"] == s) & (D["clase"] == cl)
            ax.scatter(hx[m], hy[m], marker=mk, s=10, color=col, alpha=0.6,
                       label=f"{s} {cl}" if m.any() else None)
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    fig.savefig(base + "_mapa.png", dpi=130, bbox_inches="tight")

    # 2) Lo mismo pero por status, un mapa por sensor.
    fig, axs = plt.subplots(1, len(SENSORES), figsize=(6 * len(SENSORES), 6))
    sts = sorted(set(D["st"].tolist()))
    cmap = plt.get_cmap("tab10")
    for ax, s in zip(axs, SENSORES):
        mapa(ax, f"ToF {s}: dónde cayó cada lectura, por status")
        for i, st in enumerate(sts):
            m = (D["sensor"] == s) & (D["st"] == st)
            if m.any():
                ax.scatter(hx[m], hy[m], s=8, color=cmap(i % 10), label=f"{st} {STATUS.get(st, '?')} ({m.sum()})")
        ax.legend(loc="upper right", fontsize=7)
    fig.savefig(base + "_status.png", dpi=110, bbox_inches="tight")

    # 3) Medido vs esperado, y señal vs esperado (buenas vs malas).
    fig, axs = plt.subplots(2, len(SENSORES), figsize=(6 * len(SENSORES), 10))
    for j, s in enumerate(SENSORES):
        m = D["sensor"] == s
        ax = axs[0, j]
        for i, st in enumerate(sts):
            mm = m & (D["st"] == st)
            if mm.any():
                ax.scatter(D["c"][mm], D["mm"][mm], s=8, color=cmap(i % 10), label=f"{st} {STATUS.get(st, '?')}")
        lim = [0, RANGO_MAX_MM]
        ax.plot(lim, lim, "k--", lw=1)
        ax.set_xlim(0, 3200); ax.set_ylim(0, 3200)
        ax.set_xlabel("esperado (mm, centro del cono)"); ax.set_ylabel("medido (mm)")
        ax.set_title(f"ToF {s}: medido vs esperado"); ax.legend(fontsize=7)
        ax = axs[1, j]
        for cl, col in colores.items():
            mm = m & (D["clase"] == cl)
            if mm.any():
                ax.scatter(D["c"][mm], D["sig"][mm], s=8, color=col, label=cl, alpha=0.7)
        ax.set_yscale("log")
        ax.set_xlabel("esperado (mm)"); ax.set_ylabel("señal (MCPS)")
        ax.set_title(f"ToF {s}: señal (¿se separan ok y malas?)"); ax.legend(fontsize=7)
    fig.savefig(base + "_dispersion.png", dpi=110, bbox_inches="tight")

    # 4) Luz ambiente por zona de la pista.
    fig, ax = plt.subplots(figsize=(8, 8))
    mapa(ax, "Luz ambiente que vio cada ToF (por dónde estaba el carro)")
    sc = ax.scatter(D["ox"], D["oy"], c=D["amb"], cmap="inferno", s=12, zorder=4)
    fig.colorbar(sc, ax=ax, shrink=0.7, label="ambiente (MCPS)")
    fig.savefig(base + "_ambiente.png", dpi=120, bbox_inches="tight")

    print(f"\nGráficas: {base}_mapa.png, _status.png, _dispersion.png, _ambiente.png")
    if mostrar:
        plt.show()


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--no-mostrar", action="store_true", help="no abre ventanas, solo guarda los PNG")
    ap.add_argument("--escala-encoder", type=float, default=1.0,
                    help="multiplica todo el recorrido (desde el arranque) por esto")
    args = ap.parse_args()
    P, T, E, cfg = leer_log(args.log)
    if args.escala_encoder != 1.0:
        # Cada paso es ds*(cos, sin): con ds mal escalado, toda la ruta crece igual desde el arranque.
        P[:, 1:3] = P[0, 1:3] + args.escala_encoder * (P[:, 1:3] - P[0, 1:3])
        print(f"Recorrido escalado x{args.escala_encoder} desde el arranque")
    D, n = analizar(P, T)
    print(f"{len(P)} posiciones, {n} lecturas ToF analizadas")
    for s in SENSORES:
        if not (D["sensor"] == s).any():
            print(f"!! El ToF {s} no tiene NINGUNA lectura: revisa su cable / que haya arrancado.")
    resumen(D, E, cfg)
    base = os.path.splitext(args.log)[0]
    if args.escala_encoder != 1.0:
        base += f"_esc{args.escala_encoder:g}"
    graficas(P, D, E, base, not args.no_mostrar)


if __name__ == "__main__":
    main()
