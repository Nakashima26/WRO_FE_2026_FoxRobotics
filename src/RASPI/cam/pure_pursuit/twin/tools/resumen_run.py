"""Resumen rápido de una corrida real a partir de su _tele.jsonl.

Uso (en la Pi o en la PC, sin dependencias fuera de la stdlib):
  python resumen_run.py orillas256_tele.jsonl [--seg]   # --seg: línea por segmento de 0.5 s

Imprime: resultado (tc, estado final, duración), salida INICIO (rumbo del tramo recto),
mapa final, correcciones grandes del localizador, y tramos con servo oscilando
(cambios de lado del volante por segundo) por recta.
"""
import json
import sys


def main(path, seg=False):
    L = [json.loads(l) for l in open(path, encoding="utf-8")]
    t0 = L[0]["t"]
    acks, evs = [], []
    for x in L[1:]:
        s = x.get("s", {})
        a = s.get("ack")
        t = x["t"] - t0
        if a and "tc" in a and "est" in a:
            acks.append((t, a, s.get("loc", {}).get("pose")))
        for te, e in x.get("ev", []):
            evs.append((te - t0, e))
    if not acks:
        print("sin ACKs (no arrancó)")
        return
    last = acks[-1][1]
    tcmax = max(a["tc"] for _, a, _ in acks)
    print(f"== {path}  dur={L[-1]['t'] - t0:.1f}s  tc={tcmax:.0f}  est_final={last['est']}  "
          f"bld={L[1]['s'].get('pre', {}).get('bld')}")

    # Atasco: pose congelada con motor encendido al final
    tail = [a for _, a, _ in acks[-15:]]
    if tcmax < 12 and len(tail) > 10 and len({(a["px"], a["py"]) for a in tail}) <= 2 and tail[-1]["pw"] > 0:
        print("   ** ATORADO al final (pose congelada con pw>0) -> probable choque")

    ini = [(t, a) for t, a, _ in acks if a["est"] == "I"]
    if ini:
        rec = [abs(a["yaw"]) for _, a in ini if a["pw"] > 110 and abs(a["srv"] - 75) <= 50]
        print(f"   INICIO: {ini[0][0]:.1f}-{ini[-1][0]:.1f}s  pico={max(abs(a['yaw']) for _, a in ini):.0f}°  "
              f"recto={min(rec):.0f}-{max(rec):.0f}°" if rec else "   INICIO: sin tramo recto")

    print("   eventos:")
    for t, e in evs:
        if any(k in e for k in ("[DMAP]", "[MEMREJ]", "reancla", "[PARK", "INICIO", "abort", "ABORT", "CHOQUE")):
            print(f"     {t:6.2f} {e[:150]}")
        elif "[LOCUPD]" in e:
            try:
                dxy = [int(v) for v in e.split("dx=")[1].split(" dh=")[0].replace("dy=", "").split()]
            except Exception:
                dxy = [0, 0]
            if max(abs(v) for v in dxy) >= 60:
                print(f"     {t:6.2f} {e[:150]}")
        elif "[ROUTE] plan#" in e and "señales" in e:
            print(f"     {t:6.2f} {e.split(' conf=')[0][:40]} {e.split('señales=')[-1][:110]}")

    # Oscilación del servo (garrotera): cambios de lado del volante por ventana de 1 s en S/C
    print("   servo oscilando (>=3 cambios de lado en 1 s, |srv-75|>=12):")
    win = []
    reported = -9
    for t, a, pose in acks:
        if a["est"] not in ("S", "C", "R"):
            continue
        d = a["srv"] - 75
        side = 0 if abs(d) < 12 else (1 if d > 0 else -1)
        win.append((t, side))
        win = [w for w in win if t - w[0] <= 1.0]
        sides = [s for _, s in win if s]
        flips = sum(1 for i in range(1, len(sides)) if sides[i] != sides[i - 1])
        if flips >= 3 and t - reported > 1.0:
            reported = t
            p = [round(v) for v in pose] if pose else None
            print(f"     {t:6.2f} tc={a['tc']:.0f} flips={flips} pose={p} srvs={[round(x[1]['srv']) for x in acks if t-1<=x[0]<=t]}")
    if seg:
        for t, a, pose in acks:
            print(f"     {t:6.2f} {a['est']} tc={a['tc']:.0f} srv={a['srv']:.0f} pw={a['pw']:.0f} yaw={a['yaw']:.0f} "
                  f"tL={a['tL']:.0f} tR={a['tR']:.0f} tF={a['tF']:.0f} pose={[round(v) for v in pose] if pose else None}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for p in args:
        main(p, seg="--seg" in sys.argv)
