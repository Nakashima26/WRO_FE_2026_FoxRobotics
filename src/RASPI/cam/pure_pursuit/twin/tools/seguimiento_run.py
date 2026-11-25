"""Qué tan bien sigue el carro la ruta planeada, desde el _tele.jsonl de una corrida real.

Uso:  python seguimiento_run.py orillas261_tele.jsonl [más...] [--det]

Para cada ACK en S (sin INICIO ni parking) busca el punto más cercano de la ruta vigente y mide:
  cte   = error lateral con signo (mm, + = carro a la izquierda de la ruta)
  eh    = rumbo del carro - rumbo de la ruta en ese punto (°)
  giro  = cuánto gira la ruta en los próximos 400 mm (°): separa recta de curva/slalom
Tramos: "recta" si |giro| < 12°, "curva" si no. Por tramo y vuelta (L1 = tc 0-3, L23 = tc 4-11):
  rms eh, rms cte, cruces/s de eh (cambios de signo con |eh| > 4°) y periodo dominante de eh.
La oscilación de CONTROL es eh/cte cruzando cero en tramos rectos; en curvas parte del
movimiento del volante es la ruta misma.
"""
import json
import math
import sys


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def heading(dx, dy):           # 0 = +y, 90 = +x (convención del tapete)
    return math.degrees(math.atan2(dx, dy))


def prep(route):
    pts = [(float(p[0]), float(p[1])) for p in route]
    s = [0.0]
    for i in range(1, len(pts)):
        s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    return pts, s


def nearest(pts, x, y):
    best, bi, bt = 1e18, 0, 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        if L2 < 1e-6:
            continue
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
        px, py = ax + t * vx, ay + t * vy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best:
            best, bi, bt = d2, i, t
    return bi, bt, math.sqrt(best)


def seg_heading(pts, i):
    i = max(0, min(i, len(pts) - 2))
    return heading(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])


def ahead_index(s, i, mm):
    j = i
    while j < len(s) - 2 and s[j] - s[i] < mm:
        j += 1
    return j


def analiza(path, det=False):
    L = [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]
    if len(L) < 2:
        return []
    t0 = L[0]["t"]
    route = None
    rows = []
    for x in L[1:]:
        s = x.get("s", {})
        if s.get("route"):
            route = prep(s["route"])
        a = s.get("ack")
        pose = (s.get("loc") or {}).get("pose")
        if not (a and pose and route) or a["est"] != "S" or a["tc"] > 11:
            continue
        pts, ss = route
        if len(pts) < 10:
            continue
        i, tt, d = nearest(pts, pose[0], pose[1])
        if d > 400:
            continue   # ruta vieja / fuera: no cuenta
        rh = seg_heading(pts, i)
        j = ahead_index(ss, i, 400.0)
        giro = wrap(seg_heading(pts, j) - rh)
        # signo del cte: + si el carro queda a la izquierda de la ruta
        vx, vy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        cross = vx * (pose[1] - pts[i][1]) - vy * (pose[0] - pts[i][0])
        cte = d if cross > 0 else -d
        eh = wrap(pose[2] - rh)
        rows.append((x["t"] - t0, a["tc"], eh, cte, giro, a["srv"]))
    return rows


def resume(rows, nombre):
    out = []
    for lap, lo, hi in (("L1", 0, 3), ("L23", 4, 11)):
        for tramo in ("recta", "curva"):
            g = [r for r in rows if lo <= r[1] <= hi and ((abs(r[4]) < 12) == (tramo == "recta"))]
            if len(g) < 10:
                continue
            dur = len(g) * 0.1
            rms_eh = math.sqrt(sum(r[2] ** 2 for r in g) / len(g))
            rms_cte = math.sqrt(sum(r[3] ** 2 for r in g) / len(g))
            sig = [1 if r[2] > 4 else -1 for r in g if abs(r[2]) > 4]
            cruces = sum(1 for k in range(1, len(sig)) if sig[k] != sig[k - 1])
            sig2 = [1 if r[3] > 30 else -1 for r in g if abs(r[3]) > 30]
            cruces_cte = sum(1 for k in range(1, len(sig2)) if sig2[k] != sig2[k - 1])
            per = (2 * dur / cruces) if cruces else float("nan")
            out.append((nombre, lap, tramo, len(g), rms_eh, rms_cte, cruces / dur, cruces_cte / dur, per))
    return out


if __name__ == "__main__":
    files = [a for a in sys.argv[1:] if not a.startswith("--")]
    det = "--det" in sys.argv
    allr = []
    print(f"{'run':12} {'vuelta':5} {'tramo':6} {'n':>4} {'rms_eh°':>8} {'rms_cte':>8} {'eh cruces/s':>11} {'cte cruces/s':>12} {'periodo s':>9}")
    for f in files:
        rows = analiza(f, det)
        name = f.replace("\\", "/").split("/")[-1].replace("_tele.jsonl", "")
        for r in resume(rows, name):
            allr.append(r)
            print(f"{r[0]:12} {r[1]:5} {r[2]:6} {r[3]:4d} {r[4]:8.1f} {r[5]:8.0f} {r[6]:11.2f} {r[7]:12.2f} {r[8]:9.2f}")
        if det:
            for r in rows:
                print(f"   {r[0]:6.2f} tc={r[1]:.0f} eh={r[2]:+6.1f} cte={r[3]:+5.0f} giro={r[4]:+5.0f} srv={r[5]:.0f}")
    if len(files) > 1:
        for lap in ("L1", "L23"):
            for tramo in ("recta", "curva"):
                g = [r for r in allr if r[1] == lap and r[2] == tramo]
                if not g:
                    continue
                n = sum(r[3] for r in g)
                w = lambda k: sum(r[k] * r[3] for r in g) / n
                print(f"{'TOTAL':12} {lap:5} {tramo:6} {n:4d} {w(4):8.1f} {w(5):8.0f} {w(6):11.2f} {w(7):12.2f}")
