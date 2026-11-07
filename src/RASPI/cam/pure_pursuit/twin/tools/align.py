"""Scratch: alinea [DMDBG] (pose del mapa) con el trace (pose real) por frame."""
import csv, re, sys
d = sys.argv[1]; t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0; t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
every = int(sys.argv[4]) if len(sys.argv) > 4 else 1
rows = [r for r in csv.DictReader(open(d + '/trace.csv', encoding='utf-8')) if r['pi_frame'] == 'True']
dm = [l for l in open(d + '/pi.log', encoding='utf-8') if l.startswith('[DMDBG]') or l.startswith('TX')]
# un TX por frame armado; DMDBG por frame armado (blocks_turn se llama una vez)
tx = [l for l in dm if l.startswith('TX')]
dmd = [l for l in dm if l.startswith('[DMDBG]')]
print(len(rows), len(tx), len(dmd))
n = 0
for r, l in zip(rows, dmd[-len(rows):] if len(dmd) >= len(rows) else dmd):
    t = float(r['t'])
    if not (t0 <= t <= t1): continue
    n += 1
    if n % every: continue
    m = re.search(r'sec=(\w) tc=(\d+) along=(-?\d+) lat=(-?\d+) pose=\((-?\d+),(-?\d+)\) h=(-?\d+)', l)
    bt = re.search(r'blocks_turn=(\w+) needs_line=(\w+)', l)
    sec, tc, al, la, mx, my, mh = m.groups()
    ex, ey = float(r['x']) - float(mx), float(r['y']) - float(my)
    print(f"{t:6.2f} real=({float(r['x']):.0f},{float(r['y']):.0f},{float(r['heading'])%360:.0f}) map=({mx},{my},{int(mh)%360}) err=({ex:.0f},{ey:.0f}) sec={sec}{tc} al={al} lat={la} est={r['est']} srv={float(r['servo']):.0f} bt={bt.group(1)[0]}{bt.group(2)[0]}")
# Pies: proyección con la pose real (error de proyección) vs la del mapa.
import math
OFF = sum(1 for l in open(d + "/pi.log", encoding="utf-8") if l.startswith("TX(desarmado)"))
if len(sys.argv) > 5:
    for l in open(d + '/pi.log', encoding='utf-8'):
        if not l.startswith('[DMFEET]'): continue
        parts = l.split()
        n = int(parts[1][2:])
        if n - 1 >= len(rows): continue
        n += OFF; r = rows[n - 1]; t = float(r['t'])
        if not (t0 <= t <= t1): continue
        X, Y, H = float(r['x']), float(r['y']), math.radians(float(r['heading']))
        o = []
        for p in parts[2:]:
            c = p[0]; mxy, fl = p[2:].split(')@'); f, la = map(float, fl.split(','))
            o.append(f"{c} map({mxy}) real({X + f*math.sin(H) + la*math.cos(H):.0f},{Y + f*math.cos(H) - la*math.sin(H):.0f}) f={f:.0f}")
        print(f"{t:6.2f} " + " | ".join(o))
