"""Scratch: error de la odometría del ESP (px,py del ACK) y del mapa vs la pose real, por tc."""
import csv, math, re, sys
d = sys.argv[1]
for s in sys.argv[2:]:
    sd = f'{d}/s{s}'
    fld = open(f'{sd}/field.txt', encoding='utf-8').readline()
    sx, sy, sh = map(float, re.search(r'start=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)', fld).groups())
    H = math.radians(sh); fx, fy = math.sin(H), math.cos(H); lx, ly = -math.cos(H), math.sin(H)
    rows = [r for r in csv.DictReader(open(f'{sd}/trace.csv', encoding='utf-8')) if r['pi_frame'] == 'True']
    dmd = [l for l in open(f'{sd}/pi.log', encoding='utf-8') if l.startswith('[DMDBG]')][-len(rows):]
    by = {}
    for r, l in zip(rows, dmd):
        a = r['ack']
        m = re.search(r'tc=(\d+).*px=(-?\d+),py=(-?\d+)', a)
        if not m: continue
        tc, px, py = int(m.group(1)), float(m.group(2)), float(m.group(3))
        wx, wy = sx + px * fx + py * lx, sy + px * fy + py * ly
        e = math.hypot(float(r['x']) - wx, float(r['y']) - wy)
        mm = re.search(r'pose=\((-?\d+),(-?\d+)\)', l)
        em = math.hypot(float(r['x']) - int(mm.group(1)), float(r['y']) - int(mm.group(2)))
        by.setdefault(tc, []).append((e, em))
    print(s, ' '.join(f"tc{k}:esp={max(v)[0]:.0f}/map={max(x[1] for x in v):.0f}" for k, v in sorted(by.items())))
