"""Scratch: por seed fallida: choque, pose real vs mapa antes del choque, mapa vs campo, estado ESP."""
import csv, json, re, sys
d = sys.argv[1]
for s in sys.argv[2:]:
    sd = f'{d}/s{s}'
    out = open(f'{sd}.out', encoding='utf-8').read()
    line = [l for l in out.splitlines() if l.startswith('STOP ')][-1]
    _, stop, js = line.split(' ', 2); m = json.loads(js)
    col = (m.get('collisions') or [{}])[0]
    fld = open(f'{sd}/field.txt', encoding='utf-8').read().splitlines()
    signs = {tuple(l.split()[1].split('/')): l.split()[0] for l in fld[1:]}
    conf = re.findall(r"\('(\w)', '(\w\d)'\): '(\w+)'", [l for l in out.splitlines() if l.startswith('CONF')][-1]) if 'CONF' in out else []
    cm = {(a, b): c for a, b, c in conf}
    wrong = [f"{k[0]}/{k[1]}:{v}" for k, v in cm.items() if signs.get(k) != v]
    missing = [f"{k[0]}/{k[1]}:{v}" for k, v in signs.items() if k not in cm]
    rows = [r for r in csv.DictReader(open(f'{sd}/trace.csv', encoding='utf-8')) if r['pi_frame'] == 'True']
    dmd = [l for l in open(f'{sd}/pi.log', encoding='utf-8') if l.startswith('[DMDBG]')]
    dmd = dmd[-len(rows):]
    tcol = float(col.get('t', rows[-1]['t'])) if col else float(rows[-1]['t'])
    errs = []
    for r, l in zip(rows, dmd):
        t = float(r['t'])
        mm = re.search(r'pose=\((-?\d+),(-?\d+)\)', l)
        if mm and tcol - 1.0 <= t <= tcol:
            errs.append((t, float(r['x']) - int(mm.group(1)), float(r['y']) - int(mm.group(2)), r['est'], float(r['servo'])))
    e = errs[-1] if errs else None
    r = rows[-1]
    print(f"== {s} {fld[0]} {stop} tc={m.get('turns_completed')} choque={col.get('cause')} t={tcol:.2f} pose=({float(r['x']):.0f},{float(r['y']):.0f},{float(r['heading'])%360:.0f}) est={r['est']}")
    if e: print(f"   err_mapa=({e[1]:.0f},{e[2]:.0f}) |e|={(e[1]**2+e[2]**2)**.5:.0f} srv={e[4]:.0f}  mapa_mal={wrong} faltan={missing}")
