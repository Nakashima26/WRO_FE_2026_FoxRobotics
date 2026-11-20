"""Scratch: resumen de runs/all.sh: seed stop tc laps choque t park."""
import csv, json, sys, re
sys.path.insert(0, '.')
from pure_pursuit import config
from pure_pursuit.twin.world import body_corners
from pure_pursuit.wro_field import _barrier_corners
d = sys.argv[1]; seeds = sys.argv[2:]
def park(sd):
    try:
        fld = open(f'{sd}/field.txt', encoding='utf-8').readline()
        rows = list(csv.DictReader(open(f'{sd}/trace.csv', encoding='utf-8')))
    except OSError:
        return '?', ''
    pk = fld.split('parking=')[1].split()[0]; dr = fld.split('dir=')[1].split()[0]
    w0 = re.search(r'\bparkw=([\d.]+)', fld)
    if not rows:
        return f'{dr}/{pk}', ''
    r = rows[-1]
    cs = body_corners(float(r['x']), float(r['y']), float(r['heading']),
                      config.ROBOT_LENGTH_MM, config.ROBOT_WIDTH_MM, config.REAR_OVERHANG_MM)
    b1, b2 = _barrier_corners(pk, float(w0[1])) if w0 else _barrier_corners(pk)
    xs = [p[0] for p in b1 + b2]; ys = [p[1] for p in b1 + b2]
    if pk in ('E', 'W'):
        lo = min(max(p[1] for p in b1), max(p[1] for p in b2)); hi = max(min(p[1] for p in b1), min(p[1] for p in b2))
        box = (min(xs), max(xs), lo, hi)
    else:
        lo = min(max(p[0] for p in b1), max(p[0] for p in b2)); hi = max(min(p[0] for p in b1), min(p[0] for p in b2))
        box = (lo, hi, min(ys), max(ys))
    out = max(max(box[0]-cx, cx-box[1], box[2]-cy, cy-box[3], 0) for cx, cy in cs)
    return f'{dr}/{pk}', out
tot = clean = 0
print('| seed | dir/park | stop | tc | laps | t | choque | fuera_mm | pared |')
print('|---|---|---|---|---|---|---|---|---|')
for s in seeds:
    sd = f'{d}/s{s}'
    try:
        txt = open(f'{sd}.out', encoding='utf-8').read()
        line = [l for l in txt.splitlines() if l.startswith('STOP ')][-1]
    except (OSError, IndexError):
        print(f'| {s} | ? | ERROR | | | | | | |'); continue
    _, stop, js = line.split(' ', 2); m = json.loads(js)
    col = m.get('collisions') or []
    cz = (col[0].get('cause') + f"@{col[0].get('t', '')}") if col and isinstance(col[0], dict) else (str(col[0]) if col else '-')
    pared_c = [c for c in (m.get('ignored_contacts') or []) if c.get('cause') == 'pared exterior']
    pared = f"{pared_c[0].get('max_mm', '')}mm@{pared_c[0]['t']}" if pared_c else ''
    dp, out = park(sd)
    ok = stop in ('race_finished', 'terminado') and not col and out != '' and out <= 5
    tot += m.get('turns_completed', 0); clean += ok
    out_display = f'{out:.1f}' if out != '' else ''
    print(f"| {s} | {dp} | {stop} | {m.get('turns_completed')} | {len(m.get('lap_times_s') or [])} | {m.get('total_time_s')} | {cz} | {out_display}{' OK' if ok else ''} | {pared} |")
print(f'limpios={clean}/{len(seeds)} suma_tc={tot}')
