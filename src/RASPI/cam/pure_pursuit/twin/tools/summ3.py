"""Scratch: resumen de runs/all.sh: seed stop tc laps choque t park.

`run_row(sd)` es el criterio de "limpio" que también usa tools/ens.py (T16ens).
"""
import csv, json, math, sys, re
sys.path.insert(0, '.')
from pure_pursuit.wro_field import _barrier_corners
def park(sd):
    try:
        fld = open(f'{sd}/field.txt', encoding='utf-8').readline()
        rows = list(csv.DictReader(open(f'{sd}/trace.csv', encoding='utf-8')))
    except OSError:
        return '?', ''
    pk = fld.split('parking=')[1].split()[0]; dr = fld.split('dir=')[1].split()[0]
    r = rows[-1]; X, Y, H = float(r['x']), float(r['y']), math.radians(float(r['heading']))
    fx, fy = math.sin(H), math.cos(H); rx, ry = math.cos(H), -math.sin(H)
    cs = [(X + fx*a + rx*b, Y + fy*a + ry*b) for a in (-33.5, 146.5) for b in (-65, 65)]
    b1, b2 = _barrier_corners(pk)
    xs = [p[0] for p in b1 + b2]; ys = [p[1] for p in b1 + b2]
    if pk in ('E', 'W'):
        lo = min(max(p[1] for p in b1), max(p[1] for p in b2)); hi = max(min(p[1] for p in b1), min(p[1] for p in b2))
        box = (min(xs), max(xs), lo, hi)
    else:
        lo = min(max(p[0] for p in b1), max(p[0] for p in b2)); hi = max(min(p[0] for p in b1), min(p[0] for p in b2))
        box = (lo, hi, min(ys), max(ys))
    out = max(max(box[0]-cx, cx-box[1], box[2]-cy, cy-box[3], 0) for cx, cy in cs)
    return f'{dr}/{pk}', f'{out:.0f}'
def run_row(sd):
    """Una corrida (dir `sd` + `sd`.out) -> dict, o None si no hay línea STOP.
    ok = terminó (race_finished/terminado) sin choque y <= 5 mm fuera del cajón."""
    try:
        txt = open(f'{sd}.out', encoding='utf-8').read()
        line = [l for l in txt.splitlines() if l.startswith('STOP ')][-1]
    except (OSError, IndexError):
        return None
    _, stop, js = line.split(' ', 2); m = json.loads(js)
    col = m.get('collisions') or []
    cz = (col[0].get('cause') + f"@{col[0].get('t', '')}") if col and isinstance(col[0], dict) else (str(col[0]) if col else '-')
    pared_c = [c for c in (m.get('ignored_contacts') or []) if c.get('cause') == 'pared exterior']
    pared = f"{pared_c[0].get('max_mm', '')}mm@{pared_c[0]['t']}" if pared_c else ''
    dp, out = park(sd)
    ok = stop in ('race_finished', 'terminado') and not col and out != '' and float(out) <= 5
    return {'stop': stop, 'm': m, 'col': col, 'cz': cz, 'pared': pared, 'dp': dp, 'out': out, 'ok': ok}
def main(d, seeds):
    tot = clean = 0
    print('| seed | dir/park | stop | tc | laps | t | choque | fuera_mm | pared |')
    print('|---|---|---|---|---|---|---|---|---|')
    for s in seeds:
        r = run_row(f'{d}/s{s}')
        if r is None:
            print(f'| {s} | ? | ERROR | | | | | | |'); continue
        m, ok = r['m'], r['ok']
        tot += m.get('turns_completed', 0); clean += ok
        print(f"| {s} | {r['dp']} | {r['stop']} | {m.get('turns_completed')} | {len(m.get('lap_times_s') or [])} | {m.get('total_time_s')} | {r['cz']} | {r['out']}{' OK' if ok else ''} | {r['pared']} |")
    print(f'limpios={clean}/{len(seeds)} suma_tc={tot}')
if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
