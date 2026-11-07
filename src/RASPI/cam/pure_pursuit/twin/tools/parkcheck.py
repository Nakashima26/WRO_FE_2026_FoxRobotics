"""Scratch: ¿quedó el carro dentro del cajón? Pose final del trace vs barreras."""
import csv, math, sys
sys.path.insert(0, '..')
from pure_pursuit.wro_field import _barrier_corners
for d in sys.argv[1:]:
    try:
        fld = open(d + '/field.txt', encoding='utf-8').readline()
        rows = list(csv.DictReader(open(d + '/trace.csv', encoding='utf-8')))
    except OSError:
        print(d, 'sin datos'); continue
    park = fld.split('parking=')[1].split()[0]
    r = rows[-1]; X, Y, H = float(r['x']), float(r['y']), math.radians(float(r['heading']))
    fx, fy = math.sin(H), math.cos(H); rx, ry = math.cos(H), -math.sin(H)
    corners = [(X + fx * a + rx * b, Y + fy * a + ry * b) for a in (-33.5, 146.5) for b in (-65, 65)]
    b1, b2 = _barrier_corners(park)
    xs = [p[0] for p in b1 + b2]; ys = [p[1] for p in b1 + b2]
    # cajón = caja entre las caras interiores de las barreras
    if park in ('E', 'W'):
        lo_y = min(max(p[1] for p in b1), max(p[1] for p in b2)); hi_y = max(min(p[1] for p in b1), min(p[1] for p in b2))
        box = (min(xs), max(xs), lo_y, hi_y)
    else:
        lo_x = min(max(p[0] for p in b1), max(p[0] for p in b2)); hi_x = max(min(p[0] for p in b1), min(p[0] for p in b2))
        box = (lo_x, hi_x, min(ys), max(ys))
    inside = all(box[0] - 5 <= cx <= box[1] + 5 and box[2] - 5 <= cy <= box[3] + 5 for cx, cy in corners)
    out = max(max(box[0] - cx, cx - box[1], box[2] - cy, cy - box[3], 0) for cx, cy in corners)
    print(f"{d}: park={park} est={r['est']} pose=({X:.0f},{Y:.0f},{math.degrees(H)%360:.0f}) caja={tuple(round(v) for v in box)} dentro={inside} fuera_max={out:.0f}mm")
