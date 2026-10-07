"""Perfil involuto de engrane recto (modulo m, Z dientes, 20 grados) -> DXF R12 en mm.
Un solo contorno cerrado: flancos como lineas cortas, punta y raiz como ARC. Sin circulos de referencia."""
import math, sys

def inv(a): return math.tan(a) - a

def gear(m=1.0, Z=19, pa_deg=20.0, backlash=0.10, n_flank=12):
    pa = math.radians(pa_deg)
    rp = m*Z/2; rb = rp*math.cos(pa); ra = rp + m; rf = rp - 1.25*m
    psi = (math.pi*m/2 - backlash)/(2*rp)              # media espesura angular en el primitivo
    theta = lambda r: psi + inv(pa) - inv(math.acos(min(1.0, rb/r)))
    r0 = max(rb, rf)
    pol = lambda r, a: (r*math.cos(a), r*math.sin(a))
    lines, arcs = [], []
    def chain(p):
        for a, b in zip(p, p[1:]): lines.append((a, b))
    for i in range(Z):
        c = 2*math.pi*i/Z; n = 2*math.pi*(i+1)/Z
        left = [pol(r0 + (ra-r0)*k/n_flank, c - theta(r0 + (ra-r0)*k/n_flank)) for k in range(n_flank+1)]
        right = [pol(r0 + (ra-r0)*k/n_flank, c + theta(r0 + (ra-r0)*k/n_flank)) for k in range(n_flank, -1, -1)]
        if rf < rb: left = [pol(rf, c - theta(rb))] + left; right = right + [pol(rf, c + theta(rb))]
        chain(left); chain(right)
        arcs.append((ra, c - theta(ra), c + theta(ra)))                    # punta
        t0 = theta(rb if rf < rb else r0)
        arcs.append((rf, c + t0, n - t0))                                    # raiz hasta el siguiente diente
    return lines, arcs, dict(rp=rp, rb=rb, ra=ra, rf=rf)

def write_dxf(path, lines, arcs):
    L = ["0","SECTION","2","HEADER","9","$ACADVER","1","AC1009","0","ENDSEC",
         "0","SECTION","2","ENTITIES"]
    for (x1, y1), (x2, y2) in lines:
        L += ["0","LINE","8","PERFIL","10",f"{x1:.6f}","20",f"{y1:.6f}","30","0","11",f"{x2:.6f}","21",f"{y2:.6f}","31","0"]
    for r, a1, a2 in arcs:
        L += ["0","ARC","8","PERFIL","10","0","20","0","30","0","40",f"{r:.6f}",
              "50",f"{math.degrees(a1) % 360:.6f}","51",f"{math.degrees(a2) % 360:.6f}"]
    L += ["0","ENDSEC","0","EOF"]
    open(path, "w").write("\n".join(L) + "\n")

if __name__ == "__main__":
    for Z in (int(a) for a in (sys.argv[1:] or ["19", "20"])):
        lines, arcs, r = gear(Z=Z)
        write_dxf(f"pinon_m1_Z{Z}.dxf", lines, arcs)
        print(f"Z{Z}: primitivo {2*r['rp']:.2f}  exterior {2*r['ra']:.2f}  raiz {2*r['rf']:.2f}  ({len(lines)} lineas, {len(arcs)} arcos)")
