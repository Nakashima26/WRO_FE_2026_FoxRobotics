"""Lista de tornillos M3x12 del carro v2 (tornillos.json) a partir de los agujeros de cada pieza.

El CAD no trae tornillería. Los agujeros M3 de agujeros.json (agujeros_fc.py, en FreeCAD) se pasan al
ensamble con la transformación de cada pieza en Vehicle_V2.glb; los que quedan en la misma línea en dos o
más piezas son un tornillo. Verticales: cabeza arriba. Horizontales: del lado de la pieza que se monta después (orden de ORDER = el de
armado.py); si justo encima hay otra pieza (una PCB sobre un poste), la cabeza queda sobre esa pieza.
Agujeros de una sola pieza no cuentan (no se sabe qué sujetan). Piñón y diferencial no llevan tornillo.
  python models/v2/tools/tornillos.py Vehicle_V2.glb   ->  models/v2/tools/tornillos.json
"""
import json, os, sys
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER = ["BaseChasisMain_V2-1", "SteeringKnuckle-1", "SteeringKnuckle-2", "NewCremallera_V2-1", "NewLinkajeL_V2-1",
         "NewLinkajeR_V2-1", "DCsupport_V2-1", "BatterySupport-1", "BatterySupport-2", "UltrasonicSupportFront-1",
         "UltrasonicSupportHalf_V2-1", "UltrasonicSupport_V2-2", "TapaChasis_V2-1", "ToFSupport-1", "ToFBack-1",
         "RASPBERRY_PI_5.STEP", "WRO_RevB_2.step", "WRO_RevB_1.step", "CameraBase-1", "CameraFrnt-1"]
SKIP = {"Pinion_V2", "Differential_V2"}

sc = trimesh.load(sys.argv[1])
G = sc.graph
par = {b: a for a, b, *_ in G.to_edgelist()}
top = next(b for a, b, *_ in G.to_edgelist() if a == G.base_frame)
H = json.load(open(os.path.join(HERE, "agujeros.json")))
xf, meshes = {}, {}
for n in G.nodes_geometry:
    T, gname = G[n]
    c = n
    while par.get(c) not in (None, top):
        c = par[c]
    m = sc.geometry[gname].copy(); m.apply_transform(T)
    meshes.setdefault(c, []).append(m)
    base = c.rsplit("-", 1)[0].replace("Pinion.15_V2", "Pinion_V2")
    if base in H and c not in xf:
        xf[c] = (base, T)
meshes = {c: trimesh.util.concatenate(l) for c, l in meshes.items()}

holes = []
for comp, (base, T) in xf.items():
    if base in SKIP:
        continue
    for h in H[base]["holes"]:
        cw = (T @ np.r_[np.array(h["c"]) / 1000, 1])[:3] * 1000
        aw = T[:3, :3] @ np.array(h["ax"]); aw /= np.linalg.norm(aw)
        holes.append(dict(comp=comp, r=h["r"], p0=cw + aw * h["t0"], p1=cw + aw * h["t1"], a=aw))

groups = []
for h in holes:
    for g in groups:
        k = g[0]
        d = h["p0"] - k["p0"]; d = d - k["a"] * (d @ k["a"])
        if abs(h["a"] @ k["a"]) > 0.995 and np.linalg.norm(d) < 0.6:
            g.append(h); break
    else:
        groups.append([h])

out = []
for g in groups:
    comps = sorted({h["comp"] for h in g}, key=ORDER.index)
    if len(comps) < 2:
        continue
    a, o = g[0]["a"], g[0]["p0"]
    span = lambda hs: (min(min((x["p0"] - o) @ a, (x["p1"] - o) @ a) for x in hs), max(max((x["p0"] - o) @ a, (x["p1"] - o) @ a) for x in hs))
    lo, hi = span(g)
    later = comps[-1]
    l0, l1 = span([x for x in g if x["comp"] == later])
    if abs(a[1]) > 0.9 or (abs(hi - l1) < 0.3 and abs(lo - l0) < 0.3):   # vertical (o la pieza abarca todo): cabeza arriba
        sgn = 1 if a[1] >= 0 else -1
    else:
        sgn = 1 if abs(hi - l1) < abs(lo - l0) else -1
    end = hi if sgn > 0 else lo
    out_dir = a * sgn
    head = o + a * end
    step_part = later
    # ¿otra pieza justo encima de la cabeza? (PCB sobre un poste): la cabeza va sobre ella
    org = head / 1000
    for n, m in meshes.items():
        if n in comps:
            continue
        loc, _, _ = m.ray.intersects_location([org], [out_dir])
        if not len(loc):
            continue
        d = np.sort(np.linalg.norm(loc - org, axis=1)) * 1000
        if d[0] < 6:
            far = d[d < d[0] + 4][-1]
            head = head + out_dir * far
            if n in ORDER and ORDER.index(n) > ORDER.index(step_part):
                step_part = n
    out.append(dict(head=np.round(head, 2).tolist(), out=np.round(out_dir, 4).tolist(), comps=comps, step_part=step_part))

json.dump(out, open(os.path.join(HERE, "tornillos.json"), "w"), indent=1)
print(len(out), "tornillos M3x12")
for t in out:
    print(" ", t["step_part"], "<-", t["comps"], "cabeza", t["head"], "hacia", t["out"])
