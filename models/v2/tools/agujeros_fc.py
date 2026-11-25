"""Agujeros M3 (caras cilíndricas de radio 1.2-1.9 mm, hacia adentro) de cada STEP de models/v2/step.
Corre dentro de FreeCAD:  freecadcmd models/v2/tools/agujeros_fc.py   ->  models/v2/tools/agujeros.json
Coordenadas de la pieza en mm; tornillos.py las pasa al ensamble."""
import FreeCAD, Part, json, glob, os
HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else os.path.abspath("models/v2/tools")
out = {}
for f in sorted(glob.glob(os.path.join(HERE, "..", "step", "*.step"))):
    s = Part.Shape(); s.read(f)
    bb = s.BoundBox
    hs = []
    for fc in s.Faces:
        su = fc.Surface
        if su.__class__.__name__ != "Cylinder" or not (1.2 <= su.Radius <= 1.9):
            continue
        u0, u1, v0, v1 = fc.ParameterRange
        p = fc.valueAt((u0 + u1) / 2, (v0 + v1) / 2); n = fc.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
        ax, c = su.Axis, su.Center
        d = p - c; d = d - ax * d.dot(ax)
        if n.dot(d) >= 0:        # perno, no agujero
            continue
        ts = [(v.Point - c).dot(ax) for v in fc.Vertexes] or [0.0]
        hs.append(dict(r=round(su.Radius, 3), c=[c.x, c.y, c.z], ax=[ax.x, ax.y, ax.z], t0=min(ts), t1=max(ts)))
    out[os.path.basename(f)[:-5]] = dict(bb=[bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax], holes=hs)
json.dump(out, open(os.path.join(HERE, "agujeros.json"), "w"), indent=0)
print("agujeros.json:", {k: len(v["holes"]) for k, v in out.items()})
