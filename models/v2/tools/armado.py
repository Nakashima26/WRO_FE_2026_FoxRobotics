"""Video de armado del carro v2 (models/v2/renders/armado.mp4): el ensamble gira, se despieza y se arma
en 12 pasos, con colores por pieza (el CAD deja casi todo en gris).

Entrada: el ensamble exportado de SolidWorks a .glb (Vehicle_V2.glb, ~50 MB, no está en el repo:
Archivo > Guardar como > glTF binario desde cad/Vehicle_V2.SLDASM).
  pip install trimesh pyrender imageio imageio-ffmpeg pillow
  python models/v2/tools/armado.py Vehicle_V2.glb models/v2/renders/armado.mp4 [--fast]
      [--servo=sg90.stl --gear=ServoGearDirection_V2.STL]
--fast: 4x más rápido, para revisar.
Tornillos M3x12: tornillos.json (tornillos.py). Ejes LEGO de las ruedas: se generan aquí (atrás uno de 8 por llanta, adelante uno de 4 por mangueta).
El servo SG90 y su engrane no están en Vehicle_V2.SLDASM: con --servo/--gear se agregan en su lugar
(acostado sobre la cremallera, eje hacia atrás, orejas en los dos postes; engrane sobre los dientes, que miran
arriba). sg90.stl = el STEP del SG90 pasado a STL (FreeCAD: MeshPart.meshFromShape). Render en una GPU de laptop: ~1.5 min. Arriba = +Y, adelante = -Z.
"""
import pickle, sys, math, collections, os, json, numpy as np, pyrender, imageio, trimesh
from PIL import Image, ImageDraw, ImageFont
W, H, FPS = 1280, 720, 30
POS = [a for a in sys.argv[1:] if not a.startswith("--")]
OPT = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
GLB = POS[0]
OUT = POS[1] if len(POS) > 1 else "armado.mp4"
FAST = "--fast" in sys.argv


def load_parts(glb):
    """{componente de primer nivel: {mesh, sub[(malla en coords del ensamble, color)]}}; caché .pkl junto al .glb."""
    cache = glb + ".parts.pkl"
    if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(glb):
        return pickle.load(open(cache, "rb"))
    sc = trimesh.load(glb)
    G = sc.graph
    par = {b: a for a, b, *_ in G.to_edgelist()}
    top = next(b for a, b, *_ in G.to_edgelist() if a == G.base_frame)
    def comp_of(n):
        while n in par and par[n] != top:
            n = par[n]
        return n
    parts = collections.defaultdict(list)
    for n in G.nodes_geometry:
        T, gname = G[n]
        m = sc.geometry[gname].copy(); m.apply_transform(T)
        col = None
        try:
            col = m.visual.material.baseColorFactor
        except Exception:
            try:
                col = m.visual.face_colors[0]
            except Exception:
                pass
        parts[comp_of(n)].append((m, None if col is None else np.array(col)))
    out = {c: dict(mesh=trimesh.util.concatenate([m for m, _ in l]), sub=l) for c, l in parts.items()}
    pickle.dump(out, open(cache, "wb"))
    return out


def servo_parts(sg90_stl, gear_stl, shaft_x=672.6, gear_y=918.5, horn_z=1299.0):
    """SG90 acostado sobre la cremallera y su engrane, en coordenadas del ensamble (mm in, m out)."""
    sg = trimesh.load(sg90_stl)
    v = sg.vertices                      # SG90: alto en +y (base 6.2, cara del horn 38.2), grosor en z
    col = v[(v[:, 1] > 29.5) & (v[:, 1] < 33.5)]   # spline del eje
    xs = (col[:, 0].min() + col[:, 0].max()) / 2
    zb = (v[:, 2].min() + v[:, 2].max()) / 2
    R = np.eye(4); R[:3, :3] = np.column_stack([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])   # x->-X, y(eje)->+Z, z->+Y
    T0 = np.eye(4); T0[:3, 3] = [-xs, -v[:, 1].max(), -zb]
    T1 = np.eye(4); T1[:3, 3] = [shaft_x, gear_y, horn_z]
    sg.apply_transform(T1 @ R @ T0)
    g = trimesh.load(gear_stl)
    c = (g.bounds[0] + g.bounds[1]) / 2
    Tg = np.eye(4); Tg[:3, 3] = [shaft_x - c[0], gear_y - c[1], horn_z - g.bounds[0][2]]
    g.apply_transform(Tg)
    for m in (sg, g):
        m.apply_scale(0.001)
    return sg, g


P = load_parts(GLB)
if "servo" in OPT and "gear" in OPT:
    _sg, _g = servo_parts(OPT["servo"], OPT["gear"])
    P["SG90"] = dict(mesh=_sg, sub=[(_sg, None)])
    P["ServoGearDirection_V2"] = dict(mesh=_g, sub=[(_g, None)])
def lego_axle(x0, x1, y, z):
    """Eje LEGO (perfil en cruz de 4.8 mm) a lo largo de x, en mm del ensamble; regresa metros."""
    L = x1 - x0
    m = trimesh.util.concatenate([trimesh.creation.box([L, 4.8, 1.8]), trimesh.creation.box([L, 1.8, 4.8])])
    m.apply_translation([(x0 + x1) / 2, y, z]); m.apply_scale(0.001)
    return m


# Ejes LEGO de las ruedas (no están en el CAD): atrás uno de 8 (64 mm) por llanta, entrando al diferencial por cada lado;
# adelante uno de 4 (32 mm) por mangueta.
_b = {n: P[n]["mesh"].bounds * 1000 for n in ("RuedaAtras-1", "RuedaAtras-2", "RuedaEnfrente-1", "RuedaEnfrente-2")}
_c = lambda b: ((b[0][1] + b[1][1]) / 2, (b[0][2] + b[1][2]) / 2)
_xl, _xr = _b["RuedaAtras-1"][0][0], _b["RuedaAtras-2"][1][0]
_xm = (_xl + _xr) / 2
for n, m in (("EjeTrasero-1", lego_axle(_xm - 65, _xm - 1, *_c(_b["RuedaAtras-1"]))),
             ("EjeTrasero-2", lego_axle(_xm + 1, _xm + 65, *_c(_b["RuedaAtras-2"]))),
             ("EjeDelantero-1", lego_axle(_b["RuedaEnfrente-1"][0][0] + 2, _b["RuedaEnfrente-1"][0][0] + 34, *_c(_b["RuedaEnfrente-1"]))),
             ("EjeDelantero-2", lego_axle(_b["RuedaEnfrente-2"][1][0] - 34, _b["RuedaEnfrente-2"][1][0] - 2, *_c(_b["RuedaEnfrente-2"])))):
    P[n] = dict(mesh=m, sub=[(m, None)])

# Tornillos M3x12 (tornillos.json, de tornillos.py): cabeza Allen 5.5 x 3 mm hacia afuera, cuerpo de 12 mm hacia adentro.
SCREWS = []
_tj = OPT.get("tornillos", os.path.join(os.path.dirname(os.path.abspath(__file__)), "tornillos.json"))
if os.path.exists(_tj):
    for k, sc_ in enumerate(json.load(open(_tj))):
        h, o = np.array(sc_["head"]), np.array(sc_["out"])
        A = trimesh.geometry.align_vectors([0, 0, 1], o)
        parts_ = []
        for r, L, c in ((2.75, 3.0, h + o * 1.5), (1.5, 12.0, h - o * 6.0)):
            T = A.copy(); T[:3, 3] = c
            parts_.append(trimesh.creation.cylinder(radius=r, height=L, sections=20, transform=T))
        m = trimesh.util.concatenate(parts_); m.apply_scale(0.001)
        n = f"M3-{k:02d}"
        P[n] = dict(mesh=m, sub=[(m, None)])
        SCREWS.append((n, o, sc_["step_part"]))

allb = np.vstack([p["mesh"].bounds for p in P.values()])
C = (allb.min(0) + allb.max(0)) / 2
U, R, F = np.array([0, 1., 0]), np.array([1., 0, 0]), np.array([0, 0, -1.])   # arriba, derecha(x), adelante(-z)
def side(n):
    b = P[n]["mesh"].bounds; return np.sign((b[0][0] + b[1][0]) / 2 - C[0]) * R
# (título, [(componente, dirección de llegada)])
UP, DN, FW, BK = U, -U, F, -F
STEPS = [
    ("Chassis base", "BaseChasisMain_V2, printed in PLA", [("BaseChasisMain_V2-1", UP)]),
    ("Steering", "knuckles, rack, tie rods, SG90 servo and its gear", [("SteeringKnuckle-1", "S"), ("SteeringKnuckle-2", "S"),
        ("NewCremallera_V2-1", FW), ("NewLinkajeL_V2-1", "S"), ("NewLinkajeR_V2-1", "S"),
        ("SG90", UP), ("ServoGearDirection_V2", BK)]),
    ("Front wheels", "4-stud LEGO axles in the knuckles, 2 front wheels", [("EjeDelantero-1", "S"), ("EjeDelantero-2", "S"),
        ("RuedaEnfrente-1", "S"), ("RuedaEnfrente-2", "S")]),
    ("Drivetrain", "motor mount, N20 motor, 15T pinion and differential", [("DCsupport_V2-1", DN), ("N20 Mini Micro Metall Gear Motor.step-1", BK),
        ("Pinion.15_V2-1", BK), ("Differential_V2-1", BK)]),
    ("Rear wheels", "one 8-stud LEGO axle per wheel into the differential, 2 rear wheels", [("EjeTrasero-1", "S"), ("EjeTrasero-2", "S"),
        ("RuedaAtras-1", "S"), ("RuedaAtras-2", "S")]),
    ("Battery", "supports, battery in the center tunnel, bottom cover", [("BatterySupport-1", "S"), ("BatterySupport-2", "S"),
        ("pila-1", DN), ("TapaBaseChasis-1", DN)]),
    ("Ultrasonics", "supports and 3 HC-SR04 (cross-check for the ToF sensors)", [("UltrasonicSupportFront-1", FW), ("HC-SR04-UltrasonicSensor v7.step-2", FW),
        ("UltrasonicSupportHalf_V2-1", "S"), ("HC-SR04-UltrasonicSensor v7.step-1", "S"),
        ("UltrasonicSupport_V2-2", "S"), ("HC-SR04-UltrasonicSensor v7.step-3", "S")]),
    ("Top plate", "TapaChasis_V2", [("TapaChasis_V2-1", UP)]),
    ("ToF sensors", "support and 4 TOF400C (VL53L1X): left, right, front and rear", [("ToFSupport-1", UP), ("TOF400C.step-1", "S"), ("TOF400C.step-2", "S"),
        ("TOF400C.step-3", FW), ("ToFBack-1", BK), ("TOF400C.step-4", BK)]),
    ("Raspberry Pi 5", "vision and route planner", [("RASPBERRY_PI_5.STEP", UP)]),
    ("PCB RevB", "power board and ESP32 board (rear stack)", [("WRO_RevB_2.step", UP), ("WRO_RevB_1.step", UP)]),
    ("Camera", "camera base and front mount", [("CameraBase-1", UP), ("CameraFrnt-1", FW + 0.6 * UP)]),
]
DIST = 0.075
STEPS = [(t, sub, [(n, d, False) for n, d in l if n in P]) for t, sub, l in STEPS]   # sin --servo no hay SG90
# cada tornillo entra en el paso de la última pieza que sujeta; los de un paso llegan juntos, al final del paso
for n, o, sp in SCREWS:
    k = next(i for i, (_, _, l) in enumerate(STEPS) if any(x[0] == sp for x in l))
    STEPS[k][2].append((n, o, True))
STEPS = [(t, sub + (f" + {c} M3×12 screw{'s' if c > 1 else ''}" if (c := sum(b for _, _, b in l)) else ""), l) for t, sub, l in STEPS]
names = [n for _, _, l in STEPS for n, _, _ in l]
missing = set(P) - set(names); assert not missing, missing
OFF = {}
for _, _, l in STEPS:
    for n, d, b in l:
        d = side(n) if isinstance(d, str) else np.asarray(d, float)
        OFF[n] = d / np.linalg.norm(d) * (0.03 if b else DIST)

# Colores por pieza (el CAD deja casi todo en gris). Las placas compradas conservan los suyos.
HEX = lambda h: np.array([(int(h[i:i + 2], 16) / 255) ** 2.2 for i in (0, 2, 4)] + [1.0])   # sRGB -> lineal (si no, salen deslavados)
COLOR = {
    "BaseChasisMain_V2-1": "e5484d", "TapaChasis_V2-1": "e5484d", "TapaBaseChasis-1": "c93a3f",
    "SteeringKnuckle-1": "ff9f1c", "SteeringKnuckle-2": "ff9f1c", "NewCremallera_V2-1": "ffd23f",
    "NewLinkajeL_V2-1": "ff9f1c", "NewLinkajeR_V2-1": "ff9f1c",
    "RuedaEnfrente-1": "2b2d31", "RuedaEnfrente-2": "2b2d31", "RuedaAtras-1": "2b2d31", "RuedaAtras-2": "2b2d31",
    "DCsupport_V2-1": "4c9aff", "Pinion.15_V2-1": "ffd23f", "Differential_V2-1": "ffd23f",
    "BatterySupport-1": "4c9aff", "BatterySupport-2": "4c9aff", "pila-1": "3a7d44",
    "UltrasonicSupportFront-1": "9b5de5", "UltrasonicSupportHalf_V2-1": "9b5de5", "UltrasonicSupport_V2-2": "9b5de5",
    "ToFSupport-1": "00bbf9", "ToFBack-1": "00bbf9",
    "CameraBase-1": "00f5d4", "CameraFrnt-1": "00f5d4",
    "SG90": "2f6fd6", "ServoGearDirection_V2": "ffd23f",
    "EjeTrasero-1": "3a3a3d", "EjeTrasero-2": "3a3a3d", "EjeDelantero-1": "3a3a3d", "EjeDelantero-2": "3a3a3d",
}
# ── escena ──
scene = pyrender.Scene(bg_color=[0.06, 0.07, 0.09, 1], ambient_light=[0.3, 0.3, 0.32])
NODES = {}
for n, p in P.items():
    nodes = []
    for m, col in p["sub"]:
        col = np.array([0.6, 0.6, 0.6, 1]) if col is None else np.array(col, float)
        if col.max() > 1: col = col / 255
        if n in COLOR: col = HEX(COLOR[n])
        elif n.startswith("M3-"): col = HEX("c9ccd3")
        elif n.startswith("N20"): col = np.array([0.78, 0.78, 0.8, 1]) if col[:3].mean() < 0.5 else col   # motor metálico
        mm = m.copy(); mm.apply_translation(-C)
        mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=col, metallicFactor=0.6 if n.startswith("M3-") else 0.05,
                                                 roughnessFactor=0.35 if n.startswith("M3-") else 0.55)
        nodes.append(scene.add(pyrender.Mesh.from_trimesh(mm, material=mat, smooth=False)))
    NODES[n] = nodes
def look(eye, tgt=np.zeros(3)):
    f = tgt - eye; f /= np.linalg.norm(f); r = np.cross(f, U); r /= np.linalg.norm(r); u = np.cross(r, f)
    M = np.eye(4); M[:3, 0] = r; M[:3, 1] = u; M[:3, 2] = -f; M[:3, 3] = eye; return M
camn = scene.add(pyrender.PerspectiveCamera(yfov=np.radians(32), aspectRatio=W / H))
lk = scene.add(pyrender.DirectionalLight(intensity=3.2))
lf = scene.add(pyrender.DirectionalLight(intensity=1.4))
rend = pyrender.OffscreenRenderer(W, H)
fb = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 40); fs = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 24)
fx = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)

def ease(u): u = min(1, max(0, u)); return u * u * (3 - 2 * u)
def frame(state, az, dist, title="", sub="", step=None):
    """state: {comp: (visible, offset_factor)}"""
    for n, nodes in NODES.items():
        vis, k = state.get(n, (False, 1.0))
        T = np.eye(4); T[:3, 3] = OFF[n] * k
        for nd in nodes:
            nd.mesh.is_visible = vis; scene.set_pose(nd, T)
    el = math.radians(28)
    eye = dist * np.array([math.sin(az) * math.cos(el), math.sin(el), math.cos(az) * math.cos(el)])
    tgt = np.array([0, 0.008, 0])   # el carro un poco abajo del centro: arriba va el texto
    scene.set_pose(camn, look(eye + tgt, tgt))
    scene.set_pose(lk, look(eye * 1.0 + 0.25 * U * dist + 0.2 * R * dist, tgt))
    scene.set_pose(lf, look(-eye + 0.3 * U * dist, tgt))
    col, _ = rend.render(scene)
    im = Image.fromarray(col); d = ImageDraw.Draw(im)
    if step is not None:
        d.text((40, 30), f"Step {step + 1}/{len(STEPS)}", font=fs, fill=(76, 154, 255))
    if title: d.text((40, 58), title, font=fb, fill=(235, 238, 242))
    if sub: d.text((42, 108), sub, font=fs, fill=(150, 160, 175))
    d.text((40, H - 38), "Fox Robotics · WRO 2026 Future Engineers · v2 car (models/v2)", font=fx, fill=(110, 118, 130))
    return np.asarray(im)

wr = imageio.get_writer(OUT, fps=FPS, codec="libx264", quality=8, macro_block_size=8)
az0, AZV = math.radians(180 + 40), math.radians(-7)   # giro lento de la cámara (°/s)
t = 0.0
def emit(*a, **k):
    global t
    wr.append_data(frame(*a, az=az0 + AZV * t, **k)); t += 1 / FPS
sec = lambda s: range(int((s if not FAST else s / 4) * FPS))
D0, D1 = 0.37, 0.62
full = {n: (True, 0.0) for n in names}
for i in sec(2.5): emit(full, dist=D0, title="v2 car", sub="how it goes together, part by part")
N = len(sec(1.8))
for i in range(N):
    u = ease(i / (N - 1)); emit({n: (True, 1.0 * u) for n in names}, dist=D0 + (D1 - D0) * u, title="Exploded view", sub=f"{len(names)} components")
for i in sec(2.5): emit({n: (True, 1.0) for n in names}, dist=D1, title="Exploded view", sub=f"{len(names)} components")
N = len(sec(0.8))   # se vuelve a juntar y se va: empieza el armado desde cero
for i in range(N):
    u = ease(i / (N - 1)); emit({n: (True, 1.0 - u) for n in names}, dist=D1 + (D0 - D1) * u, title="Exploded view", sub="now, step by step")
done = {}
for si, (title, sub, lst) in enumerate(STEPS):
    stag, mv, hold = 0.25, 0.9, 0.7
    npart = sum(not b for _, _, b in lst)
    slots = [j if not b else npart + 0.6 for j, (_, _, b) in enumerate(lst)]   # tornillos: una tanda al final
    T = stag * max(slots) + mv + hold
    N = len(sec(T)); TT = T if not FAST else T / 4
    for i in range(N):
        tt = i / FPS * (TT / (N / FPS))
        st = dict(done)
        for (n, _, b), sl in zip(lst, slots):
            s0 = sl * stag * (TT / T)
            u = ease((tt - s0) / (mv * TT / T))
            st[n] = (tt >= s0, (1.6 if b else 2.2) * (1 - u))
        emit(st, dist=D0, title=title, sub=sub, step=si)
    for n, _, _ in lst: done[n] = (True, 0.0)
for i in sec(4): emit(full, dist=D0, title="Done", sub=f"12 steps · {len(SCREWS)} M3×12 screws · see models/v2/README.md")
wr.close()
print("ok", OUT, round(t, 1), "s")
