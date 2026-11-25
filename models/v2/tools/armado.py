"""Video de armado del carro v2 (models/v2/renders/armado.mp4): el ensamble gira, se despieza y se arma
en 12 pasos, con colores por pieza (el CAD deja casi todo en gris).

Entrada: el ensamble exportado de SolidWorks a .glb (Vehicle_V2.glb, ~50 MB, no está en el repo:
Archivo > Guardar como > glTF binario desde cad/Vehicle_V2.SLDASM).
  pip install trimesh pyrender imageio imageio-ffmpeg pillow
  python models/v2/tools/armado.py Vehicle_V2.glb models/v2/renders/armado.mp4 [--fast]
      [--servo=sg90.stl --gear=ServoGearDirection_V2.STL]
--fast: 4x más rápido, para revisar.
El servo SG90 y su engrane no están en Vehicle_V2.SLDASM: con --servo/--gear se agregan en su lugar
(acostado sobre la cremallera, eje hacia atrás, orejas en los dos postes; engrane sobre los dientes, que miran
arriba). sg90.stl = el STEP del SG90 pasado a STL (FreeCAD: MeshPart.meshFromShape). Render en una GPU de laptop: ~1.5 min. Arriba = +Y, adelante = -Z.
"""
import pickle, sys, math, collections, os, numpy as np, pyrender, imageio, trimesh
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
allb = np.vstack([p["mesh"].bounds for p in P.values()])
C = (allb.min(0) + allb.max(0)) / 2
U, R, F = np.array([0, 1., 0]), np.array([1., 0, 0]), np.array([0, 0, -1.])   # arriba, derecha(x), adelante(-z)
def side(n):
    b = P[n]["mesh"].bounds; return np.sign((b[0][0] + b[1][0]) / 2 - C[0]) * R
# (título, [(componente, dirección de llegada)])
UP, DN, FW, BK = U, -U, F, -F
STEPS = [
    ("Chasis base", "BaseChasisMain_V2 impreso en PLA", [("BaseChasisMain_V2-1", UP)]),
    ("Dirección", "manguetas, cremallera, bieletas, servo SG90 y su engrane", [("SteeringKnuckle-1", "S"), ("SteeringKnuckle-2", "S"),
        ("NewCremallera_V2-1", FW), ("NewLinkajeL_V2-1", "S"), ("NewLinkajeR_V2-1", "S"),
        ("SG90", UP), ("ServoGearDirection_V2", BK)]),
    ("Ruedas delanteras", "RuedaEnfrente ×2", [("RuedaEnfrente-1", "S"), ("RuedaEnfrente-2", "S")]),
    ("Tracción", "soporte del motor, N20, piñón 15T y diferencial", [("DCsupport_V2-1", DN), ("N20 Mini Micro Metall Gear Motor.step-1", BK),
        ("Pinion.15_V2-1", BK), ("Differential_V2-1", BK)]),
    ("Ruedas traseras", "RuedaAtras ×2", [("RuedaAtras-1", "S"), ("RuedaAtras-2", "S")]),
    ("Batería", "soportes, pila en el túnel central y tapa inferior", [("BatterySupport-1", "S"), ("BatterySupport-2", "S"),
        ("pila-1", DN), ("TapaBaseChasis-1", DN)]),
    ("Ultrasónicos", "soportes y 3 HC-SR04 (verificación cruzada de los ToF)", [("UltrasonicSupportFront-1", FW), ("HC-SR04-UltrasonicSensor v7.step-2", FW),
        ("UltrasonicSupportHalf_V2-1", "S"), ("HC-SR04-UltrasonicSensor v7.step-1", "S"),
        ("UltrasonicSupport_V2-2", "S"), ("HC-SR04-UltrasonicSensor v7.step-3", "S")]),
    ("Tapa del chasis", "TapaChasis_V2", [("TapaChasis_V2-1", UP)]),
    ("Sensores ToF", "soporte y 4 TOF400C (VL53L1X): izquierda, derecha, frente y atrás", [("ToFSupport-1", UP), ("TOF400C.step-1", "S"), ("TOF400C.step-2", "S"),
        ("TOF400C.step-3", FW), ("ToFBack-1", BK), ("TOF400C.step-4", BK)]),
    ("Raspberry Pi 5", "visión y planificador de ruta", [("RASPBERRY_PI_5.STEP", UP)]),
    ("PCB RevB", "placa de potencia y placa del ESP32 (stack trasero)", [("WRO_RevB_2.step", UP), ("WRO_RevB_1.step", UP)]),
    ("Cámara", "base y soporte frontal de la cámara", [("CameraBase-1", UP), ("CameraFrnt-1", FW + 0.6 * UP)]),
]
DIST = 0.075
STEPS = [(t, sub, [(n, d) for n, d in l if n in P]) for t, sub, l in STEPS]   # sin --servo no hay SG90
names = [n for _, _, l in STEPS for n, _ in l]
missing = set(P) - set(names); assert not missing, missing
OFF = {}
for _, _, l in STEPS:
    for n, d in l:
        d = side(n) if isinstance(d, str) else d
        OFF[n] = d / np.linalg.norm(d) * DIST

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
        elif n.startswith("N20"): col = np.array([0.78, 0.78, 0.8, 1]) if col[:3].mean() < 0.5 else col   # motor metálico
        mm = m.copy(); mm.apply_translation(-C)
        mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=col, metallicFactor=0.05, roughnessFactor=0.55)
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
    tgt = np.array([0, -0.004, 0])
    scene.set_pose(camn, look(eye + tgt, tgt))
    scene.set_pose(lk, look(eye * 1.0 + 0.25 * U * dist + 0.2 * R * dist, tgt))
    scene.set_pose(lf, look(-eye + 0.3 * U * dist, tgt))
    col, _ = rend.render(scene)
    im = Image.fromarray(col); d = ImageDraw.Draw(im)
    if step is not None:
        d.text((40, 30), f"Paso {step + 1}/{len(STEPS)}", font=fs, fill=(76, 154, 255))
    if title: d.text((40, 58), title, font=fb, fill=(235, 238, 242))
    if sub: d.text((42, 108), sub, font=fs, fill=(150, 160, 175))
    d.text((40, H - 38), "Fox Robotics · WRO 2026 Future Engineers · carro v2 (models/v2)", font=fx, fill=(110, 118, 130))
    return np.asarray(im)

wr = imageio.get_writer(OUT, fps=FPS, codec="libx264", quality=8, macro_block_size=8)
az0, AZV = math.radians(180 + 40), math.radians(-7)   # giro lento de la cámara (°/s)
t = 0.0
def emit(*a, **k):
    global t
    wr.append_data(frame(*a, az=az0 + AZV * t, **k)); t += 1 / FPS
sec = lambda s: range(int((s if not FAST else s / 4) * FPS))
D0, D1 = 0.42, 0.62
full = {n: (True, 0.0) for n in names}
for i in sec(2.5): emit(full, dist=D0, title="Carro v2", sub="cómo se arma, pieza por pieza")
N = len(sec(1.8))
for i in range(N):
    u = ease(i / (N - 1)); emit({n: (True, 1.0 * u) for n in names}, dist=D0 + (D1 - D0) * u, title="Despiece", sub=f"{len(names)} componentes")
for i in sec(2.5): emit({n: (True, 1.0) for n in names}, dist=D1, title="Despiece", sub=f"{len(names)} componentes")
N = len(sec(0.8))   # se vuelve a juntar y se va: empieza el armado desde cero
for i in range(N):
    u = ease(i / (N - 1)); emit({n: (True, 1.0 - u) for n in names}, dist=D1 + (D0 - D1) * u, title="Despiece", sub="ahora, paso por paso")
done = {}
for si, (title, sub, lst) in enumerate(STEPS):
    stag, mv, hold = 0.25, 0.9, 0.7
    T = stag * (len(lst) - 1) + mv + hold
    N = len(sec(T)); TT = T if not FAST else T / 4
    for i in range(N):
        tt = i / FPS * (TT / (N / FPS))
        st = dict(done)
        for j, (n, _) in enumerate(lst):
            s0 = j * stag * (TT / T)
            u = ease((tt - s0) / (mv * TT / T))
            st[n] = (tt >= s0, 2.2 * (1 - u))
        emit(st, dist=D0, title=title, sub=sub, step=si)
    for n, _ in lst: done[n] = (True, 0.0)
for i in sec(4): emit(full, dist=D0, title="Listo", sub="12 pasos · ver models/v2/README.md para la lista de piezas")
wr.close()
print("ok", OUT, round(t, 1), "s")
