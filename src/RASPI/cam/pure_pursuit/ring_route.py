"""Ruta de la vuelta en mm del tapete + seguidor pure pursuit por odometría.

Idea: con el mapa de señales conocido, la vuelta entera es una curva fija en
coordenadas del tapete; el carro la sigue con su pose (encoder + IMU), no con
la imagen. La visión solo hace falta para armar el mapa.

Geometría (WRO 2026, reto de obstáculos): pasillo de 1000 mm alrededor de una
isla de 1000×1000. Cada punto de la ruta se describe por una estación de la
vuelta y `d` = distancia a la isla:

* recta (along 1000..2000, lo que mide la isla): punto = cara de la isla + d
  hacia afuera;
* esquina: punto = esquina de la isla + d en la dirección del ángulo (arcos
  concéntricos), así recta y esquina empalman con el mismo d.

Planificador: programación dinámica sobre (estación, d). Estado = (d anterior,
d actual) para poder medir curvatura. Restricciones duras: holgura de la
huella del carro (rectángulo real, eje trasero de referencia) a señales,
paredes, isla y maderas del cajón; radio de giro mínimo; lado de paso de cada
señal (rojo a la derecha, verde a la izquierda). Costo: largo + curvatura² +
castigo por holgura chica. El grafo es acíclico (la estación solo avanza), así
que la DP da el óptimo de la malla.

El punto de referencia del carro es el EJE TRASERO (como el twin y el mapa
digital): con cinemática de bicicleta el eje trasero va tangente a la ruta.
"""

from __future__ import annotations

import ctypes
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np

from .wro_field import (
    INNER_HALF_MM,
    OUTER_HALF_MM,
    SECTIONS_CW,
    SEATS,
    SIGN_MM,
    _barrier_corners,
    _heading,
    section_to_world,
)
from . import config as C

_LANE_MM = OUTER_HALF_MM - INNER_HALF_MM            # 1000
_REF_D = _LANE_MM / 2.0                              # 500: centro del carril
_CORNER_SIGMA = 0.5 * math.pi * _REF_D               # largo de referencia de la esquina
_SEC_SIGMA = _LANE_MM + _CORNER_SIGMA                # recta (1000) + esquina
_CENTERS = {"N": (0.0, 1000.0), "E": (1000.0, 0.0), "S": (0.0, -1000.0), "W": (-1000.0, 0.0)}

# Carro (eje trasero = 0): adelante 147.6, atrás 21.8, medio ancho 66.
_FRONT_MM = C.ROBOT_LENGTH_MM - C.REAR_OVERHANG_MM
_REAR_MM = C.REAR_OVERHANG_MM
_HALF_W = C.ROBOT_WIDTH_MM / 2.0


@dataclass
class RouteParams:
    n_stations: int = 96            # estaciones por vuelta (~75 mm de referencia)
    d_min: float = 100.0            # rango de d de la malla
    d_max: float = 900.0
    d_step: float = 20.0
    hard_clear_mm: float = 55.0     # holgura mínima de la huella (duro)
    soft_clear_mm: float = 140.0    # debajo de esto se castiga
    r_min_mm: float = 170.0         # radio mínimo (duro), eje trasero
    w_len: float = 1.0              # costo por mm recorrido
    w_kappa: float = 3.0e4          # por mm · κ² (4e5 abría las esquinas: 38/42 limpias vs 42/42)
    w_clear: float = 8.0            # por mm · ((soft - clear)/soft)²
    # Costo por tiempo (False: largo + curvatura, como antes). Con True, el
    # término de largo se multiplica por v_max/v(κ) con el modelo del perfil.
    time_cost: bool = False
    v_min_mm_s: float = 455.0       # piso de velocidad (ROUTE_PROFILE_BASE_PWM)
    v_max_mm_s: float = 805.0       # techo de velocidad en recta (ROUTE_PROFILE_MAX_PWM)
    a_lat_mm_s2: float = 1500.0     # aceleración lateral máxima del perfil
    max_angle_deg: float = 55.0     # tramo contra la dirección del pasillo (duro)
    # Asiento sin ver: pasar pegado a él (por cualquier lado) para que, si al
    # revelarse toca el otro lado, el cambio sea de ~300 mm y no de ~550.
    w_unknown_near: float = 1.5     # por mm · (exceso lateral / 100 mm)
    unknown_near_mm: float = 180.0  # distancia centro-centro sin castigo
    # Premio por estación con un asiento sin ver dentro de la vista de la cámara.
    w_visible: float = 60.0
    view_min_mm: float = 250.0
    view_max_mm: float = 550.0
    view_half_deg: float = 40.0
    step_mm: float = 10.0           # muestreo de la ruta final


def _order(direction: str, start: str = "N") -> list[str]:
    seq = list(SECTIONS_CW) if direction == "CW" else ["N", "W", "S", "E"]
    i = seq.index(start)
    return seq[i:] + seq[:i]


@dataclass
class _Sec:
    name: str
    f: np.ndarray        # adelante (sentido de carrera)
    n_in: np.ndarray     # hacia la isla
    c0: np.ndarray       # centro del carril en along = 0 (pared de atrás)

    def frame(self, x, y):
        """(along, d) de un punto del mundo."""
        p = np.array([x, y], dtype=float) - self.c0
        u = float(p @ self.f)
        lat_in = float(p @ self.n_in)          # hacia la isla, desde el centro del carril
        return u, _REF_D - lat_in


def _sec(name: str, direction: str) -> _Sec:
    h = math.radians(_heading(name, direction))
    f = np.array([math.sin(h), math.cos(h)])
    r = np.array([math.cos(h), -math.sin(h)])            # derecha del sentido
    n_in = r if direction == "CW" else -r                # CW: la isla queda a la derecha
    c = np.array(_CENTERS[name])
    return _Sec(name, f, n_in, c - f * OUTER_HALF_MM)


def _stations(secs: list[_Sec], n: int):
    """Origen O_k y dirección v_k (afuera de la isla) por estación: P = O + d·v."""
    sig = np.arange(n) * (4.0 * _SEC_SIGMA / n)
    O = np.zeros((n, 2))
    V = np.zeros((n, 2))
    sec_idx = np.zeros(n, dtype=int)
    loc_all = np.zeros(n)
    for i, s in enumerate(sig):
        k = int(s // _SEC_SIGMA) % 4
        loc = s - (s // _SEC_SIGMA) * _SEC_SIGMA
        sec = secs[k]
        sec_idx[i] = k
        loc_all[i] = loc
        if loc < _LANE_MM:
            face = sec.c0 + sec.f * (1000.0 + loc) + sec.n_in * _REF_D
            O[i] = face
            V[i] = -sec.n_in
        else:
            th = (loc - _LANE_MM) / _CORNER_SIGMA * (0.5 * math.pi)
            corner = sec.c0 + sec.f * 2000.0 + sec.n_in * _REF_D
            O[i] = corner
            V[i] = -sec.n_in * math.cos(th) + sec.f * math.sin(th)
    return sig, O, V, sec_idx, loc_all


@dataclass
class Route:
    x: np.ndarray
    y: np.ndarray
    heading_deg: np.ndarray      # 0 = +y, 90 = +x (igual que el twin)
    kappa: np.ndarray            # 1/mm, + = a la derecha
    s: np.ndarray                # largo acumulado (mm)
    length_mm: float
    direction: str
    closed: bool = True
    report: dict = field(default_factory=dict)

    def xy(self) -> np.ndarray:
        return np.stack([self.x, self.y], axis=1)


def signs_from_field(field) -> dict:
    return {(s.section, s.seat): s.color for s in field.signs}


def _rect_dist_pose(px, py, x, y, hd_rad, fx=None, fy=None):
    """Distancia de puntos (m,) a huellas (…,) del carro; broadcast: x,y,hd (...), puntos (m).
    fx, fy: sin y cos de hd_rad ya calculados (opcional)."""
    if fx is None:
        fx, fy = np.sin(hd_rad), np.cos(hd_rad)
    dx = px - x[..., None]
    dy = py - y[..., None]
    lon = dx * fx[..., None] + dy * fy[..., None]
    lat = dx * fy[..., None] - dy * fx[..., None]
    lon_c = (_FRONT_MM - _REAR_MM) / 2.0
    hl = (_FRONT_MM + _REAR_MM) / 2.0
    qx = np.abs(lon - lon_c) - hl
    qy = np.abs(lat) - _HALF_W
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)


def _corners_pose(x, y, hd_rad, fx=None, fy=None):
    if fx is None:
        fx, fy = np.sin(hd_rad), np.cos(hd_rad)
    rx, ry = fy, -fx
    out = []
    for lon, lat in ((-_REAR_MM, -_HALF_W), (-_REAR_MM, _HALF_W), (_FRONT_MM, _HALF_W), (_FRONT_MM, -_HALF_W)):
        out.append((x + fx * lon + rx * lat, y + fy * lon + ry * lat))
    return out


def _square_pts(cx, cy, half, n=5):
    t = np.linspace(-half, half, n)
    pts = [(cx + a, cy - half) for a in t] + [(cx + a, cy + half) for a in t]
    pts += [(cx - half, cy + a) for a in t[1:-1]] + [(cx + half, cy + a) for a in t[1:-1]]
    return np.array(pts)


_STATIC_CACHE: dict = {}
_GEOM_CACHE: dict = {}
_RANGE_CACHE: dict = {}            # gkey -> (ALO, AHI) para fox_route_dp_sparse
_SIGN_CACHE: dict = {}             # (gridkey, cx, cy) -> (ks, dsg) de la señal
_VIS_CACHE: dict = {}              # (gridkey, ux, uy, ...) -> (kv, vis) de la cámara
_NATIVE_DP = True                  # False: bucle de la DP en numpy (sin fox_route_dp)


def _station_poses(P):
    """x, y, rumbo de cada (estación, d anterior, d actual): (n, m, m)."""
    prev = np.roll(P, 1, axis=0)
    dx = P[:, None, :, 0] - prev[:, :, None, 0]
    dy = P[:, None, :, 1] - prev[:, :, None, 1]
    hd = np.arctan2(dx, dy)
    xx = np.broadcast_to(P[:, None, :, 0], hd.shape)
    yy = np.broadcast_to(P[:, None, :, 1], hd.shape)
    return xx, yy, hd


def _static_clearance(x, y, hd_rad, obst_pts, fx=None, fy=None):
    """Holgura mínima de la huella en las poses dadas (arrays de igual forma)."""
    if fx is None:
        fx, fy = np.sin(hd_rad), np.cos(hd_rad)
    cs = _corners_pose(x, y, hd_rad, fx, fy)
    wall = OUTER_HALF_MM - np.max([np.maximum(np.abs(cx), np.abs(cy)) for cx, cy in cs], axis=0)
    isl = np.full(x.shape, np.inf)
    for cx, cy in cs:
        ox = np.maximum(np.abs(cx) - INNER_HALF_MM, 0.0)
        oy = np.maximum(np.abs(cy) - INNER_HALF_MM, 0.0)
        inside = (np.abs(cx) < INNER_HALF_MM) & (np.abs(cy) < INNER_HALF_MM)
        d = np.where(inside, -1.0, np.hypot(ox, oy))
        isl = np.minimum(isl, d)
    icorner = np.array([[INNER_HALF_MM * sx, INNER_HALF_MM * sy] for sx in (-1, 1) for sy in (-1, 1)])
    isl = np.minimum(isl, _rect_dist_pose(icorner[:, 0], icorner[:, 1], x, y, hd_rad, fx, fy).min(-1))
    clear = np.minimum(wall, isl)
    if obst_pts is not None and len(obst_pts):
        clear = np.minimum(clear, _rect_dist_pose(obst_pts[:, 0], obst_pts[:, 1], x, y, hd_rad, fx, fy).min(-1))
    return clear


def _dp_geometry(P, prm, side_ok):
    """Por estación k1: G = wl + w_κ·κ² (n,m,m,m), DS = |bc| (n,m,m) y
    MASK = κ fuera de rango o lado no permitido (n,m,m,m). No ve obstáculos."""
    n, m = P.shape[0], P.shape[1]
    kmax = 1.0 / prm.r_min_mm
    kg = _geom_kernel() if _NATIVE_DP else None
    if kg is not None:
        # Mismo cálculo en C (fox_route_geom): mismo orden de operaciones, bit a bit.
        Pc = np.ascontiguousarray(P, dtype=np.float64)
        sok = np.ascontiguousarray(side_ok).view(np.uint8)
        G = np.empty((n, m, m, m))
        DS = np.empty((n, m, m))
        MASK = np.empty((n, m, m, m), dtype=np.uint8)
        kg(n, m, Pc.ctypes.data, sok.ctypes.data, int(prm.time_cost), prm.w_len, prm.w_kappa,
           prm.a_lat_mm_s2, prm.v_min_mm_s, prm.v_max_mm_s, kmax,
           G.ctypes.data, DS.ctypes.data, MASK.ctypes.data)
        return G, DS, MASK.view(bool)
    G = np.empty((n, m, m, m))
    DS = np.empty((n, m, m))
    MASK = np.empty((n, m, m, m), dtype=bool)
    for k1 in range(n):
        k0, k2 = (k1 - 1) % n, (k1 + 1) % n
        A = P[k0][:, None, None, :]
        B = P[k1][None, :, None, :]
        Cc = P[k2][None, None, :, :]
        ab = B - A
        bc = Cc - B
        ac = Cc - A
        cross = ab[..., 0] * bc[..., 1] - ab[..., 1] * bc[..., 0]
        lab = np.hypot(ab[..., 0], ab[..., 1])
        lbc = np.hypot(bc[..., 0], bc[..., 1])
        lac = np.hypot(ac[..., 0], ac[..., 1])
        kap = 2.0 * cross / np.maximum(lab * lbc * lac, 1e-9)       # (m,m,m)
        if prm.time_cost:
            # Costo de tiempo: el tramo cuesta w_len·ds·(v_max/v(κ)), con v(κ) del
            # perfil (sqrt(a_lat/|κ|) acotado). En recta el factor es 1.
            v_k = np.clip(np.sqrt(prm.a_lat_mm_s2 / np.maximum(np.abs(kap), 1e-9)), prm.v_min_mm_s, prm.v_max_mm_s)
            wl = prm.w_len * (prm.v_max_mm_s / v_k)                 # (m,m,m)
        else:
            wl = prm.w_len
        G[k1] = wl + prm.w_kappa * kap * kap
        DS[k1] = lbc[0]
        MASK[k1] = (np.abs(kap) > kmax) | ~side_ok[k1][None, :, :]
    return G, DS, MASK


_KERNELS: dict = {}                # (nombre, FOX_NATIVE) -> función nativa o None


def _native_fn(name, argtypes):
    """Kernel nativo por nombre, cacheado por FOX_NATIVE (la carga cuesta ~4 ms por llamada)."""
    try:
        from .native import enabled, get_kernel
        key = (name, enabled())
        if key not in _KERNELS:
            _KERNELS[key] = get_kernel(name, argtypes, None)
        return _KERNELS[key]
    except Exception:
        return None


def _dp_kernel():
    """fox_route_dp de la biblioteca nativa, o None si no está disponible."""
    i32, vp = ctypes.c_int32, ctypes.c_void_p
    return _native_fn("fox_route_dp", [i32, i32, i32] + [vp] * 10)


def _dp_sparse_kernel():
    """fox_route_dp_sparse de la biblioteca nativa, o None si no está disponible."""
    i32, vp = ctypes.c_int32, ctypes.c_void_p
    return _native_fn("fox_route_dp_sparse", [i32, i32, i32] + [vp] * 12)


def _geom_kernel():
    """fox_route_geom de la biblioteca nativa, o None si no está disponible."""
    i32, vp, f64 = ctypes.c_int32, ctypes.c_void_p, ctypes.c_double
    return _native_fn("fox_route_geom", [i32, i32, vp, vp, i32] + [f64] * 6 + [vp] * 3)


def plan(direction: str, seats: dict, parking: str | None = None,
         prm: RouteParams | None = None, start_section: str | None = None,
         laps: int = 3, unknown=None, anchor=None, lock=None,
         park_w: float | None = None, extra=None) -> Route:
    """seats: {(sección, asiento): "Red"|"Green"} en el marco del tapete.

    park_w: w0 del cajón en su recta (None = el del programa oficial).
    extra: obstáculos sueltos [(x, y, "Red"|"Green"|None)] fuera de los
    asientos (arranque ciego: la señal vista antes de saber dónde está el
    carro, o la fila interior de la recta del cajón como cerca). Con color
    llevan regla de lado; sin color, solo holgura.

    unknown: asientos todavía sin ver (vuelta 1): obstáculo sin lado de paso,
    la ruta no pasa por donde podría haber una señal.
    anchor: (x, y[, rumbo]) del carro: la ruta pasa por ahí (±30 mm) y, con
    rumbo, sale en esa dirección. Para el primer plan.
    lock: {estación: d} de la ruta anterior delante del carro: la ruta nueva
    empata EXACTO ahí (posición y rumbo) y solo cambia después; mientras sube
    la ruta nueva al ESP el carro sigue por ese tramo de todos modos.

    Devuelve UNA vuelta cerrada (la vuelta del medio de una DP de `laps` vueltas
    desenrolladas, para que el empalme del cierre sea el régimen permanente).
    """
    t0 = time.perf_counter()
    prm = prm or RouteParams()
    order = _order(direction, start_section or "N")
    secs = [_sec(nm, direction) for nm in order]
    n = prm.n_stations
    sig, O, V, sec_idx, loc = _stations(secs, n)
    D = np.arange(prm.d_min, prm.d_max + 0.5, prm.d_step)
    m = len(D)
    P = O[:, None, :] + D[None, :, None] * V[:, None, :]          # (n, m, 2)

    # Obstáculos puntuales: señales y maderas.
    pillars = []
    side_rules = []          # (estación, máscara de d permitidos)
    for (sname, seat), color in seats.items():
        if sname not in order or seat not in SEATS:
            continue
        h, w = SEATS[seat]
        px, py = section_to_world(sname, h, w)
        pillars.append((px, py, color))
        sec = secs[order.index(sname)]
        u, d_p = sec.frame(px, py)
        k = order.index(sname)
        st = int(round((k * _SEC_SIGMA + (u - 1000.0)) / (4.0 * _SEC_SIGMA / n))) % n
        right_is_in = direction == "CW"
        go_in = (color == "Red") == right_is_in            # pasar del lado de la isla
        side_rules.append((st, D < d_p if go_in else D > d_p))
    extra_centers = []
    for ex, ey, color in (extra or ()):
        if color is None:
            extra_centers.append((ex, ey))
            continue
        pillars.append((ex, ey, color))
        best = None
        for k_s, sec in enumerate(secs):
            u, d_p = sec.frame(ex, ey)
            if -100.0 <= u <= 3100.0 and -100.0 <= d_p <= 1100.0 and 700.0 <= u <= 2300.0:   # el marco supuesto del arranque ciego corre la fila hasta ±~700 mm; con 1000..2000 se perdía el lado
                best = (k_s, u, d_p)
                break
        if best is None:
            continue
        k_s, u, d_p = best
        st = int(round((k_s * _SEC_SIGMA + (u - 1000.0)) / (4.0 * _SEC_SIGMA / n))) % n
        go_in = (color == "Red") == (direction == "CW")
        side_rules.append((st, D < d_p if go_in else D > d_p))
    # Holgura por (estación k, d anterior j, d actual i): rumbo = P[k,i] - P[k-1,j].
    # Paredes, isla y maderas no cambian en la carrera: caché por (sentido,
    # cajón, malla). Las señales (conocidas o posibles) se suman encima, solo
    # en las estaciones cercanas, como círculo de la diagonal de la señal
    # (conservador: hasta 10 mm de más contra una cara).
    xx, yy, hd = _station_poses(P)
    shd, chd = np.sin(hd), np.cos(hd)
    # La malla (O, P) solo depende de sentido, recta de salida y rango de d: paredes,
    # isla, señales y vista se cachean por malla; las maderas del cajón aparte, porque
    # park_w cambia al fijar el arranque (antes eso recalculaba todo: ~45 ms).
    gridkey = (direction, start_section or "N", n, prm.d_min, prm.d_max, prm.d_step)
    ckey = gridkey + (parking, park_w)
    clear = _STATIC_CACHE.get(ckey)
    if clear is None:
        base = _STATIC_CACHE.get(gridkey)
        if base is None:
            base = np.empty((n, m, m))
            for k in range(n):
                base[k] = _static_clearance(xx[k], yy[k], hd[k], None, shd[k], chd[k])
            _STATIC_CACHE[gridkey] = base
        clear = base.copy()
        if parking is not None:
            bpts = []
            for box in _barrier_corners(parking, *(() if park_w is None else (park_w,))):
                xs = [p[0] for p in box]
                ys = [p[1] for p in box]
                bpts += [(a, b) for a in np.linspace(min(xs), max(xs), 3)
                         for b in np.linspace(min(ys), max(ys), 9)]
            bar = np.array(bpts)
            for k in range(n):
                near = bar[np.hypot(bar[:, 0] - O[k, 0], bar[:, 1] - O[k, 1]) < 1300.0]
                if len(near):
                    clear[k] = np.minimum(clear[k], _rect_dist_pose(near[:, 0], near[:, 1], xx[k], yy[k], hd[k], shd[k], chd[k]).min(-1))
        _STATIC_CACHE[ckey] = clear
    clear = clear.copy()
    centers = [(px, py) for px, py, _c in pillars] + extra_centers
    for sname, seat in (unknown or ()):
        if sname in order and seat in SEATS and (sname, seat) not in seats:
            centers.append(section_to_world(sname, *SEATS[seat]))
    r_sign = SIGN_MM / 2.0 * math.sqrt(2.0)
    for cx, cy in centers:
        sk = (gridkey, float(cx), float(cy))
        if sk not in _SIGN_CACHE:
            ks = np.flatnonzero(np.hypot(O[:, 0] - cx, O[:, 1] - cy) < 1150.0)
            dsg = None
            if len(ks) > 0:
                dsg = _rect_dist_pose(np.array([cx]), np.array([cy]), xx[ks], yy[ks], hd[ks], shd[ks], chd[ks])[..., 0] - r_sign
            _SIGN_CACHE[sk] = (ks, dsg)
        ks, dsg = _SIGN_CACHE[sk]
        if len(ks) == 0:
            continue
        clear[ks] = np.minimum(clear[ks], dsg)
    bad = clear < prm.hard_clear_mm
    bonus = np.zeros((n, m, m))
    pen = prm.w_clear * np.clip((prm.soft_clear_mm - clear) / prm.soft_clear_mm, 0.0, None) ** 2
    if prm.w_unknown_near > 0.0:
        T = np.roll(O, -1, axis=0) - np.roll(O, 1, axis=0)
        T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
        for sname, seat in (unknown or ()):
            if sname not in order or seat not in SEATS or (sname, seat) in seats:
                continue
            ux, uy = section_to_world(sname, *SEATS[seat])
            along = np.abs((ux - O[:, 0]) * T[:, 0] + (uy - O[:, 1]) * T[:, 1])
            ks = np.flatnonzero((along < 300.0) & (np.hypot(O[:, 0] - ux, O[:, 1] - uy) < 1150.0))
            if len(ks) == 0:
                continue
            dist = np.hypot(P[ks, :, 0] - ux, P[ks, :, 1] - uy)                 # (k, m)
            near_pen = prm.w_unknown_near * np.clip(dist - prm.unknown_near_mm, 0.0, None) / 100.0
            pen[ks] += near_pen[:, None, :]
            if prm.w_visible > 0.0:
                # Premio por estación desde la que la cámara lo vería (zona de
                # detección medida ~98 %): la ruta pasa mirándolo y se resuelve.
                cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
                vk = (gridkey, float(ux), float(uy), cam, prm.view_min_mm, prm.view_max_mm, prm.view_half_deg)
                if vk not in _VIS_CACHE:
                    kv = np.flatnonzero(np.hypot(O[:, 0] - ux, O[:, 1] - uy) < 1500.0)
                    s, c = shd[kv], chd[kv]
                    cx = xx[kv] + cam * s
                    cy = yy[kv] + cam * c
                    dxs, dys = ux - cx, uy - cy
                    fwd = dxs * s + dys * c
                    latv = dxs * c - dys * s
                    dist = np.hypot(fwd, latv)
                    vis = (fwd > 0) & (dist >= prm.view_min_mm) & (dist <= prm.view_max_mm) & (
                        np.abs(latv) <= fwd * math.tan(math.radians(prm.view_half_deg)))
                    _VIS_CACHE[vk] = (kv, vis)
                kv, vis = _VIS_CACHE[vk]
                bonus[kv] -= prm.w_visible * vis
    allowed_d = np.ones((n, m), dtype=bool)
    for st, mask in side_rules:
        allowed_d[st] &= mask
    if anchor is not None:
        dist = np.hypot(P[..., 0] - anchor[0], P[..., 1] - anchor[1])     # (n, m)
        ka, ja = np.unravel_index(int(np.argmin(dist)), dist.shape)
        near_d = np.abs(D - D[ja]) <= 30.0
        allowed_d[ka] &= near_d
        # sin dejar estaciones vacías (si choca con una regla de lado, manda el ancla)
        if not allowed_d[ka].any():
            allowed_d[ka] = near_d
        if len(anchor) > 2:
            h = math.radians(anchor[2])
            f = np.array([math.sin(h), math.cos(h)])
            for step, tol in ((1, 12.0), (2, 20.0)):
                kb = (ka + step) % n
                seg = P[kb] - P[ka, ja]
                cosang = (seg @ f) / np.maximum(np.linalg.norm(seg, axis=1), 1e-9)
                ok = cosang >= math.cos(math.radians(tol))
                if (allowed_d[kb] & ok).any():
                    allowed_d[kb] &= ok
    for k, dval in (lock or {}).items():
        allowed_d[int(k) % n] &= np.abs(D - dval) <= 0.5 * prm.d_step
    if lock:
        # Un cambio de lado (~300 mm) justo al salir de las 7 estaciones fijas
        # cabe en el lookahead (200 mm): el follower va a tope y el CTE urgente
        # deja el tramo a PWM 110 (n3 s24, ~2.5 s). Se abre d a ritmo de giro.
        keys = sorted(lock, key=lambda k: (int(k) - min(int(x) for x in lock)) % n)
        k0 = int(keys[-1]) % n
        d0 = float(lock[keys[-1]])
        step = 60.0  # mm/estación: una esquina normal; más que eso es un quiebre
        for j in range(1, 13):
            k = (k0 + j) % n
            allowed_d[k] &= np.abs(D - d0) <= step * j + 0.5 * prm.d_step

    INF = 1e18
    N = laps * n
    # Geometría de la DP (no depende de obstáculos): caché por sentido, cajón de
    # salida, malla y parámetros de costo/curvatura.
    gkey = (direction, start_section or "N", n, prm.d_min, prm.d_max, prm.d_step, prm.w_len, prm.w_kappa,
            prm.time_cost, prm.a_lat_mm_s2, prm.v_min_mm_s, prm.v_max_mm_s, prm.r_min_mm, prm.max_angle_deg)
    geo = _GEOM_CACHE.get(gkey)
    if geo is None:
        # Ángulo de cada tramo (b en k -> c en k+1) contra la dirección del pasillo:
        # sin esto la DP "brinca" de lado entre estaciones (la curvatura de 3 puntos
        # no lo ve y la holgura solo se mide en las estaciones).
        lane_t = P[(np.arange(n) + 1) % n] - P                     # (n, m, 2) mismo d
        lane_t /= np.linalg.norm(lane_t, axis=-1, keepdims=True)
        max_cos = math.cos(math.radians(prm.max_angle_deg))
        side_ok = np.empty((n, m, m), dtype=bool)
        for k in range(n):
            seg = P[(k + 1) % n][None, :, :] - P[k][:, None, :]      # (b, c, 2)
            ln = np.linalg.norm(seg, axis=-1)
            cosang = (seg[..., 0] * lane_t[k][:, None, 0] + seg[..., 1] * lane_t[k][:, None, 1]) / np.maximum(ln, 1e-9)
            side_ok[k] = cosang >= max_cos
        geo = _GEOM_CACHE[gkey] = _dp_geometry(P, prm, side_ok)
    G, DS, MASK = geo
    # V[a, b]: costo de llegar con d_{t-1}=a, d_t=b. Arranque libre en t=1.
    Vc = np.where(bad[1] | ~allowed_d[1][None, :], INF, 0.0)
    back = np.zeros((N, m, m), dtype=np.int16)
    kern = _dp_kernel() if _NATIVE_DP else None
    if kern is not None:
        # Mismo bucle en C (fox_route_dp): mismas operaciones y orden, bit a bit.
        penc = np.ascontiguousarray(pen, dtype=np.float64)
        bonc = np.ascontiguousarray(bonus, dtype=np.float64)
        badu = np.ascontiguousarray(bad).view(np.uint8)
        alu = np.ascontiguousarray(allowed_d).view(np.uint8)
        Vc = np.ascontiguousarray(Vc, dtype=np.float64)
        Vtmp = np.empty((m, m))
        sparse = _dp_sparse_kernel() if os.environ.get("FOX_DP_SPARSE", "1") != "0" else None
        if sparse is not None:
            # Solo recorre a permitidos por (k,b,c): mismo resultado que el bucle denso.
            if gkey not in _RANGE_CACHE:
                ok = ~MASK                                   # (n, a, b, c)
                anyv = ok.any(axis=1)
                alo = np.where(anyv, ok.argmax(axis=1), m).astype(np.int16)
                ahi = np.where(anyv, m - 1 - ok[:, ::-1].argmax(axis=1), -1).astype(np.int16)
                _RANGE_CACHE[gkey] = (np.ascontiguousarray(alo), np.ascontiguousarray(ahi))
            alo, ahi = _RANGE_CACHE[gkey]
            sparse(n, m, N, G.ctypes.data, DS.ctypes.data, MASK.view(np.uint8).ctypes.data,
                   penc.ctypes.data, bonc.ctypes.data, badu.ctypes.data, alu.ctypes.data,
                   alo.ctypes.data, ahi.ctypes.data,
                   Vc.ctypes.data, Vtmp.ctypes.data, back.ctypes.data)
        else:
            kern(n, m, N, G.ctypes.data, DS.ctypes.data, MASK.view(np.uint8).ctypes.data,
                 penc.ctypes.data, bonc.ctypes.data, badu.ctypes.data, alu.ctypes.data,
                 Vc.ctypes.data, Vtmp.ctypes.data, back.ctypes.data)
    else:
        for t in range(1, N - 1):
            k1, k2 = t % n, (t + 1) % n
            step = DS[k1][None, :, :] * (G[k1] + pen[k2][None, :, :]) + bonus[k2][None, :, :]
            step = np.where(MASK[k1] | bad[k2][None, :, :] | ~allowed_d[k2][None, None, :], INF, step)
            tot = Vc[:, :, None] + step                                 # (a,b,c)
            arg = np.argmin(tot, axis=0)                                # (b,c)
            back[t + 1] = arg
            Vc = np.take_along_axis(tot, arg[None], axis=0)[0]
            Vc = np.minimum(Vc, INF)
    # Reconstrucción
    b, c = np.unravel_index(int(np.argmin(Vc)), Vc.shape)
    feasible = Vc[b, c] < INF / 2
    path = [c, b]
    for t in range(N - 1, 1, -1):
        a = back[t][path[-1], path[-2]]
        path.append(int(a))
    path = path[::-1]                                # d index por t = 0..N-1
    di = np.array(path[:N])
    mid = di[n:2 * n] if laps >= 3 else di[-n:]
    d_lap = D[mid].astype(float)
    xy = O + d_lap[:, None] * V
    route = _resample(xy, prm.step_mm, direction)
    route.report = check(route, pillars, parking, park_w)
    route.report["feasible"] = bool(feasible)
    route.report["junction_dd"] = float(abs(D[di[2 * n]] - D[di[n]])) if laps >= 3 else 0.0
    route.report["solve_ms"] = (time.perf_counter() - t0) * 1000.0
    route.d_stations = d_lap
    route.pillars = pillars
    route.station_O = O
    route.station_V = V
    route.feasible = bool(feasible)
    if not feasible and lock:
        # El tramo fijo ya no cabe con lo nuevo: se acorta (la mitad) y se reintenta.
        keys = sorted(lock, key=lambda k: (int(k) - min(lock)) % n)
        short = {k: lock[k] for k in keys[: len(keys) // 2]}
        return plan(direction, seats, parking, prm, start_section, laps, unknown, anchor, short or None,
                    park_w=park_w, extra=extra)
    if not feasible and anchor is not None:
        # El ancla cae donde no cabe (p. ej. el carro pegado a algo): sin ancla.
        return plan(direction, seats, parking, prm, start_section, laps, unknown, None,
                    park_w=park_w, extra=extra)
    if not feasible and unknown:
        # Lo desconocido tapa el paso: se planea solo con lo conocido.
        return plan(direction, seats, parking, prm, start_section, laps, None, None,
                    park_w=park_w, extra=extra)
    return route


def station_of(route: Route, x: float, y: float) -> int:
    """Estación de la malla más cercana a (x, y)."""
    O, V = route.station_O, route.station_V
    d = np.clip(((x - O[:, 0]) * V[:, 0] + (y - O[:, 1]) * V[:, 1]), 0.0, 1000.0)
    px = O[:, 0] + d * V[:, 0]
    py = O[:, 1] + d * V[:, 1]
    return int(np.argmin(np.hypot(px - x, py - y)))


def _resample(xy: np.ndarray, step: float, direction: str) -> Route:
    """Polilínea cerrada -> B-spline cúbica uniforme cerrada + muestreo uniforme.

    La B-spline es C2 (curvatura continua) y queda a ~κ·h²/6 de los puntos de
    la DP (h ≈ 75 mm, R ≥ 170: < 6 mm), así que no se come la holgura.
    """
    pts = xy
    B = [[], [], [], []]
    for t in np.linspace(0.0, 1.0, 16, endpoint=False):
        t2, t3 = t * t, t * t * t
        B[0].append((1 - t) ** 3 / 6.0)
        B[1].append((3 * t3 - 6 * t2 + 4) / 6.0)
        B[2].append((-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0)
        B[3].append(t3 / 6.0)
    B0, B1, B2, B3 = (np.array(b)[None, :, None] for b in B)
    # p0 = pts[i-1], p2 = pts[i+1], p3 = pts[i+2] (cíclico), como el bucle por i
    P0, P1 = np.roll(pts, 1, axis=0), pts
    P2, P3 = np.roll(pts, -1, axis=0), np.roll(pts, -2, axis=0)
    sub = (B0 * P0[:, None, :] + B1 * P1[:, None, :] + B2 * P2[:, None, :] + B3 * P3[:, None, :]).reshape(-1, 2)
    seg = np.hypot(*np.diff(np.vstack([sub, sub[:1]]), axis=0).T)
    s_cum = np.concatenate([[0.0], np.cumsum(seg)])
    length = float(s_cum[-1])
    s_new = np.arange(0.0, length, step)
    sc = np.vstack([sub, sub[:1]])
    x = np.interp(s_new, s_cum, sc[:, 0])
    y = np.interp(s_new, s_cum, sc[:, 1])
    hd, kap = _heading_kappa(x, y, length / len(x))
    return Route(x, y, hd, kap, s_new, length, direction)


def _heading_kappa(x, y, ds):
    dx = (np.roll(x, -1) - np.roll(x, 1)) / (2 * ds)
    dy = (np.roll(y, -1) - np.roll(y, 1)) / (2 * ds)
    hd = np.degrees(np.arctan2(dx, dy)) % 360.0
    psi = np.radians(hd)
    dpsi = np.roll(psi, -1) - np.roll(psi, 1)
    dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
    # rumbo crece a la derecha (0=+y, 90=+x): κ>0 = giro a la derecha
    kap = dpsi / (2 * ds)
    return hd, kap


def check(route: Route, pillars, parking: str | None, park_w: float | None = None) -> dict:
    """Holguras mínimas de la huella sobre la ruta (mm) y curvatura máxima."""
    x, y = route.x, route.y
    hd = np.radians(route.heading_deg)
    rep = {}
    if pillars:
        pts = np.vstack([_square_pts(px, py, SIGN_MM / 2.0, 7) for px, py, _c in pillars])
        dist = _rect_dist_pose(pts[:, 0], pts[:, 1], x, y, hd)
        rep["sign_mm"] = float(dist.min())
    cs = _corners_pose(x, y, hd)
    rep["wall_mm"] = float(OUTER_HALF_MM - np.max([np.maximum(np.abs(cx), np.abs(cy)) for cx, cy in cs]))
    rep["island_mm"] = float(_static_clearance(x, y, hd, None).min())
    if parking is not None:
        pts = []
        for box in _barrier_corners(parking, *(() if park_w is None else (park_w,))):
            xs = [p[0] for p in box]
            ys = [p[1] for p in box]
            pts += [(a, b) for a in np.linspace(min(xs), max(xs), 4) for b in np.linspace(min(ys), max(ys), 12)]
        pts = np.array(pts)
        rep["barrier_mm"] = float(_rect_dist_pose(pts[:, 0], pts[:, 1], x, y, hd).min())
    rep["kappa_max"] = float(np.abs(route.kappa).max())
    rep["r_min_mm"] = float(1.0 / max(rep["kappa_max"], 1e-9))
    rep["length_mm"] = route.length_mm
    return rep


# ── Seguidor ───────────────────────────────────────────────────────────────


class RouteFollower:
    """Pure pursuit sobre la ruta, con la pose del eje trasero (mm, grados)."""

    def __init__(self, route: Route, lookahead_mm: float = 200.0,
                 wheelbase_mm: float = C.WHEELBASE_MM):
        self.route = route
        self.L = lookahead_mm
        self.W = wheelbase_mm
        self.i = -1
        self.last_target = None
        self.cte = 0.0

    def reset(self) -> None:
        self.i = -1

    def _nearest(self, x, y, heading_deg):
        r = self.route
        n = len(r.x)
        if self.i < 0:
            d2 = (r.x - x) ** 2 + (r.y - y) ** 2
            err = np.abs((r.heading_deg - heading_deg + 180.0) % 360.0 - 180.0)
            d2 = d2 + np.where(err > 70.0, 1e12, 0.0)
            self.i = int(np.argmin(d2))
            return self.i
        win = (self.i + np.arange(-5, 60)) % n
        d2 = (r.x[win] - x) ** 2 + (r.y[win] - y) ** 2
        self.i = int(win[np.argmin(d2)])
        return self.i

    def steer(self, x: float, y: float, heading_deg: float) -> float:
        """Ángulo de RUEDA (deg), + = derecha."""
        r = self.route
        n = len(r.x)
        i = self._nearest(x, y, heading_deg)
        step = r.length_mm / n
        j = (i + int(round(self.L / step))) % n
        tx, ty = r.x[j], r.y[j]
        self.last_target = (float(tx), float(ty))
        h = math.radians(heading_deg)
        dx, dy = tx - x, ty - y
        fwd = dx * math.sin(h) + dy * math.cos(h)
        right = dx * math.cos(h) - dy * math.sin(h)
        ld2 = max(fwd * fwd + right * right, 1.0)
        hx, hy = math.sin(math.radians(r.heading_deg[i])), math.cos(math.radians(r.heading_deg[i]))
        ex, ey = x - r.x[i], y - r.y[i]
        self.cte = ex * hy - ey * hx                 # + = carro a la derecha de la ruta
        kappa = 2.0 * right / ld2
        return math.degrees(math.atan(self.W * kappa))
