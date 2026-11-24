"""Modo ruta del lado de la Pi: mapa -> ruta en mm -> el ESP la sigue.

Reparto:
* Pi: visión + mapa de señales (DigitalMap), qué asientos ya se conocen,
  planeación de la ruta (ring_route), localización (localizer) y la velocidad
  permitida según lo que falta por ver.
* ESP: pure pursuit sobre la ruta con su odometría, a la velocidad del loop
  (RTB/RTP/RTE + rt=1,rtx,rty,rth,rv en el V2).

Vuelta 1 (mapa incompleto): un asiento sin resolver es un obstáculo sin lado
de paso (la ruta no pasa por donde podría haber una señal). Un asiento se
resuelve si el mapa lo confirma, si las reglas de las cartas lo descartan, o
si estuvo a buena vista de la cámara suficientes frames sin aparecer nada. Si
el siguiente asiento sin resolver queda cerca sobre la ruta, el carro frena
hasta resolverlo. Cada vez que cambia lo conocido se replanea anclado a la
pose del carro.

Arranque ciego (ROUTE_START_BLIND, start_fix.py): el cajón puede estar en
cualquier punto de la recta de salida. Hasta saber el avance del carro todo va
en el marco "supuesto" (cajón lo más atrás posible); la cámara se guarda sin
votar y la recta del cajón se planea con lo que la cámara ve relativo al carro
(una sola señal posible adelante, en la fila interior). Al fijar el avance se
corre la pose, se vota lo guardado y se replanea; el ESP sigue con la
transformación del marco viejo hasta tener la ruta nueva completa.
"""

from __future__ import annotations

from copy import copy
import math
import os

import cv2
import numpy as np

from . import config as C
from . import ring_route as RR
from .digital_map import feet_world
from .localizer import Localizer, _ack_num
from .start_fix import StartFix
from .park_refine import ParkRefine
from .wro_field import (INNER_HALF_MM, OUTER_HALF_MM, PARK_W_MAX, PARK_W_MIN, PARKING_FORBIDDEN_SEATS, SEATS,
                        SECTIONS_CW, SIGN_MM, blue_segments, orange_segments, section_to_world, world_to_section)

_ROW_H = SEATS["T1"][0]          # fila interior (600 mm de la pared exterior)
_ROW_BAND_MM = 160.0             # detección que cae en la fila interior
_CLUSTER_GAP_MM = 150.0
_FENCE_STEP_MM = 100.0

def cte_urgent_holds(cte_mm: float, dh_deg: float, wall_mm: float, sign_mm: float, island_mm: float) -> bool:
    """Sigue el tope de 110 salvo en la cuerda larga.

    Ahí el carro va a más de 100 mm de la ruta y la ruta, a 500 mm, ya dobla
    (n3 s21 / n2 s24 / n3 s24: |cte| > 120 mm y más de 30°). n3 s16 es el
    mismo desvío sobre una recta (0–5°) y n1 s8 solo llega a 49 mm: soltar
    el tope ahí alarga la vuelta. Con la huella a menos de 40 mm de pared,
    lata confirmada o isla, el tope se queda.
    """
    if abs(cte_mm) < 100.0 or dh_deg < 25.0:
        return True
    return min(wall_mm, sign_mm, island_mm) < 40.0


def _grip_pwm(steer_deg: float) -> int:
    """PWM que aguanta el agarre medido (2400 mm/s²) en el radio de este volante.

    La cuerda solo suelta el 110 si esto llega al PWM máximo: a tope (R~134)
    255 derrapa y la salida queda abierta (n5 s21 roza al estacionar).
    """
    st = math.radians(min(abs(float(steer_deg)), float(C.MAX_WHEEL_STEER_DEG)))
    if st < math.radians(5.0):
        return int(getattr(C, "ROUTE_PROFILE_MAX_PWM", 255))
    v = math.sqrt(2400.0 * float(C.WHEELBASE_MM) / math.tan(st))
    K = float(getattr(C, "ROUTE_PROFILE_K_MM_S_PER_PWM", 3.5))
    pwm0 = float(getattr(C, "ROUTE_PROFILE_PWM0", 25.0))
    pwm = int(v / K + pwm0)
    lo = int(getattr(C, "ROUTE_CTE_URGENT_PWM", 110))
    hi = int(getattr(C, "ROUTE_PROFILE_MAX_PWM", 255))
    return max(lo, min(hi, pwm))


def _rv_mando(rv) -> int:
    """PWM de la Pi (escala del N20, puede pasar de 255) -> PWM del ESP con la escala de mando N30
    (vMando: igual hasta ROUTE_MANDO_CODO_PWM, recto hasta ROUTE_MANDO_V_255 a 255). None = sin traducir."""
    v255 = getattr(C, "ROUTE_MANDO_V_255", None)
    codo = float(getattr(C, "ROUTE_MANDO_CODO_PWM", 155))
    if v255 is None or rv <= codo:
        return int(min(rv, 255))
    k, p0 = float(C.ROUTE_MANDO_K), float(C.ROUTE_MANDO_PWM0)
    vc = k * (codo - p0)
    return int(min(255, round(codo + (k * (rv - p0) - vc) * (255.0 - codo) / (float(v255) - vc))))


def _chord_ahead(follower, ahead_mm: float = 500.0) -> tuple[float]:
    """Cuánto dobla la ruta (grados) en los próximos ahead_mm."""
    r = follower.route
    n = len(r.kappa)
    i = max(follower.i, 0)
    step = r.length_mm / n
    k = (i + max(1, int(round(ahead_mm / max(step, 1e-9))))) % n
    dh = abs((float(r.heading_deg[k]) - float(r.heading_deg[i]) + 180.0) % 360.0 - 180.0)
    return (dh,)


def _pursuit_kappa(route, idx, lookahead_mm, left_f=None):
    """|κ| del pure pursuit parado en idx, mirando lookahead_mm sobre la ruta.

    Misma cuenta que rutaSteer del ESP: blanco a lookahead fijo, κ = 2·lateral/ld².
    Un pico de κ más corto que el lookahead no lo pide el servo.
    """
    n = len(route.x)
    step = route.length_mm / max(n, 1)
    ahead = max(1, int(round(float(lookahead_mm) / max(step, 1e-9))))
    j = np.asarray(idx, dtype=int) % n
    q = (j + ahead) % n
    h = np.radians(route.heading_deg[j])
    dx = route.x[q] - route.x[j]
    dy = route.y[q] - route.y[j]
    fwd = dx * np.sin(h) + dy * np.cos(h)
    right = dx * np.cos(h) - dy * np.sin(h)
    k = 2.0 * right / np.maximum(fwd * fwd + right * right, 1.0)
    f = left_f or getattr(C, "ROUTE_PROFILE_LEFT_K_FACTOR", None)   # el carro agarra menos a la izquierda: κ izquierda cuenta más
    return np.abs(k) * (np.where(k < 0.0, float(f), 1.0) if f else 1.0)


def _prm_for(unknown, tc=0.0):
    """Mapa completo: no comprar arco hasta 140 mm (arquea la recta hacia la pared).

    Con asientos sin ver se queda el margen de config. Semilla 21 ruido 2 chocó
    Red N/T4 en la vuelta 1 cuando el margen bajaba también ahí.
    """
    prm = _route_params()
    l1s = getattr(C, "ROUTE_L1_SOFT_CLEAR_MM", None)
    if unknown and l1s is not None and float(tc or 0) >= 1.0:
        prm.soft_clear_mm = float(l1s)   # vuelta 1 tras la salida: margen propio (en tc=0 rompe la salida: s3/s33/s45)
    if not unknown:
        prm.soft_clear_mm = float(getattr(C, "ROUTE_FULL_SOFT_CLEAR_MM", 100.0))
        for k in ("w_kappa", "w_clear"):   # mapa completo: ROUTE_FULL_W_KAPPA / ROUTE_FULL_W_CLEAR pisan el peso
            v = getattr(C, "ROUTE_FULL_" + k.upper(), None)
            if v is not None:
                setattr(prm, k, float(v))
        if getattr(C, "ROUTE_FULL_HARD_CLEAR_MM", None) is not None:
            prm.hard_clear_mm = float(C.ROUTE_FULL_HARD_CLEAR_MM)
    return prm


def _route_params():
    """Parámetros del planificador con lo ajustable desde config.py."""
    K = float(getattr(C, "ROUTE_PROFILE_K_MM_S_PER_PWM", 3.5))
    pwm0 = float(getattr(C, "ROUTE_PROFILE_PWM0", 25.0))
    return RR.RouteParams(
        w_kappa=float(getattr(C, "ROUTE_W_KAPPA", 3.0e4)),
        d_step=float(getattr(C, "ROUTE_D_STEP", 20.0)),
        hard_clear_mm=float(getattr(C, "ROUTE_HARD_CLEAR_MM", 55.0)),
        soft_clear_mm=float(getattr(C, "ROUTE_SOFT_CLEAR_MM", 140.0)),
        w_clear=float(getattr(C, "ROUTE_W_CLEAR", 8.0)),
        corner_exit_clear_mm=float(getattr(C, "ROUTE_CORNER_EXIT_CLEAR_MM", 0.0)),
        corner_exit_inside_extra_mm=float(getattr(C, "ROUTE_CORNER_EXIT_INSIDE_EXTRA_MM", 0.0)),
        time_cost=bool(getattr(C, "ROUTE_TIME_COST", False)),
        # ROUTE_PLAN_VMIN_PWM: piso de velocidad que ve la DP (con BASE_PWM=255 el costo de tiempo no distingue curvas).
        v_min_mm_s=K * (float(getattr(C, "ROUTE_PLAN_VMIN_PWM", None) or getattr(C, "ROUTE_PROFILE_BASE_PWM", 155)) - pwm0),
        v_max_mm_s=K * (float(getattr(C, "ROUTE_PROFILE_MAX_PWM", 255)) - pwm0),
        a_lat_mm_s2=float(getattr(C, "ROUTE_PROFILE_A_LAT_MM_S2", 1500.0)),
        w_visible=float(getattr(C, "ROUTE_W_VISIBLE", 60.0)),
        w_unknown_near=float(getattr(C, "ROUTE_W_UNKNOWN_NEAR", 1.5)),
    )


def _ack_est(ack):
    if not ack or "est=" not in ack:
        return None
    return ack.split("est=", 1)[1][:1]


def _pwm(name):
    """Tope de PWM de `name`; None en config = ROUTE_SLOW_PWM."""
    v = getattr(C, name, None)
    if v is not None:
        return int(v)
    return int(getattr(C, "ROUTE_SLOW_PWM", 100))


_COL = {"T4": 0, "T2": 0, "X2": 1, "X1": 1, "T3": 2, "T1": 2}    # columna (w 1000/1500/2000)


def _stall_far_along(parking: str, direction: str, park_w: float) -> float:
    """Along (eje del carril en el sentido de carrera, 0 = centro de la recta; alongSalidaMm del ESP)
    de la madera del cajón que queda más adelante: la primera que encuentra el escaneo al regresar."""
    from .wro_field import _barrier_corners, _heading
    h = math.radians(_heading(parking, direction))
    fx, fy = math.sin(h), math.cos(h)
    return max(px * fx + py * fy for b in _barrier_corners(parking, park_w) for px, py in b)


_WARM_DONE = False


def warm_plan() -> None:
    """Llena las cachés del planificador (geometría de la DP ~2 s en la Pi 5, vistas y holguras de los 24 asientos)
    en los dos sentidos. Se llama una vez al crear el runtime (antes de armar), no en el primer plan de la carrera."""
    global _WARM_DONE
    if _WARM_DONE or not getattr(C, "ROUTE_WARM_PLAN", False):
        return
    _WARM_DONE = True
    unk = [(sec, seat) for sec in SECTIONS_CW for seat in SEATS]
    for d in ("CW", "CCW"):
        RR.plan(d, {}, None, prm=_route_params(), unknown=unk)

_WARMED = False


def warm_caches() -> None:
    """Llena _GEOM/_STATIC_CACHE (solo dependen de direccion y parametros) antes de armar: el primer plan tarda ~2 s en la Pi."""
    global _WARMED
    if _WARMED:
        return
    _WARMED = True
    for d in ("CW", "CCW"):
        RR.plan(d, {}, None, prm=_route_params())
        RR.plan(d, {}, None, prm=_prm_for([]))


def pix_line_delta(pix, pose, axis, direction, fwd_add: float = 0.0):
    """(δ, texto) de la cinta vista en `pix` (pixeles BEV) con el carro en `pose` (x, y, rumbo°), o None.
    Ver RouteMode._pix_line_vote. Función suelta para re-probarla offline con los BEV grabados."""
    pix = np.asarray(pix, float)
    a = np.asarray(axis, float)
    fwd = (float(C.ROBOT_BEV_Y) - pix[:, 1]) * C.MM_PER_PX + float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0)) + fwd_add
    lat = (pix[:, 0] - float(C.ROBOT_BEV_X)) * C.MM_PER_PX
    span = PARK_W_MAX - PARK_W_MIN
    mg = float(getattr(C, "ROUTE_FIX_LINE_RANGE_MARGIN_MM", 150.0))
    lo, hi = (-mg, span + mg) if direction == "CW" else (-span - mg, mg)
    win = float(getattr(C, "ROUTE_FIX_PIX_LINE_W_MM", 30.0))
    min_n = int(getattr(C, "ROUTE_FIX_LINE_MIN_PIX", 25))
    min_span = float(getattr(C, "ROUTE_FIX_PIX_LINE_SPAN_MM", 100.0))
    # Con la línea entera (no un punto) la cinta empinada de CCW (|n·a| 0.5) también sirve: el error
    # lateral de la pose se multiplica x1.7, pero tras la S queda en ~2-3 cm (2026-11-21).
    min_na = float(getattr(C, "ROUTE_FIX_PIX_LINE_MIN_NA", 0.45))
    dh_max = float(getattr(C, "ROUTE_FIX_PIX_LINE_DH_DEG", 6.0))
    dh_step = float(getattr(C, "ROUTE_FIX_PIX_LINE_DH_STEP", 1.0))
    segs = []
    for (sx, sy), (ex, ey) in orange_segments():
        u = np.array([ex - sx, ey - sy], dtype=float); u /= np.linalg.norm(u)
        n = np.array([-u[1], u[0]])
        if abs(float(n @ a)) >= min_na:
            segs.append((u, n, float(n @ np.array([sx, sy]))))
    best = None
    for dh in np.arange(-dh_max, dh_max + 1e-6, dh_step):
        h = math.radians(pose[2] + float(dh))
        P = np.column_stack([pose[0] + fwd * math.sin(h) + lat * math.cos(h),
                             pose[1] + fwd * math.cos(h) - lat * math.sin(h)])
        for u, n, ns in segs:
            d = (ns - P @ n) / (n @ a)
            ok = (d >= lo) & (d <= hi)
            if ok.sum() < min_n:
                continue
            ds = np.sort(d[ok])
            j = np.searchsorted(ds, ds + win, side="right")
            cnt = j - np.arange(len(ds))
            i = int(np.argmax(cnt))
            c = int(cnt[i])
            if c < min_n or (best is not None and c <= best[0]):
                continue
            sel = ok & (d >= ds[i]) & (d <= ds[i] + win)
            t = P[sel] @ u
            sp = float(np.percentile(t, 95) - np.percentile(t, 5))
            best = (c, float(np.median(d[sel])), float(dh), sp, int(ok.sum()))
    if best is None:
        return None
    c, dmed, dh, sp, tot = best
    if sp < min_span:
        return None
    bias = float(getattr(C, "ROUTE_FIX_PIX_DELTA_BIAS_MM", 0.0))
    return dmed + bias, f"n={c}/{tot} largo={sp:.0f} dh={dh:+.0f}"


class RouteMode:
    def __init__(self) -> None:
        if getattr(C, "ROUTE_WARM", False):
            warm_caches()
        self.route = None
        self.follower = None
        self.loc = None
        self.key = None
        self.upload: list[str] = []
        self._upload_full: list[str] = []   # la subida completa, para reenviarla
        self._rte_n = None           # puntos del último RTE mandado, hasta que el ACK traiga rn igual
        self._rte_wait = 0
        self.n_retries = 0
        self.esp_route = None        # última ruta completa en el ESP (tras su RTE)
        self.empty: set = set()
        self.view_count: dict = {}
        self.rv = None
        self._rv_last = None
        self.visible: set = set()
        self.stopped = 0
        self.last_print = None
        self.n_plans = 0
        self._step_frame = 0
        self._plan_frame = None
        self._l23_entry_cap = None
        self._job = None
        self._swap_left = 0
        self._job_key = None
        self._job_out = None
        self._job_conf = {}
        self._job_unknown = []
        self._job_shift = 0.0
        self._delay_left = None      # frames que faltan para adoptar el plan retrasado (ROUTE_PLAN_DELAY_S)
        self.pi_steer = False        # ps=1: el ESP gira con el obs de la Pi (ruta nueva) mientras sube
        self.pi_wheel = 0.0          # pw=: rueda (deg, + = derecha) de la Pi con la ruta nueva, para ps=1
        self._delay_plan = None
        # Arranque ciego
        self.blind = bool(getattr(C, "ROUTE_START_BLIND", False))
        self.fix = None              # StartFix mientras no se sabe el avance
        self.orange = None           # estado de la línea naranja (OrangeLineTracker) del frame
        self._pend_n = 0             # cuadros esperando evidencia del cajón con la media plana
        self._line_d: list = []      # δ por lectura de la línea naranja (ROUTE_FIX_LINE)
        self._line_vote_src: list[str] = []  # fuente de cada voto en _line_d (recta, pix, linea, azul)
        self._post_blue_d: list[float] = []  # votos de la cinta azul tras fijar el arranque (_post_blue)
        self._post_blue_done = False
        self._pix_ok = None          # None = aún no se sabe si la cinta de esta recta sirve para pixeles
        self.fixed = not self.blind
        self.shift = 0.0             # δ aplicado: verdad = supuesto + δ·w de la recta del cajón
        self.axis = (0.0, 0.0)
        self.maneuver_loc = None     # pose del visor durante E/G
        self._park_refine = None
        self._park_refined_w = None
        self.park_w = None           # w0 del cajón en uso (supuesto o medido)
        _pf = getattr(C, "ROUTE_PRUEBA_CAJON_FILE", None)
        self._prueba_cajon = bool(_pf) and os.path.exists(str(_pf))
        self._prueba_n = 0
        if self._prueba_cajon:
            print(f"[PRUEBA] modo salida del cajón ({_pf}): se para al ubicar el cajón", flush=True)
        self.route_shift = 0.0       # marco de self.route
        self.esp_shift = 0.0         # marco de la ruta activa en el ESP
        self.upload_shift = 0.0      # marco de la ruta que se está subiendo
        self.hold = False            # detenido hasta que la ruta nueva llegue al ESP
        self._def_seen = 0
        self._pre_dets: list = []    # (w, color, dist a la cámara) en la fila interior
        self._pre_poses: list = []
        self._fence = None
        self._pre_key = None
        self._pre_stop = 0
        self.frame = None            # frame de perspectiva BGR (SEAT_ROI_LOG)
        self.bev = None              # BEVTransformer del runtime (H_inv para la ROI)
        self._roi_t = 0
        self._roi_err = False
        self._roi_prev = None       # (rumbo, frame) del último frame medido (yaw)
        self._col_err = False
        self._dump_n = 0             # frames volcados a SEAT_ROI_DUMP_DIR
        self.roi_color: dict = {}    # (sec, columna) -> "Red"/"Green" por ROI (ROUTE_SEAT_ROI)
        self._col_run: dict = {}     # (sec, columna) -> (clase, frames seguidos) (ROUTE_SEAT_ROI)
        self.color_ranges = None     # rangos HSV de Vision del runtime (None -> COLOR_RANGES)

    # ── asientos ──────────────────────────────────────────────────────────
    def _resolved(self, dm) -> tuple[dict, set]:
        conf = dict(dm.confirmed)
        self.empty -= set(conf)      # un asiento confirmado ya no puede estar vacío
        empty = set(self.empty)
        for seat in PARKING_FORBIDDEN_SEATS:
            if (dm.parking, seat) not in conf:
                empty.add((dm.parking, seat))
        for sec in SECTIONS_CW:
            if not any(sc == sec for (sc, _s) in conf):
                # Toda recta tiene al menos una señal: si los 6 asientos
                # quedaron "vacíos" la visión se la perdió; se vuelven a mirar.
                if all((sec, s) in empty for s in SEATS):
                    for s in SEATS:
                        if (sec, s) in self.empty:
                            self.empty.discard((sec, s))
                            empty.discard((sec, s))
                            self.view_count[(sec, s)] = 0
        empty |= self._rule_empty(conf)
        return conf, empty

    def _rule_empty(self, seats: dict) -> set:
        """Asientos que las reglas de la recta dejan vacíos, dado {(sec, asiento): color}."""
        out = set()
        for sec in SECTIONS_CW:
            mine = [s for (sc, s) in seats if sc == sec]
            if not mine:
                continue
            cols = {_COL[s] for s in mine}
            for seat in SEATS:
                if (sec, seat) in seats:
                    continue
                # X va sola; nunca dos en la misma columna; máximo dos por recta.
                if (any(s.startswith("X") for s in mine) or seat.startswith("X")
                        or _COL[seat] in cols or len(mine) >= 2):
                    out.add((sec, seat))
        return out

    def _all_seats(self):
        return [(sec, seat) for sec in SECTIONS_CW for seat in SEATS]

    def _update_views(self, conf, empty, pose) -> None:
        """Cuenta frames con cada asiento sin resolver a buena vista de la cámara."""
        x, y, hd = pose
        h = math.radians(hd)
        fx, fy = math.sin(h), math.cos(h)
        cam_fwd = float(getattr(C, "CAMERA_FWD_MM", 140.0))
        cx, cy = x + fx * cam_fwd, y + fy * cam_fwd
        known = [section_to_world(s, *SEATS[t]) for (s, t) in conf]
        self.visible = set()
        tan_b = math.tan(math.radians(float(getattr(C, "ROUTE_VIEW_HALF_DEG", 40.0))))
        for key in self._all_seats():
            if key in conf or key in empty:
                continue
            sx, sy = section_to_world(key[0], *SEATS[key[1]])
            dx, dy = sx - cx, sy - cy
            fwd = dx * fx + dy * fy
            lat = dx * fy - dy * fx
            # Donde la visión confirma de verdad: cerca (distancia a la cámara)
            # y no muy de lado. Más lejos una señal real no alcanza a votarse y
            # se daba por vacía (semilla 50: verde a 1.44 m).
            if not (fwd >= 250.0 and math.hypot(fwd, lat) <= float(getattr(C, "ROUTE_VIEW_MAX_MM", 1000.0))
                    and abs(lat) <= fwd * tan_b):
                continue
            # tapado por una señal conocida en la línea de vista
            dist = math.hypot(dx, dy)
            hidden = False
            for kx, ky in known:
                ex, ey = kx - cx, ky - cy
                t = (ex * dx + ey * dy) / max(dist * dist, 1.0)
                if 0.0 < t < 1.0 and abs(ex * dy - ey * dx) / max(dist, 1.0) < 60.0:
                    hidden = True
                    break
            if hidden:
                continue
            self.visible.add(key)
            n = self.view_count.get(key, 0) + 1
            self.view_count[key] = n
            if n >= int(getattr(C, "ROUTE_EMPTY_FRAMES", 16)):
                self.empty.add(key)

    def _roi_yaw(self, pose) -> float:
        """Avanza el contador de frames de ROI; devuelve el yaw (grados/frame) desde el frame anterior."""
        self._roi_t += 1
        yaw = float("nan")
        hd = pose[2]
        if self._roi_prev is not None:
            dh = (hd - self._roi_prev[0] + 180.0) % 360.0 - 180.0
            yaw = dh / max(self._roi_t - self._roi_prev[1], 1)
        self._roi_prev = (hd, self._roi_t)
        return yaw

    def _seat_roi(self, conf, empty, pose) -> bool:
        """Clasificación en la imagen (lap1): [ROI]/[COL] con SEAT_ROI_LOG y, con ROUTE_SEAT_ROI, la compuerta por columna. True si cambió self.empty."""
        use = bool(getattr(C, "ROUTE_SEAT_ROI", False))
        log = bool(getattr(C, "SEAT_ROI_LOG", False))
        if not ((use or log) and self.frame is not None and self.bev is not None
                and self.bev.is_calibrated and self.fixed):
            self._col_run = {}
            return False
        yaw = self._roi_yaw(pose)
        if log:
            self._log_seats(conf, empty, pose)
        try:
            cols = self._classify_columns(conf, empty, pose, yaw)
            if log:
                self._log_columns(cols, pose)
        except Exception as e:
            self._col_run = {}
            if not self._col_err:
                self._col_err = True
                print(f"[COL] error: {e!r}", flush=True)
            return False
        if not use:
            return False
        before = set(self.empty)
        self._col_gate(cols, conf, empty)
        return self.empty != before

    def _log_seats(self, conf, empty, pose) -> None:
        """SEAT_ROI_LOG: [ROI] por asiento de frente a la cámara (solo medición)."""
        from .seat_roi import classify, seat_span
        try:
            x, y, hd = pose
            h = math.radians(hd)
            fx, fy = math.sin(h), math.cos(h)
            cam_fwd = float(getattr(C, "CAMERA_FWD_MM", 140.0))
            cx, cy = x + fx * cam_fwd, y + fy * cam_fwd
            known = [section_to_world(s, *SEATS[t]) for (s, t) in conf]
            max_mm = float(getattr(C, "SEAT_ROI_MAX_MM", 3500.0))
            for key in self._all_seats():
                sx, sy = section_to_world(key[0], *SEATS[key[1]])
                dx, dy = sx - cx, sy - cy
                fwd = dx * fx + dy * fy
                lat = dx * fy - dy * fx
                dist = math.hypot(dx, dy)
                if fwd < 150.0 or dist > max_mm:
                    continue
                # tapado por una señal confirmada más cercana en la línea de vista (como _update_views)
                hidden = False
                for kx, ky in known:
                    ex, ey = kx - cx, ky - cy
                    t = (ex * dx + ey * dy) / max(dist * dist, 1.0)
                    if 0.0 < t < 1.0 and abs(ex * dy - ey * dx) / max(dist, 1.0) < 60.0:
                        hidden = True
                        break
                if hidden:
                    continue
                b = math.degrees(math.atan2(lat, fwd))
                span = seat_span(self.bev.H_inv, pose, sx, sy)
                if span is None:
                    print(f"[ROI-OUT] t={self._roi_t} sec={key[0]} seat={key[1]} d={dist:.0f} b={b:.1f}",
                          flush=True)
                    continue
                (u0, v0, u1, v1), w = span
                cls, fr, fg = classify(self.frame, (u0, v0, u1, v1))
                unk = 0 if (key in conf or key in empty) else 1
                print(f"[ROI] t={self._roi_t} sec={key[0]} seat={key[1]} d={dist:.0f} b={b:.1f} "
                      f"w={w:.0f} cls={cls} fr={fr:.2f} fg={fg:.2f} unk={unk}", flush=True)
        except Exception as e:
            if not self._roi_err:
                self._roi_err = True
                print(f"[ROI] error: {e!r}", flush=True)

    def _col_occluded(self, conf, empty, pose, sec: str, col: int, win, dist: float) -> int:
        """0 si la columna se ve; 2 si la isla central la tapa; 1 si la tapa una señal más cercana
        (confirmada o sin resolver) o una columna confirmada más cercana (solape en u y v)."""
        from .seat_roi import column_window, island_hidden, seat_span
        cx, cy = pose[0] + math.sin(math.radians(pose[2])) * float(getattr(C, "CAMERA_FWD_MM", 140.0)), \
            pose[1] + math.cos(math.radians(pose[2])) * float(getattr(C, "CAMERA_FWD_MM", 140.0))
        seats = [t for t in SEATS if _COL[t] == col]
        if island_hidden(pose, sec, seats, float(getattr(C, "SEAT_COL_MARGIN_MM", 80.0)), INNER_HALF_MM):
            return 2

        def overlap(a, b):
            return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]

        for s2, t2 in self._all_seats():
            if s2 == sec and _COL[t2] == col:
                continue
            if (s2, t2) in empty:
                continue
            sx, sy = section_to_world(s2, *SEATS[t2])
            if math.hypot(sx - cx, sy - cy) >= dist:
                continue
            span = seat_span(self.bev.H_inv, pose, sx, sy)
            if span is None:
                continue
            (u0, v0, u1, v1), _w = span
            hw = 0.75 * (u1 - u0)                       # ROI x1.5 de ancho
            cu = (u0 + u1) / 2.0
            if overlap(win, (cu - hw, v0, cu + hw, v1)):
                return 1
        for s2 in SECTIONS_CW:
            for c2 in (0, 1, 2):
                if (s2 == sec and c2 == col) or not any((s2, t) in conf for t in SEATS if _COL[t] == c2):
                    continue
                cols2 = [t for t in SEATS if _COL[t] == c2]
                sx, sy = section_to_world(s2, *SEATS[cols2[0]])
                if math.hypot(sx - cx, sy - cy) >= dist:
                    continue
                box2 = column_window(self.bev.H_inv, pose, s2, cols2, float(getattr(C, "SEAT_COL_MARGIN_MM", 80.0)))
                if box2 is not None and overlap(win, box2[0]):
                    return 1
        return 0

    def _classify_columns(self, conf, empty, pose, yaw: float) -> list:
        """Clasificación de cada columna (dos asientos con el mismo w) en el frame: lista de dicts."""
        from .seat_roi import classify_window, column_sign_area, column_window
        x, y, hd = pose
        h = math.radians(hd)
        fx, fy = math.sin(h), math.cos(h)
        cam_fwd = float(getattr(C, "CAMERA_FWD_MM", 140.0))
        cx, cy = x + fx * cam_fwd, y + fy * cam_fwd
        max_mm = float(getattr(C, "SEAT_ROI_MAX_MM", 3500.0))
        margin = float(getattr(C, "SEAT_COL_MARGIN_MM", 80.0))
        res = []
        for sec in SECTIONS_CW:
            for col in (0, 1, 2):
                seats = [t for t in SEATS if _COL[t] == col]
                wx, wy = section_to_world(sec, 500.0, SEATS[seats[0]][1])
                dx, dy = wx - cx, wy - cy
                fwd = dx * fx + dy * fy
                lat = dx * fy - dy * fx
                dist = math.hypot(dx, dy)
                if fwd < 150.0 or dist > max_mm:
                    continue
                unk = sum(1 for t in seats if (sec, t) not in conf and (sec, t) not in empty)
                r = dict(sec=sec, col=col, seats=seats, d=dist, b=math.degrees(math.atan2(lat, fwd)),
                         yaw=yaw, unk=unk, win=None, full=False, area=0.0, cls="Empty", nr=0, ng=0, occ=1)
                cw = column_window(self.bev.H_inv, pose, sec, seats, margin)
                area = column_sign_area(self.bev.H_inv, pose, sec, seats, margin)
                if cw is not None and area is not None:
                    win, full = cw
                    r.update(win=win, full=full, area=area, occ=self._col_occluded(conf, empty, pose, sec, col, win, dist))
                    r["cls"], r["nr"], r["ng"] = classify_window(self.frame, win, area, self.color_ranges)
                res.append(r)
        return res

    def _log_columns(self, cols, pose) -> None:
        """SEAT_ROI_LOG: [COL] por columna. Solo medición."""
        dump_dir = getattr(C, "SEAT_ROI_DUMP_DIR", None)
        marks = [] if (dump_dir and self._roi_t % 15 == 0 and self._dump_n < 80) else None
        for r in cols:
            print(f"[COL] t={self._roi_t} sec={r['sec']} col={r['col']} d={r['d']:.0f} b={r['b']:.1f} "
                  f"yaw={r['yaw']:.2f} cls={r['cls']} nr={r['nr']} ng={r['ng']} area={r['area']:.0f} "
                  f"occ={r['occ']} unk={r['unk']}", flush=True)
            if marks is not None and r["win"] is not None:
                marks.append((r["win"], r["occ"], f"{r['sec']} {r['col']} {r['d']:.0f} {r['cls']}"))
        if marks is not None:
            self._dump_frame(dump_dir, marks, pose)

    def _col_gate(self, cols, conf, empty) -> None:
        """ROUTE_SEAT_ROI: una columna con la misma clase SEAT_ROI_FRAMES frames seguidos (y gates OK)
        marca sus asientos sin resolver como vacíos (Empty) o guarda su color (Red/Green)."""
        n_need = int(getattr(C, "SEAT_ROI_FRAMES", 3))
        max_mm = float(getattr(C, "SEAT_ROI_USE_MAX_MM", 1800.0))
        max_yaw = float(getattr(C, "SEAT_ROI_MAX_YAW", 1.0))
        max_b = float(getattr(C, "SEAT_ROI_MAX_BEARING", 40.0))
        run = {}                                  # contadores de este frame: un frame sin gates los reinicia
        for r in cols:
            key = (r["sec"], r["col"])
            if not (r["full"] and r["occ"] == 0 and r["d"] <= max_mm and abs(r["yaw"]) <= max_yaw
                    and abs(r["b"]) <= max_b and r["unk"] > 0 and r["cls"] in ("Red", "Green", "Empty")):
                continue
            prev = self._col_run.get(key)
            n = prev[1] + 1 if prev is not None and prev[0] == r["cls"] else 1
            run[key] = (r["cls"], n)
            if n < n_need:
                continue
            if r["cls"] == "Empty":
                self.empty.update((r["sec"], t) for t in r["seats"] if (r["sec"], t) not in conf and (r["sec"], t) not in empty)
                self.roi_color.pop(key, None)
            elif getattr(C, "SEAT_ROI_COLOR", False):
                self.roi_color[key] = r["cls"]
        self._col_run = run

    def _dump_frame(self, dump_dir: str, marks, pose) -> None:
        """SEAT_ROI_DUMP_DIR: frame con las ventanas de columna y el pie de cada asiento (diagnóstico)."""
        from .seat_roi import seat_span
        os.makedirs(dump_dir, exist_ok=True)
        img = self.frame.copy()
        for key in self._all_seats():
            sx, sy = section_to_world(key[0], *SEATS[key[1]])
            span = seat_span(self.bev.H_inv, pose, sx, sy)
            if span is not None:
                (u0, _v0, u1, v1), _w = span
                cv2.circle(img, (int((u0 + u1) / 2), int(v1)), 3, (255, 255, 0), -1)
        for (x0, y0, x1, y1), occ, label in marks:
            color = (128, 128, 128) if occ else (0, 255, 255)
            cv2.rectangle(img, (x0, y0), (x1, y1), color, 1)
            cv2.putText(img, label, (x0, max(y0 - 2, 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
        cv2.imwrite(os.path.join(dump_dir, f"f{self._roi_t}.png"), img)
        cv2.imwrite(os.path.join(dump_dir, f"f{self._roi_t}_raw.png"), self.frame)
        self._dump_n += 1

    # ── paso por frame ────────────────────────────────────────────────────
    def step(self, dm, est: str | None, ack: str | None, armed: bool):
        """(obs normalizado o None, sufijo del V2, línea de subida o None)."""
        self._step_frame += 1
        tc = _ack_num(ack or "", "tc")
        replan_frames = getattr(C, "ROUTE_L23_ENTRY_REPLAN_FRAMES", None)
        if tc == 4 and self._l23_entry_cap is None:
            self._l23_entry_cap = replan_frames is None or (
                self._plan_frame is not None and self._step_frame - self._plan_frame <= replan_frames
            )
        self._rv_last = 255 if self.rv is None else self.rv   # último mando (ROUTE_UPLOAD_RAMP_PWM)
        self.rv = None
        ack_est = _ack_est(ack)
        if (C.LOC_MANEUVER_VIEW and armed and self.fixed and self.loc is not None
                and self.loc.odom is not None and ack_est in ("E", "G")):
            if self.maneuver_loc is None:
                # El ESP usa la transformación de ruta también dentro de E.
                self.maneuver_loc = copy(self.loc)
                self.maneuver_loc.x = self.loc.x.copy()
                self.maneuver_loc.P = self.loc.P.copy()
                self.maneuver_loc.last = {}
                self.maneuver_loc.n_used = self.maneuver_loc.n_rej = 0
            self.maneuver_loc.update_maneuver(ack)
        elif not armed or not C.LOC_MANEUVER_VIEW or ack_est is not None:
            self.maneuver_loc = None
        if not (armed and getattr(C, "ROUTE_FOLLOW", False)):
            return None, "", None
        if est not in ("S", "C", "R"):
            return None, "", None
        if dm.direction is None or dm.in_stall or dm._odom0 is None:
            return None, "", None
        lap1 = bool(getattr(C, "ROUTE_LAP1", True))
        conf, empty = self._resolved(dm)
        if not lap1 and len({s for (s, _t) in conf}) < 4:
            return None, "", None

        if self.loc is None:
            init = self._init_pose(dm, ack)
            if init is None:
                return None, "", None
            from .wro_field import w_axis
            self.park_w = float(getattr(dm, "park_w", PARK_W_MIN))
            self.axis = w_axis(dm.parking)
            if not self.fixed:
                self.fix = StartFix(dm.parking, dm.direction, self.park_w)
            self.loc = Localizer(conf, dm.parking, *init, park_w=self.park_w)
        # En maniobras (INICIO, giros del ESP) solo predicción: cerca de las
        # maderas y girando rápido una lectura mal asignada giraba el rumbo.
        # Sin el avance fijado, solo lecturas que no dependen de él.
        shifts = None
        if not self.fixed:
            ax, ay = self.axis
            shifts = [(float(d) * ax, float(d) * ay) for d in (self.fix.deltas[0], self.fix.deltas[-1])]
        # Parado en el cajón la pose del arranque es exacta: un ToF a 84 mm corría y +16 mm (gemelo s26).
        quieto = (getattr(C, "ROUTE_LOC_HOLD_AT_START", None) and not self.fixed
                  and _ack_num(ack or "", "px") == 0 and _ack_num(ack or "", "py") == 0)
        if getattr(C, "LOC_TOF_SIGN_MAX_MM", None) is not None:
            self.loc.set_anchors(dm.confirmed)   # solo lo que el mapa confirmó: los fantasmas de fila no anclan
        self.loc.update(ack, correct=(est in ("S", "C", "R") and ack_est in ("S", "C", "R") and not quieto),
                        shifts=shifts)
        pose = self.loc.pose() if self.loc.odom is not None else (dm.pose_xy[0], dm.pose_xy[1], float(dm.heading))
        if self.loc.odom is not None:
            self._sync_map(dm, ack, pose)
        if not self.fixed:
            self._pre_poses.append(pose)
            # Pegado a algo (frontal <= ROUTE_FIX_STUCK_DF_CM): las llantas patinan y la odometría avanza
            # sin moverse; en la corrida 13:26 así ubicó el cajón 247 mm mal. Esas lecturas no cuentan.
            d_fx = _ack_num(ack or "", "dF")
            stuck_cm = getattr(C, "ROUTE_FIX_STUCK_DF_CM", None)
            if not (stuck_cm is not None and d_fx is not None and 0.0 < d_fx <= float(stuck_cm)):
                self.fix.update(ack, pose)
                if getattr(C, "ROUTE_FIX_LINE", None): self._line_vote(dm, pose)
            delta = self.fix.converged()
            tol = getattr(C, "ROUTE_FIX_LINE_TOL_MM", None)
            if (delta is not None and tol is not None and getattr(C, "ROUTE_FIX_LINE", None)
                    and len(self._line_d) >= int(getattr(C, "ROUTE_FIX_LINE_MIN", 3))
                    and abs(float(delta) - self._line_median()) > float(tol)):
                delta = None   # la masa converge a un pico que la línea contradice: seguir leyendo
            d_f = _ack_num(ack or "", "dF")
            if (delta is not None and len(self._line_d) < int(getattr(C, "ROUTE_FIX_LINE_MIN", 3))
                    and (d_f is None or d_f > 100.0)):
                # Sin la línea, el pico de antes de la pared del fondo es de señales
                # (n3 s24: δ=-87 a dF=200 y cruza E/T3; la base cierra con dF=97).
                delta = None
            if (delta is None and d_f is not None and d_f <= 100.0 and len(self._line_d) >= 8
                    and float(np.max(self._line_d) - np.min(self._line_d)) <= 40.0
                    and self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))[1] < 0.5
                and self._line_ok(self._line_median())):
                # Recta corta: dF ya < 100 y el sonar sigue plano, así que el tope 105
                # se come el tramo (n3 s23). Si el pico ya pesa, se espera al sonar
                # (n3 s24: la línea a n=8 iba 32 mm corrida).
                delta = self._line_median()
            edge = getattr(C, "ROUTE_FIX_EDGE_MM", None)
            if (delta is not None and edge is not None and len(self._line_d) < int(getattr(C, "ROUTE_FIX_LINE_MIN", 3))
                    and (delta - (PARK_W_MIN - self.fix.w0_guess) < float(edge)
                         or (PARK_W_MAX - self.fix.w0_guess) - delta < float(edge))):
                # Pico pegado al extremo del rango y sin la línea: una señal en la pared supuesta (s894: -691, verdad -299).
                delta = None
            n_dec = getattr(C, "ROUTE_FIX_LINE_DECIDE_N", None)
            if (delta is None and n_dec is not None and getattr(C, "ROUTE_FIX_LINE", None)
                    and len(self._line_d) >= int(n_dec)):
                # La cinta ya votó varias veces y cuadra: decide sin esperar al final de la recta.
                last = self._line_d[-int(n_dec):]
                q1, q3 = np.percentile(last, [25, 75])
                # Votos que derivan (gemelo s33: -369..-465 en 5 frames, verdad -568) pasan el IQR: también el rango.
                span = getattr(C, "ROUTE_FIX_LINE_DECIDE_SPAN_MM", None)
                if (q3 - q1 <= float(getattr(C, "ROUTE_FIX_LINE_DECIDE_IQR_MM", 60.0))
                        and (span is None or max(last) - min(last) <= float(span))
                        and self._line_ok(float(np.median(last)))):
                    delta = self._line_median(last=int(n_dec))
            seat_delta = None
            if (delta is None and getattr(C, "ROUTE_FIX_UNIQUE_PREFIX_SEAT", False)
                    and dm.direction == "CCW" and ack_est == "S" and tc == 0
                    and self._pix_blind_heading_ok(pose, 25.0)):
                seat_delta = self._unique_prefix_seat_delta(dm, pose)
                delta = seat_delta
            if (delta is None and getattr(C, "ROUTE_FIX_SEAT_ROWS", False)
                    and dm.direction == "CCW" and ack_est == "S" and tc == 0
                    and self._pix_blind_heading_ok(pose, 25.0)):
                # Como el asiento único, pero cruzando también las señales de la recta siguiente.
                seat_delta = self._seat_rows_delta(dm, pose)
                delta = seat_delta
            forced = False
            if delta is None and self._past_straight(dm, pose) and not self._force_wait(pose):
                # Sin pico al final de la recta: la media (sin lecturas que
                # decidan, el histograma queda plano cerca de lo supuesto).
                delta = self.fix.mean()
                if (getattr(C, "ROUTE_FIX_LINE", None) and len(self._line_d) >= int(getattr(C, "ROUTE_FIX_LINE_MIN", 3))
                        and not self._line_ok(self._line_median())):
                    # La cinta contradice a los ToF: el pico de los ToF (corridas 100/101: cinta -565, ToF ~-150).
                    pk, pm = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))
                    delta = pk if pm >= 0.2 else delta
                elif getattr(C, "ROUTE_FIX_LINE", None) and len(self._line_d) >= int(getattr(C, "ROUTE_FIX_LINE_MIN", 3)):
                    delta = self._line_median()
                    # 1-2 votos lejanos de la cinta van ~+60 mm corridos y el pico del sonar del otro lado
                    # (gemelo s9: cinta +649, pico +546 m=0.62, verdad +587; s22: +83 / -10, verdad +17).
                    blend = getattr(C, "ROUTE_FIX_FORCE_BLEND_MM", None)
                    pk, pm = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))
                    if (blend is not None and len(self._line_d) < 3 and pm >= 0.2
                            and abs(pk - delta) <= float(blend)):
                        delta = 0.5 * (pk + delta)
                forced = True
                cap = getattr(C, "ROUTE_FIX_FORCE_MAX_NO_LINE_MM", None)
                if (cap is not None and not self._line_d
                        and abs(float(delta)) > float(cap)):
                    delta = None
                    forced = False
                # Sin cinta y con el histograma plano la media es el centro del rango (n2 s36: -158, verdad -319):
                # esperar evidencia un rato (el carro avanza lento, ver ROUTE_FIX_PENDING_CREEP_PWM).
                pend = getattr(C, "ROUTE_FIX_PENDING_FRAMES", None)
                pk, pm = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))
                edge = float(getattr(C, "ROUTE_FIX_EDGE_MM", None) or 30.0)
                # pico pegado a un extremo del rango = señal en la pared supuesta (n2 s36: +21 con masa 0.57, verdad -319)
                flat = (pm < float(getattr(C, "ROUTE_FIX_PENDING_MASS", 0.1))
                        or pk - (PARK_W_MIN - self.fix.w0_guess) < edge or (PARK_W_MAX - self.fix.w0_guess) - pk < edge)
                if pend is not None and not self._line_d and flat and self._pend_n < int(pend):
                    self._pend_n += 1
                    delta = None
            if delta is not None:
                if C.ROUTE_FIX_PILLAR_PAIR and len(self._line_d) >= 3:
                    p_shift = 3 * SIGN_MM
                    if getattr(C, "ROUTE_FIX_LINE_VOTE_REJECT", False):
                        if self._recta_vote_span() >= float(getattr(C, "ROUTE_FIX_LINE_RECTA_DRIFT_MM", 40.0)):
                            p_shift = SIGN_MM
                    delta = self._pillar_pair_delta(dm, delta, p_shift)
                # El pie proyectado del pilar adelanta ligeramente el avance
                # estimado en CCW. Compensa el sesgo de la madera por separado.
                park_bias = 15.0 if seat_delta is not None else 0.0
                pose = self._apply_fix(dm, ack, delta, forced, park_bias=park_bias)
                conf, empty = self._resolved(dm)
        elif (getattr(C, "ROUTE_FIX_BLUE", False) and not self._post_blue_done
              and tc is not None and tc <= 0):
            self._post_blue(dm, ack, pose)
            pose = self.loc.pose()
        if C.ROUTE_PARK_REFINE and self.fixed and tc is not None and tc <= 11:
            if self._park_refine is None:
                self._park_refine = ParkRefine(dm.parking, dm.direction, self.park_w)
            new_return = (_ack_est(ack) in ("S", "C", "R")
                          and self._park_refine.update(tc, ack, pose, self.loc))
            # Use third-pass returns while the ESP is still following the
            # route at tc=11. The firmware may start parking during this turn.
            if tc == 11 and (new_return or self._park_refined_w is None):
                result = self._park_refine.estimate()
                # The prior's p90 was about 43 mm. Only act on large disagreements.
                if result is None or abs(result[0] - self.park_w) < 70:
                    result = (self.park_w, 0)
                if result[0] != self._park_refined_w:
                    self._park_refined_w, passes = result
                    print(f"[PARKREF] w0={self._park_refined_w:.0f} p0={self.park_w:.0f} "
                          f"passes={passes} tc={tc:.0f}", flush=True)
        if lap1 and self.fixed:
            self._update_views(conf, empty, pose)
            conf, empty = self._resolved(dm)
        if lap1 and self._seat_roi(conf, empty, pose):
            conf, empty = self._resolved(dm)
        unknown = [k for k in self._all_seats() if k not in conf and k not in empty]
        # Color provisional: el mapa confirma ya cerca (la detección tiene que
        # caer a <=160 mm del asiento); los votos de color llegan antes. Con
        # >=2 votos y ventaja, la ruta ya toma el lado de ese color (semilla 87:
        # rojo confirmado a 585 mm, la ruta lo rodeaba por el lado malo).
        # Sin votos, el color de la ROI de la columna (ROUTE_SEAT_ROI); los votos ganan.
        prov = {}
        roi_seats = set()
        nmin = int(getattr(C, "ROUTE_PROVISIONAL_VOTES", 2))
        for k in unknown:
            v = getattr(dm, "_votes", {}).get(k)
            if v:
                for col, oth in (("Red", "Green"), ("Green", "Red")):
                    if v.get(col, 0) >= nmin and v.get(col, 0) >= v.get(oth, 0) + 1:
                        prov[k] = col
            rc = self.roi_color.get((k[0], _COL[k[1]]))
            if rc is not None:
                roi_seats.add(k)
                prov.setdefault(k, rc)
        plan_seats = {**conf, **prov}
        plan_unknown = [k for k in unknown if k not in prov]
        if getattr(C, "ROUTE_PROV_RULES", False) and prov:
            rule = self._rule_empty(plan_seats)
            plan_unknown = [k for k in plan_unknown if k not in rule]
        # Medio vista: la fila (exterior/interior, 200 mm) es lo que la visión confunde (N/T3<->N/T1,
        # S/T4<->S/T2, E/X1<->E/X2), la columna no. Las filas posibles salen de dónde cayeron las
        # detecciones (mediana lateral, DigitalMap.row_candidates): cerca de una fila, solo esa; entre
        # las dos, se esquivan ambas con el mismo color (por regla la otra está vacía: nunca dos en la
        # misma columna). Las reglas de la recta, contadas con una sola señal, descartan la X de en
        # medio. Nada de eso frena: la ruta ya pasa por el lado bueno sea cual sea la fila.
        half_seats = set()
        if getattr(C, "ROUTE_HALF_SEEN", False) and prov:
            singles = {**conf, **{k: c for k, c in prov.items() if k not in roi_seats}}
            rule = self._rule_empty(singles)
            ghosts = {}
            tol = float(getattr(C, "ROUTE_HALF_ROW_TOL_MM", 150.0))
            rows = getattr(dm, "row_candidates", None)
            for (sec, seat), col in prov.items():
                if (sec, seat) in roi_seats:
                    continue
                half_seats.add((sec, seat))
                cands = rows(sec, seat, col, tol) if rows is not None else [seat]
                for other in cands:
                    twin_ = (sec, other)
                    if other == seat or twin_ in conf or twin_ in empty or prov.get(twin_, col) != col:
                        continue
                    ghosts[twin_] = col
                    half_seats.add(twin_)
            half_seats |= {k for k in rule if k[0] in {s for s, _ in half_seats}}
            plan_seats = {**plan_seats, **ghosts}
            plan_unknown = [k for k in plan_unknown if k not in half_seats]
        if getattr(C, "ROUTE_MAP_FREEZE", False):
            # Vueltas 2-3 (tc >= 4): lo que ya tenía color (confirmado, provisional o fantasma de fila) no vuelve a vacío ni
            # a desconocido para el planificador; solo cambia de color si el mapa actual lo dice. Un solo punto: da igual por qué se vació.
            tcn = max(float(getattr(self, "_tc_max", 0) or 0), float(_ack_num(ack or "", "tc") or 0))
            # Desde ROUTE_MAP_FREEZE_TC (con el cajón ya ubicado). orillas135/137: medio segundo antes de la esquina 1 el
            # rojo provisional de S/T1 bajó de votos, la ruta lo soltó y planeó encima; al volver ya no daba para rodearlo.
            if tcn >= float(getattr(C, "ROUTE_MAP_FREEZE_TC", 4)) and self.fixed:
                fz = self.__dict__.setdefault("_map_freeze", {})
                for k, col in plan_seats.items():
                    fz[k] = col
                back = {k: col for k, col in fz.items() if k not in plan_seats and (k[0] != dm.parking or getattr(C, "ROUTE_MAP_FREEZE_PARK", False))}
                if back:
                    plan_seats = {**plan_seats, **back}
                    plan_unknown = [k for k in plan_unknown if k not in back]
        if getattr(C, "ROUTE_COLUMN_CONFLICT", False):
            # Por regla nunca hay dos señales en la misma columna (T2/T4, T1/T3, X1/X2). Un provisional o fantasma
            # que contradice a la confirmada de su columna toma el color de la confirmada (run 182: S/T2 Green
            # provisional + S/T4 Red confirmado -> plan infactible con isla=37 y el carro chocó con la isla).
            twin_of = {"T1": "T3", "T3": "T1", "T2": "T4", "T4": "T2", "X1": "X2", "X2": "X1"}
            fixd = {}
            for (sec, seat), col in plan_seats.items():
                if (sec, seat) in conf or seat not in twin_of:
                    continue
                cc = conf.get((sec, twin_of[seat]))
                if cc is not None and cc != col:
                    fixd[(sec, seat)] = cc
            if fixd:
                if fixd != getattr(self, "_col_fix_prev", None):
                    print("[ROUTE] columna contradice a la confirmada: " + " ".join(f"{s}/{o}->{c}" for (s, o), c in sorted(fixd.items())), flush=True)
                plan_seats = {**plan_seats, **fixd}
            self._col_fix_prev = fixd
        self._ghost_fb = None
        gtc = getattr(C, "ROUTE_CONF_GHOST_TC", None)
        if gtc is not None and float(getattr(self, "_tc_max", 0) or 0) < float(gtc) and getattr(dm, "row_candidates", None):
            # Vuelta 1: una señal confirmada con la pose aún corrida (W/T1 por W/T3, s35) puede estar en la otra fila de su
            # columna; si la mediana lateral observada también cae cerca de ella, la ruta esquiva las dos con el mismo color.
            ghosts2 = {}
            for (sec, seat), col in conf.items():
                for other in dm.row_candidates(sec, seat, col, float(getattr(C, "ROUTE_CONF_GHOST_TOL_MM", 150.0))):
                    tw = (sec, other)
                    if other != seat and tw not in conf and tw not in plan_seats:
                        ghosts2[tw] = col
            hold_mm = getattr(C, "ROUTE_GHOST_HOLD_MM", None)
            if (hold_mm is not None and self.fixed
                    and float(getattr(self, "_tc_max", 0) or 0) >= float(getattr(C, "ROUTE_GHOST_HOLD_TC", 4))):
                # Cerca del carro el fantasma de fila no entra ni sale (run 180: N/X1R parpadeó 4 veces en 0.4 s
                # a media recta; cada replan subía la ruta con tope de PWM y el volantazo frenando corrió la pose).
                prev = getattr(self, "_ghost2_prev", {})
                def _near(k):
                    sx, sy = section_to_world(k[0], *SEATS[k[1]])
                    return math.hypot(sx - pose[0], sy - pose[1]) < float(hold_mm)
                g = {k: c for k, c in ghosts2.items() if k in prev or not _near(k)}
                g.update({k: c for k, c in prev.items()
                          if k not in g and k not in conf and k not in plan_seats and _near(k)})
                if set(g) != set(ghosts2):
                    print(f"[ROUTE] fantasma retenido cerca: " + " ".join(f"{s}/{o}" for s, o in sorted(set(g) ^ set(ghosts2))), flush=True)
                ghosts2 = g
            self._ghost2_prev = dict(ghosts2)
            if ghosts2:
                self._ghost_fb = ({k: v for k, v in plan_seats.items()}, list(plan_unknown))   # sin fantasmas, por si el plan sale infactible
                half_seats |= set(ghosts2)
                plan_seats = {**plan_seats, **ghosts2}
                plan_unknown = [k for k in plan_unknown if k not in ghosts2]
        ctc = getattr(C, "ROUTE_COL_GHOST_TC", None)
        if ctc is not None and float(getattr(self, "_tc_max", 0) or 0) < float(ctc) and getattr(dm, "stray_seats", None):
            # Detecciones que ninguna señal conocida explica (orillas110/111: el verde solo de E/X1 visto entrando
            # a la esquina caía entre T2 y X1, visto en la recta caía en T1; ganó T1, la carta vació X1 y la ruta le
            # pasó encima en la vuelta 2): los asientos de su fila cerca de ese grupo se esquivan con su color.
            tol = float(getattr(C, "ROUTE_COL_GHOST_TOL_MM", 320.0))
            ghosts3 = {k: c for k, c in dm.stray_seats({**conf, **prov}, tol, float(getattr(C, "ROUTE_COL_GHOST_FAR_MM", 250.0))).items()
                       if k not in plan_seats}
            if ghosts3:
                if self._ghost_fb is None:
                    self._ghost_fb = ({k: v for k, v in plan_seats.items()}, list(plan_unknown))
                if ghosts3 != getattr(self, "_col_ghosts_prev", None):
                    print(f"[ROUTE] columna medio vista: " + " ".join(f"{s}/{o}={c[0]}" for (s, o), c in sorted(ghosts3.items())), flush=True)
                half_seats |= set(ghosts3)
                plan_seats = {**plan_seats, **ghosts3}
                plan_unknown = [k for k in plan_unknown if k not in ghosts3]
            self._col_ghosts_prev = ghosts3
        extra, extra_key, anchor_replan = None, None, False
        if not self.fixed:
            # Recta del cajón sin el avance: sus asientos no se pueden ubicar;
            # se planea con lo que la cámara ve relativo al carro.
            plan_seats = {k: v for k, v in plan_seats.items() if k[0] != dm.parking}
            plan_unknown = [k for k in plan_unknown if k[0] != dm.parking]
            extra, extra_key = self._prefix_extras(dm, pose)
            if self._pre_key is not None and extra_key != self._pre_key:
                anchor_replan = True
            self._pre_key = extra_key

        hug = None
        tcf = _ack_num(ack or "", "tc")
        if tcf is not None:
            self._tc_max = max(getattr(self, "_tc_max", 0), tcf)   # el ack no llega en todos los cuadros: la ruta del final no se apaga
        tcf = getattr(self, "_tc_max", None)
        if (self.fixed and self.park_w is not None and tcf is not None
                and tcf >= int(getattr(C, "ROUTE_FINAL_TC", 11)) and getattr(C, "ROUTE_FINAL_HUG", True)):
            # Final sin U: la ruta lleva al carro a la pared del cajón antes de la primera madera.
            pk = _stall_far_along(dm.parking, dm.direction, self.park_w)
            if True:
              hug = (float(getattr(C, "ROUTE_FINAL_D_MM", 640.0)), pk - 304.0 - float(getattr(C, "ROUTE_FINAL_LEAD_MM", 1300.0)),
                   pk, float(getattr(C, "ROUTE_FINAL_W", 20.0)))
        key = (dm.direction, dm.parking, self.park_w, self.shift,
               tuple(sorted(plan_seats.items())), tuple(sorted(plan_unknown)), extra_key, hug)
        if key != self.key and key != self._job_key:
            # Continuidad: la ruta nueva empata EXACTO con la anterior en las
            # estaciones de adelante del carro (por ahí va mientras sube al
            # ESP); el primer plan sale de la pose del carro con su rumbo. Con
            # el marco recién corregido o la señal de la recta del cajón recién
            # vista, también desde la pose (la anterior ya no sirve).
            lock = None
            anchor = (pose[0], pose[1], pose[2])
            if (self.route is not None and getattr(self.route, "d_stations", None) is not None
                    and self.route_shift == self.shift and not anchor_replan):
                k0 = RR.station_of(self.route, pose[0], pose[1])
                nst = len(self.route.d_stations)
                nl = int(getattr(C, "ROUTE_LOCK_STATIONS", 7))
                lead = getattr(C, "ROUTE_LOCK_LEAD_S", None)
                if lead is not None:
                    # El plan se adopta ~lead s despues: el prefijo fijo empieza donde estara el carro, no donde esta.
                    v = float(C.ROUTE_PROFILE_K_MM_S_PER_PWM) * ((255 if self.rv is None else self.rv) - float(C.ROUTE_PROFILE_PWM0))
                    k0 = (k0 + int(round(max(v, 0.0) * float(lead) / (self.route.length_mm / nst)))) % nst
                lock = {(k0 + j) % nst: float(self.route.d_stations[(k0 + j) % nst]) for j in range(nl)}
                anchor = None
            args = (dm.direction, plan_seats, dm.parking, plan_unknown, anchor, lock, self.park_w, extra, hug)
            plan_kwargs = {"fb": self._ghost_fb, "tc": getattr(self, "_tc_max", 0)}
            hold = anchor_replan and self.route is not None
            if getattr(C, "ROUTE_PLAN_ASYNC", False):
                # Carro real: la DP tarda ~1-2 s en la Pi; corre aparte y el
                # carro sigue la ruta anterior (lento) mientras tanto.
                if self._job is None:
                    import threading
                    self._job_key = key
                    self._job_out = None
                    self._job_shift = self.shift
                    self._job_hold = hold
                    self._job = threading.Thread(target=self._run_plan, args=args, kwargs=plan_kwargs, daemon=True)
                    self._job.start()
            elif getattr(C, "ROUTE_PLAN_DELAY_S", None) is not None and self.route is not None:
                # Twin: el plan sale ya, pero se adopta tras el retraso de la Pi (ruta anterior mientras).
                if self._delay_left is None:
                    self._job_key = key
                    route, seats, unk = self._solve_plan(*args, **plan_kwargs)
                    self._delay_plan = (route, key, dict(seats), list(unk), self.shift, hold)
                    self._delay_left = int(round(float(C.ROUTE_PLAN_DELAY_S) * float(getattr(C, "PI_FPS", 40.0))))
            else:
                route, seats, unk = self._solve_plan(*args, **plan_kwargs)
                self._adopt(route, key, seats, unk, empty, dm, self.shift, hold)
        if self._delay_left is not None:
            self._delay_left -= 1
            if self._delay_left <= 0:
                route, k, seats, unk, sh, hd_ = self._delay_plan
                self._adopt(route, k, seats, unk, empty, dm, sh, hd_)
                self._delay_left = None
                self._delay_plan = None
                self._job_key = None
        if self._job is not None and not self._job.is_alive():
            if self._job_out is not None:
                self._adopt(self._job_out, self._job_key, dict(self._job_conf), self._job_unknown, empty, dm,
                            self._job_shift, getattr(self, "_job_hold", False))
            self._job = None
            self._job_key = None
        if self.follower is None:
            return None, "", None

        x, y, hd = pose
        # La ruta de la Pi puede estar en el marco anterior (planeación en curso).
        ax, ay = self.axis
        ds = self.shift - self.route_shift
        wheel = self.follower.steer(x - ds * ax, y - ds * ay, hd)
        self.pi_wheel = wheel
        servo = wheel * float(C.SERVO_FULL_LOCK_CMD_DEG) / float(C.MAX_WHEEL_STEER_DEG)
        obs = max(-1.0, min(1.0, servo / C.PP_STEER_GAIN))
        slow = int(getattr(C, "ROUTE_SLOW_PWM", 100))
        if self.fixed:
            # Los asientos con color por ROI ya tienen lado: no frenan (ni cuentan para la parada).
            speed_unknown = [k for k in unknown if k not in roi_seats and k not in half_seats]
            # Después de la vuelta 1 (tc >= ROUTE_UNSEEN_MAX_TC) lo que falta no frena: la ruta ya lo
            # esquiva como obstáculo (orillas48: N/T3 sin confirmar hasta la vuelta 3, PWM 130 en cada pasada).
            _max_tc = getattr(C, "ROUTE_UNSEEN_MAX_TC", None)
            _past = _max_tc is not None and float(getattr(self, "_tc_max", 0) or 0) >= float(_max_tc)
            self.rv = self._speed(speed_unknown, (x - ds * ax, y - ds * ay, hd)) if (lap1 and not _past) else None
        else:
            # Sin el avance: lento. Con el frontal lejos de la pared del fondo
            # (dF >= 100 cm, sin eco = 200) el perfil manda; al verla, tope ciego.
            d_f = _ack_num(ack or "", "dF")
            if not (d_f is not None and d_f >= 100.0):
                blind = getattr(C, "ROUTE_BLIND_PWM", None)
                self.rv = slow if blind is None else int(blind)
            # 2026-11-09 (corrida 13:26): sin el cajón ubicado la cámara no entra al mapa y la ruta
            # cruzó por el primer verde a rv 216. Tope de salida y alto si el frontal ve algo cerca.
            pre = getattr(C, "ROUTE_PREFIX_PWM", None)
            if pre is not None:
                self.rv = int(pre) if self.rv is None else min(self.rv, int(pre))
            stop_cm = getattr(C, "ROUTE_PREFIX_STOP_CM", None)
            if stop_cm is not None and d_f is not None and 0.0 < d_f < float(stop_cm):
                self._pre_stop += 1
                cap = 0 if self._pre_stop <= int(getattr(C, "ROUTE_PREFIX_STOP_FRAMES", 20)) else slow
                self.rv = cap if self.rv is None else min(self.rv, cap)
            elif d_f is not None and d_f > 0.0:
                self._pre_stop = 0
        if abs(self.follower.cte) > float(getattr(C, "ROUTE_CTE_SLOW_MM", 40.0)):
            # lejos de la ruta (recién replaneada o perturbado): lento hasta volver
            cte_cap = _pwm("ROUTE_CTE_PWM")
            if getattr(C, "ROUTE_CTE_URGENT_MM", None) is not None and self._cte_urgent(conf, empty):
                if not self.fixed:
                    # 110 en vez de 105 corre ~2 cm y el cajón magenta (n4 s24).
                    if getattr(C, "ROUTE_BLIND_PWM", None) is None or float(C.ROUTE_BLIND_PWM) <= 110:
                        cte_cap = min(cte_cap, int(getattr(C, "ROUTE_CTE_URGENT_PWM", 120)))
                    b = getattr(C, "ROUTE_BLIND_PWM", None)
                    cte_cap = min(cte_cap, slow if b is None else int(b))
                elif (cte_urgent_holds(self.follower.cte, *_chord_ahead(self.follower), *self._offset_body(conf))
                      or _grip_pwm(wheel) < int(C.ROUTE_PROFILE_MAX_PWM)):
                    # Soltar el 110 a tope abre el estacionamiento: sin tope, n5 s21
                    # entra 9.8 mm; con el PWM del agarre, n4 s21 queda a 3.0 mm.
                    cte_cap = min(cte_cap, int(getattr(C, "ROUTE_CTE_URGENT_PWM", 120)))
            self.rv = cte_cap if self.rv is None else min(self.rv, cte_cap)
        l0cap = getattr(C, "ROUTE_L0_CTE_PWM", None)
        if l0cap is not None and self.fixed and tc is not None and float(tc) < 1.0 and abs(self.follower.cte) > float(getattr(C, "ROUTE_L0_CTE_MM", 40.0)):
            self.rv = int(l0cap) if self.rv is None else min(self.rv, int(l0cap))   # recién ubicado el cajón (tc=0): la pose saltó y el carro vuelve a la ruta nueva lento (n6 s33)
        self.pi_steer = (bool(self.upload) and not self.hold
                         and bool(getattr(C, "ROUTE_UPLOAD_PI_STEER", False)) and self._upload_risky())
        if self.upload:
            # Mientras la ruta nueva no llega completa al ESP, lento; detenido
            # si cambió desde la pose (otro carril cerca del carro).
            # ROUTE_UPLOAD_SLOW=False: el ESP sigue la ruta anterior completa
            # (doble buffer, swap en RTE), así que solo se frena en el ALTO.
            # pi_steer: el ESP gira con la Pi, sin tope de subida (pi_steer nunca va con hold: el ALTO sigue).
            if not self.pi_steer and (self.hold or (bool(getattr(C, "ROUTE_UPLOAD_SLOW", True)) and self._upload_risky())):
                _tc = _ack_num(ack or "", "tc")
                _l1u = getattr(C, "ROUTE_L1_UPLOAD_PWM", None)
                # vuelta 1 tras la primera esquina (1<=tc<4): tope propio mientras sube la ruta. En tc=0 no:
                # la ruta recién ubicado el cajón cambia de lado (n2 s33 chocó Green E/T1 a 160). U/parking igual.
                cap = 0 if self.hold else (int(_l1u) if _l1u is not None and _tc is not None and 1 <= _tc < 4 else _pwm("ROUTE_UPLOAD_PWM"))
                ramp = getattr(C, "ROUTE_UPLOAD_RAMP_PWM", None)
                if ramp is not None and cap > 0 and self._rv_last is not None:
                    # Sin frenón: el tope baja desde el último mando a ramp PWM por cuadro (el ESP sigue la ruta vieja).
                    cap = max(cap, int(self._rv_last) - int(ramp))
                self.rv = cap if self.rv is None else min(self.rv, cap)
        else:
            self.hold = False
        if getattr(C, "ROUTE_SWAP_PWM", None) is not None and self._swap_left > 0:
            # Recien cambio la ruta del ESP (RTE): el carro aun no la sigue bien (la DP/subida llegaron tarde).
            self._swap_left -= 1
            self.rv = int(C.ROUTE_SWAP_PWM) if self.rv is None else min(self.rv, int(C.ROUTE_SWAP_PWM))
        if getattr(C, "ROUTE_PENDING_PWM", None) is not None and (self._delay_left is not None or self._job is not None):
            # Plan en curso (la DP tarda en la Pi): la ruta vieja puede pasar por la señal recién vista.
            cap = int(C.ROUTE_PENDING_PWM)
            self.rv = cap if self.rv is None else min(self.rv, cap)
        # Perfil por curvatura: solo baja el tope (los de arriba siguen ganando).
        # Solo la vuelta 2: la 3 queda con el κ local para llegar al cajón como la base.
        corner_sign = getattr(C, "ROUTE_L1_CORNER_SIGN", None)
        can_release = (corner_sign and corner_sign.get("planned_clearance_resolved") and self.fixed
                       and 1 <= (tcf or 0) < 4 and self.esp_route is self.route
                       and self.route.report.get("feasible") and not self.upload and self._rte_n is None
                       and not plan_unknown and self.key == key)
        corner_points = []
        for sec, seat in unknown:
            xy = section_to_world(sec, *SEATS[seat])
            seat_key = (sec, seat)
            if can_release and seat_key in prov and self._conf_prev.get(seat_key) == prov[seat_key]:
                # El plan activo ya respeta el color provisional. Conservar el
                # freno si el error de seguimiento puede consumir su holgura.
                r = self.route
                gap = RR._rect_dist_pose(*xy, r.x, r.y, np.radians(r.heading_deg)).min()
                reserve = float(C.ROUTE_CORNER_EXIT_CLEAR_MM) + abs(self.follower.cte)
                if gap > SIGN_MM / math.sqrt(2.0) + reserve:
                    continue
            corner_points.append(xy)
        self._unk_pts = np.asarray(corner_points, dtype=float).reshape(-1, 2)
        prof = self._profile_pwm(
            self._sign_xy(conf, empty),
            lap2=tc is not None and 4 <= tc < int(getattr(C, "ROUTE_LAP2_PROFILE_TC_MAX", 8)),
            l23=tc is not None and 4 <= tc < 12,
            l1=tc is not None and 1 <= tc < 4,
            conf=conf,
            parking=dm.parking,
        )
        if prof is not None and tc == 4 and self._l23_entry_cap and getattr(C, "ROUTE_L23_ENTRY_PWM", None) is not None:
            prof = min(prof, int(C.ROUTE_L23_ENTRY_PWM))   # primer tramo de la vuelta 2 (tc=4): el carro aún llega desviado de la vuelta 1
        if prof is not None:
            self.rv = prof if self.rv is None else min(self.rv, prof)
        fpw = getattr(C, "ROUTE_FINAL_PWM", None)
        if fpw is not None and tc is not None and tc >= int(getattr(C, "ROUTE_FINAL_TC", 11)) and self.fixed:
            self.rv = int(fpw) if self.rv is None else min(self.rv, int(fpw))   # última recta hacia el cajón: tope propio (con el perfil pursuit hasta tc 12 llegaba rápido y chocaba el cajón)
        if self._prueba_cajon and self.fixed:
            # Prueba de salida (archivo ROUTE_PRUEBA_CAJON_FILE en la Pi): ubicado el
            # cajón se para y reporta la pose cada ~1 s, para medirla con cinta.
            self.rv = 0
            self._prueba_n += 1
            if self._prueba_n % 15 == 1:
                print(f"[PRUEBA] parado: pose x={x:.0f} y={y:.0f} rumbo={hd:.1f}  cajón w0={self.park_w:.0f}  "
                      f"dL={_ack_num(ack or '', 'dL')} dR={_ack_num(ack or '', 'dR')} dF={_ack_num(ack or '', 'dF')}",
                      flush=True)
        cr = getattr(C, "ROUTE_FIX_PENDING_CREEP_PWM", None)
        if cr is not None and not self.fixed and self._pend_n > 0 and self.rv is not None and self.rv < int(cr):
            d_f = _ack_num(ack or "", "dF")
            if d_f is None or d_f <= 0.0 or d_f >= float(getattr(C, "ROUTE_PREFIX_STOP_CM", 40.0)):
                self.rv = int(cr)   # parado sin evidencia del cajón: avanza lento hasta ver la cinta
        self.dbg = (round(x), round(y), round(hd, 1), round(wheel, 1), round(self.follower.cte),
                    self.follower.i, self.rv, len(unknown))
        # ¿El ESP aceptó la última ruta? Su ACK trae rn = puntos de la ruta activa. Si tras
        # ROUTE_UPLOAD_RETRY_FRAMES no cuadra con el RTE mandado, una línea se perdió en el UART
        # y el ESP sigue con la ruta vieja: se reenvía completa.
        retry = int(getattr(C, "ROUTE_UPLOAD_RETRY_FRAMES", 0) or 0)
        if retry > 0 and self._rte_n is not None and not self.upload:
            rn = _ack_num(ack or "", "rn")
            if rn is not None and int(rn) == self._rte_n:
                self._rte_n = None
            else:
                self._rte_wait += 1
                if self._rte_wait >= retry and self._upload_full:
                    self.n_retries += 1
                    print(f"[ROUTE] reenvío #{self.n_retries}: el ESP no confirmó la ruta "
                          f"(rn={rn} esperado {self._rte_n})", flush=True)
                    self.upload = list(self._upload_full)
                    self._rte_n = None
        lines = []
        if self.upload and getattr(C, "ROUTE_ON_ESP", False):
            # shortcut: >1 línea por frame pide subir el baud (a 115200 solo cabe una junto al V2).
            _nl = getattr(C, "ROUTE_UPLOAD_LINES", 1)
            _l1n = getattr(C, "ROUTE_L1_UPLOAD_LINES", None)   # vuelta 1 (tc<4): subida entera en menos cuadros
            if _l1n is not None and getattr(self, "_tc_max", 0) < 4:
                _nl = _l1n
            for _ in range(max(1, int(_nl))):
                if not self.upload:
                    break
                lines.append(self.upload.pop(0))
                if lines[-1].startswith("RTE,"):
                    self._rte_n = int(lines[-1].split(",")[1])
                    self._rte_wait = 0
        if any(l.startswith("RTE") for l in lines):
            sd = getattr(C, "ROUTE_SWAP_DIFF_MM", None)   # solo si la ruta nueva se aparta de la del ESP en el tramo de adelante
            self._swap_left = int(getattr(C, "ROUTE_SWAP_FRAMES", 0)) if sd is None or self._upload_risky(float(sd)) else 0
            if getattr(C, "ROUTE_SWAP_NEW_ONLY", False) and not getattr(self, "_new_sign", True):
                self._swap_left = 0
            self.esp_shift = self.upload_shift
            self.esp_route = self.route
        line = "\n".join(lines) if lines else None
        if self.fixed and self.park_w is not None:
            park_w = self.park_w if self._park_refined_w is None else self._park_refined_w
            self.pka = _stall_far_along(dm.parking, dm.direction, park_w)
        suffix = self._suffix() if getattr(C, "ROUTE_ON_ESP", False) else ""
        return obs, suffix, line

    def _upload_risky(self, tol=None) -> bool:
        """Mientras sube la ruta: ¿la nueva se aparta de la del ESP en el tramo de adelante?

        Sin ROUTE_UPLOAD_DIFF_MM, sin ruta previa en el ESP o en otro marco (arranque
        ciego), siempre True (frenar, como antes).
        """
        tol = getattr(C, "ROUTE_UPLOAD_DIFF_MM", None) if tol is None else tol
        old = self.esp_route
        if tol is None or old is None or self.esp_shift != self.upload_shift:
            return True
        r = self.follower.route
        n = len(r.x)
        m = int(float(getattr(C, "ROUTE_UPLOAD_HORIZON_MM", 700.0)) / (r.length_mm / n)) + 1
        j = (max(self.follower.i, 0) + np.arange(m)) % n
        d = np.hypot(r.x[j, None] - old.x[None, :], r.y[j, None] - old.y[None, :]).min(axis=1)
        return bool(d.max() > float(tol))

    def _cte_urgent(self, conf, empty) -> bool:
        """El carro, corrido |cte| de la ruta, va hacia la isla, una señal o la pared.

        Se corre la ruta de adelante (ROUTE_CTE_HORIZON_MM) el mismo cte hacia el
        lado del carro y se mide la holgura a la isla, a las señales posibles y a la
        pared. Urgente si en algún punto queda bajo ROUTE_CTE_URGENT_MM y más cerca
        (20 mm o más) que la ruta sin correr.
        """
        r = self.follower.route
        n = len(r.kappa)
        i = max(self.follower.i, 0)
        step = r.length_mm / n
        m = int(float(getattr(C, "ROUTE_CTE_HORIZON_MM", 600.0)) / step) + 1
        j = (i + np.arange(m)) % n
        h = np.radians(r.heading_deg[j])
        # Derecha de la ruta: (cos h, -sin h); cte > 0 = carro a la derecha.
        rx, ry = np.cos(h), -np.sin(h)
        cte = float(self.follower.cte)
        signs = self._sign_xy(conf, empty)

        def clearance(px, py):
            ih = float(INNER_HALF_MM)
            isl = np.hypot(np.maximum(np.abs(px) - ih, 0.0), np.maximum(np.abs(py) - ih, 0.0))
            wall = float(OUTER_HALF_MM) - np.maximum(np.abs(px), np.abs(py))
            c = np.minimum(isl, wall)
            if len(signs):
                ds = np.hypot(px[:, None] - signs[None, :, 0], py[:, None] - signs[None, :, 1]).min(axis=1)
                c = np.minimum(c, ds - 25.0)
            return c

        c0 = clearance(r.x[j], r.y[j])
        c1 = clearance(r.x[j] + cte * rx, r.y[j] + cte * ry)
        lim = float(C.ROUTE_CTE_URGENT_MM)
        return bool(np.any((c1 < lim) & (c1 < c0 - 20.0)))

    def _offset_body(self, conf) -> tuple[float, float, float]:
        """Holgura de la huella (pared, lata confirmada, isla), corrida el cte y sobre la ruta."""
        r = self.follower.route
        n = len(r.kappa)
        i = max(self.follower.i, 0)
        step = r.length_mm / n
        m = int(float(getattr(C, "ROUTE_CTE_HORIZON_MM", 600.0)) / step) + 1
        j = (i + np.arange(m)) % n
        h = np.radians(r.heading_deg[j])
        cte = float(self.follower.cte)
        rx, ry = np.cos(h), -np.sin(h)
        px = r.x[j] + cte * rx
        py = r.y[j] + cte * ry
        wall = island = 1e9
        ih = float(INNER_HALF_MM)
        for pts in (RR._corners_pose(px, py, h), RR._corners_pose(r.x[j], r.y[j], h)):
            for cx, cy in pts:
                wall = min(wall, float(OUTER_HALF_MM - np.maximum(np.abs(cx), np.abs(cy)).max()))
                isl = np.hypot(np.maximum(np.abs(cx) - ih, 0.0), np.maximum(np.abs(cy) - ih, 0.0))
                island = min(island, float(isl.min()))
        signs = np.asarray(
            [section_to_world(sec, *SEATS[seat]) for sec, seat in conf], dtype=float).reshape(-1, 2)
        if len(signs) == 0:
            sign = 1e9
        else:
            gap = RR._rect_dist_pose(signs[:, 0], signs[:, 1], px, py, h) - SIGN_MM * 0.5 * math.sqrt(2.0)
            sign = float(gap.min())
        return wall, sign, island

    def _sign_xy(self, conf, empty) -> np.ndarray:
        """Posiciones (x, y) en mm del tapete de las señales y asientos sin resolver.

        Todo asiento que no está marcado vacío: las confirmadas y las que aún no
        se saben (podrían ser señal). Mismo marco que la ruta (section_to_world).
        """
        pts = [section_to_world(sec, *SEATS[seat]) for sec, seat in self._all_seats()
               if (sec, seat) in conf or (sec, seat) not in empty]
        return np.asarray(pts, dtype=float).reshape(-1, 2)

    def _profile_pwm(self, signs=None, lap2=False, l23=False, l1=False, conf=None, parking=None) -> int | None:
        """PWM máximo según la curvatura por delante (perfil de velocidad).

        En cada muestra adelante: v_curva = sqrt(A_LAT/|κ|) acotada a [piso, max]
        (max en recta) y, para llegar a ella frenando con A_BRAKE desde la
        distancia d (menos la latencia), v_permitida = sqrt(v_curva² + 2·A·d).
        Las muestras a menos de ROUTE_PROFILE_SIGN_MM de una señal (signs, mm)
        tienen v_curva = base: se llega a la señal a velocidad base.
        Perfil v3 (ROUTE_PROFILE_MIN_PWM): el piso baja a MIN_PWM y, cerca de la
        señal, v_curva = min(v_curva, v_señal), donde v_señal sube lineal de MIN_PWM
        (a <= ROUTE_PROFILE_SIGN_NEAR_MM) a la base (en SIGN_MM); sin NEAR, la base.
        ROUTE_PROFILE_ISLAND_MM (si no es None): igual cerca de las 4 esquinas de la isla.
        ROUTE_PROFILE_BRAKE_FF_S (si no es None): al frenar hacia adelante pide menos (retraso del motor).
        ROUTE_PROFILE_TIGHT_KAPPA (si no es None): curvas más cerradas usan A_LAT_TIGHT.
        Manda la más lenta. None si el perfil está apagado o no hay ruta.
        """
        if not getattr(C, "ROUTE_PROFILE", False) or self.follower is None:
            return None
        r = self.follower.route
        n = len(r.kappa)
        K = float(C.ROUTE_PROFILE_K_MM_S_PER_PWM)
        pwm0 = float(C.ROUTE_PROFILE_PWM0)
        if l23 and getattr(C, "ROUTE_L23_K_MM_S_PER_PWM", None) is not None:
            K = float(C.ROUTE_L23_K_MM_S_PER_PWM)       # vueltas 2-3: el PWM lo lee el lazo del ESP (LAZO_K/PWM0_FWD)
            pwm0 = float(C.ROUTE_L23_PWM0)
        base = int(C.ROUTE_PROFILE_BASE_PWM)
        top = int(C.ROUTE_PROFILE_MAX_PWM)
        if l23 and getattr(C, "ROUTE_L23_MAX_PWM", None) is not None:
            top = int(C.ROUTE_L23_MAX_PWM)   # vueltas 2-3: techo propio en recta (None = el de la ruta)
        v_base = K * (base - pwm0)
        v_max = K * (top - pwm0)
        # Perfil v3: piso real del perfil (MIN_PWM, sin pasar de la base). None: la base.
        mn = getattr(C, "ROUTE_PROFILE_MIN_PWM", None)
        if l23 and getattr(C, "ROUTE_L23_MIN_PWM", None) is not None:
            mn = C.ROUTE_L23_MIN_PWM        # vueltas 2-3: piso del perfil propio (la 1 y la U no cambian)
        if l1 and getattr(C, "ROUTE_L1_MIN_PWM", None) is not None:
            mn = C.ROUTE_L1_MIN_PWM         # vuelta 1 (1<=tc<4): piso propio; salida, U y parking no cambian
        floor = base if mn is None else min(int(mn), base)
        v_min = K * (floor - pwm0)
        tight_on = False
        corner_floor = None
        a_lat = float(C.ROUTE_PROFILE_A_LAT_MM_S2)
        a_brk = float(C.ROUTE_PROFILE_A_BRAKE_MM_S2)
        if l23:   # vueltas 2-3: frenado propio (sin tope del lazo el carro llega a ~830 mm/s)
            a_brk = float(getattr(C, "ROUTE_L23_A_BRAKE_MM_S2", None) or a_brk)
            a_lat = float(getattr(C, "ROUTE_L23_A_LAT_MM_S2", None) or a_lat)
        win = int(C.ROUTE_PROFILE_KAPPA_WIN)
        # Adelanto por latencia a la velocidad máxima (conservador).
        lat_s = float(C.ROUTE_PROFILE_LATENCY_S)
        if l23 and getattr(C, "ROUTE_L23_LATENCY_S", None) is not None:
            lat_s = float(C.ROUTE_L23_LATENCY_S)   # vueltas 2-3: menos adelanto = suelta el freno antes
        lead = v_max * lat_s
        span = float(C.ROUTE_PROFILE_HORIZON_MM) + lead
        step = r.length_mm / n
        m = int(span / step) + 1
        i = max(self.follower.i, 0)
        # Anillo cerrado: índices módulo n. Ventana ±win para |κ| máximo móvil.
        idx = (i + np.arange(-win, m + win)) % n
        j = idx[win:win + m]
        if lap2:
            kmax = _pursuit_kappa(r, j, float(C.ROUTE_LOOKAHEAD_MM), getattr(C, "ROUTE_L23_LEFT_K_FACTOR", None) if (l23 and float(getattr(self, "_tc_max", 0) or 0) < float(getattr(C, "ROUTE_L23_LEFT_K_TC_MAX", 12))) else None)
        else:
            _k = r.kappa[idx]
            _f = getattr(C, "ROUTE_PROFILE_LEFT_K_FACTOR", None)   # kappa + = izquierda
            kmax = np.lib.stride_tricks.sliding_window_view(np.abs(_k) * (np.where(_k > 0.0, float(_f), 1.0) if _f else 1.0), 2 * win + 1).max(axis=1)
        dist = np.mod(r.s[j] - r.s[i], r.length_mm)
        d = np.maximum(dist - lead, 0.0)
        tight = getattr(C, "ROUTE_PROFILE_TIGHT_KAPPA", None)
        a_c = a_lat if tight is None else np.where(
            kmax > float(tight), float(getattr(C, "ROUTE_PROFILE_A_LAT_TIGHT_MM_S2", 1500.0)), a_lat)
        v_c = np.where(kmax < 1e-9, v_max,
                       np.clip(np.sqrt(a_c / np.maximum(kmax, 1e-12)), v_min, v_max))
        if signs is not None and len(signs):
            # Cerca de una señal: velocidad base (la frenada de arriba llega a ella a base).
            sign_mm = float(getattr(C, "ROUTE_PROFILE_SIGN_MM", 300.0))
            if l23 and getattr(C, "ROUTE_L23_SIGN_MM", None) is not None:
                sign_mm = float(C.ROUTE_L23_SIGN_MM)   # vueltas 2-3: zona lenta de la señal más corta
            pxy = np.stack((r.x[j], r.y[j]), axis=1)                       # (m, 2)
            dd = np.hypot(pxy[:, None, 0] - signs[None, :, 0],
                          pxy[:, None, 1] - signs[None, :, 1]).min(axis=1)  # (m,)
            if mn is None:
                v_c = np.where(dd < sign_mm, v_base, v_c)                  # igual que antes
            else:
                # Cerca de señal: nunca más que v_señal ni que la curva (ya no se fuerza a la base).
                near = getattr(C, "ROUTE_PROFILE_SIGN_NEAR_MM", None)
                if l23 and getattr(C, "ROUTE_L23_SIGN_NEAR_MM", None) is not None:
                    near = C.ROUTE_L23_SIGN_NEAR_MM
                if near is None:
                    v_sign = np.full_like(dd, v_base)
                else:
                    near = float(near)
                    f = np.clip((dd - near) / max(sign_mm - near, 1.0), 0.0, 1.0)
                    v_sign = v_min + (v_base - v_min) * f
                tight_mm = getattr(C, "ROUTE_PROFILE_SIGN_TIGHT_MM", None)
                if tight_mm is not None and near is not None:
                    # Paso muy justo (centro de ruta a centro de señal < NEAR): baja de MIN_PWM hasta TIGHT_PWM en TIGHT_MM.
                    tight_mm = float(tight_mm)
                    v_t = K * (float(getattr(C, "ROUTE_PROFILE_SIGN_TIGHT_PWM", 110)) - pwm0)
                    g = np.clip((dd - tight_mm) / max(near - tight_mm, 1.0), 0.0, 1.0)
                    v_sign = np.where(dd < near, v_t + (v_min - v_t) * g, v_sign)
                    tight_on = True
                v_c = np.where(dd < sign_mm, np.minimum(v_c, v_sign), v_c)
            corner_sign = getattr(C, "ROUTE_L1_CORNER_SIGN", None)
            if l23:
                corner_sign = getattr(C, "ROUTE_L23_CORNER_SIGN", None)
            elif not self.fixed or float(getattr(self, "_tc_max", 0) or 0) >= 4:
                corner_sign = None
            if corner_sign:
                # El piso normal del perfil permite ~700 mm/s incluso al salir
                # de la esquina hacia una señal cercana. Se limita solo esa
                # curva; el mismo sobre de frenado anticipa y suelta el freno.
                vg = K * (float(corner_sign["pwm"]) - pwm0)
                dd_c = dd
                if l23 and corner_sign.get("inside_exit_only"):
                    # Re-paso por el interior al salir de curva: seleccionar por
                    # lado reglamentario y posición longitudinal del asiento.
                    pts = []
                    exit_end = 1000.0 + float(corner_sign.get("exit_mm", 500.0))
                    for (sec, seat), color in (conf or {}).items():
                        if (color == "Red") != (r.direction == "CW"):
                            continue
                        xy = section_to_world(sec, *SEATS[seat])
                        along, _ = RR._sec(sec, r.direction).frame(*xy)
                        if 1000.0 <= along <= exit_end:
                            pts.append(xy)
                    if pts:
                        sx, sy = np.asarray(pts).T
                        dd_c = np.hypot(pxy[:, None, 0] - sx, pxy[:, None, 1] - sy).min(axis=1)
                    else:
                        dd_c = np.full_like(dd, np.inf)
                elif corner_sign.get("unk_only"):
                    # solo asientos aún sin resolver (la ruta puede cambiar de lado): los confirmados ya los traza la DP
                    u = getattr(self, "_unk_pts", None)
                    dd_c = np.full_like(dd, 1e9) if u is None or not len(u) else np.hypot(pxy[:, None, 0] - u[None, :, 0], pxy[:, None, 1] - u[None, :, 1]).min(axis=1)
                reach = float(corner_sign.get("mm", 350.0))
                if not l23 and corner_sign.get("selective") and parking:
                    # Preserve the original reach around the stall and around
                    # confirmed outer-row signs immediately after a corner.
                    risk = [section_to_world(parking, 0.0, self.park_w)]
                    for sec, seat in conf or ():
                        if seat.startswith("X"):
                            xy = section_to_world(sec, *SEATS[seat])
                            along, _ = RR._sec(sec, r.direction).frame(*xy)
                            if 1000.0 <= along <= 1500.0:
                                risk.append(xy)
                    sx, sy = np.asarray(risk).T
                    risk_mm = float(corner_sign.get("risk_mm", 550.0))
                    close = np.any(np.hypot(pxy[:, None, 0] - sx,
                                            pxy[:, None, 1] - sy) < risk_mm, axis=1)
                    if "other_pwm" in corner_sign:
                        other_v = K * (float(corner_sign["other_pwm"]) - pwm0)
                        vg = np.where(close, vg, other_v)
                    reach = np.where(close, reach, float(corner_sign.get("other_mm", 240.0)))
                near_curve = ((dd_c < reach)
                              & (kmax > 1.0 / float(corner_sign.get("radius_mm", 400.0))))
                if l23 and "a_lat_mm_s2" in corner_sign:
                    # Escalar el tope con la demanda lateral de la curva.
                    vg = np.maximum(vg, np.sqrt(float(corner_sign["a_lat_mm_s2"]) / np.maximum(kmax, 1e-12)))
                v_c = np.where(near_curve, np.minimum(v_c, vg), v_c)
                if np.any(near_curve):
                    corner_floor = int(corner_sign["pwm"])
            osm = getattr(C, "ROUTE_L23_OUTER_SIGN_MM", None)
            outer = signs[np.maximum(np.abs(signs[:, 0]), np.abs(signs[:, 1])) >= 1000.0] if (l23 and osm is not None) else ()
            if len(outer):
                # Señales de la fila exterior (X1/X2, a ~1100 mm): entre ellas y la pared quedan ~140 mm por lado, a tope de velocidad se rozan.
                dd_o = np.hypot(pxy[:, None, 0] - outer[None, :, 0], pxy[:, None, 1] - outer[None, :, 1]).min(axis=1)
                on, osm = float(C.ROUTE_L23_OUTER_NEAR_MM), float(osm)
                v_om = K * (float(C.ROUTE_L23_OUTER_PWM) - pwm0)
                v_o = v_om + (v_base - v_om) * np.clip((dd_o - on) / max(osm - on, 1.0), 0.0, 1.0)
                v_c = np.where(dd_o < osm, np.minimum(v_c, v_o), v_c)
        if getattr(C, "ROUTE_PROFILE_ISLAND_MM", None) is not None:
            # Cerca de las esquinas de la isla: misma rampa que las señales (v_min -> base).
            h = float(INNER_HALF_MM)
            cxy = np.array([[h, h], [h, -h], [-h, h], [-h, -h]])
            pxy = np.stack((r.x[j], r.y[j]), axis=1)                       # (m, 2)
            di = np.hypot(pxy[:, None, 0] - cxy[None, :, 0],
                          pxy[:, None, 1] - cxy[None, :, 1]).min(axis=1)    # (m,)
            isl_mm = float(C.ROUTE_PROFILE_ISLAND_MM)
            isl_near = float(getattr(C, "ROUTE_PROFILE_ISLAND_NEAR_MM", 200.0))
            f = np.clip((di - isl_near) / max(isl_mm - isl_near, 1.0), 0.0, 1.0)
            v_isl = v_min + (v_base - v_min) * f
            v_c = np.where(di < isl_mm, np.minimum(v_c, v_isl), v_c)
        allowed = np.sqrt(v_c ** 2 + 2.0 * a_brk * d)
        inside = np.nonzero(dist <= span)[0]
        k_min = inside[int(np.argmin(allowed[inside]))]
        v_allow = min(v_max, float(allowed[k_min]))
        lo = floor if corner_floor is None else min(floor, corner_floor)
        ff = getattr(C, "ROUTE_PROFILE_BRAKE_FF_S", None)
        if ff is not None and d[k_min] > 0.0 and v_allow < v_max:
            # Frenando hacia una muestra de adelante: se adelanta el retraso del motor.
            v_allow -= float(ff) * a_brk
            lo = min(floor, int(getattr(C, "ROUTE_PROFILE_BRAKE_FLOOR_PWM", 60)))
        if tight_on:
            lo = min(lo, int(getattr(C, "ROUTE_PROFILE_SIGN_TIGHT_PWM", 110)))
        pwm = v_allow / K + pwm0
        return int(round(min(max(pwm, lo), top)))

    def _force_wait(self, pose) -> bool:
        """Cinta ya a la vista pero sin votos (aún más allá de los 600 mm de la ventana): el cajón
        ancho la deja 500+ mm después del final supuesto de la recta y forzar daba la media del sonar
        (gemelo s3/s7: -80 mm, 12 giros no). Espera hasta ROUTE_FIX_FORCE_WAIT_MM de avance."""
        wmm = float(getattr(C, "ROUTE_FIX_FORCE_WAIT_MM", 0.0))
        o = self.orange
        if wmm <= 0.0 or self._line_d or not (o and o.get("seen") and not o.get("dead_reckoned")):
            return False
        w0 = self.__dict__.setdefault("_wait0", (pose[0], pose[1]))
        self._wait_n = getattr(self, "_wait_n", 0) + 1
        nmax = getattr(C, "ROUTE_FIX_WAIT_FRAMES", None)   # parado (rv=0) la distancia no crece: tope por cuadros
        if nmax is not None and self._wait_n > int(nmax):
            return False
        return math.hypot(pose[0] - w0[0], pose[1] - w0[1]) < wmm

    def _past_straight(self, dm, pose) -> bool:
        """El frente del carro ya llegó (en el marco supuesto) al final de la recta del cajón."""
        _h, w = world_to_section(dm.parking, pose[0], pose[1])
        sgn = 1.0 if dm.direction == "CW" else -1.0
        front = w + sgn * RR._FRONT_MM
        return front >= 1900.0 if sgn > 0 else front <= 1100.0

    def _line_median(self, last: int | None = None) -> float:
        d = self._line_d[-last:] if last else self._line_d
        srcs = self._line_vote_src[-len(d):] if last else self._line_vote_src
        if getattr(C, "ROUTE_FIX_LINE_VOTE_REJECT", False):
            pix = [d[i] for i, s in enumerate(srcs) if s == "pix"]
            if pix:
                return float(np.median(pix))
        return float(np.median(d))

    def _recta_vote_span(self) -> float:
        srcs = self._line_vote_src
        rv = [self._line_d[i] for i, s in enumerate(srcs) if s == "recta"]
        return float(max(rv) - min(rv)) if len(rv) >= 2 else 0.0

    def _pix_blind_heading_ok(self, pose, lim=None) -> bool:
        """Mediana pix en arranque ciego solo con el carro ya en la recta del cajón (no en maniobra INICIO)."""
        if self.fixed or not getattr(C, "ROUTE_FIX_PIX_AT_INICIO", False):
            return True
        if lim is None:
            lim = getattr(C, "ROUTE_FIX_PIX_BLIND_MAX_HEADING_DEG", None)
        if lim is None:
            return True
        # Eje del cajón en mundo: rumbo paralelo a axis = atan2(ax, ay).
        ax, ay = self.axis
        h_ax = math.degrees(math.atan2(ax, ay))
        err = abs((pose[2] - h_ax + 180.0) % 360.0 - 180.0)
        err = min(err, abs((pose[2] - h_ax - 180.0 + 180.0) % 360.0 - 180.0))
        return err <= float(lim)

    def _pix_inicio_vote_ok(self, d: float) -> bool:
        """Con ToF ya informativo en el cajón, no añadir pix_line (parkc3 s3/s5); s34 sigue con masa baja."""
        if self.fix is None:
            return True
        ni = int(getattr(C, "ROUTE_FIX_LINE_TOF_MIN_INFORM", 3))
        if getattr(C, "ROUTE_FIX_PIX_INICIO_IF_TOF_WEAK", False) and self.fix.n_inform > 0:
            return False
        if self.fix.n_inform >= ni and not self._line_ok(d):
            return False
        return True

    def _line_vote_append(self, d: float, src: str) -> bool:
        d = float(d)
        if getattr(C, "ROUTE_FIX_LINE_VOTE_REJECT", False):
            pix = [self._line_d[i] for i, s in enumerate(self._line_vote_src) if s == "pix"]
            if src == "recta" and pix:
                med = float(np.median(pix))
                mad = float(np.median(np.abs(np.array(pix) - med))) or 15.0
                k = float(getattr(C, "ROUTE_FIX_LINE_VOTE_K_MAD", 2.5))
                if abs(d - med) > k * max(mad, 12.0):
                    print(f"[FIXVOTE] recta d={d:+.0f} rechazada vs pix med={med:+.0f}", flush=True)
                    return False
            if src == "recta" and self._recta_vote_span() >= float(getattr(C, "ROUTE_FIX_LINE_RECTA_DRIFT_MM", 40.0)):
                return False
        self._line_d.append(d)
        self._line_vote_src.append(src)
        return True

    def _line_vote(self, dm, pose):
        """δ del cajón según la línea naranja. Con `_pix` (pixeles de cinta del frame,
        corner_lines._center_pix): cada pixel llevado al mundo con la pose supuesta da
        un δ, y el voto es la mediana. Si no, el punto de la recta ajustada en la
        columna central del BEV."""
        if getattr(C, "ROUTE_FIX_BLUE", False):
            self._blue_vote(dm, pose)
        o = self.orange
        pix = o.get("_pix") if o else None
        if o and getattr(C, "ROUTE_FIX_PIX_LINE", False):
            # Voto por línea: con todos los pixeles naranjas de la franja, aunque el tracker no
            # haya aceptado la cinta en este frame (sol: core 0; señal roja tomada como cinta).
            pall = o.get("_pix_all")
            if pall is not None and len(pall) >= int(getattr(C, "ROUTE_FIX_LINE_MIN_PIX", 25)):
                self._pix_line_vote(dm, pose, np.asarray(pall, float))
            return
        # Con pixeles crudos del frame no hace falta que el tracker ya haya persistido la línea
        if not o or o.get("dead_reckoned") or not (o.get("seen") or (
                pix is not None and getattr(C, "ROUTE_FIX_PIX_RAW", False))):
            return
        if pix is not None and len(pix) >= int(getattr(C, "ROUTE_FIX_LINE_MIN_PIX", 25)):
            if self._pix_vote(dm, pose, np.asarray(pix, float)):
                return
        elif getattr(C, "ROUTE_FIX_LINE_PIX", False) and getattr(self, "_pix_ok", None) is not False:
            return
        if not o.get("line") or (pix is None and getattr(C, "ROUTE_FIX_LINE_FRESH", None)):
            # Sin _pix la cinta no se vio en ESTE frame y la recta es la sostenida: con la pose
            # avanzando el voto se arrastra ~1 frame de avance (gemelo N30 s34: -436 -> -81, verdad -680).
            # Con pixeles, sin _pix este frame no se vota: la recta que queda es la
            # SOSTENIDA de frames anteriores y con la pose avanzando arrastraba el
            # voto ~10 mm/frame (gemelo s19: de +327 a +74).
            return
        if (getattr(C, "ROUTE_FIX_PIX_AT_INICIO", False) and not self.fixed
                and o.get("src") == "col"):
            ny = o.get("near_y")
            band = int(o.get("band") or 0)
            max_band = int(getattr(C, "ROUTE_FIX_PIX_INICIO_MAX_BAND", 750))
            if (ny is not None and float(ny) >= float(getattr(C, "ROUTE_FIX_PIX_INICIO_MIN_Y", 150.0))
                    and band <= max_band):
                pall = o.get("_pix_all")
                if pall is None or len(np.asarray(pall)) == 0:
                    pall = o.get("_pix")
                if pall is not None and len(pall) >= int(getattr(C, "ROUTE_FIX_LINE_MIN_PIX", 25)):
                    res = pix_line_delta(np.asarray(pall, float), pose, np.array(self.axis), dm.direction,
                                         self._fwd_add(dm))
                    if res is not None and self._pix_inicio_vote_ok(res[0]):
                        self._pix_line_vote(dm, pose, np.asarray(pall, float))
                        return
                    weak = (getattr(C, "ROUTE_FIX_PIX_INICIO_IF_TOF_WEAK", False)
                            and self.fix is not None and self.fix.n_inform > 0)
                    if not weak:
                        return
        vx, vy, x0, y0 = o["line"]
        if abs(vx) < 1e-6:
            return
        by = y0 + (float(C.ROBOT_BEV_X) - x0) * vy / vx
        fwd = (C.ROBOT_BEV_Y - by) * C.MM_PER_PX + float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0)) + self._fwd_add(dm)
        if not 100.0 <= fwd <= 600.0:
            return
        k = int(getattr(C, "ROUTE_FIX_LINE_LAG_FRAMES", 0))
        if len(self._pre_poses) > k:
            pose = self._pre_poses[-1 - k]   # pose del momento de la imagen (latencia cámara->ACK)
        h = math.radians(pose[2])
        p = np.array([pose[0] + fwd * math.sin(h), pose[1] + fwd * math.cos(h)])
        a = np.array(self.axis)
        span = PARK_W_MAX - PARK_W_MIN
        # Holgura más allá del rango del reglamento (el δ final se recorta en _apply_fix). Con 150 la
        # cinta buena quedaba fuera y votaba la de otra esquina (corridas 100-102: SW a 9 cm daba
        # +166..+192 y la NE a 1.4 m -565 -> isla).
        mg = float(getattr(C, "ROUTE_FIX_LINE_RANGE_MARGIN_MM", 150.0))
        lo, hi = (-mg, span + mg) if dm.direction == "CW" else (-span - mg, mg)
        best = None
        for (sx, sy), (ex, ey) in orange_segments():
            u = np.array([ex - sx, ey - sy], dtype=float); u /= np.linalg.norm(u)
            n = np.array([-u[1], u[0]])
            if abs(n @ a) < 0.3:
                continue
            d = (n @ np.array([sx, sy]) - n @ p) / (n @ a)
            dist = math.hypot(sx - p[0], sy - p[1])
            if lo <= d <= hi and (best is None or dist < best[1]):
                best = (float(d), dist)
        if best is not None:
            if self._line_vote_append(best[0], "recta"):
                print(f"[FIXVOTE] recta d={best[0]:+.0f} fwd={fwd:.0f} by={by:.1f} pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.1f})", flush=True)

    def _blue_vote(self, dm, pose) -> None:
        """δ del cajón con la cinta AZUL (la que cruza el carril en CCW, |n·a| 0.87; la naranja ahí va a
        30° y en salidas rectas nunca pasa enfrente: corridas 125/128 decididas a la fuerza con 0 votos).
        Máscara HSV del BEV (cinta H 99-107 S 98-195 V 110-146; paredes V~40), componente más grande
        (los falsos son manchas chicas), pixeles a fwd <= ROUTE_FIX_BLUE_FAR_MM llevados al tapete con la
        pose de hace ROUTE_FIX_LINE_LAG_FRAMES; voto = mediana del δ por pixel contra el segmento azul
        más cercano. Offline 2026-11-21 (lag 14): 126 −82 vs −83 medido, 128 −118 vs −88."""
        res = self._blue_delta(dm, pose)
        if res is None:
            return
        d, n, c, pose = res
        self._line_vote_append(d, "azul")
        print(f"[FIXVOTE] azul d={d:+.0f} n={n} c=({c[0]:.0f},{c[1]:.0f}) "
              f"pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.1f})", flush=True)

    def _post_blue(self, dm, ack, pose) -> None:
        """Cinta azul vista DESPUÉS de fijar el arranque, aún en la recta 1: corrige solo la pose.

        La cinta solo se ve los últimos ~15 frames de la recta y el arranque forzado se decide en ese mismo
        momento (2026-11-23: run 184 votó en el primer frame útil; la 185 decidió 14 frames antes con los ToF,
        δ −46 al revés, y la isla corrigió +203 mm en la esquina 1). La cinta es un punto fijo del tapete:
        con la pose ya corrida, la mediana de sus votos es el error que queda. El cajón no se toca."""
        n_min = int(getattr(C, "ROUTE_FIX_POST_BLUE_N", 0) or 0)
        if n_min <= 0 or self._post_blue_done:
            return
        res = self._blue_delta(dm, pose, rng=float(getattr(C, "ROUTE_FIX_POST_BLUE_MAX_MM", 250.0)) + 100.0)
        if res is None:
            return
        self._post_blue_d.append(res[0])
        if len(self._post_blue_d) < n_min:
            return
        last = self._post_blue_d[-n_min:]
        if max(last) - min(last) > float(getattr(C, "ROUTE_FIX_POST_BLUE_SPAN_MM", 60.0)):
            return
        r = float(np.median(last))
        cap = float(getattr(C, "ROUTE_FIX_POST_BLUE_MAX_MM", 250.0))
        self._post_blue_done = True
        if abs(r) > cap or abs(r) < float(getattr(C, "ROUTE_FIX_POST_BLUE_MIN_MM", 15.0)):
            print(f"[ROUTE] cinta azul tras el arranque: resto {r:+.0f} mm, no se aplica "
                  f"(votos {', '.join(f'{d:+.0f}' for d in self._post_blue_d)})", flush=True)
            return
        ax, ay = self.axis
        self.loc.shift(r * ax, r * ay)
        self.shift = float(self.shift) + r
        self._sync_map(dm, ack, self.loc.pose())
        print(f"[ROUTE] cinta azul tras el arranque: pose {r:+.0f} mm (δ pose total {self.shift:+.0f}; "
              f"votos {', '.join(f'{d:+.0f}' for d in self._post_blue_d)})", flush=True)

    def _blue_delta(self, dm, pose, rng=None):
        """(δ, n pixeles, centroide, pose usada) de la cinta azul en este frame, o None."""
        hsv = getattr(self, "bev_hsv", None)
        if hsv is None:
            return None
        lo_hsv, hi_hsv = getattr(C, "ROUTE_FIX_BLUE_HSV", ((95, 70, 60), (115, 255, 200)))
        mk = cv2.inRange(hsv, np.array(lo_hsv, np.uint8), np.array(hi_hsv, np.uint8))
        mk = cv2.morphologyEx(mk, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        nc, lab, st, _ = cv2.connectedComponentsWithStats(mk)
        if nc < 2:
            return None
        k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        if st[k, cv2.CC_STAT_AREA] < int(getattr(C, "ROUTE_FIX_BLUE_MIN_AREA", 150)):
            return None
        ys, xs = np.nonzero(lab == k)
        fwd = (float(C.ROBOT_BEV_Y) - ys) * C.MM_PER_PX + float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0)) + self._fwd_add(dm)
        lat = (xs - float(C.ROBOT_BEV_X)) * C.MM_PER_PX
        sel = fwd <= float(getattr(C, "ROUTE_FIX_BLUE_FAR_MM", 700.0))
        if int(sel.sum()) < int(getattr(C, "ROUTE_FIX_BLUE_MIN_PIX", 80)):
            return None
        fwd, lat = fwd[sel], lat[sel]
        lag = int(getattr(C, "ROUTE_FIX_LINE_LAG_FRAMES", 0))
        if len(self._pre_poses) > lag:
            pose = self._pre_poses[-1 - lag]
        h = math.radians(pose[2])
        P = np.column_stack([pose[0] + fwd * math.sin(h) + lat * math.cos(h),
                             pose[1] + fwd * math.cos(h) - lat * math.sin(h)])
        c = P.mean(axis=0)
        a = np.array(self.axis)
        span = PARK_W_MAX - PARK_W_MIN
        mg = float(getattr(C, "ROUTE_FIX_LINE_RANGE_MARGIN_MM", 150.0))
        lo, hi = (-mg, span + mg) if dm.direction == "CW" else (-span - mg, mg)
        if rng is not None:
            lo, hi = -float(rng), float(rng)   # tras el arranque: el resto es chico, alrededor de 0
        best = None
        for (sx, sy), (ex, ey) in blue_segments():
            u = np.array([ex - sx, ey - sy], dtype=float); u /= np.linalg.norm(u)
            n = np.array([-u[1], u[0]])
            if abs(float(n @ a)) < float(getattr(C, "ROUTE_FIX_PIX_MIN_NA", 0.7)):
                continue
            ns = float(n @ np.array([sx, sy]))
            d = float(np.median((ns - P @ n) / (n @ a)))
            dist = math.hypot(sx - c[0], sy - c[1])
            if lo <= d <= hi and (best is None or dist < best[1]):
                best = (d, dist)
        if best is None:
            return None
        return best[0], len(fwd), c, pose

    def _pix_line_vote(self, dm, pose, pix) -> None:
        """Voto del cajón buscando UNA LÍNEA con la dirección de la cinta, no la mediana de todo lo naranja.

        Corridas 110/113-117 (2026-11-20, de noche): la orilla de la base de una señal roja (H 6-8, fuera
        de LINE_CONE_HSV) entraba como cinta en la franja del voto y el tracker hasta la aceptaba como la
        línea; al estar más cerca que la cinta, el carro se creía más adelante -> cajón +130..+245 mm.
        La cinta es una línea recta de 20 mm con dirección conocida (el segmento del modelo) y el rumbo
        del BNO es bueno (salida 2026-11-21: 2 cm tras la S). Por cada rumbo de prueba (BNO ±N°) se lleva
        cada pixel al tapete, se mide su distancia d a lo largo del eje de la recta a la que esa cinta
        cruzaría, y se busca la ventana de ROUTE_FIX_PIX_LINE_W_MM con más pixeles. Gana el rumbo con la
        ventana más llena (la línea más nítida). Vale solo si esos pixeles cubren al menos
        ROUTE_FIX_PIX_LINE_SPAN_MM a lo largo de la cinta: el contorno de una señal (50 mm) no llega."""
        k = int(getattr(C, "ROUTE_FIX_LINE_LAG_FRAMES", 0))
        cur_h = pose[2]
        if len(self._pre_poses) > k:
            pose = self._pre_poses[-1 - k]
        turn = abs((cur_h - pose[2] + 180.0) % 360.0 - 180.0)
        if turn > float(getattr(C, "ROUTE_FIX_PIX_MAX_TURN_DEG", 3.0)):
            return
        a = np.array(self.axis)
        h0 = math.radians(pose[2])
        if abs(math.sin(h0) * a[1] - math.cos(h0) * a[0]) > math.sin(math.radians(float(getattr(C, "ROUTE_FIX_PIX_MAX_ANG_DEG", 15.0)))):
            return
        res = pix_line_delta(pix, pose, a, dm.direction, self._fwd_add(dm))
        if res is None:
            return
        d, info = res
        if getattr(C, "ROUTE_FIX_PIX_AT_INICIO", False) and not self.fixed and not self._pix_inicio_vote_ok(d):
            return
        self._line_vote_append(d, "linea")
        print(f"[FIXVOTE] linea d={d:+.0f} {info} pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.1f})", flush=True)

    def _line_ok(self, d: float) -> bool:
        """La cinta no contradice a los ToF: en ±ROUTE_FIX_WIN_MM de su δ el histograma de los
        ToF no está más de ROUTE_FIX_LINE_TOF_LR (log) bajo su máximo. Corridas 100/101
        (2026-11-20, seed 23, mediodía): la cinta votó -565 cinco veces seguidas y decidió;
        los ToF daban ~-150 y ese δ quedaba ~e^-11..e^-25 abajo -> ruta a la isla."""
        lr = getattr(C, "ROUTE_FIX_LINE_TOF_LR", None)
        if lr is None or self.fix.n_inform < int(getattr(C, "ROUTE_FIX_LINE_TOF_MIN_INFORM", 3)):
            return True
        lp = self.fix.logp - self.fix.logp.max()
        sel = np.abs(self.fix.deltas - d) <= float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0))
        v = float(lp[sel].max()) if sel.any() else float(lp.min())
        ok = v >= -float(lr)
        if v < -3.0 and getattr(self, "_line_rej_d", None) != round(d):
            self._line_rej_d = round(d)
            pk, pm = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))
            print(f"[ROUTE] cinta δ={d:+.0f} {'ok' if ok else 'rechazada'}: ToF logp {v:.1f} (pico {pk:+.0f} masa {pm:.2f}, "
                  f"{self.fix.n_inform} informativas)", flush=True)
        return ok

    @staticmethod
    def _fwd_add(dm) -> float:
        v = getattr(C, "ROUTE_FIX_FWD_ADD_MM", 0.0)
        return float(v.get(dm.direction, 0.0)) if isinstance(v, dict) else float(v)

    def _pix_vote(self, dm, pose, pix):
        k = int(getattr(C, "ROUTE_FIX_LINE_LAG_FRAMES", 0))
        cur_h = pose[2]
        if len(self._pre_poses) > k:
            pose = self._pre_poses[-1 - k]   # pose del momento de la imagen (latencia cámara->ACK)
        fwd = (float(C.ROBOT_BEV_Y) - pix[:, 1]) * C.MM_PER_PX + float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0)) + self._fwd_add(dm)
        lat = (pix[:, 0] - float(C.ROBOT_BEV_X)) * C.MM_PER_PX
        h = math.radians(pose[2])
        P = np.column_stack([pose[0] + fwd * math.sin(h) + lat * math.cos(h),
                             pose[1] + fwd * math.cos(h) - lat * math.sin(h)])
        c = P.mean(axis=0)
        a = np.array(self.axis)
        span = PARK_W_MAX - PARK_W_MIN
        # Holgura más allá del rango del reglamento (el δ final se recorta en _apply_fix). Con 150 la
        # cinta buena quedaba fuera y votaba la de otra esquina (corridas 100-102: SW a 9 cm daba
        # +166..+192 y la NE a 1.4 m -565 -> isla).
        mg = float(getattr(C, "ROUTE_FIX_LINE_RANGE_MARGIN_MM", 150.0))
        lo, hi = (-mg, span + mg) if dm.direction == "CW" else (-span - mg, mg)
        best = None
        for (sx, sy), (ex, ey) in orange_segments():
            u = np.array([ex - sx, ey - sy], dtype=float); u /= np.linalg.norm(u)
            n = np.array([-u[1], u[0]])
            if abs(n @ a) < 0.3:
                continue
            d = float((n @ np.array([sx, sy]) - n @ c) / (n @ a))
            dist = math.hypot(sx - c[0], sy - c[1])
            if lo <= d <= hi and (best is None or dist < best[2]):
                best = (n, float(n @ np.array([sx, sy])), dist, d)
        if best is None:
            return True
        n, ns, _, _ = best
        # Cinta casi paralela a la recta (|n·a| chico, CCW con la de 30°): un error
        # lateral de la pose se multiplica por |n_perp|/|n·a| (1.7 a 30°; gemelo s23
        # −190 mm). Ahí se queda el voto de la recta ajustada de siempre.
        self._pix_ok = abs(float(n @ a)) >= float(getattr(C, "ROUTE_FIX_PIX_MIN_NA", 0.7))
        if not self._pix_ok:
            return False
        # Solo con el carro casi alineado a la recta y sin girar (cámara y pose del mismo instante).
        turn = abs((cur_h - pose[2] + 180.0) % 360.0 - 180.0)
        if turn > float(getattr(C, "ROUTE_FIX_PIX_MAX_TURN_DEG", 3.0)):
            return True
        if abs(math.sin(h) * a[1] - math.cos(h) * a[0]) > math.sin(math.radians(float(getattr(C, "ROUTE_FIX_PIX_MAX_ANG_DEG", 15.0)))):
            return True
        na = abs(float(n @ a))
        d = (ns - P @ n) / (n @ a)
        d = d[(d >= lo) & (d <= hi)]
        if len(d) < int(getattr(C, "ROUTE_FIX_LINE_MIN_PIX", 25)):
            return True
        d_pix = float(np.median(d))
        ok_h = self._pix_blind_heading_ok(pose)
        if not ok_h:
            lim_al = getattr(C, "ROUTE_FIX_PIX_BLIND_MAX_HEADING_ALIGNED_DEG", None)
            cap_d = float(getattr(C, "ROUTE_FIX_PIX_BLIND_ALIGNED_MAX_D_MM", 150.0))
            if (lim_al is not None and na >= float(getattr(C, "ROUTE_FIX_PIX_MIN_NA", 0.7))
                    and abs(d_pix) <= cap_d):
                ok_h = self._pix_blind_heading_ok(pose, lim=lim_al)
        if not ok_h:
            return True
        self._line_vote_append(d_pix, "pix")
        print(f"[FIXVOTE] pix d={self._line_d[-1]:+.0f} n={len(d)} iqr={np.percentile(d, 75) - np.percentile(d, 25):.0f} "
              f"na={float(n @ a):.2f} c=({c[0]:.0f},{c[1]:.0f}) pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.1f})", flush=True)
        return True

    def _unique_prefix_seat_delta(self, dm, pose) -> float | None:
        """Ancla el avance con una señal seguida desde dos posiciones distintas."""
        frames = dm._deferred or ()
        # La fila interior es la única legal en la recta del cajón.
        row = [w for seat, (_, w) in SEATS.items() if seat not in PARKING_FORBIDDEN_SEATS]
        _, car_w = world_to_section(dm.parking, *pose[:2])
        direction = 1 if dm.direction == "CW" else -1
        observations = []
        for feet, camera_pose, heading, *_ in frames[-16:]:
            candidates = []
            for x, y, color, fwd, _ in feet_world(feet, camera_pose, heading):
                h, w = world_to_section(dm.parking, x, y)
                if abs(h - _ROW_H) <= SIGN_MM and 0 < direction * (w - car_w) and fwd <= 850.0:
                    candidates.append((w, color))
            if len(candidates) == 1:
                _, view_w = world_to_section(dm.parking, *camera_pose)
                observations.append((*candidates[0], view_w))
        if len(observations) < 5:
            return None
        ws, colors, views = zip(*observations)
        if len(set(colors)) != 1 or max(ws) - min(ws) > 45.0 or max(views) - min(views) < 45.0:
            return None
        w = float(np.median(ws))
        # Mantén las hipótesis próximas al límite como alternativas hasta
        # distinguirlas. Una diferencia de asiento desplaza el mapa 500 mm.
        lo = PARK_W_MIN - self.fix.w0_guess
        hi = PARK_W_MAX - self.fix.w0_guess
        shifts = [seat_w - w for seat_w in row if lo - SIGN_MM <= seat_w - w <= hi + SIGN_MM]
        if len(shifts) != 1 or not lo <= shifts[0] <= hi:
            return None
        delta = shifts[0]
        if self._line_d and abs(delta - self._line_median()) > SIGN_MM:
            return None
        if self.fix.n_inform >= 3:
            near = np.abs(self.fix.deltas - delta) <= SIGN_MM
            if np.max(self.fix.logp[near]) < self.fix.logp.max() - 3.0:
                return None
        print(f"[ROUTE] avance por asiento único: {delta:+.0f} mm", flush=True)
        return delta

    def _seat_rows_delta(self, dm, pose) -> float | None:
        """Ancla el avance cruzando dos rejillas de asientos (ROUTE_FIX_SEAT_ROWS).

        Una señal de la recta del cajón (fila interior) fija el avance módulo 500 mm (columnas
        T2/X1/T1); una de la recta SIGUIENTE, vista desde el cajón, cae en una columna conocida
        (su w no depende del avance) y su fila (400/600 mm) lo fija módulo 200 mm, porque el eje
        del cajón es el eje h de esa recta. Cada señal seguida en >= 5 cuadros desde posiciones
        distintas da un juego de hipótesis; vale el avance que explica TODAS las señales con un
        asiento legal y queda solo en el rango del cajón. Si quedan dos, decide el histograma de
        los ToF solo si una está >= 3 nats por encima. (n1/s36: la verde S/T1 tras la roja W/T2
        se veía a 1.1 m desde la salida, pero sin el avance el plan no la tenía hasta la esquina.)
        Solo CCW: en CW (probado en el gemelo) cambia la fijación por cinta, que ya llega temprano, y
        sumó 2 choques en 96."""
        frames = dm._deferred or ()
        if not frames:
            return None
        ax, ay = self.axis
        step = 1 if dm.direction == "CW" else -1
        nxt = SECTIONS_CW[(SECTIONS_CW.index(dm.parking) + step) % len(SECTIONS_CW)]
        lo = PARK_W_MIN - self.fix.w0_guess
        hi = PARK_W_MAX - self.fix.w0_guess
        # Cuánto se mueve cada coordenada por mm de avance (±1 o 0).
        x0, y0 = section_to_world(dm.parking, _ROW_H, 1500.0)
        dw = world_to_section(dm.parking, x0 + ax, y0 + ay)[1] - world_to_section(dm.parking, x0, y0)[1]
        dh = world_to_section(nxt, x0 + ax, y0 + ay)[0] - world_to_section(nxt, x0, y0)[0]
        if abs(abs(dw) - 1.0) > 1e-6 or abs(abs(dh) - 1.0) > 1e-6:
            return None
        row_w = [w for seat, (_, w) in SEATS.items() if seat not in PARKING_FORBIDDEN_SEATS]
        cols: dict[float, list[float]] = {}
        for _seat, (h, w) in SEATS.items():
            cols.setdefault(w, []).append(h)
        max_fwd = float(getattr(C, "ROUTE_FIX_SEAT_ROWS_MAX_FWD_MM", 1300.0))
        col_tol = float(getattr(C, "ROUTE_FIX_SEAT_ROWS_COL_TOL_MM", 80.0))
        tracks: dict[tuple, list[tuple[float, float]]] = {}
        for feet, camera_pose, heading, *_ in frames:
            _, view_w = world_to_section(dm.parking, *camera_pose)
            per: dict[tuple, list[float]] = {}
            for x, y, color, fwd, _lat in feet_world(feet, camera_pose, heading):
                if fwd > max_fwd:
                    continue
                h, w = world_to_section(dm.parking, x, y)
                if abs(h - _ROW_H) <= SIGN_MM and 0.0 < w < 3000.0:
                    per.setdefault(("P", color), []).append(w)
                hn, wn = world_to_section(nxt, x, y)
                for cw in cols:
                    if abs(wn - cw) <= col_tol and 200.0 <= hn <= 1500.0:
                        per.setdefault(("N", cw, color), []).append(hn)
            for key, vals in per.items():
                if len(vals) == 1:   # dos pies en competencia no informan
                    tracks.setdefault(key, []).append((vals[0], view_w))
        tol = {"P": 45.0, "N": float(getattr(C, "ROUTE_FIX_SEAT_ROWS_SPAN_MM", 60.0))}
        hyps = []   # por señal: hipótesis de avance
        for key, obs in tracks.items():
            if len(obs) < 5:
                continue
            vals = [o[0] for o in obs][-12:]
            views = [o[1] for o in obs]
            if max(vals) - min(vals) > tol[key[0]] or max(views) - min(views) < 45.0:
                continue
            v = float(np.median(vals))
            if key[0] == "P":
                cand = [(seat_w - v) / dw for seat_w in row_w]
            else:
                cand = [(seat_h - v) / dh for seat_h in cols[key[1]]]
            cand = [d for d in cand if lo - SIGN_MM <= d <= hi + SIGN_MM]
            if not cand:
                return None   # una señal sin asiento legal: algo está mal, no anclar
            hyps.append((key, cand))
        if not hyps:
            return None
        # Hipótesis comunes a todas las señales (dentro de SIGN_MM).
        common = []
        for d0 in hyps[0][1]:
            matched = [d0]
            for _key, cand in hyps[1:]:
                near = [d for d in cand if abs(d - d0) <= SIGN_MM]
                if not near:
                    break
                matched.append(min(near, key=lambda d: abs(d - d0)))
            else:
                common.append(float(np.mean(matched)))
        if not common:
            return None
        if len(common) > 1:
            if self.fix.n_inform < 3:
                return None
            score = []
            for d in common:
                near = np.abs(self.fix.deltas - d) <= SIGN_MM
                score.append(float(np.max(self.fix.logp[near])) if np.any(near) else -1e9)
            order = np.argsort(score)[::-1]
            if score[order[0]] - score[order[1]] < 3.0:
                return None
            common = [common[order[0]]]
        delta = common[0]
        if not lo <= delta <= hi:
            return None
        if self._line_d and abs(delta - self._line_median()) > SIGN_MM:
            return None
        if self.fix.n_inform >= 3:
            near = np.abs(self.fix.deltas - delta) <= SIGN_MM
            if not np.any(near) or np.max(self.fix.logp[near]) < self.fix.logp.max() - 3.0:
                return None
        kinds = "+".join(sorted({k[0] for k, _ in hyps}))
        print(f"[ROUTE] avance por filas de asientos ({kinds}, {len(hyps)} señales): {delta:+.0f} mm", flush=True)
        return delta

    def _pillar_pair_delta(self, dm, delta: float, min_shift: float | None = None) -> float:
        """Contrasta el ajuste de la cinta con un par inequívoco de asientos."""
        ax, ay = self.axis
        step = 1 if dm.direction == "CW" else -1
        next_sec = SECTIONS_CW[(SECTIONS_CW.index(dm.parking) + step) % len(SECTIONS_CW)]
        seats = {
            (sec, seat): section_to_world(sec, h, w)
            for sec in (dm.parking, next_sec)
            for seat, (h, w) in SEATS.items()
            if sec != dm.parking or seat not in PARKING_FORBIDDEN_SEATS
        }
        votes: dict[tuple[str, str], list[float]] = {}
        for feet, pose, heading, *_ in dm._deferred or ():
            frame_votes: dict[tuple[str, str], list[float]] = {}
            for x, y, _, _, _ in feet_world(feet, pose, heading):
                for key, (sx, sy) in seats.items():
                    dx, dy = sx - x, sy - y
                    shift = dx * ax + dy * ay
                    if (abs(dx * ay - dy * ax) <= SIGN_MM
                            and PARK_W_MIN <= self.fix.w0_guess + shift <= PARK_W_MAX):
                        frame_votes.setdefault(key, []).append(shift)
            for key, values in frame_votes.items():
                # Dos pies en competencia no informan de este asiento.
                if len(values) == 1:
                    votes.setdefault(key, []).extend(values)

        shifts = {sec: [] for sec in (dm.parking, next_sec)}
        for (sec, _), values in votes.items():
            if len(values) >= 5:
                shifts[sec].append(float(np.median(values)))
        pairs = [(a + b) / 2 for a in shifts[dm.parking] for b in shifts[next_sec]
                 if abs(a - b) <= SIGN_MM]
        # La cinta conserva la prioridad para discrepancias pequeñas o pares ambiguos.
        if min_shift is None:
            min_shift = 3 * SIGN_MM
        if len(pairs) == 1 and abs(pairs[0] - delta) >= min_shift:
            print(f"[ROUTE] avance por par de pilares: {delta:+.0f} -> {pairs[0]:+.0f} mm", flush=True)
            return pairs[0]
        return delta

    def _apply_fix(self, dm, ack, delta: float, forced: bool, park_bias: float = 0.0):
        """Avance resuelto: corre la pose, vota lo guardado y repasa las vistas."""
        # w0 fuera de [PARK_W_MIN, PARK_W_MAX] no existe en el reglamento (corrida 99: 1820 con máx 1696).
        lo, hi = PARK_W_MIN - self.fix.w0_guess, PARK_W_MAX - self.fix.w0_guess
        delta_raw = float(delta)
        delta = float(min(max(delta_raw, lo), hi))   # δ del cajón (dónde está la madera)
        # δ de la pose: lo que midió la cinta es dónde está el CARRO, no el cajón. Recortarlo al rango del cajón
        # tiraba el error de la odometría de la salida (CCW con el cajón supuesto en el máximo: δ=+0 siempre con la
        # cinta votando +38..+102; la isla corregía 137-175 mm en la esquina 1 en las 6 runs del 2026-11-23).
        pm = getattr(C, "ROUTE_FIX_POSE_MARGIN_MM", None)
        delta_pose = delta if pm is None else float(min(max(delta_raw, lo - float(pm)), hi + float(pm)))
        ax, ay = self.axis
        dx, dy = delta_pose * ax, delta_pose * ay
        mass = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))[1]
        self.shift = float(delta_pose)
        self.fixed = True
        self.fix.fixed = float(delta_pose)
        self.loc.shift(dx, dy)
        # El δ corre la POSE (lo que ve la cinta = dónde está el carro), pero la odometría de la salida
        # del cajón se pasa siempre lo mismo: en CCW ~+86 mm (131/126/128 medidos 86/83/88). Ese error
        # no es del cajón: sin esto el cajón quedaba ~9 cm corrido y el carro chocaba al estacionar (133).
        adj = getattr(C, "ROUTE_FIX_PARK_ADJ_MM", 0.0)
        adj = float(adj.get(dm.direction, 0.0)) if isinstance(adj, dict) else float(adj)
        adj += park_bias
        self.park_w = float(min(max(self.fix.w0_guess + delta + adj, PARK_W_MIN), PARK_W_MAX))
        dm.park_w = self.park_w
        n = dm.replay_deferred(dx, dy)
        pose = self.loc.pose()
        self._sync_map(dm, ack, pose)
        if bool(getattr(C, "ROUTE_LAP1", True)):
            for px, py, ph in self._pre_poses:
                conf, empty = self._resolved(dm)
                self._update_views(conf, empty, (px + dx, py + dy, ph))
        self._pre_poses = []
        how = f"FORZADO masa {mass:.2f}" if forced else f"masa {mass:.2f}"
        if getattr(C, "ROUTE_FIX_LINE", None):
            how += f" línea n={len(self._line_d)}"
        print(f"[ROUTE] arranque: cajón en w0={self.park_w:.0f} (δ={delta:+.0f} mm, crudo {delta_raw:+.0f}, pose {delta_pose:+.0f}, ajuste {adj:+.0f}, {how}) tras "
              f"{self.fix.n_frames} lecturas ({self.fix.n_inform} informativas); "
              f"{n} frames de cámara votados", flush=True)
        if self._prueba_cajon:
            ld = ", ".join(f"{d:+.0f}" for d in self._line_d)
            est_d, est_m = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))
            print(f"[PRUEBA] cajón: supuesto w0={self.fix.w0_guess:.0f}  δ={delta:+.0f}  "
                  f"forzado={forced}  sonar pico={est_d} masa={est_m:.2f} media={self.fix.mean():+.0f}  "
                  f"votos línea=[{ld}]  pose x={pose[0]:.0f} y={pose[1]:.0f} rumbo={pose[2]:.1f}", flush=True)
        return pose

    def _prefix_extras(self, dm, pose):
        """Recta del cajón sin el avance: en su fila interior puede quedar UNA
        señal adelante del carro (X1 o la T del final; la fila exterior está
        prohibida). Se planea con lo que la cámara ve relativo al carro: la
        señal con color (>=2 votos) donde se vio; si no hay, la fila entera
        adelante como cerca sin lado (la ruta no la cruza hasta saber)."""
        buf = dm._deferred or []
        cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
        for feet, p, vh, _oy, _sec, _tc, _st in buf[self._def_seen:]:
            for wx, wy, color, fwd, lat in feet_world(feet, p, vh):
                h, w = world_to_section(dm.parking, wx, wy)
                if abs(h - _ROW_H) <= _ROW_BAND_MM:
                    self._pre_dets.append((w, color, math.hypot(fwd - cam, lat)))
        self._def_seen = len(buf)
        sgn = 1.0 if dm.direction == "CW" else -1.0
        _h, w_car = world_to_section(dm.parking, pose[0], pose[1])
        ahead = sorted(d for d in self._pre_dets if (d[0] - w_car) * sgn > 0.0)
        clusters: list[list] = []
        for d in ahead:
            if clusters and d[0] - clusters[-1][-1][0] <= _CLUSTER_GAP_MM:
                clusters[-1].append(d)
            else:
                clusters.append([d])
        nmin = int(getattr(C, "ROUTE_PROVISIONAL_VOTES", 2))
        if clusters:
            best = max(clusters, key=len)
            r = sum(1 for d in best if d[1] == "Red")
            g = len(best) - r
            col = ("Red" if r >= nmin and r >= g + 1 else
                   "Green" if g >= nmin and g >= r + 1 else None)
            if col is not None:
                wts = [1.0 / (100.0 + d[2]) for d in best]
                wm = sum(d[0] * wt for d, wt in zip(best, wts)) / sum(wts)
                x, y = section_to_world(dm.parking, _ROW_H, wm)
                if self.fix is not None:
                    self.fix.set_car_signs([(x, y)])     # vista: se mueve con el carro
                return [(x, y, col)], ("senal", col, int(round(wm / 50.0)))
        if self._fence is None:
            front = w_car + sgn * RR._FRONT_MM
            end = 2025.0 if sgn > 0 else 975.0
            w = front + sgn * 250.0
            pts = []
            while (end - w) * sgn >= 0.0:
                pts.append((*section_to_world(dm.parking, _ROW_H, w), None))
                w += sgn * _FENCE_STEP_MM
            self._fence = pts
        return list(self._fence), ("cerca", len(self._fence))

    @staticmethod
    def _section_at(direction, x, y, hd):
        """Recta donde está el carro según la pose (en la esquina, la de su rumbo)."""
        from .wro_field import _heading
        best = None
        for sec in SECTIONS_CW:
            s = RR._sec(sec, direction)
            u, d = s.frame(x, y)
            if not (-100.0 <= u <= 3100.0 and -100.0 <= d <= 1100.0):
                continue
            err = abs((hd - _heading(sec, direction) + 180.0) % 360.0 - 180.0)
            if best is None or err < best[0]:
                best = (err, sec)
        return None if best is None else best[1]

    def _sync_map(self, dm, ack, pose) -> None:
        """El mapa digital proyecta lo que ve con SU pose (odometría sin corregir
        en modo ruta) y con la recta del conteo de giros del ESP: con la deriva
        movía latas confirmadas de fila (semilla 46: W/T2 -> W/T4, choque en la
        vuelta 3). Se le pone la pose del localizador y la recta según la pose."""
        from .localizer import _ack_num
        from .wro_field import _heading
        yaw = _ack_num(ack, "yaw") if ack else None
        if yaw is None or dm._odom_last is None:
            return
        x, y, hd = pose
        dm._pose_fix = (x - dm._dr_xy[0], y - dm._dr_xy[1])
        dm.pose_xy = (x, y)
        h0 = _heading(dm.parking, dm.direction)
        dm._yaw_fix = ((hd - h0 + yaw + 180.0) % 360.0) - 180.0
        dm._view_heading = hd
        dm.heading = hd
        if getattr(C, "ROUTE_MAP_SECTION_FROM_POSE", False):
            sec = self._section_at(dm.direction, x, y, hd)
            if sec is not None and not dm.in_stall:
                dm.section = sec

    @staticmethod
    def _init_pose(dm, ack):
        """Pose del carro para el ACK actual con la transformación odometría -> tapete
        del mapa digital. La pose del mapa puede ser de un ACK de 1-2 frames antes:
        girando (~50 °/s al salir del cajón) eso eran ~3.6° de rumbo que el EKF no
        recuperaba (semilla 3, isla)."""
        from .localizer import _ack_num
        from .wro_field import _heading
        if not ack or dm._odom_last is None:
            return None
        px, py, yaw = _ack_num(ack, "px"), _ack_num(ack, "py"), _ack_num(ack, "yaw")
        if px is None or py is None or yaw is None:
            return None
        th = _heading(dm.parking, dm.direction) + float(dm._yaw_fix)
        r = math.radians(th)
        fx, fy, lx, ly = math.sin(r), math.cos(r), -math.cos(r), math.sin(r)
        p0x, p0y = dm._odom_last
        tx = dm.pose_xy[0] - (p0x * fx + p0y * lx)
        ty = dm.pose_xy[1] - (p0x * fy + p0y * ly)
        return tx + px * fx + py * lx, ty + px * fy + py * ly, (th - yaw) % 360.0

    def _run_plan(self, *args, **kwargs) -> None:
        self._job_out, self._job_conf, self._job_unknown = self._solve_plan(*args, **kwargs)

    @staticmethod
    def _solve_plan(direction, conf, parking, unknown, anchor, lock, park_w, extra, hug, *, fb, tc):
        for seats, unk in ((conf, unknown), fb) if fb is not None and getattr(C, "ROUTE_GHOST_FALLBACK", False) else ((conf, unknown),):
            route = RR.plan(direction, seats, parking, prm=_prm_for(unk, tc), unknown=unk, anchor=anchor,
                            lock=lock, park_w=park_w, extra=extra, hug=hug)
            if route.report.get("feasible"):
                break   # con fantasmas infactible (dos filas de la misma columna pegadas): se planea sin ellos
        return route, seats, unk

    def _adopt(self, route, key, conf, unknown, empty, dm, shift: float = 0.0, hold: bool = False) -> None:
        prev = getattr(self, "_conf_prev", None) or {}
        self._new_sign = any(prev.get(k) != v for k, v in conf.items())   # la ruta nueva trae una señal (o color) que la anterior no
        self._conf_prev = dict(conf)
        self.route = route
        self.route_shift = shift
        self.follower = RR.RouteFollower(route, float(getattr(C, "ROUTE_LOOKAHEAD_MM", 200.0)))
        self.key = key
        self.n_plans += 1
        self._plan_frame = self._step_frame + 1  # la siguiente llamada ya encuentra la ruta adoptada
        self.upload = self._upload_lines(route, float(getattr(C, "ROUTE_UPLOAD_STEP_MM", 40.0)))
        self._upload_full = list(self.upload)
        self._rte_n = None
        self.upload_shift = shift
        self.hold = hold
        self.loc.set_map(conf, unknown, dm.parking, self.park_w)
        rep = route.report
        print(f"[ROUTE] plan#{self.n_plans} {dm.direction} park={dm.parking}@{self.park_w:.0f}"
              f"{'' if self.fixed else ' (supuesto)'}{' ALTO' if hold else ''} conf={len(conf)} "
              f"vacios={len(empty)} desconocidos={len(unknown)} feasible={rep.get('feasible')} "
              f"sign={rep.get('sign_mm', 0):.0f} wall={rep.get('wall_mm', 0):.0f} "
              f"isla={rep.get('island_mm', 0):.0f} Rmin={rep.get('r_min_mm', 0):.0f} "
              f"L={route.length_mm:.0f} {rep.get('solve_ms', 0):.0f}ms "
              f"señales=[{' '.join(f'{sc}/{sd}{str(col)[0]}' for (sc, sd), col in sorted(conf.items()))}]", flush=True)

    def _speed(self, unknown, pose) -> int | None:
        """PWM máximo según lo que falta por ver.

        Distancia a cada asiento sin resolver = la menor entre (a) sobre la
        ruta, si la ruta pasa a < 260 mm, y (b) en línea recta desde la cámara
        si está adelante: al confirmarse puede cambiar de lado la ruta aunque
        hoy pase lejos (semilla 9: rojo N/T4 confirmado a 540 mm, a toda
        velocidad, sin alcanzar a cruzar).
        """
        if not unknown:
            return None
        r = self.route
        n = len(r.x)
        step = r.length_mm / n
        i0 = self.follower.i
        horizon = int(1600.0 / step)
        idx = (i0 + np.arange(horizon)) % n
        px, py = r.x[idx], r.y[idx]
        x, y, hd = pose
        h = math.radians(hd)
        fx, fy = math.sin(h), math.cos(h)
        cam = float(getattr(C, "CAMERA_FWD_MM", 140.0))
        cx, cy = x + fx * cam, y + fy * cam
        best = None          # el más cercano
        best_vis = None      # el más cercano que la cámara está viendo ahora
        wall = None          # el más cercano de la fila exterior que la ruta pasa por el lado de la pared
        for sec, seat in unknown:
            sx, sy = section_to_world(sec, *SEATS[seat])
            cand = []
            d = np.hypot(px - sx, py - sy)
            hit = np.flatnonzero(d < 260.0)
            if len(hit):
                cand.append(hit[0] * step)
                # Ruta por fuera de un asiento exterior sin ver (semilla 34: el plan
                # llega tarde, ya no cabe por dentro; si sale verde no hay espacio para cruzar).
                j = hit[np.argmin(d[hit])]
                ring = max(abs(sx), abs(sy))
                if ring > 1000.0 and max(abs(px[j]), abs(py[j])) > ring:
                    wall = hit[0] * step if wall is None else min(wall, hit[0] * step)
            fwd = (sx - cx) * fx + (sy - cy) * fy
            if fwd > -100.0:
                cand.append(math.hypot(sx - cx, sy - cy))
            if not cand:
                continue
            s = min(cand)
            best = s if best is None else min(best, s)
            if (sec, seat) in self.visible:
                best_vis = s if best_vis is None else min(best_vis, s)
        wall_pwm = getattr(C, "ROUTE_UNSEEN_WALL_PWM", None)

        def wcap(v):
            if wall_pwm is None or wall is None or wall > float(getattr(C, "ROUTE_SLOWDOWN_MM", 1300.0)):
                return v
            return int(wall_pwm) if v is None else min(v, int(wall_pwm))

        cap = int(getattr(C, "ROUTE_LAP1_PWM", 120))
        # Nada sin ver cerca adelante: lo ya visto no frena (manda el perfil).
        open_cap = None if getattr(C, "ROUTE_LAP1_OPEN", False) else cap
        if best is None:
            return wcap(open_cap)
        stop = float(getattr(C, "ROUTE_STOP_MM", 750.0))
        slow_mm = float(getattr(C, "ROUTE_SLOWDOWN_MM", 1300.0))
        # Recta de salida del cajón (tc=0): piso propio cerca de asientos sin ver
        # (orillas69-72: ~1 s a rv 130 = ~320 mm/s justo al salir). None = ROUTE_UNSEEN_PWM.
        unseen_pwm = _pwm("ROUTE_UNSEEN_PWM")
        _exit = getattr(C, "ROUTE_EXIT_UNSEEN_PWM", None)
        if _exit is not None and float(getattr(self, "_tc_max", 0) or 0) < 1:
            unseen_pwm = int(_exit)
        # Detenerse solo sirve si la cámara lo está viendo (en ~0.4 s queda
        # resuelto). Fuera de vista se sigue lento: la ruta ya lo esquiva
        # como obstáculo y al avanzar/girar entra a la vista.
        if best_vis is not None and best_vis <= stop:
            # Nunca detenido indefinidamente: la ruta ya esquiva lo desconocido.
            self.stopped += 1
            if self.stopped <= int(getattr(C, "ROUTE_STOP_MAX_FRAMES", 80)):
                return 0
            return wcap(min(cap, unseen_pwm))
        self.stopped = 0
        if best <= slow_mm:
            unseen = min(cap, unseen_pwm)
            ramp = getattr(C, "ROUTE_UNSEEN_RAMP_MM", None)
            if ramp is None or best <= float(ramp):
                return wcap(unseen)
            # v tal que, con latencia y frenando a A_BRAKE, llega a v_unseen a `ramp` mm del asiento.
            K = float(C.ROUTE_PROFILE_K_MM_S_PER_PWM)
            pwm0 = float(C.ROUTE_PROFILE_PWM0)
            a = float(C.ROUTE_PROFILE_A_BRAKE_MM_S2)
            lat = float(C.ROUTE_PROFILE_LATENCY_S)
            vu = K * (unseen - pwm0)
            v = -a * lat + math.sqrt((a * lat) ** 2 + vu * vu + 2.0 * a * (best - float(ramp)))
            top = cap if open_cap is not None else int(C.ROUTE_PROFILE_MAX_PWM)
            return wcap(int(round(min(max(v / K + pwm0, unseen), top))))
        return wcap(open_cap)

    def _suffix(self) -> str:
        tf = self.loc.transform() if self.loc is not None else None
        if tf is None:
            return ""
        tx, ty, th = tf
        # La ruta activa del ESP puede estar en el marco anterior al arranque
        # ciego resuelto: se le da la transformación de ESE marco.
        ds = self.shift - self.esp_shift
        tx -= ds * self.axis[0]
        ty -= ds * self.axis[1]
        s = f",rt=1,rtx={tx:.1f},rty={ty:.1f},rth={th:.3f}"
        if self.pi_steer:
            s += f",ps=1,pw={self.pi_wheel:.2f}"
        if self.fixed and getattr(self, "pka", None) is not None:
            s += f",pka={self.pka:.0f}"
        if self.rv is not None:
            s += f",rv={_rv_mando(self.rv)}"
        return s

    @staticmethod
    def _upload_lines(route, paso_mm: float = 40.0, per_line: int = 10) -> list[str]:
        """RTB / RTP / RTE de la ruta (mm del tapete, enteros) para el ESP."""
        k = max(1, int(round(paso_mm / (route.length_mm / len(route.x)))))
        xs = route.x[::k]
        ys = route.y[::k]
        n = len(xs)
        paso = route.length_mm / n
        lines = [f"RTB,{n},{paso:.2f}"]
        for i in range(0, n, per_line):
            pts = ",".join(f"{int(round(xs[j]))},{int(round(ys[j]))}" for j in range(i, min(n, i + per_line)))
            lines.append(f"RTP,{i},{pts}")
        lines.append(f"RTE,{n}")
        return lines
