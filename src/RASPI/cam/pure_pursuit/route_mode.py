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

import math
import os

import cv2
import numpy as np

from . import config as C
from . import ring_route as RR
from .digital_map import feet_world
from .localizer import Localizer
from .start_fix import StartFix
from .wro_field import (INNER_HALF_MM, OUTER_HALF_MM, PARK_W_MAX, PARK_W_MIN, PARKING_FORBIDDEN_SEATS, SEATS,
                        SECTIONS_CW, orange_segments, section_to_world, world_to_section)

_ROW_H = SEATS["T1"][0]          # fila interior (600 mm de la pared exterior)
_ROW_BAND_MM = 160.0             # detección que cae en la fila interior
_CLUSTER_GAP_MM = 150.0
_FENCE_STEP_MM = 100.0

def _route_params():
    """Parámetros del planificador con lo ajustable desde config.py."""
    K = float(getattr(C, "ROUTE_PROFILE_K_MM_S_PER_PWM", 3.5))
    pwm0 = float(getattr(C, "ROUTE_PROFILE_PWM0", 25.0))
    return RR.RouteParams(
        w_kappa=float(getattr(C, "ROUTE_W_KAPPA", 3.0e4)),
        hard_clear_mm=float(getattr(C, "ROUTE_HARD_CLEAR_MM", 55.0)),
        soft_clear_mm=float(getattr(C, "ROUTE_SOFT_CLEAR_MM", 140.0)),
        w_clear=float(getattr(C, "ROUTE_W_CLEAR", 8.0)),
        time_cost=bool(getattr(C, "ROUTE_TIME_COST", False)),
        v_min_mm_s=K * (float(getattr(C, "ROUTE_PROFILE_BASE_PWM", 155)) - pwm0),
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


class RouteMode:
    def __init__(self) -> None:
        self.route = None
        self.follower = None
        self.loc = None
        self.key = None
        self.upload: list[str] = []
        self.esp_route = None        # última ruta completa en el ESP (tras su RTE)
        self.empty: set = set()
        self.view_count: dict = {}
        self.rv = None
        self.visible: set = set()
        self.stopped = 0
        self.last_print = None
        self.n_plans = 0
        self._job = None
        self._job_key = None
        self._job_out = None
        self._job_conf = {}
        self._job_unknown = []
        self._job_shift = 0.0
        self._delay_left = None      # frames que faltan para adoptar el plan retrasado (ROUTE_PLAN_DELAY_S)
        self._delay_plan = None
        # Arranque ciego
        self.blind = bool(getattr(C, "ROUTE_START_BLIND", False))
        self.fix = None              # StartFix mientras no se sabe el avance
        self.orange = None           # estado de la línea naranja (OrangeLineTracker) del frame
        self._line_d: list = []      # δ por lectura de la línea naranja (ROUTE_FIX_LINE)
        self.fixed = not self.blind
        self.shift = 0.0             # δ aplicado: verdad = supuesto + δ·w de la recta del cajón
        self.axis = (0.0, 0.0)
        self.park_w = None           # w0 del cajón en uso (supuesto o medido)
        self.route_shift = 0.0       # marco de self.route
        self.esp_shift = 0.0         # marco de la ruta activa en el ESP
        self.upload_shift = 0.0      # marco de la ruta que se está subiendo
        self.hold = False            # detenido hasta que la ruta nueva llegue al ESP
        self._def_seen = 0
        self._pre_dets: list = []    # (w, color, dist a la cámara) en la fila interior
        self._pre_poses: list = []
        self._fence = None
        self._pre_key = None
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
        self.rv = None
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
        self.loc.update(ack, correct=(est in ("S", "C", "R") and _ack_est(ack) in ("S", "C", "R")),
                        shifts=shifts)
        pose = self.loc.pose() if self.loc.odom is not None else (dm.pose_xy[0], dm.pose_xy[1], float(dm.heading))
        if self.loc.odom is not None:
            self._sync_map(dm, ack, pose)
        if not self.fixed:
            self._pre_poses.append(pose)
            self.fix.update(ack, pose)
            if getattr(C, "ROUTE_FIX_LINE", None): self._line_vote(dm, pose)
            delta = self.fix.converged()
            forced = False
            if delta is None and self._past_straight(dm, pose):
                # Sin pico al final de la recta: la media (sin lecturas que
                # decidan, el histograma queda plano cerca de lo supuesto).
                delta = self.fix.mean()
                if getattr(C, "ROUTE_FIX_LINE", None) and len(self._line_d) >= int(getattr(C, "ROUTE_FIX_LINE_MIN", 3)):
                    delta = float(np.median(self._line_d))
                forced = True
            if delta is not None:
                pose = self._apply_fix(dm, ack, delta, forced)
                conf, empty = self._resolved(dm)
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

        key = (dm.direction, dm.parking, self.park_w, self.shift,
               tuple(sorted(plan_seats.items())), tuple(sorted(plan_unknown)), extra_key)
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
                lock = {(k0 + j) % nst: float(self.route.d_stations[(k0 + j) % nst]) for j in range(nl)}
                anchor = None
            args = (dm.direction, plan_seats, dm.parking, plan_unknown, anchor, lock, self.park_w, extra)
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
                    self._job = threading.Thread(target=self._run_plan, args=args, daemon=True)
                    self._job.start()
            elif getattr(C, "ROUTE_PLAN_DELAY_S", None) is not None and self.route is not None:
                # Twin: el plan sale ya, pero se adopta tras el retraso de la Pi (ruta anterior mientras).
                if self._delay_left is None:
                    self._job_key = key
                    self._delay_plan = (RR.plan(args[0], args[1], args[2], prm=_route_params(), unknown=args[3],
                                                anchor=args[4], lock=args[5], park_w=args[6], extra=args[7]),
                                        key, dict(plan_seats), list(plan_unknown), self.shift, hold)
                    self._delay_left = int(round(float(C.ROUTE_PLAN_DELAY_S) * float(getattr(C, "PI_FPS", 40.0))))
            else:
                self._adopt(RR.plan(args[0], args[1], args[2], prm=_route_params(), unknown=args[3],
                                    anchor=args[4], lock=args[5], park_w=args[6], extra=args[7]),
                            key, plan_seats, plan_unknown, empty, dm, self.shift, hold)
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
        servo = wheel * float(C.SERVO_FULL_LOCK_CMD_DEG) / float(C.MAX_WHEEL_STEER_DEG)
        obs = max(-1.0, min(1.0, servo / C.PP_STEER_GAIN))
        slow = int(getattr(C, "ROUTE_SLOW_PWM", 100))
        if self.fixed:
            # Los asientos con color por ROI ya tienen lado: no frenan (ni cuentan para la parada).
            speed_unknown = [k for k in unknown if k not in roi_seats]
            self.rv = self._speed(speed_unknown, (x - ds * ax, y - ds * ay, hd)) if lap1 else None
        else:
            # Sin el avance: lento, la esquina todavía no se sabe dónde está.
            blind = getattr(C, "ROUTE_BLIND_PWM", None)
            self.rv = slow if blind is None else int(blind)
        if abs(self.follower.cte) > float(getattr(C, "ROUTE_CTE_SLOW_MM", 40.0)):
            # lejos de la ruta (recién replaneada o perturbado): lento hasta volver
            cte_cap = _pwm("ROUTE_CTE_PWM")
            if getattr(C, "ROUTE_CTE_URGENT_MM", None) is not None and self._cte_urgent(conf, empty):
                cte_cap = min(cte_cap, int(getattr(C, "ROUTE_CTE_URGENT_PWM", 120)))
            self.rv = cte_cap if self.rv is None else min(self.rv, cte_cap)
        if self.upload:
            # Mientras la ruta nueva no llega completa al ESP, lento; detenido
            # si cambió desde la pose (otro carril cerca del carro).
            # ROUTE_UPLOAD_SLOW=False: el ESP sigue la ruta anterior completa
            # (doble buffer, swap en RTE), así que solo se frena en el ALTO.
            if self.hold or (bool(getattr(C, "ROUTE_UPLOAD_SLOW", True)) and self._upload_risky()):
                cap = 0 if self.hold else _pwm("ROUTE_UPLOAD_PWM")
                self.rv = cap if self.rv is None else min(self.rv, cap)
        else:
            self.hold = False
        if getattr(C, "ROUTE_PENDING_PWM", None) is not None and (self._delay_left is not None or self._job is not None):
            # Plan en curso (la DP tarda en la Pi): la ruta vieja puede pasar por la señal recién vista.
            cap = int(C.ROUTE_PENDING_PWM)
            self.rv = cap if self.rv is None else min(self.rv, cap)
        # Perfil por curvatura: solo baja el tope (los de arriba siguen ganando).
        prof = self._profile_pwm(self._sign_xy(conf, empty))
        if prof is not None:
            self.rv = prof if self.rv is None else min(self.rv, prof)
        self.dbg = (round(x), round(y), round(hd, 1), round(wheel, 1), round(self.follower.cte),
                    self.follower.i, self.rv, len(unknown))
        line = self.upload.pop(0) if (self.upload and getattr(C, "ROUTE_ON_ESP", False)) else None
        if line is not None and line.startswith("RTE"):
            self.esp_shift = self.upload_shift
            self.esp_route = self.route
        suffix = self._suffix() if getattr(C, "ROUTE_ON_ESP", False) else ""
        return obs, suffix, line

    def _upload_risky(self) -> bool:
        """Mientras sube la ruta: ¿la nueva se aparta de la del ESP en el tramo de adelante?

        Sin ROUTE_UPLOAD_DIFF_MM, sin ruta previa en el ESP o en otro marco (arranque
        ciego), siempre True (frenar, como antes).
        """
        tol = getattr(C, "ROUTE_UPLOAD_DIFF_MM", None)
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

    def _sign_xy(self, conf, empty) -> np.ndarray:
        """Posiciones (x, y) en mm del tapete de las señales y asientos sin resolver.

        Todo asiento que no está marcado vacío: las confirmadas y las que aún no
        se saben (podrían ser señal). Mismo marco que la ruta (section_to_world).
        """
        pts = [section_to_world(sec, *SEATS[seat]) for sec, seat in self._all_seats()
               if (sec, seat) in conf or (sec, seat) not in empty]
        return np.asarray(pts, dtype=float).reshape(-1, 2)

    def _profile_pwm(self, signs=None) -> int | None:
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
        base = int(C.ROUTE_PROFILE_BASE_PWM)
        top = int(C.ROUTE_PROFILE_MAX_PWM)
        v_base = K * (base - pwm0)
        v_max = K * (top - pwm0)
        # Perfil v3: piso real del perfil (MIN_PWM, sin pasar de la base). None: la base.
        mn = getattr(C, "ROUTE_PROFILE_MIN_PWM", None)
        floor = base if mn is None else min(int(mn), base)
        v_min = K * (floor - pwm0)
        tight_on = False
        a_lat = float(C.ROUTE_PROFILE_A_LAT_MM_S2)
        a_brk = float(C.ROUTE_PROFILE_A_BRAKE_MM_S2)
        win = int(C.ROUTE_PROFILE_KAPPA_WIN)
        # Adelanto por latencia a la velocidad máxima (conservador).
        lead = v_max * float(C.ROUTE_PROFILE_LATENCY_S)
        span = float(C.ROUTE_PROFILE_HORIZON_MM) + lead
        step = r.length_mm / n
        m = int(span / step) + 1
        i = max(self.follower.i, 0)
        # Anillo cerrado: índices módulo n. Ventana ±win para |κ| máximo móvil.
        idx = (i + np.arange(-win, m + win)) % n
        kmax = np.lib.stride_tricks.sliding_window_view(np.abs(r.kappa[idx]), 2 * win + 1).max(axis=1)
        j = idx[win:win + m]
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
            pxy = np.stack((r.x[j], r.y[j]), axis=1)                       # (m, 2)
            dd = np.hypot(pxy[:, None, 0] - signs[None, :, 0],
                          pxy[:, None, 1] - signs[None, :, 1]).min(axis=1)  # (m,)
            if mn is None:
                v_c = np.where(dd < sign_mm, v_base, v_c)                  # igual que antes
            else:
                # Cerca de señal: nunca más que v_señal ni que la curva (ya no se fuerza a la base).
                near = getattr(C, "ROUTE_PROFILE_SIGN_NEAR_MM", None)
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
        lo = floor
        ff = getattr(C, "ROUTE_PROFILE_BRAKE_FF_S", None)
        if ff is not None and d[k_min] > 0.0 and v_allow < v_max:
            # Frenando hacia una muestra de adelante: se adelanta el retraso del motor.
            v_allow -= float(ff) * a_brk
            lo = min(floor, int(getattr(C, "ROUTE_PROFILE_BRAKE_FLOOR_PWM", 60)))
        if tight_on:
            lo = min(lo, int(getattr(C, "ROUTE_PROFILE_SIGN_TIGHT_PWM", 110)))
        pwm = v_allow / K + pwm0
        return int(round(min(max(pwm, lo), top)))

    def _past_straight(self, dm, pose) -> bool:
        """El frente del carro ya llegó (en el marco supuesto) al final de la recta del cajón."""
        _h, w = world_to_section(dm.parking, pose[0], pose[1])
        sgn = 1.0 if dm.direction == "CW" else -1.0
        front = w + sgn * RR._FRONT_MM
        return front >= 1900.0 if sgn > 0 else front <= 1100.0

    def _line_vote(self, dm, pose):
        """δ del cajón según la línea naranja: punto de la recta en la columna central del BEV, llevado al mundo con la pose supuesta."""
        o = self.orange
        if not o or not o.get("seen") or not o.get("line") or o.get("dead_reckoned"):
            return
        vx, vy, x0, y0 = o["line"]
        if abs(vx) < 1e-6:
            return
        by = y0 + (float(C.ROBOT_BEV_X) - x0) * vy / vx
        fwd = (C.ROBOT_BEV_Y - by) * C.MM_PER_PX + float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0))
        if not 100.0 <= fwd <= 600.0:
            return
        h = math.radians(pose[2])
        p = np.array([pose[0] + fwd * math.sin(h), pose[1] + fwd * math.cos(h)])
        a = np.array(self.axis)
        span = PARK_W_MAX - PARK_W_MIN
        lo, hi = (-150.0, span + 150.0) if dm.direction == "CW" else (-span - 150.0, 150.0)
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
            self._line_d.append(best[0])

    def _apply_fix(self, dm, ack, delta: float, forced: bool):
        """Avance resuelto: corre la pose, vota lo guardado y repasa las vistas."""
        ax, ay = self.axis
        dx, dy = delta * ax, delta * ay
        mass = self.fix.estimate(float(getattr(C, "ROUTE_FIX_WIN_MM", 20.0)))[1]
        self.shift = float(delta)
        self.fixed = True
        self.fix.fixed = float(delta)
        self.loc.shift(dx, dy)
        self.park_w = self.fix.w0_guess + float(delta)
        dm.park_w = self.park_w
        n = dm.replay_deferred(dx, dy)
        pose = self.loc.pose()
        self._sync_map(dm, ack, pose)
        if bool(getattr(C, "ROUTE_LAP1", True)):
            for px, py, ph in self._pre_poses:
                conf, empty = self._resolved(dm)
                self._update_views(conf, empty, (px + dx, py + dy, ph))
        self._pre_poses = []
        how = "FORZADO" if forced else f"masa {mass:.2f}"
        if getattr(C, "ROUTE_FIX_LINE", None):
            how += f" línea n={len(self._line_d)}"
        print(f"[ROUTE] arranque: cajón en w0={self.park_w:.0f} (δ={delta:+.0f} mm, {how}) tras "
              f"{self.fix.n_frames} lecturas ({self.fix.n_inform} informativas); "
              f"{n} frames de cámara votados", flush=True)
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

    def _run_plan(self, direction, conf, parking, unknown, anchor, lock, park_w=None, extra=None) -> None:
        self._job_conf = conf
        self._job_unknown = unknown
        self._job_out = RR.plan(direction, conf, parking, prm=_route_params(), unknown=unknown, anchor=anchor,
                                lock=lock, park_w=park_w, extra=extra)

    def _adopt(self, route, key, conf, unknown, empty, dm, shift: float = 0.0, hold: bool = False) -> None:
        self.route = route
        self.route_shift = shift
        self.follower = RR.RouteFollower(route, float(getattr(C, "ROUTE_LOOKAHEAD_MM", 200.0)))
        self.key = key
        self.n_plans += 1
        self.upload = self._upload_lines(route)
        self.upload_shift = shift
        self.hold = hold
        self.loc.set_map(conf, unknown, dm.parking, self.park_w)
        rep = route.report
        print(f"[ROUTE] plan#{self.n_plans} {dm.direction} park={dm.parking}@{self.park_w:.0f}"
              f"{'' if self.fixed else ' (supuesto)'}{' ALTO' if hold else ''} conf={len(conf)} "
              f"vacios={len(empty)} desconocidos={len(unknown)} feasible={rep.get('feasible')} "
              f"sign={rep.get('sign_mm', 0):.0f} wall={rep.get('wall_mm', 0):.0f} "
              f"isla={rep.get('island_mm', 0):.0f} Rmin={rep.get('r_min_mm', 0):.0f} "
              f"L={route.length_mm:.0f} {rep.get('solve_ms', 0):.0f}ms", flush=True)

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
        for sec, seat in unknown:
            sx, sy = section_to_world(sec, *SEATS[seat])
            cand = []
            d = np.hypot(px - sx, py - sy)
            hit = np.flatnonzero(d < 260.0)
            if len(hit):
                cand.append(hit[0] * step)
            fwd = (sx - cx) * fx + (sy - cy) * fy
            if fwd > -100.0:
                cand.append(math.hypot(sx - cx, sy - cy))
            if not cand:
                continue
            s = min(cand)
            best = s if best is None else min(best, s)
            if (sec, seat) in self.visible:
                best_vis = s if best_vis is None else min(best_vis, s)
        cap = int(getattr(C, "ROUTE_LAP1_PWM", 120))
        # Nada sin ver cerca adelante: lo ya visto no frena (manda el perfil).
        open_cap = None if getattr(C, "ROUTE_LAP1_OPEN", False) else cap
        if best is None:
            return open_cap
        stop = float(getattr(C, "ROUTE_STOP_MM", 750.0))
        slow_mm = float(getattr(C, "ROUTE_SLOWDOWN_MM", 1300.0))
        # Detenerse solo sirve si la cámara lo está viendo (en ~0.4 s queda
        # resuelto). Fuera de vista se sigue lento: la ruta ya lo esquiva
        # como obstáculo y al avanzar/girar entra a la vista.
        if best_vis is not None and best_vis <= stop:
            # Nunca detenido indefinidamente: la ruta ya esquiva lo desconocido.
            self.stopped += 1
            if self.stopped <= int(getattr(C, "ROUTE_STOP_MAX_FRAMES", 80)):
                return 0
            return min(cap, _pwm("ROUTE_UNSEEN_PWM"))
        self.stopped = 0
        if best <= slow_mm:
            unseen = min(cap, _pwm("ROUTE_UNSEEN_PWM"))
            ramp = getattr(C, "ROUTE_UNSEEN_RAMP_MM", None)
            if ramp is None or best <= float(ramp):
                return unseen
            # v tal que, con latencia y frenando a A_BRAKE, llega a v_unseen a `ramp` mm del asiento.
            K = float(C.ROUTE_PROFILE_K_MM_S_PER_PWM)
            pwm0 = float(C.ROUTE_PROFILE_PWM0)
            a = float(C.ROUTE_PROFILE_A_BRAKE_MM_S2)
            lat = float(C.ROUTE_PROFILE_LATENCY_S)
            vu = K * (unseen - pwm0)
            v = -a * lat + math.sqrt((a * lat) ** 2 + vu * vu + 2.0 * a * (best - float(ramp)))
            top = cap if open_cap is not None else int(C.ROUTE_PROFILE_MAX_PWM)
            return int(round(min(max(v / K + pwm0, unseen), top)))
        return open_cap

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
        if self.rv is not None:
            s += f",rv={int(self.rv)}"
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
