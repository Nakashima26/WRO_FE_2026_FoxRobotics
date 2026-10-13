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

import numpy as np

from . import config as C
from . import ring_route as RR
from .digital_map import feet_world
from .localizer import Localizer
from .start_fix import StartFix
from .wro_field import (PARK_W_MAX, PARK_W_MIN, PARKING_FORBIDDEN_SEATS, SEATS, SECTIONS_CW,
                        section_to_world, world_to_section)

_ROW_H = SEATS["T1"][0]          # fila interior (600 mm de la pared exterior)
_ROW_BAND_MM = 160.0             # detección que cae en la fila interior
_CLUSTER_GAP_MM = 150.0
_FENCE_STEP_MM = 100.0

def _route_params():
    """Parámetros del planificador con lo ajustable desde config.py."""
    return RR.RouteParams(w_kappa=float(getattr(C, "ROUTE_W_KAPPA", 3.0e4)))


def _ack_est(ack):
    if not ack or "est=" not in ack:
        return None
    return ack.split("est=", 1)[1][:1]


_COL = {"T4": 0, "T2": 0, "X2": 1, "X1": 1, "T3": 2, "T1": 2}    # columna (w 1000/1500/2000)


class RouteMode:
    def __init__(self) -> None:
        self.route = None
        self.follower = None
        self.loc = None
        self.key = None
        self.upload: list[str] = []
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
        # Arranque ciego
        self.blind = bool(getattr(C, "ROUTE_START_BLIND", False))
        self.fix = None              # StartFix mientras no se sabe el avance
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

    # ── asientos ──────────────────────────────────────────────────────────
    def _resolved(self, dm) -> tuple[dict, set]:
        conf = dict(dm.confirmed)
        empty = set(self.empty)
        for seat in PARKING_FORBIDDEN_SEATS:
            if (dm.parking, seat) not in conf:
                empty.add((dm.parking, seat))
        for sec in SECTIONS_CW:
            mine = [s for (sc, s) in conf if sc == sec]
            if not mine:
                # Toda recta tiene al menos una señal: si los 6 asientos
                # quedaron "vacíos" la visión se la perdió; se vuelven a mirar.
                if all((sec, s) in empty for s in SEATS):
                    for s in SEATS:
                        if (sec, s) in self.empty:
                            self.empty.discard((sec, s))
                            empty.discard((sec, s))
                            self.view_count[(sec, s)] = 0
                continue
            cols = {_COL[s] for s in mine}
            for seat in SEATS:
                if (sec, seat) in conf:
                    continue
                # X va sola; nunca dos en la misma columna; máximo dos por recta.
                if (any(s.startswith("X") for s in mine) or seat.startswith("X")
                        or _COL[seat] in cols or len(mine) >= 2):
                    empty.add((sec, seat))
        return conf, empty

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
            delta = self.fix.converged()
            forced = False
            if delta is None and self._past_straight(dm, pose):
                # Sin pico al final de la recta: la media (sin lecturas que
                # decidan, el histograma queda plano cerca de lo supuesto).
                delta = self.fix.mean()
                forced = True
            if delta is not None:
                pose = self._apply_fix(dm, ack, delta, forced)
                conf, empty = self._resolved(dm)
        if lap1 and self.fixed:
            self._update_views(conf, empty, pose)
            conf, empty = self._resolved(dm)
        unknown = [k for k in self._all_seats() if k not in conf and k not in empty]
        # Color provisional: el mapa confirma ya cerca (la detección tiene que
        # caer a <=160 mm del asiento); los votos de color llegan antes. Con
        # >=2 votos y ventaja, la ruta ya toma el lado de ese color (semilla 87:
        # rojo confirmado a 585 mm, la ruta lo rodeaba por el lado malo).
        prov = {}
        nmin = int(getattr(C, "ROUTE_PROVISIONAL_VOTES", 2))
        for k in unknown:
            v = getattr(dm, "_votes", {}).get(k)
            if not v:
                continue
            for col, oth in (("Red", "Green"), ("Green", "Red")):
                if v.get(col, 0) >= nmin and v.get(col, 0) >= v.get(oth, 0) + 1:
                    prov[k] = col
        plan_seats = {**conf, **prov}
        plan_unknown = [k for k in unknown if k not in prov]
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
            else:
                self._adopt(RR.plan(args[0], args[1], args[2], prm=_route_params(), unknown=args[3],
                                    anchor=args[4], lock=args[5], park_w=args[6], extra=args[7]),
                            key, plan_seats, plan_unknown, empty, dm, self.shift, hold)
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
            self.rv = self._speed(unknown, (x - ds * ax, y - ds * ay, hd)) if lap1 else None
        else:
            # Sin el avance: lento, la esquina todavía no se sabe dónde está.
            self.rv = slow
        if abs(self.follower.cte) > float(getattr(C, "ROUTE_CTE_SLOW_MM", 40.0)):
            # lejos de la ruta (recién replaneada o perturbado): lento hasta volver
            self.rv = slow if self.rv is None else min(self.rv, slow)
        if self.upload:
            # Mientras la ruta nueva no llega completa al ESP, lento; detenido
            # si cambió desde la pose (otro carril cerca del carro).
            cap = 0 if self.hold else slow
            self.rv = cap if self.rv is None else min(self.rv, cap)
        else:
            self.hold = False
        self.dbg = (round(x), round(y), round(hd, 1), round(wheel, 1), round(self.follower.cte),
                    self.follower.i, self.rv, len(unknown))
        line = self.upload.pop(0) if (self.upload and getattr(C, "ROUTE_ON_ESP", False)) else None
        if line is not None and line.startswith("RTE"):
            self.esp_shift = self.upload_shift
        suffix = self._suffix() if getattr(C, "ROUTE_ON_ESP", False) else ""
        return obs, suffix, line

    def _past_straight(self, dm, pose) -> bool:
        """El frente del carro ya llegó (en el marco supuesto) al final de la recta del cajón."""
        _h, w = world_to_section(dm.parking, pose[0], pose[1])
        sgn = 1.0 if dm.direction == "CW" else -1.0
        front = w + sgn * RR._FRONT_MM
        return front >= 1900.0 if sgn > 0 else front <= 1100.0

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
        if best is None:
            return cap
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
            return min(cap, int(getattr(C, "ROUTE_SLOW_PWM", 100)))
        self.stopped = 0
        if best <= slow_mm:
            return min(cap, int(getattr(C, "ROUTE_SLOW_PWM", 100)))
        return cap

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
