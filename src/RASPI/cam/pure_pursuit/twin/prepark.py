"""Escenarios cortos de ESTACIONAMIENTO (T15b): el carro arranca a la salida de
la esquina 12, ya con tc=12, y solo hace la maniobra final.

Escenario = sentido (CW/CCW) × carta de la recta del cajón (solo las que el
reglamento deja ahí: 1-6 y 25-30, fila interior; 28≡26 y 29≡27 son la misma
pista) × perturbación de la pose de entrada (lateral ±30 mm, rumbo ±5°) y del
error de odometría del ESP (lo que deriva en 3 vueltas).

Entrada nominal medida en carreras completas hw_nuevo (pose real cuando tc
pasa a 12): CW h≈360-380 mm de la pared exterior, w≈630-900 (todavía en la
esquina), rumbo 12-18° hacia la pared (el giro no terminó); CCW h≈460-800,
w≈2480-2720, rumbo 2-10° hacia la pared.

El firmware entra por PARK_TEST_RECTA_COMPLETA (tc := 12 y estaciona) y lee
de sil_param la pose que tendría su odometría en el marco del arranque en el
cajón (pp_x, pp_y, pp_yaw_total, pp_ang) y de qué lado está la pared del
cajón (pp_wall_left). La Pi ve tc=12 en el ACK como en la carrera.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pure_pursuit import wro_field as W

STALL_CARDS = (1, 2, 3, 4, 5, 6, 25, 26, 27, 30)

# Entrada nominal (h = mm del eje trasero a la pared exterior, w = mm a lo
# largo de la recta como en el programa oficial, rumbo hacia la pared en °).
ENTRY = {
    "CW": dict(h=370.0, w=850.0, toward_deg=10.0),
    "CCW": dict(h=460.0, w=2400.0, toward_deg=5.0),
}
# (dh lateral mm, d_rumbo °, error odometría x mm, error odometría y mm)
PERTURB = {
    "p0": (0.0, 0.0, 0.0, 0.0),
    "p1": (30.0, 5.0, 60.0, -40.0),
    "p2": (-30.0, -5.0, -60.0, 40.0),
    "p3": (30.0, -5.0, -60.0, -40.0),
    "p4": (-30.0, 5.0, 60.0, 40.0),
}


@dataclass
class Scenario:
    direction: str
    card: int
    pert: str
    parking: str = "N"

    @property
    def name(self) -> str:
        return f"{self.direction}_c{self.card}_{self.pert}"


def all_scenarios(perts=None, dirs=("CW", "CCW"), cards=STALL_CARDS) -> list[Scenario]:
    perts = list(perts or PERTURB)
    return [Scenario(d, c, p) for d in dirs for c in cards for p in perts]


def _units(heading_deg: float):
    h = math.radians(heading_deg)
    fwd = (math.sin(h), math.cos(h))
    left = (-math.cos(h), math.sin(h))
    return fwd, left


def build(sc: Scenario):
    """Field con solo las señales de la recta del cajón + parámetros SIL."""
    sec = sc.parking
    signs = []
    for seat, color in W.CARDS[sc.card - 1]:
        h, w = W.SEATS[seat]
        x, y = W.section_to_world(sec, h, w)
        signs.append(W.Sign(x, y, color, sec, seat))
    lane = W._heading(sec, sc.direction)
    e = ENTRY[sc.direction]
    dh, dth, ex, ey = PERTURB[sc.pert]
    sx, sy = W.section_to_world(sec, e["h"] + dh, e["w"])
    fwd0, left0 = _units(lane)
    # "Hacia la pared": la pared del cajón es la exterior. CW = a la izquierda
    # del sentido de carrera, CCW = a la derecha.
    wall_left = sc.direction == "CW"
    toward = e["toward_deg"] + dth
    yaw_ccw = toward if wall_left else -toward       # + = antihorario (ESP)
    heading = (lane - yaw_ccw) % 360.0                # twin: + = horario
    field = W.Field(
        seed=0,
        direction=sc.direction,
        single_section=sec,
        parking_section=sec,
        cards={s: (sc.card if s == sec else 0) for s in W.SECTIONS_CW},
        signs=signs,
        barriers=W._barrier_corners(sec),
        start_x=sx,
        start_y=sy,
        start_heading=heading,
        start_mode="pre_park",
    )
    # Odometría del ESP: origen = eje trasero del arranque en el cajón, x hacia
    # el rumbo de salida, y a la izquierda; yaw continuo (12 giros = ∓1080°).
    x0, y0 = W.stall_start_xy(sec, sc.direction)
    dx, dy = sx - x0, sy - y0
    px = dx * fwd0[0] + dy * fwd0[1]
    py = dx * left0[0] + dy * left0[1]
    turns = -1080.0 if sc.direction == "CW" else 1080.0
    params = {
        "pp_on": 1.0,
        "pp_x": px + ex,
        "pp_y": py + ey,
        "pp_yaw_total": turns + yaw_ccw,
        "pp_ang": yaw_ccw,
        "pp_wall_left": 1.0 if wall_left else 0.0,
    }
    return field, params


def stall_box(section: str):
    """Caja libre del cajón (entre caras interiores de las maderas, de la pared
    exterior a las puntas): (x0, x1, y0, y1) en mm del tapete."""
    b1, b2 = W._barrier_corners(section)
    xs = [p[0] for p in b1 + b2]
    ys = [p[1] for p in b1 + b2]
    if section in ("E", "W"):
        lo = min(max(p[1] for p in b1), max(p[1] for p in b2))
        hi = max(min(p[1] for p in b1), min(p[1] for p in b2))
        return (min(xs), max(xs), lo, hi)
    lo = min(max(p[0] for p in b1), max(p[0] for p in b2))
    hi = max(min(p[0] for p in b1), min(p[0] for p in b2))
    return (lo, hi, min(ys), max(ys))


def park_metrics(field, x: float, y: float, heading_deg: float, *, rear_oh: float, length: float,
                 width: float, wheelbase: float) -> dict:
    """Huella final vs caja del cajón y criterio de paralelo del reglamento
    (ruedas motrices de un lado a la pared con diferencia <= 20 mm)."""
    h = math.radians(heading_deg)
    fx, fy = math.sin(h), math.cos(h)
    rx, ry = math.cos(h), -math.sin(h)
    cs = [(x + fx * a + rx * b, y + fy * a + ry * b)
          for a in (-rear_oh, length - rear_oh) for b in (-width / 2, width / 2)]
    box = stall_box(field.parking_section)
    out = max(max(box[0] - cx, cx - box[1], box[2] - cy, cy - box[3], 0.0) for cx, cy in cs)
    lane = W._heading(field.parking_section, field.direction)
    err = (heading_deg - lane + 180.0) % 180.0
    if err > 90.0:
        err -= 180.0                                   # paralelo vale de frente o de reversa
    wheel_diff = abs(math.sin(math.radians(err))) * wheelbase
    return {"fuera_mm": round(out, 1), "rumbo_err_deg": round(err, 1),
            "ruedas_dif_mm": round(wheel_diff, 1), "paralelo": wheel_diff <= 20.0}
