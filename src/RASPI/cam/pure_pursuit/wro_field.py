"""
Pista del Obstacle Challenge, WRO 2026 Future Engineers.

Medidas del reglamento (WRO-2026-Future-Engineers-Self-Driving-Cars):
  - Tapete de juego 3000 × 3000 mm, sin contar el grosor de la pared exterior.
  - Reto de obstáculos: pasillo siempre 1000 mm. La isla interior queda
    de 1000 × 1000 mm (pared interior a 1000 mm del borde exterior).
  - Cuatro rectas y cuatro esquinas. Cada recta es el tramo central de
    1000 mm de cada lado.
  - Señal: paralelepípedo 50 × 50 × 100 mm. Rojo se pasa por la derecha,
    verde por la izquierda. Hasta 7 de cada color.
  - Cajón: dos maderas magenta 200 × 20 × 100 mm. El largo libre es
    1.5 × el largo del robot (180 mm → 270 mm). En el programa oficial
    de 2024 ese hueco está fijo en 300 mm; aquí se usa el reglamento 2026.

Asientos y cartas: el reglamento describe el sorteo (moneda + 36 cartas)
pero las figuras no traen coordenadas. Las coordenadas y el contenido de
las 36 cartas salen del programa oficial
World-Robot-Olympiad-Association/fe-randomization-app (1 px = 1 mm):
  - Asientos T1 T2 T3 T4 X1 X2 en cada recta.
  - Una recta lleva solo X2 (carta 9 verde o 10 roja), otra una de las
    parejas mixtas 22/23/28/29 (required_obstacles_sets) y las otras dos
    cartas al azar. Se re-sortea hasta que |verdes - rojas| <= 1 y haya
    al menos 5 señales.
  - En la recta del cajón no puede haber señal en T3, T4 ni X2
    (forbidden_intersections_in_parking_section): solo entran las cartas
    1-6 y 25-30, todas en la fila interior. Es lo mismo que la figura 8e
    del reglamento describe como "las señales se acercan a la pared
    interior"; el programa oficial no corre ninguna.
  - El reglamento 2026 pone el cajón en la recta de salida. Las zonas de
    salida de obstáculos son Z3 y Z4 (entre las dos filas), cada una con
    asientos que no pueden estar ocupados enfrente del carro según el
    sentido (forbidden_intersections_in_start_zone).

Cintas, figura 11 del reglamento 2026 (no están en las rectas):
  - Dos por esquina, 20 mm, salen de la esquina de la isla.
  - En el noreste, azul a 30° del este y naranja a 60°. Las otras
    tres esquinas son ese mismo dibujo girado 90°.
  - Terminan 50 mm antes de la pared exterior (la cota de esa figura).
  - En sentido horario se cruza primero la naranja y después la azul; en
    antihorario al revés (open.py de ANTi decide el sentido así).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from . import config as _C

# Tapete, origen en el centro. +x este, +y norte. Milímetros.
MAT_MM = 3000.0
OUTER_HALF_MM = MAT_MM / 2.0          # 1500, borde interior de la pared exterior
CORRIDOR_MM = 1000.0                  # reto de obstáculos
INNER_HALF_MM = OUTER_HALF_MM - CORRIDOR_MM  # 500

SIGN_MM = 50.0                        # 13.19
LINE_MM = 20.0                        # 13.9
PARK_BARRIER_THICK_MM = 20.0
PARK_BARRIER_INTO_MM = 200.0          # los 20 cm de la madera, hacia el pasillo
PARK_GAP_MM = 270.0                   # 1.5 × 180 mm (el programa oficial dibuja 300)

# Coordenadas de una recta norte, como el programa oficial:
# (h, w) con h = mm desde el borde exterior y w = mm desde el oeste del tapete.
_FIRST = 400.0
_SECOND = 600.0
_LEFT = 1000.0
_MID = 1500.0
_RIGHT = 2000.0

SEATS = {
    "T4": (_FIRST, _LEFT),
    "X2": (_FIRST, _MID),
    "T3": (_FIRST, _RIGHT),
    "T2": (_SECOND, _LEFT),
    "X1": (_SECOND, _MID),
    "T1": (_SECOND, _RIGHT),
}

# 36 cartas del reglamento. Los duplicados (14/16, 15/17, …) están a propósito.
# Cada carta es una lista de (asiento, color).
_G, _R = "Green", "Red"
CARDS: list[list[tuple[str, str]]] = [
    [("T1", _G)],                          # 1
    [("T1", _R)],                          # 2
    [("X1", _G)],                          # 3
    [("X1", _R)],                          # 4
    [("T2", _G)],                          # 5
    [("T2", _R)],                          # 6
    [("T3", _G)],                          # 7
    [("T3", _R)],                          # 8
    [("X2", _G)],                          # 9  — señal única verde
    [("X2", _R)],                          # 10 — señal única roja
    [("T4", _G)],                          # 11
    [("T4", _R)],                          # 12
    [("T3", _G), ("T2", _G)],              # 13
    [("T3", _G), ("T2", _R)],              # 14
    [("T3", _R), ("T2", _G)],              # 15
    [("T3", _G), ("T2", _R)],              # 16 duplicada
    [("T3", _R), ("T2", _G)],              # 17 duplicada
    [("T3", _R), ("T2", _R)],              # 18
    [("T1", _G), ("T4", _G)],              # 19
    [("T1", _G), ("T4", _R)],              # 20
    [("T1", _R), ("T4", _G)],              # 21
    [("T1", _G), ("T4", _R)],              # 22 duplicada
    [("T1", _R), ("T4", _G)],              # 23 duplicada
    [("T1", _R), ("T4", _R)],              # 24
    [("T1", _G), ("T2", _G)],              # 25
    [("T1", _G), ("T2", _R)],              # 26
    [("T1", _R), ("T2", _G)],              # 27
    [("T1", _G), ("T2", _R)],              # 28 duplicada
    [("T1", _R), ("T2", _G)],              # 29 duplicada
    [("T1", _R), ("T2", _R)],              # 30
    [("T3", _G), ("T4", _G)],              # 31
    [("T3", _G), ("T4", _R)],              # 32
    [("T3", _R), ("T4", _G)],              # 33
    [("T3", _G), ("T4", _R)],              # 34 duplicada
    [("T3", _R), ("T4", _G)],              # 35 duplicada
    [("T3", _R), ("T4", _R)],              # 36
]
assert len(CARDS) == 36
CARD_SINGLE_GREEN = 9
CARD_SINGLE_RED = 10

# Orden horario alrededor de la isla.
SECTIONS_CW = ("N", "E", "S", "W")


def _square(half: float):
    return [
        (half, -half),
        (half, half),
        (-half, half),
        (-half, -half),
    ]


def section_to_world(section: str, h: float, w: float) -> tuple[float, float]:
    """(h, w) de la recta norte del programa oficial → mm del tapete."""
    if section == "N":
        return w - OUTER_HALF_MM, OUTER_HALF_MM - h
    if section == "S":
        return OUTER_HALF_MM - w, h - OUTER_HALF_MM
    if section == "E":
        return OUTER_HALF_MM - h, OUTER_HALF_MM - w
    if section == "W":
        return h - OUTER_HALF_MM, w - OUTER_HALF_MM
    raise ValueError(section)


@dataclass
class Sign:
    x: float
    y: float
    color: str          # "Red" | "Green"
    section: str
    seat: str


@dataclass
class Field:
    seed: int
    direction: str      # "CW" | "CCW"
    single_section: str
    parking_section: str
    cards: dict         # section -> número de carta (1..36)
    signs: list[Sign] = field(default_factory=list)
    barriers: list[list[tuple[float, float]]] = field(default_factory=list)
    start_x: float = 0.0
    start_y: float = 0.0
    start_heading: float = 0.0
    start_mode: str = "centro"   # "centro" | "cajon" | "Z3" | "Z4"

    @property
    def outer(self):
        return _square(OUTER_HALF_MM)

    @property
    def inner(self):
        return _square(INNER_HALF_MM)

    def summary(self) -> str:
        lines = [
            f"Obstacle Challenge  semilla {self.seed}  sentido {self.direction}",
            f"  señal única en {self.single_section} "
            f"(carta {self.cards[self.single_section]}, asiento X2)",
        ]
        for sec in SECTIONS_CW:
            seats = ", ".join(
                f"{s.seat} {s.color}" for s in self.signs if s.section == sec
            ) or "—"
            mark = "  [cajón]" if sec == self.parking_section else ""
            lines.append(f"  recta {sec}: carta {self.cards[sec]} → {seats}{mark}")
        lines.append(
            f"  arranque en recta {self.parking_section}, {self.start_mode}, "
            f"heading {self.start_heading:.0f}°"
        )
        return "\n".join(lines)


def _heading(section: str, direction: str) -> float:
    """Grados. 0 = +y, 90 = +x, igual que el twin. Horario = avanzar en +w."""
    cw = {
        "N": 90.0,
        "E": 180.0,
        "S": 270.0,
        "W": 0.0,
    }
    h = cw[section]
    return h if direction == "CW" else (h + 180.0) % 360.0


# Centro del chasis respecto al eje trasero (180 de largo, 33.5 de voladizo).
# Fuente única: config.py (56.5 con las medidas actuales).
_CAR_CENTER_AHEAD_OF_REAR_MM = _C.ROBOT_LENGTH_MM / 2.0 - _C.REAR_OVERHANG_MM

# Zonas de salida del programa oficial, (h0, w0, h1, w1) en la recta norte.
START_ZONES = {
    "Z3": (_FIRST, _MID, _SECOND, _RIGHT),
    "Z4": (_FIRST, _LEFT, _SECOND, _MID),
}
# Asientos que quedarían enfrente del carro en cada zona.
START_ZONE_FORBIDDEN = {
    "CW": {"Z3": ("T1", "T3"), "Z4": ("X1", "X2")},
    "CCW": {"Z3": ("X1", "X2"), "Z4": ("T2", "T4")},
}
PARKING_FORBIDDEN_SEATS = ("T3", "T4", "X2")
REQUIRED_CARDS = (22, 23, 28, 29)


def _rear_axle_at_center(section: str, direction: str, h: float, w: float) -> tuple[float, float]:
    """Eje trasero con el centro del chasis en (h, w), mirando en el sentido de carrera."""
    dw = _CAR_CENTER_AHEAD_OF_REAR_MM if direction == "CW" else -_CAR_CENTER_AHEAD_OF_REAR_MM
    return section_to_world(section, h, w - dw)


def _start_of(section: str) -> tuple[float, float]:
    return {
        "N": (0.0, 1000.0),
        "E": (1000.0, 0.0),
        "S": (0.0, -1000.0),
        "W": (-1000.0, 0.0),
    }[section]


def _barrier_corners(section: str) -> list[list[tuple[float, float]]]:
    """Dos maderas junto al radio izquierdo de la recta, desde la pared exterior."""
    gap = PARK_GAP_MM
    t = PARK_BARRIER_THICK_MM
    into = PARK_BARRIER_INTO_MM
    left = _LEFT
    boxes = [
        (0.0, left, into, left + t),
        (0.0, left + t + gap, into, left + t + gap + t),
    ]
    out = []
    for h0, w0, h1, w1 in boxes:
        corners = [
            section_to_world(section, h0, w0),
            section_to_world(section, h0, w1),
            section_to_world(section, h1, w1),
            section_to_world(section, h1, w0),
        ]
        out.append(corners)
    return out


# Figura 11. Ángulo 0 = este, 90 = norte. En el noreste el azul queda a 30°
# y el naranja a 60°; el resto sale de girar esa esquina. Yendo en sentido
# horario (al este por la recta norte) se cruza primero la naranja.
_CORNER_LINES = (
    (( INNER_HALF_MM,  INNER_HALF_MM),  30.0,  60.0),
    ((-INNER_HALF_MM,  INNER_HALF_MM), 120.0, 150.0),
    ((-INNER_HALF_MM, -INNER_HALF_MM), -150.0, -120.0),
    (( INNER_HALF_MM, -INNER_HALF_MM),  -60.0,  -30.0),
)
# La figura deja 50 mm entre la punta de la cinta y la pared exterior.
_LINE_END_GAP_MM = 50.0


def _ray_to_outer(x, y, ang_deg):
    inset = OUTER_HALF_MM - _LINE_END_GAP_MM
    ang = math.radians(ang_deg)
    dx, dy = math.cos(ang), math.sin(ang)
    hits = []
    if abs(dx) > 1e-9:
        for wall in (inset, -inset):
            t = (wall - x) / dx
            if t > 0 and abs(y + t * dy) <= inset + 1e-6:
                hits.append(t)
    if abs(dy) > 1e-9:
        for wall in (inset, -inset):
            t = (wall - y) / dy
            if t > 0 and abs(x + t * dx) <= inset + 1e-6:
                hits.append(t)
    t = min(hits)
    return ((x, y), (x + t * dx, y + t * dy))


def orange_segments():
    """Cinta naranja de cada esquina. Figura 11 del reglamento 2026."""
    return [_ray_to_outer(xy[0], xy[1], ang_o) for xy, _ang_b, ang_o in _CORNER_LINES]


def blue_segments():
    """Cinta azul de cada esquina. Figura 11 del reglamento 2026."""
    return [_ray_to_outer(xy[0], xy[1], ang_b) for xy, ang_b, _ang_o in _CORNER_LINES]


def _card_seats(number: int) -> list[str]:
    return [seat for seat, _color in CARDS[number - 1]]


def _valid_zones(number: int, direction: str) -> list[str]:
    seats = set(_card_seats(number))
    return [z for z, bad in START_ZONE_FORBIDDEN[direction].items() if not seats & set(bad)]


def randomize(seed: int | None = None, start: str = "centro") -> Field:
    """Sorteo como randomize_and_draw_layout_for_obstacle del programa oficial.

    start: "centro" (eje trasero en el centro de la recta del cajón, sobre la
    línea media), "cajon" (estacionado dentro del cajón, paralelo a la pared
    exterior), "Z3" / "Z4" (centro del chasis en esa zona) o "zona" (una de
    las dos que el sorteo permite, al azar).
    """
    rng = random.Random(seed)
    used_seed = seed if seed is not None else rng.randrange(1, 10_000_000)
    if seed is None:
        # El randrange ya movió el Random; se regenera con la semilla guardada
        # para que --seed N reproduzca el mismo sorteo.
        return randomize(used_seed, start=start)

    while True:
        direction = rng.choice(("CW", "CCW"))
        single_color = rng.choice((_G, _R))
        single_card = CARD_SINGLE_GREEN if single_color == _G else CARD_SINGLE_RED
        required = rng.choice(REQUIRED_CARDS)
        pool = [i for i in range(1, 37) if i not in (single_card, required)]
        o1 = rng.choice(pool)
        o2 = rng.choice([i for i in pool if i != o1])
        numbers = [single_card, required, o1, o2]

        colors = [c for n in numbers for _s, c in CARDS[n - 1]]
        greens, reds = colors.count(_G), colors.count(_R)
        if abs(greens - reds) > 1 or len(colors) < 5:
            continue
        if not any(_valid_zones(n, direction) for n in numbers):
            continue
        sections = list(SECTIONS_CW)
        rng.shuffle(sections)
        cards = dict(zip(sections, numbers))
        # 2026: el cajón va en la recta de salida.
        parking_ok = [
            sec for sec, n in cards.items()
            if not set(_card_seats(n)) & set(PARKING_FORBIDDEN_SEATS)
        ]
        if start in ("zona", "Z3", "Z4"):
            parking_ok = [
                sec for sec in parking_ok
                if (start == "zona" and _valid_zones(cards[sec], direction))
                or start in _valid_zones(cards[sec], direction)
            ]
        if parking_ok:
            break

    parking = rng.choice(parking_ok)
    single = next(sec for sec, n in cards.items() if n == single_card)

    signs: list[Sign] = []
    for sec, number in cards.items():
        for seat, color in CARDS[number - 1]:
            h, w = SEATS[seat]
            x, y = section_to_world(sec, h, w)
            signs.append(Sign(x, y, color, sec, seat))

    heading = _heading(parking, direction)
    if start == "centro":
        sx, sy = _start_of(parking)
        mode = "centro"
    elif start == "cajon":
        # Hueco: 200 mm de fondo, 270 mm de largo (1.5 × 180). La cola queda a
        # 5 mm de la madera de atrás. El costado va a 12 mm de la pared: el
        # voladizo corto (33.5 mm) cabe en el swing; a 5 mm la cola roza.
        half_len = 90.0
        half_w = 65.0
        h_center = half_w + 12.0
        rear_face = _LEFT + PARK_BARRIER_THICK_MM
        front_face = rear_face + PARK_GAP_MM
        if direction == "CW":
            w_center = (rear_face + 5.0) + half_len
        else:
            w_center = (front_face - 5.0) - half_len
        sx, sy = _rear_axle_at_center(parking, direction, h_center, w_center)
        mode = "cajon"
    elif start in ("zona", "Z3", "Z4"):
        zone = start if start != "zona" else rng.choice(_valid_zones(cards[parking], direction))
        h0, w0, h1, w1 = START_ZONES[zone]
        sx, sy = _rear_axle_at_center(parking, direction, (h0 + h1) / 2.0, (w0 + w1) / 2.0)
        mode = zone
    else:
        raise ValueError(f"start desconocido: {start}")

    return Field(
        seed=used_seed,
        direction=direction,
        single_section=single,
        parking_section=parking,
        cards=cards,
        signs=signs,
        barriers=_barrier_corners(parking),
        start_x=sx,
        start_y=sy,
        start_heading=heading,
        start_mode=mode,
    )
