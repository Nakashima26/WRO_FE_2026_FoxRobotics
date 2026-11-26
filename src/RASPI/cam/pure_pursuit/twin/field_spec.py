"""Campo del twin desde un acomodo real (TWIN_FIELD_JSON): {"<semilla>": {"dir":"CCW","park":"W","w":1000,"sig":["S/T2G","E/T3R"]}}.
El acomodo sale de los eventos [ROUTE] plan#N del log de la Pi (la creencia final del mapa, no verdad medida)."""
from __future__ import annotations

import dataclasses
import json
import os

from pure_pursuit import wro_field as F


def field_from_spec(spec: dict, seed: int, start: str = "cajon") -> F.Field:
    base = F.randomize(seed, start=start, park_pos=spec["w"])
    d, sec = spec["dir"], spec["park"]
    signs = []
    for s in spec["sig"]:
        place, color = s[:-1], {"R": "Red", "G": "Green"}[s[-1]]
        sc, seat = place.split("/")
        h, w = F.SEATS[seat]
        x, y = F.section_to_world(sc, h, w)
        signs.append(F.Sign(x, y, color, sc, seat))
    sx, sy = F.stall_start_xy(sec, d, float(spec["w"]))
    return dataclasses.replace(
        base, direction=d, parking_section=sec, signs=signs, barriers=F._barrier_corners(sec, float(spec["w"])),
        start_x=sx, start_y=sy, start_heading=F._heading(sec, d), start_mode=start, parking_w=float(spec["w"]))


def field_from_env(seed: int, start: str):
    p = os.environ.get("TWIN_FIELD_JSON")
    if not p:
        return None
    spec = json.load(open(p, encoding="utf-8")).get(str(seed))
    return field_from_spec(spec, seed, start) if spec else None
