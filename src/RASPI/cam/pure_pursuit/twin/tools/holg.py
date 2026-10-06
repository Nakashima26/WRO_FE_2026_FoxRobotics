"""Holgura casco-lata (mm) de una corrida de carrera, para tools/ens.py (T16ens).

Misma definición que summ3.py de T16gr (holg_todas / holg_1a), recalculada desde
trace.csv + field.txt, así que sirve para cualquier corrida (también las de fox_base):
  - pasadas = cruces del plano de cada señal hacia adelante con tc < 12 (las de la
    fase de estacionamiento no cuentan);
  - valor por pasada = holgura mínima chasis-señal (menos el radio de la lata) en
    ±PASS_WINDOW_MM del cruce; lado equivocado = -1; choque con una señal sin cruce
    registrado = 0;
  - holg_todas = todas las pasadas; holg_1a = la primera lata de cada recta tras la
    esquina (clave tc + recta).
Usa metrics.sign_passes si existe (T16gr fusionado); si no, la copia de abajo
(idéntica a la de T16gr al 2026-10-05).
"""
from __future__ import annotations

import csv
import math
import re
from types import SimpleNamespace
from typing import Any

from pure_pursuit import config as _C
from pure_pursuit.wro_field import OUTER_HALF_MM, SIGN_MM, Sign
from pure_pursuit.twin.metrics import _parse_ack_field

VEH = SimpleNamespace(length_mm=_C.ROBOT_LENGTH_MM, width_mm=_C.ROBOT_WIDTH_MM,
                      rear_overhang_mm=_C.REAR_OVERHANG_MM)
RX_SIGN = re.compile(r'^(Red|Green) ([NESW])/(\w+) \((-?\d+),(-?\d+)\)')
PASS_WINDOW_MM = 300.0


def _sec_frame(section: str, x: float, y: float) -> tuple[float, float]:
    """(h, w) en la recta: h = mm desde la pared exterior, w = mm a lo largo en CW."""
    if section == "N":
        return OUTER_HALF_MM - y, x + OUTER_HALF_MM
    if section == "S":
        return y + OUTER_HALF_MM, OUTER_HALF_MM - x
    if section == "E":
        return OUTER_HALF_MM - x, OUTER_HALF_MM - y
    if section == "W":
        return x + OUTER_HALF_MM, y + OUTER_HALF_MM
    raise ValueError(section)


def _body_to_point_mm(x, y, heading_deg, px, py, length_mm, width_mm, rear_overhang_mm) -> float:
    h = math.radians(heading_deg)
    fx, fy = math.sin(h), math.cos(h)
    rx, ry = math.cos(h), -math.sin(h)
    dx, dy = px - x, py - y
    a = dx * fx + dy * fy
    b = dx * rx + dy * ry
    y0, y1 = -rear_overhang_mm, length_mm - rear_overhang_mm
    ea = max(y0 - a, 0.0, a - y1)
    eb = max(abs(b) - width_mm / 2.0, 0.0)
    return math.hypot(ea, eb)


def _sign_passes_local(trace, signs, direction: str, vehicle_params) -> list[dict[str, Any]]:
    L, W, ro = vehicle_params.length_mm, vehicle_params.width_mm, vehicle_params.rear_overhang_mm
    c_ahead = L / 2.0 - ro
    rad = SIGN_MM / 2.0
    race_sg = 1.0 if direction == "CW" else -1.0
    out: list[dict[str, Any]] = []
    n = len(trace)
    if n < 2:
        return out
    cen, tcs, tc = [], [], None
    for row in trace:
        v = _parse_ack_field(row.get("ack") or "", "tc")
        if v is not None:
            try:
                tc = int(v)
            except ValueError:
                pass
        tcs.append(tc)
        hh = math.radians(row["heading"])
        cen.append((row["x"] + c_ahead * math.sin(hh), row["y"] + c_ahead * math.cos(hh)))
    for sign in signs:
        hs, ws = _sec_frame(sign.section, sign.x, sign.y)
        hw = [_sec_frame(sign.section, cx, cy) for cx, cy in cen]
        for i in range(1, n):
            h0, w0 = hw[i - 1]
            h1, w1 = hw[i]
            d0, d1 = w0 - ws, w1 - ws
            if (d0 < 0.0) == (d1 < 0.0) or d0 == d1:
                continue
            if not (0.0 <= h1 <= 1000.0 and 0.0 <= h0 <= 1000.0):
                continue
            sg = 1.0 if w1 > w0 else -1.0
            f = d0 / (d0 - d1)
            s = (h0 + f * (h1 - h0) - hs) * sg
            side = "R" if s > 0 else "L"
            want = "R" if sign.color == "Red" else "L"
            clear = float("inf")
            j = i - 1
            while j >= 0 and abs(hw[j][1] - ws) < PASS_WINDOW_MM:
                r = trace[j]
                clear = min(clear, _body_to_point_mm(r["x"], r["y"], r["heading"], sign.x, sign.y, L, W, ro))
                j -= 1
            j = i
            while j < n and abs(hw[j][1] - ws) < PASS_WINDOW_MM:
                r = trace[j]
                clear = min(clear, _body_to_point_mm(r["x"], r["y"], r["heading"], sign.x, sign.y, L, W, ro))
                j += 1
            out.append({
                "sign": f"{sign.color} {sign.section}/{sign.seat}",
                "t": round(float(trace[i]["t"]), 3), "side": side, "ok": side == want,
                "offset_mm": round(s, 1),
                "clear_mm": round(clear - rad, 1) if math.isfinite(clear) else None,
                "reverse": sg != race_sg, "tc": tcs[i],
            })
    out.sort(key=lambda e: e["t"])
    return out


try:  # T16gr la mueve a metrics.py; si está, manda esa.
    from pure_pursuit.twin.metrics import sign_passes  # type: ignore[attr-defined]
except ImportError:
    sign_passes = _sign_passes_local


def run_holg(sd: str, row: dict | None) -> tuple[list[float], list[float]] | None:
    """(holg_todas, holg_1a) de la corrida en `sd`; `row` = summ3.run_row(sd).
    None si faltan trace.csv o field.txt."""
    try:
        with open(f'{sd}/field.txt', encoding='utf-8') as f:
            lines = f.read().splitlines()
        with open(f'{sd}/trace.csv', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return None
    if not lines or 'dir=' not in lines[0]:
        return None
    dr = lines[0].split('dir=')[1].split()[0]
    signs = []
    for ln in lines[1:]:
        m = RX_SIGN.match(ln)
        if m:
            signs.append(Sign(float(m.group(4)), float(m.group(5)), m.group(1), m.group(2), m.group(3)))
    tr = [{'t': float(r['t']), 'x': float(r['x']), 'y': float(r['y']), 'heading': float(r['heading']),
           'ack': r.get('ack', '')} for r in rows]
    ps = sign_passes(tr, signs, dr, VEH)
    fw = [p for p in ps if (p.get('tc') or 0) < 12 and not p['reverse']]
    h_all: list[float] = []
    h_1a: list[float] = []
    seen = set()
    for p in fw:
        v = -1.0 if not p['ok'] else (p['clear_mm'] if p['clear_mm'] is not None else 0.0)
        h_all.append(v)
        k = (p.get('tc'), p['sign'].split()[1].split('/')[0])
        if k not in seen:
            seen.add(k)
            h_1a.append(v)
    col = (row or {}).get('col') or []
    cz = (row or {}).get('cz') or ''
    if cz.startswith('señal ') and col and isinstance(col[0], dict):
        sg = cz[len('señal '):].split('@')[0]
        ct = float(col[0].get('t') or 0.0)
        if not any(p['sign'] == sg and abs(p['t'] - ct) < 1.5 for p in fw):
            h_all.append(0.0)
            if ((row or {}).get('m', {}).get('turns_completed'), sg.split()[1].split('/')[0]) not in seen:
                h_1a.append(0.0)
    return h_all, h_1a


def dist(v: list[float]) -> dict[str, float] | None:
    """Percentiles como summ3 de T16gr (índice redondeado, sin interpolar)."""
    v = sorted(v)
    if not v:
        return None
    q = lambda f: v[min(len(v) - 1, int(f * (len(v) - 1) + 0.5))]  # noqa: E731
    return {"n": len(v), "min": v[0], "p10": q(.10), "p25": q(.25), "med": q(.5),
            "le0": sum(x <= 0 for x in v), "lt25": sum(x < 25 for x in v), "lt50": sum(x < 50 for x in v)}


def fmt_dist(name: str, v: list[float]) -> str:
    d = dist(v)
    if d is None:
        return f'{name}: n=0'
    return (f"{name}: n={d['n']} min={d['min']:.0f} p10={d['p10']:.0f} p25={d['p25']:.0f} med={d['med']:.0f} "
            f"<=0={d['le0']} <25={d['lt25']} <50={d['lt50']}")
