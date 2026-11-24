"""Calibración del lugar (sede): un JSON fuera del repo que el runtime aplica encima de carrera.json.

Ruta: $FOX_CALIB o ~/wro_calib.json. Lo escribe el dashboard (botones de calibración); a mano
también vale. Un archivo roto o con claves raras nunca tumba la carrera: se avisa y se ignora.

{
  "camera": {"gains": [1.10, 1.37], "ExposureTime": 12000, "AnalogueGain": 2.0},
  "colors": {"Red": [[[0,110,40],[5,255,200]], [[168,110,40],[179,255,235]]], "Green": [[[30,35,25],[85,255,255]]]},
  "config": {"COLOR_CORR_FLOOR_REF_BGR": [175,198,213], "ROUTE_FIX_BLUE_HSV": [[95,70,60],[115,255,200]]},
  "tof": {"L": [95,193,292,380,481,559,626,669], "R": [...]}   (solo registro: la tabla vive en el ESP/NVS)
  "_hist": [...]   (versiones anteriores, las escribe el dashboard)
}
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np

# Claves de config que el dashboard puede cambiar (las demás se ignoran: esto no es otro carrera.json).
CONFIG_KEYS = {
    "COLOR_CORR_FLOOR_REF_BGR", "ROUTE_FIX_BLUE_HSV", "LINE_ORANGE_HSV", "LINE_CORE_HSV",
    "PARK_PINK_HSV",
}


def path() -> Path:
    return Path(os.environ.get("FOX_CALIB") or (Path.home() / "wro_calib.json"))


def load() -> dict:
    p = path()
    if not p.is_file():
        return {}
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
        return d if isinstance(d, dict) else {}
    except Exception as e:
        print(f"[CALIB] AVISO: {p} no se pudo leer ({e}); se ignora", flush=True)
        return {}


def save(d: dict, keep: int = 10) -> Path:
    """Guarda con historial: la versión anterior (sin su propio _hist) va a _hist[0]."""
    p = path()
    old = load()
    hist = list(old.pop("_hist", []))
    if old:
        hist.insert(0, old)
    d = dict(d)
    d.pop("_hist", None)
    d["_hist"] = hist[:keep]
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(d, indent=1, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, p)
    return p


def _coerce(old, new):
    """Lleva `new` (listas del JSON) a la forma de `old` (listas de (np.array, np.array), tuplas...)."""
    if isinstance(old, list) and old and isinstance(old[0], tuple) and isinstance(old[0][0], np.ndarray):
        return [(np.array(lo, dtype=old[0][0].dtype), np.array(hi, dtype=old[0][0].dtype)) for lo, hi in new]
    if isinstance(old, tuple) and old and isinstance(old[0], tuple):
        return tuple(tuple(v) for v in new)
    if isinstance(old, tuple):
        return tuple(new)
    return new


def apply(C, vision_module=None, d: dict | None = None) -> int:
    """Aplica la calibración a config (C) y a vision. Devuelve cuántos valores aplicó."""
    d = load() if d is None else d
    n = 0
    for k, v in (d.get("config") or {}).items():
        if k not in CONFIG_KEYS or not hasattr(C, k):
            continue
        try:
            setattr(C, k, _coerce(getattr(C, k), v))
            n += 1
        except Exception as e:
            print(f"[CALIB] {k} no aplicado ({e})", flush=True)
    if vision_module is not None:
        cols = d.get("colors") or {}
        for name, ranges in cols.items():
            if name in vision_module.COLOR_RANGES:
                try:
                    vision_module.COLOR_RANGES[name] = [(np.array(lo), np.array(hi)) for lo, hi in ranges]
                    n += 1
                except Exception as e:
                    print(f"[CALIB] color {name} no aplicado ({e})", flush=True)
        cam = d.get("camera") or {}
        g = cam.get("gains")
        if g and len(g) == 2:
            vision_module.CAM_GAINS_PI5 = (float(g[0]), float(g[1]))
            n += 1
        if cam.get("ExposureTime") and cam.get("AnalogueGain"):
            vision_module.EXPO_OVERRIDE = {"ExposureTime": int(cam["ExposureTime"]),
                                           "AnalogueGain": float(cam["AnalogueGain"])}
            n += 1
    if n:
        print(f"[CALIB] {n} valores de {path()}", flush=True)
    return n
