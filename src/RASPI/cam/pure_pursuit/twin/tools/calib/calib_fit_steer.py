"""Mapeo servo -> rueda equivalente (bicicleta) y ppServoGain recomendado.

Desde src/RASPI/cam (PYTHONUTF8=1 PYTHONPATH=.):
  python -m pure_pursuit.twin.tools.calib.calib_fit_steer steer.csv [--center 90]
         [--wheelbase 113] [--kingpin 60] [--out-dir DIR] [--apply] [--root RAIZ]

CSV de entrada (cabecera obligatoria; una fila por medicion; cualquiera de estas
formas de dar el giro, por fila):
  servo_deg, radius_mm        radio de la trayectoria del CENTRO DEL EJE TRASERO
  servo_deg, diam_mm          diametro de ese mismo circulo
  servo_deg, inner_deg [,outer_deg]  angulo medido con transportador de la rueda
                              interior (y exterior) -> se convierte a equivalente
  servo_deg, v_mm_s, yaw_dps  velocidad y velocidad de giro (R = v/omega)
  opcionales: rep, note
Convenciones (las del firmware/twin): servo > centro = IZQUIERDA; rueda + = derecha.
Modelo: bicicleta con eje trasero de referencia, tan(delta_eq) = L / R_trasero.
Salida: steer_fit.json, steer_params.patch (twin/params.py: gain_left/right en
grados de rueda por 90 grados de servo) y steer_ino_ppservogain.patch (SOLO consejo).

ppServoGain (PurePursuit.ino l.212) = grados de servo por grado de rueda que pide la Pi:
  servo = centro - steer_deg*ppServoGain  ->  rueda = g*steer_deg*ppServoGain
  rueda == steer_deg  <=>  ppServoGain = 1/g   (g = grados de rueda por grado de servo)
El control de la Pi se afino con ppServoGain=1 (rueda = g*steer); cambiarlo re-escala TODO
el control. Por eso el patch del .ino es opcional y va aparte; primero actualiza el twin.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
from pathlib import Path

import numpy as np

try:
    from . import calib_common as cc
except ImportError:  # script
    import calib_common as cc  # type: ignore


def delta_eq_from_radius(R_mm: float, L: float) -> float:
    return math.degrees(math.atan2(L, R_mm))


def delta_eq_from_inner(inner_deg: float, L: float, T: float) -> float:
    """Igual que config.MAX_WHEEL_STEER_DEG: R = L/tan(int) + T/2 ; delta_eq = atan(L/R)."""
    R = L / math.tan(math.radians(inner_deg)) + T / 2.0
    return math.degrees(math.atan2(L, R))


def row_to_delta(row: dict, L: float, T: float) -> tuple[float, str] | None:
    def g(k):
        v = row.get(k)
        return float(v) if v not in (None, "") else None

    R = g("radius_mm")
    if R is None and g("diam_mm") is not None:
        R = g("diam_mm") / 2.0
    if R is not None and R > 0:
        return delta_eq_from_radius(R, L), "radio"
    if g("inner_deg") is not None:
        return delta_eq_from_inner(abs(g("inner_deg")), L, T), "transportador"
    v, w = g("v_mm_s"), g("yaw_dps")
    if v and w:
        return delta_eq_from_radius(abs(v) / math.radians(abs(w)), L), "yaw"
    return None


def load_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        return [dict(r) for r in csv.DictReader(f)]


def fit_steer(rows: list[dict], center: float = 90.0, L: float = 113.0, T: float = 60.0) -> dict:
    """Devuelve ganancias por lado, linealidad, tope y ppServoGain recomendado."""
    by_servo: dict[float, list[float]] = {}
    srcs = set()
    for r in rows:
        s = float(r["servo_deg"])
        d = row_to_delta(r, L, T)
        if d is None:
            continue
        by_servo.setdefault(s, []).append(math.tan(math.radians(d[0])))   # promediar curvatura (~1/R)
        srcs.add(d[1])
    if not by_servo:
        raise ValueError("ninguna fila tiene radio/diametro/angulo/yaw usable")
    table = []
    for s in sorted(by_servo):
        d = math.degrees(math.atan(float(np.mean(by_servo[s]))))
        x = s - center
        table.append({"servo_deg": s, "x": x, "side": "L" if x > 0 else "R" if x < 0 else "C",
                      "delta_eq_deg": d, "n": len(by_servo[s]),
                      "gain_deg_per_deg": d / abs(x) if abs(x) > 1e-9 else None,
                      "radius_mm": L / math.tan(math.radians(d)) if d > 1e-6 else None})
    res: dict = {"wheelbase_mm": L, "kingpin_mm": T, "center_deg": center, "sources": sorted(srcs), "table": table}
    warn: list[str] = []

    def side_fit(side: str):
        pts = [(abs(t["x"]), t["delta_eq_deg"]) for t in table if t["side"] == side and abs(t["x"]) > 1e-9]
        if len(pts) < 2:
            return None
        x = np.array([p[0] for p in pts])
        y = np.array([p[1] for p in pts])
        # saturacion: si el ultimo punto ya no crece, el tope mecanico esta antes
        sat_x = None
        if len(pts) >= 3 and y[-1] < y[-2] * 1.02 and x[-1] > x[-2]:
            sat_x = float(x[-2])
            x, y = x[:-1], y[:-1]
        g0 = float((x * y).sum() / (x * x).sum())            # por el origen
        a, b = np.polyfit(x, y, 1)                           # con intercepto (centro mal puesto)
        res0 = y - g0 * x
        return {"gain_origin": g0, "gain_affine": float(a), "intercept_deg": float(b),
                "rms_deg": float(np.sqrt((res0 ** 2).mean())), "max_abs_resid_deg": float(np.abs(res0).max()),
                "n": int(x.size), "saturates_at_servo_offset": sat_x,
                "delta_max_deg": float(y.max()), "x_max": float(x.max())}

    res["left"] = side_fit("L")
    res["right"] = side_fit("R")
    sides = [s for s in (res["left"], res["right"]) if s]
    if not sides:
        raise ValueError("hacen falta >= 2 angulos distintos de servo por lado")
    gl = res["left"]["gain_origin"] if res["left"] else res["right"]["gain_origin"]
    gr = res["right"]["gain_origin"] if res["right"] else res["left"]["gain_origin"]
    res["gain_left_deg_per_deg"], res["gain_right_deg_per_deg"] = gl, gr
    res["gain_left_per90"], res["gain_right_per90"] = gl * 90.0, gr * 90.0
    gm = 0.5 * (gl + gr)
    res["gain_mean"] = gm
    res["ppServoGain_for_wheel_eq_steer"] = 1.0 / gm
    res["ppServoGain_current"] = 1.0
    res["wheel_eff_current_deg_per_steer_deg"] = gm * 1.0
    dm = max(s["delta_max_deg"] for s in sides)
    res["wheel_max_deg"] = dm
    res["min_turn_radius_mm"] = L / math.tan(math.radians(dm))
    if res["left"] and res["right"]:
        if abs(gl - gr) / gm > 0.08:
            warn.append(f"asimetria izq/der {100 * abs(gl - gr) / gm:.0f} %: el twin ya tiene gain_left/right separados")
        for k in ("left", "right"):
            if abs(res[k]["intercept_deg"]) > 1.5:
                warn.append(f"{k}: intercepto {res[k]['intercept_deg']:.1f} deg -> el centro del servo no es {center:g}; "
                            "ajusta centroServo o mide el centro (carro recto)")
    for k in ("left", "right"):
        if res[k] and res[k]["max_abs_resid_deg"] > 1.5:
            warn.append(f"{k}: no lineal (residuo max {res[k]['max_abs_resid_deg']:.1f} deg); la cremallera es no lineal, "
                        "el twin usa una ganancia: considera ajustar solo el rango usado por la Pi (+-35)")
        if res[k] and res[k]["saturates_at_servo_offset"]:
            warn.append(f"{k}: la rueda deja de girar mas alla de ~{res[k]['saturates_at_servo_offset']:.0f} deg de servo "
                        "(tope mecanico): baja los topes del .ino 30/160 a ese valor")
    if len(table) < 5:
        warn.append("menos de 5 angulos medidos: la linealidad no se puede juzgar")
    res["warnings"] = warn
    return res


def build_patches(res: dict, root: Path) -> dict:
    f = cc.SrcFile.read(root, cc.PARAMS_REL)
    t = f.text
    stamp = dt.date.today().isoformat()
    t = cc.replace_field_default(t, "gain_left", cc.fmt_num(round(res["gain_left_per90"], 3)))
    t = cc.replace_field_default(t, "gain_right", cc.fmt_num(round(res["gain_right_per90"], 3)))
    t = cc.replace_prov(t, "steering.gain_left/right", "medido",
                        f"medido {stamp} ({'+'.join(res['sources'])}): {res['gain_left_deg_per_deg']:.3f}/"
                        f"{res['gain_right_deg_per_deg']:.3f} deg rueda eq. por deg servo (izq/der); "
                        f"rueda max {res['wheel_max_deg']:.1f} deg, R min {res['min_turn_radius_mm']:.0f} mm", f.eol)
    out = {cc.PARAMS_REL: (cc.unified_diff(cc.PARAMS_REL, f.text, t), t, f)}
    g = cc.SrcFile.read(root, cc.INO_REL)
    import re
    pat = re.compile(r"^(float\s+ppServoGain\s*=\s*)([-+0-9.eE]+)(\s*;)", re.M)
    m = pat.search(g.text)
    if not m:
        raise ValueError("no encuentro `float ppServoGain = ...;` en el .ino")
    u = g.text[:m.start()] + f"{m.group(1)}{res['ppServoGain_for_wheel_eq_steer']:.3f}{m.group(3)}" + g.text[m.end():]
    out[cc.INO_REL] = (cc.unified_diff(cc.INO_REL, g.text, u), u, g)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--center", type=float, default=90.0)
    ap.add_argument("--wheelbase", type=float, default=113.0)
    ap.add_argument("--kingpin", type=float, default=60.0)
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--root", type=Path, default=None)
    ap.add_argument("--apply", action="store_true", help="aplica SOLO el parche de params.py (el del .ino nunca)")
    a = ap.parse_args(argv)

    res = fit_steer(load_rows(a.csv), a.center, a.wheelbase, a.kingpin)
    out = a.out_dir or a.csv.parent
    root = a.root or cc.repo_root()
    (out / "steer_fit.json").write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
    print("servo  delta_eq  ganancia  R_mm")
    for t in res["table"]:
        gg = f"{t['gain_deg_per_deg']:.3f}" if t["gain_deg_per_deg"] else "  -  "
        rr = f"{t['radius_mm']:.0f}" if t["radius_mm"] else "-"
        print(f"{t['servo_deg']:5.0f}  {t['delta_eq_deg']:7.2f}  {gg:>8}  {rr:>6}")
    print(f"ganancia izq={res['gain_left_deg_per_deg']:.3f} der={res['gain_right_deg_per_deg']:.3f} deg/deg "
          f"(CAD supuesto: 0.627); rueda max={res['wheel_max_deg']:.1f} deg; R min={res['min_turn_radius_mm']:.0f} mm")
    print(f"ppServoGain para rueda = steer_deg: {res['ppServoGain_for_wheel_eq_steer']:.3f} (hoy 1.0 -> rueda = "
          f"{res['wheel_eff_current_deg_per_steer_deg']:.2f} x steer_deg)")
    for w in res["warnings"]:
        print("AVISO:", w)
    patches = build_patches(res, root)
    names = {cc.PARAMS_REL: "steer_params.patch", cc.INO_REL: "steer_ino_ppservogain.patch"}
    for rel, (diff, new, f) in patches.items():
        (out / names[rel]).write_text(diff, encoding="utf-8", newline="")
        print(f"parche -> {out / names[rel]}")
        if a.apply and rel == cc.PARAMS_REL and diff:
            cc.write_src(root, f, new)
            print(f"  APLICADO a {root / rel}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
