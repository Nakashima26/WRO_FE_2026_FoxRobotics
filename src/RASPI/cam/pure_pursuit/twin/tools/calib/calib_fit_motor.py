"""Ajusta MotorParams/EncoderParams del twin con los CSV de calib_capture.py.

Desde src/RASPI/cam (PYTHONUTF8=1 PYTHONPATH=.):
  python -m pure_pursuit.twin.tools.calib.calib_fit_motor runs/calib/d1 [--steer-json steer.json]
                                                          [--apply] [--root RAIZ_REPO]

Entradas (nombres que pone calib_capture.py; basta lo que tengas):
  enc_runs.csv           -> counts_per_mm   (HACER PRIMERO: escala todas las velocidades)
  ramp_*.csv, dead_*.csv -> k_mm_s_per_pwm, pwm0, k_rev (escalera en estado estable)
  step_p*, stepn_p*      -> tau_drive_s (escalon desde reposo; ajuste en distancia)
  coast_p*, brake_p*     -> tau_coast_s y distancia que rueda tras cortar
  lock_p*_s*             -> scrub_coeff (velocidad con volante fijo vs recta)
Salida en la carpeta: calib_motor.json, calib_params.patch (twin/params.py),
calib_ino_consts.patch (ODOM_* y ENC_CUENTAS_POR_MM de PurePursuit.ino; deben quedar
iguales al twin: paridad.py los compara). Los parches NO se aplican salvo --apply.
config.py no tiene constantes de motor/encoder: no necesita parche.

Modelo (twin/vehicle.py + params.steady_speed_mm_s):
  v_ss = k*(pwm - pwm0) [* (1 - scrub*(rueda/rueda_max)^2)];  dv/dt = (v_ss - v)/tau
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

try:
    from . import calib_common as cc
except ImportError:  # script
    import calib_common as cc  # type: ignore

NOMINAL_CPMM = 20.73   # PurePursuit.ino l.434 (7 PPR x4 x50 x2 / (pi*43)); ver nota de CPR en la hoja


# ── encoder ───────────────────────────────────────────────────────────────────
def fit_counts_per_mm(path: Path) -> dict | None:
    if not path.exists():
        return None
    rows = np.genfromtxt(path, delimiter=",", names=True, dtype=float, encoding="utf-8")
    rows = np.atleast_1d(rows)
    c = rows["counts_fwd"]
    d = rows["dist_mm"]
    cpmm = np.abs(c) / d
    out = {
        "n": int(cpmm.size),
        "mean": float(cpmm.mean()),
        "std": float(cpmm.std(ddof=1)) if cpmm.size > 1 else 0.0,
        "sem": float(cpmm.std(ddof=1) / np.sqrt(cpmm.size)) if cpmm.size > 1 else 0.0,
        "values": [float(x) for x in cpmm],
    }
    names = rows.dtype.names or ()
    if "counts_net" in names:
        net = rows["counts_net"]
        net = net[np.isfinite(net)]
        if net.size:
            # tras ir y volver al mismo punto el neto debe ser ~0; lo que sobra = cuentas perdidas
            out["net_after_return_mean"] = float(np.mean(net))
            out["net_after_return_pct_of_fwd"] = float(100.0 * np.mean(np.abs(net)) / np.mean(np.abs(c)))
    if "enc_err" in names:
        out["enc_err_total"] = int(np.nansum(rows["enc_err"]))
    return out


# ── velocidad en estado estable ───────────────────────────────────────────────
def steady_points(runs: list[cc.Run], cpmm: float, min_s: float = 0.25, tail: float = 0.4) -> list[dict]:
    pts = []
    for r in runs:
        for i0, i1, pwm in cc.segments(r, min_s):
            if pwm == 0:
                continue
            v, ok = cc.window_speed(r, i0, i1, cpmm, tail)
            pts.append({"run": r.name, "pwm": pwm, "v": v, "stable": ok})
    return pts


def fit_speed_line(pwm: np.ndarray, v: np.ndarray) -> dict | None:
    """v = k*(pwm - pwm0) sobre los puntos por encima de la zona muerta."""
    x = np.abs(np.asarray(pwm, float))
    y = np.abs(np.asarray(v, float))
    if x.size < 4 or y.max() <= 0:
        return None
    vmax = y.max()
    sel = y > 0.15 * vmax
    a = b = 0.0
    for _ in range(6):
        if sel.sum() < 3:
            return None
        a, b = np.polyfit(x[sel], y[sel], 1)
        pred = a * x + b
        sel = (pred > 0.10 * vmax) & (y > 0.05 * vmax)
    pred = a * x[sel] + b
    res = y[sel] - pred
    ss_tot = float(((y[sel] - y[sel].mean()) ** 2).sum())
    r2 = 1.0 - float((res ** 2).sum()) / ss_tot if ss_tot > 0 else 1.0
    if a <= 0:
        return None
    started = x[y >= 5.0]
    return {
        "k": float(a), "pwm0": float(-b / a), "r2": r2,
        "rms_mm_s": float(np.sqrt((res ** 2).mean())),
        "max_resid_pct_vmax": float(100 * np.abs(res).max() / vmax),
        "n_points": int(sel.sum()),
        "pwm_start_5mm_s": float(started.min()) if started.size else None,
        "vmax_mm_s": float(vmax),
    }


# ── respuesta al escalon / coast (ajuste sobre la distancia) ──────────────────
def _fit_rise(t: np.ndarray, s: np.ndarray, td_grid, tau_grid) -> tuple[float, float, float, float]:
    best = None
    for td in td_grid:
        u = np.clip(t - td, 0.0, None)[:, None]
        tau = tau_grid[None, :]
        g = u - tau * (1.0 - np.exp(-u / tau))
        den = (g * g).sum(0)
        den = np.where(den > 0, den, np.inf)
        v = (s[:, None] * g).sum(0) / den
        sse = ((s[:, None] - v * g) ** 2).sum(0)
        j = int(np.argmin(sse))
        if best is None or sse[j] < best[0]:
            best = (float(sse[j]), float(tau_grid[j]), float(v[j]), float(td))
    return best  # type: ignore[return-value]


def fit_step(run: cc.Run, cpmm: float) -> dict | None:
    segs = [sg for sg in cc.segments(run, 0.4) if sg[2] != 0]
    if not segs or segs[0][0] > 3:
        return None
    i0, i1, pwm = segs[0]
    t = run.t_s[i0:i1 + 1] - run.t_s[i0]
    s = (run.counts[i0:i1 + 1] - run.counts[i0]) / cpmm
    s = np.abs(s)
    tau_grid = np.geomspace(0.01, 1.5, 300)
    out = {}
    for label, tds in (("td0", [0.0]), ("delay", np.arange(0.0, 0.205, 0.005))):
        _, tau, v, td = _fit_rise(t, s, tds, tau_grid)
        fine = np.linspace(tau * 0.9, tau * 1.1, 41)
        _, tau, v, td = _fit_rise(t, s, [td], fine)
        out[label] = {"tau": tau, "v_ss": v, "td": td}
    v_end, stable = cc.window_speed(run, i0, i1, cpmm, 0.3)
    out.update({"pwm": pwm, "duration_s": float(t[-1]), "v_end_mm_s": abs(v_end), "settled": stable,
                "seems_short": bool(t[-1] < 4 * out["td0"]["tau"])})
    return out


def fit_coast(run: cc.Run, cpmm: float) -> dict | None:
    segs = [sg for sg in cc.segments(run, 0.2) if sg[2] != 0]
    if not segs:
        return None
    i0, i1, pwm = segs[0]
    if i1 >= run.n - 5:
        return None
    t = run.t_s[i1:] - run.t_s[i1]
    s = np.abs(run.counts[i1:] - run.counts[i1]) / cpmm
    v0_meas, _ = cc.window_speed(run, i0, i1, cpmm, 0.25)
    best = None
    for tau in np.geomspace(0.01, 3.0, 400):
        h = tau * (1.0 - np.exp(-t / tau))
        den = float((h * h).sum())
        v0 = float((s * h).sum() / den)
        sse = float(((s - v0 * h) ** 2).sum())
        if best is None or sse < best[0]:
            best = (sse, float(tau), v0)
    _, tau, v0 = best  # type: ignore[misc]
    # velocidad residual al final del log: si sigue rodando, el log fue corto
    n_end = max(3, run.n // 20)
    v_final = abs((run.counts[-1] - run.counts[-n_end]) / cpmm / (run.t_s[-1] - run.t_s[-n_end]))
    return {"pwm": pwm, "tau": tau, "v0_fit": abs(v0), "v0_meas": abs(v0_meas),
            "stop_mm": float(s[-1]), "v_final_mm_s": float(v_final), "log_too_short": bool(v_final > 8.0)}


# ── direccion twin (para el scrub) ────────────────────────────────────────────
class _Steer:
    def __init__(self, gain_l: float, gain_r: float, c: float = 90.0, wmax: float = 44.54):
        self.gl, self.gr, self.c, self.wmax = gain_l, gain_r, c, wmax

    def wheel(self, servo: float) -> float:
        d = self.c - servo
        return d * (self.gr if d >= 0 else self.gl) / 90.0


def load_steer(steer_json: Path | None) -> _Steer:
    if steer_json and steer_json.exists():
        j = json.loads(steer_json.read_text(encoding="utf-8"))
        gl, gr = j["gain_left_per90"], j["gain_right_per90"]
        wm = j.get("wheel_max_deg", None)
        st = _Steer(gl, gr)
        st.wmax = wm or max(abs(st.wheel(30)), abs(st.wheel(160)))
        return st
    try:
        from pure_pursuit.twin.params import SteeringParams, max_wheel_deg, wheel_deg_from_servo
        sp = SteeringParams()
        st = _Steer(sp.gain_left, sp.gain_right)
        st.wmax = max_wheel_deg(sp)
        st.wheel = lambda servo, sp=sp: wheel_deg_from_servo(servo, sp)  # type: ignore[method-assign]
        return st
    except Exception:  # noqa: BLE001
        st = _Steer(56.46, 56.46)
        st.wmax = max(abs(st.wheel(30)), abs(st.wheel(160)))
        return st


def fit_scrub(lock_runs: list[cc.Run], cpmm: float, line: dict | None, steer: _Steer) -> dict | None:
    meas = []
    for r in lock_runs:
        segs = [sg for sg in cc.segments(r, 0.5) if sg[2] != 0]
        if not segs:
            continue
        i0, i1, pwm = segs[0]
        v, _ = cc.window_speed(r, i0, i1, cpmm, 0.5)
        meas.append({"run": r.name, "pwm": abs(pwm), "servo": float(np.median(r.servo[i0:i1 + 1])), "v": abs(v)})
    if not meas:
        return None
    straight = {}
    for m in meas:
        if abs(m["servo"] - 90.0) <= 2.0:
            straight.setdefault(m["pwm"], []).append(m["v"])
    est = []
    for m in meas:
        if abs(m["servo"] - 90.0) <= 2.0:
            continue
        w = abs(steer.wheel(m["servo"]))
        frac = w / steer.wmax
        if frac < 0.3:
            continue
        if m["pwm"] in straight:
            vref = float(np.mean(straight[m["pwm"]]))
        elif line:
            vref = line["k"] * (m["pwm"] - line["pwm0"])
        else:
            continue
        if vref <= 5:
            continue
        est.append({"run": m["run"], "servo": m["servo"], "ratio": m["v"] / vref,
                    "scrub": (1.0 - m["v"] / vref) / frac ** 2})
    if not est:
        return None
    vals = np.array([e["scrub"] for e in est])
    return {"scrub_coeff": float(np.median(vals)), "n": len(est), "spread": float(vals.std()), "detail": est}


# ── ensamblado ────────────────────────────────────────────────────────────────
def collect(d: Path) -> dict[str, list[cc.Run]]:
    kinds: dict[str, list[cc.Run]] = {}
    for p in sorted(d.glob("*.csv")):
        if p.stem in ("enc_runs", "steer", "looptime") or p.stem.startswith("tof_"):
            continue
        try:
            r = cc.load_run(p)
        except Exception:  # noqa: BLE001
            continue
        kinds.setdefault(cc.run_kind(p.stem), []).append(r)
    return kinds


def fit_all(d: Path, cpmm_override: float | None = None, steer_json: Path | None = None) -> dict:
    kinds = collect(d)
    res: dict = {"folder": str(d), "warnings": []}
    meta = d / "session.json"
    if meta.exists():
        res["session"] = json.loads(meta.read_text(encoding="utf-8"))

    enc = fit_counts_per_mm(d / "enc_runs.csv")
    if cpmm_override:
        cpmm = cpmm_override
        res["counts_per_mm_source"] = "--cpmm"
    elif enc:
        cpmm = enc["mean"]
        res["encoder"] = enc
        res["counts_per_mm_source"] = f"enc_runs.csv (n={enc['n']})"
        rel = abs(enc["mean"] - NOMINAL_CPMM) / NOMINAL_CPMM
        res["cpmm_differs_from_firmware"] = rel > 0.10
        if rel > 0.10:
            res["warnings"].append(
                f"counts_per_mm medido {enc['mean']:.2f} difiere {100 * rel:.0f} % del firmware ({NOMINAL_CPMM}): "
                "ENC_CUENTAS_POR_MM del .ino y el twin deben actualizarse (la odometria saldria mal por ese factor)")
        if enc["n"] < 3:
            res["warnings"].append("counts_per_mm con < 3 repeticiones")
        if enc["std"] / enc["mean"] > 0.01:
            res["warnings"].append(f"dispersion de counts_per_mm {100 * enc['std'] / enc['mean']:.1f} % (> 1 %): medir con regla mejor")
    else:
        cpmm = NOMINAL_CPMM
        res["counts_per_mm_source"] = "NOMINAL 20.73 (sin enc_runs.csv): k y tau quedan escalados por cpmm_real/20.73"
        res["warnings"].append("sin enc_runs.csv: las velocidades usan el cpmm nominal, no medido")
    res["counts_per_mm"] = cpmm

    errs = sum(float(r.enc_err[-1]) for rs in kinds.values() for r in rs)
    cnt = sum(abs(float(r.counts[-1])) for rs in kinds.values() for r in rs)
    if errs > 0:
        res["warnings"].append(f"el encoder reporto {int(errs)} saltos dobles en {int(cnt)} cuentas "
                               f"({100 * errs / max(cnt, 1):.3f} %): hay cuentas perdidas (ruido/pull-ups/ISR)")

    # (a) velocidad vs PWM
    sp = []
    for k in ("ramp", "dead"):
        sp += steady_points(kinds.get(k, []), cpmm, min_s=0.2 if k == "dead" else 0.25)
    for k in ("step", "stepn"):
        for r in kinds.get(k, []):
            sp += steady_points([r], cpmm, 0.4, tail=0.3)
    unstable = [p for p in sp if not p["stable"]]
    if unstable:
        res["warnings"].append(f"{len(unstable)} escalones sin estabilizar (sube --hold a >= 5*tau): se usan igual")
    fwd = [p for p in sp if p["pwm"] > 0]
    rev = [p for p in sp if p["pwm"] < 0]
    line = fit_speed_line(np.array([p["pwm"] for p in fwd]), np.array([p["v"] for p in fwd])) if fwd else None
    line_r = fit_speed_line(np.array([p["pwm"] for p in rev]), np.array([p["v"] for p in rev])) if rev else None
    if line:
        res["speed_fwd"] = line
        if line["max_resid_pct_vmax"] > 6:
            res["warnings"].append(f"v(PWM) no es lineal (residuo max {line['max_resid_pct_vmax']:.1f} % de vmax): "
                                   "el twin usa recta; considerar tabla o ajustar solo el rango 90..120")
        if line["n_points"] < 5:
            res["warnings"].append("pocos puntos por encima de la zona muerta")
    if line_r:
        res["speed_rev"] = line_r
    res["speed_points"] = sp

    # tau_drive
    steps = [fit_step(r, cpmm) for k in ("step", "stepn") for r in kinds.get(k, [])]
    steps = [s for s in steps if s]
    if steps:
        taus = np.array([s["td0"]["tau"] for s in steps])
        res["tau_drive"] = {"median_s": float(np.median(taus)), "min_s": float(taus.min()), "max_s": float(taus.max()),
                            "n": len(steps), "delay_median_s": float(np.median([s["delay"]["td"] for s in steps])),
                            "per_run": steps}
        if any(s["seems_short"] for s in steps):
            res["warnings"].append("algun escalon dura < 4 tau: alarga --ms para ajustar tau con confianza")

    # tau_coast
    for kind, key in (("coast", "tau_coast"), ("brake", "tau_brake")):
        cs = [fit_coast(r, cpmm) for r in kinds.get(kind, [])]
        cs = [c for c in cs if c]
        if cs:
            taus = np.array([c["tau"] for c in cs])
            res[key] = {"median_s": float(np.median(taus)), "min_s": float(taus.min()), "max_s": float(taus.max()),
                        "n": len(cs), "stop_mm_median": float(np.median([c["stop_mm"] for c in cs])), "per_run": cs}
            if any(c["log_too_short"] for c in cs):
                res["warnings"].append(f"{kind}: el log termina con el carro aun rodando; alarga --log-ms")

    # scrub
    sc = fit_scrub(kinds.get("lock", []), cpmm, line, load_steer(steer_json))
    if sc:
        res["scrub"] = sc

    # valores propuestos para el parche
    prop: dict = {}
    if enc:
        prop["counts_per_mm"] = round(enc["mean"], 3)
    if line:
        prop["k_mm_s_per_pwm"] = round(line["k"], 4)
        prop["pwm0"] = round(line["pwm0"], 2)
    if line_r:
        prop["k_rev_mm_s_per_pwm"] = round(line_r["k"], 4)
    if "tau_drive" in res:
        prop["tau_drive_s"] = round(res["tau_drive"]["median_s"], 3)
    if "tau_coast" in res:
        prop["tau_coast_s"] = round(res["tau_coast"]["median_s"], 3)
    if sc:
        prop["scrub_coeff"] = round(sc["scrub_coeff"], 3)
    res["proposed"] = prop
    return res


# ── parches ───────────────────────────────────────────────────────────────────
def build_patches(res: dict, root: Path) -> dict[str, tuple[str, str, str, "cc.SrcFile"]]:
    """{ruta_rel: (diff, texto_viejo, texto_nuevo, SrcFile)}"""
    prop = res["proposed"]
    stamp = dt.date.today().isoformat()
    sess = res.get("session", {})
    vb = sess.get("vbat_V")
    ctx = f"medido {stamp}" + (f" a {vb} V" if vb else "")
    out = {}

    f = cc.SrcFile.read(root, cc.PARAMS_REL)
    t = f.text
    for fld in ("k_mm_s_per_pwm", "pwm0", "k_rev_mm_s_per_pwm", "tau_drive_s", "tau_coast_s", "scrub_coeff", "counts_per_mm"):
        if fld in prop:
            t = cc.replace_field_default(t, fld, cc.fmt_num(prop[fld]))
    if "k_mm_s_per_pwm" in prop:
        L = res["speed_fwd"]
        t = cc.replace_prov(t, "motor.k/pwm0", "medido",
                            f"{ctx}: v=k(PWM-pwm0), k={prop['k_mm_s_per_pwm']}, pwm0={prop['pwm0']}, "
                            f"R2={L['r2']:.3f}, vmax={L['vmax_mm_s']:.0f} mm/s (calib_fit_motor)", f.eol)
    if "tau_drive_s" in prop:
        n = res["tau_drive"]["n"]
        t = cc.replace_prov(t, "motor.tau_drive_s", "medido",
                            f"{ctx}: tau del escalon desde reposo (mediana de {n}); retardo puro "
                            f"{1000 * res['tau_drive']['delay_median_s']:.0f} ms absorbido en tau", f.eol)
    if "tau_coast_s" in prop:
        c = res["tau_coast"]
        t = cc.replace_prov(t, "motor.tau_coast_s", "medido",
                            f"{ctx}: coast A1=A2=0, rueda {c['stop_mm_median']:.0f} mm tras cortar (mediana de {c['n']})", f.eol)
    if "scrub_coeff" in prop:
        t = cc.replace_prov(t, "motor.scrub_coeff", "medido",
                            f"{ctx}: v(volante)/v(recta) en {res['scrub']['n']} corridas", f.eol)
    if "counts_per_mm" in prop:
        e = res["encoder"]
        t = cc.replace_prov(t, "encoder.counts_per_mm", "medido",
                            f"{ctx}: {e['n']} rodadas con regla, std {e['std']:.3f} cuentas/mm (calib_fit_motor)", f.eol)
    out[cc.PARAMS_REL] = (cc.unified_diff(cc.PARAMS_REL, f.text, t), f.text, t, f)

    g = cc.SrcFile.read(root, cc.INO_REL)
    u = g.text
    if "k_mm_s_per_pwm" in prop:
        u = cc.replace_ino_const(u, "ODOM_PWM_K", f"{prop['k_mm_s_per_pwm']:g}f")
        u = cc.replace_ino_const(u, "ODOM_PWM_0", f"{int(round(prop['pwm0']))}")
    if "tau_drive_s" in prop:
        u = cc.replace_ino_const(u, "ODOM_TAU_S", f"{prop['tau_drive_s']:g}f")
    if "counts_per_mm" in prop:
        u = cc.replace_ino_const(u, "ENC_CUENTAS_POR_MM", f"{prop['counts_per_mm']:g}f")
    out[cc.INO_REL] = (cc.unified_diff(cc.INO_REL, g.text, u), g.text, u, g)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--cpmm", type=float, default=None, help="forzar cuentas/mm (si no hay enc_runs.csv)")
    ap.add_argument("--steer-json", type=Path, default=None)
    ap.add_argument("--root", type=Path, default=None, help="raiz del repo cuyos params.py/.ino se parchean")
    ap.add_argument("--apply", action="store_true", help="escribe los cambios en el arbol (por defecto solo .patch)")
    a = ap.parse_args(argv)

    d = Path(a.folder)
    res = fit_all(d, a.cpmm, a.steer_json)
    root = a.root or cc.repo_root()
    patches = build_patches(res, root) if res["proposed"] else {}

    (d / "calib_motor.json").write_text(json.dumps(res, indent=2, ensure_ascii=False, default=float), encoding="utf-8")
    print(f"cuentas/mm = {res['counts_per_mm']:.3f}   [{res['counts_per_mm_source']}]")
    for k, v in res["proposed"].items():
        print(f"  {k:22s} = {v}")
    if "speed_fwd" in res:
        L = res["speed_fwd"]
        print(f"  v(PWM) adelante: R2={L['r2']:.4f} rms={L['rms_mm_s']:.1f} mm/s arranque(>=5mm/s)=PWM {L['pwm_start_5mm_s']}")
    for w in res["warnings"]:
        print("AVISO:", w)
    if not res["proposed"]:
        print("No hay datos suficientes para proponer valores.")
        return 1
    names = {cc.PARAMS_REL: "calib_params.patch", cc.INO_REL: "calib_ino_consts.patch"}
    for rel, (diff, _old, new, f) in patches.items():
        (d / names[rel]).write_text(diff, encoding="utf-8", newline="")
        print(f"parche -> {d / names[rel]} ({diff.count(chr(10))} lineas)")
        if a.apply and diff:
            cc.write_src(root, f, new)
            print(f"  APLICADO a {root / rel}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
