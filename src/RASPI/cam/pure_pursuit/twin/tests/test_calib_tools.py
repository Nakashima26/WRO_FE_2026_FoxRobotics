"""Herramientas de calibracion (tools/calib): ajuste con datos sinteticos, parches y
CalibFox.ino compilado contra un mock de Arduino (se salta si no hay compilador C++).

Desde src/RASPI/cam:  PYTHONUTF8=1 PYTHONPATH=. python -m pytest pure_pursuit/twin/tests/test_calib_tools.py
"""

from __future__ import annotations

import ast
import math
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit.twin.tools.calib import calib_capture as cap
from pure_pursuit.twin.tools.calib import calib_common as cc
from pure_pursuit.twin.tools.calib import calib_fit_motor as fm
from pure_pursuit.twin.tools.calib import calib_fit_steer as fs

ROOT = cc.repo_root(Path(__file__))

# Planta verdadera sintetica (distinta de los defaults del twin)
K, PWM0, TAU, TAUC, CPMM = 4.1, 19.0, 0.21, 0.11, 9.4


# ── generadores sinteticos (mismo formato CSV que CalibFox) ───────────────────
def _csv(t_ms, pwm, counts, servo=90, err=0):
    lines = ["t_ms,pwm,enc_counts,v_mm_s,servo_deg,enc_err"]
    for a, b, c in zip(t_ms, pwm, counts):
        lines.append(f"{a:.3f},{int(b)},{int(c)},0.0,{servo},{err}")
    return "\n".join(lines) + "\n"


def synth(profile, total_ms, k=K, pwm0=PWM0, tau=TAU, tauc=TAUC, cpmm=CPMM, dt=0.01, scrub_f=1.0):
    """profile(t_ms) -> (pwm, coast_flag). Euler fino, cuentas enteras."""
    n = int(total_ms / (dt * 1000))
    t = np.arange(n) * dt * 1000
    v = s = 0.0
    pw, cn = [], []
    for ti in t:
        p, coast = profile(ti)
        tgt = np.sign(p) * k * (abs(p) - pwm0) * scrub_f if abs(p) > pwm0 else 0.0
        ta = tauc if coast else tau
        sub = 10
        for _ in range(sub):
            v += (tgt - v) * (dt / sub / ta)
            s += v * dt / sub
        pw.append(p)
        cn.append(round(s * cpmm))
    return cc.parse_csv_text(_csv(t, pw, cn), "synth")


def test_fit_speed_line_recovers_k_pwm0():
    levels = list(range(20, 131, 10))
    hold = 800

    def prof(t):
        return levels[min(int(t // hold), len(levels) - 1)], False

    run = synth(prof, hold * len(levels))
    pts = fm.steady_points([run], CPMM)
    line = fm.fit_speed_line(np.array([p["pwm"] for p in pts]), np.array([p["v"] for p in pts]))
    assert line["k"] == pytest.approx(K, rel=0.03)
    assert line["pwm0"] == pytest.approx(PWM0, abs=1.5)
    assert line["r2"] > 0.999


def test_fit_step_tau_and_coast():
    run = synth(lambda t: (90, False), 2500)
    r = fm.fit_step(run, CPMM)
    assert r["td0"]["tau"] == pytest.approx(TAU, rel=0.08)
    assert r["td0"]["v_ss"] == pytest.approx(K * (90 - PWM0), rel=0.03)

    run = synth(lambda t: (100, False) if t < 1500 else (0, True), 3500)
    c = fm.fit_coast(run, CPMM)
    assert c["tau"] == pytest.approx(TAUC, rel=0.12)
    expect_stop = K * (100 - PWM0) * TAUC
    assert c["stop_mm"] == pytest.approx(expect_stop, rel=0.08)
    assert not c["log_too_short"]


def test_fit_scrub():
    class St:  # direccion con gain conocida
        wmax = 44.54

        @staticmethod
        def wheel(servo):
            return (90.0 - servo) * 0.627

    straight = synth(lambda t: (100, False), 2500)
    f = 0.80
    # el servo viaja en la columna servo_deg: se rehace el Run con servo=40
    locked = synth(lambda t: (100, False), 2500, scrub_f=f)
    locked.servo[:] = 40.0
    straight.servo[:] = 90.0
    line = {"k": K, "pwm0": PWM0}
    out = fm.fit_scrub([straight, locked], CPMM, line, St())
    frac = abs(St.wheel(40)) / St.wmax
    assert out["scrub_coeff"] == pytest.approx((1 - f) / frac ** 2, rel=0.08)


def test_counts_per_mm(tmp_path):
    p = tmp_path / "enc_runs.csv"
    p.write_text("counts_fwd,dist_mm,counts_net,enc_err\n9410,1000,3,0\n9390,1000,-2,0\n9402,1000,1,0\n")
    e = fm.fit_counts_per_mm(p)
    assert e["mean"] == pytest.approx(9.4007, abs=1e-3)
    assert e["net_after_return_pct_of_fwd"] < 0.1


def test_cpmm_warns_when_far_from_firmware(tmp_path):
    (tmp_path / "enc_runs.csv").write_text("counts_fwd,dist_mm,counts_net,enc_err\n8900,1000,0,0\n8910,1000,1,0\n8905,1000,0,0\n")
    res = fm.fit_all(tmp_path)
    assert res["cpmm_differs_from_firmware"] is True
    assert any("difiere" in w for w in res["warnings"])

    (tmp_path / "enc_runs.csv").write_text("counts_fwd,dist_mm,counts_net,enc_err\n20730,1000,0,0\n20740,1000,1,0\n20725,1000,0,0\n")
    res = fm.fit_all(tmp_path)
    assert res["cpmm_differs_from_firmware"] is False
    assert not any("difiere" in w for w in res["warnings"])


# ── parches ───────────────────────────────────────────────────────────────────
def _fake_res():
    return {
        "proposed": {"k_mm_s_per_pwm": 4.1, "pwm0": 19.0, "k_rev_mm_s_per_pwm": 3.9, "tau_drive_s": 0.21,
                     "tau_coast_s": 0.11, "scrub_coeff": 0.2, "counts_per_mm": 9.4},
        "speed_fwd": {"r2": 0.999, "vmax_mm_s": 400.0},
        "tau_drive": {"n": 3, "delay_median_s": 0.03},
        "tau_coast": {"n": 3, "stop_mm_median": 30.0},
        "scrub": {"n": 4},
        "encoder": {"n": 5, "std": 0.01},
        "session": {"vbat_V": 12.1},
    }


def test_motor_patch_content_and_applies(tmp_path):
    patches = fm.build_patches(_fake_res(), ROOT)
    diff_p, _, new_p, _ = patches[cc.PARAMS_REL]
    assert "+    k_mm_s_per_pwm: float = 4.1" in diff_p
    assert "+    counts_per_mm: float = 9.4" in diff_p
    assert '"medido"' in diff_p
    ast.parse(new_p)                              # sigue siendo Python valido
    diff_i, _, new_i, _ = patches[cc.INO_REL]
    assert "ODOM_PWM_K           = 4.1f;" in diff_i
    assert "ENC_CUENTAS_POR_MM   = 9.4f;" in diff_i
    assert "ODOM_PWM_0           = 19;" in diff_i
    if shutil.which("git"):
        for name, d in (("a.patch", diff_p), ("b.patch", diff_i)):
            f = tmp_path / name
            f.write_text(d, encoding="utf-8", newline="")
            r = subprocess.run(["git", "apply", "--check", str(f)], cwd=ROOT, capture_output=True, text=True)
            assert r.returncode == 0, r.stderr


def test_params_patch_roundtrip_values(tmp_path):
    """El params.py parcheado, ejecutado, entrega los valores medidos y los marca 'medido'."""
    _, _, new_p, _ = fm.build_patches(_fake_res(), ROOT)[cc.PARAMS_REL]
    (tmp_path / "pure_pursuit").mkdir()
    # copia la jerarquia minima: reemplaza solo el modulo y lo importa como paquete hermano
    src_twin = ROOT / "src/RASPI/cam/pure_pursuit/twin"
    dst = tmp_path / "pure_pursuit"
    shutil.copy(ROOT / "src/RASPI/cam/pure_pursuit/config.py", dst / "config.py")
    (dst / "__init__.py").write_text("")
    (dst / "twin").mkdir()
    (dst / "twin" / "__init__.py").write_text("")
    (dst / "twin" / "params.py").write_text(new_p, encoding="utf-8", newline="")
    code = ("from pure_pursuit.twin.params import TwinParams;p=TwinParams();"
            "print(p.motor.k_mm_s_per_pwm,p.motor.pwm0,p.motor.tau_drive_s,p.encoder.counts_per_mm,"
            "p.motor.scrub_coeff);print([l for l in p.describe() if 'motor.k' in l or 'counts_per_mm' in l])")
    import os
    import sys
    env = dict(os.environ, PYTHONPATH=str(tmp_path), PYTHONUTF8="1")
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env, cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert r.stdout.splitlines()[0].startswith("4.1 19.0 0.21 9.4 0.2")
    assert "[MEDIDO] motor.k/pwm0" in r.stdout and "[MEDIDO] encoder.counts_per_mm" in r.stdout


# ── direccion ─────────────────────────────────────────────────────────────────
def test_steer_inner_conversion_matches_config():
    d = fs.delta_eq_from_inner(C.WHEEL_INNER_FULL_LOCK_DEG, C.WHEELBASE_MM, C.KINGPIN_TRACK_MM)
    assert d == pytest.approx(C.MAX_WHEEL_STEER_DEG, abs=0.02)


def test_steer_fit_gains_and_ppservogain(tmp_path):
    L = 113.0
    gl, gr = 0.55, 0.60
    rows = []
    for s in (110, 120, 130, 150, 160):
        d = gl * (s - 90)
        rows.append({"servo_deg": s, "radius_mm": L / math.tan(math.radians(d))})
    for s in (70, 60, 50, 40, 30):
        d = gr * (90 - s)
        rows.append({"servo_deg": s, "radius_mm": L / math.tan(math.radians(d))})
    res = fs.fit_steer(rows, 90.0, L, 60.0)
    assert res["gain_left_deg_per_deg"] == pytest.approx(gl, abs=0.02)   # tan() no es lineal: tolera
    assert res["gain_right_deg_per_deg"] == pytest.approx(gr, abs=0.02)
    assert res["ppServoGain_for_wheel_eq_steer"] == pytest.approx(1 / (0.5 * (gl + gr)), rel=0.03)
    assert res["min_turn_radius_mm"] > 0
    patches = fs.build_patches(res, ROOT)
    dp = patches[cc.PARAMS_REL][0]
    assert "gain_left: float = " in dp and "C.MAX_WHEEL_STEER_DEG" in dp   # lo viejo en las lineas '-'
    di = patches[cc.INO_REL][0]
    assert "+float ppServoGain = 1.8" in di or "+float ppServoGain = 1.7" in di


def test_steer_fit_flags_saturation_and_other_inputs():
    L = 113.0
    rows = [{"servo_deg": 90 + x, "radius_mm": L / math.tan(math.radians(min(x, 60) * 0.6))} for x in (10, 20, 40, 60, 70)]
    rows += [{"servo_deg": 90 - x, "inner_deg": 0.8 * x} for x in (10, 30, 50)]
    rows += [{"servo_deg": 80, "v_mm_s": 200.0, "yaw_dps": math.degrees(200 / 400)}]
    res = fs.fit_steer(rows, 90.0, L, 60.0)
    assert any("tope mecanico" in w for w in res["warnings"])
    assert set(res["sources"]) >= {"radio", "transportador", "yaw"}


# ── CalibFox.ino contra el mock ───────────────────────────────────────────────
@pytest.fixture(scope="session")
def mock_exe(tmp_path_factory):
    from pure_pursuit.twin.tools.calib import mock_build
    try:
        return mock_build.build(tmp_path_factory.mktemp("mock"))
    except RuntimeError as e:
        if "sin compilador" in str(e):
            pytest.skip(str(e))
        raise


def _session(mock_exe, tmp_path, **env):
    from pure_pursuit.twin.tools.calib import mock_build
    port = mock_build.MockPort(mock_exe, env)
    for _ in range(4):
        port.readline()
    link = cap.CalibLink(port)
    link.wait_ready()
    s = cap.Session(link, tmp_path, vbat=12.0, say=lambda m: None)
    s.settle_s = 0.0
    return s, port


def test_calibfox_commands_and_csv(mock_exe, tmp_path):
    s, port = _session(mock_exe, tmp_path)
    r = s.link.command("PWM 100 800")
    assert r.ok and r.rows[0].startswith("t_ms,pwm,enc_counts")
    run = cc.parse_csv_text(r.csv_text)
    assert run.pwm[0] == 100 and run.counts[-1] > 1000 and run.n > 70    # periodo 10 ms
    assert np.all(np.diff(run.counts) >= 0)                              # adelante = cuentas +
    assert s.link.command("PWM 999 100").status == "ERR"
    assert s.link.command("PWM 100 99999").status == "ERR"
    assert s.link.command("SERVO 5").status == "ERR"
    assert s.link.command("SERVO 120").ok
    assert s.link.command("FOO").status == "ERR"
    # reversa: cuentas decrecen y el firmware esperó el freno (no se invierte con el motor girando)
    r = s.link.command("PWM -100 800")
    rr = cc.parse_csv_text(r.csv_text)
    assert rr.counts[-1] < -1000 and rr.pwm[0] == -100
    assert s.link.command("MAXPWM 60").ok
    r = s.link.command("PWM 120 300")
    assert max(cc.parse_csv_text(r.csv_text).pwm) == 60                  # recorte a MAXPWM
    port.close()


def test_calibfox_session_to_fit_roundtrip(mock_exe, tmp_path):
    """CalibFox (mock, planta K=4.2 pwm0=18 tau=.18 tauc=.10 cpmm=9) -> capture -> fit."""
    s, port = _session(mock_exe, tmp_path, MOCK_G="0.627", MOCK_SCRUB="0.25")
    s.ramp(1, 20, 130, 10, 900, prompt=False)
    s.step([90], 2, 2500, prompt=False)
    s.coast([100], 2, 1500, 2500, brake=False, prompt=False)
    s.lock(100, [90, 40], 1, 2500, prompt=False)
    port.close()
    (tmp_path / "enc_runs.csv").write_text("counts_fwd,dist_mm,counts_net,enc_err\n9000,1000,0,0\n9010,1000,1,0\n")
    res = fm.fit_all(tmp_path)
    assert res["counts_per_mm"] == pytest.approx(9.005, abs=0.01)
    assert res["speed_fwd"]["k"] == pytest.approx(4.2, rel=0.04)
    assert res["speed_fwd"]["pwm0"] == pytest.approx(18.0, abs=2.0)
    assert res["speed_rev"]["k"] == pytest.approx(3.9, rel=0.05)
    assert res["tau_drive"]["median_s"] == pytest.approx(0.18, rel=0.15)
    assert res["tau_coast"]["median_s"] == pytest.approx(0.10, rel=0.2)
    assert res["scrub"]["scrub_coeff"] == pytest.approx(0.25, abs=0.06)
    assert not any("encoder reporto" in w for w in res["warnings"])
    patches = fm.build_patches(res, ROOT)
    assert "k_mm_s_per_pwm" in patches[cc.PARAMS_REL][0]


def test_calibfox_detects_lost_counts(mock_exe, tmp_path):
    s, port = _session(mock_exe, tmp_path, MOCK_LOSS="0.02")
    s.ramp(1, 40, 100, 20, 900, prompt=False)
    port.close()
    res = fm.fit_all(tmp_path, cpmm_override=9.0)
    assert any("saltos dobles" in w for w in res["warnings"])


def test_calibfox_looptime_and_tof(mock_exe, tmp_path):
    s, port = _session(mock_exe, tmp_path)
    r = s.link.command("LOOPTIME 50", 60)
    assert r.ok
    tab = {ln.split(",")[0]: ln.split(",") for ln in r.rows[1:]}
    assert int(tab["mpu_update"][2]) >= 3600                  # 2 x 1800 us del stub
    assert int(tab["ping_F"][6]) == 50                        # frontal sin eco: timeout x50
    assert int(tab["ping_F"][2]) >= 7000
    assert int(tab["ping_L"][6]) == 0
    assert int(tab["total"][2]) <= int(tab["total"][3]) <= int(tab["total"][5])
    # TOF: reproduce el hallazgo (presupuesto por defecto 50 ms => ~20 Hz aunque se pida 33 ms)
    def rate(cmd):
        rr = s.link.command(cmd, 30)
        assert rr.ok, rr.text
        sm = [c for c in rr.comments if c.startswith("# TOFSUM")][0]
        return float(sm.split("rate_hz=")[1].split()[0])
    assert rate("TOF F LONG DEF 2000 33") == pytest.approx(20.0, rel=0.1)
    assert rate("TOF L SHORT 20 2000") == pytest.approx(50.0, rel=0.1)
    assert rate("TOF F LONG 33 2000") == pytest.approx(30.0, rel=0.1)
    assert s.link.command("TOF X SHORT 20 1000").status == "ERR"
    port.close()
