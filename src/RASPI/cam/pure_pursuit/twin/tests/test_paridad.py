"""tools/paridad.py: cada diferencia twin <-> carro tiene motivo o rompe."""

from __future__ import annotations

import importlib

from pure_pursuit.twin.sim import resolve_preset

par = importlib.import_module("pure_pursuit.twin.tools.paridad")

_INO = """\
#ifndef FOX_ENCODER
#define FOX_ENCODER 0
#endif
#define FOX_SUELTO 1   // sin #ifndef
#include <MPU6050_tockn.h>
const int TURNS_PER_RACE = 4;   // TEST
const float GIRO_PI_CUENTA_DEG = 0.0f;
PARK_AJ int PARK_X_MM = 120;
int centroServo = 90;
void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
}
void f() {
  escribirServo(constrain(centroServo + out, 30, 160));
  escribirServo(izq ? 160 : 30);
  int ticks = constrain((int)(delta / 46.32f * 70.0f), 28, 70);
  parkServoU = izq ? centroServo + ticks : centroServo - ticks;
  SIL_AJ(PARK_X_MM);
}
"""


def _env(monkeypatch):
    monkeypatch.delenv("DIGITAL_MAP_INJECT_TRUTH", raising=False)
    monkeypatch.delenv("TWIN_SERVO_SLEW", raising=False)


def test_parse_ino():
    info = par.parse_ino(_INO)
    enc, = info.defines["FOX_ENCODER"]
    assert (enc.val, enc.line, enc.guarded) == ("0", 2, True)
    assert info.defines["FOX_SUELTO"][0].guarded is False
    assert info.consts["TURNS_PER_RACE"][0].val == "4"
    assert info.consts["GIRO_PI_CUENTA_DEG"][0].val == "0.0f"
    assert info.consts["PARK_X_MM"][0].kind == "const"
    assert info.consts["centroServo"][0].kind == "global"
    assert info.sil_aj == {"PARK_X_MM"}
    assert "MPU6050_tockn.h" in info.includes


def test_valores_normalizados():
    assert par._same("70.0f", 70.0) and par._same("12", "12")
    assert par._same(True, "true") and not par._same(True, 1.0)
    assert not par._same("0.0f", "70.0f")


def test_hw_nuevo_todo_aceptado(monkeypatch):
    _env(monkeypatch)
    rows, errors = par.compare("hw_nuevo")
    _, code = par.render(rows, errors, "hw_nuevo")
    assert errors == [] and code == 0
    by = {r.name: r for r in rows}
    # Lo que distingue al carro nuevo del .ino desplegado.
    for k in ("TURNS_PER_RACE", "FOX_ENCODER", "FOX_TOF", "imu", "pi_period_s",
              "inject_map_truth", "servo.fisico"):
        assert by[k].difiere, k
    assert not by["servo.centro"].difiere and not by["servo.rueda_izq"].difiere
    assert not by["servo.rueda_max"].difiere and by["servo.rueda_der"].difiere


def test_diferencia_sin_motivo_rompe(monkeypatch):
    _env(monkeypatch)
    cfg = resolve_preset("hw_nuevo")
    cfg["pi_overrides"] = {**cfg["pi_overrides"], "PI_KP_NUEVO_X": 1}  # no existe
    rows, errors = par.compare("hw_nuevo", cfg=cfg)
    assert any("PI_KP_NUEVO_X" in e for e in errors)
    assert par.render(rows, errors, "x")[1] == 2

    cfg = resolve_preset("hw_nuevo")
    cfg["pi_overrides"] = {**cfg["pi_overrides"], "OBS_BIAS_SHIFT": par.C.OBS_BIAS_SHIFT + 1}
    rows, errors = par.compare("hw_nuevo", cfg=cfg)
    report, code = par.render(rows, errors, "x")
    assert errors == [] and code == 1
    assert "sin aceptar (OBS_BIAS_SHIFT)" in report
    assert "OBS_BIAS_SHIFT" not in par.ACEPTADAS


def test_preset_contra_ino_sintetico(monkeypatch):
    _env(monkeypatch)
    cfg = {"fw_overrides": {"TURNS_PER_RACE": "12", "NO_ESTA": "1"},
           "fw_defines": {"FOX_ENCODER": "1", "FOX_SUELTO": "1", "FOX_NADA": "1"},
           "fw_params": {"PARK_X_MM": "130", "PARK_Y_MM": "5"},
           "inject_map_truth": False}
    rows, errors = par.compare("x", ino_text=_INO, cfg=cfg)
    by = {r.name: r for r in rows}
    assert by["TURNS_PER_RACE"].difiere and by["PARK_X_MM"].difiere
    assert not by["inject_map_truth"].difiere
    assert by["servo.fuera_de_topes"].real.startswith("20 ")
    # El 0..180 de escribirServo no cuenta como tope de maniobra.
    assert by["servo.topes"].real.startswith("30..160") and not by["servo.topes"].difiere
    assert by["servo.rueda_der"].difiere and not by["servo.rueda_izq"].difiere
    joined = " | ".join(errors)
    for k in ("NO_ESTA", "FOX_SUELTO", "FOX_NADA", "PARK_Y_MM"):
        assert k in joined, k
