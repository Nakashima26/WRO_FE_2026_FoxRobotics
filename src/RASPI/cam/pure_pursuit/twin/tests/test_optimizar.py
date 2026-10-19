"""tools/banco/optimizar.py: score, parseo de .out y encode/decode.

Desde src/RASPI/cam:  PYTHONUTF8=1 PYTHONPATH=. python -m pytest pure_pursuit/twin/tests/test_optimizar.py
"""
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

_PATH = Path(__file__).resolve().parents[1] / "tools" / "banco" / "optimizar.py"
_spec = importlib.util.spec_from_file_location("optimizar", _PATH)
opt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(opt)


def test_score_media_pareada_y_penalizacion_por_fallo_nuevo():
    base = {(1, 1): (True, 40.0), (1, 2): (True, 41.0), (2, 1): (True, 39.0)}
    cand = {(1, 1): (True, 39.0), (1, 2): (True, 40.0), (2, 1): (False, None)}
    s, info = opt.score(base, cand)
    assert s == pytest.approx(0.0)  # media -1 s, +1 s por el fallo nuevo
    assert info["new_fail"] == [(2, 1)]
    assert info["n"] == 2


def test_score_fallo_de_base_que_sigue_fallando_no_cuesta():
    base = {(1, 1): (True, 40.0), (1, 5): (False, None)}
    cand = {(1, 1): (True, 40.0), (1, 5): (False, None)}
    s, info = opt.score(base, cand)
    assert s == pytest.approx(0.0)
    assert info["new_fail"] == []


def test_score_semilla_fallida_en_dos_ruidos_rechaza():
    base = {(1, 7): (True, 40.0), (2, 7): (True, 40.0), (1, 8): (True, 40.0)}
    cand = {(1, 7): (False, None), (2, 7): (False, None), (1, 8): (True, 30.0)}
    s, _ = opt.score(base, cand)
    assert s == math.inf


def test_parse_out_con_basura_antes_y_despues(tmp_path):
    rec = {"lap_times_s": [13.9, 9.1, 9.0], "stop_reason": "race_finished", "collisions": 0}
    f = tmp_path / "s1.out"
    f.write_text("aviso previo\nSTOP race_finished " + json.dumps(rec) + "\nbasura al final\n")
    assert opt.ok_t(opt.parse_out(f)) == (True, pytest.approx(32.0))


def test_parse_out_sin_stop_o_con_pocas_vueltas_falla(tmp_path):
    f = tmp_path / "s2.out"
    f.write_text("sin linea de parada\n")
    assert opt.ok_t(opt.parse_out(f)) == (False, None)
    f.write_text('STOP x {"lap_times_s": [10.0, 9.0]}\n')
    assert opt.ok_t(opt.parse_out(f)) == (False, None)
    assert opt.ok_t(opt.parse_out(tmp_path / "no_existe.out")) == (False, None)


def test_roce_de_pared_exterior_es_falla():
    rec = {"lap_times_s": [13.9, 9.1, 9.0],
           "ignored_contacts": {"pared exterior": {"t": 5.0, "cause": "pared exterior", "max_mm": 3.0}}}
    assert opt.ok_t(rec) == (False, None)
    assert opt.ok_t({"lap_times_s": [13.9, 9.1, 9.0], "ignored_contacts": {}}) == (True, pytest.approx(32.0))


def test_encode_decode_round_trip():
    x = np.array([0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 1.0])
    pi = opt.decode(x)
    assert np.allclose(opt.encode(pi), x)


def test_decode_redondea_enteros_y_clampa_sign_near():
    pi = opt.decode([0.5, 0.5, 0.5, 0.5, 0.0, 1.0, 0.37, 0.6])
    assert pi["ROUTE_BLIND_PWM"] == 104.0  # 95 + 0.37*25 = 104.25
    assert pi["ROUTE_PROFILE_MIN_PWM"] == 217.0
    assert pi["ROUTE_PROFILE_SIGN_NEAR_MM"] == 160.0  # SIGN_MM 180 - 20
