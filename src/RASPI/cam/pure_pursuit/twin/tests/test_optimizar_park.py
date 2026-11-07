"""tools/banco/optimizar_park.py: encode/decode de perillas fw, make_cfg, ok_t y score.

Desde src/RASPI/cam:  PYTHONUTF8=1 PYTHONPATH=. python -m pytest pure_pursuit/twin/tests/test_optimizar_park.py
"""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

_PATH = Path(__file__).resolve().parents[1] / "tools" / "banco" / "optimizar_park.py"
_spec = importlib.util.spec_from_file_location("optimizar_park", _PATH)
opt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(opt)

CAM = Path(__file__).resolve().parents[3]
FW = {"PARK_REV_PWM": "79", "PARK_PWM": "90", "PARK_CENTER_PWM_PAR": "70",
      "PARK_RETORNO_PWM": "86", "PARK_RETORNO_REV_PWM": "86", "PARK_APPROACH_PWM": "90",
      "PARK_SCAN_RAPIDO_MM": "350", "PARK_ARMADO_MS": "400", "MANIOBRA_FRENO_MS": "55",
      "PARK_ATRAS_FINAL_CM": "4.0"}


def test_encode_decode_round_trip_fw():
    pi = opt.decode(opt.encode(FW))
    for k, v in FW.items():
        if k == "PARK_ATRAS_FINAL_CM":
            assert pi[k] == pytest.approx(float(v), abs=1e-6)
        else:
            assert pi[k] == float(v)


def test_make_cfg_escribe_fw_como_strings_y_no_toca_pi():
    cfg = {"fw": dict(FW), "pi": {"ROUTE_BLIND_PWM": 104.0}}
    ctx = SimpleNamespace(cfg=cfg)
    _, js = opt.make_cfg(ctx, opt.encode(FW))
    out = json.loads(js)
    assert out["fw"]["PARK_REV_PWM"] == "79"
    assert out["fw"]["PARK_ATRAS_FINAL_CM"] == "4.00"
    assert all(isinstance(out["fw"][k], str) for k in opt.KEYS)
    assert out["pi"] == cfg["pi"]


def test_ok_t_run_real_pbase():
    sd = CAM / "runs" / "pbase" / "cand_n1" / "s1"
    if not Path(f"{sd}.out").exists() or not sd.exists():
        pytest.skip("falta runs/pbase/cand_n1/s1")
    ok, t = opt.ok_t(opt.parse_out(f"{sd}.out"), sd)
    assert ok is True
    assert 38 <= t <= 40


def test_score_media_pareada_y_penalizacion_por_fallo_nuevo():
    base = {(1, 1): (True, 40.0), (1, 2): (True, 41.0), (2, 1): (True, 39.0)}
    cand = {(1, 1): (True, 39.0), (1, 2): (True, 40.0), (2, 1): (False, None)}
    s, info = opt.score(base, cand)
    assert s == pytest.approx(0.0)  # media -1 s, +1 s por el fallo nuevo
    assert info["new_fail"] == [(2, 1)]
    assert info["n"] == 2
