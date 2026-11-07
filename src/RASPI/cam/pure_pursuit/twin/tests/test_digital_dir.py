"""Sentido del mapa digital desde el ACK (dir=), no desde config ni el twin."""

from __future__ import annotations

import pytest

from pure_pursuit import config as C
from pure_pursuit.digital_map import DigitalMap
from pure_pursuit.twin.sim import Sim, map_truth_keys
from pure_pursuit.twin.tests.test_runtime_step import _make_runtime


def _acks(dir_from: int, d: str) -> list[str | None]:
    """Arranque en el cajón como en el twin: dir=? un par de frames (y frames
    sin ACK), luego el ESP decide y el carro avanza por la recta."""
    out: list[str | None] = []
    for i in range(14):
        dd = d if i >= dir_from else "?"
        est = "S" if i == 0 else ("I" if i < 8 else "S")
        px = 0.0 if i < 5 else 40.0 * (i - 4)
        if i in (1, 2, 3):
            out.append(None)
            continue
        out.append(
            f"ACK:V2,ang=0.0,est={est},dir={dd},tc=0,tpr=12,px={px:.1f},py=0.0,"
            f"yaw=0.0,dF=180,dL=60,dR=30,tL=600,tR=300,tB=50,tF=1800"
        )
    return out


def _feet(i: int) -> list[tuple[float, float, str]]:
    # Rejilla sobre la hoja: algunas caen cerca de un asiento y votan.
    color = "Red" if i % 2 else "Green"
    return [(float(bx), float(by), color)
            for bx in range(0, int(C.BEV_W), 40)
            for by in range(10, int(C.ROBOT_BEV_Y), 40)]


def _orange(i: int) -> dict:
    return {"seen": i > 10, "near_y": 120.0 + i}


@pytest.fixture
def map_cfg(monkeypatch):
    monkeypatch.setattr(C, "DIGITAL_MAP_STEER", True, raising=False)
    monkeypatch.setattr(C, "DIGITAL_MAP_WALL_FIX", True, raising=False)
    monkeypatch.setattr(C, "CORNER_TURN_DIR_OVERRIDE", None, raising=False)
    return monkeypatch


def test_map_undecided_is_neutral(map_cfg):
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", None)
    dm = DigitalMap()
    assert dm.direction is None
    assert dm.in_stall and not dm._aligned
    assert not dm.blocks_turn()
    assert not dm.needs_line()
    assert not dm.holds_for_center()
    assert not dm.hold_pasado()
    assert dm.line_bev() == []
    assert dm.render(120).shape == (120, 120, 3)


@pytest.mark.parametrize("d", ["CW", "CCW"])
def test_set_direction_equals_forced(map_cfg, d):
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", d)
    forced = DigitalMap()
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", None)
    late = DigitalMap()
    assert late.set_direction(d) is True
    assert vars(late) == vars(forced)
    # Una sola vez: un dir= distinto después no espejea el mapa.
    other = "CCW" if d == "CW" else "CW"
    assert late.set_direction(other) is False
    assert late.direction == d


def test_config_garbage_is_undecided(map_cfg):
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", "norte")
    assert DigitalMap().direction is None


@pytest.mark.parametrize("d,want", [("R", "CW"), ("L", "CCW")])
def test_update_digital_replays_until_dir(map_cfg, d, want):
    acks = _acks(5, d)
    # Referencia: el mapa nace con el sentido correcto (lo que hacía la
    # inyección del twin) y recibe exactamente los mismos updates.
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", want)
    ref = DigitalMap()
    for i, ack in enumerate(acks):
        ref.update(ack, _feet(i), _orange(i))

    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", None)
    rt, _serial = _make_runtime()
    assert rt.digital.direction is None
    for i, ack in enumerate(acks):
        rt._update_digital(ack, _feet(i), _orange(i))
        if i < 5:
            assert rt.digital.direction is None
            assert len(rt._dmap_pending) == i + 1
    assert rt.digital.direction == want
    assert rt._dmap_pending == []
    assert vars(rt.digital) == vars(ref)


def test_update_digital_pending_cap(map_cfg):
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", None)
    rt, _serial = _make_runtime()
    rt._DMAP_PENDING_MAX = 3
    acks = _acks(6, "R")
    for i in range(6):
        rt._update_digital(acks[i], _feet(i), _orange(i))
    assert rt.digital.direction is None
    # Se guardan los primeros (ancla de odometría), no los últimos.
    assert [p[0] for p in rt._dmap_pending] == acks[:3]
    rt._update_digital(acks[6], _feet(6), _orange(6))
    assert rt.digital.direction == "CW" and rt._dmap_pending == []


def test_update_digital_ignores_vision_tracker(map_cfg):
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", None)
    rt, _serial = _make_runtime()
    rt.turn_dir_tracker.direction = "L"
    ack = "ACK:V2,ang=0.0,est=I,dir=?,tc=0,tpr=12,px=0.0,py=0.0,yaw=0.0"
    assert rt._resolve_map_drive_dir({}, ack) == "L"
    assert rt._resolve_map_drive_dir({}, ack, use_vision=False) is None
    rt._update_digital(ack, [], None)
    assert rt.digital.direction is None
    assert len(rt._dmap_pending) == 1


def test_update_digital_override(map_cfg):
    map_cfg.setattr(C, "DIGITAL_MAP_DIRECTION", None)
    map_cfg.setattr(C, "CORNER_TURN_DIR_OVERRIDE", "R")
    rt, _serial = _make_runtime()
    rt._update_digital("ACK:V2,est=S,dir=?,tc=0", [], None)
    assert rt.digital.direction == "CW"


def test_map_truth_keys():
    both = ("parking", "direction")
    assert map_truth_keys(None) == both
    assert map_truth_keys(True) == both
    assert map_truth_keys("1") == both
    assert map_truth_keys(False) == ()
    assert map_truth_keys("0") == ()
    assert map_truth_keys("False") == ()
    assert map_truth_keys("direction") == ("direction",)
    assert map_truth_keys("parking, direction") == both
    with pytest.raises(ValueError):
        map_truth_keys("sentido")


def test_sim_inject_option(monkeypatch):
    monkeypatch.delenv("DIGITAL_MAP_INJECT_TRUTH", raising=False)
    assert Sim(1, preset="hw_nuevo").inject_map_truth == ("parking", "direction")
    assert Sim(1, preset="hw_nuevo", inject_map_truth=False).inject_map_truth == ()
    monkeypatch.setenv("DIGITAL_MAP_INJECT_TRUTH", "0")
    assert Sim(1, preset="hw_nuevo").inject_map_truth == ()
    # El argumento manda sobre el env.
    assert Sim(1, preset="hw_nuevo", inject_map_truth="direction").inject_map_truth == ("direction",)
