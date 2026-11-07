"""Integración mapa en PPRuntime (shadow / V2 / odometría)."""

from __future__ import annotations

import time

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit.bev import BEVTransformer
from pure_pursuit.runtime_nuevo import PPConfig, PPRuntime
from pure_pursuit.twin.tests.test_runtime_step import FakeSerialLink, _floor_frame, _make_runtime


def _run_one_v2(monkeypatch, shadow: bool) -> str:
    monkeypatch.setattr(C, "TRACK_MAP_ENABLED", False, raising=False)
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", shadow, raising=False)
    rt, serial = _make_runtime()
    serial.queue_ack("ACK:V2,ang=0.0,est=S,dir=L,tc=0,tpr=12,dL=80,dR=80")
    frame = _floor_frame()
    now = time.perf_counter()
    rt.process_frame(frame, now, armed=True)
    assert len(serial.sent) == 1
    return serial.sent[0]


def test_v2_byte_identical_shadow_on_off(monkeypatch):
    off = _run_one_v2(monkeypatch, False)
    on = _run_one_v2(monkeypatch, True)
    assert off == on


def test_v2_gr_gf_when_enabled(monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_ENABLED", True, raising=False)
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    rt, serial = _make_runtime()
    t = time.perf_counter()
    serial.queue_ack("ACK:V2,ang=0.0,est=S,dir=L,tc=0,tpr=12")
    rt.process_frame(_floor_frame(), t, armed=True)
    serial.queue_ack("ACK:V2,ang=0.0,est=S,dir=L,tc=0,tpr=12")
    rt.process_frame(_floor_frame(), t + 0.05, armed=True)
    msg = serial.sent[-1]
    assert ",gr=" in msg and ",gf=" in msg


def test_map_log_in_shadow(capsys, monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_ENABLED", False, raising=False)
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    rt, serial = _make_runtime()
    serial.queue_ack("ACK:V2,ang=0.0,est=S,dir=L,tc=0")
    rt.process_frame(_floor_frame(), time.perf_counter(), armed=True)
    out = capsys.readouterr().out
    assert "[MAP]" in out


def test_render_map_panel_size(monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    rt, serial = _make_runtime()
    serial.queue_ack("ACK:V2,ang=0.0,est=S,dir=L,tc=0")
    rt.process_frame(_floor_frame(), time.perf_counter(), armed=True)
    panel = rt.render_map_panel(240)
    assert panel.shape == (240, 240, 3)


def test_odom_integrates_od_yaw(monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    rt, serial = _make_runtime()
    t0 = time.perf_counter()
    serial.queue_ack("ACK:V2,yaw=0,od=0,dir=L,est=S,tc=0")
    rt.process_frame(_floor_frame(), t0, armed=True)
    serial.queue_ack("ACK:V2,yaw=0,od=500,dir=L,est=S,tc=0")
    rt.process_frame(_floor_frame(), t0 + 0.1, armed=True)
    assert rt._odom.pose is not None
    assert rt._odom.pose.source == "od_yaw"
    assert rt._odom.pose.x_mm > 400


def test_legacy_ack_pose(monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    rt, serial = _make_runtime()
    t = time.perf_counter()
    for ack in (
        "ACK:V2,ang=0,est=S,tc=0,dir=?",
        "ACK:V2,ang=-30,est=G,tc=0,dir=R",
        "ACK:V2,ang=0,est=S,tc=1,dir=R",
    ):
        serial.queue_ack(ack)
        rt.process_frame(_floor_frame(), t, armed=True)
        t += 0.08
    assert rt._odom.pose is not None
    assert rt._odom.pose.source == "legacy"
    assert rt._map_field_pose is not None


def _shadow_only(monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    monkeypatch.setattr(C, "TRACK_MAP_ENABLED", False, raising=False)
    monkeypatch.setattr(C, "CORNER_TURN_DIR_OVERRIDE", None, raising=False)


def test_map_pose_includes_motion_before_direction_known(monkeypatch):
    _shadow_only(monkeypatch)
    rt, serial = _make_runtime()
    t = time.perf_counter()
    for i, d in enumerate(("?", "?", "?", "L")):
        serial.queue_ack(f"ACK:V2,yaw=0,od={i * 200},dir={d},est=S,tc=0")
        rt.process_frame(_floor_frame(), t + 0.1 * i, armed=True)
        if d == "?":
            assert rt._map_geometry is None
    assert rt._map_geometry.drive_dir == "L"
    assert 550 < rt._map_field_pose.x_mm < 650


def test_map_start_lateral_snap(monkeypatch):
    _shadow_only(monkeypatch)
    rt, serial = _make_runtime()
    # Carro 250 mm a la derecha del centro del pasillo de salida (hacia la pared exterior).
    serial.queue_ack("ACK:V2,yaw=0,od=0,dir=L,est=S,tc=0,dL=68,dR=18")
    rt.process_frame(_floor_frame(), time.perf_counter(), armed=True)
    assert abs(rt._map_field_pose.y_mm + 1250) < 15
    assert rt._map_snap_tries == 0


def test_map_error_does_not_break_frame(monkeypatch, capsys):
    _shadow_only(monkeypatch)
    rt, serial = _make_runtime()
    sent_ok = _run_one_v2(monkeypatch, False)

    def boom(*_a, **_k):
        raise RuntimeError("boom")

    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    monkeypatch.setattr(rt._odom, "update", boom)
    serial.queue_ack("ACK:V2,ang=0.0,est=S,dir=L,tc=0,tpr=12,dL=80,dR=80")
    rt.process_frame(_floor_frame(), time.perf_counter(), armed=True)
    assert rt._map_failed
    assert serial.sent[-1] == sent_ok
    assert "[MAP] ERROR" in capsys.readouterr().out
    assert rt.render_map_panel(120).shape == (120, 120, 3)


def test_gr_gf_from_mapped_entry_sign(monkeypatch):
    from pure_pursuit.odometry import OdomPose
    from pure_pursuit.track_map import _SeatState

    monkeypatch.setattr(C, "TRACK_MAP_ENABLED", True, raising=False)
    rt, _serial = _make_runtime()
    rt._ensure_map_geometry("L", OdomPose(0.0, 0.0, 0.0, 0.0, "esp_pose", "high"))
    geom = rt._map_geometry
    near = min(geom.seats_of_section(1),
               key=lambda st: geom.along_lateral(1, st.x_mm, st.y_mm)[0])
    rt._map_signmap._seat_states[(1, near.seat_id)] = _SeatState(green=3.0)
    assert rt._map_gr_gf_suffix() == f",gr=1,gf={C.GR_GF_INNER_CM}"   # L: verde por dentro
    rt._map_signmap._seat_states[(1, near.seat_id)] = _SeatState(red=3.0)
    assert rt._map_gr_gf_suffix() == f",gr=1,gf={C.GR_GF_OUTER_CM}"
    assert not rt._map_failed


def test_shadow_cost_budget(monkeypatch):
    monkeypatch.setattr(C, "TRACK_MAP_SHADOW", True, raising=False)
    monkeypatch.setattr(C, "TRACK_MAP_ENABLED", False, raising=False)
    rt, serial = _make_runtime()
    t = time.perf_counter()
    for i in range(5):
        serial.queue_ack(
            f"ACK:V2,ang={i},est=S,dir=L,tc=0,dL=70,dR=90,yaw={i},od={i * 100}"
        )
        rt.process_frame(_floor_frame(), t + i * 0.05, armed=True)
    assert rt._map_shadow_ms < 15.0
    rt.render_map_panel(400)
    assert rt._map_render_ms < 50.0
