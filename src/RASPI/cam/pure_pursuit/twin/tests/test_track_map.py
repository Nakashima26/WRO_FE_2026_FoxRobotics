"""Tests sintéticos: odometría, SignMap, Localizer, map_view."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from pure_pursuit.odometry import Odometry, OdomPose, parse_ack
from pure_pursuit.map_view import render_map
from pure_pursuit.track_map import (
    FieldPose,
    Localizer,
    SensorMount,
    SensorReading,
    SignMap,
    TrackGeometry,
    TrackFrame,
    field_heading_to_twin,
)


def test_parse_ack():
    d = parse_ack("ACK:V2,ang=1.5,est=S,tc=2,dL=45")
    assert d["ang"] == "1.5"
    assert d["tc"] == "2"


def test_odom_esp_pose():
    o = Odometry()
    p = o.update({"px": "100", "py": "50", "yaw": "10"}, 1.0, 200.0)
    assert p.source == "esp_pose"
    assert p.x_mm == 100 and p.y_mm == 50 and p.yaw_deg == 10


def test_odom_od_yaw_straight():
    o = Odometry()
    o.update({"od": "0", "yaw": "0"}, 0.0, 200.0)
    p = o.update({"od": "200", "yaw": "0"}, 1.0, 200.0)
    assert p.source == "od_yaw"
    assert abs(p.x_mm - 200) < 1.0


def test_odom_od_yaw_circle():
    o = Odometry()
    o.update({"od": "0", "yaw": "0"}, 0.0, 0.0)
    steps = 36
    for i in range(1, steps + 1):
        yaw = i * 10.0
        od = i * 50.0
        p = o.update({"od": str(od), "yaw": str(yaw)}, i * 0.1, 0.0)
    assert abs(p.x_mm) < 80 and abs(p.y_mm) < 80


def test_odom_legacy_turn():
    o = Odometry()
    t = 0.0
    acks = [
        "ACK:V2,ang=0,est=S,tc=0,dir=?",
        "ACK:V2,ang=0,est=G,tc=0,dir=R",
        "ACK:V2,ang=-45,est=G,tc=0,dir=R",
        "ACK:V2,ang=0,est=S,tc=1,dir=R",
    ]
    poses = []
    for a in acks:
        t += 0.1
        poses.append(o.update(parse_ack(a), t, 300.0))
    assert poses[-1].yaw_deg == pytest.approx(-90.0, abs=1.0)
    assert poses[1].x_mm == poses[0].x_mm  # sin traslación en G


def test_odom_delta_frame():
    a = OdomPose(0, 0, 0, 0, "x", "h")
    b = OdomPose(100, 0, 90, 1, "x", "h")
    dx, dy, dyaw = Odometry.delta(a, b)
    assert dyaw == pytest.approx(90.0, abs=0.1)
    assert abs(dx - 100) < 1 and abs(dy) < 1


def _field_center_to_bev_foot(cx, cy, pose: FieldPose) -> tuple[float, float]:
    """Pie BEV desde centro de señal (inversa de SignMap)."""
    h = field_heading_to_twin(pose.heading_deg)
    hr = math.radians(h)
    dx, dy = cx - pose.x_mm, cy - pose.y_mm
    fwd = dx * math.sin(hr) + dy * math.cos(hr)
    right = dx * math.cos(hr) - dy * math.sin(hr)
    cam = 60.0
    vx, vy = fwd - cam, right
    ln = math.hypot(vx, vy)
    if ln > 1e-6:
        fwd -= 25.0 * vx / ln
        right -= 25.0 * vy / ln
    bx = 200 + right / 2.0
    by = 380 - (fwd - 100.0) / 2.0   # el origen del BEV es el eje delantero
    return bx, by


def test_signmap_confirm_section0():
    geom = TrackGeometry("L")
    sm = SignMap(geom)
    # Señales conocidas en T1 roja y T2 verde (cartas simples)
    seats = {s.seat_id: s for s in geom.seats_of_section(0)}
    truth = [("T1", "Red"), ("T2", "Green")]
    rng = np.random.default_rng(42)
    pose = FieldPose(0, -1000, 0)
    for frame in range(40):
        pose = FieldPose(pose.x_mm + 40, pose.y_mm, 0)
        dets = []
        for sid, col in truth:
            s = seats[sid]
            bx, by = _field_center_to_bev_foot(s.x_mm, s.y_mm, pose)
            bx += rng.normal(0, 15)
            by += rng.normal(0, 15)
            dets.append((bx, by, col))
        sm.observe(dets, pose, frame * 0.1)
    # X1 (único asiento libre de la recta del cajón) a ~400 mm, de frente.
    pose = FieldPose(-450, -1000, 0)
    for frame in range(12):
        sm.observe([], pose, 10 + frame * 0.1)
    confirmed = sm.signs(0, "confirmed")
    colors = {s["seat"]: s["color"] for s in confirmed}
    assert colors.get("T1") == "red"
    assert colors.get("T2") == "green"
    empty = [s for s in sm.seats(0) if s["state"] == "empty" or s["empty"] > 0.5]
    assert len(empty) >= 1


def test_drive_order():
    assert TrackGeometry("L").wro_section_name(1) == "E"
    assert TrackGeometry("R").wro_section_name(1) == "W"
    # La recta del cajón no tiene T3/T4/X2.
    ids = {s.seat_id for s in TrackGeometry("L").seats_of_section(0)}
    assert ids == {"T1", "T2", "X1"}


def test_signmap_next_section():
    geom = TrackGeometry("L")
    sm = SignMap(geom)
    seats_e = {s.seat_id: s for s in geom.seats_of_section(1)}
    # En la esquina SE, ya girado hacia el norte: T1 de la recta este a ~400 mm.
    pose = FieldPose(1000, -900, 90)
    s = seats_e["T1"]
    bx, by = _field_center_to_bev_foot(s.x_mm, s.y_mm, pose)
    sm.observe([(bx, by, "Red")], pose, 0.0)
    hit = sm._seat_states.get((1, "T1"))
    assert hit is not None and hit.red > 0


def test_query_bev_roundtrip():
    from pure_pursuit.track_map import _SeatState

    geom = TrackGeometry("L")
    sm = SignMap(geom)
    seat = geom.seats_of_section(0)[0]
    sm._seat_states[(0, seat.seat_id)] = _SeatState(red=3.0)
    pose = FieldPose(0, -1000, 0)
    pts = sm.query_bev(pose, [0])
    bx, by, col, _ = pts[0]
    bx2, by2 = _field_center_to_bev_foot(seat.x_mm, seat.y_mm, pose)
    assert math.hypot(bx - bx2, by - by2) < 5.0


@pytest.mark.parametrize("drive_dir", ["L", "R"])
def test_localizer_with_twin_sensors(drive_dir):
    """Lecturas generadas por los modelos de sensor del twin (montajes reales)."""
    from pure_pursuit import wro_field
    from pure_pursuit.track_map import DEFAULT_MOUNTS
    from pure_pursuit.twin.params import TwinParams
    from pure_pursuit.twin.sensors import Pose, ToF, Ultrasonic
    from pure_pursuit.twin.world import World

    field = wro_field.randomize(1)
    field.signs.clear()
    field.barriers.clear()
    world = World(field)
    params = TwinParams()
    rng = np.random.default_rng(3)
    us, tof = Ultrasonic(params.ultrasonic, rng), ToF(params.tof, rng)
    geom = TrackGeometry(drive_dir)
    # Cerca del final de la recta sur, con la pared de enfrente a ~65 cm.
    h_field = 0.0 if drive_dir == "L" else 180.0
    x_true = 700.0 if drive_dir == "L" else -700.0
    y_true = -1080.0
    sgn = 1.0 if drive_dir == "L" else -1.0
    loc = Localizer(geom)
    loc.pose = FieldPose(x_true - sgn * 120.0, y_true + 80.0, h_field)
    tp = Pose(x_true, y_true, field_heading_to_twin(h_field))

    def us_mm(i):
        e = us.echo_us(i, world, tp)
        return None if e <= 0 else e * 0.343 / 2.0

    def tof_mm(i, t):
        v = tof.read_mm(i, world, tp, t)
        return None if v < 0 else float(v)

    for k in range(30):
        t = k * 0.05
        readings = [
            SensorReading("dL", "us", DEFAULT_MOUNTS["us_left"], 15.0, us_mm(0)),
            SensorReading("dR", "us", DEFAULT_MOUNTS["us_right"], 15.0, us_mm(1)),
            SensorReading("dF", "us", DEFAULT_MOUNTS["us_front"], 15.0, us_mm(2)),
            SensorReading("tL", "tof", DEFAULT_MOUNTS["tof_left"], 0.0, tof_mm(0, t)),
            SensorReading("tR", "tof", DEFAULT_MOUNTS["tof_right"], 0.0, tof_mm(1, t)),
        ]
        loc.update(readings)
    assert abs(loc.pose.y_mm - y_true) < 20
    assert abs(loc.pose.x_mm - x_true) < 30


def test_localizer_us_tof_inconsistent_prefers_tof():
    from pure_pursuit.track_map import DEFAULT_MOUNTS

    geom = TrackGeometry("L")
    loc = Localizer(geom)
    loc.pose = FieldPose(0, -1300, 0)   # pared exterior a ~130 mm del ToF derecho
    r_us = SensorReading("dR", "us", DEFAULT_MOUNTS["us_right"], 15.0, None)
    r_tof = SensorReading("tR", "tof", DEFAULT_MOUNTS["tof_right"], 0.0, None)
    p_us, p_tof = loc._predict(r_us), loc._predict(r_tof)
    r_us.value_mm = p_us - 200.0     # el cono del sonar agarró una señal
    r_tof.value_mm = p_tof + 5.0
    innov, st, _ray = loc._fuse_side([r_us, r_tof], "right", None)
    assert st == "tof_only"
    assert innov is not None and abs(innov - 5.0) < 1e-6


def _side_readings(true_pose: FieldPose, drive: str = "L") -> list[SensorReading]:
    from pure_pursuit.track_map import DEFAULT_MOUNTS

    truth = Localizer(TrackGeometry(drive))
    truth.pose = true_pose
    out = []
    for name in ("us_left", "us_right"):
        r = SensorReading(name, "us", DEFAULT_MOUNTS[name], 15.0, None)
        r.value_mm = truth._predict(r)
        out.append(r)
    return out


@pytest.mark.parametrize("drive,heading", [("L", 0.0), ("R", 180.0)])
def test_snap_lateral_recovers_start_offset(drive, heading):
    loc = Localizer(TrackGeometry(drive))
    loc.pose = FieldPose(-200, -1000, heading)
    assert loc.snap_lateral(_side_readings(FieldPose(-200, -1270, heading), drive))
    assert abs(loc.pose.y_mm + 1270) < 5 and abs(loc.pose.x_mm + 200) < 1


def test_snap_lateral_ignores_sign_in_cone():
    loc = Localizer(TrackGeometry("L"))
    loc.pose = FieldPose(0, -1000, 0)
    rs = _side_readings(FieldPose(0, -1200, 0))
    rs[0].value_mm -= 300.0     # el cono izquierdo agarró una señal
    assert not loc.snap_lateral(rs)
    assert loc.pose.y_mm == -1000


def test_track_frame_uses_yaw_at_reference():
    frame = TrackFrame(TrackGeometry("L"))
    frame.set_from_odom(100.0, 50.0, 30.0, start=FieldPose(0, -1000, 0))
    a = math.radians(30.0)
    p = frame.odom_to_field(100.0 + 400 * math.cos(a), 50.0 + 400 * math.sin(a), 30.0)
    assert abs(p.x_mm - 400) < 1e-6 and abs(p.y_mm + 1000) < 1e-6
    assert abs(p.heading_deg) < 1e-6


def test_render_map_speed():
    geom = TrackGeometry("L")
    sm = SignMap(geom)
    pose = FieldPose(0, -1000, 0)
    for _ in range(3):
        render_map(geom, sm, pose, size=400)
    t0 = time.perf_counter()
    for _ in range(20):
        img = render_map(geom, sm, pose, size=400)
    dt = (time.perf_counter() - t0) / 20
    assert img.shape == (400, 400, 3)
    assert dt < 0.010
