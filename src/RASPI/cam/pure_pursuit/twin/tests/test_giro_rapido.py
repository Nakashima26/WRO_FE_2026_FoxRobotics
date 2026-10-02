"""GIRO_RAPIDO vs MANIOBRA corner timing in closed-loop twin + FirmwareSIL."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

import numpy as np
import pytest

from pure_pursuit import wro_field
from pure_pursuit.wro_field import Field
from pure_pursuit.twin.firmware.fw import FirmwareSIL
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.sensors import Gyro, Pose, Ultrasonic
from pure_pursuit.twin.vehicle import Vehicle
from pure_pursuit.twin.world import World, collision

V2_LINE = (
    "V2,obs=+0.000,turn=0,state=pp_follow,prio=0,mem=0,pp=1,"
    "pasado=0,intr=0,inicio=0,park=0,pd=0"
)

DT_LOOP_S = 0.025
V2_PERIOD_S = 0.07


@dataclass
class CornerRun:
    corner_time_s: float | None
    heading_delta_deg: float
    min_clearance_mm: float
    exit_outer_wall_mm: float
    exit_centerline_mm: float
    saw_reverse: bool


def _parse_ack(ln: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in ln.split(",")[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            out[k] = v
    return out


def _echo_mm(us: int) -> float:
    if us <= 0:
        return 9999.0
    return us * 0.343 / 2.0


def _synthetic_field(seed: int = 42) -> Field:
    parking = "N"
    sx, sy = wro_field._start_of(parking)
    return Field(
        seed=seed,
        direction="CW",
        single_section="E",
        parking_section=parking,
        cards={"N": 1, "E": 2, "S": 3, "W": 4},
        signs=[],
        barriers=[],
        start_x=sx,
        start_y=sy,
        start_heading=wro_field._heading(parking, "CW"),
    )


def _run_corner(giro_rapido_modo: int, seed: int = 42, max_s: float = 25.0) -> CornerRun:
    field = _synthetic_field(seed)
    world = World(field)
    params = TwinParams()
    rng = np.random.default_rng(seed)
    usonic = Ultrasonic(params.ultrasonic, rng)
    gyro_s = Gyro(params.gyro, rng)

    hx = math.radians(field.start_heading)
    fx, fy = math.sin(hx), math.cos(hx)
    x = field.start_x + fx * 350.0
    y = field.start_y + fy * 350.0
    vehicle = Vehicle(params, x, y, field.start_heading)
    pose = Pose(x, y, field.start_heading)

    sim_t = 0.0
    next_v2 = 0.0
    last_gyro_rate = 0.0
    min_clear = 1e9
    heading0 = vehicle.heading_deg
    crucero_t: float | None = None
    corner_done_t: float | None = None
    saw_reverse = False

    overrides = {
        "TURNS_PER_RACE": "12",
        "GIRO_RAPIDO_MODO": str(giro_rapido_modo),
        "piTimeoutMs": "600000",
    }
    fw = FirmwareSIL(overrides=overrides, force_rebuild=True)
    fw.set_param("loop_overhead_us", int(DT_LOOP_S * 1e6))
    fw.set_param("mpu_update_us", 0)
    fw.set_param("serial_us_per_char", 0)
    fw.set_param("serial2_us_per_char", 0)
    fw.set_param("gyro_z_offset", 0.0)

    veh = params.vehicle

    def advance(us: int) -> None:
        nonlocal sim_t, pose, last_gyro_rate, min_clear, next_v2
        dt = us / 1e6
        if dt <= 0:
            return
        info = vehicle.step(
            dt,
            fw.servo_angle(),
            float(fw.motor_pwm()),
            fw.motor_dir(),
        )
        last_gyro_rate = info.yaw_rate_ccw_dps
        pose = Pose(vehicle.x, vehicle.y, vehicle.heading_deg)
        sim_t += dt
        for sid in (0, 1, 2):
            us_val = usonic.echo_us(sid, world, pose)
            min_clear = min(min_clear, _echo_mm(us_val))
        while sim_t >= next_v2:
            fw.push_serial2(V2_LINE, arrival_us=int(sim_t * 1e6))
            next_v2 += V2_PERIOD_S

    def sonar(sid: int) -> int:
        return usonic.echo_us(sid, world, pose)

    def gyro() -> float:
        return gyro_s.read(last_gyro_rate, sim_t)

    fw.set_callbacks(advance=advance, sonar=sonar, gyro=gyro)
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_LINE)

    heading_at_corner = heading0
    steps = int(max_s / DT_LOOP_S)
    tc_done = False
    last_tc = 0
    for _ in range(steps):
        fw.loop()
        if fw.motor_dir() < 0:
            saw_reverse = True
        for _t, ln in fw.pop_serial2_lines():
            if not ln.startswith("ACK:V2,"):
                continue
            d = _parse_ack(ln)
            tc = int(d.get("tc", "0"))
            est = d.get("est", "S")
            if not tc_done and est in ("C", "G") and crucero_t is None:
                crucero_t = sim_t
            if tc >= 1 and not tc_done:
                corner_done_t = sim_t
                heading_at_corner = vehicle.heading_deg
                tc_done = True
            last_tc = max(last_tc, tc)
        if tc_done:
            break

    fw.close()

    dh = ((heading_at_corner - heading0 + 180) % 360) - 180
    ct = None
    if corner_done_t is not None:
        ct = corner_done_t - (crucero_t if crucero_t is not None else 0.0)

    # Heurística: distancia al eje del pasillo (centro del tapete)
    exit_centerline = min(abs(vehicle.x), abs(vehicle.y))
    # Pared exterior aproximada: mínimo eco lateral
    exit_outer = min_clear

    return CornerRun(
        corner_time_s=ct,
        heading_delta_deg=dh,
        min_clearance_mm=min_clear,
        exit_outer_wall_mm=exit_outer,
        exit_centerline_mm=exit_centerline,
        saw_reverse=saw_reverse,
    )


def test_giro_rapido_vs_maniobra(capsys):
    run_a = _run_corner(0)
    run_b = _run_corner(1)

    print(
        f"corner A (MANIOBRA) time={run_a.corner_time_s} s "
        f"outer_echo~={run_a.exit_outer_wall_mm:.0f} mm center~={run_a.exit_centerline_mm:.0f} mm "
        f"min_clear={run_a.min_clearance_mm:.0f} mm dH={run_a.heading_delta_deg:.1f}"
    )
    print(
        f"corner B (GIRO_RAPIDO) time={run_b.corner_time_s} s "
        f"outer_echo~={run_b.exit_outer_wall_mm:.0f} mm center~={run_b.exit_centerline_mm:.0f} mm "
        f"min_clear={run_b.min_clearance_mm:.0f} mm dH={run_b.heading_delta_deg:.1f}"
    )

    for run in (run_a, run_b):
        assert run.min_clearance_mm > 5.0
        assert abs(abs(run.heading_delta_deg) - 90.0) < 15.0

    assert run_a.corner_time_s is not None and run_b.corner_time_s is not None
    assert run_b.corner_time_s <= run_a.corner_time_s + 0.05


def test_giro_rapido_abort_inner_hug():
    """Escenario interior: puede caer en MANIOBRA por abort; no debe colisionar."""
    field = _synthetic_field(7)
    world = World(field)
    params = TwinParams()
    rng = np.random.default_rng(7)
    usonic = Ultrasonic(params.ultrasonic, rng)
    gyro_s = Gyro(params.gyro, rng)

    hx = math.radians(field.start_heading)
    rx, ry = math.cos(hx), -math.sin(hx)
    x = field.start_x + rx * 120.0
    y = field.start_y + ry * 120.0
    vehicle = Vehicle(params, x, y, field.start_heading)
    pose = Pose(x, y, field.start_heading)
    sim_t = 0.0
    next_v2 = 0.0
    last_gyro_rate = 0.0

    fw = FirmwareSIL(
        overrides={
            "TURNS_PER_RACE": "12",
            "GIRO_RAPIDO_MODO": "1",
            "piTimeoutMs": "600000",
        },
        force_rebuild=True,
    )
    fw.set_param("loop_overhead_us", 25000)
    fw.set_param("mpu_update_us", 0)
    fw.set_param("gyro_z_offset", 0.0)

    veh = params.vehicle

    def advance(us: int) -> None:
        nonlocal sim_t, pose, last_gyro_rate, next_v2
        dt = us / 1e6
        if dt <= 0:
            return
        info = vehicle.step(dt, fw.servo_angle(), float(fw.motor_pwm()), fw.motor_dir())
        last_gyro_rate = info.yaw_rate_ccw_dps
        pose = Pose(vehicle.x, vehicle.y, vehicle.heading_deg)
        sim_t += dt
        while sim_t >= next_v2:
            fw.push_serial2(V2_LINE, arrival_us=int(sim_t * 1e6))
            next_v2 += V2_PERIOD_S

    fw.set_callbacks(
        advance=advance,
        sonar=lambda sid: usonic.echo_us(sid, world, pose),
        gyro=lambda: gyro_s.read(last_gyro_rate, sim_t),
    )
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_LINE)

    hit = None
    for _ in range(int(12.0 / 0.025)):
        fw.loop()
        hit = collision(
            world,
            vehicle.x,
            vehicle.y,
            vehicle.heading_deg,
            veh.length_mm,
            veh.width_mm,
            veh.rear_overhang_mm,
        )
        if hit:
            break
    fw.close()
    assert hit is None
