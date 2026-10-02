"""Tests for firmware SIL harness."""

from __future__ import annotations

import re

import pytest

from pure_pursuit.twin.firmware import build
from pure_pursuit.twin.firmware.fw import FirmwareSIL, FirmwareSetupError

V2_LINE = (
    "V2,obs=+0.350,turn=0,state=pp_follow,prio=0,mem=0,pp=1,"
    "pasado=0,intr=0,inicio=0,park=0,pd=0"
)

SONAR_50CM_US = int(round(2 * 500 / 0.343))  # ~2915 µs


def test_build_default_and_override():
    p1 = build.build(force=True)
    assert p1.is_file()
    p2 = build.build(overrides={"TURNS_PER_RACE": "12"})
    assert p2.is_file()
    assert p1 != p2 or p1.name != p2.name


def test_setup_ready():
    fw = FirmwareSIL()
    fw.push_serial2("READY")
    fw.setup()
    dbg = fw.pop_debug()
    assert "Recibido READY" in dbg
    fw.close()


def test_v2_ack_and_tpr_override():
    fw = FirmwareSIL(overrides={"TURNS_PER_RACE": "12"})
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_LINE)
    for _ in range(30):
        fw.loop()
        lines = fw.pop_serial2_lines()
        for _t, ln in lines:
            if ln.startswith("ACK:V2,ang=") and ",est=S" in ln and "tpr=12" in ln:
                fw.close()
                return
    fw.close()
    pytest.fail("no ACK:V2 with est=S and tpr=12")


def test_sonar_motor_servo():
    fw = FirmwareSIL()

    def sonar(sid: int) -> int:
        if sid in (0, 1):
            return SONAR_50CM_US
        return 0

    fw.set_callbacks(advance=lambda _us: None, sonar=sonar, gyro=lambda: 0.0)
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_LINE)
    for _ in range(80):
        fw.loop()
        if fw.motor_dir() == 1 and fw.motor_pwm() >= 100:
            assert abs(fw.servo_angle() - 90.0) < 15.0
            fw.close()
            return
    fw.close()
    pytest.fail("motor did not go forward with expected PWM")


def test_pp_steer_sign():
    base = (
        "V2,obs={obs},turn=0,state=pp_follow,prio=1,mem=0,pp=1,"
        "pasado=0,intr=0,inicio=0,park=0,pd=0"
    )

    def run_steer(obs: str) -> float:
        fw = FirmwareSIL(overrides={"piTimeoutMs": "600000"})
        fw.set_param("loop_overhead_us", 0)
        fw.set_param("mpu_update_us", 0)
        fw.set_param("serial_us_per_char", 0)
        fw.set_param("serial2_us_per_char", 0)
        fw.set_callbacks(
            advance=lambda _us: None,
            sonar=lambda _i: SONAR_50CM_US,
            gyro=lambda: 0.0,
        )
        fw.push_serial2("READY")
        fw.setup()
        fw.push_serial2(base.format(obs=obs))
        fw.loop()
        ang = fw.servo_angle()
        fw.close()
        return ang

    assert run_steer("+0.500") < 89.0
    assert run_steer("-0.500") > 91.0


def test_average_loop_period(capsys):
    fw = FirmwareSIL()
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_LINE)
    t0 = fw.now_us
    n = 50
    for _ in range(n):
        fw.loop()
    dt_ms = (fw.now_us - t0) / 1000.0 / n
    print(f"average_loop_period_ms={dt_ms:.3f}")
    fw.close()
    assert 1.0 < dt_ms < 200.0


def test_two_instances_isolated():
    fw1 = FirmwareSIL()
    fw1.push_serial2("READY")
    fw1.setup()
    fw1.push_serial2(V2_LINE)
    fw1.loop()
    assert "Recibido READY" in fw1.pop_debug()

    fw2 = FirmwareSIL()
    fw2.set_param("setup_timeout_ms", 500.0)
    with pytest.raises(FirmwareSetupError):
        fw2.setup()
    fw2.push_serial2("READY")
    fw2.setup()
    assert "Recibido READY" in fw2.pop_debug()
    fw2.close()
    fw1.close()
