"""Firmware odometry, yaw, encoder and ToF ACK fields (SIL)."""

from __future__ import annotations

import re
import subprocess
import sys

import pytest

from pure_pursuit.twin.firmware.fw import FirmwareSIL

V2_PP = (
    "V2,obs=+0.000,turn=0,state=pp_follow,prio=0,mem=0,pp=1,"
    "pasado=0,intr=0,inicio=0,park=0,pd=0"
)

SONAR_50CM_US = int(round(2 * 500 / 0.343))


def _latest_ack(fw: FirmwareSIL) -> str | None:
    ack = None
    for _t, ln in fw.pop_serial2_lines():
        if ln.startswith("ACK:V2,"):
            ack = ln
    return ack


def test_defaults_ack_odom_fields():
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "pure_pursuit/twin/tests/test_firmware_sil.py",
            "-q",
        ],
        cwd=str(__import__("pathlib").Path(__file__).resolve().parents[3]),
        check=True,
    )

    fw = FirmwareSIL()
    fw.set_callbacks(
        advance=lambda _us: None,
        sonar=lambda sid: SONAR_50CM_US if sid in (0, 1) else 0,
        gyro=lambda: 0.0,
    )
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_PP)
    for _ in range(5):
        fw.loop()
    ack = _latest_ack(fw)
    fw.close()
    assert ack is not None
    assert ",yaw=" in ack
    assert ",od=" in ack
    assert ",px=" in ack
    assert ",py=" in ack
    assert ",enc=0" in ack
    assert ",tL=-1" in ack
    assert ",tF=" not in ack  # FOX_TOF=0: ACK igual al del carro actual


def test_pwm_odom_and_yaw_integration():
    gyro_rate = 15.0  # deg/s CCW
    fw = FirmwareSIL(overrides={"piTimeoutMs": "600000", "MOTOR_MAX": "120"})
    fw.set_param("loop_overhead_us", 20000)
    fw.set_param("mpu_update_us", 5000)
    fw.set_param("serial_us_per_char", 0)
    fw.set_param("serial2_us_per_char", 0)

    def sonar(sid: int) -> int:
        if sid in (0, 1):
            return SONAR_50CM_US
        return 0

    fw.set_callbacks(advance=lambda _us: None, sonar=sonar, gyro=lambda: gyro_rate)
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_PP)

    px0 = py0 = od0 = yaw0 = None
    for i in range(200):
        fw.push_serial2(V2_PP)
        fw.loop()
        ack = _latest_ack(fw)
        if not ack:
            continue
        m = re.search(r",od=(-?\d+),px=(-?\d+),py=(-?\d+),enc=\d+,tL=", ack)
        if not m:
            continue
        od = int(m.group(1))
        px = int(m.group(2))
        py = int(m.group(3))
        yaw_m = re.search(r",yaw=([-+0-9.]+)", ack)
        yaw = float(yaw_m.group(1)) if yaw_m else 0.0
        if px0 is None:
            px0, py0, od0, yaw0 = px, py, od, yaw
        if od > od0 + 50 and px > px0 + 30:
            assert abs(py - py0) < 25
            break
    else:
        fw.close()
        pytest.fail("od/px did not grow as expected with forward PWM odom")

    assert yaw > yaw0 + 0.5
    fw.close()


def test_encoder_odom():
    counts_per_mm = 20.73
    target_mm = 500.0
    target_counts = int(round(target_mm * counts_per_mm))

    fw = FirmwareSIL(defines={"FOX_ENCODER": "1"}, force_rebuild=True)
    fw.set_param("loop_overhead_us", 0)
    fw.set_param("mpu_update_us", 0)

    enc = [0]

    def encoder() -> int:
        return enc[0]

    fw.set_callbacks(
        advance=lambda _us: None,
        sonar=lambda _i: SONAR_50CM_US,
        gyro=lambda: 0.0,
        encoder=encoder,
    )
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_PP)

    enc[0] = target_counts
    for _ in range(5):
        fw.push_serial2(V2_PP)
        fw.loop()
    ack = _latest_ack(fw)
    fw.close()
    assert ack is not None
    assert ",enc=1" in ack
    od_m = re.search(r",od=(\d+)", ack)
    assert od_m is not None
    od = int(od_m.group(1))
    assert abs(od - target_mm) / target_mm < 0.01


def test_tof_ack():
    fw = FirmwareSIL(defines={"FOX_TOF": "1"}, force_rebuild=True)

    def tof(idx: int) -> int:
        return (100, 200, 300, 400)[idx]

    fw.set_callbacks(
        advance=lambda _us: None,
        sonar=lambda _i: SONAR_50CM_US,
        gyro=lambda: 0.0,
        tof=tof,
    )
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_PP)
    fw.push_serial2(V2_PP)
    fw.loop()
    ack = _latest_ack(fw)
    fw.close()
    assert ack is not None
    assert ",tL=100" in ack
    assert ",tR=200" in ack
    assert ",tB=300" in ack
    assert ",tF=400" in ack


def test_encoder_odom_sub_mm_ticks_accumulate():
    """Ticks de <0.5 mm: con lroundf por tick od/px se quedaban en 0."""
    counts_per_mm = 20.73
    fw = FirmwareSIL(defines={"FOX_ENCODER": "1"})
    fw.set_param("loop_overhead_us", 20000)
    fw.set_param("mpu_update_us", 0)
    enc = [0]
    fw.set_callbacks(
        advance=lambda _us: None,
        sonar=lambda _i: SONAR_50CM_US,
        gyro=lambda: 0.0,
        encoder=lambda: enc[0],
    )
    fw.push_serial2("READY")
    fw.setup()
    fw.push_serial2(V2_PP)
    for _ in range(400):
        enc[0] += 6  # 0.29 mm por loop
        fw.push_serial2(V2_PP)
        fw.loop()
    ack = _latest_ack(fw)
    fw.close()
    assert ack is not None
    od = int(re.search(r",od=(-?\d+)", ack).group(1))
    px = int(re.search(r",px=(-?\d+)", ack).group(1))
    expect = enc[0] / counts_per_mm
    assert abs(od - expect) <= 2.0
    assert abs(px - expect) <= 2.0
