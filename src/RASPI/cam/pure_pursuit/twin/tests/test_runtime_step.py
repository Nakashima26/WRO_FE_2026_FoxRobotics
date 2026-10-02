"""Step tests for PPRuntime.process_frame / arm (no camera or UART hardware)."""

import time

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit.bev import BEVTransformer
from pure_pursuit.runtime_nuevo import PPConfig, PPRuntime, FrameResult
from vision import Vision


class FakeSerialLink:
    """Records sent lines; try_readline returns one queued ACK then clears."""

    def __init__(self):
        self.sent: list[str] = []
        self._ack_queue: list[str] = []

    def open(self, timeout=30.0):
        return True

    def close(self):
        pass

    def send_line(self, line: str):
        self.sent.append(line)

    def try_readline(self) -> str:
        if self._ack_queue:
            return self._ack_queue.pop(0)
        return ""

    def queue_ack(self, ack: str):
        self._ack_queue.append(ack)


def _floor_frame(h: int = 480, w: int = 640) -> np.ndarray:
    b, g, r = C.COLOR_CORR_FLOOR_REF_BGR
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :] = (int(b), int(g), int(r))
    return frame


def _make_runtime() -> tuple[PPRuntime, FakeSerialLink]:
    serial = FakeSerialLink()
    rt = PPRuntime(
        PPConfig(show_window=False, record_orillas=False),
        vision=Vision(open_cam=False),
        serial_link=serial,
        bev=BEVTransformer(),
    )
    rt._last_fps_time = time.perf_counter()
    rt._t_prev_end = time.perf_counter()
    return rt, serial


def test_process_frame_disarmed_sends_nothing():
    rt, serial = _make_runtime()
    frame = _floor_frame()
    now = time.perf_counter()
    result = rt.process_frame(frame, now, armed=False)
    assert isinstance(result, FrameResult)
    assert serial.sent == []
    assert result.serial_ack is None


def test_arm_sends_ready_three_times():
    rt, serial = _make_runtime()
    rt.arm(sleep_fn=lambda _s: None)
    assert serial.sent == ["READY", "READY", "READY"]


def test_process_frame_armed_sends_v2():
    rt, serial = _make_runtime()
    frame = _floor_frame()
    now = time.perf_counter()
    result = rt.process_frame(frame, now, armed=True)
    assert len(serial.sent) == 1
    msg = serial.sent[0]
    assert msg.startswith("V2,obs=")
    expected_prefix = (
        "V2,obs=",
        ",turn=0,",
        "state=",
        ",prio=",
        ",mem=",
        ",pp=1,",
        "pasado=",
        ",intr=",
        "inicio=",
        ",park=",
        ",pd=",
    )
    for part in expected_prefix:
        assert part in msg
    assert result.serial_msg == msg


def test_ack_turn_g_debounce_and_recovery():
    rt, serial = _make_runtime()
    frame = _floor_frame()
    now = time.perf_counter()
    g_ack = "ACK:V2,ang=5.0,est=G,dir=L,tc=0,tpr=12"

    rt.process_frame(frame, now, armed=True)
    serial.sent.clear()

    serial.queue_ack(g_ack)
    rt.process_frame(frame, now + 0.05, armed=True)
    assert not rt._is_turning
    assert rt._g_streak == 1

    serial.queue_ack(g_ack)
    rt.process_frame(frame, now + 0.10, armed=True)
    assert rt._is_turning

    serial.queue_ack("ACK:V2,ang=5.0,est=S,dir=L,tc=1,tpr=12")
    rt.process_frame(frame, now + 0.15, armed=True)
    assert not rt._is_turning
    assert rt._turn_recovery_frames == C.TURN_RECOVERY_FRAMES
    assert rt._last_heading == 5.0


def test_chassis_twin_frame_optional():
    try:
        from pure_pursuit.chassis_twin import render_camera
        from pure_pursuit.wro_field import randomize
    except ImportError:
        pytest.skip("chassis_twin / wro_field not available")
    rt, serial = _make_runtime()
    field = randomize(1)
    frame = render_camera(field, 100.0, 200.0, 0.0)
    assert frame is not None and frame.size > 0
    rt.process_frame(frame, time.perf_counter(), armed=False)
    assert serial.sent == []
