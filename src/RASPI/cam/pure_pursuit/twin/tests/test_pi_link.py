from pure_pursuit import config
from pure_pursuit.twin import pi_link


class StubFirmware:
    def __init__(self):
        self.tx = []

    def push_serial2(self, line, arrival_us):
        self.tx.append((line, arrival_us))

    def pop_serial2_lines(self):
        return [(10, "old"), (20, "latest"), (30, "future")]


def test_send_line_serializes_burst(monkeypatch):
    monkeypatch.setattr(config, "UART_CHECKSUM", False)
    monkeypatch.setattr(pi_link, "_UART_US_PER_BYTE", 10)
    fw = StubFirmware()
    link = pi_link.SimSerialLink(fw)

    link.set_now_us(100)
    link.send_line("RTP,12345")
    link.send_line("RTE")
    link.set_now_us(300)
    link.send_line("V2")

    assert fw.tx == [("RTP,12345", 200), ("RTE", 240), ("V2", 330)]


def test_try_readline_keeps_only_latest_ready_ack():
    link = pi_link.SimSerialLink(StubFirmware())
    link.drain_tx()
    link.set_now_us(25)

    assert link.try_readline() == "latest"
    assert link.try_readline() == ""

    link.set_now_us(30)
    assert link.try_readline() == "future"
