"""UART Pi↔ESP32 modelado para el twin."""

from __future__ import annotations

from pure_pursuit.twin.firmware.fw import FirmwareSIL

_UART_US_PER_BYTE = 86.8


class SimSerialLink:
    """Misma API que wro_runtime.SerialLink para PPRuntime."""

    def __init__(self, fw: FirmwareSIL) -> None:
        self._fw = fw
        self._pi_now_us: int = 0
        self._pending: list[tuple[int, str]] = []
        self._last_ack: str = ""

    def set_now_us(self, t_us: int) -> None:
        self._pi_now_us = int(t_us)

    def drain_tx(self) -> None:
        for t_done, line in self._fw.pop_serial2_lines():
            self._pending.append((int(t_done), line.rstrip("\r\n")))

    def open(self, timeout: float = 30.0) -> bool:
        return True

    def close(self) -> None:
        pass

    def send_line(self, line: str) -> None:
        raw = line if line.endswith("\n") else line + "\n"
        arrival = self._pi_now_us + int(len(raw) * _UART_US_PER_BYTE)
        self._fw.push_serial2(line, arrival_us=arrival)

    def try_readline(self) -> str:
        ready = [p for p in self._pending if p[0] <= self._pi_now_us]
        if not ready:
            return ""
        t_done, line = max(ready, key=lambda p: p[0])
        self._pending = [p for p in self._pending if p[0] != t_done]
        self._last_ack = line
        return line
