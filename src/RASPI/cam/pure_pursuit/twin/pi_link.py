"""UART Pi↔ESP32 modelado para el twin."""

from __future__ import annotations

import os
import re

from pure_pursuit.twin.firmware.fw import FirmwareSIL

_PARK_PRIOR = re.compile(r",pka=(-?\d+)(?=,|$)")

_UART_US_PER_BYTE = float(os.environ.get("TWIN_PI_UART_US_PER_BYTE", "86.8"))  # 115200 baud


class SimSerialLink:
    """Misma API que wro_runtime.SerialLink para PPRuntime."""

    def __init__(self, fw: FirmwareSIL, *, park_prior_error_mm: int = 0) -> None:
        self._fw = fw
        self._park_prior_error_mm = park_prior_error_mm
        self._pi_now_us: int = 0
        self._tx_free_us: int = 0
        self._pending: list[tuple[int, str]] = []
        self._last_ack: str = ""
        self.ack_times: dict[str, int] = {}   # ACK -> µs de fin de transmisión (últimos pocos; ruta_seed)

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
        from pure_pursuit import config as C
        if self._park_prior_error_mm and ",pkc=1" not in line:
            line = _PARK_PRIOR.sub(
                lambda match: f",pka={int(match[1]) + self._park_prior_error_mm}",
                line,
            )
        if getattr(C, "UART_CHECKSUM", False):   # mismo formato que wro_runtime.con_checksum
            parts = []
            for l in line.split("\n"):
                ck = 0
                for b in l.encode("utf-8"):
                    ck ^= b
                parts.append(f"{l}*{ck:02X}")
            line = "\n".join(parts)
        raw = line if line.endswith("\n") else line + "\n"
        self._tx_free_us = max(self._pi_now_us, self._tx_free_us) + int(
            len(raw.encode("utf-8")) * _UART_US_PER_BYTE
        )
        self._fw.push_serial2(line, arrival_us=self._tx_free_us)

    def try_readline(self) -> str:
        ready = [p for p in self._pending if p[0] <= self._pi_now_us]
        if not ready:
            return ""
        t_done, line = max(ready, key=lambda p: p[0])
        # La Pi conserva solo la última línea recibida entre lecturas.
        self._pending = [p for p in self._pending if p[0] > self._pi_now_us]
        self._last_ack = line
        self.ack_times[line] = t_done
        if len(self.ack_times) > 16:
            del self.ack_times[next(iter(self.ack_times))]
        return line
