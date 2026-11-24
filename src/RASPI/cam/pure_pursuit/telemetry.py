"""Telemetría local del runtime para el dashboard (pure_pursuit/dashboard).

El runtime manda su estado como UN datagrama UDP JSON a 127.0.0.1 (TELE_PORT) y lee
comandos del dashboard por otro puerto UDP local (CMD_PORT). Todo es no bloqueante y
nunca lanza: si el dashboard no corre, los datagramas se pierden y la carrera sigue igual.

Comandos (JSON por datagrama):
  {"esp": "<línea>"}   línea al ESP (solo DESARMADO: CALQ / CALT de la calibración de los ToF)
  {"snap": 1}          guarda el próximo frame crudo de la cámara en RAW_SNAP_PATH (calibración de color)
"""
from __future__ import annotations

import json
import math
import os
import socket
import time

TELE_PORT = int(os.environ.get("FOX_TELE_PORT", "5010"))
CMD_PORT = int(os.environ.get("FOX_CMD_PORT", "5011"))
RAW_SNAP_PATH = os.environ.get("FOX_RAW_SNAP", "/dev/shm/wro_raw.jpg")

# Campos del ACK que se publican tal cual (números o letras cortas).
ACK_KEYS = ("est", "dir", "tc", "tpr", "yaw", "od", "px", "py", "vel", "srv", "pw",
            "tL", "tR", "tB", "tF", "dL", "dR", "dF", "rn", "ue", "uf", "enc", "bno",
            "pkf", "hug", "rst", "tfh", "ltf", "fase", "rev", "lv")


def parse_ack(ack: str | None) -> dict:
    """ACK:V2,k=v,... -> {k: número|texto} solo con ACK_KEYS."""
    out = {}
    if not ack or not ack.startswith("ACK:V2"):
        return out
    for part in ack[7:].split(","):
        k, sep, v = part.partition("=")
        if not sep or k not in ACK_KEYS:
            continue
        try:
            f = float(v)
            out[k] = f if math.isfinite(f) else None
        except ValueError:
            out[k] = v
    return out


def parse_pre(line: str | None) -> dict:
    """'PRE L=.. R=.. B=.. F=.. rl=a/b/c/d ...' del ESP antes del READY -> dict."""
    out = {}
    if not line or not line.startswith("PRE "):
        return out
    for tok in line[4:].split():
        k, sep, v = tok.partition("=")
        if not sep:
            continue
        if "/" in v:
            try:
                out[k] = [float(x) for x in v.split("/")]
            except ValueError:
                out[k] = v
        else:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


class Telemetry:
    def __init__(self, hz: float | None = None):
        self.period = 1.0 / float(hz or os.environ.get("FOX_TELE_HZ", "12"))
        self._t = 0.0
        self._tx = None
        self._rx = None
        try:
            self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._tx.setblocking(False)
            self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx.bind(("127.0.0.1", CMD_PORT))
            self._rx.setblocking(False)
        except OSError as e:
            print(f"[TELE] sin telemetría ({e})", flush=True)
            self._rx = None

    def due(self) -> bool:
        return self._tx is not None and time.monotonic() - self._t >= self.period

    def send(self, state: dict) -> None:
        self._t = time.monotonic()
        try:
            data = json.dumps(state, separators=(",", ":"), default=_jsonable).encode()
            if len(data) < 60000:
                self._tx.sendto(data, ("127.0.0.1", TELE_PORT))
        except Exception:
            pass

    def commands(self) -> list[dict]:
        out = []
        if self._rx is None:
            return out
        for _ in range(8):
            try:
                data, _addr = self._rx.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            try:
                cmd = json.loads(data.decode())
                if isinstance(cmd, dict):
                    out.append(cmd)
            except Exception:
                pass
        return out


def _jsonable(o):
    try:
        import numpy as np
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, np.generic):
            return o.item()
    except Exception:
        pass
    if isinstance(o, (set, tuple)):
        return list(o)
    return str(o)
