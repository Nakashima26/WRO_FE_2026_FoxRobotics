"""Registro de cada corrida para el dashboard (pestaña Corridas).

Mientras se graba el .avi, el runtime escribe junto a él <video>_tele.jsonl:
  1ra línea  {"hdr": 1, "t": <epoch>, "video": "orillasN.avi", ...}
  luego      {"t": <epoch>, "s": <estado del dashboard>, "ev": [[t, línea], ...], "esp": [[t, línea], ...]}
"s" es el mismo estado que va por UDP al dashboard (telemetry), así el replay usa el mismo dibujo
que "En vivo". "route" solo va cuando cambia. "ev" son los prints importantes del runtime
(se copian de stdout) y "esp" las líneas del ESP que no son ACK/PRE.
Nada de esto puede tumbar la carrera: cualquier error apaga el registro y ya.
"""
from __future__ import annotations

import json
import sys
import threading
import time

from .telemetry import _jsonable

EV_TAGS = ("[ROUTE]", "[PARK", "[INICIO]", "[REC]", "[FIXVOTE]", "[DMAP]", "[CAM]", "[carrera]",
           "[LOCUPD]", "[LOC]", "[TURNDIR]", "[LINEUP]", "[MEMREJ]", "[PROVBLOCK]", "[TELE]", "[GPIO]",
           "[CALIB]", "[INFO]")
EV_ANY = ("AVISO", "Traceback", "Error")


class _Tee:
    """stdout que además guarda las líneas con EV_TAGS (mismo texto que va al journal)."""

    def __init__(self, orig, sink: list, lock: threading.Lock):
        self.orig, self.sink, self.lock, self.buf = orig, sink, lock, ""

    def write(self, s):
        n = self.orig.write(s)
        try:
            with self.lock:
                self.buf += s
                if "\n" in self.buf:
                    *done, self.buf = self.buf.split("\n")
                    now = time.time()
                    for line in done:
                        if line.startswith(EV_TAGS) or any(k in line for k in EV_ANY):
                            self.sink.append([round(now, 3), line[:500]])
                if len(self.buf) > 4000:
                    self.buf = ""
        except Exception:
            pass
        return n

    def flush(self):
        self.orig.flush()

    def __getattr__(self, a):
        return getattr(self.orig, a)


class RunLog:
    def __init__(self, path, header: dict):
        self.path = path
        self.f = open(path, "w", encoding="utf-8")
        self.lock = threading.Lock()
        self.ev: list = []
        self.esp: list = []
        self._route = None
        self._n = 0
        self._orig = sys.stdout
        hdr = {"hdr": 1, "t": round(time.time(), 3)}
        hdr.update(header)
        self.f.write(json.dumps(hdr, default=_jsonable) + "\n")
        sys.stdout = _Tee(self._orig, self.ev, self.lock)

    def add_esp(self, items) -> None:
        with self.lock:
            self.esp.extend(items)

    def write(self, st: dict) -> None:
        if self.f is None:
            return
        try:
            s = dict(st)
            s.pop("esp_rx", None)
            r = s.get("route")
            if r is not None:
                if r == self._route:
                    del s["route"]
                else:
                    self._route = r
            with self.lock:
                ev, self.ev[:] = list(self.ev), []
                esp, self.esp = self.esp, []
            rec = {"t": round(float(st.get("t", time.time())), 3), "s": s}
            if ev:
                rec["ev"] = ev
            if esp:
                rec["esp"] = esp
            self.f.write(json.dumps(rec, separators=(",", ":"), default=_jsonable) + "\n")
            self._n += 1
            if self._n % 24 == 0:   # ~2 s: si matan el servicio a media corrida queda casi todo
                self.f.flush()
        except Exception as e:
            self.close()
            print(f"[REC] registro de la corrida apagado: {e}", flush=True)

    def close(self) -> None:
        if sys.stdout is not self._orig and isinstance(sys.stdout, _Tee):
            sys.stdout = self._orig
        if self.f is not None:
            try:
                self.f.close()
            except Exception:
                pass
            self.f = None
