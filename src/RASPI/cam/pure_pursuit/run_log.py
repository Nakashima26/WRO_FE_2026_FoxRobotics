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


class _Disk:
    """Hilo que hace TODA la escritura a disco/journal de la corrida (orillas263/266/270).

    Con el SD ocupado (video + BEV + journal) un write() de 1.5 kB se bloquea hasta 4 s (medido en la
    Pi 5, dd de 400 MB en paralelo). Escrito desde el loop de control, la Pi dejaba de mandar a la ESP
    2-3 s: la ESP soltaba la ruta y se iba recto (270 contra la pared E, 266 a PWM 255 en la recta E).
    Aquí el loop solo encola; el hilo escribe. Si el disco no avanza, la cola se recorta (se pierde log,
    nunca control)."""

    MAX_ITEMS = 50000

    def __init__(self, f, orig):
        import queue
        self.f, self.orig = f, orig
        self.q = queue.SimpleQueue()
        self.n = 0
        self.dropped = 0
        self.t = threading.Thread(target=self._run, name="runlog-disk", daemon=True)
        self.t.start()

    def put(self, kind, data=None):
        if self.n > self.MAX_ITEMS and kind == "o":
            self.dropped += 1
            return
        self.n += 1
        self.q.put((kind, data))

    def _run(self):
        n_f = 0
        while True:
            kind, data = self.q.get()
            self.n -= 1
            try:
                if kind == "f":
                    if self.f is None:
                        continue
                    self.f.write(data)
                    n_f += 1
                    if n_f % 24 == 0:   # ~2 s: si matan el servicio a media corrida queda casi todo
                        self.f.flush()
                elif kind == "o":
                    self.orig.write(data)
                    if self.q.empty():
                        self.orig.flush()
                elif kind == "of":
                    self.orig.flush()
                elif kind == "close":
                    # Solo el archivo: el hilo sigue sacando los prints (el carro aún estaciona tras cortar).
                    f, self.f = self.f, None
                    f.close()
                    self.orig.flush()
            except Exception:
                pass


class _Tee:
    """stdout que además guarda las líneas con EV_TAGS (mismo texto que va al journal).
    La escritura real al journal la hace el hilo _Disk (journald también escribe al SD)."""

    def __init__(self, orig, sink: list, lock: threading.Lock, disk: "_Disk"):
        self.orig, self.sink, self.lock, self.buf, self.disk = orig, sink, lock, "", disk

    def write(self, s):
        self.disk.put("o", s)
        if self.sink is None:   # registro cerrado: solo journal (por el hilo)
            return len(s)
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
        return len(s)

    def flush(self):
        self.disk.put("of")

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
        prev = sys.stdout
        self._orig = prev.orig if isinstance(prev, _Tee) else prev   # corrida anterior: su Tee queda sin uso
        self._disk = _Disk(self.f, self._orig)
        hdr = {"hdr": 1, "t": round(time.time(), 3)}
        hdr.update(header)
        self._disk.put("f", json.dumps(hdr, default=_jsonable) + "\n")
        sys.stdout = _Tee(self._orig, self.ev, self.lock, self._disk)

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
            self._disk.put("f", json.dumps(rec, separators=(",", ":"), default=_jsonable) + "\n")
            self._n += 1
        except Exception as e:
            self.close()
            print(f"[REC] registro de la corrida apagado: {e}", flush=True)

    def close(self) -> None:
        # stdout se queda en el Tee (sin capturar eventos): los prints siguen yendo al journal por el hilo.
        if isinstance(sys.stdout, _Tee) and sys.stdout.disk is self._disk:
            with self.lock:
                sys.stdout.sink = None
        if self.f is not None:
            self._disk.put("close")   # el hilo vacía la cola y cierra; el loop no espera al disco
            self.f = None
