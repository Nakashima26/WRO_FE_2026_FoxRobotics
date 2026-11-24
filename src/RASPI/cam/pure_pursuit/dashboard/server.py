"""Dashboard del carro: http://<Pi>:8080  (sin internet: todo sale de la Pi, sin CDN).

Proceso APARTE del runtime (systemd wro-dashboard.service): si se cae, la carrera sigue.
Recibe el estado del runtime por UDP local (pure_pursuit/telemetry.py) y le manda comandos
por el otro puerto. Solo librería estándar + numpy/cv2 (ya están en el venv de la Pi).

  cd ~/FoxRobotics/src/RASPI/cam && ~/FoxRobotics/.venv/bin/python -m pure_pursuit.dashboard.server

Rondas oficiales: Wi-Fi apagado (regla 11.10). El botón "Modo ronda" apaga los radios.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

from .. import calib_file
from . import runs
from ..telemetry import CMD_PORT, RAW_SNAP_PATH, TELE_PORT

HERE = Path(__file__).resolve().parent
STATIC = HERE / "static"
REPO = HERE.parents[4]                      # .../FoxRobotics
CAM_FRAME = os.environ.get("FOX_CAM_FRAME", "/tmp/wro_cam_frame.jpg")
VIDEOS = Path(os.environ.get("FOX_VIDEOS", str(REPO / "videos_orillas")))
SERVICE = os.environ.get("FOX_SERVICE", "wro-runtime")
ON_PI = sys.platform.startswith("linux")


# ── Estado del runtime (UDP) ─────────────────────────────────────────────────
class Telemetry:
    def __init__(self):
        self.state: dict = {}
        self.t_rx = 0.0
        self.seq = 0
        self.esp_rx: list[tuple[float, str]] = []      # respuestas CAL* del ESP
        self.pre_hist: list[tuple[float, dict]] = []   # PRE recientes (calibración de ToF)
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", TELE_PORT))
        self.cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                st = json.loads(data.decode())
            except Exception:
                continue
            now = time.time()
            with self.lock:
                self.state = st
                self.t_rx = now
                self.seq += 1
                for line in st.get("esp_rx") or []:
                    self.esp_rx.append((now, line))
                self.esp_rx = self.esp_rx[-30:]
                pre = st.get("pre")
                if pre and (not self.pre_hist or self.pre_hist[-1][1] != pre):
                    self.pre_hist.append((now, pre))
                    self.pre_hist = [p for p in self.pre_hist if now - p[0] < 20.0]

    def send(self, cmd: dict) -> None:
        self.cmd.sendto(json.dumps(cmd).encode(), ("127.0.0.1", CMD_PORT))

    def snapshot(self) -> dict:
        with self.lock:
            st = dict(self.state)
            st["_age"] = round(time.time() - self.t_rx, 2) if self.t_rx else None
            st["_seq"] = self.seq
            return st


TELE: Telemetry | None = None


# ── Sistema ──────────────────────────────────────────────────────────────────
def _run(cmd: list[str], timeout: float = 5.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except Exception as e:
        return f"ERR {e}"


_SYS = {"t": 0.0, "d": {}}


def system_info() -> dict:
    if time.time() - _SYS["t"] < 2.0:
        return _SYS["d"]
    d: dict = {"on_pi": ON_PI}
    try:
        d["temp_c"] = round(int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0, 1)
    except Exception:
        d["temp_c"] = None
    if ON_PI:
        thr = _run(["vcgencmd", "get_throttled"])
        try:
            d["throttled"] = int(thr.split("=")[1], 16)
        except Exception:
            d["throttled"] = None
        d["service"] = _run(["systemctl", "is-active", SERVICE])
        d["service_since"] = _run(["systemctl", "show", SERVICE, "-p", "ActiveEnterTimestamp", "--value"])
        d["ips"] = _run(["hostname", "-I"]).split()
        d["radios"] = _radios() or None
        d["wifi_ssid"] = _run(["iwgetid", "-r"]) or None
    try:
        d["load"] = [round(v, 2) for v in os.getloadavg()]
    except Exception:
        d["load"] = None
    try:
        du = shutil.disk_usage(str(REPO))
        d["disk_free_gb"] = round(du.free / 1e9, 1)
    except Exception:
        d["disk_free_gb"] = None
    d["git"] = _run(["git", "-C", str(REPO), "log", "-1", "--format=%h %s"])[:90]
    d["calib"] = str(calib_file.path()) if calib_file.path().is_file() else None
    _SYS.update(t=time.time(), d=d)
    return d


def service_action(action: str) -> str:
    """start / stop / restart. NUNCA `systemctl restart` (deja la cámara CSI trabada): stop + 4 s + start."""
    if not ON_PI:
        return "no es la Pi"
    sc = ["sudo", "-n", "systemctl"]
    if action == "stop":
        return _run(sc + ["stop", SERVICE], 20) or "ok"
    if action == "start":
        return _run(sc + ["start", SERVICE], 20) or "ok"
    if action == "restart":
        def bg():
            _run(sc + ["stop", SERVICE], 20)
            time.sleep(4)
            _run(sc + ["start", SERVICE], 20)
        threading.Thread(target=bg, daemon=True).start()
        return "reiniciando (stop, 4 s, start)"
    return "acción desconocida"


CARRERA_JSON = HERE.parent / "carrera.json"


def surprise_get() -> bool | None:
    """SURPRISE_RULE de carrera.json (lo que cargará el runtime al reiniciar). None si no se puede leer."""
    try:
        return bool(json.loads(CARRERA_JSON.read_text(encoding="utf-8")).get("SURPRISE_RULE", False))
    except Exception:
        return None


def surprise_set(on: bool) -> str:
    """Cambia SURPRISE_RULE en carrera.json sin reformatear el archivo y reinicia el runtime.

    En la Pi esa línea es un cambio local (no está en git): `git stash; pull; stash pop` la conserva."""
    import re
    txt = CARRERA_JSON.read_text(encoding="utf-8")
    val = "true" if on else "false"
    if re.search(r'"SURPRISE_RULE"\s*:\s*(true|false)', txt):
        txt = re.sub(r'("SURPRISE_RULE"\s*:\s*)(true|false)', r"\g<1>" + val, txt, count=1)
    else:
        txt = txt.replace("{", '{\n "SURPRISE_RULE": ' + val + ",", 1)
    json.loads(txt)   # que siga siendo JSON válido antes de escribir
    tmp = CARRERA_JSON.with_suffix(".tmp")
    tmp.write_text(txt, encoding="utf-8")
    os.replace(tmp, CARRERA_JSON)
    return f"regla sorpresa {'ON' if on else 'OFF'} en carrera.json; " + service_action("restart")


def _radios() -> dict:
    """{wlan: encendido, bluetooth: encendido} leyendo /sys/class/rfkill."""
    out = {}
    for dev in Path("/sys/class/rfkill").glob("rfkill*"):
        try:
            typ = (dev / "type").read_text().strip()
            out[typ] = (dev / "soft").read_text().strip() == "0" and (dev / "hard").read_text().strip() == "0"
        except OSError:
            pass
    return out


def radio_action(on: bool) -> str:
    if not ON_PI:
        return "no es la Pi"
    # Por sysfs (la Pi no trae el binario rfkill): soft=1 bloquea.
    out = ""
    for dev in sorted(Path("/sys/class/rfkill").glob("rfkill*")):
        r = subprocess.run(["sudo", "-n", "tee", str(dev / "soft")], input="0" if on else "1",
                           capture_output=True, text=True, timeout=5)
        if r.returncode:
            out += (r.stderr or "error").strip() + " "
    _SYS["t"] = 0.0
    return out or "ok"


def logs(n: int, filt: str) -> str:
    if not ON_PI:
        return "(no es la Pi)"
    txt = _run(["journalctl", "-u", SERVICE, "-n", str(max(50, min(n * 8, 20000))), "--no-pager", "-o", "cat"], 10)
    lines = txt.splitlines()
    if filt == "importante":
        keep = ("[ROUTE]", "[LOCUPD]", "[DMAP]", "[FIXVOTE]", "[INICIO]", "[CALIB]", "[carrera]", "[CAM]",
                "[TELE]", "[REC]", "[INFO]", "[PARKREF]", "Traceback", "Error", "AVISO", "PARK", "CAL")
        lines = [l for l in lines if any(k in l for k in keep) and "[SERIAL] TX" not in l]
    elif filt:
        lines = [l for l in lines if filt in l]
    return "\n".join(lines[-n:])


# ── Geometría de la pista (para el mapa) ─────────────────────────────────────
def field(parking: str | None, w0: float | None) -> dict:
    from .. import config as C
    from ..wro_field import (INNER_HALF_MM, OUTER_HALF_MM, SEATS, SIGN_MM, _barrier_corners,
                             blue_segments, orange_segments, section_to_world)
    seats = {}
    for sec in ("N", "E", "S", "W"):
        for seat, (h, w) in SEATS.items():
            x, y = section_to_world(sec, h, w)
            seats[f"{sec}/{seat}"] = [round(float(x), 1), round(float(y), 1)]
    barriers = []
    if parking:
        try:
            barriers = [[[round(float(x), 1), round(float(y), 1)] for x, y in box]
                        for box in _barrier_corners(parking, float(w0 or 1000.0))]
        except Exception:
            barriers = []
    return {
        "outer": OUTER_HALF_MM, "inner": INNER_HALF_MM, "sign": SIGN_MM,
        "seats": seats,
        "orange": [[list(map(float, a)), list(map(float, b))] for a, b in orange_segments()],
        "blue": [[list(map(float, a)), list(map(float, b))] for a, b in blue_segments()],
        "barriers": barriers,
        "mounts": {k: list(v) for k, v in getattr(C, "SENSOR_MOUNTS", {}).items()},
        "car": {"len": float(getattr(C, "ROBOT_LENGTH_MM", 176.0)), "width": float(getattr(C, "ROBOT_WIDTH_MM", 125.0)),
                "rear_overhang": float(getattr(C, "REAR_OVERHANG_MM", 21.8))},
        "car_top": car_top(),
    }


_CAR_TOP: list = []


def car_top() -> list:
    """Vista de arriba del modelo 3D del carro (la del gemelo, twin/assets/car_top.json):
    piezas [{pts [[derecha, adelante] mm desde el eje trasero], rgb}] de abajo hacia arriba."""
    if not _CAR_TOP:
        try:
            src = Path(__file__).resolve().parents[1] / "twin" / "assets" / "car_top.json"
            for part in json.loads(src.read_text()):
                b, g, r = part["bgr"]
                _CAR_TOP.append({"pts": part["pts"], "rgb": f"rgb({r},{g},{b})"})
        except Exception:
            pass
    return _CAR_TOP


# ── Calibración de color ─────────────────────────────────────────────────────
def _hsv_img():
    import cv2
    img = cv2.imread(RAW_SNAP_PATH)
    if img is None:
        raise FileNotFoundError("sin foto: toma una primero")
    return img, cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def _region(arr, r, shape):
    h, w = shape[:2]
    x0 = int(max(0, min(w - 1, r["x"]))); y0 = int(max(0, min(h - 1, r["y"])))
    x1 = int(max(x0 + 1, min(w, r["x"] + r["w"]))); y1 = int(max(y0 + 1, min(h, r["y"] + r["h"])))
    return arr[y0:y1, x0:x1].reshape(-1, arr.shape[2])


def color_analyze(rects: list[dict]) -> dict:
    img, hsv = _hsv_img()
    out = []
    for r in rects:
        px = _region(hsv, r, hsv.shape).astype(np.float32)
        bgr = _region(img, r, img.shape).astype(np.float32)
        if px.size == 0:
            continue
        hcol = px[:, 0]
        # Rojo cruza 0/180: percentiles sobre H rotado si la mayoría está en los extremos.
        wrap = float(np.mean((hcol < 20) | (hcol > 160))) > 0.6
        hr = np.where(hcol > 90, hcol - 180, hcol) if wrap else hcol
        pc = lambda a, q: float(np.percentile(a, q))
        b, g, rr = (np.median(bgr[:, i]) for i in range(3))
        out.append({
            "label": r.get("label"), "n": int(px.shape[0]), "wrap": wrap,
            "H": [round(pc(hr, 5), 1), round(pc(hr, 50), 1), round(pc(hr, 95), 1)],
            "S": [round(pc(px[:, 1], 5)), round(pc(px[:, 1], 50)), round(pc(px[:, 1], 95))],
            "V": [round(pc(px[:, 2], 5)), round(pc(px[:, 2], 50)), round(pc(px[:, 2], 95))],
            "BGR": [round(float(b)), round(float(g)), round(float(rr))],
            "GR": round(float(g / max(rr, 1)), 2), "BR": round(float(b / max(rr, 1)), 2),
            "BG": round(float(b / max(g, 1)), 2),
        })
    return {"regions": out}


def _ranges_from(colors_cfg: dict, name: str):
    """Rangos [(lo,hi)...] de `colors` (JSON) o los de vision.py vigentes."""
    if name in colors_cfg:
        return [(np.array(lo), np.array(hi)) for lo, hi in colors_cfg[name]]
    import vision
    return [(lo, hi) for lo, hi in vision.COLOR_RANGES.get(name, [])]


def color_masks(colors: dict, rects: list[dict]) -> tuple[bytes, dict]:
    """Imagen con las máscaras pintadas + % detectado de cada color en cada región."""
    import cv2
    img, hsv = _hsv_img()
    paint = {"Red": (0, 0, 255), "Green": (0, 200, 0)}
    over = img.copy()
    stats = {}
    for name, col in paint.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in _ranges_from(colors, name):
            m |= cv2.inRange(hsv, np.asarray(lo, np.uint8), np.asarray(hi, np.uint8))
        over[m > 0] = (0.35 * over[m > 0] + 0.65 * np.array(col)).astype(np.uint8)
        for r in rects:
            sub = _region(m[..., None], r, m.shape + (1,))
            stats.setdefault(r.get("label") or "?", {})[name] = round(100.0 * float(np.mean(sub > 0)), 1) if sub.size else None
    ok, buf = cv2.imencode(".jpg", over, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return buf.tobytes(), stats


# ── HTTP ─────────────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code: int, body: bytes, ctype: str, extra: dict | None = None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200):
        self._send(code, json.dumps(obj, ensure_ascii=False, default=str).encode(), "application/json; charset=utf-8")

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode())
        except Exception:
            return {}

    # GET
    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        p = u.path
        try:
            if p in ("/", "/index.html"):
                return self._static("index.html")
            if p.startswith("/static/"):
                return self._static(p[len("/static/"):])
            if p == "/api/state":
                return self._json({"tele": TELE.snapshot(), "sys": system_info()})
            if p == "/api/stream":
                return self._stream()
            if p == "/api/hud.mjpg":
                return self._mjpeg(CAM_FRAME, float(q.get("fps", 8)))
            if p == "/api/raw.jpg":
                return self._file(RAW_SNAP_PATH, "image/jpeg")
            if p == "/api/field":
                return self._json(field(q.get("parking"), float(q["w0"]) if q.get("w0") else None))
            if p == "/api/logs":
                return self._send(200, logs(int(q.get("n", 200)), q.get("f", "")).encode(), "text/plain; charset=utf-8")
            if p == "/api/calib":
                return self._json({"path": str(calib_file.path()), "data": calib_file.load()})
            if p == "/api/surprise":
                return self._json({"on": surprise_get()})
            if p == "/api/esp_rx":
                with TELE.lock:
                    return self._json({"lines": [[round(t, 2), l] for t, l in TELE.esp_rx]})
            if p == "/api/pre":
                since = float(q.get("since", 0))
                with TELE.lock:
                    return self._json({"pre": [[round(t, 2), d] for t, d in TELE.pre_hist if t > since]})
            if p == "/api/runs":
                return self._json(runs.list_runs(VIDEOS))
            if p.startswith("/api/runs/") and p.endswith("/timeline"):
                name = p[len("/api/runs/"):-len("/timeline")]
                try:
                    return self._json(runs.load(VIDEOS, name))
                except Exception as e:
                    return self._json({"error": str(e)}, 404)
            if p.startswith("/api/runs/") and p.endswith(("/acks", "/log")):
                name, _sp, kind = p[len("/api/runs/"):].rpartition("/")
                rango = dict(desde=float(q.get("t0", -1e9)), hasta=float(q.get("t1", 1e9)))
                try:
                    if kind == "acks":
                        txt = runs.acks_tsv(VIDEOS, name, q.get("f", runs.ACK_CAMPOS), paso=float(q.get("paso", 0)),
                                            cambio=q.get("cambio", ""), **rango)
                    else:
                        txt = runs.log_grep(VIDEOS, name, q.get("q", r"\[PARK|\[ROUTE\] plan"),
                                            maxc=int(q.get("c", 240)), n=int(q.get("n", 400)), **rango)
                except Exception as e:
                    return self._send(404, str(e).encode(), "text/plain; charset=utf-8")
                return self._send(200, txt.encode(), "text/plain; charset=utf-8")
            if p.startswith("/api/runs/") and p.endswith("/frame"):
                name = p[len("/api/runs/"):-len("/frame")]
                jpg = runs.frame_jpeg(VIDEOS, name, int(float(q.get("i", 0))), int(q.get("w", 640)))
                if jpg is None:
                    return self._send(404, b"sin frame", "text/plain")
                return self._send(200, jpg, "image/jpeg")
            if p.startswith("/api/runs/"):
                name = Path(p[len("/api/runs/"):]).name
                return self._file(str(VIDEOS / name), "application/octet-stream",
                                  {"Content-Disposition": f'attachment; filename="{name}"'})
            return self._send(404, b"no", "text/plain")
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            try:
                self._json({"error": str(e)}, 500)
            except Exception:
                pass

    # POST
    def do_POST(self):
        p = urlparse(self.path).path
        b = self._body()
        try:
            if p == "/api/service":
                return self._json({"msg": service_action(str(b.get("action")))})
            if p == "/api/radio":
                return self._json({"msg": radio_action(bool(b.get("on")))})
            if p == "/api/surprise":
                return self._json({"msg": surprise_set(bool(b.get("on"))), "on": surprise_get()})
            if p == "/api/esp":
                line = str(b.get("line", ""))
                if not line.startswith(("CALQ", "CALT")):
                    return self._json({"error": "solo CALQ / CALT"}, 400)
                TELE.send({"esp": line})
                return self._json({"msg": "enviado"})
            if p == "/api/snap":
                t0 = os.path.getmtime(RAW_SNAP_PATH) if os.path.exists(RAW_SNAP_PATH) else 0
                TELE.send({"snap": 1})
                for _ in range(30):
                    time.sleep(0.1)
                    if os.path.exists(RAW_SNAP_PATH) and os.path.getmtime(RAW_SNAP_PATH) > t0:
                        return self._json({"ok": True})
                return self._json({"error": "el runtime no respondió (¿está corriendo?)"}, 504)
            if p == "/api/color/analyze":
                return self._json(color_analyze(b.get("rects") or []))
            if p == "/api/color/preview":
                jpg, stats = color_masks(b.get("colors") or {}, b.get("rects") or [])
                import base64
                return self._json({"img": "data:image/jpeg;base64," + base64.b64encode(jpg).decode(), "stats": stats})
            if p == "/api/calib":
                d = b.get("data")
                if not isinstance(d, dict):
                    return self._json({"error": "data"}, 400)
                path = calib_file.save(d)
                _SYS["t"] = 0.0
                return self._json({"msg": f"guardado en {path}. Reinicia el servicio para aplicarlo."})
            if p == "/api/calib/undo":
                d = calib_file.load()
                hist = d.get("_hist") or []
                if not hist:
                    return self._json({"error": "no hay versión anterior"}, 400)
                prev = dict(hist[0]); prev["_hist"] = hist[1:]
                tmp = calib_file.path().with_suffix(".tmp")
                tmp.write_text(json.dumps(prev, indent=1, ensure_ascii=False), encoding="utf-8")
                os.replace(tmp, calib_file.path())
                return self._json({"msg": "restaurada la versión anterior. Reinicia el servicio para aplicarla."})
            return self._send(404, b"no", "text/plain")
        except Exception as e:
            self._json({"error": str(e)}, 500)

    # helpers
    def _static(self, name: str):
        f = (STATIC / name).resolve()
        if STATIC not in f.parents or not f.is_file():
            return self._send(404, b"no", "text/plain")
        ctype = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml"}.get(f.suffix, "application/octet-stream")
        return self._send(200, f.read_bytes(), ctype)

    def _file(self, path: str, ctype: str, extra: dict | None = None):
        if not os.path.isfile(path):
            return self._send(404, b"no", "text/plain")
        size = os.path.getsize(path)
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(size))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        with open(path, "rb") as fh:
            shutil.copyfileobj(fh, self.wfile, 1 << 16)

    def _stream(self):
        """Server-Sent Events: estado del runtime ~10 Hz + sistema."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        last = -1
        while True:
            st = TELE.snapshot()
            if st["_seq"] != last or int(time.time() * 2) % 2 == 0:
                last = st["_seq"]
                msg = json.dumps({"tele": st, "sys": system_info()}, default=str)
                self.wfile.write(f"data: {msg}\n\n".encode())
                self.wfile.flush()
            time.sleep(0.1)

    def _mjpeg(self, path: str, fps: float):
        bnd = "frame"
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={bnd}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = 0.0
        period = 1.0 / max(1.0, min(fps, 15.0))
        while True:
            try:
                m = os.path.getmtime(path)
            except OSError:
                time.sleep(0.5)
                continue
            if m != last:
                last = m
                try:
                    data = Path(path).read_bytes()
                except OSError:
                    data = b""
                if data:
                    self.wfile.write(f"--{bnd}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            time.sleep(period)


def main():
    global TELE
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=int(os.environ.get("FOX_DASH_PORT", "8080")))
    a = ap.parse_args()
    TELE = Telemetry()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    srv.daemon_threads = True
    print(f"[DASH] http://0.0.0.0:{a.port}  (telemetría UDP {TELE_PORT}, comandos {CMD_PORT})", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
