"""Dashboard del carro desde TU COMPU + flasheo de la ESP32 por USB (sin abrir el Arduino IDE).

    python scripts/fox_pc.py            (o doble clic en scripts/dashboard_pc.bat)

Abre http://localhost:8091 :
  - todo lo demás se lo pide a la Pi (proxy al dashboard de la Pi, :8080), así que se ve igual;
  - la pestaña "ESP" usa este programa para compilar y subir con arduino-cli al puerto USB
    de la ESP conectada a esta compu, y al terminar reinicia wro-runtime en la Pi.

La Pi se busca sola (cable 10.43.0.1, hotspot 10.42.0.1, Tailscale); o pásala con --pi.
Solo librería estándar de Python.
"""
from __future__ import annotations

import argparse
import hashlib
import http.client
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

REPO = Path(__file__).resolve().parents[1]
ESP_DIR = REPO / "src" / "ESP32"
# La página (html/js/css) sale del repo de ESTA compu: así trae la pestaña ESP aunque la Pi
# tenga una versión anterior. Los datos (/api/*) sí vienen de la Pi. --ui-pi usa la de la Pi.
STATIC = REPO / "src" / "RASPI" / "cam" / "pure_pursuit" / "dashboard" / "static"
UI_LOCAL = True
SKETCHES = {
    "open": ("RondaAbierta", "Open (RondaAbierta)"),
    "obs": ("PurePursuitCarrera", "Obstáculos (PurePursuitCarrera)"),
}
FQBN = "esp32:esp32:esp32"
CLI_CANDIDATOS = [
    os.environ.get("FOX_ARDUINO_CLI", ""),
    str(Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"),
    shutil.which("arduino-cli") or "",
]
PIS = ["10.43.0.1", "10.42.0.1", "100.79.110.111"]   # cable, hotspot FoxCar, Tailscale
# Puentes USB-serie de las placas ESP32 (CP210x, CH340/CH9102, FTDI).
USB_VIDS = {"10c4", "1a86", "0403"}


def cli_path() -> str | None:
    for c in CLI_CANDIDATOS:
        if c and Path(c).is_file():
            return c
    return None


# ── Pi ────────────────────────────────────────────────────────────────────────
class Pi:
    def __init__(self, fijo: str | None, port: int):
        self.fijo = fijo
        self.port = port
        self.host: str | None = fijo
        self.lock = threading.Lock()

    def _vive(self, host: str) -> bool:
        try:
            c = http.client.HTTPConnection(host, self.port, timeout=1.5)
            c.request("GET", "/api/surprise")
            ok = c.getresponse().status < 500
            c.close()
            return ok
        except OSError:
            return False

    def buscar(self) -> str | None:
        with self.lock:
            if self.fijo:
                self.host = self.fijo
                return self.host
            if self.host and self._vive(self.host):
                return self.host
            for h in PIS:
                if self._vive(h):
                    self.host = h
                    print(f"[PI] dashboard en http://{h}:{self.port}", flush=True)
                    return h
            self.host = None
            return None

    def post(self, path: str, body: dict, timeout: float = 10) -> dict:
        host = self.host or self.buscar()
        if not host:
            raise OSError("no encuentro la Pi")
        c = http.client.HTTPConnection(host, self.port, timeout=timeout)
        c.request("POST", path, json.dumps(body), {"Content-Type": "application/json"})
        r = c.getresponse()
        d = json.loads(r.read().decode() or "{}")
        c.close()
        return d


PI: Pi


# ── Flasheo ──────────────────────────────────────────────────────────────────
class Job:
    def __init__(self):
        self.lock = threading.Lock()
        self.busy = False
        self.sketch = ""
        self.state = "sin correr"
        self.ok: bool | None = None
        self.log: list[str] = []
        self.t0 = 0.0
        self.t1 = 0.0

    def add(self, line: str):
        with self.lock:
            self.log.append(line)
            del self.log[:-400]

    def snapshot(self) -> dict:
        with self.lock:
            return {"busy": self.busy, "sketch": self.sketch, "state": self.state, "ok": self.ok,
                    "log": self.log[-200:], "secs": round((self.t1 or time.time()) - self.t0, 1) if self.t0 else None}


JOB = Job()


def puertos() -> list[dict]:
    """Puertos serie con placa: los USB-serie de ESP32 primero."""
    cli = cli_path()
    out = []
    if cli:
        try:
            r = subprocess.run([cli, "board", "list", "--format", "json"], capture_output=True, text=True, timeout=20)
            d = json.loads(r.stdout or "{}")
            for b in d.get("detected_ports", d if isinstance(d, list) else []):
                p = b.get("port", {})
                props = p.get("properties", {}) or {}
                vid = str(props.get("vid", "")).lower().replace("0x", "")
                if p.get("protocol") != "serial":
                    continue
                out.append({"port": p.get("address"), "label": p.get("label") or p.get("address"),
                            "esp": vid in USB_VIDS, "vid": vid})
        except Exception as e:
            print(f"[USB] board list falló: {e}", flush=True)
    out.sort(key=lambda x: (not x["esp"], x["port"] or ""))
    return out


def git_info() -> dict:
    def g(*a):
        try:
            return subprocess.run(["git", *a], cwd=REPO, capture_output=True, text=True, timeout=10,
                                  encoding="utf-8", errors="replace").stdout
        except Exception:
            return ""
    # porcelain: "XY ruta" (no recortar la primera línea: su X puede ser un espacio)
    sucios = [l[3:] for l in g("status", "--porcelain", "--", "src/ESP32").splitlines() if l.strip()]
    return {"commit": g("log", "-1", "--format=%h %s").strip(), "branch": g("rev-parse", "--abbrev-ref", "HEAD").strip(),
            "esp_sin_commit": sucios}


BUILD_ROOT = Path(tempfile.gettempdir()) / "fox_build"


def huella_sketch(nombre: str) -> str:
    """sha1 de todos los archivos del sketch (+ placa). Las librerías de fuera no cuentan:
    si actualizas una librería, usa "compilar de nuevo"."""
    h = hashlib.sha1(FQBN.encode())
    raiz = ESP_DIR / nombre
    for f in sorted(p for p in raiz.rglob("*") if p.is_file()):
        h.update(str(f.relative_to(raiz)).encode())
        h.update(f.read_bytes())
    return h.hexdigest()


def correr(cmd: list[str]) -> bool:
    JOB.add("$ arduino-cli " + " ".join(cmd[1:]))
    pr = subprocess.Popen(cmd, cwd=ESP_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                          encoding="utf-8", errors="replace", bufsize=1)
    for line in pr.stdout:
        line = line.rstrip()
        if line:
            JOB.add(line)
    pr.wait()
    if pr.returncode != 0:
        JOB.add(f"!! arduino-cli terminó con código {pr.returncode}")
    return pr.returncode == 0


def flashear(sketch: str, port: str, sorpresa: bool | None, forzar: bool = False):
    nombre, titulo = SKETCHES[sketch]
    cli = cli_path()
    ok = False
    try:
        if not cli:
            JOB.add("!! no encuentro arduino-cli (instala el Arduino IDE o pon FOX_ARDUINO_CLI)")
            return
        # Compilar solo si el sketch cambió: el binario de cada sketch se guarda en su
        # carpeta de build con la huella de sus archivos (compilar ~40 s, subir ~8 s).
        build = BUILD_ROOT / sketch
        huella = huella_sketch(nombre)
        marca = build / "fox_huella.txt"
        previa = marca.read_text(encoding="utf-8").strip() if marca.is_file() else ""
        if forzar or previa != huella or not any(build.glob(f"{nombre}.ino.bin")):
            with JOB.lock:
                JOB.state = f"compilando {titulo}"
            JOB.add("compilando: " + ("forzado" if forzar else "el código cambió" if previa else "primera vez"))
            marca.unlink(missing_ok=True)
            if not correr([cli, "compile", "--fqbn", FQBN, "--build-path", str(build), nombre]):
                return
            marca.write_text(huella, encoding="utf-8")
        else:
            JOB.add("sin cambios desde la última compilación: solo se sube el binario guardado")
        with JOB.lock:
            JOB.state = f"subiendo {titulo} a {port}"
        if not correr([cli, "upload", "--fqbn", FQBN, "-p", port, "--input-dir", str(build), nombre]):
            return
        JOB.add(f"✓ {titulo} subido")
        # Siempre reiniciar wro-runtime tras flashear (el ESP arrancó de cero).
        with JOB.lock:
            JOB.state = "reiniciando wro-runtime en la Pi"
        try:
            if sketch == "obs" and sorpresa is not None:
                d = PI.post("/api/surprise", {"on": sorpresa})
            else:
                d = PI.post("/api/service", {"action": "restart"})
            JOB.add("Pi: " + str(d.get("msg") or d.get("error") or d))
        except Exception as e:
            JOB.add(f"!! no pude reiniciar el servicio en la Pi ({e}). Reinícialo desde el dashboard.")
        ok = True
    except Exception as e:
        JOB.add(f"!! {e}")
    finally:
        with JOB.lock:
            JOB.busy = False
            JOB.ok = ok
            JOB.t1 = time.time()
            JOB.state = "listo" if ok else "falló"


# ── HTTP: /local/* aquí, lo demás a la Pi ────────────────────────────────────
HOP = {"connection", "keep-alive", "transfer-encoding", "te", "trailer", "upgrade", "proxy-authorization",
       "proxy-authenticate", "host", "content-length"}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"   # sin keep-alive: el proxy cierra al terminar (sirve para SSE y MJPEG)

    def log_message(self, *a):
        pass

    def _json(self, obj, code: int = 200):
        b = json.dumps(obj, ensure_ascii=False, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def _body(self) -> bytes:
        n = int(self.headers.get("Content-Length") or 0)
        return self.rfile.read(n) if n else b""

    def do_GET(self):
        if self.path.startswith("/local/"):
            return self._local("GET", b"")
        if UI_LOCAL:
            p = urlparse(self.path).path
            if p in ("/", "/index.html"):
                return self._static("index.html")
            if p.startswith("/static/"):
                return self._static(p[len("/static/"):])
        self._proxy("GET", b"")

    def do_POST(self):
        body = self._body()
        if self.path.startswith("/local/"):
            return self._local("POST", body)
        self._proxy("POST", body)

    def _static(self, name: str):
        f = (STATIC / name).resolve()
        if STATIC.resolve() not in f.parents or not f.is_file():
            return self._json({"error": "no"}, 404)
        ctype = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml"}.get(f.suffix, "application/octet-stream")
        b = f.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def _local(self, method: str, body: bytes):
        p = urlparse(self.path).path
        try:
            if p == "/local/flash/status":
                return self._json({"job": JOB.snapshot(), "ports": puertos(), "git": git_info(),
                                   "cli": bool(cli_path()), "pi": PI.host,
                                   "sketches": {k: v[1] for k, v in SKETCHES.items()}})
            if p == "/local/flash/job":
                return self._json({"job": JOB.snapshot()})
            if p == "/local/flash" and method == "POST":
                b = json.loads(body.decode() or "{}")
                sketch = b.get("sketch")
                if sketch not in SKETCHES:
                    return self._json({"error": "sketch: open | obs"}, 400)
                port = b.get("port") or next((x["port"] for x in puertos() if x["esp"]), None)
                if not port:
                    return self._json({"error": "no veo la ESP por USB (¿cable conectado?)"}, 400)
                sorpresa = b.get("surprise")
                with JOB.lock:
                    if JOB.busy:
                        return self._json({"error": "ya está flasheando"}, 409)
                    JOB.busy, JOB.sketch, JOB.ok, JOB.log = True, sketch, None, []
                    JOB.t0, JOB.t1, JOB.state = time.time(), 0.0, "empezando"
                threading.Thread(target=flashear, args=(sketch, port, None if sorpresa is None else bool(sorpresa),
                                                        bool(b.get("force"))), daemon=True).start()
                return self._json({"msg": f"flasheando {SKETCHES[sketch][1]} en {port}"})
            return self._json({"error": "no"}, 404)
        except Exception as e:
            return self._json({"error": str(e)}, 500)

    def _proxy(self, method: str, body: bytes):
        host = PI.host or PI.buscar()
        if not host:
            if self.path in ("/", "/index.html"):
                msg = ("<h2>No encuentro la Pi</h2><p>Probé " + ", ".join(PIS) +
                       ". Conecta el cable/hotspot o corre con <code>--pi IP</code>. Recarga en unos segundos.</p>")
                b = msg.encode()
                self.send_response(502)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
                return
            return self._json({"error": "sin Pi"}, 502)
        try:
            c = http.client.HTTPConnection(host, PI.port, timeout=15)
            hdr = {k: v for k, v in self.headers.items() if k.lower() not in HOP}
            if body:
                hdr["Content-Length"] = str(len(body))
            c.request(method, self.path, body or None, hdr)
            r = c.getresponse()
        except OSError as e:
            PI.host = None if not PI.fijo else PI.host
            return self._json({"error": f"Pi: {e}"}, 502)
        try:
            self.send_response(r.status)
            for k, v in r.getheaders():
                if k.lower() not in HOP:
                    self.send_header(k, v)
            self.end_headers()
            streaming = (r.getheader("Content-Type") or "").startswith(("text/event-stream", "multipart/"))
            if streaming and c.sock is not None:
                c.sock.settimeout(None)
            while True:
                chunk = r.read1(65536) if streaming else r.read(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)
                if streaming:
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError):
            pass
        finally:
            c.close()


def main():
    global PI
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi", help="IP de la Pi (si no, la busca: " + ", ".join(PIS) + ")")
    ap.add_argument("--pi-port", type=int, default=8080)
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--ui-pi", action="store_true", help="usar la página del dashboard de la Pi en vez de la del repo local")
    a = ap.parse_args()
    global UI_LOCAL
    UI_LOCAL = not a.ui_pi
    PI = Pi(a.pi, a.pi_port)
    print(f"[PC] arduino-cli: {cli_path() or 'NO ENCONTRADO'}", flush=True)
    print(f"[PC] buscando la Pi…", flush=True)
    print(f"[PC] Pi: {PI.buscar() or 'no encontrada (sigo intentando)'}", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    srv.daemon_threads = True
    url = f"http://localhost:{a.port}"
    print(f"[PC] dashboard: {url}   (Ctrl+C para salir)", flush=True)
    if not a.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
