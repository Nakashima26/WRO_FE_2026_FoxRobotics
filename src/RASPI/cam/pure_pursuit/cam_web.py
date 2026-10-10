"""
cam_web.py — Ver la cámara EN VIVO desde el navegador (sin monitor) y calibrar
las ganancias de color (balance de blancos manual por el lente sin filtro IR).

Dos modos:

  1) Calibración (default): esta herramienta ABRE la cámara con Picamera2.
     Página con sliders de ganancia roja/azul, botón "balancear con la hoja"
     (pon una hoja blanca en el recuadro) y comparación de tuning normal/noir.
       ~/FoxRobotics/.venv/bin/python -m pure_pursuit.cam_web            (desde src/RASPI/cam)
       ~/FoxRobotics/.venv/bin/python -m pure_pursuit.cam_web --tuning noir

  2) HUD de carrera (--hud): NO toca la cámara. Transmite /tmp/wro_cam_frame.jpg,
     el HUD que escribe runtime_nuevo.py mientras corre (cámara + BEV).
       ~/FoxRobotics/.venv/bin/python -m pure_pursuit.cam_web --hud

Abrir en el navegador:  http://<IP Tailscale de la Pi>:8080   (raspi5 = 100.79.110.111)

OJO: la cámara solo la puede abrir UN proceso. Con wro-runtime corriendo usa --hud.
Rondas oficiales: Wi-Fi apagado (regla 11.10) -> esto es solo para desarrollo.

Los mismos valores de la cámara se ponen en vision.py (open_camera):
  colour-gains=<R,B>  y, si se usa el tuning noir,
  LIBCAMERA_RPI_TUNING_FILE=/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json
"""

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

HUD_PATH = "/tmp/wro_cam_frame.jpg"
TUNING_DIR = "/usr/share/libcamera/ipa/rpi/pisp"
# Tuning propio: imx219_noir + tabla ALSC (sombreado del lente) calibrada con
# rpi-ctt sobre fotos de hoja blanca. Ver camera_tuning/README.md.
WRO_TUNING = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "camera_tuning", "imx219_noir_wro_pi5.json")

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cámara raspi5</title>
<style>
 body{font-family:system-ui,sans-serif;margin:12px;background:#111;color:#eee}
 img{max-width:100%;border:1px solid #444}
 .row{margin:8px 0} label{display:inline-block;width:110px}
 input[type=range]{width:min(420px,60vw)} button{margin:4px 4px 4px 0;padding:6px 10px}
 code{background:#222;padding:2px 6px} #st{white-space:pre;font-family:monospace;font-size:13px}
</style></head><body>
<h3>Cámara raspi5 — __MODE__</h3>
<img src="/stream" alt="stream">
<div id="ctl" style="display:__CTL__">
 <div class="row"><label>Ganancia R</label><input id="r" type="range" min="0.3" max="4" step="0.01"> <span id="rv"></span></div>
 <div class="row"><label>Ganancia B</label><input id="b" type="range" min="0.3" max="4" step="0.01"> <span id="bv"></span></div>
 <div class="row">
  <button onclick="post('/balance')">Balancear con la hoja blanca del recuadro</button>
  <button onclick="post('/awb?on=1')">AWB automático (para comparar)</button>
  <button onclick="post('/awb?on=0')">Fijar ganancias actuales</button>
 </div>
 <div class="row">Tuning:
  <button onclick="post('/tuning?t=normal')">normal (imx219.json)</button>
  <button onclick="post('/tuning?t=noir')">noir (imx219_noir.json)</button>
  <button onclick="post('/tuning?t=wro')">calibrado WRO (lente propio)</button>
 </div>
 <div class="row"><a href="/snap.jpg" style="color:#8cf">Bajar foto</a></div>
</div>
<div id="st"></div>
<script>
const r=document.getElementById('r'), b=document.getElementById('b');
let busy=false;
function send(){ if(busy) return; busy=true;
  fetch('/gains?r='+r.value+'&b='+b.value,{method:'POST'}).finally(()=>busy=false); }
r.oninput=b.oninput=()=>{document.getElementById('rv').textContent=r.value;
  document.getElementById('bv').textContent=b.value; send();};
function post(u){ fetch(u,{method:'POST'}); }
async function poll(){ try{ const s=await (await fetch('/state')).json();
  if(document.activeElement!==r && document.activeElement!==b && s.r){
    r.value=s.r; b.value=s.b; document.getElementById('rv').textContent=s.r;
    document.getElementById('bv').textContent=s.b; }
  document.getElementById('st').textContent=s.text; }catch(e){} setTimeout(poll,700); }
poll();
</script></body></html>"""


class Cam:
    """Picamera2 en el MISMO modo de sensor que la carrera (1640x1232, campo
    completo) escalado por el ISP a 640x480, como el pipeline de vision.py."""

    def __init__(self, tuning, r, b):
        self.lock = threading.Lock()
        self.jpeg = None
        self.frame = None
        self.meta = {}
        self.r, self.b = r, b
        self.awb = False
        self.tuning = tuning
        self.cam = None
        self.msg = ""
        self._open()
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        from picamera2 import Picamera2
        if self.tuning in ("normal", "noir"):
            tun = Picamera2.load_tuning_file(f"imx219{'_noir' if self.tuning == 'noir' else ''}.json",
                                             dir=TUNING_DIR)
        else:   # ruta a un archivo propio (p. ej. camera_tuning/imx219_noir_wro_pi5.json)
            tun = Picamera2.load_tuning_file(os.path.abspath(self.tuning))
        self.cam = Picamera2(tuning=tun)
        cfg = self.cam.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},   # RGB888 = arreglo BGR (OpenCV)
            raw={"size": (1640, 1232)},
            controls={"FrameRate": 30})
        self.cam.configure(cfg)
        self.cam.start()
        self._apply()

    def _apply(self):
        if self.awb:
            self.cam.set_controls({"AwbEnable": True})
        else:
            self.cam.set_controls({"AwbEnable": False, "ColourGains": (self.r, self.b)})

    def set_gains(self, r, b):
        with self.lock:
            self.r, self.b, self.awb = r, b, False
            self._apply()

    def set_awb(self, on):
        with self.lock:
            if not on:   # fija lo que el AWB estaba usando
                g = self.meta.get("ColourGains")
                if g:
                    self.r, self.b = round(g[0], 3), round(g[1], 3)
            self.awb = on
            self._apply()

    def set_tuning(self, t, port):
        # Reabrir Picamera2 dentro del mismo proceso deja el hilo de captura
        # bloqueado en la cámara vieja -> reiniciar el proceso limpio, con las
        # mismas ganancias.
        import sys
        self.cam.stop()
        self.cam.close()
        os.execv(sys.executable, [sys.executable, os.path.abspath(__file__), "--tuning", t,
                                  "--r", str(self.r), "--b", str(self.b), "--port", str(port)])

    @staticmethod
    def roi(f):
        h, w = f.shape[:2]
        return f[int(h * 0.40):int(h * 0.70), int(w * 0.35):int(w * 0.65)]

    def balance(self):
        """Ajusta R y B para que el recuadro (hoja blanca) quede gris neutro."""
        self.msg = ""
        for _ in range(10):   # el ISP tarda unos frames en aplicar -> iterar
            f = self.frame
            if f is None:
                return
            bgr = self.roi(f).reshape(-1, 3).astype(np.float32)
            bgr = bgr[(bgr.max(1) < 250)]            # sin píxeles saturados
            if len(bgr) < 0.2 * self.roi(f).shape[0] * self.roi(f).shape[1]:
                # recuadro casi todo saturado (hoja muy iluminada): bajar las
                # dos ganancias y volver a medir
                with self.lock:
                    self.r = round(max(0.3, self.r * 0.75), 3)
                    self.b = round(max(0.3, self.b * 0.75), 3)
                    self.awb = False
                    self._apply()
                time.sleep(0.6)
                continue
            mb, mg, mr = np.median(bgr, axis=0)
            if max(mb, mg, mr) < 40:
                self.msg = "Balance cancelado: el recuadro está muy oscuro (¿hay luz sobre la hoja?)"
                return
            if abs(mr / max(mg, 1) - 1) < 0.02 and abs(mb / max(mg, 1) - 1) < 0.02:
                return   # ya neutro
            with self.lock:
                self.r = float(np.clip(self.r * mg / max(mr, 1), 0.3, 4.0))
                self.b = float(np.clip(self.b * mg / max(mb, 1), 0.3, 4.0))
                self.r, self.b = round(self.r, 3), round(self.b, 3)
                self.awb = False
                self._apply()
            time.sleep(0.6)

    def _loop(self):
        while True:
            try:
                req = self.cam.capture_request()
                f = req.make_array("main")
                self.meta = req.get_metadata()
                req.release()
            except Exception:
                time.sleep(0.05)
                continue
            self.frame = f
            v = f.copy()
            h, w = v.shape[:2]
            cv2.rectangle(v, (int(w * 0.35), int(h * 0.40)), (int(w * 0.65), int(h * 0.70)), (0, 255, 255), 1)
            ok, j = cv2.imencode(".jpg", v, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self.jpeg = j.tobytes()

    def state(self):
        f = self.frame
        txt = ""
        if f is not None:
            mb, mg, mr = np.median(self.roi(f).reshape(-1, 3), axis=0)
            txt += (f"Recuadro (mediana) B={mb:.0f} G={mg:.0f} R={mr:.0f}   "
                    f"R/G={mr / max(mg, 1):.2f}  B/G={mb / max(mg, 1):.2f}  (neutro = 1.00)\n")
        m = self.meta
        g = m.get("ColourGains")
        txt += (f"tuning={self.tuning}  AWB={'auto' if self.awb else 'fijo'}  "
                f"ganancias ISP={tuple(round(x, 3) for x in g) if g else '?'}\n"
                f"exposición={m.get('ExposureTime', '?')} us  ganancia analógica={m.get('AnalogueGain', 0):.2f}  "
                f"lux≈{m.get('Lux', 0):.0f}\n\n"
                f"Para vision.py:  colour-gains=<{self.r:.2f},{self.b:.2f}>"
                + ("" if self.tuning == "normal" else "   + tuning=" + (
                    TUNING_DIR + "/imx219_noir.json" if self.tuning == "noir" else self.tuning)))
        if self.msg:
            txt += "\n\n" + self.msg
        return {"r": self.r, "b": self.b, "text": txt}


class HudSource:
    """Modo --hud: transmite el HUD que escribe runtime_nuevo.py, sin tocar la cámara."""

    def __init__(self, path):
        self.path = path

    @property
    def jpeg(self):
        try:
            with open(self.path, "rb") as fh:
                return fh.read()
        except OSError:
            return None

    def state(self):
        try:
            age = time.time() - os.path.getmtime(self.path)
            return {"text": f"HUD de {self.path}: hace {age:.1f} s"}
        except OSError:
            return {"text": "Sin HUD: ¿está corriendo wro-runtime?"}


def make_handler(src, hud):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                page = (PAGE.replace("__MODE__", "HUD de carrera" if hud else "calibración de color")
                        .replace("__CTL__", "none" if hud else "block"))
                self._send(200, "text/html; charset=utf-8", page.encode())
            elif u.path == "/state":
                self._send(200, "application/json", json.dumps(src.state()).encode())
            elif u.path == "/snap.jpg":
                j = src.jpeg
                self._send(200 if j else 503, "image/jpeg", j or b"")
            elif u.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
                self.end_headers()
                last = None
                try:
                    while True:
                        j = src.jpeg
                        if j and j is not last:
                            last = j
                            self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, "text/plain", b"404")

        def do_POST(self):
            if hud:
                return self._send(400, "text/plain", b"modo --hud: sin controles")
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            if u.path == "/gains":
                src.set_gains(float(q["r"]), float(q["b"]))
            elif u.path == "/balance":
                threading.Thread(target=src.balance, daemon=True).start()
            elif u.path == "/awb":
                src.set_awb(q.get("on") == "1")
            elif u.path == "/tuning":
                self._send(200, "text/plain", b"ok")
                self.wfile.flush()
                t = {"noir": "noir", "wro": WRO_TUNING}.get(q.get("t"), "normal")
                threading.Timer(0.3, src.set_tuning, args=(t, self.server.server_port)).start()
                return
            self._send(200, "text/plain", b"ok")
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--hud", action="store_true", help="transmitir el HUD de wro-runtime (no abre la cámara)")
    ap.add_argument("--tuning", default="normal",
                    help="normal | noir | ruta a un .json propio (p. ej. ../camera_tuning/imx219_noir_wro_pi5.json)")
    ap.add_argument("--r", type=float, default=1.2, help="ganancia roja inicial (la de vision.py)")
    ap.add_argument("--b", type=float, default=1.5, help="ganancia azul inicial (la de vision.py)")
    a = ap.parse_args()
    src = HudSource(HUD_PATH) if a.hud else Cam(a.tuning, a.r, a.b)
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(src, a.hud))
    print(f"[cam_web] {'HUD' if a.hud else 'calibración'} en http://0.0.0.0:{a.port}  (Ctrl+C para salir)", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
