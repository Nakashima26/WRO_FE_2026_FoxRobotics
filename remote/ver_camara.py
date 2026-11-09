"""Vista en vivo de lo que ve el carro, en el navegador (corre EN LA PI).

Sirve como MJPEG el cuadro con HUD que wro-runtime escribe en /tmp/wro_cam_frame.jpg
(cada cam_frame_every_n cuadros). No abre la cámara ni habla con el runtime: solo lee
ese archivo, así que no le quita nada al pipeline salvo leer ~50 KB unas 10 veces por s.

    ~/FoxRobotics/.venv/bin/python ~/FoxRobotics/remote/ver_camara.py      # puerto 8080
    -> abrir http://<ip tailscale de la pi>:8080
"""
import http.server
import os
import socketserver
import sys
import time

FRAME = "/tmp/wro_cam_frame.jpg"
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
FPS = 12

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Carro en vivo</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{margin:0;background:#111;color:#aaa;font:14px monospace}
img{width:100%;height:auto;display:block}#s{padding:6px}</style></head>
<body><img src="/stream"><div id="s">wro-runtime en vivo (si se congela: el runtime no esta corriendo o no hay HUD)</div></body></html>"""


def leer_cuadro(ultimo_mtime):
    """El runtime reescribe el archivo en el lugar: se descarta un JPEG a medio escribir."""
    try:
        m = os.path.getmtime(FRAME)
        if m == ultimo_mtime:
            return None, m
        with open(FRAME, "rb") as f:
            d = f.read()
        if len(d) < 4 or d[:2] != b"\xff\xd8" or d[-2:] != b"\xff\xd9":
            return None, ultimo_mtime
        return d, m
    except OSError:
        return None, ultimo_mtime


class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(PAGE)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        mt = 0.0
        try:
            while True:
                d, mt = leer_cuadro(mt)
                if d:
                    self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(d)).encode() + b"\r\n\r\n" + d + b"\r\n")
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass


class S(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    print(f"vista en vivo en http://0.0.0.0:{PORT}", flush=True)
    S(("0.0.0.0", PORT), H).serve_forever()
