"""Editor del recorte del carro para las portadas.

python video/portadas/editor/servidor.py   -> abre http://localhost:8790/editor.html
"Guardar" escribe editor/mascara.png (tamaño completo, para seguir editando),
../carro_recorte.png (recortado) y regenera portada_open.png / portada_obstacle.png.
"""
import io
import shutil
import subprocess
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from PIL import Image

AQUI = Path(__file__).resolve().parent
PORTADAS = AQUI.parent
FOTO = PORTADAS.parents[1] / "v-photos/isometric.jpeg"
PUERTO = 8790


class Manejador(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        nombre = path.split("?")[0].lstrip("/")
        if nombre == "foto.jpeg":
            return str(FOTO)
        if nombre.startswith("portada_"):
            return str(PORTADAS / nombre)
        return super().translate_path(path)

    def do_POST(self):
        if self.path != "/guardar":
            self.send_error(404)
            return
        try:
            datos = self.rfile.read(int(self.headers["Content-Length"]))
            img = Image.open(io.BytesIO(datos)).convert("RGBA")
            img.save(AQUI / "mascara.png")
            img.crop(img.getbbox()).save(PORTADAS / "carro_recorte.png")
            subprocess.run([sys.executable, str(PORTADAS / "portada.py")], check=True)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"ok")
        except Exception as e:  # noqa: BLE001 - mostrar el error en el editor
            self.send_response(500)
            self.end_headers()
            self.wfile.write(str(e).encode())

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


if __name__ == "__main__":
    if not (AQUI / "mascara.png").exists():
        shutil.copy(PORTADAS / "carro_recorte.png", AQUI / "mascara.png")
    print(f"http://localhost:{PUERTO}/editor.html")
    ThreadingHTTPServer(("127.0.0.1", PUERTO), partial(Manejador, directory=str(AQUI))).serve_forever()
