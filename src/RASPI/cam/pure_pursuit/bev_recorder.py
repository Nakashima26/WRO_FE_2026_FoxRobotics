"""
Grabación del BEV LIMPIO (diagnóstico, no cambia el comportamiento del carro).

El .avi del HUD no sirve para re-probar la detección de la naranja: el panel BEV
lleva encima la ruta amarilla, los círculos de los conos y la propia línea
naranja dibujada, justo donde están los píxeles que importan, y además solo guarda
1 de cada 6 frames (el tracker necesita frames seguidos: PERSIST_FRAMES=4).

Esto guarda el BEV tal cual lo reciben detect_lines()/detect_centerline(), en
CADA frame procesado mientras se graba, en un archivo aparte junto al .avi:

    orillasNNN_bev.bin = secuencia de registros [u32 len][u32 n][len bytes JPEG]

n = record_count del runtime en ese frame. El frame k del .avi del HUD es el de
n = 6*(k+1) (record_every_n=6), y en el journal el frame con n=6 es el que imprime
"[REC] Grabando en ..." -> el frame del log de cualquier registro sale directo.

El encode (JPEG q95, croma 4:4:4 si la versión de OpenCV lo permite) corre en un
hilo aparte. Si la cola se llena se descarta el frame (se cuenta en `dropped`)
en vez de frenar el loop principal; n viaja en cada registro, así que un
descarte no desalinea nada.
"""

import struct
import threading
from queue import Empty, Full, Queue

import cv2


class BevRecorder:
    def __init__(self, path: str, quality: int = 95, max_queue: int = 16):
        self.path = path
        self._f = open(path, "wb")
        self._q: Queue = Queue(maxsize=max_queue)
        self._params = [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
        # 4:4:4 = sin submuestreo de croma: la cinta naranja lejana mide ~2-4 px
        # y con 4:2:0 su saturación se embarra con el crema de al lado.
        _sf = getattr(cv2, "IMWRITE_JPEG_SAMPLING_FACTOR", None)
        _444 = getattr(cv2, "IMWRITE_JPEG_SAMPLING_FACTOR_444", None)
        if _sf is not None and _444 is not None:
            self._params += [_sf, _444]
        self.written = 0
        self.dropped = 0
        self._stopped = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def write(self, n: int, bev) -> None:
        """Encola el BEV de este frame. `bev` no se copia: el runtime crea un
        array nuevo en cada warp y no vuelve a tocar el del frame anterior."""
        if self._stopped or bev is None:
            return
        try:
            self._q.put_nowait((int(n), bev))
        except Full:
            self.dropped += 1

    def _run(self) -> None:
        while not self._stopped or not self._q.empty():
            try:
                n, bev = self._q.get(timeout=0.05)
            except Empty:
                continue
            try:
                ok, buf = cv2.imencode(".jpg", bev, self._params)
                if ok:
                    data = buf.tobytes()
                    self._f.write(struct.pack("<II", len(data), n))
                    self._f.write(data)
                    self.written += 1
            except Exception:
                self.dropped += 1
            finally:
                self._q.task_done()

    def stop(self) -> None:
        self._stopped = True
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        try:
            self._f.close()
        except Exception:
            pass
        print(f"[REC] BEV limpio: {self.written} frames en {self.path} "
              f"(descartados {self.dropped})", flush=True)
