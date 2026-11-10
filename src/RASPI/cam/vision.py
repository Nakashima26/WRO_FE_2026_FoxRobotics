import os
import ctypes

import cv2
import numpy as np


cv2.setUseOptimized(True)
fox_cv_threads = int(os.environ.get("FOX_CV_THREADS", str(min(4, os.cpu_count() or 1))))
cv2.setNumThreads(fox_cv_threads)


_LENS_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pure_pursuit", "lens_calib.npz")


class _UndistortCap:
    """Envuelve el VideoCapture y endereza cada frame con la calibración de lente."""

    def __init__(self, cap, m1, m2):
        self._cap, self._m1, self._m2 = cap, m1, m2

    def read(self):
        ok, frame = self._cap.read()
        if ok and frame is not None:
            frame = cv2.remap(frame, self._m1, self._m2, cv2.INTER_LINEAR)
        return ok, frame

    def __getattr__(self, name):
        return getattr(self._cap, name)


def _wrap_undistort(cap):
    # FOX_LENS_CALIB: ruta del npz; "" o "0" = apagado. Sin archivo = frames crudos.
    path = os.environ.get("FOX_LENS_CALIB", _LENS_DEFAULT)
    if path in ("", "0") or not os.path.exists(path):
        return cap
    try:
        d = np.load(path)
        size = tuple(int(v) for v in d["size"])   # tamaño con que se calibró (640x480)
        m1, m2 = cv2.initUndistortRectifyMap(d["K"], d["dist"], None, d["newK"], size, cv2.CV_16SC2)
        print(f"[CAM] lente enderezado con {path} (sig {str(d['sig'])})", flush=True)
        return _UndistortCap(cap, m1, m2)
    except Exception as e:   # una calibración rota no debe tumbar la carrera
        print(f"[CAM] AVISO: no se pudo cargar {path} ({e}); sin enderezar", flush=True)
        return cap


def open_camera(cam_index=0, undistort=True):
    if os.name == "nt":
        cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)
    else:
        # Color: el lente gran angular no tiene filtro IR -> imagen rojiza y aro
        # MAGENTA en las orillas (ver camera_tuning/README.md). Pi 5 (ISP PiSP):
        # tuning propio imx219_noir_wro_pi5.json + ganancias <1.10,1.37>
        # (1.51 -> 1.37 el 2026-11-04 en la pista: el piso blanco salía B/G 1.16, ahora 1.06)
        # (calibrado 2026-10-07). Pi 4 (ISP VC4): <1.2,1.5> con el tuning del sistema.
        tuning_pi5 = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "camera_tuning", "imx219_noir_wro_pi5.json")
        if os.path.isdir("/usr/share/libcamera/ipa/rpi/pisp") and os.path.exists(tuning_pi5):
            os.environ.setdefault("LIBCAMERA_RPI_TUNING_FILE", tuning_pi5)
            gains = "<1.10,1.37>"
        else:
            gains = "<1.2,1.5>"
        # 40 fps = máximo del modo 1640x1232 del IMX219 (el modo de 3280x2464 solo da
        # 21). format=BGRx: en la Pi 5 libcamerasrc sin formato explícito falla con
        # "not-negotiated"; BGRx sale directo del ISP (NV12 cae al CPU y baja fps).
        fps = int(os.environ.get("FOX_CAM_FPS", "40"))
        # Escalar antes de convertir: el BGRx de 2 MP baja a 640x480 y recién ahí pasa a BGR (FOX_CAM_PIPE=old: orden viejo).
        conv = ("! videoconvert ! videoscale ! video/x-raw, width=640, height=480, format=BGR "
                if os.environ.get("FOX_CAM_PIPE") == "old" else
                "! videoscale n-threads=2 ! video/x-raw, width=640, height=480 "
                "! videoconvert n-threads=2 ! video/x-raw, format=BGR ")
        pipeline = (
            f"libcamerasrc awb-enable=false colour-gains={gains} "
            "! queue max-size-buffers=1 leaky=downstream "
            f"! video/x-raw, width=1640, height=1232, framerate={fps}/1, format=BGRx "
            f"{conv}"
            "! appsink drop=true max-buffers=1 sync=false"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened() and os.environ.get("FOX_CAM_PIPE") != "old":
            # GStreamer viejo sin n-threads: reintenta con la cadena original
            os.environ["FOX_CAM_PIPE"] = "old"
            return open_camera(cam_index, undistort)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        raise RuntimeError(
            "No se pudo abrir la cámara. En PC usa una webcam conectada y verifica que otro programa no la esté usando."
        )

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    if undistort:
        cap = _wrap_undistort(cap)
    return cap


# Rangos HSV de conos. Módulo-nivel para que calibra_luz.py los lea sin abrir
# la cámara. Si los cambias, actualiza LINE_CONE_HSV en
# pure_pursuit/config.py (borra px de cono de la cinta naranja).
#
# Banda 2 (cola azulada del rojo, wrap-around): H 173 -> 177.
# La pared magenta del estacionamiento (rectas 1/5/9 y tras el 4o
# giro) cae en H 172-175 con S~164 V~81 -> la banda 173-176 la
# detectaba como cono Red y el carro daba un volantazo a "esquivarla"
# (orillas809 ~seg 25: bbox Red sobre el panel magenta, mem R(x=241),
# steer +46->+50). Medido en frames: pared H med=173 / B-R=0.50 / 87%
# de sus px caen en PARK_PINK_HSV; un cono rojo real es H med=0-1 /
# B-R=0.07 / 1% rosa, y su NÚCLEO está en la banda 1 [0,5] -> subir
# el piso de la banda 2 no lo afecta (solo pierde la cola H 173-176).
#
# 2026-09-16 (sede nueva, orillas1013): con la luz de ahí el cono
# rojo NO sale en H 0-1 sino en H 169-178 — la cámara corre con AWB
# apagado y ganancias fijas <1.2,1.5> medidas en el cuarto de
# pruebas, así que una luz más fría empuja el rojo hacia el
# magenta. Y cerca del carro llega a V 208 (el tope 160 también lo
# tiraba). Resultado: el cono grande que tenía ENFRENTE (frames
# 276-285, 18k px rojos) daba CERO bboxes y el carro no lo esquivó.
# color_corr.py no lo tapó: allá el piso sale QUEMADO (V~237, 33%
# de px a 255) y un piso quemado ya no tiene tono -> ganancias ~1.
#
# La banda 2 baja 177 -> 168 y los topes de V suben. Lo que antes
# protegía de la pared magenta (subir el piso de H) ya no puede
# hacerlo, así que esa defensa se movió a RED_BR_MAX en
# process_color(), que separa por B/R y NO depende de la luz:
# cono rojo real B/R 0.11-0.35, pared rosa 0.45-0.60.
# Verificado sobre .avi grabados: orillas1013 (sede) 72 -> 130
# frames con rojo y el cono de los frames 276-285 pasa de 0 a
# detectado; orillas942 (pared rosa) 124 -> 125 sin falsos de
# pared; orillas1005 (luz de siempre) idéntico.
#
# Competition green RGB(68,214,44) → HSV≈(56, 203, 214)
#
# 2026-11-04, Pi 5 (ISP PiSP + imx219_noir_wro_pi5.json, ganancias <1.10,1.37>):
# el rojo sale más pálido y rosado que en la Pi 4 y los filtros de arriba lo
# tiraban entero (0 detecciones a 50 cm). Medido en la pista, misma imagen
# (p5-p95 de px; mediana entre paréntesis):
#   rojo 50 cm      H 170-177  S 107-167 (144-158)  G/R (0.38-0.44)  B/R (0.47-0.54)
#   pared rosa      H 139      S 109-118            G/R 0.67         B/R 1.19
#   cinta naranja   H 0-10     S 50-109  (61-75)    G/R (0.76-0.78)  B/R (0.71-0.78)
#   verde           H 70-73    S 118-134            G/R 1.95
# En la Pi 5 la pared rosa ya no cae en la banda del rojo (H 139) y la cinta se
# separa por G/R y S. Valores de la Pi 5: S >= 110, RED_GR_MAX 0.60, RED_BR_MAX 0.70.
# La Pi 4 conserva los suyos.
PI5_CAMARA = os.path.isdir("/usr/share/libcamera/ipa/rpi/pisp")
_RED_S_MIN = (110, 110) if PI5_CAMARA else (150, 170)
COLOR_RANGES = {
    "Red": [(np.array([0, _RED_S_MIN[0], 40]), np.array([5, 255, 200])),
            (np.array([168, _RED_S_MIN[1], 40]), np.array([179, 255, 235]))],
    "Green": [(np.array([30, 35, 25]), np.array([85, 255, 255]))],
}


_HSV_MASK_KERNEL = []


def _hsv_mask_kernel():
    if not _HSV_MASK_KERNEL:
        from pure_pursuit.native import get_kernel
        vp = ctypes.c_void_p
        _HSV_MASK_KERNEL.append(get_kernel(
            "fox_hsv_masks", [ctypes.c_int64, ctypes.c_int32] + [vp] * 4, None))
    return _HSV_MASK_KERNEL[0]


class Vision:
    def __init__(self, cam_index=0, *, open_cam=True):
        """Inicializa la cámara y define los rangos de colores."""
        self.cap = open_camera(cam_index) if open_cam else None
        self.color_ranges = {
            k: [(lo.copy(), hi.copy()) for lo, hi in ranges]
            for k, ranges in COLOR_RANGES.items()
        }
        self.kernel = np.ones((3, 3), np.uint8)
        # True: máscaras por LUT de bits (ver hsv_masks); False: cv2.inRange por rango.
        # El runtime lo pone desde config.VISION_LUT.
        self.use_lut = True
        self._lut_key = None

    def _ranges_key(self):
        return tuple((c, tuple((tuple(int(v) for v in lo), tuple(int(v) for v in hi)) for lo, hi in rs))
                     for c, rs in self.color_ranges.items())

    def _build_lut(self, key):
        """LUT (256,1,3) uint8 de bits desde self.color_ranges: el bit i del canal c
        está prendido en v si lo_i[c] <= v <= hi_i[c] (lo mismo que cv2.inRange).
        Un pixel cae en el rango i si el bit i está en los 3 canales."""
        self._lut_key = key
        if sum(len(rs) for _c, rs in key) > 8:
            self._lut = None              # no caben en un byte: hsv_masks usa cv2.inRange
            return
        lut = np.zeros((256, 1, 3), np.uint8)
        bits = {}
        i = 0
        for color, rs in key:
            bits[color] = 0
            for lo, hi in rs:
                for c in range(3):
                    a, b = max(lo[c], 0), min(hi[c], 255)
                    if a <= b:
                        lut[a:b + 1, 0, c] |= 1 << i
                bits[color] |= 1 << i
                i += 1
        self._lut, self._lut_bits = lut, bits
        self._lut_colors = tuple(c for c, b in bits.items() if b)
        self._lut_color_bits = np.array([bits[c] for c in self._lut_colors], dtype=np.uint8)

    def hsv_masks(self, hsv):
        """{color: máscara 0/255} de self.color_ranges, idéntica a OR de cv2.inRange."""
        if self.use_lut:
            key = self._ranges_key()
            if key != self._lut_key:      # rangos nuevos (o cambiados en el sitio): se rearma
                self._build_lut(key)
        if not self.use_lut or self._lut is None:
            masks = {}
            for color, ranges in self.color_ranges.items():
                m = None
                for lower, upper in ranges:
                    r = cv2.inRange(hsv, lower, upper)
                    m = r if m is None else cv2.bitwise_or(m, r, dst=m)
                masks[color] = m
            return masks
        kern = _hsv_mask_kernel() if hsv.dtype == np.uint8 and hsv.ndim == 3 and hsv.shape[2] == 3 else None
        if kern is not None:
            hsv = np.ascontiguousarray(hsv)
            out = np.empty((len(self._lut_colors), *hsv.shape[:2]), dtype=np.uint8)
            kern(hsv.shape[0] * hsv.shape[1], len(self._lut_colors), hsv.ctypes.data,
                 self._lut.ctypes.data, self._lut_color_bits.ctypes.data, out.ctypes.data)
            masks = dict.fromkeys(self._lut_bits)
            masks.update(zip(self._lut_colors, out))
            return masks
        h, s, v = cv2.split(cv2.LUT(hsv, self._lut))
        cv2.bitwise_and(h, s, dst=h)
        cv2.bitwise_and(h, v, dst=h)
        return {color: (cv2.compare(cv2.bitwise_and(h, b), 0, cv2.CMP_GT) if b else None)   # sin rangos: None, igual que inRange
                for color, b in self._lut_bits.items()}

    # Tope de B/R para aceptar un blob como cono ROJO. Medido en frames:
    # cono rojo real 0.11-0.35 (el rojo del cono casi no tiene azul), pared
    # magenta del estacionamiento 0.45-0.60 (es rosa: mucho azul). A
    # diferencia del piso de H que se usaba antes, esta razón NO se mueve con
    # la temperatura de color de la sede — las dos se escalan parejo.
    RED_BR_MAX = 0.70 if PI5_CAMARA else 0.42   # Pi 5: ver COLOR_RANGES (2026-11-04)

    # 2026-09-22 (orillas1190, luz fuerte): falsos en cada vuelta, separados
    # por razones que no se mueven con la luz. Medido re-pasando vision sobre
    # 6 runs (1190/1189 hoy, 1013 sede, 1005/1064/1150 luz normal):
    #   - cinta NARANJA de esquina como Red: G/R 0.38-0.46 (tiene verde, por
    #     eso es naranja); cono rojo real G/R 0.05-0.13. Tope 0.28.
    #   - piso blanco/celeste y zona quemada del park como Green: S mediana
    #     37-44 (la banda Green acepta S>=35); cono verde real S 120-160.
    #     Piso 70.
    # 0 conos reales perdidos en las 6 runs; en 1005/1064/1150 casi no rechaza.
    RED_GR_MAX = 0.60 if PI5_CAMARA else 0.28   # Pi 5: ver COLOR_RANGES (2026-11-04)
    GREEN_S_MIN_MED = 70

    def process_color(self, frame, mask, color_name, bgr=None, hsv=None):
        """Encuentra contornos y devuelve posiciones.

        Filtra por solidez (area_contorno / area_bbox) para descartar formas
        delgadas y alargadas como líneas pintadas en el tapete, que tienen
        solidez baja. Una lata se ve como un blob compacto → solidez alta.
        Ademas descarta bounding boxes con aspect ratio extremo (muy
        anchos/planos), típico de una línea diagonal o casi horizontal.
        """
        min_area = getattr(self, "min_contour_area", 1000.0)
        if np.count_nonzero(mask) < min(500, min_area):
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []

        MIN_SOLIDITY = 0.2   # blob compacto (lata) ~0.7-0.9; línea delgada suele ser < 0.4
        MAX_ASPECT   = 2.2    # w/h o h/w máximo permitido antes de considerarlo "línea"

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            solidity = area / bbox_area if bbox_area > 0 else 0
            aspect = max(w, h) / max(1, min(w, h))

            # La cinta es angosta y poco sólida. Con la mira a 45° y el lente
            # de 120° una lata a 40–65 cm se ve alta (aspecto ~2.3–2.5) pero
            # compacta (solidez ~0.85); el tope 2.2 se la comía.
            if solidity < MIN_SOLIDITY:
                continue
            if aspect > MAX_ASPECT and solidity < 0.55:
                continue

            # Máscara del contorno SOLO dentro de su recuadro (mismos px y mismo
            # orden row-major que el frame completo -> medianas idénticas).
            if (color_name == "Red" and bgr is not None) or (color_name == "Green" and hsv is not None):
                cnt_mask = np.zeros((h, w), np.uint8)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1, offset=(-x, -y))
                sel = cnt_mask > 0
                sel_m = sel & (mask[y:y + h, x:x + w] > 0)

            # Rechazo de la pared magenta del estacionamiento por B/R (ver
            # RED_BR_MAX). Se mide solo sobre los px del contorno, no del bbox.
            if color_name == "Red" and bgr is not None:
                bgr_r = bgr[y:y + h, x:x + w]
                b_px = bgr_r[..., 0][sel].astype(np.float32)
                r_px = bgr_r[..., 2][sel].astype(np.float32)
                if float(np.median(b_px / np.maximum(r_px, 1.0))) > self.RED_BR_MAX:
                    continue
                g_px = bgr_r[..., 1][sel_m].astype(np.float32)
                r_m = np.maximum(bgr_r[..., 2][sel_m].astype(np.float32), 1.0)
                if g_px.size and float(np.median(g_px / r_m)) > self.RED_GR_MAX:
                    continue   # cinta naranja (ver RED_GR_MAX)

            if color_name == "Green" and hsv is not None:
                if sel_m.any() and float(np.median(hsv[y:y + h, x:x + w, 1][sel_m])) < self.GREEN_S_MIN_MED:
                    continue   # piso claro / zona quemada (ver GREEN_S_MIN_MED)

            # En modo sorpresa se admiten blobs pequeños solo si su altura
            # física es compatible con el bloque bajo. Los normales conservan
            # el umbral de siempre; no se añaden señales lejanas al mapa.
            small_filter = getattr(self, "small_object_filter", None)
            if area <= 1000 and small_filter is not None and not small_filter((x, y, w, h)):
                continue
            objects.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return objects

    def detect_on(self, frame):
        """Detecta colores SIN voltear el frame (ya está en el espacio de trabajo)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.last_hsv = hsv   # el runtime la usa para saber si hay naranja antes de hacer el BEV
        # El original queda limpio para el filtro B/R del rojo; los recuadros
        # se dibujan en una sola copia, sin modificar el buffer de captura.
        bgr = frame
        frame = frame.copy()

        masks = self.hsv_masks(hsv)

        positions = {color: self.process_color(frame, mask, color, bgr, hsv)
                     for color, mask in masks.items()}

        return frame, positions

    def process_frame(self, frame):
        """Detecta colores optimizado con NumPy."""
        frame = cv2.flip(frame, 1)
        return self.detect_on(frame)
