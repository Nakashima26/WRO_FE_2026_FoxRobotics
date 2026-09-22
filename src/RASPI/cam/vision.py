import os

import cv2
import numpy as np


cv2.setUseOptimized(True)
cv2.setNumThreads(min(4, os.cpu_count() or 1))


def open_camera(cam_index=0):
    if os.name == "nt":
        cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)
    else:
        pipeline = (
            "libcamerasrc awb-enable=false colour-gains=<1.2,1.5> "
            "! queue max-size-buffers=1 leaky=downstream "
            "! video/x-raw, width=1640, height=1232, framerate=30/1 "
            "! videoconvert ! videoscale ! video/x-raw, width=640, height=480, format=BGR "
            "! appsink drop=true max-buffers=1 sync=false"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
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
COLOR_RANGES = {
    "Red": [(np.array([0, 150, 40]), np.array([5, 255, 200])),
            (np.array([168, 170, 40]), np.array([179, 255, 235]))],
    "Green": [(np.array([30, 35, 25]), np.array([85, 255, 255]))],
}


class Vision:
    def __init__(self, cam_index=0, *, open_cam=True):
        """Inicializa la cámara y define los rangos de colores."""
        self.cap = open_camera(cam_index) if open_cam else None
        self.color_ranges = {
            k: [(lo.copy(), hi.copy()) for lo, hi in ranges]
            for k, ranges in COLOR_RANGES.items()
        }
        self.kernel = np.ones((3, 3), np.uint8)

    # Tope de B/R para aceptar un blob como cono ROJO. Medido en frames:
    # cono rojo real 0.11-0.35 (el rojo del cono casi no tiene azul), pared
    # magenta del estacionamiento 0.45-0.60 (es rosa: mucho azul). A
    # diferencia del piso de H que se usaba antes, esta razón NO se mueve con
    # la temperatura de color de la sede — las dos se escalan parejo.
    RED_BR_MAX = 0.42

    # 2026-09-22 (orillas1190, luz fuerte): falsos en cada vuelta, separados
    # por razones que no se mueven con la luz. Medido re-pasando vision sobre
    # 6 runs (1190/1189 hoy, 1013 sede, 1005/1064/1150 luz normal):
    #   - cinta NARANJA de esquina como Red: G/R 0.38-0.46 (tiene verde, por
    #     eso es naranja); cono rojo real G/R 0.05-0.13. Tope 0.28.
    #   - piso blanco/celeste y zona quemada del park como Green: S mediana
    #     37-44 (la banda Green acepta S>=35); cono verde real S 120-160.
    #     Piso 70.
    # 0 conos reales perdidos en las 6 runs; en 1005/1064/1150 casi no rechaza.
    RED_GR_MAX = 0.28
    GREEN_S_MIN_MED = 70

    def process_color(self, frame, mask, color_name, bgr=None, hsv=None):
        """Encuentra contornos y devuelve posiciones.

        Filtra por solidez (area_contorno / area_bbox) para descartar formas
        delgadas y alargadas como líneas pintadas en el tapete, que tienen
        solidez baja. Una lata se ve como un blob compacto → solidez alta.
        Ademas descarta bounding boxes con aspect ratio extremo (muy
        anchos/planos), típico de una línea diagonal o casi horizontal.
        """
        if np.count_nonzero(mask) < 500:
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []

        MIN_SOLIDITY = 0.2   # blob compacto (lata) ~0.7-0.9; línea delgada suele ser < 0.4
        MAX_ASPECT   = 2.2    # w/h o h/w máximo permitido antes de considerarlo "línea"

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 1000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            solidity = area / bbox_area if bbox_area > 0 else 0
            aspect = max(w, h) / max(1, min(w, h))

            if solidity < MIN_SOLIDITY or aspect > MAX_ASPECT:
                continue

            # Rechazo de la pared magenta del estacionamiento por B/R (ver
            # RED_BR_MAX). Se mide solo sobre los px del contorno, no del bbox.
            if color_name == "Red" and bgr is not None:
                cnt_mask = np.zeros(mask.shape, np.uint8)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
                sel = cnt_mask > 0
                b_px = bgr[..., 0][sel].astype(np.float32)
                r_px = bgr[..., 2][sel].astype(np.float32)
                if float(np.median(b_px / np.maximum(r_px, 1.0))) > self.RED_BR_MAX:
                    continue
                sel_m = sel & (mask > 0)
                g_px = bgr[..., 1][sel_m].astype(np.float32)
                r_m = np.maximum(bgr[..., 2][sel_m].astype(np.float32), 1.0)
                if g_px.size and float(np.median(g_px / r_m)) > self.RED_GR_MAX:
                    continue   # cinta naranja (ver RED_GR_MAX)

            if color_name == "Green" and hsv is not None:
                cnt_mask = np.zeros(mask.shape, np.uint8)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
                sel = (cnt_mask > 0) & (mask > 0)
                if sel.any() and float(np.median(hsv[..., 1][sel])) < self.GREEN_S_MIN_MED:
                    continue   # piso claro / zona quemada (ver GREEN_S_MIN_MED)

            objects.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return objects

    def detect_on(self, frame):
        """Detecta colores SIN voltear el frame (ya está en el espacio de trabajo)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Copia limpia para el filtro B/R del rojo: `frame` se va ensuciando
        # con los rectángulos blancos que dibuja process_color().
        bgr = frame.copy()

        masks = {color: np.bitwise_or.reduce([cv2.inRange(hsv, lower, upper) for lower, upper in ranges])
                 for color, ranges in self.color_ranges.items()}

        positions = {color: self.process_color(frame, mask, color, bgr, hsv)
                     for color, mask in masks.items()}

        return frame, positions

    def process_frame(self, frame):
        """Detecta colores optimizado con NumPy."""
        frame = cv2.flip(frame, 1)
        return self.detect_on(frame)