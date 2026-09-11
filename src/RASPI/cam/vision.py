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


class Vision:
    def __init__(self, cam_index=0):
        """Inicializa la cámara y define los rangos de colores."""
        self.cap = open_camera(cam_index)

        self.color_ranges = {
            # Banda 2 (cola azulada del rojo, wrap-around): H 173 -> 177.
            # La pared magenta del estacionamiento (rectas 1/5/9 y tras el 4o
            # giro) cae en H 172-175 con S~164 V~81 -> la banda 173-176 la
            # detectaba como cono Red y el carro daba un volantazo a "esquivarla"
            # (orillas809 ~seg 25: bbox Red sobre el panel magenta, mem R(x=241),
            # steer +46->+50). Medido en frames: pared H med=173 / B-R=0.50 / 87%
            # de sus px caen en PARK_PINK_HSV; un cono rojo real es H med=0-1 /
            # B-R=0.07 / 1% rosa, y su NÚCLEO está en la banda 1 [0,5] -> subir
            # el piso de la banda 2 no lo afecta (solo pierde la cola H 173-176).
            "Red": [(np.array([0, 150, 40]), np.array([5, 255, 160])),
                        (np.array([177, 150, 40]), np.array([179, 255, 160]))],
            # Competition green RGB(68,214,44) → HSV≈(56, 203, 214)
            "Green": [(np.array([35,60,40]), np.array([75, 255, 200]))],           
            # "Pink": [(np.array([140, 100, 100]), np.array([170, 255, 255]))],
        }

        self.kernel = np.ones((3, 3), np.uint8)

        # Umbral de área POR DISTANCIA (opcional, lo instala runtime_nuevo con la
        # calibración BEV). Un cono lejano es un blob chico: a ~45 cm mide
        # 300-700 px y el umbral fijo MIN_AREA lo tiraba hasta que quedaba a
        # ~20 cm (orillas845 vueltas 2-3: el rojo tras el verde entraba a memoria
        # a 21 cm -> cruce de carril a 70°). area_min_fn(x, y, w, h, color) ->
        # área mínima para un blob MENOR a MIN_AREA (inf = no cuenta); los >=
        # MIN_AREA pasan igual que siempre. None = sin cambio.
        self.area_min_fn = None
        self.area_floor = self.MIN_AREA       # piso absoluto para blobs chicos
        self.small_min_solidity = 0.45        # los chicos deben ser compactos
        self.last_small = []                  # blobs chicos aceptados en el último frame (diag)

    MIN_AREA = 1000

    def process_color(self, frame, mask, color_name):
        """Encuentra contornos y devuelve posiciones.

        Filtra por solidez (area_contorno / area_bbox) para descartar formas
        delgadas y alargadas como líneas pintadas en el tapete, que tienen
        solidez baja. Una lata se ve como un blob compacto → solidez alta.
        Ademas descarta bounding boxes con aspect ratio extremo (muy
        anchos/planos), típico de una línea diagonal o casi horizontal.
        """
        floor = self.MIN_AREA if self.area_min_fn is None else min(self.MIN_AREA, self.area_floor)
        if np.count_nonzero(mask) < min(500, floor):
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []

        MIN_SOLIDITY = 0.2   # blob compacto (lata) ~0.7-0.9; línea delgada suele ser < 0.4
        MAX_ASPECT   = 2.2    # w/h o h/w máximo permitido antes de considerarlo "línea"

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= floor:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            solidity = area / bbox_area if bbox_area > 0 else 0
            aspect = max(w, h) / max(1, min(w, h))

            if solidity < MIN_SOLIDITY or aspect > MAX_ASPECT:
                continue

            # Blob chico: solo cuenta si es compacto y está lo bastante LEJOS
            # para que su tamaño sea el de un cono (ver area_min_fn).
            if area <= self.MIN_AREA:
                if self.area_min_fn is None or solidity < self.small_min_solidity:
                    continue
                if area <= self.area_min_fn(x, y, w, h, color_name):
                    continue
                self.last_small.append((color_name, x, y, w, h, int(area)))

            objects.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return objects

    def process_frame(self, frame):
        """Detecta colores optimizado con NumPy."""
        frame = cv2.flip(frame, 1)
        self.last_small = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = {color: np.bitwise_or.reduce([cv2.inRange(hsv, lower, upper) for lower, upper in ranges])
                 for color, ranges in self.color_ranges.items()}

        positions = {color: self.process_color(frame, mask, color) for color, mask in masks.items()}

        return frame, positions