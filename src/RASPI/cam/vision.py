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
            "Red": [(np.array([0, 150, 40]), np.array([5, 255, 160])),
                        (np.array([173, 150, 40]), np.array([179, 255, 160]))],
            # Competition green RGB(68,214,44) → HSV≈(56, 203, 214)
            "Green": [(np.array([35,60,40]), np.array([75, 255, 200]))],           
            # "Pink": [(np.array([140, 100, 100]), np.array([170, 255, 255]))],
        }

        self.kernel = np.ones((3, 3), np.uint8)

    def process_color(self, frame, mask, color_name):
        """Encuentra contornos y devuelve posiciones.

        Filtra por solidez (area_contorno / area_bbox) para descartar formas
        delgadas y alargadas como líneas pintadas en el tapete, que tienen
        solidez baja. Una lata se ve como un blob compacto → solidez alta.
        Ademas descarta bounding boxes con aspect ratio extremo (muy
        anchos/planos), típico de una línea diagonal o casi horizontal.

        Y para Red: descarta blobs que NO bajan del ~40% superior del frame.
        La pared rosa/magenta del estacionamiento (rectas 1/5/9) cae dentro del
        rango Red (su parte oscura/sombreada lee hue 0-5) y su blob vive SIEMPRE
        pegado al borde superior del frame (bottom del bbox muy arriba,
        orillas808 ~45s). Un cono rojo real se apoya en el PISO: su bbox baja
        mucho más, y uno "en la cara" llega casi al borde inferior -> este
        umbral no lo puede descartar. SOLO Red (no hay backdrop verde en pista).
        """
        if np.count_nonzero(mask) < 500:
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []

        MIN_SOLIDITY = 0.2   # blob compacto (lata) ~0.7-0.9; línea delgada suele ser < 0.4
        MAX_ASPECT   = 2.2    # w/h o h/w máximo permitido antes de considerarlo "línea"
        RED_MIN_BBOX_BOTTOM = int(0.40 * frame.shape[0])   # ver docstring

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 1000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            solidity = area / bbox_area if bbox_area > 0 else 0
            aspect = max(w, h) / max(1, min(w, h))

            # DIAG 2026-09-09 (solo-log): TODO contorno rojo con área real, para
            # calibrar el umbral RED_MIN_BBOX_BOTTOM contra lo que ve el detector
            # EN VIVO. Quitar cuando el gate esté afinado en el tapete.
            if color_name == "Red":
                _why = ("drop-sol" if solidity < MIN_SOLIDITY
                        else "drop-asp" if aspect > MAX_ASPECT
                        else "drop-top" if (y + h) < RED_MIN_BBOX_BOTTOM
                        else "KEEP")
                print(f"[REDBLOB] x={x} y={y} w={w} h={h} bottom={y + h}/{frame.shape[0]} "
                      f"area={area:.0f} sol={solidity:.2f} asp={aspect:.2f} "
                      f"thr={RED_MIN_BBOX_BOTTOM} -> {_why}", flush=True)

            if solidity < MIN_SOLIDITY or aspect > MAX_ASPECT:
                continue

            if color_name == "Red" and (y + h) < RED_MIN_BBOX_BOTTOM:
                continue   # blob pegado al borde superior = pared del estacionamiento, no un cono

            objects.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return objects

    def process_frame(self, frame):
        """Detecta colores optimizado con NumPy."""
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = {color: np.bitwise_or.reduce([cv2.inRange(hsv, lower, upper) for lower, upper in ranges])
                 for color, ranges in self.color_ranges.items()}

        positions = {color: self.process_color(frame, mask, color) for color, mask in masks.items()}

        return frame, positions