"""
color_corr.py — Corrección de color por el PISO (balance de blancos por software).

Problema: la cámara corre con el balance de blancos APAGADO y ganancias fijas
(vision.py: awb-enable=false colour-gains=<1.2,1.5>) y todos los rangos HSV del
proyecto (conos, cinta naranja, piso, rosa) son números fijos medidos con la luz
del cuarto de pruebas. Con otra luz los tonos se corren: en orillas900 (cuarto
más oscuro) la cinta naranja bajó de H~9 a H~4 y cayó DENTRO del rango rojo.

Idea: el tapete es el mismo en todas las sedes. En cada actualización se mide el
color del piso en la franja de abajo del frame (lo que está justo frente al
carro) y se escala B,G,R para que ese piso quede del color que tenía en el cuarto
de pruebas (COLOR_CORR_FLOOR_REF_BGR). Con la luz de siempre las ganancias salen
~1 y no cambia nada; con otra luz regresa los colores a donde los rangos los
esperan.

Costo (medido en la Pi 4):
  - cada frame      : cv2.LUT sobre el frame (tabla de 256 valores por canal),
                      ~1.65 ms. Si las ganancias están a <SKIP_DELTA de 1 (luz de
                      siempre) no se aplica: 0 ms.
  - cada EVERY_N    : mediana del piso sobre la franja submuestreada (1 de cada
                      4 px en x e y) + reconstruir la tabla si cambió, ~1.3 ms.
Nada se calcula "una sola vez al principio": se va ajustando despacio (EMA), así
aguanta zonas con distinta luz en la misma pista. Si la franja no parece piso
(cono enfrente, pared muy cerca, lote rosa) se salta la actualización y se
conserva la última ganancia buena.
"""

import time

import cv2
import numpy as np

from . import config as C


class FloorColorCorrector:
    def __init__(self):
        self.enabled   = bool(getattr(C, "COLOR_CORR_ENABLED", True))
        self.ref       = np.array(getattr(C, "COLOR_CORR_FLOOR_REF_BGR", (175.0, 198.0, 213.0)),
                                  dtype=np.float32)
        self.every_n   = max(1, int(getattr(C, "COLOR_CORR_EVERY_N", 5)))
        self.alpha     = float(getattr(C, "COLOR_CORR_ALPHA", 0.3))
        self.roi_top   = float(getattr(C, "COLOR_CORR_ROI_TOP", 0.70))
        self.s_max     = int(getattr(C, "COLOR_CORR_FLOOR_S_MAX", 90))
        self.v_min     = int(getattr(C, "COLOR_CORR_FLOOR_V_MIN", 60))
        self.min_frac  = float(getattr(C, "COLOR_CORR_MIN_FLOOR_FRAC", 0.5))
        self.gain_min  = float(getattr(C, "COLOR_CORR_GAIN_MIN", 0.6))
        self.gain_max  = float(getattr(C, "COLOR_CORR_GAIN_MAX", 1.8))
        self.brillo_max = float(getattr(C, "COLOR_CORR_BRILLO_MAX", 1.2))
        self.skip_delta = float(getattr(C, "COLOR_CORR_SKIP_DELTA", 0.02))
        self.log_every = float(getattr(C, "COLOR_CORR_LOG_EVERY_S", 2.0))

        self.gains      = np.ones(3, dtype=np.float32)
        self._lut       = None          # None = identidad (todavía no hay medida buena)
        self._lut_gains = None
        self._n         = 0
        self._last_log  = 0.0
        self._ms_acc    = 0.0
        self._ms_cnt    = 0
        # Diagnóstico
        self.last_floor = None          # BGR crudo del piso en la última medida buena
        self.last_frac  = 0.0
        self.updates    = 0
        self.skipped    = 0

    def process(self, frame):
        """Actualiza (cada EVERY_N) y aplica la corrección. Devuelve el frame corregido."""
        if not self.enabled or frame is None or getattr(frame, "size", 0) == 0:
            return frame
        t0 = time.perf_counter()
        if self._n % self.every_n == 0:
            self._update(frame)
        self._n += 1
        out = frame if self._lut is None else cv2.LUT(frame, self._lut)
        self._ms_acc += (time.perf_counter() - t0) * 1000.0
        self._ms_cnt += 1
        self._maybe_log()
        return out

    def _update(self, frame):
        h = frame.shape[0]
        roi = frame[int(h * self.roi_top)::4, ::4]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        floor = ((hsv[..., 1] <= self.s_max)
                 & (hsv[..., 2] >= self.v_min)
                 & (hsv[..., 2] <= 250))          # quemado no dice nada del color
        frac = float(floor.mean())
        self.last_frac = frac
        if frac < self.min_frac:
            self.skipped += 1
            return
        med = np.median(roi[floor].reshape(-1, 3), axis=0).astype(np.float32)
        g = self.ref / np.maximum(med, 1.0)
        # El TONO lo dan las proporciones entre canales; la media es el brillo. Se
        # corrige el tono completo pero el brillo solo sube hasta BRILLO_MAX: con
        # más, los conos rojos reales pasan el tope V<=160 del rango Red y se
        # pierden (orillas900 f382: V 130 -> 158-207 con ganancia 1.2-1.5).
        m = float(g.mean())
        if m > self.brillo_max:
            g = g * (self.brillo_max / m)
        g = np.clip(g, self.gain_min, self.gain_max)
        if self.updates == 0:
            self.gains = g                       # 1ª medida: corrige de inmediato
        else:
            self.gains = (1.0 - self.alpha) * self.gains + self.alpha * g
        self.last_floor = med
        self.updates += 1
        if self._lut_gains is None or float(np.max(np.abs(self.gains - self._lut_gains))) > 0.005:
            self._lut_gains = self.gains.copy()
            if float(np.max(np.abs(self.gains - 1.0))) < self.skip_delta:
                self._lut = None                 # ~identidad (luz de siempre): no gasta el LUT
            else:
                ramp = np.arange(256, dtype=np.float32)
                lut = np.stack([np.clip(ramp * self.gains[c], 0, 255) for c in range(3)], axis=-1)
                self._lut = lut.astype(np.uint8).reshape(1, 256, 3)

    def _maybe_log(self):
        if self.log_every <= 0:
            return
        now = time.monotonic()
        if now - self._last_log < self.log_every:
            return
        self._last_log = now
        ms = self._ms_acc / max(1, self._ms_cnt)
        self._ms_acc, self._ms_cnt = 0.0, 0
        fl = ("-" if self.last_floor is None
              else f"({self.last_floor[0]:.0f},{self.last_floor[1]:.0f},{self.last_floor[2]:.0f})")
        print(f"[COLOR] piso_BGR={fl} ref=({self.ref[0]:.0f},{self.ref[1]:.0f},{self.ref[2]:.0f}) "
              f"gan=({self.gains[0]:.2f},{self.gains[1]:.2f},{self.gains[2]:.2f}) "
              f"frac_piso={self.last_frac:.2f} upd={self.updates} skip={self.skipped} "
              f"ms/frame={ms:.2f}", flush=True)
