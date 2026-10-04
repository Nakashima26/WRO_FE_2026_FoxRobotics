"""
Hint direccional para obstáculos lejanos — vistos en la imagen de cámara CRUDA
pero que aún no proyectan dentro del rango BEV calibrado.

"""

from . import config as C


class FarHintPD:
    """Controlador P (o PD) stateful para el hint de centrado anticipado."""

    def __init__(self, kp: float | None = None, kd: float | None = None):
        self.kp = C.FAR_HINT_KP if kp is None else kp
        # La derivada es la diferencia entre frames (sin /dt): a más fps sale
        # más chica para el mismo movimiento.
        self.kd = (C.FAR_HINT_KD if kd is None else kd) * C.PI_FPS / C.FPS_NOMINAL
        self._prev_error = 0.0
        self._has_prev = False

    def compute(self, offset_px: float) -> float:
        derivative = (offset_px - self._prev_error) if self._has_prev else 0.0
        self._prev_error = offset_px
        self._has_prev = True
        return (self.kp * offset_px) + (self.kd * derivative)

    def reset(self):
        self._prev_error = 0.0
        self._has_prev = False


class FarHintManager:
    """
    Mantiene un FarHintPD por color.  Cada frame, solo el objeto lejano de
    mayor área "gana" y alimenta su PD; los demás colores se resetean.
    """

    def __init__(self):
        self._pd = {
            "Red":   FarHintPD(),
            "Green": FarHintPD(),
        }

    def compute(self, far_objects: list[tuple[float, float, float, str]]) -> float:
        """
        far_objects: lista de (obj_center_x, w, h, color_str) — SOLO objetos
                     detectados en cámara que NO proyectaron a BEV este frame.

        Retorna: steer_hint_deg acotado a ±FAR_HINT_MAX_STEER.
        """
        valid = [obj for obj in far_objects if obj[1] * obj[2] >= C.FAR_HINT_MIN_AREA_PX]

        if not valid:
            self.reset_all()
            return 0.0

        # Gana el objeto más grande (≈ más cercano), sin importar color.
        obj_center_x, w, h, color = max(valid, key=lambda o: o[1] * o[2])

        sign = +1.0 if color == "Red" else -1.0
        offset_px = obj_center_x - C.CAM_CENTER_X
        hint = sign * self._pd[color].compute(offset_px)

        # Resetear el PD de los colores que NO ganaron este frame, para que
        # no arrastren una derivada vieja si vuelven a aparecer después.
        for c, pd in self._pd.items():
            if c != color:
                pd.reset()

        return max(-C.FAR_HINT_MAX_STEER, min(C.FAR_HINT_MAX_STEER, hint))

    def reset_all(self):
        for pd in self._pd.values():
            pd.reset()