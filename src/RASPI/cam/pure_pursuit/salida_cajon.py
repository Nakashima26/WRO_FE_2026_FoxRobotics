"""Lado de paso de la 1ª señal adelante del cajón, para la salida (INICIO).

El carro arranca con el costado al ras del borde interior del cajón. En la recta
del cajón solo hay señales en la fila INTERIOR (T1, X1, T2; el reglamento prohíbe
T3/T4/X2 ahí), así que la salida tiene dos formas:
  - afuera (isal=1): la señal se pasa entre ella y la pared exterior -> basta
    una S corta;
  - adentro (isal=2): hay que cruzar al carril interior, más allá de la fila
    (rojo con la isla a la derecha = CW, verde con la isla a la izquierda = CCW)
    -> el ESP abre la S a ~90° (PurePursuit.ino `case INICIO`).
No necesita saber el sentido de la pista: la fila interior queda del lado de la
isla, y el lado de la lata respecto al carro + su color deciden.

Mientras el ESP está en INICIO (est=I) se proyecta cada pie de lata del BEV al
marco de arranque del odómetro (px adelante, py izquierda, yaw izquierda +) y se
vota. Las cuentas de votos son evidencia (no ventanas de tiempo): no se escalan
con C.PI_FPS.
"""
from __future__ import annotations

import math

from . import config as C

_BEV_AHEAD_MM = float(getattr(C, "BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM", 100.0))
_CAM_AHEAD_MM = float(getattr(C, "CAMERA_FWD_MM", 140.0))
_FOOT_TO_CENTER_MM = float(getattr(C, "SIGN_FOOT_TO_CENTER_MM", 25.0))

# Ventana en el marco de arranque (eje trasero). Fila interior a ~467 mm del eje
# (60 cm del borde exterior, carro al ras de la madera de 20 cm). Adelante: T2 en
# CCW a ~270 mm, X1 en CW a ~440, T1 en CW a ~940. T2 en CW (~-60) queda atrás.
# T1 queda fuera a propósito: con ~70 cm por delante el PP cruza solo, y la S
# corta de siempre deja el carro igual que antes para la 1ª esquina.
SALIDA_ALONG_MIN_MM = 120.0
SALIDA_ALONG_MAX_MM = 700.0
SALIDA_LAT_MIN_MM = 300.0
SALIDA_LAT_MAX_MM = 700.0
SALIDA_BIN_MM = 200.0        # agrupa los pies por asiento a lo largo de la recta
SALIDA_VOTES = 3             # pies del mismo color en el mismo asiento para decidir


def _num(ack: str, key: str) -> float | None:
    i = ack.find("," + key + "=")
    if i < 0:
        return None
    s = ack[i + len(key) + 2:].split(",", 1)[0]
    try:
        return float(s)
    except ValueError:
        return None


class SalidaCajon:
    def __init__(self) -> None:
        self.side = 0                     # 0 sin dato | 1 afuera | 2 adentro
        self._votes: dict[tuple[int, int], dict[str, int]] = {}
        self.decided: tuple[float, float, str] | None = None

    def update(self, ack: str | None, feet) -> int:
        if self.side or not ack or ",est=I" not in ack:
            return self.side
        px, py, yaw = _num(ack, "px"), _num(ack, "py"), _num(ack, "yaw")
        if px is None or py is None or yaw is None:
            return self.side
        ps = math.radians(yaw)
        cs, sn = math.cos(ps), math.sin(ps)
        for bx, by, color in feet or []:
            if color not in ("Red", "Green"):
                continue
            lat = (bx - C.ROBOT_BEV_X) * C.MM_PER_PX        # + derecha del carro
            fwd = (C.ROBOT_BEV_Y - by) * C.MM_PER_PX
            if fwd < 80.0 or fwd > 1800.0:
                continue
            fwd += _BEV_AHEAD_MM                              # hoja -> eje trasero
            ray = math.hypot(fwd - _CAM_AHEAD_MM, lat)
            if ray > 1e-6:                                    # pie -> centro de la lata
                fwd += _FOOT_TO_CENTER_MM * (fwd - _CAM_AHEAD_MM) / ray
                lat += _FOOT_TO_CENTER_MM * lat / ray
            x = px + fwd * cs + lat * sn                      # marco de arranque
            y = py + fwd * sn - lat * cs
            if not (SALIDA_ALONG_MIN_MM <= x <= SALIDA_ALONG_MAX_MM
                    and SALIDA_LAT_MIN_MM <= abs(y) <= SALIDA_LAT_MAX_MM):
                continue
            key = (int(round(x / SALIDA_BIN_MM)), 1 if y > 0 else -1)
            v = self._votes.setdefault(key, {"Red": 0, "Green": 0})
            v[color] += 1
        # La más cercana a lo largo de la recta con mayoría clara decide.
        for key in sorted(self._votes):
            v = self._votes[key]
            for color, other in (("Red", "Green"), ("Green", "Red")):
                if v[color] >= SALIDA_VOTES and v[color] > 2 * v[other]:
                    left = key[1] > 0
                    # Rojo se pasa por la derecha, verde por la izquierda. Si
                    # ese lado es el de la isla (el de la lata), toca cruzar.
                    inner = (color == "Red") != left
                    self.side = 2 if inner else 1
                    self.decided = (key[0] * SALIDA_BIN_MM, key[1], color)
                    print(f"[SALIDA] {color} along~{key[0] * SALIDA_BIN_MM:.0f} "
                          f"{'izq' if left else 'der'} votos={v} -> isal={self.side} "
                          f"({'adentro' if inner else 'afuera'})", flush=True)
                    return self.side
            if v["Red"] + v["Green"] >= SALIDA_VOTES:
                break   # asiento más cercano sin mayoría: no decidir por uno más lejano
        return self.side
