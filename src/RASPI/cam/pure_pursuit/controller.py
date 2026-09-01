"""
Controlador geométrico Pure Pursuit operando en espacio de píxeles BEV.
...
"""

import math

from . import config as C


class PurePursuitController:
    """
    Controlador Pure Pursuit puro.
    Opera en coordenadas BEV donde el robot siempre apunta "hacia arriba"
    (Y decreciente = dirección de marcha).
    """

    def __init__(self):
        # Último steer_deg entregado por compute(), para el límite de slew.
        self._prev_steer_deg: float = 0.0

    @staticmethod
    def adaptive_lookahead(
        bev_obstacles: list[tuple[float, float, str]],
        robot_x: int = C.ROBOT_BEV_X,
        robot_y: int = C.ROBOT_BEV_Y,
    ) -> float:
        """
        Calcula el lookahead efectivo según qué tan cerca está el obstáculo
        más próximo al robot.

        Sin obstáculos, o con el más cercano lejos  -> LOOKAHEAD_MAX_PX
        (trayectoria suave, estable en recta/curvas normales).

        Con un obstáculo a <= LOOKAHEAD_OBS_NEAR_PX  -> LOOKAHEAD_MIN_PX
        (apunta a un punto cercano del path -> geometría exige steer más
        cerrado -> giro fuerte para esquivar de inmediato).

        Entre ambos umbrales, interpola linealmente para que la transición
        no sea un salto brusco de steer.
        """
        if not bev_obstacles:
            return C.LOOKAHEAD_MAX_PX

        # Distancia LONGITUDINAL (px por delante del eje), NO euclidiana. Con la
        # euclidiana, un obstáculo aún de lado (offset lateral grande) mantenía
        # nearest_d alto -> lookahead largo -> PP apuntaba lejos -> steer suave ->
        # el carro seguía DERECHO hasta tener el cono encima y ahí giraba tarde
        # (rozaba, orillas456). Con la longitudinal el lookahead se acorta cuando
        # el obstáculo se acerca DE FRENTE, sin importar cuánto lo tenga de lado.
        nearest_d = min(
            max(0.0, robot_y - oy) for ox, oy, _ in bev_obstacles
        )

        if nearest_d <= C.LOOKAHEAD_OBS_NEAR_PX:
            return C.LOOKAHEAD_MIN_PX
        if nearest_d >= C.LOOKAHEAD_OBS_FAR_PX:
            return C.LOOKAHEAD_MAX_PX

        t = (nearest_d - C.LOOKAHEAD_OBS_NEAR_PX) / (
            C.LOOKAHEAD_OBS_FAR_PX - C.LOOKAHEAD_OBS_NEAR_PX
        )
        return C.LOOKAHEAD_MIN_PX + t * (C.LOOKAHEAD_MAX_PX - C.LOOKAHEAD_MIN_PX)

    @staticmethod
    def _distance_steer_gain(bev_obstacles, robot_x, robot_y) -> float:
        if not bev_obstacles:
            return 1.0
        # LONGITUDINAL (px por delante del eje), no euclidiana — mismo motivo que
        # adaptive_lookahead: con la euclidiana un obstáculo aún de lado mantenía
        # nearest_d alto y el steer se atenuaba al 30% -> el carro iba derecho y
        # esquivaba tarde (rozaba, orillas456).
        nearest_d = min(max(0.0, robot_y - oy) for ox, oy, _ in bev_obstacles)
        if nearest_d <= C.STEER_DIST_GAIN_NEAR_PX:
            return 1.0
        if nearest_d >= C.STEER_DIST_GAIN_FAR_PX:
            return C.STEER_DIST_GAIN_MIN
        t = (nearest_d - C.STEER_DIST_GAIN_NEAR_PX) / (C.STEER_DIST_GAIN_FAR_PX - C.STEER_DIST_GAIN_NEAR_PX)
        return 1.0 - t * (1.0 - C.STEER_DIST_GAIN_MIN)

    def compute(
        self,
        path_points: list[tuple[int, int]],
        robot_x: int = C.ROBOT_BEV_X,
        robot_y: int = C.ROBOT_BEV_Y,
        lookahead_px: float = C.LOOKAHEAD_PX,
        bev_obstacles: list[tuple[float, float, str]] | None = None,
    ) -> tuple[float, tuple[float, float]]:
        if not path_points:
            return 0.0, (float(robot_x), float(robot_y))

        # NOTA: el sesgo de paso WRO (pasar a la derecha del rojo / izquierda
        # del verde) ya está horneado DENTRO de path_points por
        # centerline.py (_pass_side_cx + blend por best_w) -- este método NO
        # necesita buscar el punto "más lateral posible" para respetarlo.
        #
        # Antes, con bev_obstacles, se elegía el candidato con mayor
        # |offset_x| dentro del anillo de lookahead. Eso agarraba con la
        # misma prioridad un pico real de esquiva Y un pico de RUIDO de una
        # sola fila (p.ej. cuando el hueco libre más ancho de esa fila cae
        # del lado contrario al reglamentario y el "probe" que empuja cx
        # hacia el lado correcto tiene que recorrer mucho para salir de la
        # zona bloqueada -- ver _enforce_free_mask/blend en centerline.py).
        # Ese pico sobrevive el suavizado y termina siendo el target, dando
        # un steer pegado al límite mecánico y un lookahead visualmente
        # "colgado" del resto del path.
        #
        # Selección por distancia más cercana al lookahead ideal (igual que
        # la rama sin obstáculos) es robusta a ese ruido: un solo punto
        # desviado no gana solo por ser el más lateral, tiene que además
        # caer a la distancia correcta.
        if bev_obstacles:
            lo, hi = lookahead_px * 0.85, lookahead_px * 1.25
            candidates = [
                pt for pt in path_points
                if lo <= math.hypot(pt[0] - robot_x, pt[1] - robot_y) <= hi
            ]
            if candidates:
                target = min(
                    candidates,
                    key=lambda pt: abs(math.hypot(pt[0] - robot_x, pt[1] - robot_y) - lookahead_px),
                )
            else:
                target = path_points[-1]
                for pt in path_points:
                    if math.hypot(pt[0] - robot_x, pt[1] - robot_y) >= lookahead_px:
                        target = pt
                        break
        else:
            target = path_points[-1]
            for pt in path_points:
                dist = math.hypot(pt[0] - robot_x, pt[1] - robot_y)
                if dist >= lookahead_px:
                    target = pt
                    break

        dx = target[0] - robot_x
        dy = robot_y - target[1]
        ld = max(1.0, math.hypot(dx, dy))
        alpha = math.atan2(dx, dy)
        steer_rad = math.atan2(2.0 * C.WHEELBASE_PX * math.sin(alpha), ld)

        steer_deg = math.degrees(steer_rad)

        # ── NUEVO: atenúa el steer si el obstáculo más cercano aún está lejos ──
        if bev_obstacles:
            gain = self._distance_steer_gain(bev_obstacles, robot_x, robot_y)
            steer_deg *= gain

        steer_deg = max(-C.MAX_STEER_DEG, min(C.MAX_STEER_DEG, steer_deg))

        # ── Límite de slew: capa el cambio de steer entre frames procesados ──
        # Evita el latigazo (+0.56 -> +0.20 -> +0.79 -> -0.28 norm. visto en
        # pista). PP_STEER_SLEW_DEG <= 0 lo desactiva.
        slew = getattr(C, "PP_STEER_SLEW_DEG", 0.0)
        if slew > 0.0:
            lo = self._prev_steer_deg - slew
            hi = self._prev_steer_deg + slew
            steer_deg = max(lo, min(hi, steer_deg))
        self._prev_steer_deg = steer_deg

        return steer_deg, (float(target[0]), float(target[1]))

    def normalize(self, steer_deg: float) -> float:
        return max(-1.0, min(1.0, steer_deg / C.PP_STEER_GAIN))