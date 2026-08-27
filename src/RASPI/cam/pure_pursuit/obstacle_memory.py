"""
Memoria de obstáculos — mapa rodante DISPERSO en coordenadas BEV relativas al robot.

Problema que resuelve:
  El BEV se recalcula cada frame y solo contiene lo que la cámara ve AHORA.  Al
  acercarse a una lata, ésta se sale por abajo del cuadro, su inflación
  desaparece y la centerline brinca al centro → el carro se corta sobre la lata.

Idea (igual que el occupancy grid de simulacion_v2, pero sparse):
  Guardamos las latas vistas como una lista de (x, y, color, conf) en coords BEV.
  Cada frame las "arrastramos" según cuánto se movió el robot:
    - avance  : ds_px hacia abajo (la lata se acerca al robot)  → velocidad asumida
    - giro    : rotación −dθ alrededor del robot                → heading del IMU (ESP32)
  Luego fusionamos con las detecciones nuevas, decaemos la confianza de las que
  no se re-vieron y tiramos las que quedaron detrás del robot o ya expiraron.

  La lista resultante se pasa tal cual a detect_centerline() — así la inflación de
  la lata sigue presente hasta que el robot físicamente la rebasa.

Convención BEV (ver bev.py):
  X crece a la derecha, Y crece hacia ABAJO, robot en (ROBOT_BEV_X, ROBOT_BEV_Y)
  mirando hacia ARRIBA (Y decreciente = dirección de marcha).
"""

import math

from . import config as C


class _Obs:
    __slots__ = ("x", "y", "color", "conf", "x0", "beyond", "_cls_vote", "_cls_votes")

    def __init__(self, x: float, y: float, color: str, conf: float):
        self.x = x
        self.y = y
        self.color = color
        self.conf = conf
        # x cuando se detectó por primera vez -- para el "rebase lateral" en
        # _prune(): solo cuenta como pasada de lado si la lata CRUZÓ el eje del
        # robot (empezó del lado de esquiva y terminó del otro), no si ya
        # estaba del otro lado desde el principio.
        self.x0 = x
        # Clasificación respecto a la línea de esquina (ver
        # ObstacleMemory.classify_and_split()): None = aún sin veredicto,
        # True = más allá (siguiente recta), False = mía (recta actual).
        # NO es permanente: se re-evalúa cada frame, pero cambiar el veredicto
        # exige histéresis (LINE_CLASSIFY_FRAMES_TO_MINE/TO_BEYOND) -- así ni un
        # brinco de posición ni una mala lectura de línea de un frame lo voltean,
        # pero una mala clasificación fijada SÍ se puede corregir cuando la línea
        # vuelve a leer bien varios frames seguidos.
        self.beyond: bool | None = None
        self._cls_vote: bool | None = None    # cambio pendiente (True = a "más allá")
        self._cls_votes: int = 0              # frames seguidos apoyando ese cambio


class ObstacleMemory:
    """
    Mantiene el mapa rodante disperso.  Una instancia por runtime.

    Uso por frame:
        merged = mem.update(new_obs, dt_s, heading_deg)
        path   = detect_centerline(bev_frame, merged)
    """

    def __init__(self,
                 robot_x: int = C.ROBOT_BEV_X,
                 robot_y: int = C.ROBOT_BEV_Y):
        self.rx = robot_x
        self.ry = robot_y
        self._obs: list[_Obs] = []
        self._prev_heading: float | None = None
        # Tiempo acumulado de vida de la memoria (suma de dt_s). NO se reinicia
        # en reset() -- es para la rampa de arranque (ver update()), que es un
        # evento de una sola vez al empezar la carrera, no algo por-giro.
        self._elapsed_s: float = 0.0
        self.last_passed: bool = False   # ver _prune() / update()
        self.last_prune_reason: str = "-"   # último motivo de poda, para overlay en pantalla
        self.last_confidences: list[float] = []   # alineado 1:1 con la lista que
                                                    # devolvió el último update() —
                                                    # ver detect_centerline(obstacle_conf=)

    def reset(self):
        self._obs.clear()
        self._prev_heading = None
        self.last_passed = False
        self.last_prune_reason = "-"
        self.last_confidences = []

    # ── Transformación por movimiento del robot ─────────────────────────────────

    def _advance(self, ds_px: float, dheading_deg: float):
        """
        Lleva cada obstáculo recordado del frame anterior al frame actual.

        El robot avanzó ds_px (hacia arriba/−Y) y giró dheading_deg.  En el marco
        relativo al robot eso equivale a: la lata baja ds_px y el mundo rota −dθ
        alrededor del robot.
        """
        # Rotación −dθ.  NOTA: si al doblar el mapa se desalinea hacia el lado
        # equivocado, invierte el signo aquí (depende de la orientación del gyro).
        phi = math.radians(-dheading_deg)
        cos_p, sin_p = math.cos(phi), math.sin(phi)

        for o in self._obs:
            # 1) avance: la lata se acerca → baja en la imagen
            uy = (o.y - self.ry) + ds_px
            ux = (o.x - self.rx)
            # 2) giro: rotar el vector relativo al robot
            rx_new = ux * cos_p - uy * sin_p
            ry_new = ux * sin_p + uy * cos_p
            o.x = self.rx + rx_new
            o.y = self.ry + ry_new

    # ── Fusión con detecciones nuevas ───────────────────────────────────────────

    def _merge(self, new_obs: list[tuple[float, float, str]]):
        match_r2 = C.OBS_MEM_MATCH_PX ** 2
        # Decaer todos primero; los que se re-vean recuperan confianza al fusionar.
        for o in self._obs:
            o.conf -= C.OBS_MEM_DECAY

        for nx, ny, color in new_obs:
            best = None
            best_d2 = match_r2
            for o in self._obs:
                if o.color != color:
                    continue
                d2 = (o.x - nx) ** 2 + (o.y - ny) ** 2
                if d2 <= best_d2:
                    best_d2 = d2
                    best = o
            if best is not None:
                # Re-visto: confiar en la posición fresca de la cámara.
                best.x, best.y = nx, ny
                best.conf = C.OBS_MEM_REFRESH
            else:
                if len(self._obs) < C.OBS_MEM_MAX:
                    self._obs.append(_Obs(nx, ny, color, C.OBS_MEM_REFRESH))

    # ── Reduce duplicados fantasma ─────────────────────────────────────────────────────────────────
    def _dedupe(self):
        """
        Si dos (o más) registros del mismo color quedan a menos de
        OBS_MEM_DEDUPE_PX entre sí, son casi seguro la MISMA lata física
        vista/proyectada dos veces (drift de velocidad asumida, re-detección
        que no hizo match, etc). Los fusiona en uno solo:
          - posición: la del registro con mayor confianza (más "fresco")
          - confianza: la máxima entre los fusionados
        y descarta el resto para que no sigan bloqueando espacio en
        centerline.py.
        """
        if len(self._obs) < 2:
            return

        dedupe_r2 = C.OBS_MEM_DEDUPE_PX ** 2
        # Procesar de mayor a menor confianza: el más confiable "absorbe"
        # a los cercanos de menor confianza.
        ordered = sorted(self._obs, key=lambda o: o.conf, reverse=True)
        kept: list[_Obs] = []

        for o in ordered:
            merged_into_existing = False
            for k in kept:
                if k.color != o.color:
                    continue
                d2 = (k.x - o.x) ** 2 + (k.y - o.y) ** 2
                if d2 <= dedupe_r2:
                    # o es un duplicado de k (k ya tiene >= confianza) → descartar o
                    merged_into_existing = True
                    break
            if not merged_into_existing:
                kept.append(o)

        self._obs = kept

    # ── Poda ────────────────────────────────────────────────────────────────────

    def _prune(self) -> bool:
        """
        Poda obstáculos vencidos.  Distingue POR QUÉ sale cada uno, porque
        "perdí la confianza" (decay) y "quedó detrás del robot" (lo rebasé
        físicamente) no son el mismo evento — solo el segundo debe disparar
        RECUPERANDO en el ESP32 (ver runtime_nuevo.py / PurePursuit.ino).

        Retorna True si en esta llamada algún obstáculo se descartó
        específicamente por quedar detrás del robot ("evento pasado").
        """
        behind_y = self.ry + C.OBS_MEM_BEHIND_PAD
        lat_margin = getattr(C, "OBS_MEM_LATERAL_MARGIN_PX", 8.0)
        lat_y_band = getattr(C, "OBS_MEM_LATERAL_Y_BAND_PX", 140.0)
        kept: list[_Obs] = []
        passed = False
        for o in self._obs:
            if o.conf < C.OBS_MEM_MIN_CONF:
                self.last_prune_reason = f"BAJA_CONF y={o.y:.0f} conf={o.conf:.2f}"
                continue                             # perdido de vista, no rebasado
            if o.y > behind_y:                       # ya quedó detrás del robot
                self.last_prune_reason = f"PASADO y={o.y:.0f}>{behind_y} conf={o.conf:.2f}"
                passed = True
                continue
            # ── Rebase LATERAL: en una esquiva de ángulo el carro pasa la lata
            # de LADO, no de frente. El mapa rota con el heading, así que si la
            # lata CRUZÓ el eje del robot al lado opuesto de la esquiva
            # (Red se rebasa por su derecha -> la lata queda a la IZQUIERDA del
            # robot; Green al revés) y sigue a la altura del robot (no muy
            # adelante), el carro ya la libró -> PASADO / RECUPERANDO ya.
            # Requiere que la lata haya EMPEZADO del lado de esquiva o centrada
            # (o.x0), para no disparar sobre una lata que siempre estuvo del
            # otro lado. A mayor ángulo de giro cruza antes -> recuperando entra
            # antes, que es justo lo que se quiere.
            crossed = (
                (o.color == "Red"
                 and o.x0 >= self.rx - lat_margin
                 and o.x < self.rx - lat_margin)
                or
                (o.color == "Green"
                 and o.x0 <= self.rx + lat_margin
                 and o.x > self.rx + lat_margin)
            )
            if crossed and o.y > self.ry - lat_y_band:
                self.last_prune_reason = (
                    f"PASADO(lat) x={o.x:.0f} x0={o.x0:.0f} y={o.y:.0f} conf={o.conf:.2f}"
                )
                passed = True
                continue
            if not (0.0 <= o.x < C.BEV_W and 0.0 <= o.y < C.BEV_H):
                # Salida del frame BEV. Si fue por ABAJO (y >= BEV_H) y cerca de
                # la columna del robot, el robot lo rebasó -> PASADO (dispara
                # recuperando). Con ds_px grande la lata salta de y<behind_y a
                # y>=BEV_H en un frame sin tocar el check de arriba -- ese hueco
                # dejaba "pasado" sin dispararse nunca. Salidas por lados/arriba
                # = ruido de rotación, NO es un rebase.
                half_w = getattr(C, "OBS_MEM_BEHIND_X_HALFWIDTH", 90.0)
                if o.y >= C.BEV_H and abs(o.x - self.rx) < half_w:
                    self.last_prune_reason = f"PASADO(borde) y={o.y:.0f} conf={o.conf:.2f}"
                    passed = True
                continue
            kept.append(o)
        self._obs = kept
        return passed

    # ── Debug ───────────────────────────────────────────────────────────────────

    def debug_closest(self) -> str:
        """
        Resumen de una línea del obstáculo MÁS CERCANO al robot (mayor y)
        actualmente en memoria, para overlay en pantalla -- permite comparar
        el ritmo real de avance de `y` frame a frame contra lo que se ve en
        el video, y así confirmar si ROBOT_SPEED_MMS está bien calibrado
        (independiente del margen con el que `pasado` termine disparando).
        """
        if not self._obs:
            return "-"
        o = max(self._obs, key=lambda x: x.y)
        behind_y = self.ry + C.OBS_MEM_BEHIND_PAD
        return f"y={o.y:.0f} conf={o.conf:.2f} falta={behind_y - o.y:+.0f}px"

    def debug_all(self) -> str:
        """
        Un renglón por CADA obstáculo en memoria (x,y,color,conf) -- a
        diferencia de debug_closest(), esto expone de inmediato si hay más
        de un registro del mismo color separados en el espacio (fantasma)
        en vez de tener que inferirlo mirando el BEV a ojo.
        """
        if not self._obs:
            return "-"
        return " | ".join(
            f"{o.color[0]}(x={o.x:.0f},y={o.y:.0f},c={o.conf:.2f})" for o in self._obs
        )

    # ── API ─────────────────────────────────────────────────────────────────────

    def update(self,
               new_obs: list[tuple[float, float, str]],
               dt_s: float,
               heading_deg: float | None,
               estado: str | None = None
               ) -> list[tuple[float, float, str]]:
        """
        Avanza el mapa, fusiona detecciones nuevas y devuelve la lista combinada
        (x, y, color) lista para detect_centerline().

        new_obs     : obstáculos detectados este frame en coords BEV
        dt_s        : segundos transcurridos desde el update anterior
        heading_deg : ángulo del IMU (anguloGyro del ESP32) o None si no llegó
        estado      : est= del ACK del ESP32 ('S'/'G'/'R') o None. En 'R'
                      (reversa) el carro NO avanza -> no arrastrar la memoria
                      hacia el robot (si no, empuja las latas "detrás" mientras
                      el carro se aleja de ellas de espaldas).
        """
        self._elapsed_s += dt_s if dt_s > 0 else 0.0

        # Delta de heading de este frame -- se usa para (a) escalar ds_px
        # (freno por giro, abajo) y (b) rotar el mapa en _advance().
        if heading_deg is not None and self._prev_heading is not None:
            dheading = heading_deg - self._prev_heading
        else:
            dheading = 0.0
        if heading_deg is not None:
            self._prev_heading = heading_deg

        ds_px = (C.ROBOT_SPEED_MMS * dt_s) / C.MM_PER_PX if dt_s > 0 else 0.0

        # Reversa: el avance asumido no aplica (y de hecho sería al revés).
        if estado == "R":
            ds_px = 0.0

        # Rampa de arranque: los primeros OBS_MEM_LAUNCH_RAMP_S el carro sale de
        # 0 (y girando en el sitio), no a ROBOT_SPEED_MMS -- escalar el arrastre
        # para no marchar la lata fuera de memoria antes de rebasarla de verdad.
        ramp_s = getattr(C, "OBS_MEM_LAUNCH_RAMP_S", 0.0)
        if ramp_s > 0.0 and self._elapsed_s < ramp_s:
            ds_px *= self._elapsed_s / ramp_s

        # Freno por giro: en un latiguazo de esquiva el carro ROTA pero casi no
        # avanza de frente. El avance forward asumido (ROBOT_SPEED_MMS fijo)
        # sobre-marcha la lata hacia atras y dispara PASADO/RECUPERANDO cuando
        # fisicamente la lata sigue AL LADO del carro, no detras -- por eso
        # "recuperando entra tarde" en esquivas de mucho angulo (confirmado en
        # pista, run5: lata y=293->378 en 0.5s mientras el carro solo giraba).
        # Escala ds_px hacia abajo segun |dheading| de este frame: giro suave
        # (recta) -> 1.0 intacto; latiguazo -> hasta OBS_MEM_TURN_SCALE_MIN.
        dz = getattr(C, "OBS_MEM_TURN_DEADZONE_DEG", 4.0)
        fl = getattr(C, "OBS_MEM_TURN_FLOOR_DEG", 12.0)
        mn = getattr(C, "OBS_MEM_TURN_SCALE_MIN", 0.3)
        if fl > dz:
            frac = min(1.0, max(0.0, abs(dheading) - dz) / (fl - dz))
            ds_px *= 1.0 - (1.0 - mn) * frac

        self._advance(ds_px, dheading)
        self._merge(new_obs)
        self._dedupe()
        self.last_passed = self._prune()

        # Alineado 1:1, mismo orden, con la lista que se retorna abajo --
        # quien la consuma (detect_centerline vía obstacle_conf=) puede
        # atenuar la urgencia de esquiva de un obstáculo que lleva varios
        # frames sin verse de verdad (solo arrastrado por posición asumida)
        # en vez de tratarlo con la misma autoridad que uno recién visto.
        self.last_confidences = [o.conf for o in self._obs]

        return [(o.x, o.y, o.color) for o in self._obs]

    def classify_and_split(
        self, classify_fn
    ) -> tuple[list[tuple[float, float, str]], list[tuple[float, float, str]], list[float]]:
        """
        Separa los obstáculos en memoria entre "mi recta" y "más allá" de la
        línea de esquina, con HISTÉRESIS por objeto (ver _Obs.beyond): se
        re-evalúa cada frame, pero para cambiar el veredicto de un objeto la
        nueva clasificación debe repetirse varios frames seguidos --
        LINE_CLASSIFY_FRAMES_TO_MINE (pocos, lado seguro / recupera de una
        mala fijación) o LINE_CLASSIFY_FRAMES_TO_BEYOND (más, no abandona una
        esquiva a medias por un tramo malo de lectura de línea). Así ni un
        brinco de posición ni una mala lectura puntual lo voltean, pero un
        "más allá" mal fijado SÍ se corrige cuando la línea vuelve a leer bien.

        classify_fn(ox, oy) -> True (mía) | False (más allá) | None (todavía
        no se puede saber, p.ej. línea sin estabilizar) — mismo contrato que
        OrangeLineTracker.classify() pero solo con ox, oy (robot_x/robot_y
        ya deben venir "horneados" en un lambda/partial del caller).

        Llamar SOLO cuando la línea es visible y estable (mismo criterio que
        ya se usaba en runtime_nuevo.py) — si no, no llamar y tratar todo
        como "mío", igual que siempre.

        Retorna (mine, beyond, mine_conf) — mine/beyond son listas de
        (x, y, color); mine_conf alineado 1:1 con mine.
        """
        mine: list[tuple[float, float, str]] = []
        beyond: list[tuple[float, float, str]] = []
        mine_conf: list[float] = []
        for o in self._obs:
            result = classify_fn(o.x, o.y)   # True=mía, False=más allá, None=sin dato
            if result is not None:
                want_beyond = (result is False)
                if want_beyond == (o.beyond is True):
                    # La línea confirma el veredicto vigente -> no hay cambio
                    # pendiente, se limpia cualquier conteo a medias.
                    o._cls_vote = None
                    o._cls_votes = 0
                else:
                    # La línea discrepa del veredicto vigente (o aún no hay).
                    if want_beyond == o._cls_vote:
                        o._cls_votes += 1
                    else:
                        o._cls_vote = want_beyond
                        o._cls_votes = 1
                    need = (C.LINE_CLASSIFY_FRAMES_TO_BEYOND if want_beyond
                            else C.LINE_CLASSIFY_FRAMES_TO_MINE)
                    if o._cls_votes >= need:
                        o.beyond = want_beyond
                        o._cls_vote = None
                        o._cls_votes = 0
            # result is None -> la línea no opina este frame; no se toca el conteo.
            if o.beyond is True:
                beyond.append((o.x, o.y, o.color))
            else:                       # False o None (sin veredicto) -> "mía" (seguro)
                mine.append((o.x, o.y, o.color))
                mine_conf.append(o.conf)
        return mine, beyond, mine_conf
