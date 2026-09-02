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
    __slots__ = ("x", "y", "color", "conf", "x0", "y0", "y_min", "heading0",
                 "xr", "yr", "anchored", "beyond", "_cls_vote", "_cls_votes",
                 "next_seg", "_next_seg_streak", "cam_h")

    def __init__(self, x: float, y: float, color: str, conf: float,
                 heading0: float | None = None):
        self.x = x
        self.y = y
        self.color = color
        self.conf = conf
        # x cuando se detectó por primera vez -- para el "rebase lateral" en
        # _prune(): solo cuenta como pasada de lado si la lata CRUZÓ el eje del
        # robot (empezó del lado de esquiva y terminó del otro), no si ya
        # estaba del otro lado desde el principio.
        self.x0 = x
        # y cuando se detectó por primera vez. Junto con x0 y heading0 es el
        # ANCLA del trigger "geom" (ver _prune / OBS_MEM_LAT_TURN_MODE="geom").
        self.y0 = y
        # Posición dead-reckoning pura de la lata: nace en (x,y) y SOLO la mueve
        # _advance() (ego-movimiento: giro real del IMU + avance asumido). NUNCA
        # se re-ancla con detecciones frescas (a diferencia de x,y). Es la base
        # geométrica de "¿ya la rodeé de lado?" sin depender de que la x medida
        # cruce un umbral frágil.
        self.xr = x
        self.yr = y
        # El ancla (x0,y0,heading0,xr,yr) NO se fija hasta que la lata se detecta
        # a una `y` confiable (>= OBS_MEM_ANCHOR_MIN_Y). Antes de eso la
        # proyección cámara->BEV cerca del horizonte es basura (error enorme en
        # x/y por ángulo rasante) -> el dead-reckoning arrancaría de un punto
        # inventado (visto en pista: ancla Xr=-124 Yr=+212 con la lata enfrente).
        # Mientras anchored=False, _merge re-siembra el ancla con cada detección.
        self.anchored = y >= getattr(C, "OBS_MEM_ANCHOR_MIN_Y", 200.0)
        # Heading del carro (IMU) cuando se detectó por primera vez. El rebase
        # LATERAL primario compara el heading actual contra éste: si el carro
        # rotó >= OBS_MEM_LAT_TURN_DEG hacia el lado correcto desde que vio la
        # lata, ya la rodeó -> PASADO/RECUPERANDO. Mide el GIRO REAL del IMU,
        # no una x inferida frágil -> no le puede ganar la condición por
        # distancia en un latiguazo.
        self.heading0 = heading0
        # y más chica (más adelante) que ha tenido esta lata según DETECCIONES
        # de cámara (no estima muerta). _prune() solo dispara "PASADO" si la
        # lata estuvo de verdad adelante en algún momento -- así una detección
        # espuria que NACE proyectada con y grande (borde inferior del BEV, lata
        # muy cerca) no dispara RECUPERANDO como si la hubiéramos rebasado.
        self.y_min = y
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
        # Latch por TAMAÑO de bbox de cámara (ver ObstacleMemory.flag_next_seg /
        # el LOCK en runtime_nuevo). SOLO se activa cuando hay >=2 conos `mia` a
        # la vez y el LOCK descartó éste por ser MUCHO más chico que el primario
        # (recta siguiente proyecta bbox chico y estable; ver [DET] h=). Una vez
        # True, classify_and_split() lo fuerza a `beyond` cada frame IGNORANDO la
        # naranja y que su bbox crezca, hasta reset()/forget (el giro). Es el
        # respaldo del clasificador por naranja para el caso de 2 conos con el
        # carro ladeado, que es cuando la naranja falla (orillas496/498).
        self.next_seg: bool = False
        self._next_seg_streak: int = 0
        # Alto del bbox de CÁMARA de la última detección FRESCA (proximidad real,
        # NO miente con el yaw como la naranja ni con la proyección BEV como `y`).
        # classify_and_split() fuerza `mia` a un cono con cam_h grande + conf
        # fresca: un cono físicamente CERCA es de mi recta, la naranja no vota.
        # 0.0 = nunca visto fresco / solo dead-reckoning.
        self.cam_h: float = 0.0


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
        self._last_dheading: float = 0.0   # Δheading del último update (predicción _prune)
        self._last_ds_anchor: float = 0.0  # avance px del último frame para el ancla "geom" (lead)
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

    def forget_color_obstacles(self):
        """
        Olvida TODAS las latas Red/Green de la memoria (deja líneas u otros si
        los hubiera). Lo llama runtime_nuevo cuando el trigger de RECUPERANDO
        decide que la lata ya se rebasó: así la centerline deja de rodearla en
        el MISMO frame y el carro sale del arco de esquiva en vez de seguir
        clavando el volante alrededor de un cono que ya pasó (el modo "angle"
        viejo hacía justo esto al podar la lata en _prune -> era lo que mantenía
        la esquiva fluida; "off" lo quitó y el carro se quedaba pivoteando).
        """
        self._obs = [o for o in self._obs if o.color not in ("Red", "Green")]
        self.last_confidences = [o.conf for o in self._obs]

    # ── Transformación por movimiento del robot ─────────────────────────────────

    def _advance(self, ds_px: float, ds_anchor: float, dheading_deg: float):
        """
        Lleva cada obstáculo recordado del frame anterior al frame actual.

        El robot avanzó ds_px (hacia arriba/−Y) y giró dheading_deg.  En el marco
        relativo al robot eso equivale a: la lata baja ds_px y el mundo rota −dθ
        alrededor del robot.

        ds_anchor: avance px aplicado al ANCLA dead-reckoning (o.xr,o.yr) del
        trigger "geom" -- normalmente == ds_px, pero se escala aparte con
        OBS_MEM_GEOM_SPEED_SCALE para calibrar sin tocar el mapa que ve la
        centerline.
        """
        # Rotación −dθ.  NOTA: si al doblar el mapa se desalinea hacia el lado
        # equivocado, invierte el signo aquí (depende de la orientación del gyro).
        phi = math.radians(-dheading_deg)
        cos_p, sin_p = math.cos(phi), math.sin(phi)

        # El ANCLA geom usa el signo de rotación OPUESTO: verificado contra la
        # forma cerrada (rotar el punto -Δθ para expresarlo en el marco nuevo)
        # da el signo +dheading. El mapa (o.x/o.y) se corrige con detecciones
        # frescas así que su signo "ajustado a ojo" pasó desapercibido; el
        # ancla es dead-reckoning puro y necesita el físicamente correcto.
        phi_a = math.radians(dheading_deg)
        cos_a, sin_a = math.cos(phi_a), math.sin(phi_a)

        for o in self._obs:
            # 1) avance: la lata se acerca → baja en la imagen
            uy = (o.y - self.ry) + ds_px
            ux = (o.x - self.rx)
            # 2) giro: rotar el vector relativo al robot
            rx_new = ux * cos_p - uy * sin_p
            ry_new = ux * sin_p + uy * cos_p
            o.x = self.rx + rx_new
            o.y = self.ry + ry_new

            # Ancla "geom": misma forma, rotación con signo físico (phi_a), sin
            # re-anclado por detecciones -> ego-movimiento puro desde (x0,y0).
            ay = (o.yr - self.ry) + ds_anchor
            ax = (o.xr - self.rx)
            o.xr = self.rx + (ax * cos_a - ay * sin_a)
            o.yr = self.ry + (ax * sin_a + ay * cos_a)

    # ── Fusión con detecciones nuevas ───────────────────────────────────────────

    def _merge(self, new_obs: list[tuple[float, float, str]],
               new_obs_h: list[float] | None = None):
        match_r2 = C.OBS_MEM_MATCH_PX ** 2
        # Decaer todos primero; los que se re-vean recuperan confianza al fusionar.
        for o in self._obs:
            o.conf -= C.OBS_MEM_DECAY
        _touched: set[int] = set()   # _Obs refrescados en ESTA llamada (para cam_h)

        for i, (nx, ny, color) in enumerate(new_obs):
            nh = float(new_obs_h[i]) if (new_obs_h and i < len(new_obs_h)) else 0.0
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
                best.y_min = min(best.y_min, ny)   # detección, no estima
                # cam_h: si dos detecciones caen en el MISMO _Obs este frame
                # (near+far fusionados por _merge/_dedupe), quedarse con la MÁS
                # GRANDE -> el _Obs representa al cono CERCANO (el que hay que
                # rodear), no al de la recta siguiente.
                if id(best) in _touched:
                    best.cam_h = max(best.cam_h, nh)
                else:
                    best.cam_h = nh
                    _touched.add(id(best))
                # Ancla aún sin fijar: re-sembrarla con esta detección. Se fija
                # (deja de re-sembrarse) en cuanto la lata se ve a `y` confiable.
                if not best.anchored:
                    best.x0, best.y0 = nx, ny
                    best.xr, best.yr = nx, ny
                    best.heading0 = self._prev_heading
                    if ny >= getattr(C, "OBS_MEM_ANCHOR_MIN_Y", 200.0):
                        best.anchored = True
            else:
                if len(self._obs) < C.OBS_MEM_MAX:
                    _no = _Obs(nx, ny, color, C.OBS_MEM_REFRESH,
                               heading0=self._prev_heading)
                    _no.cam_h = nh
                    self._obs.append(_no)

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
                    # o es un duplicado de k (k ya tiene >= confianza) → descartar o.
                    # cam_h del que queda = el MÁS GRANDE de los dos: si _dedupe
                    # se comió un cono near + uno far (misma color, ~50px en BEV),
                    # el registro que sobrevive debe representar al CERCANO.
                    k.cam_h = max(k.cam_h, o.cam_h)
                    k.next_seg = k.next_seg and o.next_seg
                    merged_into_existing = True
                    break
            if not merged_into_existing:
                kept.append(o)

        self._obs = kept

    # ── Rebase lateral: ¿ya rodeé esta lata de lado? ────────────────────────────

    def _lat_pass(self, o: "_Obs") -> tuple[bool, str]:
        """
        Veredicto de rebase LATERAL de una lata (distinto del rebase por "y
        detrás", que se maneja aparte en _prune con behind_y).

        En una esquiva de ángulo el carro rodea la lata de COSTADO: la lata
        nunca cruza `behind_y` porque el carro gira más que avanza. Tres vías,
        se elige con OBS_MEM_LAT_TURN_MODE:

          "angle" : el carro giró (IMU) >= OBS_MEM_LAT_TURN_DEG hacia el lado
                    correcto desde que vio la lata. Simple pero IGNORA dónde
                    estaba la lata -> mismo ángulo despeja distinto según x0.

          "geom"  : reproyecta la posición inicial de la lata (ancla o.xr/o.yr,
                    movida SOLO por ego-movimiento: giro real IMU + avance
                    asumido·SPEED_SCALE) y pregunta si ya quedó longitudinalmente
                    al costado/detrás (fyr <= AHEAD_MARGIN) Y separada del eje
                    del robot por CLEAR_PX a CUALQUIER lado (dirección-agnóstico:
                    si el latiguazo la rodeó, |exr| crece; si el carro le pasó
                    por encima sin rodearla, |exr| se queda chico y NO dispara).
                    LEAD_FRAMES adelanta el disparo para tapar la latencia
                    serial. MIN_DTHETA_DEG evita disparos por ruido en recta.

          (x)     : respaldo en TODOS los modos -- la x medida (frágil, oscila)
                    cruzó el eje del robot al lado opuesto de x0.

        Exige, en todos los casos: la lata estuvo DE VERDAD adelante en algún
        frame (y_min de detección) y sigue a la altura del robot (banda y).
        """
        ahead_gate = self.ry - getattr(C, "OBS_MEM_PASSED_MIN_AHEAD_PX", 40.0)
        lat_y_band = getattr(C, "OBS_MEM_LATERAL_Y_BAND_PX", 140.0)
        lat_margin = getattr(C, "OBS_MEM_LATERAL_MARGIN_PX", 8.0)
        if not (o.y_min < ahead_gate and o.y > self.ry - lat_y_band):
            return False, ""

        mode = getattr(C, "OBS_MEM_LAT_TURN_MODE", None)
        if mode is None:   # compat con el flag viejo
            mode = "angle" if getattr(C, "OBS_MEM_LAT_TURN_ENABLED", True) else "off"

        # "off" = SIN rebase lateral de ningún tipo (ni giro, ni geom, ni cruce
        # de x). RECUPERANDO SOLO por "PASADO y" (la lata quedó detrás del eje
        # del robot, ver _prune / OBS_MEM_BEHIND_PAD) o "PASADO(borde)". Es el
        # comportamiento base que funcionaba -- para acelerar RECUPERANDO en
        # este modo, baja OBS_MEM_BEHIND_PAD (más negativo = dispara antes).
        if mode == "off":
            return False, ""

        have_head = o.heading0 is not None and self._prev_heading is not None
        dtheta = ((self._prev_heading - o.heading0 + 180.0) % 360.0 - 180.0) if have_head else 0.0

        if mode == "angle" and have_head:
            turn_deg = getattr(C, "OBS_MEM_LAT_TURN_DEG", 35.0)
            # +|dheading| de lead: contrarresta el undershoot de 1 frame.
            d_pred = dtheta + math.copysign(abs(self._last_dheading), dtheta) if dtheta != 0 else dtheta
            if ((o.color == "Red"   and d_pred <= -turn_deg) or
                    (o.color == "Green" and d_pred >=  turn_deg)):
                return True, (
                    f"PASADO(lat-giro) x={o.x:.0f} y={o.y:.0f} "
                    f"h0={round(o.heading0)} h={round(self._prev_heading)} conf={o.conf:.2f}"
                )
        elif mode == "geom" and have_head and o.anchored:
            # Gate: hubo una esquiva de verdad (no ruido de recta) Y el ancla
            # está fijada de una detección confiable (no del horizonte del BEV).
            if abs(dtheta) >= getattr(C, "OBS_MEM_GEOM_MIN_DTHETA_DEG", 12.0):
                ahead_m  = getattr(C, "OBS_MEM_GEOM_AHEAD_MARGIN_PX", 10.0)
                # Ancla (o.xr, o.yr): posición inicial de la lata propagada por
                # el ego-movimiento (rotación IMU exacta + arco s=R·Δθ). En el
                # marco ACTUAL del robot (mira -y en imagen => +y-arriba):
                X_r = o.xr - self.rx            # + = lata a la derecha
                Y_r = self.ry - o.yr           # + = lata ADELANTE del eje trasero
                # Rodeada = la lata ya quedó al través / detrás (Y_r <= am).
                # (Se quitó la rama |X_r|>=CLEAR: un ancla corrupta que deriva
                #  de lado disparaba con la lata aún 200px enfrente -> choque.)
                if Y_r <= ahead_m:
                    return True, (
                        f"PASADO(lat-geom/trav) Xr={X_r:+.0f} Yr={Y_r:+.0f} "
                        f"dth={dtheta:+.0f} am={ahead_m:.0f} conf={o.conf:.2f}"
                    )

        # (x) respaldo -- SOLO en modo "angle". En "geom" la x medida cruza el
        # eje por el YAW del carro (no porque lo rebasó) y disparaba en falso;
        # geom decide solo con el ancla.
        if mode == "angle" and (
            (o.color == "Red"
             and o.x0 >= self.rx - lat_margin and o.x < self.rx - lat_margin) or
            (o.color == "Green"
             and o.x0 <= self.rx + lat_margin and o.x > self.rx + lat_margin)):
            return True, (
                f"PASADO(lat-x) x={o.x:.0f} y={o.y:.0f} "
                f"h0={o.heading0 if o.heading0 is None else round(o.heading0)} "
                f"h={self._prev_heading if self._prev_heading is None else round(self._prev_heading)} "
                f"conf={o.conf:.2f}"
            )
        return False, ""

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
        # Para disparar "PASADO" (RECUPERANDO) exigimos que la lata haya estado
        # DE VERDAD adelante en algún frame (detección de cámara, y_min chico).
        # Si nació ya proyectada con y grande -- p.ej. una detección espuria en
        # el borde inferior del BEV con la lata muy cerca al arrancar -- NO es
        # un rebase: se descarta en silencio, sin mandar recuperando.
        ahead_gate = self.ry - getattr(C, "OBS_MEM_PASSED_MIN_AHEAD_PX", 40.0)
        # Para que la poda por "quedó detrás" dispare PASADO (=> RECUPERANDO en el
        # ESP) hay DOS vías (basta una), y en ambas la lata debió estar de verdad
        # adelante (was_ahead) y NO ser del siguiente segmento (o.beyond):
        #   (a) rebase DE FRENTE: x0 centrado (|x0 - robot_x| <= pass_halfw). El
        #       trigger MEDIDO de runtime necesita |heading|>=25° -> no cubre un
        #       rebase recto, ésta es su red de respaldo.
        #   (b) rebase DE LADO (esquiva de ángulo): el carro GIRÓ (IMU) >=
        #       OBS_MEM_PASSED_YAW_DEG desde que vio la lata -> la rodeó de
        #       verdad; su x0 de lado es natural (cono slot 1/2/5/6), no basura.
        # Una lata con x0 de lado Y poco giro Y clasificada beyond es un cono del
        # SIGUIENTE segmento mal proyectado que cruza behind_y por dead-reckoning
        # (orillas487: 2do rojo x0=256) -> DESCARTE_DE_LADO, no dispara. Sin la
        # vía (b) el verde esquivado (x0=144) también caía en DESCARTE y el ESP
        # llegaba a la esquina ladeado -> MANIOBRA a ~50° (orillas ~490).
        # Se usa x0 (fijo) y no o.x, que _advance rota con el yaw.
        pass_halfw = getattr(C, "OBS_MEM_PASSED_X_HALFWIDTH", 50.0)
        pass_yaw   = getattr(C, "OBS_MEM_PASSED_YAW_DEG", 30.0)
        kept: list[_Obs] = []
        passed = False
        for o in self._obs:
            was_ahead = o.y_min < ahead_gate
            if o.conf < C.OBS_MEM_MIN_CONF:
                # Perdido de vista, no rebasado. (Se probó evaluar _lat_pass aquí
                # para el modo "geom" pero disparaba RECUPERANDO en falso al
                # decaer latas que nunca se rodearon -- revertido.)
                self.last_prune_reason = f"BAJA_CONF y={o.y:.0f} conf={o.conf:.2f}"
                continue
            if o.y > behind_y:                       # ya quedó detrás del robot
                centered = abs(o.x0 - self.rx) <= pass_halfw
                have_h = o.heading0 is not None and self._prev_heading is not None
                yawed = (abs((self._prev_heading - o.heading0 + 180.0) % 360.0 - 180.0)
                         if have_h else 0.0)
                dodged = yawed >= pass_yaw and o.beyond is not True
                if was_ahead and (centered or dodged):
                    self.last_prune_reason = (
                        f"PASADO y={o.y:.0f}>{behind_y} ymin={o.y_min:.0f} "
                        f"x0={o.x0:.0f} yaw={yawed:.0f} "
                        f"{'frente' if centered else 'esquiva'} conf={o.conf:.2f}"
                    )
                    passed = True
                elif was_ahead:
                    self.last_prune_reason = (
                        f"DESCARTE_DE_LADO x0={o.x0:.0f} "
                        f"|dx0|={abs(o.x0 - self.rx):.0f}>{pass_halfw:.0f} "
                        f"yaw={yawed:.0f}<{pass_yaw:.0f} beyond={o.beyond}"
                    )
                else:
                    self.last_prune_reason = f"DESCARTE_NO_ADELANTE y={o.y:.0f} ymin={o.y_min:.0f}"
                continue
            # Rebase LATERAL (el carro pasó la lata de LADO en una esquiva de
            # ángulo, no de frente): ver _lat_pass() -- modo angle / geom / x.
            lp, why = self._lat_pass(o)
            if lp:
                self.last_prune_reason = why
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
                if o.y >= C.BEV_H and abs(o.x - self.rx) < half_w and was_ahead:
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
        if o.heading0 is not None and self._prev_heading is not None:
            turned = (self._prev_heading - o.heading0 + 180.0) % 360.0 - 180.0
        else:
            turned = None
        return (f"y={o.y:.0f} x={o.x:.0f} conf={o.conf:.2f} "
                f"falta={behind_y - o.y:+.0f}px "
                f"ymin={o.y_min:.0f} giro={'?' if turned is None else round(turned)} "
                f"camh={o.cam_h:.0f} nseg={int(o.next_seg)}")

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
               estado: str | None = None,
               steer_deg: float = 0.0,
               new_obs_h: list[float] | None = None
               ) -> list[tuple[float, float, str]]:
        """
        Avanza el mapa, fusiona detecciones nuevas y devuelve la lista combinada
        (x, y, color) lista para detect_centerline().

        new_obs     : obstáculos detectados este frame en coords BEV
        dt_s        : segundos transcurridos desde el update anterior
        heading_deg : ángulo del IMU (anguloGyro del ESP32) o None si no llegó
        estado      : est= del ACK del ESP32 ('S'=recto, 'G'=giro de esquina,
                      'R'=RECUPERANDO). Se acepta por compatibilidad; hoy no
                      cambia el avance porque en RECUPERANDO el carro SIGUE
                      avanzando (corrige heading e va derecho, sin visión), y
                      en 'G' runtime_nuevo ya apaga la memoria aparte. El carro
                      no tiene reversa.
        """
        self._elapsed_s += dt_s if dt_s > 0 else 0.0

        # Delta de heading de este frame -- se usa para (a) escalar ds_px
        # (freno por giro, abajo), (b) rotar el mapa en _advance() y (c) la
        # predicción del rebase lateral por giro en _prune().
        if heading_deg is not None and self._prev_heading is not None:
            dheading = heading_deg - self._prev_heading
        else:
            dheading = 0.0
        self._last_dheading = dheading
        if heading_deg is not None:
            self._prev_heading = heading_deg
            # Backfill: una lata detectada ANTES de que llegara el primer ACK
            # del ESP32 nace con heading0=None -> el rebase lateral por giro
            # queda apagado para ella justo cuando más se necesita (primera lata
            # del arranque). En cuanto hay heading, se lo asignamos.
            for _o in self._obs:
                if _o.heading0 is None:
                    _o.heading0 = heading_deg

        ds_px = (C.ROBOT_SPEED_MMS * dt_s) / C.MM_PER_PX if dt_s > 0 else 0.0

        # Rampa de arranque: los primeros OBS_MEM_LAUNCH_RAMP_S el carro sale de
        # 0 (y girando en el sitio), no a ROBOT_SPEED_MMS -- escalar el arrastre
        # para no marchar la lata fuera de memoria antes de rebasarla de verdad.
        ramp_s = getattr(C, "OBS_MEM_LAUNCH_RAMP_S", 0.0)
        if ramp_s > 0.0 and self._elapsed_s < ramp_s:
            ds_px *= self._elapsed_s / ramp_s
        ds_ramped = ds_px   # avance libre (con rampa, SIN freno por giro) -- tope del ancla geom

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

        # ── Avance del ancla "geom" ───────────────────────────────────────────
        # Dos estimaciones del avance de frente, se toma la MENOR:
        #  (a) arco de bicicleta s = R·Δθ, R = WHEELBASE/tan(steer). Es el
        #      avance SIN deslizamiento -> cota SUPERIOR.
        #  (b) ds_px ya frenado por giro (OBS_MEM_TURN_SCALE_MIN): modela que en
        #      un latiguazo el carro DERRAPA y casi no avanza -> más realista
        #      cuando |dheading| es grande.
        # Con solo (a) el ancla sobre-marchaba (verde: Yr decía "al costado" a
        # 36° de giro cuando físicamente seguía 120px adelante -> RECUPERANDO
        # prematuro -> choque). El min() la mantiene honesta en el pivote.
        steer_rad = abs(math.radians(steer_deg))
        if abs(dheading) > 0.3 and steer_rad > math.radians(3.0):
            steer_rad = min(steer_rad, math.radians(C.MAX_STEER_DEG))
            R_px = C.WHEELBASE_PX / math.tan(steer_rad)
            ds_arc = R_px * abs(math.radians(dheading))
            ds_anchor = min(ds_arc, ds_px)          # ds_px ya viene frenado por giro
        else:
            ds_anchor = ds_px
        ds_anchor *= getattr(C, "OBS_MEM_GEOM_SPEED_SCALE", 1.0)
        self._last_ds_anchor = ds_anchor

        self._advance(ds_px, ds_anchor, dheading)
        self._merge(new_obs, new_obs_h)
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
        self, classify_fn, rescue_fn=None
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

        rescue_fn(ox, oy, color) -> bool (opcional). Si devuelve True para un
        objeto que la clasificación dejó en "más allá", ese objeto se devuelve
        igual en `mine` (se esquiva + bloquea el giro) SIN tocar su o.beyond:
        la histéresis sigue intacta por debajo, así que si el giro llega y el
        cono sí pasa a la recta siguiente, la clasificación no quedó corrupta.
        Uso: cono EXTERIOR pegado a la boca de la esquina (ver
        CORNER_EXTERIOR_PASS_ENABLED en config.py) — geométricamente está al
        otro lado de la naranja pero hay que rodearlo ANTES de girar.

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
            # Cono físicamente CERCA (bbox de cámara grande + conf fresca) = de
            # MI recta, SIEMPRE. La naranja no vota: es justo cuando el carro
            # está ladeado (esquivando) que la naranja miente y mandaba este
            # cono a `beyond` -> no se esquivaba -> sin RECUPERANDO -> GIRANDO
            # (orillas496/498/500). El bbox NO miente con el yaw.
            if (o.cam_h >= getattr(C, "CLASSIFY_FORCE_MINE_BBOX_PX", 70.0)
                    and o.conf >= getattr(C, "CLASSIFY_FORCE_MINE_MIN_CONF", 0.65)):
                mine.append((o.x, o.y, o.color))
                mine_conf.append(o.conf)
                continue
            # Latch por bbox (ver flag_next_seg): cono de la recta SIGUIENTE
            # fijado por tamaño cuando había 2 conos y el carro estaba ladeado
            # (la naranja no era confiable). Va directo a `beyond`, sin naranja
            # ni rescue, hasta reset()/forget.
            if o.next_seg:
                beyond.append((o.x, o.y, o.color))
                continue
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
                    if o.beyond is None:
                        # PRIMER veredicto: rápido en ambos sentidos. Antes "mío"
                        # era instantáneo y "siguiente recta" tardaba 12 frames
                        # -> se esquivaba un obstáculo de la recta que sigue.
                        need = getattr(C, "LINE_CLASSIFY_FRAMES_FIRST", 4)
                    elif want_beyond:
                        need = C.LINE_CLASSIFY_FRAMES_TO_BEYOND
                    else:
                        need = C.LINE_CLASSIFY_FRAMES_TO_MINE
                    if o._cls_votes >= need:
                        o.beyond = want_beyond
                        o._cls_vote = None
                        o._cls_votes = 0
            # result is None -> la línea no opina este frame; no se toca el conteo.
            if o.beyond is True:
                if rescue_fn is not None and rescue_fn(o.x, o.y, o.color):
                    # Cono exterior de esquina: se trata como "mío" (esquiva +
                    # bloqueo de giro) aunque geométricamente esté "más allá".
                    # o.beyond NO se toca -> la histéresis sigue intacta.
                    mine.append((o.x, o.y, o.color))
                    mine_conf.append(o.conf)
                else:
                    beyond.append((o.x, o.y, o.color))
            else:                       # False o None (sin veredicto) -> "mía" (seguro)
                mine.append((o.x, o.y, o.color))
                mine_conf.append(o.conf)
        return mine, beyond, mine_conf

    def flag_next_seg(self, dropped_with_h: list[tuple[float, float, float]],
                      primary_h: float) -> None:
        """
        Marca como "recta SIGUIENTE" (latch `_Obs.next_seg`) a un cono que el
        LOCK de runtime_nuevo descartó por ser MUCHO más chico que el primario.
        SOLO se llama cuando hay >=2 conos `mia` a la vez (es el único caso en
        que la clasificación por naranja falla feo -- carro ladeado, 2 blobs
        confunden el ajuste de recta). Con 1 cono la naranja manda, sin cambio.

        dropped_with_h: [(x, y, h_bbox_camara), ...] de los conos NO primarios.
        primary_h:      h del bbox de cámara del primario.

        Latch tras NEXT_SEG_BBOX_FRAMES frames SEGUIDOS cumpliendo:
          h <= NEXT_SEG_BBOX_MAX_PX  (absoluto: tan chico con otro grande = otra recta)
          h <= NEXT_SEG_BBOX_RATIO * primary_h  (relativo: no latchear un cono
               de mi recta que solo está algo más lejos)
        Una vez True, classify_and_split() lo fuerza a `beyond` hasta reset()/
        forget (el giro). Números de calibración en [DET] h= (orillas499):
        cono de mi recta h~74-240, cono recta siguiente h~40-54.
        """
        max_h = getattr(C, "NEXT_SEG_BBOX_MAX_PX", 62.0)
        ratio = getattr(C, "NEXT_SEG_BBOX_RATIO", 0.6)
        need  = getattr(C, "NEXT_SEG_BBOX_FRAMES", 3)
        # Si el propio primario es chico (<= max_h) NO se puede confiar en el
        # bbox para desambiguar (ambos conos lejos / LOCK fijó al equivocado por
        # continuidad de posición) -> no latchear nada este frame.
        if primary_h <= max_h:
            return
        r2 = getattr(C, "LOCK_MATCH_RADIUS_PX", 70.0) ** 2
        for (dx, dy, dh) in dropped_with_h:
            best = None
            bd = r2
            for o in self._obs:
                d = (o.x - dx) ** 2 + (o.y - dy) ** 2
                if d < bd:
                    bd, best = d, o
            if best is None or best.next_seg:
                continue
            if 0.0 < dh <= max_h and dh <= ratio * primary_h:
                best._next_seg_streak += 1
                if best._next_seg_streak >= need:
                    best.next_seg = True
                    print(f"[NEXTSEG] latch {best.color[0]}@({best.x:.0f},{best.y:.0f}) "
                          f"h={dh:.0f} <= {max_h:.0f} y <= {ratio:.2f}*{primary_h:.0f} "
                          f"-> recta siguiente hasta el giro", flush=True)
            else:
                best._next_seg_streak = 0
