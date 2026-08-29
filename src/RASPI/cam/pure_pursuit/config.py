"""
Parámetros de configuración del pipeline Pure Pursuit — WRO Future Engineers.

Hardware:
  Cámara  : Raspberry Pi Camera v2  (FOV ~62°, resolución proceso 640×480)
  Motor   : N20 DC 50:1 + etapa LEGO 2:1  → ratio total 100:1
  Servo   : SG90  (centro = 80°, límite mecánico ±35° efectivo en rueda)
  Chassis : 210×140×80 mm  |  batalla ~100 mm (estimado)
  ESP32   : recibe protocolo V2 por UART Serial2 (RX=17, TX=16) @ 115200
  Pi GPIO : LED en 27, botón arranque en 17

Ajusta primero CALIB_REAL_MM para que coincida con tu montaje de cámara,
luego corre calibrate.py para generar bev_calib.npz.
"""

import numpy as np
from pathlib import Path

# ─── Rutas ────────────────────────────────────────────────────────────────────
CALIB_FILE = Path(__file__).parent / "bev_calib.npz"

# ─── BEV — imagen de salida ───────────────────────────────────────────────────
BEV_W      = 400          # píxeles de ancho
BEV_H      = 400          # píxeles de alto
MM_PER_PX  = 2.0          # escala: 1px = 2 mm  →  400px = 800 mm de cobertura

# Posición del robot en la imagen BEV (eje trasero, centro horizontal)
ROBOT_BEV_X = BEV_W // 2   # 200
ROBOT_BEV_Y = BEV_H - 20   # 380

# ─── Puntos de calibración en el suelo (mm, relativo a eje delantero del robot) ──
# x_mm: lateral  (+ = derecha del robot)
# y_mm: distancia hacia adelante desde el eje delantero
#
# IMPORTANTE — por qué 9 puntos y no 4:
#   Una homografía con exactamente 4 puntos es un ajuste EXACTO: no hay forma
#   de detectar ni corregir un clic impreciso durante la calibración, y la
#   transformación se vuelve muy poco confiable fuera del área que cubren esos
#   4 puntos (extrapolación agresiva).  Con 9 puntos en grilla 3×3,
#   cv2.findHomography(..., RANSAC) puede promediar/filtrar el ruido de los
#   clics, y el área cubierta coincide con el rango real que usa
#   centerline.py (desde muy cerca del robot hasta ~550mm adelante).
#
#   Coloca marcadores físicos en el suelo en estas posiciones exactas y haz
#   clic en ellos EN ESTE MISMO ORDEN dentro de calibrate.py.
#
#          izquierda        centro         derecha
#   lejos:   P7 (-220,550)  P8 (0,550)   P9 (220,550)
#   media:   P4 (-150,300)  P5 (0,300)   P6 (150,300)
#   cerca:   P1  (-80, 80)  P2 (0, 80)   P3  (80, 80)
CALIB_REAL_MM = np.float32([
    [ -80.0,  80.0],   # P1: cerca-izquierda
    [   0.0,  80.0],   # P2: cerca-centro
    [  80.0,  80.0],   # P3: cerca-derecha
    [-150.0, 300.0],   # P4: media-izquierda
    [   0.0, 300.0],   # P5: media-centro
    [ 150.0, 300.0],   # P6: media-derecha
    [-220.0, 550.0],   # P7: lejos-izquierda
    [   0.0, 550.0],   # P8: lejos-centro
    [ 220.0, 550.0],   # P9: lejos-derecha
])

# Etiquetas legibles para cada punto (mismo orden que CALIB_REAL_MM)
CALIB_POINT_LABELS = [
    "P1 cerca-izq", "P2 cerca-centro", "P3 cerca-der",
    "P4 media-izq", "P5 media-centro", "P6 media-der",
    "P7 lejos-izq", "P8 lejos-centro", "P9 lejos-der",
]

# Umbral de error medio de reproyección (px BEV) a partir del cual calibrate.py
# advierte que probablemente un clic quedó mal puesto.
CALIB_MAX_MEAN_ERR_PX = 4.0

WALL_MARGIN_PX = 22

# ─── Color del piso (HSV) — tapete WRO: beige / madera cálida ────────────────
FLOOR_LOWER = np.array([0, 0, 140])
FLOOR_UPPER = np.array([35, 80, 255])

FLOOR_LOWER_BLUE = np.array([95, 50, 10])
FLOOR_UPPER_BLUE = np.array([150, 255, 220])

FLOOR_LOWER_BLUE_WIDE = np.array([90, 15, 60])   # baja el mínimo de S y sube V
FLOOR_UPPER_BLUE_WIDE = np.array([150, 255, 230])

WALL_STRUCTURE_PX = 15   # <-- ajusta viendo tu imagen BEV real

FLOOR_LOWER_ORANGE = np.array([1,  70, 130])
FLOOR_UPPER_ORANGE = np.array([17, 160, 220])

# Lista usada por centerline.py — agrega más tuplas aquí si detectas más
# colores de piso que no quieres que bloqueen la ruta.
FLOOR_COLOR_RANGES = [
    (FLOOR_LOWER, FLOOR_UPPER),
    (FLOOR_LOWER_BLUE, FLOOR_UPPER_BLUE),
    (FLOOR_LOWER_ORANGE, FLOOR_UPPER_ORANGE),
]

# ─── Detección de centerline ──────────────────────────────────────────────────
CENTERLINE_ROW_STEP  = 15    # muestrear cada N filas
CENTERLINE_MIN_WIDTH = 20    # píxeles mínimos de espacio libre para aceptar fila
CENTERLINE_TOP_Y     = BEV_H // 3   # no subir más allá de 1/3 de la imagen
CENTERLINE_RAMP_PX   = 200   # horizonte de anticipo: empieza a abrir el path
                             # hacia el lado de paso a esta distancia Y de la lata
                             # (200 px ≈ 400 mm; debe ser > LOOKAHEAD_PX).
                             # Subido de 140 -> 200: en pista la esquiva no
                             # arrancaba hasta que la lata estaba a ~1 radio de
                             # inflado, forzando un volantazo tardío. 200 da
                             # ~120 mm más de anticipo para mover el path de lado
                             # gradualmente. NO capa el steer máximo (volantazo
                             # sigue disponible cuando la lata está cerca).
CENTERLINE_SMOOTH_WIN = 5    # ventana (impar) de media móvil sobre X post-muestreo

# ─── Manejo de obstáculos en BEV ─────────────────────────────────────────────
# Tamaño físico real de los obstáculos (latas de refresco WRO ≈ 65 mm diámetro)
OBS_REAL_DIAMETER_MM = 75.0
OBS_PHYSICAL_R_PX    = round(OBS_REAL_DIAMETER_MM / 2.0 / MM_PER_PX)  # ≈ 16 px
OBS_SAFETY_R_PX      = 20    # margen de seguridad adicional (px)
OBS_INFLATE_R        = OBS_PHYSICAL_R_PX + OBS_SAFETY_R_PX             # ≈ 35 px

OBS_BIAS_SHIFT = 28    # desplazamiento lateral para sesgo de color WRO (px)

# Clamp del punto de paso de obstáculo (_pass_side_cx): el path pasa como
# máximo a OBS_INFLATE_R + esto del centro del cono. Evita que el trazador
# agarre el sliver de piso pegado a la pared (elige por ancho) y curle el
# path -> sobre-giro. ~30 px = medio carril de aire tras la inflación.
PASS_LANE_MARGIN_PX = 30
#  Rojo  → el robot debe pasar por la DERECHA → se infla más a la izquierda
#  Verde → el robot debe pasar por la IZQUIERDA → se infla más a la derecha

# ─── Escala única de urgencia por distancia (mm reales) ──────────────────────
# A esta distancia (o menos) del obstáculo: máxima agresividad de esquiva.
OBSTACLE_URGENT_MM = 180.0   # 
# A esta distancia (o más): comportamiento normal/suave, sin urgencia.
OBSTACLE_CASUAL_MM = 350.0   # 

# ─── Pure Pursuit ─────────────────────────────────────────────────────────────
LOOKAHEAD_PX   = 100.0    # distancia look-ahead en px BEV  (= 160 mm)
WHEELBASE_PX   = 50.0    # batalla del vehículo en px BEV   (= 100 mm)
MAX_STEER_DEG  = 60.0    # límite mecánico del servo en grados
MIN_PATH_PTS   = 4       # puntos mínimos de path para considerar PP válido

# ── Límite de slew: cuánto puede cambiar el steer entre frames procesados ──
# En pista el steer saltaba +0.56 -> +0.20 -> +0.79 -> -0.28 (norm.) frame a
# frame = latigazo. Esto lo capa. 0 desactiva. 12°/frame @ ~15fps ≈ 180°/s,
# suficiente para esquivar sin dar el volantazo. En grados de steer (pre-norm).
# 2026-08-28: bajado 12->6 al pasar el pipeline de ~7fps a ~14fps (mismo °/s).
PP_STEER_SLEW_DEG = 6.0

# Lookahead variable — derivado de la escala de urgencia
# NOTA: 45 px saturaba el steer al tope mecánico (obs=±1.0) en cada esquiva
# con lata cerca -> el carro clavaba el volante, sobrepasaba y raspaba la lata
# al pasar (confirmado en pista, choque en rojo de inicio y en cada verde).
# 70 px conserva el "acortar para esquivar" sin pegar el límite.
LOOKAHEAD_MIN_PX      = 60.0   # 2026-08-28: 70->60, esquiva se sentía floja a ~14fps (más steer con lata cerca)
LOOKAHEAD_MAX_PX      = 100.0
LOOKAHEAD_OBS_NEAR_PX = OBSTACLE_URGENT_MM / MM_PER_PX   # ≈110px
LOOKAHEAD_OBS_FAR_PX  = OBSTACLE_CASUAL_MM / MM_PER_PX   # ≈250px

CENTERLINE_URGENCY_RELAX = 1.6

# ─── Urgencia frontal por pared (chasis apuntando DE FRENTE, no paralelo) ────
# DESACTIVADA por defecto: fuerza cx al lado despejado con peso 1.0 SIN rampa
# (steer duro instantáneo) y en rectas cerca de una esquina, con el piso del
# BEV parchado adelante, misfireaba. main no tiene esta lógica. Poner True para
# reactivarla.
FRONT_WALL_URGENCY_ENABLED = False
# Si hay pared/no-piso dentro de esta distancia MEDIDA A LO LARGO DEL EJE DEL
# CHASIS (columna ROBOT_BEV_X, no del hueco que el centerline elige), el
# chasis mismo está apuntando hacia ella -- distinto de "hay pared cerca a un
# lado", que es normal en un corredor angosto y no debe disparar esto.
# Empieza en 150mm, ajustar según comportamiento real en pista.
FRONT_WALL_CRITICAL_MM   = 150.0
FRONT_WALL_CRITICAL_PX   = FRONT_WALL_CRITICAL_MM / MM_PER_PX   # ≈75px
FRONT_CHECK_HALFWIDTH_PX = 30    # medio-ancho del "carril" frontal revisado,
                                   # en px BEV -- ajustar al ancho real del
                                   # chasis si hace falta más/menos margen

# Suaviza el steer de esquiva cuando el obstáculo aún no está crítico.
# Atenuación de steer por distancia — misma escala
STEER_DIST_GAIN_NEAR_PX = OBSTACLE_URGENT_MM / MM_PER_PX   # ≈110px
STEER_DIST_GAIN_FAR_PX  = OBSTACLE_CASUAL_MM / MM_PER_PX   # ≈250px
STEER_DIST_GAIN_MIN     = 0.8   # subido de 0.5: la atenuación de steer en la
                                  # zona lejana (110-250px) hacía que el carro
                                  # casi no se moviera de lado hasta tener la
                                  # lata encima, y luego esquivara de volantazo.
                                  # 0.8 deja la reacción lejana casi a fuerza
                                  # completa. Dentro de 110px sigue siendo 1.0
                                  # (volantazo intacto).


# ─── Memoria de obstáculos (mapa rodante disperso) — obstacle_memory.py ───────
# El robot recuerda las latas vistas y las "arrastra" hacia sí mismo cuadro a
# cuadro usando avance asumido (velocidad) + giro del IMU (anguloGyro del ESP32),
# para no perder la inflación cuando la lata sale del campo de visión.
ROBOT_SPEED_MMS    = 350.0   # velocidad de marcha asumida (mm/s). Ajustar al carro real.
# A esta velocidad y ~7fps, el arrastre por frame ya es ~25px -- con el
# match radius en 50px (original), apenas 2 frames seguidos sin re-detección
# fresca (blur, oclusión momentánea) ya alcanzan a superarlo y _merge() crea
# un registro NUEVO en vez de actualizar el existente = objeto fantasma
# duplicado. Subido a ~3 frames de tolerancia -- NO más que eso, porque
# _merge() solo filtra por color, no por qué lado de una esquina está el
# objeto: dos obstáculos del mismo color pegados al interior en extremos
# opuestos de una esquina (fin de una recta / inicio de la siguiente) sí
# pueden quedar a esta distancia en BEV, y un radio más grande arriesgaría
# confundirlos entre sí en esa transición. Si el fantasma sigue apareciendo
# a este valor, el fix correcto ya no es este radio -- es hacer que
# _merge() respete la clasificación mine/beyond de OrangeLineTracker.
OBS_MEM_MATCH_PX   = 75.0    # radio para fusionar una detección nueva con una recordada (px BEV)
OBS_MEM_DECAY      = 0.06    # confianza perdida por frame sin re-ver el obstáculo (0..1)
                            # 2026-08-28: 0.12->0.06 al doblar fps (~7->~14), mismo decay/seg
OBS_MEM_MIN_CONF   = 0.4    # por debajo de esto el obstáculo recordado se descarta
OBS_MEM_REFRESH    = 1.0     # confianza al re-detectar (se satura en 1.0)
OBS_MEM_BEHIND_PAD = -18    # px: tirar el obstáculo (y disparar "pasado" ->
                              # RECUPERANDO en el ESP32) cuando bev_y > robot_y + pad.
                              # NEGATIVO a propósito: umbral en 380-18 = 362, o sea
                              # dispara cuando el CENTRO de la lata pasa ~18px por
                              # DELANTE del eje del robot (el morro ya la rebasó).
                              # En pista RECUPERANDO entraba tarde: con pad=12
                              # (umbral 392) la lata ya estaba "debajo" del carro y
                              # éste ya cruzado 45° -> el countersteer no alcanzaba
                              # a enderezar antes de la pared. -18 lo adelanta ~0.3s.
                              # Costo en esquiva ~nulo: a y=362 la lata ya casi no
                              # influye en las filas del path adelante (_ramp_weight
                              # ~0.14). OJO robot_y=380, BEV_H=400: no bajar de
                              # ~-19 sin revisar; el fallback y>=BEV_H de _prune()
                              # cubre el caso de que se salte el umbral por abajo.

# ── Arranque: rampa de velocidad asumida para el arrastre de la memoria ──
# _advance() acredita ROBOT_SPEED_MMS completos desde el frame 1, pero el carro
# sale de 0 y los primeros ~1s va más lento (y girando en el sitio). Eso marcha
# la lata recordada fuera de memoria antes de que el carro físicamente la
# rebase -> "pasado" falso en el rojo de inicio. Rampa lineal 0->1 sobre este
# tiempo (acumulado, NO se reinicia en cada giro).
OBS_MEM_LAUNCH_RAMP_S = 1.2

# ── Freno de ds_px por giro (obstacle_memory.update) ──
# En una esquiva de mucho ángulo el carro ROTA pero casi no avanza de frente.
# ds_px asume ROBOT_SPEED_MMS fijo -> sobre-marcha la lata hacia atrás ->
# PASADO/RECUPERANDO dispara con la lata todavía al lado (entra tarde por
# geometría). Se escala ds_px según |dheading| por frame:
#   |dheading| <= DEADZONE  -> factor 1.0 (recta / curva suave, sin cambio)
#   |dheading| >= FLOOR     -> factor SCALE_MIN (latiguazo)
#   entre medias            -> lineal
# Valores de pista (run5): recta ~1-3°/frame, latiguazo de esquiva ~11°/frame.
OBS_MEM_TURN_DEADZONE_DEG = 4.0
OBS_MEM_TURN_FLOOR_DEG    = 12.0
OBS_MEM_TURN_SCALE_MIN    = 0.3

# ── Rebase LATERAL (obstacle_memory._prune) ──
# En una esquiva de ángulo el carro pasa la lata DE LADO, no de frente: el
# mapa rota con el heading y la lata cruza el eje del robot al lado opuesto
# de la esquiva. Ése es el momento de RECUPERANDO (enderezar y seguir), no
# cuando la lata cruza detrás en Y (que con mucho ángulo pasa ~1s después,
# con el carro ya sobregirado hacia la pared -- confirmado run5/run6).
OBS_MEM_LATERAL_MARGIN_PX  = 8.0    # px que la lata debe cruzar PASADO el eje (respaldo por x)
OBS_MEM_LATERAL_Y_BAND_PX  = 140.0  # solo cuenta si o.y > robot_y - esto
# Rebase lateral PRIMARIO: grados que el carro debe rotar (IMU) desde que vio
# la lata para contar que ya la rodeó -> PASADO/RECUPERANDO. Perilla de
# ganancia: más chico = RECUPERANDO se lanza más rápido (con menos giro).
# Bajado 35 -> 22: en pista disparaba a ~50° (a 8fps el heading salta ~12°/
# frame y se pasaba del umbral). 22 + la predicción (+|dheading|, ver
# _prune) lo lanzan ~20° antes.
# 2026-08-28: 29->33 fue un error — a ~14fps el lead d_pred (=|dheading/frame|,
# ~5° a 14fps vs ~12° a 8fps) es más chico, así que 33 quedaba por ARRIBA de lo
# que d_pred alcanza y lat-giro casi no disparaba: RECUPERANDO entraba por el
# respaldo lat-x a ~21° real y como el carro ya estaba regresando, salía por
# headingOk en 2-3 frames ("casi no dura"). Bajado a 22 -> lat-giro dispara
# PRIMERO a ~17° real (22 - 5 de lead), a mitad de esquiva, con error de heading
# todavía grande -> RECUPERANDO tiene trabajo que hacer y dura.
OBS_MEM_LAT_TURN_DEG       = 35

# ─── Trigger de "ya rodeé la lata de lado" -> PASADO/RECUPERANDO ──────────────
# Cuál de los 3 métodos decide el rebase lateral (obstacle_memory._prune):
#   "angle" : el carro giró >= OBS_MEM_LAT_TURN_DEG (arriba). Simple pero
#             IGNORA dónde estaba la lata: 22° de giro despejan una lata muy
#             lateral pero NO una casi al frente -> dispara pronto o tarde.
#   "geom"  : reproyecta la posición inicial de la lata (x0,y0) por el
#             ego-movimiento acumulado (giro real IMU + avance asumido) y
#             pregunta si YA quedó al costado y despejada por _CLEAR_PX.
#             Toma en cuenta posición inicial + giro -> ni antes ni después.
#   "off"   : sin trigger por rotación; solo rebase físico (lat-x / PASADO y /
#             borde). RECUPERANDO entra tarde en esquivas de ángulo.
# (OBS_MEM_LAT_TURN_ENABLED se ignora si esta línea está presente; se deja por
#  compatibilidad: sin MODE, True->"angle", False->"off".)
OBS_MEM_LAT_TURN_MODE      = "angle"   # 2026-08-28: "geom" mató el rendimiento en
                                       # pista (disparaba con la lata detectada ya
                                       # descentrada + ruido de heading). Vuelto a
                                       # "angle" @ 35 (estado que el usuario dejó a
                                       # mano). "geom" sigue disponible pero NO
                                       # calibrado -- ver perillas abajo.
OBS_MEM_LAT_TURN_ENABLED   = True

# ── Perillas del modo "geom" — calibrar en pista, en este orden ──────────────
# 1) _CLEAR_PX: cuántos px BEV al costado del eje del robot debe quedar la lata
#    para contar como librada. Default = radio de inflado (~35).
#    SUBIR  -> rebasa más limpio, RECUPERANDO más TARDE.
#    BAJAR  -> RECUPERANDO más PRONTO (riesgo: rozar la lata).
OBS_MEM_GEOM_CLEAR_PX        = OBS_INFLATE_R
# 2) _AHEAD_MARGIN_PX: gate longitudinal. La lata reproyectada al marco actual
#    del robot debe estar a <= esta "y" adelante para contar como rodeada
#    (NO se exige que quede detrás: basta que la recta de recuperación no la
#    toque). Default ~= lookahead mínimo.
#    BAJAR (o negativo) -> exige la lata más al costado/detrás -> más TARDE.
#    SUBIR -> dispara con la lata aún más adelante -> más PRONTO.
OBS_MEM_GEOM_AHEAD_MARGIN_PX = 55.0
# 3) _MIN_DTHETA_DEG: giro total mínimo del IMU antes de que "geom" pueda
#    disparar. Anti-ruido: en recta el heading tiembla y (x0,y0) puede venir
#    sucio de una detección lejana. Subir si dispara en falso en rectas.
OBS_MEM_GEOM_MIN_DTHETA_DEG  = 8.0
# 4) _SPEED_SCALE: escala SOLO el avance asumido del ancla geom (no toca el
#    mapa que ve la centerline). Si RECUPERANDO entra TARDE -> subir (>1.0);
#    si entra PRONTO -> bajar (<1.0). Corrige que ROBOT_SPEED_MMS=350 no sea
#    la velocidad real durante el latiguazo.
OBS_MEM_GEOM_SPEED_SCALE     = 1.0
# 5) _LEAD_FRAMES: dispara N frames antes para tapar el round-trip serial
#    Pi->ESP32 (~0.3 s). A ~14 fps, 2 frames ≈ 0.14 s. Subir si RECUPERANDO
#    llega tarde por latencia; 0 lo desactiva.
OBS_MEM_GEOM_LEAD_FRAMES     = 2.0

# Anti "pasado" espurio: para disparar RECUPERANDO, la lata debió estar de
# verdad adelante en algún frame (y_min de DETECCIÓN < robot_y - esto). Una
# detección que nace proyectada con y grande (borde inferior del BEV, lata muy
# cerca al arrancar) NO cuenta como rebase -> se descarta callada.
OBS_MEM_PASSED_MIN_AHEAD_PX = 40.0
                                      # (la lata sigue a la altura del carro,
                                      # no muy adelante todavía)

OBS_MEM_MAX        = 12      # tope de obstáculos recordados (seguridad)
OBS_MEM_BEHIND_X_HALFWIDTH = 90.0   # px: al salir la lata por el borde inferior
                                      # del BEV, se cuenta como "pasada" solo si
                                      # |x - robot_x| < esto (rebase real, no
                                      # ruido de rotación que la saca de lado)
OBS_MEM_DEDUPE_PX  = 55.0    # red de seguridad secundaria si igual se duplica -- ver OBS_MEM_MATCH_PX
                              # (mismo riesgo de confundir obstáculos de esquina: se
                              # queda deliberadamente por debajo de OBS_MEM_MATCH_PX)

# Red de seguridad para runtime_nuevo.py: si el ESP32 se queda atorado
# reportando est=G (ack perdido, giro real que nunca termina, etc.), no
# dejar la memoria de obstáculos apagada para siempre — un giro real dura
# bastante menos que esto.
TURN_TIMEOUT_S = 3.0

# ─── Hint direccional (obstáculo lejano, fuera de rango BEV) ─────────────────
# Un objeto rojo/verde detectado en la imagen de cámara CRUDA (no en BEV) que
# todavía no proyecta dentro del rango calibrado.  Se usa SOLO para empezar a
# centrar el steer con anticipación.
FAR_HINT_ENABLED     = True
FAR_HINT_MIN_AREA_PX = 1200    # área mínima del bbox en cámara para confiar (ruido/falsos positivos)
FAR_HINT_MAX_STEER   = 12.0     # grados máx que puede aportar el hint (<< MAX_STEER_DEG)
FAR_HINT_KP          = 0.015   # ganancia proporcional: grados por px de offset
FAR_HINT_KD          = 0.004   # ganancia derivativa: amortigua saltos por ruido de detección
CAM_CENTER_X         = 320     # centro horizontal del frame de cámara (640/2)

# ─── Líneas de esquina (naranja/azul, ver reglamento WRO) — corner_lines.py ───
# Marcadores FÍSICOS FIJOS del tapete, en cada esquina. Recuperado del
# historial (commit 2ab26e2, revertido junto con un intento incompleto de
# disparo de giro — la detección en sí funcionaba).
# Naranja: NO reutiliza FLOOR_LOWER/UPPER_ORANGE (S>=70 ahí es a propósito
# permisivo porque detect_centerline() lo limpia con apertura/cierre
# morfológico antes de usarlo; corner_lines.py no tiene esa limpieza, así
# que ese umbral daba falsos positivos con ruido disperso). Se usa en su
# lugar la última afinación histórica (commit e59a8f0), más estricta.
# Azul: quitado de la ecuación — daba muchos falsos positivos/negativos y no
# era confiable. Por ahora corner_lines.py solo sigue la línea naranja. Se
# deja el rango medido (ver [HSV banda debajo de naranja] en runtime_nuevo.py)
# por si se retoma más adelante, pero detect_lines() ya no lo usa.
# Value tope subido a 255 (antes 210) para no recortar los reflejos brillantes
# de la cinta naranja bajo la luz de arena -- esos pixeles recortados hacían que
# la corrida contigua cruzara/no-cruzara LINE_MIN_RUN_PX frame a frame y
# 'seen'/near_y parpadearan. Hue se mantiene >=7 para no invadir el rojo de las
# latas; S y V se abren un poco para tolerar sombra/desgaste de la cinta.
LINE_ORANGE_HSV = [(np.array([7, 85, 140]), np.array([18, 200, 255]))]
LINE_BLUE_HSV   = [(np.array([120, 30, 5]), np.array([150, 200, 100]))]   # sin usar por ahora

LINE_MIN_RUN_PX   = 8   # ancho mínimo de corrida CONTIGUA en una fila para
                          # contar como línea real (no puntos de ruido dispersos)
LINE_PROXIMITY_PX = 60   # si el punto más cercano de la línea está a esta
                          # distancia (o menos) del robot en Y-BEV, cuenta como "cerca"

# Ajuste de recta CON PENDIENTE (no solo Y) una vez que near_y ya es estable
# — ver corner_lines._fit_line_near()/OrangeLineTracker. Necesario porque la
# línea puede verse inclinada/diagonal en el BEV, no necesariamente horizontal;
# comparar solo Y contra un obstáculo puede clasificarlo mal si está a una X
# distinta de por donde se ancló la lectura.
LINE_FIT_BAND_PX     = 35   # (antes 60) +-px alrededor de near_y de donde se toman
                             # pixeles para el ajuste. Más angosto = menos riesgo de
                             # colar pixeles de OTRO segmento más lejano y torcer la pendiente.
LINE_FIT_MIN_POINTS  = 70   # (antes 30) pixeles mínimos en la banda para confiar en la
                             # pendiente ajustada; si no alcanza, se usa el fallback
                             # horizontal (solo Y). Subido para exigir un segmento bien
                             # visible antes de comprometerse a una pendiente.
LINE_FIT_MAX_SLOPE_DEG = 30  # si la recta ajustada queda más inclinada que esto respecto
                             # a la horizontal se descarta (la línea de esquina en BEV es
                             # ~perpendicular al avance; un ajuste muy diagonal casi
                             # siempre es ruido) -> fallback plano en near_y.

# ─── Suavizado temporal de la línea naranja — ver OrangeLineTracker ───────────
LINE_MASK_CLOSE_KERNEL    = (5, 3)  # cierre morfológico (ancho, alto) sobre la máscara
                                    # naranja antes del run-length: puentea huecos de
                                    # 1-3 px por oclusión parcial / sombra para que un
                                    # segmento real no se parta en dos.
LINE_TRACK_PERSIST_FRAMES = 6    # (2026-08-28: 3->6 al doblar fps ~7->~14) frames seguidos que una lectura nueva debe repetirse
                                 # (mismo 'seen', near_y dentro de tolerancia) antes de
                                 # aceptarla como estado estable.
LINE_TRACK_TOLERANCE_PX   = 20   # (antes 15) margen en near_y para seguir contando la
                                 # misma lectura como "la misma" entre frames.
LINE_TRACK_HOLD_FRAMES    = 4    # (2026-08-28: 2->4 al doblar fps ~7->~14) si 'seen' se pierde, cuántos frames se mantiene la
                                 # última línea estable antes de darla por perdida
                                 # (absorbe dropouts cortos). 0 = soltar de inmediato.
LINE_TRACK_NEAR_Y_EMA     = 0.4  # peso de la lectura nueva al mezclar near_y (EMA).
                                 # 1.0 = sin suavizado. Sube a 0.7 automáticamente cuando
                                 # la línea se ACERCA (near_y crece) para no frenar un
                                 # dato relevante para frenar/clasificar.
LINE_TRACK_LINE_EMA       = 0.35 # idem para los extremos de la recta con pendiente:
                                 # amortigua el "baile" de la diagonal frame a frame.

# Frames tras TERMINAR un giro durante los cuales NO se filtra por la línea
# naranja (todo cuenta como "mi recta", sin excepción) — justo al salir de
# un giro, OrangeLineTracker se reseteó y apenas está re-acumulando lecturas
# sobre la recta nueva; un obstáculo real recién detectado ahí no debe
# arriesgarse a que se clasifique "más allá" por una lectura de línea que
# todavía no se estabilizó sobre datos reales de esta recta.
# 2026-08-28: 10->20 al pasar el pipeline de ~7fps a ~14fps (misma ventana en seg).
TURN_RECOVERY_FRAMES = 20

# Clasificación "mía" / "más allá" de la línea naranja
# (obstacle_memory.classify_and_split): NO es un latch permanente. Cada frame se
# re-evalúa contra la línea, pero para CAMBIAR el veredicto de un objeto hace
# falta que el nuevo salga N frames seguidos -- histéresis asimétrica:
#   - pasar a "mía" (empezar a esquivar algo que se estaba ignorando): pocos
#     frames. Es el lado SEGURO del error (esquivar de más < chocar de menos), y
#     recupera rápido de un "más allá" que se fijó por una mala lectura de línea.
#   - pasar a "más allá" (dejar de esquivar): más frames. Un tramo corto de
#     lectura de línea ruidosa no debe abandonar una esquiva a medias -- ese era
#     el motivo original de la clasificación pegajosa.
# 2026-08-28: duplicados (3->6, 6->12) al pasar el pipeline de ~7fps a ~14fps.
LINE_CLASSIFY_FRAMES_TO_MINE   = 6
LINE_CLASSIFY_FRAMES_TO_BEYOND = 12

# ─── Protocolo serial ESP32 ───────────────────────────────────────────────────
# Cuando pp=1:  ESP32 usa ppSteerGain=60  →  obs=steer_deg/60
# Cuando pp=0:  ESP32 usa visionSteerGain=80  (comportamiento V1 actual)
PP_STEER_GAIN     = MAX_STEER_DEG   # 60.0

# ─── PID de fallback (igual que wro_runtime.py) ───────────────────────────────
RED_TARGET_PX    = 140
GREEN_TARGET_PX  = 500
PID_KP           = 1.000
PID_KD           = 0.33
CORR_LIMIT_PX    = 160.0

# ─── Cámara / captura ─────────────────────────────────────────────────────────
CAM_INDEX      = 1
SERIAL_PORT    = "/dev/ttyS0"
BAUDRATE       = 115200
PROCESS_EVERY  = 3       # procesar 1 de cada N frames capturados
WARMUP_FRAMES  = 40      # frames descartados para estabilizar exposición
