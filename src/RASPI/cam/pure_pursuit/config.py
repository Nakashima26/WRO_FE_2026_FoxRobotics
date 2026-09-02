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
CENTERLINE_RAMP_PX   = 130   # horizonte de anticipo: empieza a abrir el path
                             # hacia el lado de paso a esta distancia Y de la lata.
                             # 2026-08-28: 200 -> 130 (con 200 el peso de las filas
                             # cerca del robot ya era ~0.5 con el cono lejos -> path
                             # se abría fuerte de lejos -> volantazo temprano).
                             # 2026-08-29: probé 180, EMPEORÓ (peso near-row 0.38
                             # -> 0.60 con el cono a 190mm -> obs saturaba) ->
                             # revertido a 130.
CENTERLINE_EXIT_RAMP_PX = 90   # 2026-08-28: sobre cuántos px de Y (pasada la
                              # lata) el peso de esquiva decae 1->0. Antes era
                              # OBS_INFLATE_R (~39px, ~2 filas) -> el path se
                              # enganchaba de golpe al centro apenas librado el
                              # círculo ("brinco hacia adentro"). 90px = arco de
                              # salida limpio, sin sesgar tanto tramo por delante
                              # que estorbe a la siguiente lata.
CENTERLINE_SMOOTH_WIN = 5    # ventana (impar) de media móvil sobre X post-muestreo
CENTERLINE_DEBUG      = False  # 2026-08-29: APAGADO. El log [CLDBG] fila-a-fila
                              # llenaba el journal de la Pi (74MB) y rotaba los
                              # datos útiles en minutos. Encender solo para
                              # depurar un brinco de path puntual.
CENTERLINE_COMMIT_W   = 0.12   # peso de esquiva a partir del cual el punto del
                              # path se FUERZA a la banda [ox+INFLATE_R,
                              # ox+off_max] del lado WRO correcto, pase lo que
                              # pase con la máscara de piso (siempre hay paso
                              # físico por ahí). Bajar = se compromete al lado
                              # antes; subir = deja que la máscara mande más
                              # tiempo.

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
OBSTACLE_URGENT_MM = 180.0   # <= esto: reacción full (gain 1.0, lookahead corto)
# A esta distancia (o más): comportamiento normal/suave, sin urgencia.
# 2026-08-28: 350 -> 260. 2026-08-29: probé 450 (+ GAIN_MIN 0.15) para suavizar
# la reacción lejana; no ayudó al problema real (pivote por lookahead corto) y
# metía ruido -> revertido a 300 (compromiso, era 260).
OBSTACLE_CASUAL_MM = 300.0   # >= esto: reacción suave (gain STEER_DIST_GAIN_MIN)

# 2026-09-01: margen antes de la pared frontal (dF del ESP) para el filtro
# geométrico de runtime_nuevo: un cono a >= (dF*10 - esto) mm de frente se
# considera de la SIGUIENTE recta (está en/pasando la pared) y no se esquiva.
# Absorbe el offset sensor-frontal ↔ origen BEV y el diámetro del cono.
DODGE_WALL_MARGIN_MM = 100.0

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
LOOKAHEAD_MIN_PX      = 78.0   # 2026-08-28: 70->60 ("se sentía floja"). 2026-08-29: 60->78.
                              # orillas416: obs se iba a +0.85..+0.93 (casi tope) en cuanto
                              # veía el cono -> a ese lock y velocidad de crucero el carro
                              # NO traslada, PIVOTEA en el sitio (y del cono clavada, ~40mm/s
                              # vs ~300 crucero) -> la esquiva nunca "sale" como arco. 60px
                              # (120mm) hace que CUALQUIER offset lateral del path cerca del
                              # carro sature el steer (el propio comentario: "45px saturaba",
                              # "70px sin pegar el límite"). 78 lo baja a ~0.5 -> el carro
                              # sigue avanzando y esquiva en arco, no pivote.
LOOKAHEAD_MAX_PX      = 100.0
LOOKAHEAD_OBS_NEAR_PX = OBSTACLE_URGENT_MM / MM_PER_PX   # =90px  (180/2)
LOOKAHEAD_OBS_FAR_PX  = OBSTACLE_CASUAL_MM / MM_PER_PX   # =225px (450/2)

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
STEER_DIST_GAIN_NEAR_PX = OBSTACLE_URGENT_MM / MM_PER_PX   # =90px  (180/2)
STEER_DIST_GAIN_FAR_PX  = OBSTACLE_CASUAL_MM / MM_PER_PX   # =225px (450/2)
STEER_DIST_GAIN_MIN     = 0.30  # 2026-08-28: 0.8 -> 0.45 -> 0.30. (2026-08-29 probé
                                  # 0.15 junto con OBSTACLE_CASUAL_MM=450; revertido,
                                  # el problema real era el lookahead corto.)
                                  # Reacción lejana suave; dentro de ~90px sube a 1.0.


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
OBS_MEM_BEHIND_PAD = -35    # 2026-08-28: -18 -> -75 (rojo bien) pero -75 mató al
                            # verde: layout con verde a 40cm de la pared EXTERIOR
                            # -> traverse gigante -> a y=305 el verde sigue 150mm
                            # ADELANTE + al lado, RECUPERANDO enderezó el morro
                            # contra el verde. -35 (behind_y=345) es el punto medio:
                            # rojo un poco más tarde, verde no dispara prematuro.
                            # px: tirar el obstáculo (y disparar "pasado" ->
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
# obstacle_memory._lat_pass(), elegido por OBS_MEM_LAT_TURN_MODE:
#
#   "geom"  : GEOMÉTRICO por dead-reckoning del ancla (x0,y0 + giro + arco). Se
#             re-parchó 4 veces (angle, geom, geom v2, SPEED_SCALE 1.0/0.35/0.60)
#             y NUNCA funcionó en pista: el ancla integra IMU + modelo de
#             bicicleta + velocidad no medida y deriva 200-400mm en 1-2s ->
#             disparo prematuro (morro contra la lata) o tardío (57° de error).
#   "angle" : el carro giró >= OBS_MEM_LAT_TURN_DEG. Ignora dónde estaba la lata.
#   "off"   : sin rebase lateral en obstacle_memory; el rebase de ángulo lo
#             decide runtime_nuevo._measured_recup_trigger() por ESTADO MEDIDO
#             (el planner dejó de rodear la lata + chasis chueco), que NO integra
#             error. "PASADO y" de _prune (OBS_MEM_BEHIND_PAD) queda como
#             respaldo para el rebase DE FRENTE (la lata sale por abajo del BEV).
# 2026-08-29: "off". geom retirado como disparo (código sigue, dormido). El
# rebase lateral ahora es el trigger medido de runtime_nuevo (RECUP_MEAS_*).
# 2026-09-01 (rama SectionTurning): se probó "angle" y disparaba RECUPERANDO
# TEMPRANO (orillas445: con la lata aún 40-58px adelante) — lat-giro y lat-x
# disparan por el YAW de la esquiva, no porque la lata quedó atrás. De vuelta a
# "off" + trigger MEDIDO. Lo que sí se mantiene: supresión por naranja apagada
# (RECUP_SUPPRESS_NEAR_ORANGE_Y=9999). El giro falso cerca de la esquina se
# arregla en el ESP32 (trigger de giro nuevo), no matando el pasado en la Pi.
OBS_MEM_LAT_TURN_MODE      = "off"
OBS_MEM_LAT_TURN_ENABLED   = False  # compat (sin MODE: True->"angle")

# ─── Trigger de RECUPERANDO por ESTADO MEDIDO — runtime_nuevo._measured_recup_trigger ─
# Sustituto del ancla "geom". No hace dead-reckoning de la lata: mira el estado
# MEDIDO este frame y dispara RECUPERANDO cuando se cumplen las 3 cosas:
#   1) hubo una esquiva DE VERDAD en curso  (peso de rampa de detect_centerline
#      >= RECUP_MEAS_ARM_W en alguna fila junto al eje en algún frame),
#   2) el planner YA no rodea nada en las filas junto al eje (peso <=
#      RECUP_MEAS_CLEAR_W) por RECUP_MEAS_CLEAR_FRAMES frames seguidos,
#   3) el chasis quedó chueco respecto a la recta (|heading - heading_ref| >=
#      RECUP_MEAS_HEADING_DEG)  -> hay algo que enderezar.
# Separa solo con (3) el caso "esquiva suave con espacio" (heading chico -> PP +
# wall PID solos, NO dispara) del "latiguazo sin espacio" (heading grande ->
# dispara justo al terminar de rodearla).
RECUP_MEAS_ENABLED        = True   # rama SectionTurning: de vuelta a True. "angle" disparaba
                                   # RECUPERANDO temprano (orillas445). El medido espera a que
                                   # la lata quede atrás de verdad.
RECUP_MEAS_NEAR_PX        = 120.0  # px BEV por delante del eje: banda de filas cuyo peso
                                   # de esquiva se mira para ARMAR (hubo lata rodeada)
RECUP_MEAS_ARM_W          = 0.35   # peso de esquiva que ARMA el trigger
RECUP_MEAS_AHEAD_TOL_PX   = 60.0   # 2026-08-29: 30->90 (orillas412) -> 50 (orillas416).
                                   # La lata deja de "estorbar" cuando queda a <= esto
                                   # LONGITUDINALMENTE del eje (ry-oy). Con 90 el VERDE
                                   # disparaba measured a herr+45 con la lata aún 84px
                                   # adelante (su proyección BEV de cono alto la lee ~70px
                                   # más lejos de lo que está) -> RECUPERANDO metió el morro.
                                   # 50: la lata que se lee mal (verde) cae al respaldo
                                   # BEHIND_PAD, que solo dispara con y>345 (ya detrás de
                                   # verdad, y como la proyección la lee CORTA, la real está
                                   # aún más atrás -> seguro). El rojo, cuya y sí avanza,
                                   # sigue disparando measured ~10° antes que BEHIND_PAD.
# 2026-08-29 (orillas415): ahead_tol ENCOGE cuando el giro es grande. Un
# latiguazo grande barre un arco grande al enderezar; si la lata sigue adelante,
# RECUPERANDO le mete el morro (verde disparó a herr+59 con la lata 72px
# adelante -> choque). Rampa: tol pleno hasta HARD_LO grados, baja lineal a 0 en
# HARD_HI. El rojo de orillas415 (herr-47, funcionó bien) queda casi intacto
# (tol ~82); el verde (herr+59) sube su umbral a ~35 -> espera a que quede al
# lado antes de disparar.
RECUP_MEAS_AHEAD_TOL_HARD_LO = 32.0
RECUP_MEAS_AHEAD_TOL_HARD_HI = 58.0
RECUP_MEAS_CLEAR_W        = 0.05   # respaldo: peso junto al eje por debajo de esto = despejado
RECUP_MEAS_ARM_FRAMES     = 3      # 2026-08-29: la esquiva debe estar en curso tantos frames
                                   # seguidos (peso alto) antes de ARMAR. Un verde que se ve
                                   # 1 frame y se pierde (tras giro, FOV rasante) ya no arma
                                   # -> no entra a RECUPERANDO "super rápido" con la lata aún
                                   # enfrente (orillas417/418, reporte del usuario).
RECUP_MEAS_CLEAR_FRAMES   = 3      # frames seguidos "despejado" antes de poder disparar (~0.2s @14fps)
RECUP_MEAS_CLEAR_FRAMES_CORNER = 1  # ...PERO si la línea naranja ya está encima
                                   # (near_y >= RECUP_SUPPRESS_NEAR_ORANGE_Y): disparar en
                                   # cuanto el path despeja 1 frame. El debounce de 3 dejaba
                                   # 2 frames con prio=0/mem=0/pasado=0 y el chasis chueco ->
                                   # el ESP32 metía un detectarEsquina() falso -> GIRANDO en
                                   # vez de RECUPERANDO (Verde6/Rojo2/CW, orillas440, ~2/3).
                                   # El pasado espurio (memory.last_passed) sigue suprimido
                                   # cerca de la esquina; esto solo adelanta el MEDIDO.
RECUP_MEAS_GENTLE_FRAMES  = 10     # despejado tantos frames CON heading siempre < HEADING_DEG
                                   # => fue esquiva suave, se desarma sin RECUPERANDO (~0.7s)
RECUP_MEAS_HEADING_DEG    = 25.0   # 2026-08-29 (15->25 tras orillas412): giro mín. vs la
                                   # recta para disparar. En pista una esquiva de verdad
                                   # llega a 35-64°; por debajo de 25 es deriva/esquiva
                                   # suave -> PP + wall PID solos.

# Frames que runtime repite pasado=1 al ESP32 (un mensaje serial perdido si no
# retrasaría/perdería RECUPERANDO). El ESP32 consume el pulso e ignora repeticiones.
PASADO_HOLD_FRAMES        = 6

# 2026-09-01: tras TERMINAR un pulso pasado=1, no arrancar otro por estos frames.
# El chasis todavía se está asentando tras RECUPERANDO; un 2º pasado encima
# (típicamente un fantasma de memoria cruzando behind_y, o un smear del mismo
# cono) mete al ESP a RECUPERANDO de nuevo y el carro sobre-corrige. orillas473:
# el fantasma del rojo re-disparó pasado ~2s después, justo sobre el verde ->
# doble RECUPERANDO -> heading a +42° -> se comió el verde. ~15 frames (~1.2s).
PASADO_COOLDOWN_FRAMES    = 15

# Suprimir pasado=1 (=> no RECUPERANDO) si la línea naranja está a near_y >= esto
# en el BEV (robot en y=380, así que 285 = línea a <=~190mm = esquina inminente).
# En RECUPERANDO el ESP32 no evalúa detectarEsquina() -> un pasado justo antes de
# la esquina metía el giro ~0.5s tarde y el carro se llevaba el cono de la recta
# siguiente (orillas420/421). El giro mismo endereza el heading.
# 2026-09-01 (rama SectionTurning): NEUTRALIZADO (9999) — esta supresión era la
# causa del "GIRANDO en vez de RECUPERANDO cerca de la naranja". Con la maniobra
# por-tramos la esquina la maneja APROXIMANDO (no pasado->RECUPERANDO), así que
# ya no hace falta suprimir. Volver a 285 si se retoma el giro continuo.
RECUP_SUPPRESS_NEAR_ORANGE_Y = 9999.0

# ...PERO NO suprimir si el pasado vino del trigger MEDIDO (esquiva de ÁNGULO
# real — cono rojo/verde en la MISMA recta, cerca de la esquina). Ahí el chasis
# quedó chueco y RECUPERANDO SÍ hace falta: sin él, el carro ladeado dispara un
# FALSO detectarEsquina() (ultrasónico lateral lee "sin pared" por el yaw) y
# gira ~90° encima del ladeo (reporte del usuario 2026-08-31: "esquiva rojo en
# la recta, tenía que entrar recuperando, en cambio entró a girando"). La
# supresión sigue matando el pasado ESPURIO (memory.last_passed / BEHIND_PAD
# head-on, sin esquiva de ángulo), que era el caso de orillas420/421.
# False = supresión ciega de siempre (revertir si esto retrasa el giro y se
# lleva el cono de la recta siguiente).
RECUP_SUPPRESS_KEEP_MEASURED = True

# Tras un rebase MEDIDO cerca de una esquina (RECUP_SUPPRESS_KEEP_MEASURED):
# una vez que RECUPERANDO terminó de enderezar, forzar prio=1 (giro bloqueado)
# estos frames más antes de soltar el giro. Sin esto el ESP32 disparaba
# detectarEsquina() ~0.5s ANTES del punto real (reporte del usuario,
# orillas429: "el giro se activó mucho antes, le faltó medio segundo").
# ~3 frames @ ~15fps ~= 0.2s. Subir si el giro sigue entrando temprano.
RECUP_CORNER_TURN_DELAY_FRAMES = 3

# est=G debounce: frames CONSECUTIVOS de est=G en el ACK del ESP32 antes de dar
# el giro por real y borrar/apagar la memoria de obstáculos. Un est=G espurio
# (ACK con ruido, "est=G fantasma tras verde") ya NO dispara el wipe de memoria
# a media esquiva -> RECUPERANDO deja de perder la lata que venía siguiendo.
TURN_EST_G_CONFIRM_FRAMES = 2

# ─── Detector de obstáculos DURANTE el giro (mid_turn.py) ────────────────────
# FASE 1: solo observa y registra (línea [MTURN] en journalctl). NO cambia el
# steering ni manda nada al ESP32. Sirve para medir en pista si la detección
# mid-turn confirma latas reales sin fantasmas, antes de cablearla al firmware.
# La memoria rodante sigue apagada durante el giro; esto es aparte.
MIDTURN_WINDOW            = 4      # frames de historia (ring buffer)
MIDTURN_CONFIRM_FRAMES    = 3      # de esos, cuántos deben coincidir en color+posición
MIDTURN_ROI_MAX_MM        = 280.0  # distancia real máx. robot->lata para contarla
MIDTURN_ROI_HALF_ANGLE_DEG = 45.0 # semiapertura del cono "hacia adelante" del ROI
MIDTURN_POS_TOL_PX        = 50.0   # tolerancia de posición BEV entre frames (100 mm)
MIDTURN_MIN_GYRO_DEG      = 25.0   # no mirar antes de este avance de giro (los
                                   # primeros grados ven la esquina / la recta que
                                   # se deja atrás, no la recta nueva)
MIDTURN_SIDE_DEADBAND_PX  = 24.0   # |bev_x - eje| bajo esto -> lado '?' (indeciso)

# ── MODO "geom" — cómo funciona ─────────────────────────────────────────────
# Al detectar la lata se guarda su posición (x0,y0) y el heading del IMU.
# Cada frame se reproyecta esa posición al marco ACTUAL del robot aplicando el
# ego-movimiento: rotación = Δheading REAL del IMU (exacto), avance = longitud
# de arco s = R·Δθ con R = WHEELBASE/tan(steer) del modelo de bicicleta (usa el
# steer que de verdad se comandó, NO una velocidad adivinada). El ancla o.xr/
# o.yr en obstacle_memory hace justo esa integración.
#
# La lata está "rodeada" cuando, en el marco actual, queda:
#     |X_r| >= CLEAR_PX      (fuera del pasillo recto del robot, a cualquier lado)
#   Ó  Y_r  <= AHEAD_MARGIN  (ya al costado / detrás del eje)
#
# ── Constantes DERIVADAS (no ajustar salvo que cambie el hardware) ──────────
# CLEAR_PX = medio ancho del chasis + inflado de la lata. Si el centro de la
# lata está a >= esto del eje de avance, el borde del carro libra el borde
# inflado de la lata yendo recto.
ROBOT_HALF_WIDTH_PX      = round(140.0 / 2.0 / MM_PER_PX)        # chasis 140mm -> 35 px
OBS_MEM_GEOM_CLEAR_PX    = ROBOT_HALF_WIDTH_PX + OBS_INFLATE_R   # 35 + 36 = 71 px
# ── Perillas de AJUSTE FINO (mover solo estas en pista) ─────────────────────
# AHEAD_MARGIN_PX: cuánto puede seguir ADELANTE la lata (marco actual) y aún
# contar como rebasada. 0 = exactamente al través. +N = adelanto (latencia
# serial). NEGATIVO = exige que quede N px DETRÁS del eje trasero -> más margen.
# Subir -> RECUPERANDO más PRONTO.  Bajar/negativo -> más TARDE.
OBS_MEM_GEOM_AHEAD_MARGIN_PX = 0.0
# MIN_DTHETA_DEG: giro real mínimo del IMU antes de que "geom" pueda disparar.
# Garantiza que hubo una esquiva de verdad (una esquiva rota >=20-30°; el ruido
# en recta <8°). Subir si dispara en falso en tramos rectos.
OBS_MEM_GEOM_MIN_DTHETA_DEG   = 12.0
# SPEED_SCALE: multiplicador final del avance del ancla (escape hatch). Si en
# pista RECUPERANDO entra sistemáticamente TARDE -> subir (1.1-1.3); si entra
# PRONTO -> bajar. Dejar en 1.0 salvo evidencia.
OBS_MEM_GEOM_SPEED_SCALE      = 0.60  # 2026-08-28: 1.0 -> 0.35. En el pivote del
                                      # verde (steer suave, yaw ~2°/frame, casi
                                      # sin traslación) el modelo de bicicleta
                                      # sobre-marcha el ancla ~3x -> geom disparaba
                                      # PASADO con el verde aún 130mm al frente.
                                      # 0.35 alinea el avance del ancla con el
                                      # avance real medido (detección y).
# El ancla geom (x0,y0,heading0) NO se fija hasta que la lata se detecta a
# `y >= esto` en el BEV. Más arriba (horizonte) la proyección cámara->BEV es
# basura -> el dead-reckoning arrancaba de un punto inventado (visto en pista:
# ancla Xr=-124 Yr=+212 con la lata enfrente -> RECUPERANDO -> choque).
OBS_MEM_ANCHOR_MIN_Y          = 200.0

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
OBS_MEM_DEDUPE_PX  = 85.0    # 2026-08-29: 55 -> 85. Un cono cerca de la cámara se
                              # re-proyecta saltando >55px frame a frame -> _merge
                              # creaba 2 registros que _dedupe no fusionaba -> nobs=2
                              # fantasma, steer oscilando (lap 3 orillas420). Los
                              # obstáculos WRO reales están a >=200mm (>=100px BEV) así
                              # que 85 no confunde dos latas de verdad.

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
LINE_FIT_BAND_PX     = 45   # (2026-08-29: 35 -> 45) +-px alrededor de near_y de donde
                             # se toman pixeles para el ajuste. Se amplió para juntar
                             # los MIN_POINTS antes (línea recién vista, delgada).
LINE_FIT_MIN_POINTS  = 35   # 2026-08-29: 70 -> 35. Con 70, la línea naranja recién
                             # vista (lejos, delgada) no juntaba 70px -> _fit_line_near
                             # devolvía None -> classify() caía al FALLBACK HORIZONTAL
                             # -> un rojo que está PASANDO la esquina caía en "mi recta"
                             # y el carro lo esquivaba (orillas420, reporte del usuario).
                             # 35 permite estimar la pendiente desde los primeros frames;
                             # el EMA de _smooth_line (line_ema=0.35) amortigua el ruido.
LINE_FIT_MAX_SLOPE_DEG = 72  # 2026-08-29: 30 -> 45. Al acercarse a la esquina en ángulo,
                             # o esquivando (chasis ladeado ~20°), la línea SÍ se ve
                             # diagonal en el BEV -> descartar >30° forzaba el horizontal
                             # equivocado. 45° deja pasar la inclinación real sin colar
                             # una vertical de ruido.
                             # 2026-08-31: 45 -> 72. La MISMA línea de esquina se ve a
                             # ~30° en un sentido de vuelta y a ~60° en el otro (el
                             # ángulo con que la cruza el chasis cambia con la
                             # dirección) -> con 45 en medio, un sentido ajustaba bien
                             # y el otro caía SIEMPRE al fallback horizontal
                             # (clasificación mala, el giro no se armaba -- reporte del
                             # usuario). 72 acepta los ~60° reales; una vertical de
                             # ruido (>72°, columna de píxeles / borde de cono) sigue
                             # fuera. DIST_HUBER + MIN_POINTS + EMA + PERSIST_FRAMES
                             # filtran un ajuste malo de un frame suelto.
LINE_FIT_MIN_X_SPAN_PX = 150 # 2026-09-01: los pixeles del ajuste deben abarcar al
                             # menos esto en X. Una línea de esquina real cruza el
                             # BEV (~400px de ancho); el borde de un cono / una
                             # columna de ruido abarca ~30-50px pero puede colar un
                             # fit de ~60° (< MAX_SLOPE_DEG) que luego manda TODO lo
                             # de adelante a "beyond" (orillas471: line[1]~+728 con
                             # un rojo de la recta actual justo ahí).

# 2026-09-01: OrangeLineTracker.classify() SOLO opina cuando hay un ajuste de
# recta con pendiente real (stable["line"] != None). El fallback `oy > near_y`
# (línea "vista" pero sin ajuste) rompía una y otra vez: tras un RECUPERANDO el
# tracker re-adquiere sobre ruido, near_y se congela CERCA y espurio con
# line=None, y un cono de MI recta queda `beyond` -> ignorado (orillas471/473/
# 475/476). Con LINE_FIT_MIN_X_SPAN_PX filtrando el ruido, line=None ~= "no hay
# línea real". False = volver al fallback frágil.
LINE_CLASSIFY_REQUIRE_FIT = True

# ─── Dirección de giro — TurnDirectionTracker ───────────────────────────────
# PRIMARIA: posición lateral de un obstáculo "beyond" (ver corner_lines.py).
#
# La PENDIENTE de la naranja se probó como primaria (2026-08-31) y FALLÓ: el
# `vy` (=line[1]) sigue a la DISTANCIA a la línea, no al sentido de giro —
# distorsión del BEV en campo lejano. En una sola aproximación al 1er giro:
#   near_y=173 (lejos) vy=+170  |  near_y=291 (cerca) vy=-5  |  near_y=318 vy=+126
# Latcheó "L" con la pista girando a la DERECHA. Queda como CONFIRMACIÓN
# opcional, apagada por defecto: si se enciende, solo se lee con la línea
# cerca (near_y >= LINE_DIR_MIN_NEAR_Y) y solo puede VETAR al obstáculo
# (conflicto -> no vota), nunca fijar sola.
LINE_DIR_FROM_SLOPE_ENABLED = False
LINE_DIR_MIN_NEAR_Y    = 285.0   # solo mirar la pendiente con la línea a <=~190mm
LINE_DIR_SLOPE_DEADBAND = 20.0   # |vy| por debajo de esto = no opina

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

# 2026-09-01: misma idea pero tras un RECUPERANDO medido (esquiva de ángulo). El
# chasis se asienta y OrangeLineTracker re-adquiere sobre ruido -> un near_y
# espurio y CERCA se congela y clasifica "beyond" un cono de MI recta
# (orillas473/475: rojo esquivado -> RECUPERANDO -> verde ignorado). Durante
# estos frames no se filtra por la naranja (todo = mío) ni vota turn_dir.
RECUP_RECOVERY_FRAMES = 20

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
# 2026-08-29: veredicto por PRIMERA vez de un obstáculo sin clasificar
# (o.beyond is None). Antes, un obstáculo nuevo era "mío" AL INSTANTE pero
# "siguiente recta" tardaba TO_BEYOND(12) -> un rojo que estaba pasando la
# esquina se esquivaba ~1s antes de corregirse (orillas420). Este umbral chico
# aplica SOLO al primer veredicto; cambiar uno ya fijado sigue usando 6/12.
LINE_CLASSIFY_FRAMES_FIRST     = 4

# 2026-09-01: PISO DE CERCANÍA. Un obstáculo con y-BEV >= esto está casi bajo la
# nariz (behind_y de _prune = ry+BEHIND_PAD = 345): físicamente NO puede ser de
# la siguiente recta -- para tenerlo tan cerca ya habrías cruzado la esquina, y
# ahí manda CRUCERO/MANIOBRA, no la esquiva. classify_and_split() lo fuerza a
# "mía" ignorando la línea y rompe el latch `beyond` de una (sin esperar
# LINE_CLASSIFY_FRAMES_TO_MINE). Motivo: orillas471 -- la naranja se fijó mal
# (near_y saltando 157->318, fits de pendiente basura line[1]~+728) -> un rojo de
# la recta actual quedó `beyond` desde el 1er frame, el latch nunca se soltó y el
# carro lo embistió. Ventana 290->345 = ~4 frames para mandarlo al ESP antes de
# que _prune lo tire como PASADO.
CLASSIFY_FORCE_MINE_Y          = 290.0
# 2026-09-01: el piso SOLO aplica con confianza >= esto. Un fantasma de memoria
# (camX=0, conf decayendo ~0.06/frame) extrapolado hasta y>=290 no debe forzar
# esquiva ni romper el latch -- orillas473: un rojo ghost conf 0.52 metió steer
# izquierdo espurio durante la esquiva del verde. 0.65 ~= visto en los últimos
# ~5 frames.
CLASSIFY_FORCE_MINE_MIN_CONF   = 0.65

# ─── Protocolo serial ESP32 ───────────────────────────────────────────────────
# Cuando pp=1:  ESP32 usa ppSteerGain=60  →  obs=steer_deg/60
# Cuando pp=0:  ESP32 usa visionSteerGain=80  (comportamiento V1 actual)
PP_STEER_GAIN     = MAX_STEER_DEG   # 60.0

# 2026-08-28: Excepción "interior pass". Cuando el obstáculo se esquiva hacia
# el mismo lado que la pista va a girar, la Pi mandaba intr=1 y el ESP32 dejaba
# de bloquear detectarEsquina() por ese obstáculo. Rompió la garantía dura
# "veo obstáculo -> no giro": el TurnDirectionTracker fijó "L" MAL (el rojo del
# arranque se clasificó "beyond"), eso puso intr=1 para todo verde, y el ESP32
# giró en pleno latiguazo ladeado a +37deg -> falsa esquina -> choque.
# False = intr SIEMPRE 0 = garantía dura restaurada. Volver a True solo cuando
# el turn-dir esté confiable Y el ESP32 tenga el gate de alineación.
INTERIOR_PASS_ENABLED = False

# ─── Obstáculo EXTERIOR pegado a la boca de la esquina ───────────────────────
# Caso (reporte del usuario, HUD orillas424): la pista va a girar (naranja
# cerca) y justo en la boca de la esquina hay un cono cuyo lado de paso WRO es
# el CONTRARIO al giro -> verde con giro a la DERECHA, o rojo con giro a la
# IZQUIERDA (o sea, NOT is_interior_pass). Ahí no se puede girar todavía: hay
# que pasar el cono COMPLETO por el lado correcto (el exterior del giro), yendo
# recto, y solo cuando ya quedó atrás soltar el giro.
#
# Sin esto, classify_and_split() lo manda a "beyond" (está al otro lado de la
# naranja) -> desaparece del plan -> prio=0/mem=0 -> el ESP32 ve vía libre y
# gira CONTRA el cono.
#
# Fix (solo Pi, NO toca PurePursuit.ino): mientras se cumpla el caso, ese cono
# se RESCATA de "beyond" y se trata como "mío":
#   (a) sigue en bev_obstacles -> la centerline lo esquiva por el lado WRO
#       correcto (_clamp_to_pass_side: verde->izquierda, rojo->derecha),
#   (b) prio=1/mem>0 -> el ESP32 mantiene bloqueado detectarEsquina().
# Cuando el cono cruza el eje, _prune lo tira; como la naranja está cerca,
# RECUP_SUPPRESS_NEAR_ORANGE_Y ya anula el pulso `pasado` -> el ESP32 pasa
# directo a GIRANDO. NO es el intento viejo de "interior pass" (ese soltaba el
# giro ANTES via intr=1; esto lo RETIENE). No se toca intr ni el .ino.
# 2026-09-01 (rama SectionTurning): OFF. En el modelo por-tramos todo lo que
# está pasando la naranja es "beyond" y se maneja cuando ya estás en esa recta
# (tras la maniobra). No se rescata ningún cono exterior -> rescue_fn=None,
# _ext_corner_hold nunca se activa, y toda la lógica CORNER_EXT_PASS_* queda
# inerte.
CORNER_EXTERIOR_PASS_ENABLED = False

# Dirección de giro de la pista para ESTE campeonato. El equipo la sabe al
# montar la pista. Si se fija ("L" o "R"), se usa para decidir interior/exterior
# en vez de la que infiere TurnDirectionTracker por visión — que en el intento
# viejo de interior-pass se fijó MAL (rojo de arranque mal clasificado) y causó
# el choque. None = usar el tracker de visión (comportamiento de siempre).
CORNER_TURN_DIR_OVERRIDE = None   # None | "L" | "R"

# El cono cuenta como "en la boca de la esquina" si la naranja está a near_y
# >= esto. 2026-08-31: era RECUP_SUPPRESS_NEAR_ORANGE_Y (285) y en los frames
# del usuario el verde se escapaba a "beyond" a near_y~281 (antes de llegar a
# 285) -> se dejaba de esquivar a media aproximación -> reaparecía como
# fantasma. 230 lo agarra apenas la naranja se ve estable, con pista por
# delante, y no se suelta.
CORNER_EXT_PASS_NEAR_ORANGE_Y = 230.0

# ...y solo si el cono no está más de esto ADELANTE de la línea naranja (px
# BEV; y decrece hacia adelante -> "no más allá de BAND" = oy >= near_y - BAND).
# Un cono ya metido en la recta siguiente NO se rescata: lo resuelve esa recta
# después de girar. Subir si el cono de esquina se ve más lejos de la línea.
CORNER_EXT_PASS_BAND_PX = 70.0

# Cap de |steer_deg| mientras se rescata el cono exterior. 2026-08-31: era 12
# ("ir vertical"); el usuario pidió que ESQUIVE de verdad (como un cono
# normal) porque ir recto lo rozaba. 25 deja arquear lo que pida la centerline
# sin dar un volantazo. 0 = sin cap.
CORNER_EXT_PASS_MAX_STEER_DEG = 25.0

# Tras rescatar un cono exterior, cuántos frames más se mantiene prio=1 (giro
# BLOQUEADO) DESPUÉS de que el cono desaparece de memoria. Arregla lo de los
# frames del usuario: el fantasma del verde se poda como "PASADO" por arrastre
# (y>345) cuando el carro NO lo pasó de verdad -> GIRANDO entraba y barría
# hacia el cono. Con esto el giro no se suelta hasta ~10 frames (~0.7s ~=
# 240mm de avance) después de que el cono se fue = genuinamente atrás.
CORNER_EXT_PASS_TURN_BLOCK_FRAMES = 10

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
