# Handoff: posición lateral y ToF laterales (2026-11-11)

Sesión del 11-nov: carro v2 (Pi 5 + ESP32, solo ToF, sin ToF trasero) en modo ruta.
Aquí va lo que se aprendió, lo que ya quedó en `main` y lo que falta.

## Estado al cerrar

- **ESP:** firmware de carrera de `e89a6c64`, con `MOTOR_LAZO_VEL 1` (control de velocidad prendido).
  Flasheado por COM9. La versión de banco (`FOX_BANCO_SIN_MOTOR`) **no** se llegó a flashear:
  falló la subida porque el ESP no estaba conectado a la PC.
- **Pi:** en `e89a6c64`, servicio `wro-runtime` activo. El log del servicio es persistente
  (`/var/log/journal`): se puede leer el de un arranque anterior con `journalctl -u wro-runtime -b -1`.
- **Logs y videos de las corridas:** en el Pi, `videos_orillas/orillas35..42`. Los de la 35, 36
  y 40 están en `video/corridas_2026-11-11/` (sin commitear).

## 1. Control de velocidad con encoder (`MOTOR_LAZO_VEL`)

- **Problema:** con PWM directo la velocidad depende de la pila. A 255 PWM, ~710 mm/s con 11.1 V
  y ~800 con la pila llena. Con la pila llena la salida del estacionamiento cambiaba
  (se ladeaba −52..−56° contra −45°) y el carro ubicaba el cajón en +337..+458 mm en vez de +170.
  Las corridas 37, 38 y 40 salieron mal.
- **Arreglo (`2318dce6`, prendido en `e89a6c64`):** el PWM que piden los estados se lee como
  velocidad objetivo con el modelo a ~12 V (adelante 3.82·(pwm−45.5), reversa 3.61·(pwm−61)),
  con tope `LAZO_V_MAX` = 700 mm/s. En `setMotorTope()` hay feedforward + PI sobre la velocidad
  del encoder (Kp 0.10, Ki 0.60, I ±80, anti-windup). No se mete al frenar con contramarcha.
- **Gemelo, campo de la 36:** con el lazo, del GO al cajón tarda 29.4–29.5 s de 11.1 a 12.2 V.
  Sin el lazo, de 29.3 a 26.1 s.
- **Real, corrida 41 (pila llena + lazo):** 12 esquinas, `est=E` a 33.45 s (la 36: 33.13 s),
  cajón otra vez en +170.
- **Para apagarlo:** `#define MOTOR_LAZO_VEL 0`. Con `LAZO_V_MAX 0` se quita el tope de 700.
- **Gemelo:** `TWIN_VBAT=<V>` escala k y pwm0 del motor con la pila (`twin/params.py`).

## 2. Cable UART Pi→ESP (corridas 37–38)

- **Síntoma:** `ue=` subía ~400/s con el carro en movimiento, la ruta nunca se confirmaba
  (`[ROUTE] reenvío #N: el ESP no confirmó la ruta`) y el carro manejaba sin plan.
- **Arreglo:** el usuario cambió el cable. En el banco y en las corridas 40–42: `ue=0`.
- **Para diagnosticar:**
  - `ue/uf/ut/ub` en el ACK.
  - Por USB el ESP imprime `UARTERR n=.. brk buf fifo frame`.
  - Antes del GO el ESP no lee el Serial2, así que mandarle texto desde la Pi en ese momento
    siempre da `buf/fifo`: eso es normal y no indica cable malo.

## 3. Problema abierto: la posición lateral está corrida → roza signos en la sección 2

- **Síntomas:**
  - Corrida 41: completa, pero roza un rojo a 12 mm y un verde a 39 mm, ambos en la sección 2.
  - Corrida 42: se atora contra un verde entrando a la esquina 7.
  - El mapa mueve signos entre asientos de la misma recta (S/T1↔S/T3, N/T2↔N/T4).
- **Medición:** se comparó la `lat` de la pose (`[MAP] src=... lat=`) con la que dan los ToF
  laterales, `(tL−tR)/2`, en recta con rumbo ±8°. Script en el scratchpad de la sesión: `latcmp.py`.

  | | Secciones 0 y 1 | Ancho del pasillo (tL+tR+70) |
  |---|---|---|
  | Gemelo | pose − ToF = ±10 mm | ~990 mm |
  | Real (35/36/41/42) | pose − ToF = −110..−250 mm (sección 1 ~−120 en todas) | ~865–920 mm (debería ser 1000) |

- **Sesgo del ToF lejano:** se calculó lo que debería leer usando el cercano como referencia y
  suponiendo pasillo de 1000:

  | Distancia real | ToF derecho | ToF izquierdo |
  |---|---|---|
  | ~57 cm | −111 mm | — |
  | ~62 cm | −161 mm | −68 mm |
  | ~67 cm | −202 mm | — |
  | ~72 cm | −214 mm | −95 mm |

  Concuerda con `twin/calib/medidas_20261021/tof/NOTAS_TOF.md`: Short ROI 16 da −5 cm a 60 cm
  y −19 cm a 80. El derecho nunca se calibró: es otra unidad y otro montaje.
- **Hipótesis:** el localizador acepta laterales hasta `LOC_TOF_MAX_MM` 500 mm. Más allá de
  ~45 cm leen corto, el derecho mucho más, así que la pose se recorre hacia esa pared y la ruta
  pasa 100–250 mm más pegada a los signos. El gemelo modela los ToF sin ese sesgo; por eso ahí
  la posición sí cuadra y no reproduce el problema.
- **Siguiente paso:**
  1. Medir con flexómetro el ancho real del pasillo en dos rectas, para descartar que la pista
     no mida 1000.
  2. Calibración estática de tL y tR:
     - Agregar `#define FOX_BANCO_SIN_MOTOR 1` solo al `PurePursuitCarrera.ino` generado
       (no commitear) y flashear.
     - GO desde la Pi: `pinctrl set 17 op dh; sleep 0.4; pinctrl set 17 ip pd`.
     - Poner el carro paralelo a la pared a 20/30/40/50/60/70 cm por cada lado y leer tL/tR
       del ACK en el journal.
  3. Con la curva: corregir tL/tR en la Pi antes del localizador (o bajar `LOC_TOF_MAX_MM`
     laterales a lo que salga confiable) y poner el mismo sesgo en el modelo de ToF del gemelo.
  4. Probar en el gemelo con el campo de la 36 (`TWIN_FIELD36=1`) y varias semillas.
     El gemelo ya chocaba en N/T2 a 11.4–11.7 V.
  5. Volver a flashear la versión de carrera (`exportar_carrera ruta_perfil
     pure_pursuit/twin/tools/carrera_bench.json --pi-baud 460800`).

## Otros datos de la sesión

- **Corrida 36 contra el gemelo a 11.1 V:** la velocidad por PWM cuadra (~700–720 mm/s a 255).
  El real tardó 33.1 s contra 29.2 s porque recorrió ~8% más (20.9 contra 19.3 m): zigzaguea más
  (servo 24–119 contra 39–92 en el gemelo).
- **Pi:**
  - Cámara con Picamera2 escalando en el ISP: 40 fps con 5% de CPU.
  - El grabber espera frame nuevo y `PROCESS_EVERY 1`: ~33 Hz.
  - Llega a 77–84 °C sin ventilador.
- **Grabación:** se cortaba a los ~7 s; arreglado en `35b35cf7`. El video de la 40 grabó casi
  todo la mesa, no la pista.
- **Log:** journald todavía suprime líneas (`Suppressed N messages`) a pesar del drop-in
  `LogRateLimitIntervalSec=0`; falta revisar el límite global.
