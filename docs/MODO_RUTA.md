# Modo ruta — la vuelta como curva en mm que el ESP sigue con su odometría

Estado: desarrollado y validado en el digital twin (2026-10-13), **no probado en el carro real**.
Apagado por defecto (`ROUTE_FOLLOW = False`); en el twin se usa con `--preset ruta`.

## Por qué

Con el control anterior (PP sobre la imagen BEV + gyro-hold/PID de pared/MANIOBRA del ESP) el twin
con `MOTOR_MAX=155` terminaba **1 de 21** carreras. Lo que se encontró:

- La Pi calculaba el ángulo de **rueda** y el ESP lo escribía como grados de **servo** 1:1: el carro giraba
  ~63 % de lo pedido (71° de servo = 44.54° de rueda). El resto del sistema estaba calibrado alrededor
  de ese error (arreglarlo solo empeoraba).
- La línea del mapa digital (`_build_line`) tenía escalones de hasta 180 mm y quiebres de 90°.
- Varias capas mandaban sobre el servo según el estado (gyro-hold, PID de pared, visión ±15°, maniobras).
- La pose por odometría llega a la Pi con 4–6 frames de atraso (ACK de ~250 bytes por UART a 115200).

## Arquitectura

| | Pi | ESP |
|---|---|---|
| Percepción | visión, mapa de señales (`DigitalMap`) | — |
| Ruta | planea la vuelta (`ring_route.py`) y la sube al ESP | guarda la ruta (doble buffer) |
| Localización | EKF (`localizer.py`): odometría + 3 ultrasónicos + 4 ToF contra el mapa conocido | odometría (encoder + BNO085) a la velocidad del loop |
| Control | velocidad permitida (`rv`) | **pure pursuit sobre la ruta** (`rutaSteer()`) |

### Planificador (`pure_pursuit/ring_route.py`)
- La ruta se describe por estación de la vuelta (~75 mm) y `d` = distancia a la isla (100–900 mm).
  En las esquinas los puntos van en arcos concéntricos alrededor de la esquina de la isla.
- Programación dinámica (óptimo de la malla). **Duro:** holgura de la huella completa del carro
  (eje trasero +147.6/−21.8 mm, ±66 mm) ≥ 55 mm a señales, paredes, isla y maderas; radio ≥ 170 mm;
  tramo ≤ 55° contra el pasillo; lado de paso (rojo derecha, verde izquierda).
  **Costo:** largo + curvatura² + castigo por holgura < 140 mm.
- Suavizado con B-spline cúbica cerrada; ~0.4 s por plan en PC (caché de paredes/isla/cajón).

### Vuelta 1 sin mapa (`pure_pursuit/route_mode.py`)
- Asiento **sin resolver** = obstáculo sin lado de paso: la ruta nunca pasa por donde podría haber una señal.
  La DP además pasa pegada a él (por cualquier lado) para que, si al revelarse toca el otro lado, el
  cambio sea de ~300 mm.
- Se **resuelve** si el mapa lo confirma, si las reglas de las cartas lo descartan (X va sola, nunca dos
  en la misma columna, máximo dos por recta, asientos prohibidos del cajón) o si estuvo a ≤ 550 mm de la
  cámara y ±45° (detección medida: ~98 %) por 16 frames sin aparecer nada.
- **Color provisional**: con ≥ 2 votos de un color, la ruta ya toma el lado de ese color antes de la
  confirmación (que exige que la detección caiga a ≤ 160 mm del asiento, o sea ya cerca).
- **Velocidad**: vuelta 1 con tope de PWM 120 mientras quede algo sin resolver; lento (100) si un asiento
  sin resolver está a < 1.3 m; detenido solo si la cámara lo está viendo (máx. 2 s).
- **Continuidad**: cada replaneación deja iguales las 7 estaciones (~500 mm) delante del carro.

### Localizador (`pure_pursuit/localizer.py`)
EKF sobre (x, y, rumbo). Predicción con los incrementos de odometría del ACK; corrección con cada
sensor de distancia contra un ray-casting del mapa completo. Solo se usa la medición si el rayo
esperado pega en pared o isla; si pega en una señal o una madera, se descarta. Solo corrige en
S/C/R (no en el INICIO).

### Arranque ciego: el cajón puede estar en cualquier punto de la recta (`pure_pursuit/start_fix.py`)
Los jueces pueden correr el cajón a lo largo de la recta de salida, pegado a la pared exterior (pasó en
el nacional 2026): w0 (cara de atrás de la primera madera) entre 1000 y 1690 mm. Se conocen la recta, el
rumbo y la distancia a la pared exterior; **no** el avance.

- Se arranca suponiendo el cajón **lo más atrás** posible en el sentido de carrera (CW: 1000, CCW: 1690).
  Mientras no se sepa, el carro cree ir más atrás de lo que va: la ruta dobla tarde, nunca contra la isla.
- Un **histograma** sobre el corrimiento δ (pasos de 5 mm) acumula cada lectura de los sonares
  (frontal y laterales): cada δ se califica según si pared, isla o maderas explican la distancia. El
  sonar frontal alcanza la pared del fondo a ≤ 119 cm (timeout de `leerDistancia`), en la segunda mitad de
  la recta: ahí se colapsa en un pico. Se da por fijo con 97 % de la masa en ±20 mm.
- Una lectura que pudo venir de una **señal posible** (cono ancho de ±21°, calculado con el ancho angular
  de cada asiento) no decide entre esas hipótesis: los asientos están cada 500 mm y la T1 vista en un δ es
  la X1 en otro. Solo descarta los δ donde ni pared ni señal la explican. Solo lecturas con el ESP en
  S/C/R y sin giro rápido (en la maniobra de salida fijaba mal).
- Modelo del sonar: paredes e isla analíticas (rayos cada 1.5°), maderas del cajón con rayos cada 0.5°
  (miden 20 mm: a 3° se escapaban). ~12 ms por frame en la PC, solo antes de fijar.
- Antes de fijar, la cámara **no vota** asientos (`DigitalMap._deferred`): se guarda y se vota al fijar
  con la pose corregida. La recta del cajón se planea con lo que la cámara ve relativo al carro: en su
  fila interior puede quedar **una sola** señal adelante (X1 o la T del final); con color (≥ 2 votos) se
  esquiva donde se vio; sin ver nada, la fila adelante es una cerca sin lado que la ruta no cruza.
- Al fijar: la pose se corre δ, las maderas pasan a su lugar real, se vota lo guardado, se repasan las
  vistas ("visto vacío") y se replanea desde la pose. El ESP sigue recibiendo la transformación del
  **marco viejo** hasta que la ruta nueva llega completa (`RTE`), para no saltar a mitad de un esquive.
- Respaldo: si el frente llega (en el marco supuesto) al final de la recta sin fijar, se toma la media
  del histograma (`FORZADO` en el log).
- Probado y descartado: fijar "grueso" (media en cuanto el 90 % cabe en 160 mm) para no esperar a la pared
  del fondo. Fijaba antes de tiempo con hasta 235 mm de error y rompió 9 de 30 que pasaban: en la recta del
  cajón esquivar con la posición relativa de la cámara es más exacto que con un asiento mal ubicado.

En el twin: `--park-pos random` (o `FOX_PARK_POS=random`) pone el cajón al azar según la semilla
(el sorteo de señales no cambia); `--park-pos 1400` lo pone en ese w0. La Pi nunca recibe ese dato.

## Protocolo serial nuevo (Pi → ESP)

```
RTB,n,paso            empieza una ruta de n puntos separados `paso` mm (cerrada)
RTP,i,x,y,x,y,...     puntos desde el índice i (mm enteros del tapete), 10 por línea
RTE,n                 fin: la ruta queda activa (doble buffer) si llegaron los n
V2,...,rt=1,rtx=..,rty=..,rth=..,rv=..
                      rt=1: seguir la ruta; rtx/rty/rth: odometría -> tapete
                      (mundo = (rtx,rty) + px·F + py·L, F=(sin rth, cos rth),
                      L=(-cos rth, sin rth), rumbo = rth - yaw); rv: PWM máximo
```
El ACK agrega `rn=` (puntos de la ruta activa). Una línea RTP por frame (el UART a 115200 no da para más).

## Configuración (`config.py`)

| Constante | Default | Qué hace |
|---|---|---|
| `ROUTE_FOLLOW` | `False` | activa el modo ruta |
| `ROUTE_ON_ESP` | `True` | el ESP sigue la ruta (si no, el steer lo calcula la Pi) |
| `ROUTE_LOCALIZER` | `True` | EKF propio |
| `ROUTE_LAP1` | `True` | vuelta 1 sin mapa (si no, espera a conocer las 4 rectas) |
| `ROUTE_PLAN_ASYNC` | `False` | **poner `True` en el carro**: planear en otro hilo |
| `ROUTE_LOOKAHEAD_MM` | 200 | lookahead del pure pursuit |
| `ROUTE_LAP1_PWM`, `ROUTE_SLOW_PWM` | 120, 100 | velocidades de la vuelta 1 |
| `ROUTE_VIEW_MAX_MM`, `ROUTE_VIEW_HALF_DEG`, `ROUTE_EMPTY_FRAMES` | 550, 45, 16 | "visto vacío" |

En el modo ruta se usaron `DIGITAL_MAP_WALL_FIX=False` y `DIGITAL_MAP_SIGN_PULL=False` (las correcciones
viejas confundían maderas y señales con pared).

## Cómo correrlo en el twin

```powershell
cd src\RASPI\cam
$env:PYTHONUTF8=1; $env:FOX_CXX="C:\001Escul\WRO2026\WRO2026\.venv-sim\Scripts\python.exe -m ziglang c++"
..\..\..\.venv-sim\Scripts\python -m pure_pursuit.chassis_twin --preset ruta --seed 3170839 --speed 10
```
La línea naranja del panel "mapa real" es la ruta planeada.

## Lista para el carro real

1. Flashear el `.ino` nuevo (modo RUTA, doble buffer, `rn=` en el ACK).
2. `ROUTE_FOLLOW=True`, `ROUTE_PLAN_ASYNC=True`, `DIGITAL_MAP_WALL_FIX=False`, `DIGITAL_MAP_SIGN_PULL=False`.
3. Calibrar y verificar en el banco:
   - relación servo→rueda (`RUTA_SERVO_POR_RUEDA` = 71/44.54 en el `.ino`; `SERVO_FULL_LOCK_CMD_DEG` /
     `MAX_WHEEL_STEER_DEG` en `config.py`) y el centro del servo;
   - `ENC_CUENTAS_POR_MM` (medido 20.91 vs 20.73 en el `.ino`);
   - signo y deriva del yaw del BNO085 (`BNO_YAW_SIGN`);
   - montajes de sensores (`SENSOR_MOUNTS`) y alcances reales de ToF en pared negra.
4. Medir el fps real de la Pi 5 con el pipeline completo y el tiempo de una replaneación.
5. Probar primero con el carro en un tramo recto: subida de la ruta (`rn=` en el ACK), que siga la línea
   recta, luego una vuelta con mapa sembrado, luego la vuelta 1.

## Pendiente
- Estacionamiento al final (`PARK_MODO`), hoy apagado en el modo ruta.
- Robustez con la física perturbada (ganancia/centro del servo, encoder, motor) — ver resultados abajo.
- Velocidad: validado con `MOTOR_MAX=155`.

## Resultados (twin, `MOTOR_MAX=155`, `PARK_NINGUNO`)

| Configuración | Limpias |
|---|---|
| Control anterior | 1/21 |
| Modo ruta, mapa conocido | 69/69 |
| Modo ruta, vuelta 1 sin mapa (21 seeds) | 21/21 |
| Modo ruta, vuelta 1 sin mapa, **cajón al azar en la recta**, código anterior (cajón fijo supuesto) | 7/42 |
| Modo ruta, vuelta 1 sin mapa, **cajón al azar**, arranque ciego | 41/42 (error del cajón: mediana 13 mm, máx. 67; vuelta 1 27.7 s, vueltas 2-3 17.7 s) |

Pendiente del arranque ciego: semilla 32 (CW, cajón en 1034, casi donde se supone). El carro cruza la recta
en diagonal para pasar una roja por dentro, ningún sonar ve paredes hasta la esquina, fija forzado (+61 mm)
y la verde de la esquina (W/T2) seguía sin votar: la ruta nueva pide rodearla por fuera cuando el carro
ya va por dentro.
