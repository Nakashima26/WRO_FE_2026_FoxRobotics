# Handoff — digital twin Fox (rama `digital-twin`)

Punto de entrada para una sesión nueva. El detalle histórico está en `docs/twin_plan.md`
(léelo por secciones con grep, no completo: es largo).

## Cómo trabajar (preferencia del usuario)
- El hilo principal es el **planner**: arma la lista de tareas, delega la ejecución a subagentes,
  verifica lo que reportan en el código/logs antes de aceptarlo, fusiona y documenta.
- Subagentes en **worktree aislado** cuando tocan los mismos archivos que otro; tope de **10 jobs**
  cada uno (22 hilos de CPU compartidos); presupuesto de iteraciones explícito.
- Cada cambio que se queda: commit + entrada en `docs/twin_plan.md`. No commitear `runs/` ni `_build/`.
- Leer logs con grep/resúmenes, nunca completos (tokens).
- Español; términos técnicos en inglés.

## Entorno (Windows)
- venv: `.venv-sim` en la raíz (numpy, opencv-python con GUI, matplotlib, imageio, pytest, ziglang).
- Desde `src/RASPI/cam/`: `PYTHONUTF8=1 ../../../.venv-sim/Scripts/python -m pure_pursuit.chassis_twin --preset hw_nuevo --seed 3170839 [--no-show --log-dir runs/x] < /dev/null`
- Firmware SIL se compila con zig (~5 min la 1ª vez por cambio del .ino; caché por hash).
- Herramientas: `pure_pursuit/twin/tools/` (all.sh = 21 seeds con drive.py, summ3.py, salida.py = harness salida del cajón, triage.py, parkcheck.py…). Salidas en `src/RASPI/cam/runs/` (ignorado).
- Tests: `../../../.venv-sim/Scripts/python -m pytest pure_pursuit/twin/tests -q` (falla conocida previa: `test_corner_hint::test_v2_carries_hint_when_enabled`).

## Presets
- `hw_nuevo` = hardware de competencia v2 (CAD GLB, 2026-10-09): 169.4×132 mm, batalla 113, giro
  44.54° equivalente (53.1° rueda interior), cámara 45° a ~97 mm (supuesto), encoder en motor, 4 ToF,
  IMU BNO085, Pi 5 a 40 fps (`pi_period_s` 0.025), corrección del mapa con sonares.
  **Servo 19..161 (±71°)** desde 2026-10-09. Los datos viejos de este bullet (180×130, 46.32°, 30/160, 28 fps)
  ya no valen: ver la sección "Estado 2026-10-10" más abajo.
  **Todo el trabajo nuevo se mide aquí.**
- `giro_rapido` = carro actual (PWM sin encoder, MPU6050, 14 fps). Solo referencia.
- Dimensiones/montajes: fuente única en `config.py` (bloque GEOMETRÍA DEL VEHÍCULO).

## Reglas duras (aprendidas, 2026-10-04)
- **Planear antes de delegar**: el planner primero lee este archivo + las entradas de twin_plan.md que
  toque, arma hipótesis y criterio de éxito por tarea, y solo entonces delega. Cada prompt de subagente
  lleva: causa a verificar, archivos/zonas que puede tocar, criterio de validación, presupuesto de iteraciones.
- **Causa raíz antes de editar**: el subagente debe demostrar la causa con pose real (`trace.csv`:
  columnas `t,x,y,heading,est,servo,motor_dir,motor_pwm,speed_mm_s,...`) + `fw_debug.log`, citando líneas.
  Prohibido "subir el umbral hasta que el síntoma desaparezca" sin explicar por qué el valor viejo estaba mal.
- **Auditoría**: todo fix que se vaya a commitear lo revisa otro subagente en contexto fresco (solo lectura)
  antes de aceptarlo. Las conclusiones de un subagente se verifican en la fuente antes de reportarlas.
- **Ruido del twin** (medido 2026-10-05, rama `t15ruido`): la carrera SÍ es bit-determinista dentro de un
  mismo host — seed 2 en la Mac ×3 serie, ×3 en paralelo con carga (10 jobs) y ×2 sin fijar hilos dan el
  mismo SHA256 de `trace.csv` sin `pi_frame_ms`; seed 3170839 ×3 igual. Mac (arm64) vs Windows (x86_64)
  NO: divergen en t=21.99 s (fila 621, ACK `ang=3.12` vs `3.13`) por ULPs de punto flotante acumulados
  en `anguloGyro += gz * dt` (PurePursuit.ino:2593). `millis()` del SIL es tiempo virtual; los
  `perf_counter` de runtime_nuevo.py solo alimentan HUD/logs. La "alternancia" vieja de seed 2 casi seguro
  mezclaba hosts o una caché `_build/` vieja. Regla: 1 corrida por escenario basta; comparar SIEMPRE
  en el mismo host (baseline y fix); nunca Mac contra Windows.
- **Métricas normalizadas**: contar eventos (REVERSA, choques) por esquina recorrida, no totales — una
  carrera que choca antes "tiene menos" de todo.
- `prepark.py` NO ejercita la U ni la recta previa: 98/100 ahí no garantiza nada en carrera completa.
  Validación final = prepark 100 + `all.sh` 21 seeds.

## Mac (corridas pesadas — Windows está al tope de RAM)
- `ssh -o BatchMode=yes jesse@192.168.68.59` (arm64, 18 núcleos). Autorizado por el usuario para lotes.
  Darle el permiso al subagente **en el prompt inicial** (si no, lo rechaza como sospechoso).
- Cada agente su dir `~/Projects/fox_<tag>`, sincronizado por tar desde `git ls-files -co --exclude-standard`
  sin `runs/` ni `_build/`. **Borrar `_build/` tras cada sync que cambie el .ino** (caché de `libfw_*.dylib`
  puede servir un binario viejo).
- **CRLF**: el checkout de Windows tiene finales CRLF; un tar hecho desde Windows rompe los `.sh` en la
  Mac (`xargs -P 10\r`). Tras cada sync: `find . -name '*.sh' -o -name '*.py' | xargs sed -i '' $'s/\r$//'`
  (o empaquetar con `git archive`/`git ls-files` + `dos2unix`). El SHA del .ino cambia con eso: comparar
  SHAs siempre del lado Mac.
- Python `~/Documents/GitHub/FoxRobotics/.venv/bin/python`, `PYTHONPATH=.`, desde `src/RASPI/cam`.
  Hasta 10 jobs por agente; coordinar si hay varios.
- NO tocar `~/Documents/GitHub/FoxRobotics` (repo del usuario; solo su `.venv`) ni dirs de otros agentes.
- No sondear con `sleep`: ssh en primer plano con timeout largo, o Bash `run_in_background` y esperar aviso.
- `SOLO_CAJON=1 SEEDS="..." pure_pursuit/twin/tools/all.sh <tag> [jobs]` (en la Mac cambiar la ruta del
  python del script o llamar drive.py directo); `summ3.py runs/<tag> <seeds>` resume.

## Estado 2026-10-10 (rama `fox/seed-3170839-red-object`, HEAD b1833b0, NO pusheada)
Plan escrito con lista de tareas: `docs/PLAN_v2.md`. Lo de abajo es lo VERIFICADO; lo marcado "supuesto" o
"sin medir" no lo está. Las cifras del twin son de la Mac (arm64), nunca comparar con Windows.

**Geometría v2** (fuente: `VehicleDirTest_V2.glb` del usuario + README de `origin/main` 8117629)
- `config.py`: largo 169.4 (−21.8…147.6 desde el eje trasero), ancho 132 (con llantas; el 128 del README
  no las incluye), batalla 113 (CAD 112.85), vía 110, voladizos 21.8/34.6, montajes de sensores en la CARA que
  mide (ToF izq/der ±35.2 fwd 110.6, trasero −19.6, frontal 141.4; US lat ±63.5 fwd 56.3, frontal 147.6).
  El GLB solo trae el ToF izquierdo; el derecho es espejo (supuesto).
- Dirección: rueda interior 53.1° / exterior 37.0° a ±71° de servo (README §2.4, piñón 19T, CAD). El twin es
  bicicleta: ángulo equivalente 44.54° (`MAX_WHEEL_STEER_DEG`, derivado: `atan(L/(L/tan53.1+T/2))`, T=60 pivotes),
  ganancia simétrica `44.54·90/71`. El GLB está en pose recta: NO confirma los 53°/±71° (vienen del README).
- Servo: `SERVO_MIN_DEG=19`, `SERVO_MAX_DEG=161` en el .ino y `servo_min/max_deg` en `twin/params.py`
  (antes 30/160 del carro viejo, que dejaban el giro a la derecha en ~37.6° y rompían la salida del cajón).
  U de 180: `ticks = delta/43.91*70` (43.91 = giro a 70 ticks).
- **Hallazgo abierto**: la Pi manda `steer_deg` como ángulo de rueda y el ESP lo aplica 1:1 como grados de
  servo (`ppServoGain=1.0`), pero 1° servo ≈ 0.627° rueda → el pure pursuit trabaja a ~63 % de la ganancia
  que cree. Probar `ppServoGain=1.6` en el twin EMPEORA (1/41 terminan, jitter 15.5°/frame): no tocar sin medir
  servo→rueda en el carro (`twin/tools/calib/`).
- Cámara: `CAMERA_FWD_MM=140`, `CAMERA_HEIGHT_MM=97` son SUPUESTOS no medidos (el nodo `CameraFrnt` del GLB da
  134/85 pero es el origen del nodo, no el centro óptico). El usuario dice: no asumir dónde va la cámara;
  `bev_calib.npz` es viejo y no se ha recalibrado. No usar la homografía para sacar la pose.

**Pi 5 / cámara** (traído de `origin/main`)
- `vision.open_camera`: tuning `camera_tuning/imx219_noir_wro_pi5.json` + ganancias `<1.10,1.51>`, `format=BGRx`,
  `framerate=40` (máx. del modo 1640×1232 del IMX219; `FOX_CAM_FPS`). `config.PI_FPS = 40` (`FOX_PI_FPS`).
  `cam_web.py` y `camera_tuning/` traídos. Que el pipeline completo llegue a 40 fps NO está medido.
- Pendiente: `calibration.py` sigue con `framerate=30/1` sin BGRx (falla en Pi 5: "not-negotiated").
- `twin/tests/conftest.py` fija `PI_FPS=FPS_NOMINAL` (14) en los tests.

**INICIO / rosa**: `_park_pink` mide la fracción de píxeles en HSV 135–175 (no "si se ve rosa"). Umbral real 0.28
(calibrado en el carro). Con los voladizos del CAD la cola queda a 5 mm de la madera (`wro_field.stall_start_xy`)
y el rosa simulado cae de 0.29 a 0.25. `hw_nuevo` usa `PARK_PINK_RATIO_MIN=0.20` (compensación SOLO del twin,
aceptada en `paridad.py`); `config.py` sigue en 0.28 para calibrar en el carro con `pick_color.py`.

**Firmware v2** (`PurePursuit.ino`; NO compilado para ESP32: no hay toolchain; verificado solo el SIL)
- Pinout de los sketches probados del usuario: ToF XSHUT F=0, L=15, R=5, B=4 (GPIO 0/15 son strapping); BNO085
  UART-RVC en GPIO25 (Serial1 RX). Antes el firmware tenía L=25/R=4/B=5/F=15 (chocaba con el BNO en 25).
- `FOX_ENCODER`, `FOX_TOF`, `FOX_BNO` por defecto 1 fuera del SIL (en SIL siguen 0; el twin pasa sus defines).
- BNO: `leerBNO()` acumula el delta de yaw de CADA paquete (100 Hz; el loop tarda >10 ms). `BNO_YAW_SIGN=1.0`:
  VERIFICAR girando el carro a la derecha a mano. Antes el firmware solo leía el MPU6050.
- ToF: la librería de Pololu fija 50 ms de presupuesto en `init()`; `startContinuous(33)` no lo cambiaba → ~20 Hz.
  Ahora L/R/trasero en Short 20 ms (50 Hz, ~1.3 m) y frontal en Long 33 ms (30 Hz, 4 m). Twin: `ToFParams`
  por sensor. El twin da lectura inválida en pared negra/lejana con el umbral de señal actual (calibrado a 250 mm).
- `counts_per_mm=20.73` es del N20: con el Pololu 12 CPR×50×2 serían ~8.9 (HIPÓTESIS, medir rodando 1 m).
- `escribirServo` usa 90 fijo y no `centroServo`; `PARK_UTURN_EXT_MIN_CM` no se usa en ningún sitio.

**U de 180° (esquina 13)**: `vigilarUturn` arma con pared frontal <100 cm + lateral >100 cm (2 lecturas) o
frontal ≤26 cm. Radio = `(hueco − EXT_OBJ·10)/2`. Barrido en el twin (37 seeds que disparan la U, Mac):
- `PARK_UTURN_ESPERA_MM` (espera recta antes de la U): NO ayuda; 200–300 mm chocan antes de la U. Default 0.
- `PARK_UTURN_EXT_OBJ_CM` 40→**28** (ahora PARK_AJ ajustable): margen a la lata del cuadrante inicial 52/107→149/190
  mm (CW/CCW), pared exterior a ~180 mm. 46 de 50 choques tras la U son contra la lata más cercana (42 de la
  sección del cajón). Segundo modo de fallo SIN resolver: si la U arranca a ≤35 cm de la pared exterior el radio ya
  es el mínimo (~115 mm, 70 ticks) y EXT_OBJ no actúa: hay que arrancar la U antes/más lejos.
- CW 24/27 sin choque, CCW 7/10 (n pequeño; de 120 seeds solo 37 llegan a la U: 27 CW, 10 CCW; falta la
  división por sentido de las 120). `blocks_turn` sin espejo CCW (twin_plan ~:235) es la pista.
- `prepark.py` no ejercita la U: solo carreras completas hasta tc=12.

**Planificador de ruta** (`pure_pursuit/route_planner.py`, tests `twin/tests/test_route_planner.py`, 10 pasan):
QP periódico de toda la vuelta (SQP + punto interior en numpy; sin scipy), lado de pilar por color, huella de
3 discos, paredes/isla. API `plan_lap(direction, pillars, params, ...)`. 30 tableros aleatorios: 6441 mm/vuelta,
~20 s/vuelta (309–332 mm/s), 59–64 s 3 vueltas, lado correcto 30/30. Apagado (`ROUTE_PLANNER=False`), enganche
`DigitalMap._planned_line`. NO probado conduciendo. Pendiente: correrlo en el twin (P0/P1/P2), lookahead constante,
replan en hilo (bloquea ~0.2 s PC, 3–5× en Pi), quitar residuo `RP_DEBUG`, `OPENBLAS_NUM_THREADS=1` obligatorio.
La comparación contra `_build_line` es aproximada (sobreestima sus fallos). Los presets del twin ya activan
`DIGITAL_MAP_STEER=True`: la línea base de jitter ya seguía la línea del mapa, no el BEV puro.

**Calibración del carro real** (`twin/tools/calib/`, `src/ESP32/CalibFox/CalibFox.ino`; no usado todavía):
orden en `HOJA_MEDICION.md` (~2 h 10 min): cuentas/mm (1 m con regla) → velocidad vs PWM → zona muerta → tau_drive →
coast → scrub → servo→rueda y radio mínimo → duración de `loop()` → ToF Short vs 50 ms. Dos parches sin aplicar
(`patches/`): instrumentar `loop()` y pings HC-SR04 asíncronos (el `pulseIn` bloquea hasta 7 ms×3).
Velocidad: solo hay ~245 mm/s a PWM 95 (twin y firmware); a PWM 120 → 309 (regla de 3) o 332 mm/s (supuesto).

**Resultados del twin sobre la rama integrada** (Mac, `hw_nuevo --solo-cajon`, sin overrides): 11/41 terminan
(F0 8/41, pre-b3f2712 4/21: diferencias = ruido), 0 excepciones, salida del cajón 41/41. Jitter en carreras >10 s:
Δsteer std 8.8°/frame, 1.07 cambios de signo/s (antes 12.5 y 1.7; atribuido al periodo de 25 ms, NO separado de los
ToF a 50 Hz). Steer máx 71° = tope del servo. `WHEELBASE_PX` 50 vs 56.5, lookahead 100 constante y
`ppServoGain` 1.6: sin mejora (peor los dos últimos).

**Descartado tras análisis (subagentes; datos arriba)**: STM32 (el cuello es software: `pulseIn`, doble
`mpu.update()`, no el MCU), Gazebo (el twin ya cubre sensores/latencia; faltan choque físico y fotometría de cámara),
Nav2/SLAM (la pose ya se corrige con sonares; robar: inflado con huella real y lattice con R mínimo), LQR (pure
pursuit bien afinado iguala a LQR/Stanley en el modelo de bicicleta).

**Reglas nuevas del usuario (2026-10-09/10)**: el código lo escriben subagentes Haiku con especificación precisa y
el planner verifica; Mac autorizada por SSH (solo dirs `~/Projects/fox_v2_*`); no preguntar entre pasos, dejar plan
escrito y lista de tareas; no asumir la posición de la cámara; nunca decir "implementado" sin comprobarlo (varios
puntos de arriba se habían dado por hechos y no lo estaban: servo 30/160, pines ToF, BNO, defaults FOX_*).

## Estado (2026-10-05, rama `t15b2` HEAD e2914de, NO fusionada a `digital-twin`, NO pusheada)
Reglamento 9.23 (decisión del usuario): el cambio de sentido para estacionar se hace **dentro de la
esquina 13**; luego solo se anda por esa esquina y la recta de salida. Criterio de parking: tocar la pared
exterior está bien, tocar el cajón no.

- Estacionamiento: `PARK_MODO=PARK_PARALELO` + U de 180° (`PARK_PARALELO_USA_UTURN=true`) disparada por
  `vigilarUturn(distL,distR,distF)` con la señal de esquina (`contadorEsquina13`, commit a6e29b3). 10/11 en la
  esquina 13; **s9 dispara en la esquina 12** (`85.257 MANIOBRA completada 12/12` → `86.456 PARK U servo=20 ext=9`;
  piso `PARK_UTURN_ARRANQUE_MM=300`). Luego fase 0 (escaneo) → fases 1-13.
  - `prepark.py --solo-cajon`: **98/100** (fallan `CW_c2_p4`, `CW_c3_p4`: poste 2 en fase 8, holgura de la S ≈12 mm).
  - Carrera (11 seeds que llegan a tc=12, tag `t15b2_esquina13`): estacionan 6, 8, 18, 3170839; cajón en
    2, 7, 14, 19, 20; señal en 5, 9.
  - Patrón 2 (7, 19 CCW): tras la U queda pegado a la pared (ext 9-11) y no se separa a 28 cm antes del poste 1.
  - Patrón 3 (2, 14, 20 CW): entra a fase 9 aún girando ~80°/s, 25° fuera (`dif=25.7 ext=9 ATRAS PEGADO`).
  - **Causa común de 2 y 3 (verificado 2026-10-05)**: la U de un solo arco (fase 24) satura cuando arranca con
    ext 30-34 cm (`PARK U servo=160/20`) y termina a ext ≈ ext0 − 22 cm (7-11 cm): 5 de 6 chocan el cajón.
    Con ext0 67-71 (s8, s18, s3170839) el servo queda proporcional (144-150), termina a 38-40 cm y estaciona 3/3.
    Además: en CCW la fase 24 escribe servo 20 (bajo el tope 30; ticks ×70 a ambos lados).
  - **Rumbo del ESP en la U** (verificado 2026-10-05, ACK crudo vs `trace.csv`, 11 seeds base): error RMS 4.06°
    (s14 −7.26 CW, s2 −6.37, s9 +7.65, s7 +4.43; resto ≤2.1). Dos partes: (1) contabilidad relativa,
    RMS 3.27° — el tope ±10 de `MANIOBRA_INCL_ENTRADA_MAX_DEG` (ino:531, aplicado en 1791 y 4147) pensado
    contra picos del MPU tira inclinación REAL de entrada (s14 `6.510 ... ErrGyro:13.38 ... obj=-80.0`,
    Ang −15.9 → ref −3.36 hasta la U; 10 GR saturadas en 21 logs) y en REVERSA el cero (4186) llega
    300 ms después de la captura; residual/GIRO PI/U(−180) son exactos y lo conservan. (2) modelo BNO
    (escala σ 0.1% + caminata, "supuesto"): RMS 1.82°, máx +3.90 (s14, escala sorteada 2.9σ). El firmware
    no lee yaw absoluto (sin driver BNO, T14): integra `gz·dt` (ino:2593). Nada re-referencia (MANIOBRA 13
    no corre; la Pi no manda rumbo). Arreglo: referencia por recta `yaw0 + s·90·k` desde `anguloTotal`
    (quita la parte 1), luego corrección contra pared con ToF. La cifra vieja "186.5° vs Ang:-0.14"
    emparejaba instantes distintos; twin_plan:208 "BNO ≤ 1°" es falso (7/11 seeds > 1°).
  - prepark arranca con la pose de tc=12 sin la U (h=370/460): mide una entrada que la carrera no produce.
- `PARK_PARALELO_REV` (estacionar en reversa, fases 30-38): 0/100 limpios con el mejor valor
  (`PARK_REV_B_MAX_MM=30`); el costado roza el poste a Ang≈150-165° del swing C con servo fijo. Ver
  twin_plan "barrido PARK_REV_*".
- **Parking: diagnóstico geométrico T16** (2026-10-05, diagnóstico + verificación adversarial de cada modo).
  Supuesto R=109 con servo a tope; R=163 si 46.32° es la rueda interior. Ningún modo llega hoy a 100/100
  de forma robusta:
  - **PARALELO** (98/100 en prepark, fallan CW_c2_p4/c3_p4 por −1.5/−0.8 mm contra el poste 2).
    - El contragiro de fase 8 pasa con un margen de ~0±2 mm (ρ−r_FL +1.9/+1.8/−1.3/−1.3 frente a minP2 1.7/0.2/−0.8/−1.5).
    - Cota analítica de la S simple: 7.1 mm con R=109 y −6.8 con R=163.
    - S2 de 4 cúspides: 19.7 mm en ideal. Con dispersión el peor caso sale −12.7 mm (R=109) y −23.9 (R=163),
      y termina fuera o torcido. **No viable.**
    - El lazo de 33-72 ms existe pero su causa NO está identificada: no son los timeouts de pulseIn, refutado.
    - Tiempo medido: CW 6.22 s, CCW 10.03 s (fase 0→TERMINADO); P2→fin 4.36 s.
  - **PARALELO_REV** (0/100 hoy).
    - Causa raíz: el barrido arranca ~26 mm antes de la ventana de x0 (~10 mm).
    - Diseño de 5 tramos con R=109: 4.3 mm en cinemática. Con la dinámica del twin las paradas se pasan
      +2.8..+4.6°, y el corte del ToF trasero llega −16±7.5 mm tarde (sample-and-hold a 30 Hz + EMA).
      Calibrado sobre los mismos 100 casos quedan 0.4-1.8 mm.
    - Sensibilidad: R=115 da 11/100 contactos, R=120 da 64/100, R=163 inviable.
    - Con el contrato de entrada de t16u, CW tiene 53/90 contactos (la fase 0 no converge: Y32 varía 117 mm).
    - No hay evidencia medida de que sea más rápido que PARALELO: P2→fin 4.5-4.9 s en emulación.
    - `distB_filtrada` conserva una lectura vieja si el ToF no es válido (ino:3201).
  - Lo que bloquea a los dos modos: (1) R por lado sin medir (±3 mm de R cambia el desenlace); (2) la dispersión
    de entrada (fase 0 y U); (3) la sensibilidad a <0.4° de rueda cerca del centro (ver t16servo abajo).
  - Scratch de las verificaciones (ignorado por git): `src/RASPI/cam/runs/t16pfv/`, `runs/t16revv/`;
    Mac `~/Projects/fox_t16revv`.
- **t16servo (dd2831f) auditado: FUSIONAR_CON_NOTAS**, pero solo junto con el commit de dinámica SG90 (su slew
  900/771°/s es imposible para un SG90).
  - El shim es correcto: ctl_q reproduce la base 100/100 en prepark y 21/21 en carrera.
  - **La caída del prepark 98→87 NO es caos.** b_db pierde 11 escenarios y no gana ninguno; 8 son p2. La
    descomposición CTLQ_NEW la ubica en la cuantización cerca del centro (84..96: caen los 6 CW p2), no
    en el tope 160 (0 fallos).
  - Esos p2 pasaban gracias al sesgo de +0.03..0.08° a la derecha que deja la truncación vieja.
  - **Artefacto del twin** (vehicle.py:80-88, anterior a la rama): paso de slew 0.6 u/ms > banda 0.5 u, así que
    el reposo depende del sub-paso. Con L=89 la rueda asienta en 0.851° (dt 1 ms), 0.695° (0.5 ms) y 0.579° (0.25 ms).
  - `_en_banda_muerta` (vehicle.py:134) congela el servo si la banda de un lado vale 0.
  - plano600 (slew del SG90) da prepark 79/100, roce_pared 15/100.
  - FOX_SERVO_180/FOX_ENCODER/FOX_TOF solo valen 1 en el SIL: hay que activarlos al compilar el ESP del
    carro nuevo.
  - escribirServo usa 90 fijo y no centroServo (ino:1621).
- Carrera completa 21 seeds (baseline Mac, `~/Projects/fox_man/.../runs/baseline`): terminan 4/21; **12/21
  chocan con una señal** (7 en SIGUIENDO con esquive activo pero sin margen, 2 en GIRO_RAPIDO, 2 en
  ESTACIONANDO, 1 SIG-GYRO; siempre la PRIMERA señal). "Ninguna por lado equivocado" NO está medido:
  `metrics.py:219` solo cuenta un cruce si `along` salta >50 mm entre filas (~12 mm/fila) — casi nunca.
- Giros con REVERSA ("corre raro"): 23 en el baseline, todos con `distExt` 36-81 cm (el firmware reversea
  cuando está LEJOS de la exterior; adelante solo si `<= HUG_CM=20`). Causa verificada en s20: entra a
  CRUCERO torcido (`13.473 F:52 Ang:-20.25`), un `prio:1` lo regresa a SIGUIENDO y vuelve a CRUCERO con
  `F:38` < `grMin=max(20,95-55)=40` → sin giro rápido → `14.342 REVERSA distExt=61 distF=25`. La hipótesis
  "piGr=0 → objetivo 50 cm" es falsa en el twin (`GIRO_RAPIDO_MODO=1` en el preset).
- Perf: commit 0e3f6f1 (×1.14, bit-idéntico). Ideas no hechas: vectorizar `collision()` (~8%), cachear
  paredes en `camera.py`, C/numba (requiere instalar; pedir permiso).

### Trabajo sin commitear (auditado; por defecto DESCARTAR)
- `t15b2`: diff en `PurePursuit.ino` (~98 líneas): fase 26 "separarse de la pared" para el patrón 2.
  Auditoría: diagnóstico correcto, fix malo — el carro no gira (`f=26 ext=11 avance=1 girado=0.1`), dispara
  en seeds que no debía (`muyPegado` sin filtro de sentido) y `t15b2_fixfull` empeora (seed 6 pasa a cajón).
  Conservar solo los comentarios de diagnóstico de patrón 2/3 si sirven.
- `t15man` (worktree aparte): `HUG_CM` 20→90, `HUG_HIST_CM` 6→10. Tapa el síntoma (anula REVERSA), no
  explica por qué llega a 36-81 cm; la baja de REVERSA es en parte artefacto (seed 5 choca una esquina
  antes); nunca terminó la comparación serial. Corridas en `~/Projects/fox_man/src/RASPI/cam/runs/`.
- Stash `fox-opt-t15b2-1791161724` en la pila compartida: copia redundante de 0e3f6f1, se puede borrar.

Hecho y fusionado antes: fix del verde en esquina (CW), mapa digital (pose eje trasero, proyección,
rumbo de arranque real, corrección con sonares), encoder+4 ToF, BNO085, Pi 5 + constantes en frames
independientes de fps, arranque al ras del borde interior del cajón, salida del cajón sin reversa.

## Ramas viejas (sesiones anteriores; revisar antes de borrar)
Dos worktrees en `.claude/worktrees/` (ver `git worktree list`):
- `agent-a6501da593f9e5146` (rama `worktree-agent-a6501da593f9e5146`, base cf8f97d — anterior a la salida nueva):
  **T15b estacionamiento en PARALELO**. Fases de tiempo → distancia de encoder + barrido de parámetros
  (mejor combo vista: POSTE2_M100 / ENDEREZA10 / FINAL_CM4 / ANG_IN_D50). Harness `twin/tools/prepark.py`
  arranca con tc=12 y **se salta la vuelta en U**: en la carrera (seed 6) la media vuelta choca el cajón.
  Pendiente: harness desde tc=11; la centerline de la Pi no está adaptada a la aproximación al cajón.
- `agent-a47a3f8910d0a9fc8` (rama `t15c-esquinas`, base 1250703): **T15c esquinas**. Generalizar
  "lata en la boca de la recta siguiente" a ambos sentidos/colores (falta el espejo CCW: seed 11,
  rojo S/T3); caso rojo por DENTRO que retrasa el giro en la vuelta 1 (seed 3170839, esquina S→W);
  línea del mapa sin validar contra la lata viva (runtime_nuevo.py ~1847 `path_points = mapped`);
  hitbox del carro en el mapa (hoy es un punto); seeds 1,3,4,15 chocan rojo T2 de la recta del cajón.
Ambos agentes se detuvieron; todo lo que tenían quedó commiteado en sus ramas (nada sin commitear):
- `worktree-agent-a6501da593f9e5146`: `ae1e3ed` (harness pre_park tc=12 + paralelo ruta A por encoder,
  completo) y `203d36c` WIP (estaba metiendo: empate de media vuelta, hook SIL tc=11, modo "curva" del
  harness — incompleto, sin validar).
- `t15c-esquinas`: `9cbc6ad` WIP (digital_map/config: lata en la boca para ambos sentidos, `huella.py`
  hitbox, harness `twin/tools/esquina.py` + `esq_cmp.py` — incompleto, sin validar; estaba por lanzar
  la validación). Sin entrada T15c en twin_plan.md todavía.
Para cada uno: ver `git -C <worktree> log` y `diff --stat`; si hay commits útiles, fusionar a
`digital-twin` (conflictos esperables en PurePursuit.ino/runtime_nuevo.py/twin_plan.md), correr tests,
validar con `tools/all.sh` y `tools/salida.py all`, y borrar el worktree.

(`ae1e3ed` ya está en t15b2 vía `t15b-merge`; `203d36c` WIP y `t15c-esquinas` 9cbc6ad WIP no.
`t16-perf` ya está en t15b2; `t16-oracle` descartado.)

## Siguiente (en orden)
0. Sesión 2026-10-05 (ola T16, reemplaza A/B/C): B (REV) cerrado sin solución robusta (fc0d6db).
   - `t16u`: U geométrica en la esquina 13 (arco a tope → recta perpendicular cerrada con sonar F →
     arco), armado de la U por odometría, servo 30/160, re-referencia de rumbo con la pared, harness `pre_U`.
   - `t16gr`: ventana del giro rápido calculada (REVERSA) + holgura geométrica del arco contra la primera
     lata (en lugar de `TURN_HOLD_INNER_CM=80`) + métrica de lado de paso.
   - `t16infra`: sentido CW/CCW de una sola fuente (ACK `dir=`, hoy `DigitalMap` toma `'CW'` de config y el
     twin le inyecta la verdad) + reporte de paridad twin↔carro (`GIRO_RAPIDO_MODO`, `FOX_*`, `TURNS_PER_RACE`…).
   - `t16servo`: servo de dirección del carro nuevo = SG90 de 0-180. El firmware sigue pensando en unidades
     internas 30..160 (= tope); `escribirServo` las mapea a valor de servo 0..180 bajo un define del carro
     nuevo; el twin `hw_nuevo` modela 0-180 → ±46.32°. Validar equivalencia (21 seeds + prepark contra
     baseline) y luego dinámica del SG90 (600°/s, deadband 10 µs). `t16u`/`t16gr` diseñan en unidades internas.
1. ~~T16 rendimiento~~ hecho (t16-perf + 0e3f6f1); oráculo descartado.
2. T15c esquinas (rama WIP `t15c-esquinas`) — evaluar si sirve al frente C.
3. T12 escenarios explícitos por cartas (`--scenario`, `--noise-seed`, enumerador) y corregir
   `REQUIRED_CARDS` en wro_field.py:226 (debe ser 26, 27, 32, 33) después de guardar los escenarios de las seeds actuales.
4. T11 mapa congelado tras vuelta 1 + ruta fija vueltas 2-3 (visión solo como sanity check).
5. T14 driver BNO085 en el firmware real; recalibrar `bev_calib.npz` al mover la cámara.

## Pendiente de medir en el carro nuevo
Altura real del lente y grosor de la LiPo; montajes de US/ToF; PPR del encoder y pull-ups (GPIO 34/39);
GPIO15 libre (XSHUT ToF frontal); VL53L0X vs VL53L1X; radio real de la contravuelta de la salida
(`INICIO_R_CONTRA_MM`); fps reales en la Pi 5. **Servo de dirección** — datos del usuario (2026-10-05):
es el **SG90** del repo, recorre **0-180** y el modelo de SolidWorks da **46.32° de rueda a tope**.
README §2.4 (piñón 14T, 2.5:1, ±43°, trim 80, clamp 20-150) está desactualizado: no usarlo.
Los topes 30/160 del firmware vienen del carro viejo (fc2bc3e, 2026-09-18), igual que la ganancia
60°/70° de comando por lado del twin. Si 0/180 = ±46.32° lineal, comandar 30/160 sin el shim de
`t16servo` da ~30.9°/36.0° de rueda (R≈189/155 mm, no 108).
Sin medir: valor de servo con ruedas rectas (se supone `centroServo=90`), linealidad, ángulo de cada
rueda en 0 y en 180 (Ackermann o paralelas) y **radio a tope por lado** (círculo del eje trasero en 0 y
en 180): el twin usa bicicleta con 46.32° (R≈108); si 46.32° es la rueda interior, R≈163 (caso pesimista).
Dinámica del SG90 por datasheet: 0.1 s/60° @4.8 V → 600°/s de servo, deadband 10 µs ≈ 0.9°.
El firmware manda 500-2500 µs y el nominal del SG90 es 1-2 ms: riesgo de tope mecánico en los extremos.
Slew del servo y rodado en coast (hoy 'supuesto' en params.py).
