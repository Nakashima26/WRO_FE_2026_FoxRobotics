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
- `hw_nuevo` = hardware de competencia: 180×130 mm, batalla 113, rueda 46.32°, cámara 45° a ~97 mm,
  encoder en motor, 4 ToF (L/R/F/B), IMU BNO085, Pi 5 (~28 fps), corrección del mapa con sonares.
  **Servo de dirección 0-180** (dato del usuario, dicho varias veces): el firmware y el twin todavía usan
  los topes 30/160 del carro viejo — ver "Pendiente de medir" y el frente `t16servo`.
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
   - `t16servo`: servo físico 0-180 del carro nuevo. Firmware sigue pensando en unidades lógicas 30..160
     (= tope); `escribirServo` las mapea a físico 0..180 bajo un define del carro nuevo; el twin `hw_nuevo`
     modela el físico 0-180 → ±46.32°. Validar equivalencia (21 seeds + prepark contra baseline) y luego
     sensibilidad al slew. `t16u`/`t16gr` diseñan en unidades lógicas.
1. ~~T16 rendimiento~~ hecho (t16-perf + 0e3f6f1); oráculo descartado.
2. T15c esquinas (rama WIP `t15c-esquinas`) — evaluar si sirve al frente C.
3. T12 escenarios explícitos por cartas (`--scenario`, `--noise-seed`, enumerador) y corregir
   `REQUIRED_CARDS` en wro_field.py:226 (debe ser 26, 27, 32, 33) después de guardar los escenarios de las seeds actuales.
4. T11 mapa congelado tras vuelta 1 + ruta fija vueltas 2-3 (visión solo como sanity check).
5. T14 driver BNO085 en el firmware real; recalibrar `bev_calib.npz` al mover la cámara.

## Pendiente de medir en el carro real
Altura real del lente y grosor de la LiPo; montajes de US/ToF; PPR del encoder y pull-ups (GPIO 34/39);
GPIO15 libre (XSHUT ToF frontal); VL53L0X vs VL53L1X; radio real de la contravuelta de la salida
(`INICIO_R_CONTRA_MM`); fps reales en la Pi 5. **Servo del carro nuevo** — dato del usuario: recorre
**0-180**. Los topes 30/160 del firmware vienen del carro viejo (fc2bc3e, 2026-09-18) y el twin supone que
en ellos la rueda llega a 46.32° (ganancia 60°/70° de comando por lado, también herencia del carro viejo).
Si 0/180 = ±46.32° lineal, comandar 30/160 da ~30.9°/36.0° de rueda (R≈189/155 mm, no 108).
Falta medir: valor de servo con ruedas rectas (el firmware supone `centroServo=90`), ángulo de cada rueda
en 0 y en 180 (Ackermann o paralelas) y **radio a tope por lado** (círculo del eje trasero en 0 y en 180):
el twin usa bicicleta con 46.32° (R≈108); si 46.32° es la rueda interior, R≈163.
Slew del servo y rodado en coast (hoy 'supuesto' en params.py).
