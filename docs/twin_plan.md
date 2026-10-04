# Plan digital twin / mapeo (rama `digital-twin`)

Bitácora entre sesiones. Actualizar al cerrar cada paso.

## Meta
El twin completa la ronda de obstáculos (3 vueltas) **y se estaciona** en el cajón,
sin chocar paredes ni latas, en varias semillas.

## Entorno (Windows)
- venv: `.venv-sim` en la raíz (`numpy opencv-python` [NO headless: chassis_twin abre ventana con cv2.namedWindow] matplotlib imageio pytest ziglang`).
- Compilador SIL: `twin/firmware/build.py` usa `FOX_CXX` > `c++` > `python -m ziglang c++`; en Windows genera `.dll`.
- Correr desde `src/RASPI/cam/` con `PYTHONUTF8=1` (si no, `print` de `π`/`°` revienta en cp1252) y stdin cerrado (`< /dev/null`):
  - una semilla: `PYTHONUTF8=1 ../../../.venv-sim/Scripts/python -m pure_pursuit.chassis_twin --preset giro_rapido --seed 3170839 --no-show --log-dir runs/s3170839 < /dev/null`
  - lote: `PYTHONUTF8=1 ../../../.venv-sim/Scripts/python -m pure_pursuit.twin.batch --preset giro_rapido --seeds 1-6 --jobs 4 --out runs/batch.csv < /dev/null`
  - con trace + debug del ESP: `PYTHONUTF8=1 PYTHONPATH=. ../../../.venv-sim/Scripts/python runs/drive.py 3170839 runs/s3170839` (script scratch, sin trackear: escribe `pi.log`, `fw_debug.log` con tiempo, `trace.csv`, `field.txt` y una línea `[DMDBG]` por frame con el estado de `DigitalMap.blocks_turn`).
- Arreglos Windows hechos: `build.py` corre el compilador con `stdin=DEVNULL` (zig se quedaba colgado con CPU 0 heredando el stdin); `fw.py` cierra el `NamedTemporaryFile` antes de `ctypes.CDLL` (WinError 32) y hace `FreeLibrary` antes de borrar la copia. La 1ª compilación con zig tarda ~5 min (arma mingw + libc++); luego queda en caché.

## Lista de tareas
- [x] T1 Correr el twin en Windows, capturar logs (`pi.log`, métricas), describir qué se ve.
- [x] T2 (diagnóstico; fix pendiente) Bug: esquina de abajo ve el verde pero no espera para girar (`CORNER_EXTERIOR_PASS_*`, `obstacle_memory`, `runtime_nuevo`).
- [x] T3 Auditoría de la arquitectura de mapeo: mapa original (`wro_field`/`digital_map`), mapa conocido (`track_map`/`obstacle_memory`), fusión BEV + ultrasónicos + ToF, cómo se genera la línea (`centerline`/`digital_map` steer).
- [x] T4 Huella completa del carro (largo, ancho, voladizo, posición de la cámara) en el mapa conocido / inflado de obstáculos.
- [x] T5 Diseño independiente (sin ver el código): cómo lo haría desde cero.
- [x] T6 Investigación: práctica profesional (occupancy grid, map-then-race tipo F1TENTH, equipos WRO FE).
- [x] T7 Comparar T3 vs T5/T6 → decidir cambios (vuelta 1 = mapear; vueltas 2–3 = seguir ruta óptima del mapa; corrección BEV ¿apagar/atenuar después de la vuelta 1?).
- [ ] T8 Implementar e iterar en el twin hasta que se estacione. Alcance acordado: Fase A ítems 1, 2, 3 y 5 (sin footprint ni Fase B: llega hardware nuevo).
  - [x] A1 giro tardío con verde en la boca: el PP dobla y el ESP lo cuenta (`GIRO_PI_CUENTA_DEG`)
  - [x] A2 proyección BEV→campo (eje trasero + pie→centro) y pose genérica (no solo cajón W/CW)
  - [x] A3 `DigitalMap` nuevo en GO, sin votar desarmado
  - [ ] A5 estacionamiento (3, 4, 5)

## Hallazgos

### T1 — qué produce una corrida
- `chassis_twin --seed N --no-show --log-dir D` imprime `[TWIN] stop=… metrics={…}` y escribe `D/pi.log` (stdout de `PPRuntime`). El debug serie del ESP (`Serial.print`) solo queda en memoria (`fw_debug`, últimos 8000 chars) y el trace solo en `RunResult.trace`: para verlos hay que usar `runs/drive.py`.
- `pi.log`, por frame (sin timestamp; se alinea por orden con el trace): `[DET]` (latas cámara/BEV/`far`), `[LINEUP]`, `[LINEA] Orange={near_y,…}`, `[DIR] … ext_corner_hold turn_block turn_delay`, `[MEMDBG]` (memoria de obstáculos), `[RECUP]`, `[PPDIAG] steer obs nobs`, `[MAP]` (track_map shadow), y `TX: V2,…,prio,mem,pasado,…,es,gv | RX: ACK:V2,ang,est,dL,dR,dF,…,tc,px,py,yaw`. Eventos: `[DMAP] sec/asiento color`, `[MEM] Giro detectado/terminado`, `[LOCK]`.
- `fw_debug.log`: una línea por `loop()` con `Estado`, `Servo`, `L/R/F`, `Ang`, `prio`, `mem`, `giros`, y transiciones `-> CRUCERO`, `-> GIRO_RAPIDO dir=… obj=…`, `TERMINADO`.
- Hueco de observabilidad: `[DIR] turn_block=` es solo `_ext_corner_block`; el bloqueo que viene de `digital.blocks_turn()/needs_line()` (runtime_nuevo.py:2004-2005) no se ve en ningún log, solo como `prio=1,mem=1` con `nobs=0`.
- Semilla 3170839 (CW, cajón W): `stop=collision`, `turns_completed=3`, choque `señal Green S/T4` a t=26.65 s. Determinista (dos corridas idénticas).

### T2 — esquina SE (abajo-derecha, giro 3): ve el verde y gira igual
Escenario: carro bajando por E (rumbo 180°) pegado a la exterior (x≈1200), gira a la derecha hacia S. `Green S/T4` en (500,-1100): primera columna de S, fila exterior; verde = pasar por la izquierda = por el sur, entre la lata y la pared. Hay que girar tarde (salir a y≈-1250); el GIRO_RAPIDO sale a y≈-844, al norte de la lata.

Cadena causal (frames consecutivos de `runs/s3170839/pi.log` con `[DMDBG]`):
1. Visión: el verde solo aparece como `far` / pie fuera de la hoja: `[DET] camR=0 camG=1 bev=[] far=[(544, 'G')]`, `[LINEUP] Green@(578,-29)`. Nunca entra al BEV ni a la memoria (`[MEMDBG] closest=-` todo el tramo), así que el rescate `CORNER_EXTERIOR_PASS` (runtime_nuevo.py:1603-1618, opera sobre `memory.classify_and_split`) no corre: `[DIR] … ext_corner_hold=0 turn_block=0` en todos los frames. `CORNER_EXT_PASS_BAND_PX=160` y `TURN_BLOCK_FRAMES=0` son irrelevantes en este caso.
2. Mapa digital: `[DMAP] S/T2 Green` — lo asienta una fila adentro (T2 = y -900; real T4 = y -1100). Pose del mapa al soltar (1321,-423) vs real (1205,-546): ~120 mm de deriva. No cambia la decisión, pero el mapa está corrido.
3. El único bloqueo activo era `DigitalMap.blocks_turn()` → `_turn_block` → `prio=1,mem=1` (runtime_nuevo.py:2004, :424-425):
   `[DMDBG] sec=E tc=2 along=1897 lat=-326 … green_ahead=(752, False) outside_release=False blocks_turn=True` → `TX: …prio=1,mem=1… dF=85`
   frame siguiente:
   `[DMDBG] sec=E tc=2 along=1923 lat=-321 … green_ahead=(718, False) outside_release=True blocks_turn=False` → `TX: …prio=0,mem=0… dF=83`
   El verde seguía a 718 mm (umbral 260); lo suelta la salida temprana de digital_map.py:540-541:
   `if self.along_mm > 1900.0 and self.lat_mm < -80.0: return False` ("ya en la esquina y del lado de afuera… el verde no está en el arco"). Falso para un verde en la primera columna de la recta siguiente: el arco del GIRO_RAPIDO desde la exterior sale por el carril interior.
4. `gv=0` siempre: `_green_ahead()` (runtime_nuevo.py:859) solo mira `new_obstacles` del BEV a <360 mm; el verde nunca estuvo ahí → no se aplica `GR_VERDE_ABRE` y el giro va a tope (servo 30).
5. Firmware (`fw_debug.log`): `23.887 … Estado:SIGUIENDO … F:81 … prio:0 | mem:0` → tras `POST_DODGE_CRUCERO_GRACE_MS=400` (PurePursuit.ino:556, condición :3310) `24.240 … -> CRUCERO … F:69` → `24.311 … -> GIRO_RAPIDO dir=DER obj=-80.0` (F:67 dentro de la ventana 40–95, `!_hayLataMia`, PurePursuit.ino:3638-3660). El ESP no gira por su cuenta: obedece en cuanto la Pi suelta `prio`.
6. Sale rumbo 261° en (997,-844), al norte (derecha) del verde; la línea del mapa pide ir al sur del verde → cruza enfrente → `señal Green S/T4` en (641,-990), rumbo 223°.

Experimento (solo monkeypatch en `runs/drive.py`, `NO_OUTSIDE_RELEASE=1`, sin tocar el código): sin esa salida temprana, `prio=1` se mantiene, el ESP nunca entra a CRUCERO y la línea del mapa (PP en SIGUIENDO) dobla sola por el sur del verde (pasa x=500 en y≈-1284). Pero el ESP no cuenta ese giro (`tc` se queda en 2) y la corrida termina en `pared exterior` a t=40.7 s. Quitar la línea no basta: falta que el giro "esperado" lo ejecute/cuente el ESP (p. ej., soltar `prio` cuando el arco sí deja la lata a la izquierda, mandar `gf`/umbral frontal más bajo, o `gv=1` desde el mapa).

Lote `giro_rapido` (`runs/batch_gr.csv`):

| seed | stop | tc | vueltas | choque |
|---|---|---|---|---|
| 1 | collision | 1 | 0 | señal Green W/T4 (1.2 s tras el giro) |
| 2 | collision | 2 | 0 | señal Green W/X2 |
| 3 | collision | 12 | 3 | cajón magenta (estacionando) |
| 4 | race_finished | 12 | 3 | — (TERMINADO en est=E; pose final (-365,1281) rumbo 341°, de punta, con la cola fuera del cajón (estimado)) |
| 5 | collision | 0 | 0 | cajón magenta a t=0.6 s (CCW, sale del cajón) |
| 6 | collision | 1 | 0 | señal Green S/X1 (1.5 s tras el giro) |
| 3170839 | collision | 3 | 0 | señal Green S/T4 |

4 de 7 chocan un verde 1–3 s después de un giro a la derecha (todas CW); 1, 6 y 3170839 encajan con el patrón de T2, pero solo 3170839 está verificada frame a frame. 2 chocan el cajón; solo la 4 termina las 12 esquinas y la maniobra (aunque el estacionamiento no queda limpio).

### T6 investigación (resumen)
- Equipos WRO FE públicos (Neural-Navigators 2025, Team Rulo Bot 2025, QSTSS 2024): reactivos por sección/visión + PD; ninguno documenta mapear vuelta 1 y reproducir 2–3.
- F1TENTH "map-then-race": vuelta de mapeo, luego localización-only (particle filter) sobre mapa fijo + raceline; replanear solo si cambia el costmap. Inflado por footprint (Nav2 costmap).
- Transferible: localización por sección + resets con paredes/líneas, odometría encoder+gyro con corrección por landmarks, mapa discreto de slots en vez de grid.

### T5 diseño desde cero (resumen)
- Mapa = prior paramétrico (rectas/esquinas fijas) + slots discretos {vacío, rojo, verde}; no occupancy grid.
- Localización desacoplada: lateral por US/ToF contra pared conocida, heading por gyro, longitudinal por encoder con reset en cada línea de esquina (deriva acotada a un tramo).
- Vuelta 1 lenta con voto ≥3 frames por slot; al cerrarla se congela el mapa y se calcula la ruta una vez.
- Vueltas 2–3: visión NO decide lado; solo sanity-check (si contradice con alta confianza, re-plan local de ese slot) + corrección fina cross-track.
- Ruta = cadena de corredores por slot; el "esperar para girar" con verde en esquina sale solo como restricción de corredor (el apex se recorre), no regla ad-hoc.
- Footprint: C-space (Minkowski) en rectas + swept-polygon con bicicleta Ackermann (incluye voladizo trasero) en esquinas; curvatura ≤ 1/R_min.
- Parking: maniobra ensayada de 3 fases con cierre por ToF/US lateral.

### T3/T4 auditoría (verificado en código)
- Tres mapas, ninguno manda en el carro real: `DIGITAL_MAP_STEER=False` (config.py:764), `TRACK_MAP_ENABLED=False` (config.py:1216). Solo el preset twin `giro_rapido` prende el steer.
- `digital_map` pose: rotación hardcodeada a cajón W/CW (digital_map.py:184-185), ancla `_stall_xy` fija.
- Proyección BEV→campo sin `BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM` (100 mm) ni pie→centro de lata (digital_map.py:233-234); track_map sí lo suma.
- Firmware: `FOX_ENCODER 0`, `FOX_TOF 0`; avance por modelo PWM; `lroundf` por tick pierde lateral (PurePursuit.ino:44,47,1657-1660).
- Sin fusión US/ToF ni resets por pared/naranja en digital_map; drift sin cota.
- `DigitalMap` no se resetea en GO (runtime_nuevo.py:1356); vota desarmado.
- La línea del mapa reemplaza la BEV sin validarse contra la lata viva (runtime_nuevo.py:1847-1850).
- Footprint: todo es punto. Inflado 72 mm < 65 (medio ancho) + 35 (medio diag lata) + margen. Pure pursuit origen eje delantero pero fórmula de eje trasero; MAX_STEER 60° vs 46° reales.
- Dimensiones contradictorias: params.py 180×130 / README 210×140; tilt 45° vs 15°. Hay que medir.
- No existe modo vueltas 2–3 ni ruta precomputada.

### T7 decisiones (plan de cambios, en orden)
Fase A — que el twin termine y se estacione (arreglos puntuales):
1. `blocks_turn()` (digital_map.py:540): quitar/condicionar la salida temprana "afuera en la esquina"; soltar solo cuando el arco del giro deja el verde a la izquierda (verde en 1a columna de la recta siguiente ⇒ girar tarde). Hacer que el ESP haga Y CUENTE el giro tardío (gv=1 desde el mapa / gf más bajo / soltar prio en el punto correcto). Loggear el bloqueo del mapa en `[DIR]`.
2. Proyección BEV→campo en digital_map: +`BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM` y pie→centro de lata (como track_map). Debe corregir T2 vs T4.
3. `DigitalMap` reset en GO; no votar desarmado; votos ponderados por distancia (no confiar >~1 m).
4. Footprint: inflado/carril de paso ≥ medio ancho + medio diag lata + margen; chequeo de rectángulo barrido (eje trasero + 180×130 + voladizo) contra latas/paredes para la línea del mapa.
5. Parking: seeds 3 (choca cajón), 4 (queda de punta), 5 (choca al salir del cajón CCW).
Fase B — arquitectura "mapear vuelta 1, ruta fija 2–3":
6. Al cerrar la vuelta 1 congelar asientos confirmados y precomputar la ruta completa (corredores por asiento, swept-polygon); vueltas 2–3: visión solo sanity-check (re-plan local si contradice con alta confianza) + corrección lateral. NO apagar visión del todo.
7. Localización: resets por pared (US) y naranja; pose genérica (no hardcode W/CW).
Hardware confirmado por el usuario (2026-10-03): cambio de dimensiones/estructura en camino; ToF a izquierda, derecha, frente y atrás; encoder en el motor; IMU BNO085 (fusión en chip, rumbo consistente) en lugar del MPU6050; Raspberry Pi 5 en lugar de Pi 4 (twin hw_nuevo ~28 fps; cámara topada a 30 fps en vision.py:20; ojo constantes en frames).
- T14 pendiente: driver BNO085 en el firmware real (SH-2 por I2C).
Dimensiones confirmadas (aprox): 180x130 mm, giro max de rueda 46.32 deg (coincide con twin/params.py; README 210x140 obsoleto).
Batalla 113 mm (confirmado). Voladizos 33.5 = (180-113)/2 derivado. Cámara tilt 45° (confirmado), misma posición pero más alta: LiPo 3S Ovonic 2200 (~24 mm acostada) al centro en lugar de la Pi 4 (~17 mm con puertos) => +7 mm estimado, height_mm 90 -> 97 (supuesto).
Pendiente con el usuario: medir con regla grosor de la LiPo y altura real del lente; montajes ToF. (180×130 vs 210×140; tilt 45° vs 15°); ¿hay encoder en el robot final? (FOX_ENCODER 0).

### Reparto de trabajo (2026-10-03)
- Agente T8a (en curso): solo Fase A 1-3 + parking. No toca params/sensores/FOX_ENCODER/FOX_TOF.
- T9 HECHO (rama `worktree-agent-af77b20e11052fcc8`, commit df84b28, pendiente merge tras T8a; descartar los .dSYM borrados del working tree del worktree): encoder + 4 ToF + fuente unica de dimensiones + preset `hw_nuevo`.
- Después, en serie (tocan los mismos archivos):
  - T9 Twin con hardware nuevo: ToF L/R/F/B + encoder en params/sensors/firmware SIL; dimensiones nuevas cuando estén medidas (una sola fuente de verdad para dims).
  - T10 Footprint (Fase A 4) sobre las dims nuevas.
  - T11 Fase B: localización con encoder + ToF (resets pared/naranja), mapa congelado tras vuelta 1, ruta fija vueltas 2-3.

### Revisión seeds / randomizer (verificado contra app.py oficial, 2026-10-03)
- `randomize(seed)` (wro_field.py:317) sortea todo con un `random.Random(seed)` y reintenta hasta cumplir reglas: cualquier cambio en el bucle cambia qué pista es cada seed. sim.py:243-244 usa el mismo entero para el ruido de sensores (campo y ruido acoplados).
- BUG: `REQUIRED_CARDS = (22, 23, 28, 29)` (wro_field.py:226). La app oficial usa `required_obstacles_sets = [21, 22, 27, 28]` como ÍNDICES de su lista deduplicada de 32 → cartas 26, 27, 32, 33 (mixtas T1/T2 y T3/T4). El port los tomó como número de carta en la lista de 36 → mixtas T1/T4 y T1/T2. Corregir a (26, 27, 32, 33) — pero cambia el significado de cada seed.
- Duplicados: wro_field sigue las 36 cartas físicas (14≡16, 15≡17, 20≡22, 21≡23, 26≡28, 27≡29, 32≡34, 33≡35); la app quitó 16/17/22/23. Diferencia de probabilidad menor; las 36 físicas son defendibles.
- Archivos raros: batch.py mete listas de dicts en celdas CSV; drive.py (script scratch) mezcla texto + JSON en .out.
- T12 (después de T8a/T9, toca sim/batch/chassis_twin): escenario explícito `CW|single=E|park=W|N=9,E=10,S=22,W=28` con format/parse en wro_field, `--scenario` en chassis_twin/batch, `--noise-seed` separado, imprimir el escenario en cada log, enumerador de escenarios válidos, salida summary.json + CSV plano. ANTES de corregir REQUIRED_CARDS: guardar el escenario de seed 3170839 y de las seeds 1-6 para no perder los casos de prueba.

### T8 — bitácora de implementación (en curso)
Herramientas scratch (sin trackear, en `src/RASPI/cam/runs/`): `go.sh <tag> <seeds…>` corre `drive.py` en paralelo y resume eventos del ESP; `align.py <dir> t0 t1 [cada] [feet]` alinea pose del mapa (`[DMDBG]`) con la real (trace) por frame y, con `feet`, proyecta cada pie con la pose real (separa error de proyección vs de pose). `EXTRA='{"def":{"FOX_ENCODER":"1"}}'` corre con encoder (solo experimento).

Cambios (todos detrás de `DIGITAL_MAP_STEER` o de constantes off por defecto, salvo los bugs marcados):
- `digital_map.blocks_turn`: fuera la salida temprana "afuera en la esquina". Si sigue bloqueado con dF ≤ `DIGITAL_MAP_PP_TURN_CM` (75) se engancha "esquina por PP" hasta que suba tc. Esquina del verde redibujada: vértice = cruce del lado actual del carro con el lado de paso del verde, redondeo `_KNEE_MM`=260 (antes la curva salía del centro del carril y, con el carro adentro, terminaba detrás de él → seguía derecho a la pared).
- Firmware: `GIRO_PI_CUENTA_DEG` (0=off; preset 70): en SIGUIENDO con prio, si el gyro ya rotó eso hacia el lado de la pista → `completarTurnoObstaculos()`. Y con giro rápido, si la ventana 40–95 ya pasó (dF<grMin) cae a MANIOBRA (antes seguía de frente a la pared; 3170839 a3).
- `holds_for_center` (`DIGITAL_MAP_TURN_HOLD_CM`=65 / `_INNER_CM`=80 en el preset): el giro rápido a 95 cm pivotea con ~8 cm de avance y sale a ~10 cm de la isla (semillas 1 y 6, `pared`/`isla`).
- Bugs (sin flag): pose del mapa = eje trasero (antes centro del chasis, 56 mm); proyección +`BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM` +25 mm pie→centro; `line_bev` resta el origen BEV; pose rotada con el rumbo de arranque real (antes fija W/CW: en las semillas 1–6 el mapa giraba al revés); naranja *dead-reckoned* ignorada para decidir "ya pasó la cinta" (3170839: E/X2 nunca se asentaba, todas las latas votaban en la recta siguiente); `needs_line` también para rojo; `hold_pasado` si todavía no llega al lado de paso; `_pull_pose` hasta 45° (antes 25°: tras el INICIO el carro cruza a ~38° y nunca corregía 50 mm de deriva).
- `build.py`: compilación paralela en Windows (WinError 5 en `os.replace`).

Hallazgo principal (bloqueo fuera de alcance): la odometría del ESP (modelo PWM, `FOX_ENCODER 0`) deriva ~50–100 mm por giro (scrub del twin). Ej. semilla 2: `17.26 err=(34,-48)` → tras GIRO_RAPIDO `19.81 err=(133,-103)` → tras la esquina por PP `24.85 err=(145,-211)`; con eso un verde T1 se asienta en T3 y la línea pasa por encima de la lata. Con `FOX_ENCODER=1` (experimento) la semilla 2 completa 12 giros y llega a estacionar.

### T9 resultado (verificado diff)
- ToF frontal (XSHUT GPIO15, 4 sensores 0x30-0x33), `tF=` en ACK solo con FOX_TOF=1 (con 0 el ACK queda igual byte a byte). Odometría float con FOX_ENCODER; 0.0482 mm/cuenta (7 PPR supuesto ×4×50×2 / π·43) — calibrar en pista.
- Fuente única de dimensiones: config.py (`GEOMETRÍA DEL VEHÍCULO`, `SENSOR_MOUNTS`, provenance). twin/params, track_map, map_view, wro_field leen de ahí.
- Bug twin arreglado: ToF compartía reloj/valor retenido entre sensores (sensors.py:98).
- Pose por encoder: error medio 55 mm / máx 111 en 3 vueltas vs PWM 178 / 342.
- ToF válidos solo 15-29 % de frames (pared negra corta a 250 mm).
- OJO: subir cámara 90→97 mm cambia mucho los resultados (seed 2: tc 2→10, seed 6: 1→4) → el twin es caótico; evaluar con más seeds, no 6.
- Real: recalibrar `bev_calib.npz` tras mover la cámara (~7 % de escala). Medir: PPR encoder, pull-ups GPIO34/39, GPIO15 libre, VL53L0X vs VL53L1X (firmware usa L1X), montajes.
- Pendiente: MAX_STEER_DEG=60 vs 46.32; BEV_ORIGIN 100 vs batalla 113.

### Estado tras merge (commit 0698d87, 2026-10-03)
- Mergeado T8a (commit 1c4a478, WIP detenido por el usuario) + T9. Tests: 66 ok, 1 falla preexistente (test_corner_hint gv=0).
- Perfilado (detenido): dejó cambios en chassis_twin.py (visor); el usuario reporta que ya corría bien.
- Worktree `C:/Users/jbanda/Documents/GitHub/wt_base` = 60e0356 (base sin cambios) para comparar.

### Lecciones (para cualquier agente nuevo)
1. El twin es caótico: un cambio de 7 mm en la cámara cambia seeds enteras. Evaluar SIEMPRE en ≥20 seeds y contra la base con la misma config; nunca declarar mejora con 6.
2. Evitar "whack-a-mole" de heurísticas por seed: T8a iteró 12 veces y mejoraba unas seeds rompiendo otras. Buscar causa raíz común (pose/odometría, proyección, footprint) antes de tunear umbrales.
3. Con odometría PWM la deriva (50-100 mm por giro) era el bloqueo. Con encoder (hw_nuevo) el error medio baja a 55 mm. Trabajar sobre `hw_nuevo`.
4. ToF contra pared negra solo válidos 15-29 % de frames; US + naranja pesan más para localizar.
5. Correr siempre `--no-show` y `< /dev/null` (no abrir ventanas al usuario); PYTHONUTF8=1.
6. No editar archivos de otro agente en paralelo; un agente por área.

### T13 — hw_nuevo: 3 vueltas + estacionar (agente 2026-10-03)
Herramientas scratch nuevas (sin trackear): `runs/all.sh <tag>` corre `drive.py` con `hw_nuevo` en seeds 1-20 + 3170839 (logs completos por seed en `runs/<tag>/s<N>`) y `runs/summ3.py` resume (stop, tc, choque, `fuera_mm` = cuánto sale la huella final de la caja del cajón; limpio = terminado sin choque y fuera ≤ 5 mm).

Fase 0 (HEAD 5bce711, `runs/base`): 0/21 limpios, suma tc 85. Las 7 CCW (5, 7, 9, 11, 12, 17, 19) chocan `cajón magenta` a t=0.6 s: `[INICIO] rosa avg=0.22 umbral=0.28 n=15 -> arranque normal` (CW: `rosa avg=0.29 … MANIOBRA DE SALIDA`). Sin INICIO el ESP sale en SIGUIENDO directo contra la madera.

Cambio 1 (bug del twin, sin flag) — `twin/camera.py` `render_camera`: el pintor ordenaba cada pared entera (quad de 3 m) por su profundidad media; la pared exterior quedaba "delante" de la madera del cajón pegada a ella y la tapaba (solo en CCW, la madera queda del lado donde la media de la pared cae más cerca). Ahora la pared va en tramos de 100 mm con cortes también en los cantos de las maderas. Rosa al arranque: CW 0.294→0.293, CCW 0.218→0.293 (simétrico). Render ~12→18 ms/frame. Resultado (`runs/c1`): suma tc 85→114, 0/21 limpios; las CCW ya hacen INICIO (7 y 19 llegan a 12 giros).

Cambio 2 (corrección del escenario, pedida por el usuario; sin flag) — arranque en el cajón: el costado del carro va al ras del borde INTERIOR del cajón (puntas de las maderas, 2 mm adentro: `h_center = PARK_BARRIER_INTO_MM - 2 - ROBOT_WIDTH_MM/2` = 133 mm), no a 12 mm de la pared exterior. Cola a 5 mm de la madera (igual que antes). Fuente única `wro_field.stall_start_xy()` (lee dims de config.py); `digital_map._stall_xy()` la usa, así el ancla del mapa no puede divergir del spawn. **Cambia el ancla del mapa en el robot real** (es como el equipo coloca el carro). Resultado (`runs/c2`, = baseline nuevo): suma tc 114→107, 2/21 terminan (2 y 3170839) pero quedan 41–43 mm fuera de la caja; las CCW 5, 11, 12 chocan el verde T2 justo a la salida del cajón a t≈6.8 s.

Baseline Fase 0 definitivo (`runs/c2`, HEAD + cambios 1-2):

| seed | dir/park | stop | tc | choque | categoría |
|---|---|---|---|---|---|
| 1 | CW/S | collision | 11 | señal Red S/T2 | lata (pose) |
| 2 | CW/E | race_finished | 12 | - | estaciona, 43 mm fuera |
| 3 | CW/E | collision | 4 | isla | pose (err 198) |
| 4 | CW/N | collision | 5 | isla | pose (err 274) |
| 5 | CCW/N | collision | 0 | señal Green N/T2 | salida CCW, verde en la boca del cajón |
| 6 | CW/E | collision | 2 | pared | pose (err 166) |
| 7 | CCW/N | collision | 8 | señal Red N/T2 | pose (err 256) |
| 8 | CW/S | collision | 3 | cajón magenta | cajón en carrera |
| 9 | CCW/S | collision | 4 | cajón magenta | cajón en carrera / pose (268) |
| 10 | CW/S | collision | 1 | señal Green W/T3 | lata |
| 11 | CCW/W | collision | 0 | señal Green W/T2 | salida CCW |
| 12 | CCW/N | collision | 0 | señal Green N/T2 | salida CCW |
| 13 | CW/W | collision | 4 | señal Red S/T1 | pose (err 353) |
| 14 | CW/E | collision | 9 | pared | pose (158) |
| 15 | CW/E | collision | 8 | señal Red E/T2 | pose (182) |
| 16 | CW/S | collision | 3 | cajón magenta | cajón en carrera |
| 17 | CCW/W | collision | 2 | pared exterior | salida CCW / pared |
| 18 | CW/S | collision | 8 | señal Red S/T1 | pose (200) |
| 19 | CCW/W | collision | 4 | cajón magenta | pose (258) |
| 20 | CW/W | collision | 7 | señal Green S/T2 | pose (388) |
| 3170839 | CW/W | race_finished | 12 | - | estaciona, 41 mm fuera |

0/21 limpios, suma tc 107. Categorías: pose del mapa a la deriva (err 150–390 mm al chocar) 10; salida CCW con verde T2 en la boca 3 (+17); cajón tocado en carrera 3; estacionamiento no limpio 2; otra lata 1. La deriva no es del encoder sino del rumbo: el yaw del ACK (gyro con error de escala ~1 %) acumula hasta ±12° en 3 vueltas (`runs/hderr.py`: seed 3 `0:+3 1:+5 2:+7 3:+9 4:+11 5:+12`).

Cambio 3 (flag `DIGITAL_MAP_WALL_FIX`, default False, prendido en `hw_nuevo`) — `digital_map._wall_fix` / `_yaw_from_wall`: la pose del mapa ahora se integra por incrementos del odómetro (con `_yaw_fix`=0 es idéntica a la de antes) y se corrige con los sonares:
- lateral: sonar del lado exterior contra la pared exterior (toda la recta), el interior solo frente a la isla; descarta si una lata conocida/votada queda entre sonar y pared; compuerta 150 mm. Si L+R suman el ancho del carril (±40 mm) es pared a pared y se acepta aunque pase la compuerta; si la pared exterior repite la misma innovación 6 frames (dispersión < 50 mm) también (la deriva ya pasó la compuerta: seed 13 en E, `dL=15` con `l=-145` mapa vs −300 real).
- longitudinal: frontal contra la pared del fondo, sin lata ni punta de isla en el cono (±17°).
- rumbo: ajuste lineal de (lateral a estima − lateral por pared) contra el recorrido en ventanas de 400 mm → sesgo del gyro → `_yaw_fix` (ganancia 0.6, tope 3°/ventana).
Resultado (`runs/c4`): suma tc 107→151; 9 seeds llegan a 12 giros (antes 2); error de rumbo del mapa típico ≤ ±3° (antes hasta ±12°) y de posición < 100 mm. Limpios 0/21: ahora la categoría grande es el estacionamiento (2, 6, 7, 19, 20 tocan `cajón magenta`, 16/18 una lata y 9 la isla, todos con tc 11–12).

Cambio 4 (twin, solo `hw_nuevo`) — IMU BNO085: `twin/sensors.py` `Bno085Heading` + `twin/params.py` `BnoParams` (supuesto datasheet BNO08x Game Rotation Vector, medir): escala ±0.1 %, deriva en caminata 0.065°/√s (~0.5°/min), ruido Gauss-Markov 0.15° (τ 0.5 s), refresco 200 Hz. Como el .ino integra una velocidad (`gz·dt`, dt en ms enteros, zona muerta 1 °/s), el modelo devuelve la velocidad que hace que esa integración siga el yaw fusionado. `giro_rapido` sigue con el MPU6050. El firmware real necesita el driver SH-2 (T14). Error de rumbo del ACK en 3 vueltas: MPU ±12° → BNO ≤ 1°.
Re-medido con BNO085 (21 seeds): baseline sin `DIGITAL_MAP_WALL_FIX` (`runs/b0`) suma tc 139, 10 seeds a 12 giros, 1 termina (3170839); con wall fix completo (`runs/b1`) 159, 12 a 12 giros, 1 termina; wall fix sin rumbo (`DIGITAL_MAP_WALL_YAW=False`, `runs/b2`) 149, 11 a 12 giros. El wall fix sigue ayudando (la deriva de posición del encoder sigue); se queda completo. Con BNO la categoría dominante pasa a ser el estacionamiento: 9 seeds tocan `cajón magenta` o la lata S/T1 con tc=12.

Cambio 5 (Pi 5) — `hw_nuevo` corre a `pi_period_s=0.035` (~28 fps; supuesto Pi 5 ≈ 2× Pi 4, medir fps reales; la cámara está topada a 30 fps en vision.py:20) y `PI_FPS=28.6`. Nuevo en config.py: `FPS_NOMINAL=14`, `PI_FPS=14` y `fr()` / `per_frame()` / `ema_per_frame()`; con `PI_FPS == FPS_NOMINAL` devuelven el valor tal cual (verificado: seeds 3170839, 3, 13 idénticas a `runs/b1` a 14 fps). Escalados (ventanas de tiempo en frames → `C.fr`): `PASADO_HOLD_FRAMES` (runtime_nuevo.py:1528, :1824, :1967), `RECUP_MEAS_ARM_FRAMES` (:955), `RECUP_MEAS_GHOST_CLEAR_FRAMES` (:1031), `RECUP_MEAS_CLEAR_FRAMES(_CORNER)` (:1065), `RECUP_MEAS_GENTLE_FRAMES` (:1085), `RECUP_CORNER_TURN_DELAY_FRAMES` (:1844), `CORNER_EXT_PASS_TURN_BLOCK_FRAMES` (:1994), `TURN_EST_G_CONFIRM_FRAMES` (:2075), `TURN_RECOVERY_FRAMES` (:2095), literales del cajón 4/3/25 (:1323, :1326); `LINE_TRACK_PERSIST/HOLD_FRAMES`, `ORANGE_POST_TURN_CD_FRAMES`, `ORANGE_DR_MAX_FRAMES` (corner_lines.py `OrangeLineTracker`, ahora leídos en `__init__`, no como default del def); `MIDTURN_WINDOW/CONFIRM_FRAMES` (mid_turn.py:69-70); `LINE_PROVISIONAL_MAX_AGE`, `LINE_CLASSIFY_FRAMES_*` (obstacle_memory.py:938, :974-978); `COLOR_CORR_EVERY_N` (color_corr.py:42); el 6 de `_wall_rej` (digital_map). Pasos por frame → `C.per_frame`: `PP_STEER_SLEW_DEG` (controller.py:166), `OBS_MEM_DECAY` (obstacle_memory.py:271), `OBS_MEM_TURN_DEADZONE/FLOOR_DEG` y el 0.3°/frame literal (obstacle_memory.py:793-794, :823). EMA por frame → `C.ema_per_frame`: `LINE_TRACK_NEAR_Y_EMA`, `LINE_TRACK_LINE_EMA` y el 0.7 literal (corner_lines.py). `FAR_HINT_KD` × PI_FPS/FPS_NOMINAL (derivada sin /dt, far_hint.py). NO escalados (juicio): votos del mapa digital (`>=3`, `+2`, digital_map `_vote`) y `PARK_PINK_SAMPLES` (cuentas de evidencia/promedio, más frames = más evidencia); `CAM_CHECK_FRAMES`, `CAM_BLACK_FRAMES`, `WARMUP_FRAMES` (cuentan frames de la cámara a 30 fps, no del pipeline); `_VOTES` está muerta.
Resultado 28 fps (21 seeds, BNO085 + wall fix): con escalado (`runs/p5`) suma tc 94, sin escalado (`runs/p5n`, PI_FPS=14) 107, a 14 fps (`runs/b1`) 159. El escalado no es la causa: en ambos 28 fps, 8 seeds CW (1, 2, 3, 4, 14, 15, 3170839, +13) chocan el rojo T2 de la recta del cajón a t≈7.07 s, justo al salir del INICIO. La reversa "colchón" del INICIO deja el carro detrás de la fila T2 (3170839: reversa de y=-235 a y=-671; rojo W/T2 en y=-500) y pasarlo por la derecha ya no cabe; a 14 fps la Pi reaccionaba más tarde y el carro lo dejaba pasar por fuera sin tocarlo.

### Estado tras T13 (commit 25de149, agente detenido por el usuario)
hw_nuevo + BNO085 a 14 fps: suma tc 159/252, 12/21 seeds completan 12 giros, 0/21 limpios. A 28 fps (Pi 5): 94 — 8 seeds CW chocan el rojo T2 de la recta del cajón a t≈7 s: la reversa "colchón" del INICIO (INICIO_REV_MS=2600, PurePursuit.ino:711) lleva el carro de y=-235 a y=-671, detrás de la fila T2 (y=-500). La maniobra se diseñó para el arranque viejo (pegado a la pared exterior). Lo que queda son las dos MANIOBRAS, no el mapa: salida del cajón y estacionamiento (9 seeds tocan cajón o S/T1 con tc=12).

### Plan T15 (2026-10-03) — atacar las maniobras con escenarios cortos
Problema de método hasta ahora: cada prueba era una carrera completa de 90 s × 21 seeds (~10 min por lote) para medir algo que pasa en los primeros 10 s (salida) o los últimos 15 s (estacionamiento). Se cambia a harnesses cortos por maniobra, y la carrera completa solo como validación final.

Dos agentes en paralelo, cada uno en su worktree (tocan regiones distintas del .ino: INICIO vs ESTACIONANDO*):
- T15a SALIDA DEL CAJÓN: harness = correr solo hasta pasar la primera lata / primer giro (~12 s) en todas las combinaciones de la recta del cajón (CW/CCW × cartas permitidas en la recta del cajón: 1-6 y 25-30, solo fila interior) a 28 fps con hw_nuevo. Rediseñar INICIO para el arranque al ras del borde interior (¿hace falta la reversa?), que quede del lado de paso correcto de la primera lata. Meta: 100 % sin contacto en el harness.
- T15b ESTACIONAMIENTO: harness = arrancar el carro justo antes de la recta final con tc=12 (start mode nuevo o PARK_TEST_RECTA_COMPLETA), CW/CCW × cartas de la recta del cajón. Verificar contra el reglamento WRO FE 2026 qué cuenta (paralelo, totalmente dentro) y elegir PARK_MODO (hoy PARK_PUNTA). Meta: 100 % sin contacto y huella ≤ 5 mm fuera.
- Después (yo): merge, validación completa 21 seeds a 28 fps, y T15c para las fallas restantes de carrera (latas en recta), y luego T11 ruta fija vueltas 2-3.
Herramientas: pure_pursuit/twin/tools/ (all.sh, drive.py, summ3.py, parkcheck.py…). Salidas en src/RASPI/cam/runs/ (ignorado).

### T15a — salida del cajón (agente 2026-10-03, rama `worktree-agent-a346d5e1c9b862ea2`)
Harness: `pure_pursuit/twin/tools/salida.py all <tag> [jobs]` (desde `src/RASPI/cam`; `summ <dir>` re-resume, `one <esc> <dir>` una corrida). hw_nuevo truncado a 14 s (`SALIDA_MAX_T`), 41 escenarios: CW/CCW × las 10 cartas distintas de la recta del cajón (1-6, 25, 26, 27, 30; 28≡26, 29≡27) con campo armado a mano (`build_field`, cajón N, resto fijo) + las 21 semillas. Mide cada 5 ms la holgura de la huella completa a señales (de la recta / otras), maderas e isla, y el lado de cada cruce de las señales de la recta del cajón (`(rev)` = cruce en reversa, `!` = lado malo). Falla = choque hasta 1 s después de tc=1, o cruce hacia adelante por el lado malo. `EXTRA='{"src":"head"}'` corre el .ino de HEAD (base); `{"period":0.07,"pi":{"PI_FPS":14.0}}` = 14 fps.

Base (HEAD cf8f97d): 24/41 a 28 fps y 24/41 a 14 fps. CW con rojo T2 (cartas 6, 26, 30 y 7 semillas): la reversa colchón cruza T2 por fuera (`R T2(rev):izq!`) y luego lo choca o lo pasa por el lado malo (CW_6: la reversa termina en c(h=372, w=887), detrás de la fila; `señal Red N/T2@7.075`). CCW con verde T2 (5, 25, 27, s5, s11, s12): tras la reversa (c(h=360, w=1428)) el PP no alcanza a cruzar 360 mm al carril interior en 310 mm (`servo=102…124`) → `señal Green N/T2@7.85`. Además la fase 1 se pasaba ~27° (`delay(100)` en cada loop: pico 68.9° con objetivo 42°).

Rediseño (PurePursuit.ino `case INICIO` + `salida_cajon.py`):
- Sin reversa (`INICIO_REVERSA=false`; el código viejo queda detrás del flag). La fase 2 termina cuando lateral (odometría `poseYf`) + `INICIO_R_CONTRA_MM`·(1−cos ang) llega a `INICIO_LAT_AFUERA_MM`=200 (centro a ~33 cm de la pared); la contravuelta cierra hasta `INICIO_CONTRA_FIN_DEG`=6 y pasa al settle. El `delay(100)` del servo solo al INIT.
- La Pi (`SalidaCajon`, solo con est=I) proyecta los pies de lata al marco de arranque del odómetro y vota (3) la primera lata de la fila interior entre 12 y 70 cm adelante (X1 en CW ~44 cm, T2 en CCW ~27 cm; T2 en CW queda atrás, T1 a ~94 cm la cruza el PP). Si se pasa por el lado de la isla (rojo a la derecha / verde a la izquierda; no necesita el sentido) manda `isal=2` y el ESP abre la S a `INICIO_ANG_ADENTRO_DEG`=90 hasta `INICIO_LAT_ADENTRO_MM`=620 (centro a ~75 cm, más allá de la fila a 60). La decisión llega a ~30° de swing (`INICIO isal=2 (adentro) en fase 1 ang=34.2`); si llega en la fase 2, vuelve a la fase 1.
- Resultado: 40/41 a 28 fps, 38/41 a 14 fps; 100 % sin contacto y lado correcto en las señales de la recta del cajón en ambos (mín 15 mm a 28 fps, s18 rojo T1 por el PP; 25 mm a 14 fps). INICIO termina a t≈3.4 s (afuera) / 5.2 s (adentro) vs 6.2 s antes.
- Fallas restantes, todas en la 1ª esquina (no en la salida): s11 (28 y 14 fps) CCW, verde T2 por dentro y rojo S/T3 en la boca de la recta siguiente: `[DMAP] S/T3 Red` asentado pero `dmap_block=(0, 0, 0)` y el PP dobla temprano (`[PPDIAG] steer=-29.7deg obs=-0.495 lka=78 nobs=1`) → `señal Red S/T3@7.16`. Causa (lectura de código): `digital_map._left_pass/_green_ahead/blocks_turn` solo esperan por una lata de la boca que se pasa por la IZQUIERDA (el verde de CW); el espejo CCW (rojo por la derecha = afuera) no está. A 14 fps además s10 (rojo W/T2 tras la esquina: RECUPERANDO con el giro aún sin contar endereza hacia atrás) y 3170839 (verde N/T2 en la boca saliendo del carril interior).
- Carrera completa 21 semillas a 28 fps (`runs/race_fin`): suma tc 177 (HEAD documentado `runs/p5`: 94), 11 seeds a 12 giros, 2 terminan (20 y 3170839, 36 mm fuera). Nadie choca en la salida; 1, 3, 4, 15 chocan el rojo T2 de la recta del cajón al final de la vuelta 2 (tc 7-8, llegando desde la recta anterior).
- Defaults cambiados (robot real): `INICIO_REVERSA` nuevo = false (antes la reversa colchón de `INICIO_REV_MS`=2600 corría siempre); fase 2 por lateral en vez de `INICIO_MID_MS`=700 ms; fase 3 hasta 6° en vez de pico−25°; `delay(100)` solo al INIT; constantes nuevas `INICIO_ANG_ADENTRO_DEG`, `INICIO_LAT_AFUERA_MM`, `INICIO_LAT_ADENTRO_MM`, `INICIO_R_CONTRA_MM`, `INICIO_RECTO_MAX_MM`, `INICIO_RECTO_TIMEOUT_MS`, `INICIO_RECTO_DF_MIN_CM`, `INICIO_CONTRA_FIN_DEG`. V2: campo nuevo `,isal=` (solo con est=I y decisión tomada). Medir en el tapete: `INICIO_R_CONTRA_MM` (radio real de la contravuelta) y que el lateral del odómetro sea fiable (con `FOX_ENCODER 0` usa el modelo PWM).

### T15b (ruta A, fusionada) — estacionamiento en paralelo por encoder (agente 2026-10-04, rama `worktree-agent-a6501da593f9e5146`, commit `ae1e3ed`)
Harness corto `pure_pursuit/twin/tools/prepark.py <tag> [--jobs N] [--dirs] [--cards] [--par K=V,...]`: arranca directo en la recta final con `PARK_TEST_RECTA_COMPLETA` (tc=12), CW/CCW × cartas de la recta del cajón × perturbaciones de entrada/odometría (`twin/prepark.py`). Mide contacto, mm fuera de la caja, error de rumbo y paralelo (reglamento: ruedas a <=20 mm). Ruta A en el `.ino`: avance tras poste 2 y recta/hold/meneo de la fase 14 ahora por mm de encoder (`PARK_POR_ENCODER`, solo con `FOX_ENCODER`) en vez de tiempo; combo visto mejor POSTE2_M100/ENDEREZA10/FINAL_CM4/ANG_IN_D50; defaults equivalentes a los tiempos de antes.

Base del commit era `cf8f97d`, anterior a la salida del cajón sin reversa (`6d1234e`); se fusionó por cherry-pick sobre `digital-twin` (commit `54031cd`, merge `a852362`) — auto-merge limpio en `PurePursuit.ino`, sin conflictos, preservando el `case INICIO` de la salida.

Validación de la fusión: `pytest pure_pursuit/twin/tests` 72/73 (falla previa conocida `test_corner_hint::test_v2_carries_hint_when_enabled`, igual que antes). `salida.py all`: 40/41, idéntico a la base (misma falla s11). `all.sh` completo no se corrió (decisión del planner, CPU compartida con otro agente); en su lugar, seed 6 (llega a 12 giros y entra a estacionar) con `drive.py` da traza idéntica a la de `digital-twin` antes del merge: mismo `total_time_s=84.704`, `turns_completed=12`, mismos `lap_times_s`, misma colisión `cajón magenta@t=85.512574`, mismas distancias y `sim_loops=3209` (solo cambian `wall_time_s`/`mean_pi_frame_ms`, ruido de CPU) — sin regresión.

Harness `prepark.py` (jobs=8, cartas 1-6/25-30 × CW/CCW × 5 perturbaciones, 100 escenarios): `limpios=0/100 sin_contacto=100/100 fuera<=5=0/100 paralelo=0/100`. Nadie choca, pero ninguno queda paralelo (rumbo final ~88-90° de error, ruedas_dif≈113 mm, fuera de caja ≈33-46 mm): confirma lo ya documentado en HANDOFF — el harness entra bien pero la centerline de la Pi no está adaptada a la aproximación al cajón; falta esa parte para declarar T15b resuelto (pendiente harness desde tc=11 + vuelta en U, y ajustar la ruta final a la caja).

### Plan T16 — rendimiento del twin (2026-10-04)
Problema: ~7 s reales por s simulado (28 fps + render en tramos + CPU compartida); iteraciones de 15-30 min y muchos tokens.
1. Modo "percepción oráculo" (flag de Sim/preset): saltar render de cámara + visión; inyectar detecciones (pies de lata con color, naranja/azul, magenta) desde la verdad del twin con ruido, latencia, FOV y oclusión realistas, en el mismo formato que consume runtime_nuevo. Para iterar lógica (salida, esquinas, estacionamiento). Validación final siempre con visión completa.
2. Perfilar (cProfile) render + process_frame + loop de física/ctypes; optimizar sin cambiar resultados (métricas idénticas por seed).
3. Pasos de física 1 ms -> 2-5 ms solo si las métricas no cambian de forma relevante.
4. Herramientas que resumen (no leer logs completos).

### T16 — resultados (2026-10-04, rama t16-perf)
- El "~7 s/s" era medición con cProfile + CPU compartida. Real: ~1.8 s/s una seed sola; 10 concurrentes ~2.2 s/s cada una (10 seeds × 20 s sim = 47 s de pared). Con --no-show no hay ritmo por reloj real: el sim ya es cálculo puro y determinista.
- `World.cast` vectorizado (idéntico; 11 rayos 1.11 → 0.09 ms). `twin/tools/detcheck.py --save/--compare` = huella SHA256 de la traza sin campos de reloj real (IDENTICO = refactor exacto).
- Métricas nuevas `render_s`/`pi_s`/`other_s` (reloj real, fuera de la huella). 10 concurrentes, 20 s sim: render ~15.7 s, Pi ~15 s, resto (física 1 ms + firmware SIL) ~12 s — reparto parejo, no hay un cuello único.
- `FOX_CV_THREADS` (default min(4,cpu)) en vision.py/detect_orillas.py. Medido en lote de 10: 4 hilos 43.3 s vs 1 hilo 48.2 s por corrida (pi_s 15 → 22 s) → los lotes NO lo fuerzan a 1. all.sh/salida.py: 10 jobs, OMP/OPENBLAS/MKL=1.
- Oráculo (rama t16-oracle) descartado: 1.87 s/s vs 1.77 con visión (mean_pi_frame_ms 39.5 vs 16.9) y diverge (seed 6 choca pared exterior en 83.9 porque `_check_parking_search` usa processed_frame). La palanca real para iterar son los harness cortos (prepark/salida) + lotes paralelos.
- "Palos→puntos" en la centerline (rama cl-palos 3823db5, `CL_SMEAR_RECLAIM_MM`: recuperar como piso los píxeles rojo/verde cerca del pie) descartado: cambia la centerline en 141/352 frames con latas (seed 1, 50 s) pero las carreras 0 vs 150 mm salen bit-idénticas (seeds 3170839, 6, 1, 3, 4). Con `DIGITAL_MAP_STEER` la ruta es `digital.line_bev()` (runtime_nuevo.py:1857-1860), que ya trabaja con pies de lata como puntos; la centerline solo cuenta para `MIN_PATH_PTS` y durante el giro.

### T15b — diagnóstico y dos fixes de lógica (2026-10-04, rama t15b2)
- **Causa real de "queda a ~90°"**: `PARK_MODO` (PurePursuit.ino:83) estaba en `PARK_PUNTA`, no `PARK_PARALELO` — el estacionamiento real que corría siempre era el perpendicular (`ESTACIONANDO_PUNTA`), nunca el paralelo (`ESTACIONANDO`); confirmado en fw_debug.log ("PUNTA fase 4: ARCO..." en vez de "PARK fase N") y en el despacho único de PurePursuit.ino:2155. Esto ya estaba así en el worktree original del agente T15b (`agent-a6501da593f9e5146`, cf8f97d) y en `digital-twin` HEAD, así que el barrido de parámetros POSTE2_M100/ENDEREZA10/FINAL_CM4/ANG_IN_D50 de HANDOFF.md nunca ejercitó el código paralelo. Fix de una línea: `PARK_MODO = PARK_PARALELO` (commit `d069b84`).
- Con el modo corregido, diagnóstico de las colisiones contra el cajón: en la fase 1->2 el servo se pre-colocaba FULL EXTERNO mientras el motor solo entraba a coast (sin frenar), así que el carro seguía avanzando con las ruedas a tope y la trompa se abría hacia la pared ANTES de la reversa real (medido: `Ang` 0.0 -> 8.8° entre el fin de fase 1 y el fin de fase 2). Fix: servo en `centroServo` durante el coast de las fases 1->2; FULL EXTERNO recién al entrar a la fase 3 (commit `b0406cc`).
- Maniobra pedida por el usuario (pasa al lado -> avanza tras poste 2 -> reversa HASTA QUE el ToF trasero lea cerca -> corrige adelante -> reversa de nuevo -> repite hasta derecho): agregado `distB_filtrada`/`parkAtrasPegado()` (ToF trasero, `PARK_ATRAS_MIN_CM` PARK_AJ=5 cm) como corte de seguridad en las reversas de entrada (fases 3/14/4/8, van a fase 9 en vez de terminar) y en la fase 13 (acomodo final, junto al corte por dF que ya tenía); ciclo 9/11↔12/13 hasta `PARK_CICLOS_MAX`=3 si el rumbo sigue fuera de `PARK_FINAL_TOL_DEG` y no fue por "pegado" (commit `2b8f528`). Inerte sin `FOX_TOF` (hardware viejo no cambia de comportamiento).
- Validación: `prepark.py` con constantes default (sin `--par`) da el mismo resultado que el baseline ya documentado (`limpios=0/100`) — los dos fixes no rompen nada ni regresionan. En el set de escenarios probado el ToF trasero nunca bajó de `PARK_ATRAS_MIN_CM` (el choque medido es un corte/esquina del carro contra el cajón durante el swing de entrada, no un acercamiento recto a la pared de atrás), así que el ciclo tampoco se disparó en estas corridas; el ajuste fino de los parámetros de entrada (POSTE2_MM, ANG_IN_DEG, RADIO_CM, etc. — PARK_AJ, tuneables con `--par` sin recompilar) para que el carro quede efectivamente limpio y paralelo se deja al usuario, a mano.
- Pendiente (sin tocar en esta sesión): harness desde tc=11 con la vuelta en U (WIP en commit `203d36c` de la rama `worktree-agent-a6501da593f9e5146`, incompleto/sin validar) y la validación completa de seed 6 / `salida.py all` / pytest con la maniobra ya ajustada.
