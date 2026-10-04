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
Hardware confirmado por el usuario (2026-10-03): cambio de dimensiones/estructura en camino; ToF a izquierda, derecha, frente y atrás; encoder en el motor.
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
