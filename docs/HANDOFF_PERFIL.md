# Handoff: perfil de velocidad y vuelta 1 en modo ruta (twin)

Fecha: 2026-10-15. Rama: `fox/perfil-isla`. Sigue a `PLAN_TIEMPO.md`.

Todo lo de aquí es **twin**. El modo ruta nunca ha corrido en el carro real y el
motor del twin no está calibrado (lineal, `v = 3.5·(PWM−25)`). Las ganancias son
provisionales hasta medir el carro (Fase 0 de `PLAN_TIEMPO.md`).

Prioridad: ronda de obstáculos. Se ignora el estacionamiento. Batería llena.

## Resultado en una línea

De ~65 s (v3b) a ~55 s de mediana en el twin. Las fallas bajan de 17 a 14 (de 210 corridas).

## Mejor configuración (base actual, bench "st80" = lu185 + `ROUTE_UPLOAD_STEP_MM` 80)

Preset `ruta_perfil`, JSON de la Pi (los defaults del código NO la activan):

```json
{"pi": {"ROUTE_PROFILE_BASE_PWM": 205, "ROUTE_TIME_COST": true,
        "ROUTE_LAP1_PWM": 185, "ROUTE_UNSEEN_PWM": 185, "ROUTE_PROFILE_MIN_PWM": 155,
        "ROUTE_CTE_PWM": 205, "ROUTE_PROFILE_SIGN_MM": 220.0,
        "ROUTE_PROFILE_SIGN_NEAR_MM": 150.0, "ROUTE_FIX_STEP_MM": 8.0,
        "ROUTE_PROFILE_A_LAT_MM_S2": 2000.0, "ROUTE_PROFILE_BRAKE_FF_S": 0.25,
        "ROUTE_CTE_URGENT_MM": 120.0, "ROUTE_STOP_MM": 0.0,
        "ROUTE_UPLOAD_DIFF_MM": 60.0, "ROUTE_LAP1_OPEN": true,
        "ROUTE_FIX_LINE": true, "ROUTE_UNSEEN_RAMP_MM": 850.0,
        "DIGITAL_MAP_ORANGE_BEHIND_MM": 1500.0, "ROUTE_UPLOAD_STEP_MM": 80.0},
 "fw": {"INICIO_PWM": "120", "VEL_INICIAL": "155"}}
```

Para verla en el visor (desde este worktree, no desde otra rama):

```bash
cd src/RASPI/cam && PYTHONUTF8=1 TWIN_NOISE_SEED=1 /Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python -m pure_pursuit.chassis_twin --preset ruta_perfil --park-pos random --seed 1 --speed 1 --pi ROUTE_PROFILE_BASE_PWM=205 --pi ROUTE_TIME_COST=True --pi ROUTE_LAP1_PWM=185 --pi ROUTE_UNSEEN_PWM=185 --pi ROUTE_PROFILE_MIN_PWM=155 --pi ROUTE_CTE_PWM=205 --pi ROUTE_PROFILE_SIGN_MM=220 --pi ROUTE_PROFILE_SIGN_NEAR_MM=150 --pi ROUTE_FIX_STEP_MM=8 --pi ROUTE_PROFILE_A_LAT_MM_S2=2000 --pi ROUTE_PROFILE_BRAKE_FF_S=0.25 --pi ROUTE_CTE_URGENT_MM=120 --pi ROUTE_STOP_MM=0 --pi ROUTE_UPLOAD_DIFF_MM=60 --pi ROUTE_LAP1_OPEN=True --pi ROUTE_FIX_LINE=True --pi ROUTE_UNSEEN_RAMP_MM=850 --pi DIGITAL_MAP_ORANGE_BEHIND_MM=1500 --pi ROUTE_UPLOAD_STEP_MM=80 --fw INICIO_PWM=120 --fw VEL_INICIAL=155
```

(Ajustar la ruta del venv en otra compu.)

## Cambios de código en esta rama

Todos apagados por defecto salvo el del ESP32.

| Commit | Qué | Estado |
|---|---|---|
| `f5b1ef9` | **ESP32 modo ruta: conteo de giros al día y paro dentro de la sección de salida.** Antes el conteo ("GIRO PI contado") tenía un hueco de 4 s y se paraba 1-3 secciones tarde. | Activo. −7.3 s en el twin. **Cargar al ESP antes de probar el modo ruta en el carro.** |
| `dc36952` | Perfil v3: piso propio (`ROUTE_PROFILE_MIN_PWM`) y freno proporcional cerca de señales (`SIGN_NEAR_MM`). | Usado en la base |
| `71acf75` | `ROUTE_PROFILE_ISLAND_MM`: frenar cerca de las 4 esquinas de la isla. | Probado: −3 fallas pero +0.47 s. No se usa |
| `4b5cf81` | `ROUTE_PROFILE_BRAKE_FF_S`: al frenar pide v − τ·a (compensa el retraso del motor) y deja bajar el PWM hasta 60. | Usado en la base (0.25) |
| `e4238af` | **`ROUTE_FIX_STEP_MM`: arranque ciego pesa cada lectura por el avance.** Con el carro casi quieto el sonar repetía la misma lectura, y contarla 9 veces fijaba el cajón 400 mm mal con 97 % de confianza (semilla 8). | Usado en la base (8 mm) |
| `8ded2c4` | `ROUTE_PROFILE_TIGHT_KAPPA`: aceleración lateral menor en curvas muy cerradas (radio menor que el lookahead de 200 mm). | Probado (h1/h2): no arregla la semilla 3. No se usa |
| `92fd3c4` | `ROUTE_CTE_URGENT_MM`: **freno por urgencia al salirse.** Si el carro está fuera de la ruta, se corre la ruta de adelante (600 mm) el mismo desvío. Solo si así quedaría a menos de 120 mm de la isla, una señal o la pared, frena a `ROUTE_CTE_URGENT_PWM`. Si se sale poquito pero tiene espacio, no frena. | Usado en la base (120 mm, PWM 120) |
| `3456498` | `ROUTE_UPLOAD_DIFF_MM`: mientras sube la ruta nueva al ESP, frenar solo si se aparta de la que tiene el ESP en los próximos 700 mm. | Usado en la base (60 mm) |
| `e4490bd` | **`ROUTE_FIX_LINE`: arranque forzado con la línea naranja.** Si el sonar no converge antes de acabar la recta, el δ del cajón sale de la mediana de la línea naranja de la esquina: el punto de la recta del BEV en la columna central se lleva al mundo con la pose supuesta y se corta con la cinta. Offline (42 semillas): peor error forzado 106 → 58 mm. | Usado en la base. Banco l1 vs o1: fallas 12 → 6, mismo tiempo, sin semillas nuevas |
| `763a72b` | `ROUTE_UNSEEN_RAMP_MM`: en vuelta 1, a más de esa distancia de lo no visto, sube el PWM según la distancia de frenado en vez de quedarse en `UNSEEN_PWM`. | Usado en la base (850). Sobre l1: 800 = −0.67 s pero 7 fallas solo suyas (no pasa, semilla 3 isla); 850 = −0.53 s, 5 fallas (pasa); 900 = −0.43 s, 6; 1000 = −0.2 s, 5 |

## Cómo se decide (banco estándar)

Herramientas en `src/RASPI/cam/pure_pursuit/twin/tools/banco/`:

- `bench.sh <worktree> <nombre> ruta ruta_perfil '<json>'`: 42 semillas × 5 ruidos, base + candidato. Con `BASE_REF=<ruta a runs/combo1>` enlaza la base ya corrida en vez de recorrerla o copiarla. Si se corta, relanzar el mismo comando: se salta las corridas ya hechas.
- `vsref.py REF VAR...`: compara corrida por corrida contra una referencia. Da las fallas, los pares que fallan solo en uno de los dos lados, las semillas nuevas que fallan, y la mediana de dt del total y de la vuelta 1 y las vueltas 2 y 3 (medidas geométricamente desde trace.csv).
- `startfix.py RUNS/<nombre>`: error del arranque ciego contra la verdad (`parkw` de field.txt).

**Regla:** ahorra ≥ 0.5 s mediano **y** solo-candidato ≤ solo-referencia + 2 **y** ninguna semilla
que nunca fallaba falla en ≥ 2 ruidos.

**Disco:** cada bench ocupa ~0.7 GB (con `BASE_REF`; ~1.7 GB si se copia la base). Revisar `df -h /` antes
de lanzar. **Limpiar al analizar:** en cuanto un bench se compara y se descarta, borrar su carpeta
`runs/<nombre>` y anotar el resultado en la tabla de abajo. Se conservan solo la base actual,
la anterior y las referencias (combo1).

## Lo que se probó (contra la base de cada momento)

| Prueba | Resultado | Decisión |
|---|---|---|
| Quitar el freno de subida de ruta / subir a 155 / STOP_MM 0 / LAP1_OPEN | −0.8 a −2 s en la vuelta 1, pero +4 a +9 fallas (semillas 5 y 36 chocan a los ~22 s) | No |
| SOFT_CLEAR 100, suavizar la ruta | +fallas | No |
| `ROUTE_CTE_PWM` 205 (el freno al salirse de la ruta ya no baja a 100) | −0.87 s | Sí |
| `SIGN_MM` 220 (zona de señal más corta) | −1.75 s | Sí |
| `A_LAT` 2000-2500 | −0.8 a −1.2 s, pero la semilla 3 choca 5/5 en la esquina de la isla | No |
| Esquinas de la isla / latencia 0.45 / frenada 400 | Más seguro pero más lento (+0.3 a +0.5 s) | No |
| `A_LAT` 2000 + `BRAKE_FF_S` 0.25 (g4) | −0.57 s vs g3, pero 7 solo-candidato (el límite es 6); la semilla 3 choca 4/5 | Casi. Falta la semilla 3 |
| `ROUTE_FIX_STEP_MM` 15 / 8 | 15: converge tarde, 28 fallas. 8: 16 fallas vs 19, mismo tiempo | 8 sí |
| Ruta más pegada a las señales (holgura 60) | Más corta (−0.2 m) pero más lenta (+0.37 s/vuelta): entra más a la zona de freno de señal | No |
| Freno de señal solo donde hay curva (cálculo en papel) | −0.13 s/vuelta como mucho | Pendiente, poca ganancia |
| g4 + `TIGHT_KAPPA` 0.004 (h1) / 0.003 (h2) | h1: −0.11 s, 15 fallas; h2: +0.62 s, 14 fallas (vs 16 de g3). La semilla 3 sigue chocando | No |
| Causa de la semilla 3 | Tras la S cerca de (−750, −230) sale a ~790 mm/s unos 60 mm por dentro de la ruta, hacia la isla, antes de la esquina. Bajar la velocidad de la esquina no ayuda: el problema es el desvío | → freno por urgencia |
| Urgencia en la semilla 3 (sobre g4, 5 ruidos) | 120 mm / PWM 120: choca 2/5 (g4: 4/5), mismo tiempo. Frenar más fuerte (PWM 100) o margen 160 mm: peor, salen choques nuevos a los ~34 s | 120/120 al banco |
| **u1 = g3 + urgencia** | Mismo tiempo, 14 fallas vs 16, solo-u1 2 vs solo-g3 4. La urgencia no cuesta tiempo | Sí |
| **u2 = g4 + urgencia** | −0.50 s vs g3, 16 fallas (igual), solo-u2 6 vs 6, sin semillas nuevas. Contra g4: mismo tiempo y 16 fallas vs 19. Semilla 3: 2/5 (antes 4/5) | **Sí, base nueva** |
| Sobre u2: A_LAT 2500 (v1) | −0.53 s, pero 19 fallas, 7 solo-v1; la semilla 3 vuelve a 5/5 | No |
| Sobre u2: frenada 800 (v4) | −0.12 s, 14 fallas | No llega a 0.5 s; más seguro |
| Sobre u2: BASE_PWM 225 cerca de señales (v2) | −0.33 s, 17 fallas, semilla 3 5/5 | No |
| Sobre u2: vuelta 1 a 170 (v3) | −0.30 s (−0.24 en vuelta 1), 13 fallas | No llega a 0.5 s; la vuelta 1 no la frena el tope |
| Sobre u2: ver asientos desde 900 mm, vacío en 10 frames (w1) | −0.1 s, 23 fallas | No |
| **Dónde está el tiempo (u2)** | Vuelta 1: 21.9 s a 315 mm/s, 1.7 s detenido, 19.6 s a PWM ≤ 160. Vueltas 2-3: 11.3 s a 655 mm/s. Arranque + primera sección a PWM 95-110 (~10 s) | La vuelta 1 tiene ~10 s de margen |
| **Sobre u2: sin parar a confirmar asientos, `ROUTE_STOP_MM` 0 (x1)** | −0.50 s (−0.56 en vuelta 1), 14 fallas vs 16, solo-x1 7 vs solo-u2 9. Con la urgencia ya no choca como antes | **Sí, base nueva** |
| Sobre u2: sin frenar al subir la ruta, `ROUTE_UPLOAD_SLOW` false (x2) | −1.15 s, pero 26 fallas, 13 solo-x2 | No. Mientras sube la ruta, el ESP sigue la vieja y choca |
| Sobre u2: x1 + x2 (x3) | −1.81 s, 26 fallas, semilla 14 nueva | No |
| **Sobre x1: al subir la ruta frenar solo si cambia >60 mm en los próximos 700 mm, `ROUTE_UPLOAD_DIFF_MM` 60 (y2)** | −0.75 s (−0.62 en vuelta 1), 14 fallas (igual), solo-y2 6 vs 6 | **Sí, base nueva** |
| Sobre x1: lo mismo con 30 mm (y1) | −0.60 s, 13 fallas | Pasa, pero y2 gana más |
| Sobre r85: holgura a señales `SIGN_MM` 190 (s19) / 205 (s205) | 190: −0.98 s, 10 fallas, semilla 14 nueva. 205: −0.42 s, 10 fallas, semilla 3 nueva | No |
| Sobre r85: premio por ver asientos `ROUTE_W_VISIBLE` 200 (v2) / pegarse al asiento sin ver `ROUTE_W_UNKNOWN_NEAR` 4 (u4) | v2: mismo tiempo, 9 fallas. u4: +0.1 s, 11 fallas | No. Los pesos del planificador ya estaban bien |
| Sobre r85: arranque ciego a PWM 155, `ROUTE_BLIND_PWM` (b155) | −0.2 s, 8 fallas, semilla 5 nueva | No. Casi no gana porque el ESP topa a `VEL_INICIAL` (110) hasta el primer giro |
| **Sobre r85: firmware `VEL_INICIAL` 155 (f1) / 140 (f140)** (fw JSON `{"VEL_INICIAL": "155"}`) | f1: **−0.89 s** (vuelta 1), 8 fallas, solo-f1 4 vs solo-r85 1 (límite 3). f150: −0.62 s, 11 fallas (semillas 32 y 3 nuevas). f140: −0.48 s, 6 fallas, semilla 3 2/5. Con arranque ciego 155 también (f2): −1.45 s pero 22 fallas (asientos T1) | Casi. Es la palanca más grande que queda: la primera sección va a 110 por el firmware. Hace falta un cambio en el ESP (en modo ruta no usar `VEL_INICIAL`) y arreglar las fallas que suma |
| Subir la ruta al ESP más rápido | A 115200 baud cada frame ya lleva ~290 bytes (V2 ~190 + una RTP ~100); el límite es ~288. Haría falta subir el baud en Pi y ESP; gana ≤0.2 s | No por ahora |
| **Sobre r85: salida del cajón más rápida, firmware `INICIO_PWM` 95 → 120 (i120)** | **−0.72 s** (−0.55 s en la salida), 6 fallas vs 5, solo-i120 4 vs solo-r85 3, sin semillas nuevas. Promedio r85: 43.6 s (mejor 37.3 s) | **Sí (base).** Es firmware: hay que cambiar `INICIO_PWM` en el .ino y revisar en el tapete que la S no se pase por inercia (`INICIO_OVERSHOOT_DEG`) |
| Sobre i120: `INICIO_PWM` 140 (i140) | −0.25 s, 10 fallas | No |
| Sobre i120: `VEL_INICIAL` 155 (fi); + `ROUTE_UNSEEN_PWM` 140 (fu); + paso justo a señal `ROUTE_PROFILE_SIGN_TIGHT_MM` 125/PWM 110 (ft) o 145/100 (ft2) | fi: −0.89 s, 10 fallas, semilla 6 nueva. fu: −0.54 s, 19 fallas. ft: −0.95 s, 8 fallas (solo 6 vs 4). ft2: −0.89 s, 10 fallas | No. La semilla 6 no es de velocidad: ver la fila siguiente |
| Semilla 6 (CW, cajón E): S/X1 verde se ve a ~870 mm pero se confirma ~500 mm después | Recién girado a S la cámara sigue viendo la cinta naranja de la esquina que quedó atrás (`near_y`≈340). El mapa la toma por la siguiente, la lata queda "pasada la cinta" y se busca en la recta W (sin asiento a <320 mm): ~45 vistas tiradas. `DIGITAL_MAP_ORANGE_BEHIND_MM` (a439ceb, apagado): ignora la cinta si `along + adelante < 1500` | Sobre i120 solo (ob): 0.00 s, 6 fallas, misma lista; semilla 6 39.8 → 37.7 s. **Con `VEL_INICIAL` 155 (obf): −1.07 s (vuelta 1 −1.0), 8 fallas, solo-obf 7 vs solo-i120 5, sin semillas nuevas → base nueva.** Firmware: `VEL_INICIAL` 110 → 155 en el .ino |
| Sobre obf: holgura a señales `SIGN_MM` 190 (os19) / 200 (os20); rampa 800 (or8) | os19: −1.0 s, 12 fallas, solo 7 vs 3, semilla 14 3/5 (entra a la esquina SO a ~790 mm/s y abre contra W/T2). os20: −0.57 s, 17 fallas, semillas 32 y 19 nuevas. or8: −0.09 s | No. `SIGN_MM` no es monótono: se queda en 220 |
| Sobre obf: `ROUTE_LAP1_PWM` 170 sola (ol17) / `ROUTE_UNSEEN_PWM` 170 sola (ou17) | Idénticas a la base: lo no visto usa `min(LAP1_PWM, UNSEEN_PWM)`, hay que subir las dos | — |
| Sobre obf: PWM base 225 (ob225); `MIN_PWM` 170 (om17) | ob225: −0.49 s, 13 fallas, semillas 9 y 32 nuevas. om17: −0.23 s, 14 fallas | No |
| **Sobre obf: `LAP1_PWM` = `UNSEEN_PWM` = 163 / 170 / 185 / 200 (lu16/lu17/lu185/lu20)** | 163: −0.2 s, 9 fallas. 170: −0.38 s, 9. **185: −0.67 s (vuelta 1 −0.65), 9 fallas, solo 5 vs 4.** 200: −0.77 s, 10 fallas, solo 7 vs 5 | **Sí: 185 (base).** 200 también pasa, pero da una falla más por 0.1 s |
| Sobre lu185: firmware `VEL_INICIAL` 185 sola (fv185) | Idéntica: en la primera sección manda el arranque ciego (`ROUTE_SLOW_PWM` 100) | — |
| Sobre lu185: arranque ciego + `VEL_INICIAL` a 185 (fb185) / 170 (fb170); `INICIO_PWM` 130 (fi130) | fb185: −0.62 s, 27 fallas (pared, semilla 30 5/5). fb170: −0.42 s, 21 fallas. fi130: −0.2 s, semillas 18 y 20 nuevas | No. El arranque ciego tiene que ir lento |
| Sobre lu185: settle de la salida del cajón con motor, firmware `INICIO_SETTLE_MOTOR` (sm, 1742c19) / + arranque ciego 120 (smb) | Antes el settle hacía coast y el carro casi paraba (230 → 30 mm/s, ~0.3 s). sm: −0.31 s, 16 fallas, semilla 15 5/5 (llega a S/T2 aún en arranque ciego). smb: −0.43 s, 21 fallas | No |
| Sobre lu185: solo el tope del arranque ciego `ROUTE_BLIND_PWM` 120 (bl12) / 135 (bl135) | bl12: −0.2 s, 12 fallas (semilla 3 isla 4/5). bl135: −0.34 s, 10 fallas, semilla 18 nueva | No |
| Plan tras fijar el arranque (e3aa662) | Al cambiar `park_w` se recalculaban paredes, isla, señales y vista (la clave de caché incluía el cajón): 113 ms contra ~12 ms normal. Ahora la caché va por malla y las maderas del cajón aparte: 37 ms, ruta idéntica bit a bit (mismo tiempo en el twin) | Sí (sin flag, mismo resultado) |
| **Sobre lu185: subir la ruta al ESP con un punto cada 60 / 80 mm en vez de 40, `ROUTE_UPLOAD_STEP_MM` (st60/st80, c70a9aa)** | Cada subida (~190 puntos, 21 líneas, una por frame) dejaba ~0.5 s a PWM 100 en la vuelta 1. 60: −0.48 s, 8 fallas. **80: −0.93 s (vuelta 1 −0.8), 10 fallas, solo 5 vs 4, sin semillas nuevas.** Mediana total de lu185: 41.1 s | **Sí: 80 (base).** En el carro: el ESP sigue la ruta con puntos a 80 mm (`RUTA_LOOKAHEAD_MM / rutaPaso`) |
| Sobre st80: paso 100 / 120 mm (st100/st120) | st100: −0.08 s, 16 fallas (isla 6). st120: −0.15 s, 21 fallas, 4 semillas nuevas. La ruta gruesa corta las esquinas | No: 80 es el punto |
| Sobre st80: PWM 130 durante la subida (up13); horizonte de riesgo 500 mm (uh5) | up13: −0.12 s, 21 fallas. uh5: −0.03 s, 10 fallas | No |
| Sobre st80: 2 / 3 líneas de subida por frame, `ROUTE_UPLOAD_LINES` (ul2/ul3, 20aeb81; en el carro pediría subir el baud) | ul2: −0.35 s, 23 fallas. ul3: −0.5 s, 18 fallas, 4 semillas nuevas | No. La subida lenta funciona como freno tras confirmar una señal: subir más rápido llega a la señal antes de reaccionar |
| Sobre st80: relación más larga simulada (`TWIN_SPEED_SCALE`/`TWIN_TAU_SCALE`, ce5ca94). Mismos PWM: 1.33× (g133) y 1:1 = 1.5× (g15). Misma velocidad mm/s en topes y arranque (todos los PWM convertidos, `ROUTE_PROFILE_K` 5.25): g15e con aceleración 1.5× más lenta, g15t sin perder aceleración | g133: −2.55 s, 20 fallas. g15: −3.64 s, 44 fallas. g15e: +1.65 s, 19 fallas. g15t: +1.02 s, 13 fallas | No en el twin. La ganancia con mismos PWM viene de ir más rápido junto a señales (eso ya se puede subir y choca); el techo de recta casi no se usa. Supuestos sin medir: tau del motor y PWM de arranque con 1:1 |
| Motor de hoja de datos en el twin, `TWIN_MOTOR=fisico` (6de8cac): N20 50:1 460 rpm / 0.14 N·m a 12 V, batería 12.6 V, rueda 43 mm, 0.7 kg (supuesto; el peso casi no influye), μ 0.8 y 55 % en el eje trasero. Con 1.5:1: k 3.03, pwm0 19 (716 mm/s a 255), τ 0.057 s (antes se suponía 0.25), tope de agarre 4.3 m/s². pf = st80 con los mismos PWM; pfe = st80 con PWM convertidos a los mismos mm/s y `ROUTE_PROFILE_K` 3.03 / `PWM0` 19 | st80 40.24 s mediana (10 fallas). pf: 42.43 s (7 fallas). pfe: 40.60 s (10 fallas) | pfe es la base con motor físico. Con los PWM de hoy el carro real iría ~2 s más lento que en el twin viejo: en el carro hay que usar los PWM de pfe (más altos) |
| Relación con motor físico, sobre pfe (`TWIN_GEAR`, PWM a los mismos mm/s): pg125 = 1.25:1 (k 3.64, τ 0.082), pg1 = 1:1 (k 4.55, τ 0.126) | pg125: −0.97 s (39.74 s), 8 fallas, sin semillas nuevas. pg1: −0.03 s, 13 fallas | **1.25:1 pasa la regla.** 1:1 no gana: con τ más lento sigue peor la ruta. Antes (sin física) la relación no ganaba porque el twin castigaba la aceleración de más |
| Frenado con motor físico, sobre pfe: `BRAKE_FF_S` 0.08 (pbf) / 0.15 (pbf15), `A_BRAKE` 1500 (pab), `LATENCY_S` 0.12 (plat) / 0.18 (plat18), **0.18 + BRAKE_FF 0.15 (pc1)** | pbf −0.59 s, 17 fallas (solo 10). pbf15 −0.37 s, 10. pab +0.67 s. plat −0.64 s, 15 (solo 8). plat18 −0.25 s, 8. **pc1 −0.62 s, 12 fallas (solo 5 vs 3)** | **pc1 base nueva** (con motor rápido, el frenado anticipado de τ 0.25 sobraba). En el carro: latencia sin medir en la Pi |
| Sobre pc1: vuelta 1 `ROUTE_LAP1_PWM` = `ROUTE_UNSEEN_PWM` +15 (pl1, 218) / **+35 (pl2, 238)**; `ROUTE_PROFILE_MIN_PWM` +15 (pmin) | pl1 −0.18 s, 9 fallas. **pl2 −0.32 s, 10 fallas vs 12**. pmin −0.2 s, 14 | **pl2 base nueva.** Regla nueva (2026-10-16): también entra si baja el tiempo y no suben las fallas ni hay semillas nuevas, aunque gane <0.5 s |
| Sobre pl2 (motor físico): vuelta 1 +55 (pl3, 258); `ROUTE_PROV_RULES` (ppr); `KAPPA_WIN` 15 (pkw); `UNSEEN_RAMP_MM` 700 (pr7); `SIGN_MM` 190 (ps19); `ROUTE_SEAT_ROI` (psr1) y + `SEAT_ROI_COLOR` (psr2); fw `INICIO_PWM` 150 (pin15); `ROUTE_SLOW_PWM` 125 (psl12) | pl3 −0.07 s, 12. ppr −0.08, 12. pkw +0.14, 11. pr7 −0.05, 12. ps19 −0.85, 19 (2 semillas nuevas). psr1 0.0, 13. psr2 −0.32, 17. pin15 −0.27, 22. psl12 −0.18, 14 | No: todos suben fallas. La vuelta 1 pierde ~5.5 s a PWM <160 (2.4 s salida del cajón, ~3 s tope lento por arranque ciego/subida). Semilla 30: choca con N/T2 en la recta del cajón (error del cajón 212 mm, la verde solo se ve 'far' sin distancia) |
| Arranque ciego con el ToF frontal (VL53L1X Long) votando en el histograma (prototipo, no se dejó en el código) | Semilla 30: δ se fija en el mismo frame que sin ToF. El ToF frontal da `tF=-1` casi siempre: el twin modela la pared negra con reflectividad 0.04 y no hay eco a distancia | No: con paredes negras el ToF no ve el fondo. Medir en el carro el alcance del ToF frontal sobre la pared negra antes de reintentar |
| Sobre pl2: tope al subir la ruta 140 en vez de 105, `ROUTE_UPLOAD_PWM` (pup14) / tope del arranque ciego 130, `ROUTE_BLIND_PWM` (pbl13) | pup14: −0.10 s, 17 fallas, semilla nueva 27. pbl13: −0.18 s, 22 fallas, semilla nueva 18 | No. Los dos topes lentos evitan choques |
| Ruta planeada de la semilla 1 (sin bench) con `ROUTE_W_KAPPA` 3e4 / 3e5 / 1e6 | La curvatura total baja solo de 11.4 a 10.5 rad y la vuelta se alarga 430 mm (+5 %). Las ondas las obligan las reglas de lado (rojo por dentro, verde por fuera) | No vale la pena: el costo por tiempo ya equilibra curva contra largo |
| Sobre y2: `ROUTE_UPLOAD_DIFF_MM` 100 (z1) | 0.00 s, 17 fallas | No |
| Sobre y2: vuelta 1 a 170 (z2) | −0.22 s, 16 fallas | No (poca ganancia) |
| Vuelta 1 siguiendo el BEV en vez de la ruta (`ROUTE_LAP1` false sobre y2, `MOTOR_MAX` 180; y `hw_nuevo` puro a 120/155/180, 12 semillas, ruido 1) | Dentro de ruta: semilla 1 vuelta 1 en 31.8 s y choca con el cajón, semilla 5 no cuenta giros. BEV puro: completan la vuelta 1 4/12 a 120, 1/12 a 155, 1/12 a 180, en 26.9-31 s (y2: ~21 s, 196/210 sin falla). Más rápido falla más | No: el BEV es más lento y choca |
| Sobre y2: un replan por señal, reglas de la recta con el color provisional (`ROUTE_PROV_RULES`, p1) | −0.08 s, 13 fallas. Menos replans (p. ej. 17→10) | Neutral en el twin (latencia 0); puede ayudar en la Pi |
| **Sobre y2: sin tope de vuelta 1 donde ya no queda nada sin ver adelante, `ROUTE_LAP1_OPEN` (o1)** | −0.75 s (−0.67 en vuelta 1), 12 fallas vs 14, solo-o1 3 vs 5 | **Sí, base nueva** |

## Cuánto queda (estimación)

- Hoy, vueltas 2-3: ~13.6 s/vuelta en el twin.
- El perfil de hoy seguido perfecto daría ~11.8 s/vuelta. El límite físico del twin es ~9.8 s/vuelta.
- Total ideal ~33 s en el twin, ~37-40 s con supuestos del carro real. Hay 17-24 s de margen.
- La línea de carrera vale poco (~0.35 s/vuelta). La velocidad en rectas y la vuelta 1 valen mucho más.

## Siguientes pasos

1. **Latencia del plan (medido sobre o1):** con `ROUTE_PLAN_DELAY_S` 0.3 las fallas suben de 12 a 61, y con 0.6 a 123 de 210. En el twin el plan es instantáneo (~60 ms en Mac). Hay que medir cuánto tarda la DP en la Pi 5 (C + OpenMP) antes de creerle al twin. Con 0.15 s: 24 fallas (+0.53 s); 0.1 s: 15 (+0.23 s); 0.05 s: 18 (+0.07 s), ruido de ±3. **Plan 4x más rápido (0241ad2):** kernel DP disperso + caché por asiento + B-spline vectorizada: tibio 55 → 14 ms en Mac, misma ruta bit a bit (mismo_trace IGUAL). Pi 5 estimado ~30-40 ms, sin medir. `ROUTE_PENDING_PWM` (b1579e3, apagado) con 0.3 s de retraso: 120 → 34 fallas (+1.57 s), 155 → 61 (no sirve). Con el plan rápido no hace falta. Medir en la Pi: líneas `[ROUTE] plan#… XXms` del log.
- **Rampa de vuelta 1 (`ROUTE_UNSEEN_RAMP_MM`, 763a72b, apagada):** frena en rampa hacia el asiento sin ver en vez del escalón a 155 dentro de 1300 mm. Sobre o1: 900 → −0.40 s, 13 fallas (solo 4/3); 800 → −0.58 s, 16 fallas (solo 8/4); 700 → −0.78 s, 15 fallas (solo 5/2). Ninguna pasa la regla; las fallas no son monótonas con la distancia (ruido de ±3). Gana tiempo real en la vuelta 1 pero suma 1-4 choques sueltos.
2. **Odometría / arranque ciego (medido en o1, 42 semillas):** con el retraso de ~150 ms descontado, el EKF queda en ~15 mm laterales p95 cuando el cajón se fija bien. El 25 % de los arranques se fija FORZADO (no converge antes de acabar la recta); siempre pasa cuando el cajón está a <200 mm del supuesto. Si el error del cajón es >50 mm, falla el 31 % de las corridas (5/16); si es ≤50 mm, falla el 3.6 % (7/194). Hecho: la línea naranja fija el caso forzado (`ROUTE_FIX_LINE`, fallas 12 → 6). Ojo: cuando el sonar sí converge, la línea es peor (mediana 21 mm contra 10), por eso solo se usa en el forzado. En el carro real hay que comprobar que el tracker naranja ve la cinta en la recta del cajón. Queda abierto: frenar antes del límite de la recta si no ha convergido. El ruido de sonar del twin (3 mm) es optimista.
3. El arranque y la primera sección (~10 s a PWM 95-110).
4. Vuelta 1 (~22.4 s): el mayor hueco que queda. Las fallas que siguen (semillas 16, 32, 21) son toques de señal en la vuelta 1.
5. **En el carro:** cargar el firmware con `f5b1ef9`, medir velocidad vs PWM y frenado (`twin/tools/calib/HOJA_MEDICION.md`), y calibrar el twin.
