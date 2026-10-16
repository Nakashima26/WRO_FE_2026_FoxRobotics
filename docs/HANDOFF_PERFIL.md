# Handoff: perfil de velocidad y vuelta 1 en modo ruta (twin)

Fecha: 2026-10-15. Rama: `fox/perfil-isla`. Sigue a `PLAN_TIEMPO.md`.

Todo lo de aquí es **twin**. El modo ruta nunca ha corrido en el carro real y el
motor del twin no está calibrado (lineal, `v = 3.5·(PWM−25)`). Las ganancias son
provisionales hasta medir el carro (Fase 0 de `PLAN_TIEMPO.md`).

Prioridad: ronda de obstáculos. Se ignora el estacionamiento. Batería llena.

## Resultado en una línea

De ~65 s (v3b) a ~55 s de mediana en el twin. Las fallas bajan de 17 a 14 (de 210 corridas).

## Mejor configuración (base actual, bench "y2")

Preset `ruta_perfil`, JSON de la Pi (los defaults del código NO la activan):

```json
{"pi": {"ROUTE_PROFILE_BASE_PWM": 205, "ROUTE_TIME_COST": true,
        "ROUTE_LAP1_PWM": 155, "ROUTE_UNSEEN_PWM": 155, "ROUTE_PROFILE_MIN_PWM": 155,
        "ROUTE_CTE_PWM": 205, "ROUTE_PROFILE_SIGN_MM": 220.0,
        "ROUTE_PROFILE_SIGN_NEAR_MM": 150.0, "ROUTE_FIX_STEP_MM": 8.0,
        "ROUTE_PROFILE_A_LAT_MM_S2": 2000.0, "ROUTE_PROFILE_BRAKE_FF_S": 0.25,
        "ROUTE_CTE_URGENT_MM": 120.0, "ROUTE_STOP_MM": 0.0,
        "ROUTE_UPLOAD_DIFF_MM": 60.0}}
```

Para verla en el visor (desde este worktree, no desde otra rama):

```bash
cd src/RASPI/cam && PYTHONUTF8=1 TWIN_NOISE_SEED=1 /Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python -m pure_pursuit.chassis_twin --preset ruta_perfil --park-pos random --seed 1 --speed 1 --pi ROUTE_PROFILE_BASE_PWM=205 --pi ROUTE_TIME_COST=True --pi ROUTE_LAP1_PWM=155 --pi ROUTE_UNSEEN_PWM=155 --pi ROUTE_PROFILE_MIN_PWM=155 --pi ROUTE_CTE_PWM=205 --pi ROUTE_PROFILE_SIGN_MM=220 --pi ROUTE_PROFILE_SIGN_NEAR_MM=150 --pi ROUTE_FIX_STEP_MM=8 --pi ROUTE_PROFILE_A_LAT_MM_S2=2000 --pi ROUTE_PROFILE_BRAKE_FF_S=0.25 --pi ROUTE_CTE_URGENT_MM=120 --pi ROUTE_STOP_MM=0 --pi ROUTE_UPLOAD_DIFF_MM=60
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

## Cuánto queda (estimación)

- Hoy, vueltas 2-3: ~13.6 s/vuelta en el twin.
- El perfil de hoy seguido perfecto daría ~11.8 s/vuelta. El límite físico del twin es ~9.8 s/vuelta.
- Total ideal ~33 s en el twin, ~37-40 s con supuestos del carro real. Hay 17-24 s de margen.
- La línea de carrera vale poco (~0.35 s/vuelta). La velocidad en rectas y la vuelta 1 valen mucho más.

## Siguientes pasos

1. Probar `ROUTE_UPLOAD_DIFF_MM` 100 y volver a probar la vuelta 1 a 170 sobre y2.
2. El arranque y la primera sección (~10 s a PWM 95-110).
3. Vuelta 1 (~22.4 s): el mayor hueco que queda. Las fallas que siguen (semillas 16, 32, 21) son toques de señal en la vuelta 1.
4. **En el carro:** cargar el firmware con `f5b1ef9`, medir velocidad vs PWM y frenado (`twin/tools/calib/HOJA_MEDICION.md`), y calibrar el twin.
