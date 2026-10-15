# Handoff: perfil de velocidad y vuelta 1 en modo ruta (twin)

Fecha: 2026-10-15. Rama: `fox/perfil-isla`. Sigue a `PLAN_TIEMPO.md`.

Todo lo de aquí es **twin**. El modo ruta nunca ha corrido en el carro real y el
motor del twin no está calibrado (lineal, `v = 3.5·(PWM−25)`). Las ganancias son
provisionales hasta medir el carro (Fase 0 de `PLAN_TIEMPO.md`).

Prioridad: ronda de obstáculos. Se ignora el estacionamiento. Batería llena.

## Resultado en una línea

De ~65 s (v3b) a ~57 s de mediana en el twin. Las fallas bajan de 17 a 16 (de 210 corridas).

## Mejor configuración (base actual, bench "g3")

Preset `ruta_perfil`, JSON de la Pi (los defaults del código NO la activan):

```json
{"pi": {"ROUTE_PROFILE_BASE_PWM": 205, "ROUTE_TIME_COST": true,
        "ROUTE_LAP1_PWM": 155, "ROUTE_UNSEEN_PWM": 155, "ROUTE_PROFILE_MIN_PWM": 155,
        "ROUTE_CTE_PWM": 205, "ROUTE_PROFILE_SIGN_MM": 220.0,
        "ROUTE_PROFILE_SIGN_NEAR_MM": 150.0, "ROUTE_FIX_STEP_MM": 8.0}}
```

Para verla en el visor (desde este worktree, no desde otra rama):

```bash
cd src/RASPI/cam && PYTHONUTF8=1 TWIN_NOISE_SEED=1 /Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python -m pure_pursuit.chassis_twin --preset ruta_perfil --park-pos random --seed 1 --speed 1 --pi ROUTE_PROFILE_BASE_PWM=205 --pi ROUTE_TIME_COST=True --pi ROUTE_LAP1_PWM=155 --pi ROUTE_UNSEEN_PWM=155 --pi ROUTE_PROFILE_MIN_PWM=155 --pi ROUTE_CTE_PWM=205 --pi ROUTE_PROFILE_SIGN_MM=220 --pi ROUTE_PROFILE_SIGN_NEAR_MM=150 --pi ROUTE_FIX_STEP_MM=8
```

(Ajustar la ruta del venv en otra compu.)

## Cambios de código en esta rama

Todos apagados por defecto salvo el del ESP32.

| Commit | Qué | Estado |
|---|---|---|
| `f5b1ef9` | **ESP32 modo ruta: conteo de giros al día y paro dentro de la sección de salida.** Antes el conteo ("GIRO PI contado") tenía un hueco de 4 s y se paraba 1-3 secciones tarde. | Activo. −7.3 s en el twin. **Cargar al ESP antes de probar el modo ruta en el carro.** |
| `dc36952` | Perfil v3: piso propio (`ROUTE_PROFILE_MIN_PWM`) y freno proporcional cerca de señales (`SIGN_NEAR_MM`). | Usado en la base |
| `71acf75` | `ROUTE_PROFILE_ISLAND_MM`: frenar cerca de las 4 esquinas de la isla. | Probado: −3 fallas pero +0.47 s. No se usa |
| `4b5cf81` | `ROUTE_PROFILE_BRAKE_FF_S`: al frenar pide v − τ·a (compensa el retraso del motor) y deja bajar el PWM hasta 60. | Candidato (ver g4) |
| `e4238af` | **`ROUTE_FIX_STEP_MM`: arranque ciego pesa cada lectura por el avance.** Con el carro casi quieto el sonar repetía la misma lectura, y contarla 9 veces fijaba el cajón 400 mm mal con 97 % de confianza (semilla 8). | Usado en la base (8 mm) |
| `8ded2c4` | `ROUTE_PROFILE_TIGHT_KAPPA`: aceleración lateral menor en curvas muy cerradas (radio menor que el lookahead de 200 mm). | En prueba (h1/h2) |

## Cómo se decide (banco estándar)

Herramientas en `src/RASPI/cam/pure_pursuit/twin/tools/banco/`:

- `bench.sh <worktree> <nombre> ruta ruta_perfil '<json>'`: 42 semillas × 5 ruidos, base + candidato. Copiar antes `runs/combo1/base_n*` para no recorrer la base. Si se corta, relanzar el mismo comando: se salta las corridas ya hechas.
- `vsref.py REF VAR...`: compara corrida por corrida contra una referencia. Da las fallas, los pares que fallan solo en uno de los dos lados, las semillas nuevas que fallan, y la mediana de dt del total y de la vuelta 1 y las vueltas 2 y 3 (medidas geométricamente desde trace.csv).
- `startfix.py RUNS/<nombre>`: error del arranque ciego contra la verdad (`parkw` de field.txt).

**Regla:** ahorra ≥ 0.5 s mediano **y** solo-candidato ≤ solo-referencia + 2 **y** ninguna semilla
que nunca fallaba falla en ≥ 2 ruidos. Cada bench ocupa ~1.7 GB: revisar `df -h` antes.

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

## Cuánto queda (estimación)

- Hoy, vueltas 2-3: ~13.6 s/vuelta en el twin.
- El perfil de hoy seguido perfecto daría ~11.8 s/vuelta. El límite físico del twin es ~9.8 s/vuelta.
- Total ideal ~33 s en el twin, ~37-40 s con supuestos del carro real. Hay 17-24 s de margen.
- La línea de carrera vale poco (~0.35 s/vuelta). La velocidad en rectas y la vuelta 1 valen mucho más.

## Siguientes pasos

1. Terminar h1/h2 (A_LAT menor solo en curvas cerradas). Si arreglan la semilla 3, g4 + TIGHT pasa a ser la base (−0.5 s).
2. Freno por urgencia al salirse de la ruta: frenar solo si va hacia una señal o la isla, no por salirse poquito.
3. Vuelta 1 (~22.4 s): el mayor hueco que queda. Las fallas que siguen (semillas 16, 32, 21) son toques de señal en la vuelta 1.
4. **En el carro:** cargar el firmware con `f5b1ef9`, medir velocidad vs PWM y frenado (`twin/tools/calib/HOJA_MEDICION.md`), y calibrar el twin.
