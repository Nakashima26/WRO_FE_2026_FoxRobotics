# g_gs_t1 — Falla Green S/T1 n1/s36 (arranque, vuelta 1): ancla del cajón CCW cruzando filas de asientos, base grok/t/g_gs_t1 1879a55 (= main c69b19f + park despegue) — FIN

- **Tipo de prueba: carrera completa con arranque desde el cajón** (salida del cajón + 3 vueltas + estacionamiento; la falla objetivo es de la **vuelta 1, arranque**). Harness `latencia/uno.sh`, n1–n2 × s1–48 = 96 corridas, `TWIN_MOTOR_REAL=1`, `TWIN_VBAT=11.5`, cámara desde calib + `lens_calib.npz`, `TWIN_COST_FIJO=1`, `TWIN_COST_FIRST=1.95`, FF 2.8 / FD 3.8, config Pi = `twin/tools/carrera_bench.json`. Sólo se compara con tandas de este mismo banco (g_w_park_base / g_geom / g_gs_t1_base).
- Cada tanda sola bajo `flock /workspace/fox/twin.lock`, 0 procesos ajenos al inicio, 0 Killed/MemoryError, mean_pi_frame_ms 3.4–3.7. Wall 127–141 s por 96 (lo normal de este box, g_w_park_base 133 s); réplicas 187–193 s por 138. **Todas válidas.** Contacto = falla.

## Diagnóstico de n1/s36 (antes de tocar código)
- CCW, cajón W en w0=1377 (el supuesto es 1696, δ real −319). Señales: **Red W/T2** (la única de la recta del cajón, fila interior) y justo después de la esquina **Green S/T1** (fila interior, 1ª columna de S). Hay que salir por fuera de la roja y cruzar enseguida hacia adentro para la verde.
- La verde ya se veía en la salida del cajón (cuadros 19–39, a 1.0–1.2 m, y ≈ −565 ±15 mm en el marco supuesto ⇒ δ ≈ −335). Pero **la ruta no tiene señales hasta que se fija el avance del cajón** (plan#1–#4 `señales=[]`), y aquí la cinta no vota (n=0) y los ToF no convergen hasta **113 lecturas** (masa 0.97, t=5.4 s), con el carro ya en la esquina en (−1060,−835). El plan#5 mete S/T1G tarde, el carro intenta cruzar hacia adentro desde y=−965 y toca la verde (t=7.05 s). La falla es de **fijación del cajón**, no de visión ni de control.

## Algoritmo nuevo (opt-in, apagado por defecto): `ROUTE_FIX_SEAT_ROWS`
`RouteMode._seat_rows_delta`. Con el carro todavía en la recta del cajón (CCW, est=S, tc=0, rumbo ≤25°), sin δ todavía:
1. Cada señal seguida en ≥5 cuadros (pie único por cuadro, a ≤1.3 m, vista desde posiciones separadas ≥45 mm, dispersión ≤45/60 mm) da **hipótesis de avance**:
   - **recta del cajón** (fila interior, lo de `asiento único`): su w fija el avance **módulo 500 mm** (columnas T2/X1/T1);
   - **recta siguiente**, vista desde el cajón: su columna (w de esa recta) **no depende del avance** y su fila (400/600 mm) sí, porque el eje del cajón es el eje h de esa recta ⇒ fija el avance **módulo 200 mm**.
2. Vale el avance que **explica todas las señales con un asiento legal** dentro de [PARK_W_MIN, PARK_W_MAX]. Si quedan dos, decide el histograma de los ToF sólo con ≥3 nats de ventaja; si la cinta ya votó, tiene que coincidir (±50 mm); si una señal no cabe en ningún asiento, no ancla.
3. Se aplica igual que `asiento único` (`_apply_fix`, mismo sesgo +15 mm del pilar en CCW). Sólo CCW (en CW la cinta ya llega temprano; probado: suma 2 choques).
- n1/s36: ancla con roja W/T2 (P) + verde S/T1 (N) a δ=−327 (verdad −319) en el cuadro 36 (antes 154); plan#3 ya tiene `S/T1G S/T3G W/T2R`; termina 43.89 s sin contacto.

## Resultados (96)
| tanda | cambio | ok | media | Δ pareado (95) | más lentas >0.3 s | contactos | wall |
|---|---|---|---|---|---|---|---|
| **g_gs_t1_base** | 1879a55 limpio | **95/96** | **41.89** | — | — | 1 (n1/s36, Green S/T1, vuelta 1) | 140.7 |
| **g_gs_t1** | `ROUTE_FIX_SEAT_ROWS=true` | **96/96** | **41.42** | **−0.49 s** (CCW −0.92, CW 0.00) | 7 (máx +1.79 n2/s12) | **0** | 127.4 |
| g_gs_t1_ups (referencia) | sólo `ROUTE_FIX_UNIQUE_PREFIX_SEAT=true` (ya existe) | 96/96 | 41.49 | −0.42 s | 5 | 0 | 136.4 |
| g_gs_t1_sr2 (descartada) | filas también en CW | 94/96 | 41.37 | −0.57 s | 13 | 2 (n1/s42, n2/s19, CW) | 129.7 |
- **Δt (media de las ok): −0.47 s** (41.89 → 41.42); pareado en las 95 ok de ambas −0.49 s. 23 semillas cambian (todas CCW), 58 corridas idénticas, todas las CW idénticas.
- Error del cajón (w0 fijado − verdad), CCW: base cinta 41 corridas mediana +25 mm, p90 82, **máx 122**; ToF 6 (fijan en el cuadro ~140); forzado 5 (cuadro ~158). Variante: **filas 38 corridas, cuadro mediano 71, error mediana −12, p90 23, máx 28 mm**; quedan 14 con cinta.

## Réplica n3–n8 de las 23 semillas que cambian (138 corridas)
| tanda | ok | media | Δ pareado (136) | contactos | wall | válida |
|---|---|---|---|---|---|---|
| g_gs_t1_base_rep (worktree limpio 1879a55) | 136/138 | 42.47 | — | 2 (n5/s36 Green S/T1 vuelta 1; n6/s23 Green S/T2 en el park) | 192.6 | sí |
| **g_gs_t1_rep** | **138/138** | **41.44** | **−1.04 s** | **0** | 186.7 | sí |
- 82 más rápidas >0.3 s, 12 más lentas >0.3 s (máx +1.05 n4/s44), 30 iguales. n5/s36 (la misma falla en otro ruido) también queda resuelta.

## Decisión
**ACEPTADA** (menos fallas: 95→96/96 y 136→138/138, y menos tiempo: −0.47 s en 96, −1.04 s en la réplica). Opt-in apagada por defecto.
- Commit `bdbdc4e` en `grok/t/g_gs_t1` (Jessebnda, sin co-autor), sobre 1879a55. Toca sólo `route_mode.py` (+113) y `config.py` (+4: `ROUTE_FIX_SEAT_ROWS = False` y 3 tolerancias). Tests `test_fix_line/test_start_fix/test_salida_cajon/test_route_rules`: 23 passed.
- Para activarla: Pi `ROUTE_FIX_SEAT_ROWS: true` (carrera.json en el carro / `TWIN_PI_EXTRA` en el gemelo). Sin cambios en el ESP (no hay que flashear).

## Hallazgo aparte (importante)
`carrera.json` (el carro) **ya tiene `ROUTE_FIX_UNIQUE_PREFIX_SEAT: true`** (d22ef63), pero `twin/tools/carrera_bench.json` (el banco) **no**: el banco mide una config distinta a la del carro. Con sólo ese flag la base ya da 96/96 41.49 s. `ROUTE_FIX_SEAT_ROWS` lo contiene (mismas señales P) y suma las de la recta siguiente: 38 vs 29 corridas ancladas temprano, −0.06 s pareado vs ese flag solo (8 más lentas / 12 más rápidas).

## Descartado / no repetido
- Filas también en CW (`sr2`): 2 choques nuevos → sólo CCW.
- No se tocaron PREFIX 160/183, PWM c1b, t7, corner sign leads, L23_SIGN_FOOTPRINT, vel3/vel4, SWEEP, FULL_W_KAPPA, A_LAT3200, PARK_REV_RECTA, t14.
- Tanda `g_gs_t1_sr1` INVÁLIDA (el parche de config borró por error `ROUTE_FIX_STUCK_DF_CM`; se repitió como `sr1b`, idéntica a la final `g_gs_t1`).

## Pendiente / riesgos en el carro
- Las tolerancias (pie a ≤1.3 m, columna ±80 mm, dispersión 60 mm) salen del gemelo; en el carro la proyección BEV a 1.0–1.3 m puede ser más ruidosa (el caso típico n1/s36 anclaba con la verde a 1.0–1.2 m).
- Decidir si el banco debe usar `ROUTE_FIX_UNIQUE_PREFIX_SEAT=true` como en carrera.json (nueva base) y medir ahí `ROUTE_FIX_SEAT_ROWS`.

FIN
