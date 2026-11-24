# senal: choques con señal (pilar) en carrera completa: diagnóstico y arreglo

**Tipo de prueba:** carrera completa de 3 vueltas con señales (salida a ciegas desde el cajón, estacionamiento al final).
Banco con latencia: `uno.sh <tag> 4.28 5.8`, `TWIN_VBAT=12.5 TWIN_COST_FIRST=1.95 TWIN_COST_FIJO=1`, lens_calib, `TWIN_FW_OVER='{}'`.
6 ruidos × 48 semillas = 288 carreras, `xargs -P 7`. Todas las tandas corrieron bajo `flock /workspace/fox/twin.lock` y sin otros bancos activos.
Código base: `origin/fox/twin-iter` @ 0844147 (detached). Variante: rama `grok/t/senal_fix` (commit 835c271 sobre 0844147).
Como el costo es FIJO, al repetir una carrera los STOP salen idénticos. Todos los choques analizados se volvieron a correr con logging (envoltorio solo de log) y dieron el mismo resultado que en la tanda.

## 1. Base (senal_base)

`senal_base: 269/288 ok  media 34.14  mediana 33.96  mejor 30.05`
Fallas (19): n1/s34 n1/s9 n2/s14 n2/s24 n2/s33 n2/s34 n3/s27 n3/s33 n3/s34 n3/s37 n4/s33 n4/s34 n4/s46 n5/s33 n5/s34 n5/s36 n5/s4 n6/s23 n6/s34
- Choque con señal: **7** (n2/s33, n3/s33, n4/s33, n5/s33, n3/s27, n5/s36, n6/s23)
- Otras: cajón magenta 4 (n1/s9, n3/s37, n4/s46, n5/s4), isla 1 (n2/s14), race_finished pero fuera del cajón 7 (s34 en los 6 ruidos, n2/s24). Esta tarea no las trata.

Validez: reloj 383.2 s, carga máx 7.98, mem máx 10872 MB, 0 MemoryError/Killed, mean_pi_frame_ms medio 3.72 → **VÁLIDA**.

## 2. Diagnóstico por carrera con choque de señal

Columnas:
- Holgura plan = distancia de la carrocería (176×125) al pilar sobre la ruta planeada.
- CTE = error lateral respecto a la ruta del ESP en el tramo del choque: real (pose verdadera del gemelo) y estimado (pose del Pi).
- Error de pose = verdad − estimación del Pi, con el retraso de ACK de 50–60 ms compensado: a lo largo / lateral / magnitud / componente hacia el pilar.
- v = velocidad al chocar (mm/s).
- Ruta ESP = momento desde el que rige la ruta del ESP (t en trace.csv) y cambios de ruta en el último segundo.

| carrera | señal | t choque | v | holgura plan (carrocería) | CTE real / est | error pose a lo largo / lat / mag / hacia pilar | ruta ESP desde (cambios últ. 1 s) | clase |
|---|---|---|---|---|---|---|---|---|
| n2/s33 | Green E/T1 | 3.955 | 509 | 101.6 | 86.7 / −14.4 | −140.8 / −66.5 / 155.8 / 101.2 | 2.803 (0) | **pose** |
| n3/s33 | Green E/T1 | 3.896 | 422 | 101.6 | 106.6 / −3.4 | −144.7 / −87.7 / 169.3 / 145.3 | 2.807 (1, misma holgura) | **pose** |
| n4/s33 | Green E/T1 | 3.776 | 411 | 101.6 | 106.7 / 7.1 | −143.2 / −83.8 / 165.9 / 130.6 | 3.278 (2, misma holgura) | **pose** |
| n5/s33 | Green E/T1 | 3.987 | 490 | 101.6 | 80.7 / 12.9 | −117.4 / −67.8 / 135.5 / 88.1 | 3.624 (1) | **pose** |
| n3/s27 | Red S/T4 | 9.940 | 732 (media 1 s: 847) | 63.1 | 57.7 / 27.1 (máx 35.3) | 51.6 / 26.7 / 58.1 / 28.8 | 8.043 (1) | **seguimiento** (mixto con pose) |
| n5/s36 | Green W/T1 | 20.217 | 1119 | 89.8 | 100.9 / 36.1 | −9.5 / −62.0 / 62.7 / 62.7 | 17.975 (0) | **pose** |
| n6/s23 | Red S/T1 | 20.244 | 1095 | 109.5 | 84.0 / −22.9 | 21.0 / 95.4 / 97.7 / 86.9 | 10.444 (0) | **pose** |

Unidades en mm, mm/s y s. La señal ya estaba en el plan mucho antes en los 7 casos (primer plan con la señal: 2.72–2.81 s en s33, 7.53 s en n3/s27, 9.96 s en n5/s36, 10.38 s en n6/s23). O sea, el mapeo no llegó tarde. El error de rumbo de la pose es ≈0 una vez compensado el retraso.

**Clasificación: pose 6, seguimiento 1 (mixto), planificador 0, otro 0.**

### Causas
- **s33 (n2, n3, n4, n5): 4 choques con la misma causa.** El FIX de salida a ciegas resuelve el avance del cajón con el voto "recta" de la línea naranja, pero la cinta está a 30° (|n·a| = 0.5) y el cruce se mide ~120 mm más lejos de lo real: el avance medido va de 589 a 513 mm, el real de 460 a 396 mm. Resultado: δ ≈ −402 a −426 mm cuando la verdad es −568, y w0 = 1270–1294 cuando el real es 1128. La pose queda ~140 mm corrida a lo largo (lateral ~70–90 mm hacia el pilar). Las lecturas laterales de ToF que lo corregirían quedan fuera de la compuerta 3σ del EKF (rechazos nr 8→29). Durante 1.1 s la cámara ve E/T1 en (724, −535), con el asiento en (900, −500), y no se usa para corregir. En n1/s33 y n6/s33 el voto dio w0 = 1099 y 1091, y esas carreras salen bien.
- **n5/s36 y n6/s23 (vuelta 2, a ~1.1 m/s).** Deriva a lo largo durante la recta anterior, sin lecturas de ToF aceptadas. Al doblar la esquina (NW y SW respectivamente) esa deriva pasa a ser lateral: 62 y 95 mm hacia el pilar.
- **n3/s27.** El plan ya pasaba justo: 63.1 mm de holgura, cuando SIGN_MM del reporte es 63.5. A 850 mm/s, el seguimiento suma 57.7 mm de CTE real (27–35 mm estimado) y la pose otros 28.8 mm hacia el pilar.

## 3. Cambio de algoritmo (opt-in): `ROUTE_SIGN_FIX`

Ataca la causa dominante (pose tras el FIX, 4 de 7). Se activa con `ROUTE_SIGN_FIX` en config/carrera_bench, que viene en false por defecto.

Después del FIX, en la primera vuelta (tc < 2), si la cámara ve un pilar se busca el asiento más cercano entre todos los asientos. Ese asiento tiene que ser uno ya conocido del plan, del mismo color y a menos de 250 mm. Si se cumple, se guarda el offset pies − asiento. Con 8 cuadros consistentes (dispersión ≤ 40 mm por eje) y mediana ≥ 120 mm, se desplaza la pose del localizador (`loc.shift`). Si no hubo esquina desde el FIX, también se corrige w0 del cajón con la componente a lo largo del eje. Condiciones: una sola corrección por carrera, giro ≤ 1.5°/cuadro, v ≤ 500 mm/s y pies a menos de 800 mm.
Cambios en archivos: route_mode.py (`_sign_fix`, +75 líneas), digital_map.py (`last_feet`), config.py (11 constantes), carrera_bench.json (`"ROUTE_SIGN_FIX": false`).

Se descartó antes una primera idea (ROUTE_FIX_COV: inflar la covarianza del EKF a lo largo del eje tras el FIX). En s33 no arreglaba nada, porque las lecturas disponibles miden casi perpendicular al error.

### Iteraciones (mismas 288 carreras)

| tanda | ok | fallas | fallas nuevas vs base | rescatadas | choques con señal | mediana ok (s) | mediana en ok comunes | correcciones |
|---|---|---|---|---|---|---|---|---|
| senal_base | 269/288 | 19 | — | — | 7 | 33.96 | — | — |
| senal_fix v1 (MIN 60 mm, v ≤ 700, cajón siempre) | 272/288 | 16 | **1 (n5/s35, fuera del cajón)** | 4 (s33) | 3 | 33.87 | 33.959 → 33.932 (268) | 28 |
| **senal_fix2 v2 (MIN 120 mm, v ≤ 500, cajón solo sin esquina desde el FIX)** | **273/288** | **15** | **0** | **4** (n2, n3, n4, n5 /s33) | **3** | **33.91** | 33.962 → 33.962 (269, todas idénticas) | 4 |

- v1 incumple el criterio por una falla nueva: en n5/s35 movió w0 de 1389 a 1482 con el real en 1279, y no estacionó. Además hizo 24 correcciones extra de 60–95 mm, varias en dirección equivocada.
- v2: solo 4 correcciones (+159, +175, +183, +189 mm a lo largo), todas en las carreras s33 y todas en la dirección correcta. w0 pasa a 1105–1111 (real ≈ 1128). En las otras 284 carreras no se activa. Las 269 carreras ok en ambas tienen tiempo idéntico.
- Validez de v2: reloj 354.6 s, carga máx 7.44, mem máx 5626 MB, 0 MemoryError/Killed, mean_pi_frame_ms medio 3.45 → **VÁLIDA**.
- Flag apagado = sin cambio: con el parche y `TWIN_PI_EXTRA='{}'` se corrieron 14 carreras (los 7 choques, n5/s35, n1/s21, n1/s1, n4/s10, n6/s40, n2/s24, n1/s9). Las 14 salen idénticas a la base en todas las métricas de simulación; solo cambian los tiempos de reloj.

**Criterio "0 fallas nuevas vs senal_base y no más lento (mediana de 3 vueltas)":** v2 lo **cumple**: 0 nuevas, +4 ok, mediana 33.96 → 33.91 s, e igual en las comunes.

## 4. Conclusión

- 6 de los 7 choques con señal son de **pose**, no del planificador. 4 vienen del FIX de salida (voto de la línea naranja a 30° que mide el cruce ~120 mm lejos) y 2 de deriva a lo largo que se vuelve lateral tras una esquina en la vuelta 2. 1 es de seguimiento con poco margen.
- `ROUTE_SIGN_FIX` (v2) corrige los 4 de s33 sin romper nada ni hacer más lenta la carrera. Sigue sin resolver n3/s27, n5/s36 y n6/s23: la corrección está limitada a la primera vuelta y v ≤ 500, y esos tres pasan a 0.85–1.1 m/s.
- Es un parche sobre el síntoma. El arreglo de raíz sería corregir el voto "recta" de la línea naranja cuando la cinta está inclinada (|n·a| bajo).
- **Atención:** mientras corría esto, `fox/twin-iter` avanzó a ad8e5dd ("Corrige la pose de salida con un par de pilares", ROUTE_FIX_PILLAR_PAIR, activado en carrera_bench). Ataca el mismo mecanismo del FIX. `grok/t/senal_fix` está sobre 0844147 y **tiene conflicto con ad8e5dd en route_mode.py**. No se probó la combinación.

Archivos: `resultados/senal_base_choques.csv` (diagnóstico por carrera).
FIN
