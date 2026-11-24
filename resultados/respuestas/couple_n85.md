# couple_n85 — respuesta de Goji

## 2026-11-23 (PST)

**Hecho:** corrí la tanda `couple_n85` (commit c32bcfe, origin/grok/t/couple_n85) en el banco NUEVO con latencia, sola bajo `twin.lock`: 2 ruidos × semillas 1-24 = 48 carreras, P=7, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_COST_FIRST=1.95 y el TWIN_PI_EXTRA del payload (CLEAR_CURVE lead_reduction 0.02, NO_FF, CLEAR_MM 85, LEAD_TC 6-10). Resultado completo en `resultados/couple_n85.md` (rama grok/resultados).

**Tipo de prueba:** carrera completa desde el cajón (costo.py `--solo-cajon`: salida, 3 vueltas y estacionamiento).

**Resumen:** `couple_n85: 19/48 ok  media 42.47  mediana 42.42  mejor 36.90` — 29 fallas.

**Lo que encontré:**
- Las 29 fallas son `collision`, y casi todas ocurren temprano: turns_completed=4 en 13 carreras, =1 en 7, =2 en 4, =0 en 2, y 3/5/6 en una cada una. O sea, la mayoría choca en la **vuelta 1**, antes de que deberían aplicar los parámetros `ROUTE_L23_*`. Mi hipótesis (sin verificar): la caída viene de la configuración del banco (TWIN_VBAT=11.5 + TWIN_CAMERA_FROM_CALIB=1) o del parche en route_mode.py, que afecta algo fuera de L2/L3. Esto no viene del ajuste CLEAR_CURVE.
- Validez: VÁLIDA. Reloj 49.9 s para 48 carreras (corto porque muchas acaban por choque), carga máx 4.73, 0 procesos de banco ajenos al inicio, sin OOM/Killed, 0 carreras sin STOP. mean_pi_frame_ms medio 5.89 (las tandas sin TWIN_CAMERA_FROM_CALIB daban ~3.7; probablemente la cámara calibrada cuesta más, no hubo carga). Media de media_ms (COSTO) 17.1 ms.
- No comparo con w_inicio_v1/v2 porque usan otra configuración (plan asíncrono, sin VBAT ni cámara calibrada).

**Pregunta:** ¿`couple_tip` es la base sin el parche con el mismo banco? Está en cola detrás del lock. Si también se cae en la vuelta 1, el problema es del banco/base y no de couple_n85.

**Qué mandar después:** la base exacta (mismo env, sin TWIN_PI_EXTRA) si couple_tip no lo es, o una tanda con trazas de 2-3 semillas que chocan en turns_completed≤1 (p. ej. n1/s1, n1/s4, n2/s3) para ver dónde se produce el choque.
