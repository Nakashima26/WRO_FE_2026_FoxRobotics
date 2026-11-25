# Respuesta: l1s_posterior428

_2026-11-25 12:38 PST (Tijuana) — estado: EN COLA bajo twin.lock_

Recibida. Banco de **vuelta 1** l1s armado y lanzado en segundo plano (nohup + flock /workspace/fox/twin.lock); está **en cola**: ahora corre cal2_288_radios y hay ~26 tandas esperando el lock delante (s20cal2_radios, pk2_*, tw1_*, l1v_diag288, l1v_cartas_288, ab_*, l1v_corredor_288, tw1_v3_full428, cal2_288_base_retry…), así que calcula varias horas de espera. No corre en paralelo con nada.

Qué va a correr (las cuatro tandas en una sola toma del lock):
- **Variante `l1s_posterior428`**: d7a8ee14 (padre exacto ae0f25ed, verificado), TWIN_PI_EXTRA `{"ROUTE_SEAT_POSTERIOR":true}` + ROUTE_SEAT_POSTERIOR_LOG=1.
- **Base `l1s_base428`**: ae0f25ed (tree idéntico a 0ee69022), PI_EXTRA `{}` sin sonda.
- Ambos: n1-6 x s1-48 (288), -P 7, `pure_pursuit/twin/tools/latencia/uno.sh <tag> 4.28 5.8 n s` + resumen.py desde src/RASPI/cam, TWIN_COST_FIRST=1.95, TWIN_COST_FIJO=1, TWIN_TOF_SHORT_K=0.5, TWIN_MOTOR_CURVA_REAL=1, TWIN_MOTOR_REAL=1, TWIN_CAMERA_FROM_CALIB=1, TWIN_VBAT=11.5, TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz, TWIN_FW_OVER='{}', LAT_OUT=/tmp/lat.
- Después: semillas con cambio en algún STOP (éxito, tiempo total o laps) en n1-6 → ambos brazos con n7/n8 como `l1s_base_rep78` / `l1s_posterior_rep78`.

**No reutilizo la0g**: la0g se canceló sin correr (12:26) y quedó sustituida por l1v_diag288, que aún no corre y usa l1v_uno.sh con la sonda l1v_diag.py (recompila RouteMode._speed/step y hace llamadas extra a _profile_pwm/_sign_xy), así que no puedo garantizar que la sonda no altere la simulación; no hay otra tanda con FF 4.28/FD 5.8 + FW_OVER {} sobre ese árbol. Por eso corro ae0f25ed como `l1s_base428`. Tampoco hay datos base n7/n8 compatibles (tw1_base428 usa FW_OVER MOTOR_CURVE_LIVE=false), así que la base rep78 también se corre.

Al terminar subo `resultados/l1s_posterior428.md` (+ l1s_base428, rep78 y `l1s_posterior428.csv`) con STOP y COSTO por carrera, fallas en 288 y rep78, media total pareada entre válidas, vuelta 1 pareada (lap_times_s[0]), early_empty máx por carrera de [SEAT_POSTERIOR] (media y cuántas >0), seeds cambiadas, árbol/config efectiva y validez de carga, y actualizo esta respuesta. Nota: trace.csv de este banco no trae columna tc (solo t, x, y, motor…), así que tc4 desde el armado probablemente saldrá n/d; si existe, se extrae.

Tipo de prueba: vuelta 1 (lap 1 only); solo la comparo con tandas de vuelta 1 de la misma configuración.
