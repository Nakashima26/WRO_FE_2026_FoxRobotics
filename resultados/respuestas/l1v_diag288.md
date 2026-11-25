# Respuesta: l1v_diag288

_2026-11-25 12:30 PST (Tijuana) — estado: EN COLA_

Recibido. Preparé la tanda diagnóstica en origin/grok/t/l1v_diag288 = 63886cd9 ("Grok: diagnóstico de límites en vuelta 1"), cuyo padre es 0ee69022 (mismo algoritmo base + la sonda l1v_diag.py). Está esperando el twin.lock global; una tanda a la vez.

Cómo correrá (tal cual pediste):
- Desde src/RASPI/cam: `sed 's|"$D/costo.py"|l1v_diag.py|' uno.sh > pure_pursuit/twin/tools/latencia/l1v_uno.sh` (mismo directorio; si el sed no tiene efecto, la tanda aborta en vez de correr con costo.py).
- n1-6 × s1-48 = 288 carreras, -P 7, FF 4.28 / FD 5.8, TWIN_PI_EXTRA {}, TWIN_FW_OVER {}, TOF_SHORT_K 0.5, MOTOR_CURVA_REAL=1, CAMERA_FROM_CALIB=1, MOTOR_REAL=1, VBAT 11.5, COST_FIJO=1, COST_FIRST 1.95, LENS_CALIB pure_pursuit/lens_calib.npz.
- Después: `l1v_diag.py summary /tmp/lat/l1v_diag288` y `resumen.py l1v_diag288`. Ambos resúmenes, las 288 líneas STOP y COSTO, config, reloj y validez (carga/OOM/mean_pi_frame_ms) irán a `grok/resultados:resultados/l1v_diag288.md`, y este archivo se reemplazará con el resultado.
- Sin flashear, sin push a main, sin exportar_carrera.

Cola: ahora corre ciego25 y hay unas 16 tandas esperando el lock (tw1_*, pk2_*, la0g, cal2_288_radios, s20cal2_radios, l1v_cartas288, l1v_cartas_288, ...), así que puede tardar más de una hora.

Sobre la0g: sigue EN COLA, pero con otra configuración: origin/grok/t/la0g = ae0f25ed (parche de prueba sobre 0ee69022), FF 2.8 / FD 3.8, TWIN_FW_OVER {"MOTOR_CURVE_LIVE":"false"}, PI_EXTRA {}, n1-6 × s1-48. No la cambié porque no está claro que lo pidas. Por lo tanto NO es comparable directamente con l1v_diag288 (FF/FD y FW_OVER distintos). Pregunta: ¿quieres que reconfigure la0g a FF 4.28 / FD 5.8 y FW_OVER {} antes de que arranque? Responde con eso explícito y lo cambio.

Sobre l1v_cartas_288 (y l1v_cartas288): también en cola con FF 2.8 / FD 3.8 y PI_EXTRA {"ROUTE_L1_CARD_SPEED":true}. Los STOP de esta tanda diagnóstica (FF 4.28 / FD 5.8) no son una base exacta para ese par por el FF/FD distinto; si quieres la base exacta del par, manda una tanda base con FF 2.8 / FD 3.8. (Ojo: siguen las dos variantes con y sin guion bajo; si una es duplicado, dime cuál omito.)
