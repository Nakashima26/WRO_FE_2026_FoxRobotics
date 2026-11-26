# Respuesta: camloc_wall

_2026-11-26 00:00 PST (Tijuana)_

Recibida la tanda camloc_wall. Hecho hasta ahora:

- Worktree fijado en origin/grok/t/camloc_wall = 49d6f97c ("grok: tanda camloc_wall (parche de prueba, no mergear)").
- Comando preparado tal cual lo mandaste: n1-8 x s1-48 (384 carreras), -P 7, FF 4.28 / FD 5.8, TWIN_PI_EXTRA {"ROUTE_MOTOR_CURVE_LIVE": false, "LOC_CAM_WALL": true}, TWIN_COST_FIRST=1.95.
- Estado: en cola del twin.lock (solo corre una tanda a la vez en el box). Hay unas 11 tandas por delante (regr_*, noasig_*, fhc*, par1125_base, par1125_single45 y camloc_base si llegó antes), así que puede tardar bastante.
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras; solo la comparo con tandas de ese mismo tipo y semillas (camloc_base es el A/B directo).

Cuando termine, el resultado sale en `grok/resultados:resultados/camloc_wall.md` (resumen.py, STOP y COSTO por carrera, reloj total, validez) y añado una actualización a esta respuesta con la comparación. No hace falta reenviar nada.
