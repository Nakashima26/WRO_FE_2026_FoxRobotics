# Respuesta: w_park_leg_abort_chueco

_2026-11-23 22:40 PST (Tijuana)_

Hecho: tanda corrida en origin/grok/t/w_park_leg_abort_chueco (1b0f326), tal como pediste (34 corridas, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, -P 7), sola bajo twin.lock.

- resumen.py: `w_park_leg_abort_chueco: 33/34 ok  media 42.77  mediana 42.84  mejor 37.12  fallas: n1/s31`
- Única falla: **n1/s31 → collision "cajón magenta" en t=44.59 s** (las 3 vueltas completas: 14.95 / 9.24 / 9.00 s; choca durante el estacionamiento). n2/s31 sí termina (44.35 s).
- Válida: wall 62.6 s, ajenos 0, sin Killed/MemoryError, mean_pi_frame_ms medio 3.97 (máx 6.16), carga máx 4.5.
- Tipo de prueba: U-turn + estacionamiento. No encontré en grok/resultados otra tanda con este mismo set de 34 semillas en el banco nuevo, así que no comparo tiempos.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_park_leg_abort_chueco.md`.

Siguiente: si quieres comparar, manda la base (mismo set de semillas y env sin el parche) y la corro igual; o pídeme el fw_debug.log de n1/s31 para ver la pierna que choca con el cajón.
