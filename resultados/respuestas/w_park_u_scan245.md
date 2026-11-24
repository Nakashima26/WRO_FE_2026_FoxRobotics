# Respuesta: w_park_u_scan245

_2026-11-24 09:03 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_scan245 (aff0020) tal como la pediste: 6 carreras (n1-2 x s16/20/32), banco nuevo de latencia, FF 2.8 / FD 3.8, -P 7, COST_FIRST 1.95, TWIN_FW_OVER PARK_GEOM_EXT/ROJO + PARK_SCAN_U_PWM 245, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT 1. Corrió sola bajo twin.lock (justo después de w_park_u_base); no faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_scan245: 6/6 ok  media 41.90  mediana 43.34  mejor 37.31  fallas:`
- Reloj total: 16.1 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 2.07 / 11254 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.4-3.6 ms. Referencia: w_park_u_base hizo las mismas 6 carreras en 11.3 s.
- Tipo de prueba: carrera completa desde el cajón con estacionamiento final (latencia/uno.sh, banco NUEVO con latencia; PARK_GEOM_EXT + PARK_GEOM_ROJO, variante PARK_SCAN_U_PWM 245).
- w_park_u_base (mismas 6 carreras, mismo FW_OVER sin PARK_SCAN_U_PWM): 5/6 ok vs scan245 6/6; en las 5 que terminan en ambas +1.92 s en total (n1/s16 -0.57, n1/s32 +0.55, n2/s16 +1.95, n2/s20 -0.28, n2/s32 +0.27); terminaban allá y ahora fallan: ninguna; nuevas que terminan: n1/s20.
- w_geom_rojo_r14 (PARK_GEOM_EXT+ROJO, sólo sus n1-2 x s16/20/32): 6/6 ok vs scan245 6/6; en las 6 que terminan en ambas -1.13 s en total (n1/s16 -0.98, n1/s20 -0.14, n1/s32 -0.15, n2/s16 +1.63, n2/s20 -0.91, n2/s32 -0.58); terminaban allá y ahora fallan: ninguna; nuevas que terminan: ninguna.
- Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_scan245.md`.

Lectura rápida: con PARK_SCAN_U_PWM 245 terminan las 6 (la base tenía 5/6, fallaba n1/s20). Es una muestra de sólo 6 carreras, así que el cambio de tiempo es ruido-sensible; si quieres confirmarlo, mándame la misma variante con más semillas (p. ej. n1-2 x s1-48) y la comparo contra una base igual.
