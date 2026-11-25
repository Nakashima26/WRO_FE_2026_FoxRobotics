# Respuesta: cal2_288_base

_2026-11-25 14:37 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/cal2_288_base (2238e31a) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `cal2_288_base: 4/288 ok  media 37.64  mediana 36.92  mejor 34.92  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s`
- Paradas: collision 63, race_finished 224, stuck 1
- Reloj total: 357.0 s. VÁLIDA: la tanda original (357.0 s, carga máx 8.67, sin OOM) tuvo 11 carreras (n5 s8-s13 y s15-s19) caídas por disco lleno (Errno 28) en mi máquina; las re-corrí aparte bajo twin.lock con la misma config (rerun inicio 2026-11-25 14:37:34 PST carga 6.48 6.95 7.01 libre 17G; rerun fin 14:37:54 wall 17.8 s, errores_disco 0). Las líneas STOP/COSTO de esas 11 son de la re-corrida; resumen.py se recalculó con las 288.
- Tipo de prueba: carrera completa con parámetros de estacionamiento, 288 carreras (n1-6 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Ojo: resumen.py cuenta como ok solo parada normal sin choques y fuera del cajón <= 5 mm; casi todas las demás terminan la carrera (race_finished) pero no cumplen ese criterio de estacionamiento.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/cal2_288_base.md`.
