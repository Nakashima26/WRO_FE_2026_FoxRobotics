# Respuesta: cal2_288_radios

_2026-11-25 12:38 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/cal2_288_radios (c82a302f) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `cal2_288_radios: 5/288 ok  media 40.85  mediana 41.72  mejor 37.34  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1`
- Paradas: collision 54, race_finished 233, stuck 1
- Reloj total: 379.3 s. VÁLIDA: corrió bajo twin.lock (? procesos de banco ajenos al inicio), 379.3 s para 288 carreras (referencia ~300-330 s sin carga), carga máx 8.17, mem máx 4720 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.11-8.61 (media 3.85).
- Tipo de prueba: carrera completa con parámetros de estacionamiento, 288 carreras (n1-6 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/cal2_288_radios.md`.

Comparación (mismo tipo de prueba y mismas semillas n1-6 x s1-48): `cal2_288_base` dio 4/288 ok (media 37.64, mediana 36.92, mejor 34.92; paradas ERROR 11, collision 57, race_finished 219, stuck 1; 357.0 s). Ojo: no es A/B puro, porque base usaba REV_WHEEL_SCALE 0.65 sin GIRO_TOPE ni FOX_V2_FOOTPRINT, y esta tanda usa REV_RADII_MM 180,162 + GIRO_TOPE 42.57,40.14 + FOX_V2_FOOTPRINT=1. Aquí hay 0 ERROR y 14 race_finished más, pero las carreras ok (criterio de resumen.py: sin choques y fuera del cajón <= 5 mm) siguen en 5/288 y son ~3 s más lentas. La gran mayoría termina la carrera pero no cumple el criterio de estacionamiento.

Siguiente: para aislar el efecto de los radios, manda la misma tanda cambiando una sola cosa (por ejemplo base + GIRO_TOPE/FOOTPRINT sin radios), así la comparación es limpia.
