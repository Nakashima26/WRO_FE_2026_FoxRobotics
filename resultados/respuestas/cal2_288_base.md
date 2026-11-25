# Respuesta: cal2_288_base

_2026-11-25 12:31 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/cal2_288_base (2238e31a) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `cal2_288_base: 4/288 ok  media 37.64  mediana 36.92  mejor 34.92  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s`
- Paradas: ERROR 11, collision 57, race_finished 219, stuck 1
- Reloj total: 357.0 s. PARCIALMENTE INVÁLIDA: 11 carreras (n5 s8-s13 y s15-s19) murieron porque el disco de mi máquina se llenó un momento hacia las 12:29:50 PST (OSError Errno 28 No space left on device, no fue OOM ni carga); salen como ERROR y cuentan como fallas en resumen.py. Las otras 277 son válidas: twin.lock sin bancos ajenos, carga máx 8.67, mem máx 4716 MB, 357.0 s para 288 carreras (~1.24 s/carrera, un poco arriba de la referencia ~300-330 s). Ya dejé las 11 re-corriéndose en cola bajo twin.lock (hay otras tandas antes); cuando terminen actualizo este archivo y resumen.py.
- Tipo de prueba: carrera completa con parámetros de estacionamiento, 288 carreras (n1-6 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Ojo: resumen.py cuenta como ok solo parada normal sin choques y fuera del cajón <= 5 mm; casi todas las demás terminan la carrera (race_finished) pero no cumplen ese criterio de estacionamiento.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/cal2_288_base.md`.
