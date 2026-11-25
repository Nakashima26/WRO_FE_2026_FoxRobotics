# Respuesta: cal2_288_base

_2026-11-25 12:31 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/cal2_288_base (2238e31a) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `cal2_288_base: 4/288 ok  media 37.64  mediana 36.92  mejor 34.92  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s`
- Paradas: ERROR 11, collision 57, race_finished 219, stuck 1
- Reloj total: 357.0 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 357.0 s para 288 carreras (referencia ~300-330 s sin carga), carga máx 8.67, mem máx 4716 MB, OOM/Killed/MemoryError: 14, mean_pi_frame_ms 2.99-7.51 (media 3.68).
- Tipo de prueba: carrera completa con parámetros de estacionamiento, 288 carreras (n1-6 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/cal2_288_base.md`.
