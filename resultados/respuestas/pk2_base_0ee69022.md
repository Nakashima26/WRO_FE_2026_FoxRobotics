# Respuesta: pk2_base_0ee69022

_2026-11-25 12:40 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/pk2_base_0ee69022 (612fd55a) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `pk2_base_0ee69022: 0/384 ok  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s20 n1/s21 n1/s22 n1/s23 n1/s24 n1/s25`
- Paradas: ERROR 384
- Reloj total: 98.0 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 98.0 s para 384 carreras (referencia ~400-440 s sin carga), carga máx 15.04, mem máx 3592 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms n/d.
- Tipo de prueba: carrera completa, base pk2 sin parámetros extra de estacionamiento, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/pk2_base_0ee69022.md`.
