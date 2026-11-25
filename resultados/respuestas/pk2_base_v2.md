# Respuesta: pk2_base_v2

_2026-11-25 13:04 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/pk2_base_v2 (2a113d2b) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `pk2_base_v2: 0/384 ok  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s20 n1/s21 n1/s22 n1/s23 n1/s24 n1/s25 n1/s2`
- Paradas: ERROR 384
- Reloj total: 101.5 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 101.5 s para 384 carreras (referencia ~400-440 s sin carga), carga máx 16.10, mem máx 3959 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms n/d.
- Tipo de prueba: carrera completa, repetición de la base de estacionamiento (árbol idéntico a 0ee69022; comparable con pk2_base_0ee69022, pk2_scan_a y pk2_scan_b), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/pk2_base_v2.md`.
