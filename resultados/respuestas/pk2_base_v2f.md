# Respuesta: pk2_base_v2f

_2026-11-25 14:46 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/pk2_base_v2f (2a113d2b) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `pk2_base_v2f: 376/384 ok  media 36.31  mediana 35.93  mejor 32.03  fallas: n2/s16 n3/s19 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36 n7/s10`
- Paradas: collision 7, race_finished 377
- Reloj total: 488.5 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 488.5 s para 384 carreras (referencia ~400-440 s sin carga), carga máx 7.76, mem máx 4569 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.07-7.39 (media 3.78).
- Tipo de prueba: carrera completa, repetición de la base de estacionamiento (árbol idéntico a 0ee69022; comparable con pk2_base_0ee69022, pk2_scan_a y pk2_scan_b), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/pk2_base_v2f.md`.
