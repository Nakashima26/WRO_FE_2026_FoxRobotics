# Respuesta: pk2_scan_v2f

_2026-11-25 14:54 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/pk2_scan_v2f (7a8c7212) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `pk2_scan_v2f: 377/384 ok  media 36.30  mediana 35.92  mejor 32.29  fallas: n3/s19 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36 n7/s10`
- Paradas: collision 6, race_finished 378
- Reloj total: 501.3 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 501.3 s para 384 carreras (referencia ~400-440 s sin carga), carga máx 7.50, mem máx 4563 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.02-7.39 (media 3.88).
- Tipo de prueba: carrera completa, variante pk2_scan_v2 del estacionamiento (comparable con pk2_base_0ee69022, pk2_scan_a y pk2_scan_b; código idéntico a pk2_scan_b), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/pk2_scan_v2f.md`.
