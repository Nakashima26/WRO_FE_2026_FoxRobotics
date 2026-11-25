# Respuesta: pk2_prior200_scan

_2026-11-25 13:29 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/pk2_prior200_scan (aa7ac1c2) con el comando que mandaste: n1-2 x s20-21 (4 carreras), -P 7, bajo twin.lock.

- resumen.py: `pk2_prior200_scan: 3/4 ok  media 32.88  mediana 32.74  mejor 32.26  fallas: n2/s21`
- Paradas: race_finished 4
- Reloj total: 14.6 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 14.6 s para 4 carreras (referencia ~10-20 s sin carga), carga máx 7.24, mem máx 10069 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.95-5.83 (media 4.88).
- Tipo de prueba: carrera completa con estacionamiento, variante pk2_prior200_scan, 4 carreras (n1-2 x s20-21); solo comparable con tandas pk2 del mismo tipo y mismas semillas (n1-2 s20-21).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/pk2_prior200_scan.md`.
