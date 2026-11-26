# Respuesta: regr_base7a

_2026-11-26 00:07 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/regr_base7a (2ee66daf) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `regr_base7a: 274/384 ok  media 36.52  mediana 36.17  mejor 31.62  fallas: n1/s13 n1/s19 n1/s20 n1/s22 n1/s23 n1/s26 n1/s29 n1/s31 n1/s32 n1/s33 n1/s39 n1/s4 n1/`
- Paradas: collision 71, race_finished 313
- Reloj total: 507.3 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 507.3 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 7.83, mem máx 4479 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.31-10.45 (media 4.30).
- Tipo de prueba: carrera completa, regr_base7a (sin FW_OVER ni PI_EXTRA), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/regr_base7a.md`.
